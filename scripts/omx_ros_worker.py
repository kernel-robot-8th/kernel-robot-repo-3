#!/usr/bin/env python3
"""ROS 2 action 서버로 동작하는 OMX 워커다: /omx/move_to_slot (omx_interfaces/action/MoveToSlot).

팔로워를 선반 칸에 기록해 둔 관절 자세로 옮긴다. HTTP 워커와 같은 이동이다(omx_slot_motion.py).
ROS 2와 인터페이스 패키지를 source한 LeRobot 환경에서 실행한다:

    source /opt/ros/jazzy/setup.bash && source cpp/install/setup.bash
    export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_ros_worker.py"          # 실물 팔
    ~/lerobot-venv/bin/python scripts/omx_ros_worker.py --fake-follower           # 팔 없이, 모의

목표가 끝나는 방식:
- 한 번에 목표 하나. 팔이 움직이는 중에 온 목표는 ROS 단계에서 거부한다(수락하지 않음)
- 검사 실패 -> aborted, outcome REJECTED. 아무것도 움직이지 않았고 진행(feedback)도 보내지 않았다
- 첫 진행(phase MOVING)은 목표 위치를 쓰기 바로 직전에 온다: 그 전에는 아무것도 움직이지 않는다
- 도착해서 멈춤 -> succeeded, outcome SUCCEEDED, pose_reached true
- 취소 -> 팔이 그 자리에 선다 -> canceled, outcome CANCELED
- 제한 시간 초과 -> FAILED, 모터 또는 취소 확인 오류 -> RESULT_UNKNOWN (둘 다 aborted). 여기서 재시도하지 않는다
--dry-run이나 --fake-follower의 결과에는 simulated true가 붙는다.
이 워커는 팔을 칸까지 데려가기만 한다. 집기는 그다음 LeRobot으로 학습한 정책이 한다.
실물 모드에서는 목표를 받으면 팔이 움직인다. 하드웨어 전원 차단을 대신하지 않는다.
HTTP 워커와 달리 ACCEPTED 핸드셰이크가 없다. ROS가 목표를 수락하면 검사를 거쳐 바로 움직인다.
"""

from __future__ import annotations

import argparse
import threading
from datetime import datetime, timezone

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from omx_interfaces.action import MoveToSlot
from omx_slot_motion import ARM_JOINTS, Outcome, Rejected, SlotMotion, add_motion_arguments



class OmxActionServer(Node):
    """MoveToSlot action 서버 노드. 잠금(busy)으로 동시에 목표 하나만 실행한다.

    ReentrantCallbackGroup + MultiThreadedExecutor라서 실행 중에도 새 목표·취소 콜백이 따로 돈다.
    """

    def __init__(self, motion: SlotMotion, action_name: str) -> None:
        super().__init__("omx_worker")
        self.motion = motion
        self.busy = threading.Lock()
        self.server = ActionServer(
            self, MoveToSlot, action_name,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            # 취소는 언제나 받는다. 실제로 멈추는 것은 move()가 is_cancel_requested를 보고 한다.
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f"{action_name} 대기 · {motion.describe()}")

    def on_goal(self, request: MoveToSlot.Goal) -> GoalResponse:
        """팔이 다른 목표를 실행 중이면 ROS 단계에서 바로 거부한다(클라이언트에는 "Goal was rejected")."""
        if self.busy.locked():
            self.get_logger().warning(f"[{request.command_id}] 팔이 다른 목표를 실행 중이라 받지 않음")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle) -> MoveToSlot.Result:
        """수락된 목표를 실행한다. on_goal과 execute 사이에 다른 목표가 끼어든 경우를 위해 잠금을 한 번 더 잡는다."""
        request: MoveToSlot.Goal = goal_handle.request
        self.get_logger().info(f"[{request.command_id}] 칸 {request.slot_id}")
        if not self.busy.acquire(blocking=False):
            return self.finish(goal_handle, Outcome("REJECTED", self.motion.simulated, "WORKER_BUSY"))
        try:
            return self.finish(goal_handle, self.run(goal_handle, request))
        finally:
            self.busy.release()

    def run(self, goal_handle, request: MoveToSlot.Goal) -> Outcome:
        """유효 시각 확인 → 포트 열기 → 검사(plan) → 이동(move). 검사에서 걸리면 움직이지 않고 REJECTED다."""
        # valid_until이 0이면 클라이언트가 유효 시각을 빠뜨린 것이다. 기한 없는 목표는 받지 않는다.
        if request.valid_until.sec == 0 and request.valid_until.nanosec == 0:
            return Outcome("REJECTED", self.motion.simulated, "VALID_UNTIL_MISSING")
        valid_until = datetime.fromtimestamp(request.valid_until.sec + request.valid_until.nanosec / 1e9, timezone.utc)

        # move()가 목표를 쓰기 직전과 이동 중 약 0.1초마다 부르는 진행 보고.
        def progress(present: dict[str, int], max_error: int) -> None:
            feedback = MoveToSlot.Feedback()
            feedback.phase = "MOVING"
            feedback.joint_names = list(ARM_JOINTS)
            feedback.present_ticks = [present[name] for name in ARM_JOINTS]
            feedback.max_error_ticks = max_error
            goal_handle.publish_feedback(feedback)

        try:
            with self.motion.follower():
                try:
                    goal = self.motion.plan(request.slot_id, request.slot_map_version, valid_until)
                except Rejected as rejected:
                    return Outcome("REJECTED", self.motion.simulated, rejected.reason_code, rejected.message)
                return self.motion.move(goal, lambda: goal_handle.is_cancel_requested, progress)
        except (ConnectionError, OSError, RuntimeError) as error:
            # 정책이나 텔레오퍼레이션이 포트를 쓰고 있을 수 있다. 모터에는 아무것도 보내지 않았다.
            return Outcome("REJECTED", self.motion.simulated, "FOLLOWER_PORT_UNAVAILABLE", str(error)[:300])

    def finish(self, goal_handle, outcome: Outcome) -> MoveToSlot.Result:
        """Outcome을 action 결과로 옮기고 ROS 상태를 정한다: SUCCEEDED만 succeeded, 취소는 canceled, 나머지는 aborted.

        클라이언트는 ROS 상태가 아니라 outcome과 pose_reached로 판단해야 한다.
        """
        result = MoveToSlot.Result()
        result.outcome = outcome.outcome
        result.pose_reached = outcome.pose_reached
        result.reason_code = outcome.reason_code
        result.message = outcome.message
        result.joint_names = list(outcome.final_ticks)
        result.final_ticks = list(outcome.final_ticks.values())
        result.max_error_ticks = outcome.max_error_ticks
        result.simulated = outcome.simulated
        if outcome.outcome == "SUCCEEDED":
            goal_handle.succeed()
        elif outcome.outcome == "CANCELED":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        line = f"  -> {outcome.outcome} {outcome.reason_code}{' (모의)' if outcome.simulated else ''}"
        # rclpy는 한 호출 위치에서 두 가지 심각도로 로그를 남기는 것을 거부하므로, 두 수준을 따로 호출한다.
        if outcome.outcome == "SUCCEEDED":
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)
        return result


def main() -> int:
    """CLI 진입점. SlotMotion을 먼저 만들어(자세 파일·보드·보정 확인) 실패하면 ROS 노드를 띄우지 않는다."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--action", default="/omx/move_to_slot")
    add_motion_arguments(parser)
    args = parser.parse_args()

    motion = SlotMotion(args)
    rclpy.init()
    node = OmxActionServer(motion, args.action)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n워커 종료 (토크는 켜 둔 채로 둡니다)", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
