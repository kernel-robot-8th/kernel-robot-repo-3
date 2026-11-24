#!/usr/bin/env python3
"""팔 하나의 PICK_WITH_POLICY를 ROS 2 action 서버로 제공한다: /arm_01/pick_object (omx_interfaces/action/PickObject).

HTTP 워커(omx_lerobot_worker.py)와 같은 집기다. 검사, ACT 정책 실행, home 정렬이 omx_policy_pick.PolicyPick이므로
같은 규칙이 적용된다(이 팔과 보드에서 녹화한 정책, 정책 파일 SHA와 모델, 시작 전 기록된 home으로 복귀, 취소하면
토크를 켠 채 그 자리에서 멈춤). 팔마다 서버 하나를, ROS 2와 인터페이스 패키지를 source한 LeRobot 환경에서 실행한다.

    source /opt/ros/jazzy/setup.bash && source cpp/install/setup.bash
    export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_pick_action_server.py --robot-id arm-01"   # 실물 팔
    ~/lerobot-venv/bin/python scripts/omx_pick_action_server.py --robot-id arm-01 --fake-follower    # 팔 없음

목표가 끝나는 방식:
- 한 번에 목표 하나. 팔이 바쁠 때 온 목표는 ROS 단계에서 거부한다(수락하지 않음)
- 검사 실패 → aborted, outcome REJECTED. 아무것도 움직이지 않았다
- feedback은 CHECKING, 정책 시작 전 RETURNING_HOME, STARTING_POLICY, RUNNING_POLICY, 그리고 정책 뒤 RETURNING_HOME
  순서다. 두 RETURNING_HOME 단계에서도 팔이 움직인다.
- home 도착 → succeeded, outcome HOME_REACHED_PLACE_NOT_VERIFIED, pose_reached true. 물체가 놓였는지는 팔이 볼 수
  없으므로 관제가 카메라로 확인한다
- 취소 → 팔이 있는 자리에서 정책이 멈춤 → canceled, outcome CANCELED
- 쉬는 자세가 아닌 곳에서의 시간 초과, 정책 오류, home 이동 실패는 FAILED / RESULT_UNKNOWN (aborted). 다시 시도하지
  않는다
--fake-follower와 --dry-run은 정책을 돌리지 않는다(카메라, 정책 서버 없음). --simulated-run-s만큼 기다린 뒤 가짜 팔에서
home 정렬을 하고(dry-run은 실물 팔을 읽기만 한다), 그런 결과에는 모두 simulated true가 붙는다.
이것은 하드웨어 전원 차단을 대신하지 않는다.

HTTP 워커와 달리 ACCEPTED 핸드셰이크가 없다. 서버는 검사를 마치면 스스로 시작하므로, 관제가 목표의 valid_until을
자기 ACK 기한 안으로 잘라 보내고, 서버는 정책을 시작하기 직전에 그 시각을 다시 확인한다.
"""

from __future__ import annotations

import argparse
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from omx_interfaces.action import PickObject
from omx_policy_pick import PickEnd, PolicyPick, add_policy_arguments, arm_env, finish_policy_arguments
from omx_slot_motion import SlotMotion, add_motion_arguments

# PickEnd.event(HTTP 워커의 이름) → action outcome. home에서의 PLACE_NOT_VERIFIED가 기대한 정상 끝이다.
HOME_REACHED = "HOME_REACHED_PLACE_NOT_VERIFIED"


def action_name(robot_id: str) -> str:
    """arm-01 → /arm_01/pick_object. ROS 이름에는 '-'를 쓸 수 없어 '_'로 바꾼다."""
    return "/" + robot_id.replace("-", "_") + "/pick_object"


def outcome_of(end: PickEnd) -> str:
    """PickEnd를 PickObject 결과의 outcome 문자열로 바꾼다."""
    if end.event == "RESULT_UNKNOWN" and end.reason_code == "PLACE_NOT_VERIFIED" and end.pose_reached:
        return HOME_REACHED
    return end.event  # FAILED, CANCELED, RESULT_UNKNOWN


class PickActionServer(Node):
    """팔 하나의 집기 action 서버 노드. 목표는 한 번에 하나만 실행한다."""

    def __init__(self, pick: PolicyPick, robot_id: str, name: str) -> None:
        super().__init__("omx_pick_" + robot_id.replace("-", "_"))
        self.pick = pick
        self.robot_id = robot_id
        self.busy = threading.Lock()  # 목표 실행 중 표시. on_goal에서 미리 보고 execute에서 실제로 잡는다
        # 정책이 수십 초 도는 동안에도 새 목표 거부와 취소 요청을 처리해야 하므로 재진입 그룹 + 멀티스레드 executor를 쓴다.
        self.server = ActionServer(
            self, PickObject, name,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f"{name} 대기 · {robot_id} · {pick.motion.describe()} · 정책 {sorted(pick.policies)}")

    def on_goal(self, request: PickObject.Goal) -> GoalResponse:
        """팔이 다른 목표를 실행 중이면 ROS 단계에서 거부한다."""
        if self.busy.locked():
            self.get_logger().warning(f"[{request.command_id}] 팔이 다른 목표를 실행 중이라 받지 않음")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle) -> PickObject.Result:
        """목표 하나를 실행한다. on_goal 뒤에 끼어든 두 번째 목표는 잠금을 못 얻어 WORKER_BUSY로 거부된다."""
        request: PickObject.Goal = goal_handle.request
        self.get_logger().info(f"[{request.command_id}] {request.arm_id} {request.object_class} ({request.policy_id})")
        if not self.busy.acquire(blocking=False):
            return self.finish(goal_handle, "REJECTED", "WORKER_BUSY")
        try:
            return self.run(goal_handle, request)
        finally:
            self.busy.release()

    def run(self, goal_handle, request: PickObject.Goal) -> PickObject.Result:
        """검사 → home 복귀 → 정책 준비 → 정책 실행 → home 정렬을 차례로 하고 결과를 돌려준다."""
        begin = time.monotonic()
        phase = {"name": "CHECKING", "message": ""}
        done = threading.Event()

        def feedback(name: str, message: str = "") -> None:
            phase.update(name=name, message=message)
            send_feedback()

        def send_feedback() -> None:
            message = PickObject.Feedback()
            message.phase = phase["name"]
            message.elapsed_s = time.monotonic() - begin
            message.message = phase["message"]
            message.simulated = self.pick.simulated
            goal_handle.publish_feedback(message)

        def heartbeat() -> None:  # 정책이 도는 동안 1초마다 경과 시간을 보낸다
            while not done.wait(1.0):
                send_feedback()

        feedback("CHECKING")
        # ---- 검사: 여기서 거부되면 팔은 움직이지 않았다 ----
        if request.arm_id != self.robot_id:
            return self.finish(goal_handle, "REJECTED", "ARM_MISMATCH", f"이 서버는 {self.robot_id}입니다 (목표 {request.arm_id})")
        if request.valid_until.sec == 0 and request.valid_until.nanosec == 0:
            return self.finish(goal_handle, "REJECTED", "VALID_UNTIL_MISSING")
        valid_until = datetime.fromtimestamp(request.valid_until.sec + request.valid_until.nanosec / 1e9, timezone.utc)
        policy_ref = {"policy_id": request.policy_id, "model": request.policy_model,
                      "registry_version": request.policy_registry_version}
        try:
            reason, policy = self.pick.check(policy_ref, valid_until, allow_simulation=True)
            if reason is None and policy["object_class"] != request.object_class:
                reason = "POLICY_UNKNOWN"
        except (ConnectionError, OSError, RuntimeError) as error:
            return self.finish(goal_handle, "REJECTED", "FOLLOWER_PORT_UNAVAILABLE", str(error)[:300])
        if reason:
            return self.finish(goal_handle, "REJECTED", reason)
        # 검사하는 동안 관제의 수락 기한이 끝날 수 있다. 기한이 지난 뒤에는 절대 정책을 시작하지 않는다.
        if datetime.now(timezone.utc) >= valid_until:
            return self.finish(goal_handle, "REJECTED", "EXPIRED")
        if goal_handle.is_cancel_requested:
            return self.finish(goal_handle, "CANCELED", "CANCELED_BEFORE_START")

        # ---- 여기부터 실물 팔이 움직인다 ----
        feedback("RETURNING_HOME", "정책 시작 전 저장된 초기자세로 복귀")
        ticker = threading.Thread(target=heartbeat, daemon=True)
        ticker.start()
        # 예외가 난 위치에 따라 사유 코드가 다르다: home 복귀 중이면 HOME_RETURN_ERROR, 정책 시작 뒤면 POLICY_RUN_ERROR.
        error_reason_code = "HOME_RETURN_ERROR"
        try:
            cancel_requested = lambda: goal_handle.is_cancel_requested
            end = self.pick.return_home_before_policy(cancel_requested)
            if end is None and goal_handle.is_cancel_requested:
                end = PickEnd("CANCELED", "OPERATOR_CANCELED", "초기자세 복귀 후 정책 시작 전에 취소되었습니다.",
                              pose_reached=True, phase="RETURNING_HOME")
            if end is None:
                self.pick.prepare()
                if goal_handle.is_cancel_requested:
                    end = PickEnd("CANCELED", "OPERATOR_CANCELED", "정책 준비 중 취소되어 정책을 시작하지 않았습니다.",
                                  pose_reached=True, phase="RETURNING_HOME")
                else:
                    error_reason_code = "POLICY_RUN_ERROR"
                    feedback("STARTING_POLICY", f"{policy['label']} 정책 시작")
                    result, details = self.pick.run(policy, cancel_requested=cancel_requested,
                                                    started=lambda: feedback("RUNNING_POLICY"))
                    end = self.pick.end(result, details, returning_home=lambda: feedback("RETURNING_HOME"))
        except Exception as error:  # 팔은 정책이 남긴 자리에서 토크를 유지한다. 사람이 가서 봐야 한다
            self.get_logger().error(f"  초기자세 복귀 또는 정책 실행 오류: {error}")
            end = PickEnd("RESULT_UNKNOWN", error_reason_code, str(error)[:300], phase="RETURNING_HOME")
        finally:
            done.set()
        return self.finish(goal_handle, outcome_of(end), end.reason_code or "", end.message or "",
                           pose_reached=bool(end.pose_reached), run_s=end.run_s)

    def finish(self, goal_handle, outcome: str, reason_code: str = "", message: str = "", *,
               pose_reached: bool = False, run_s: float = 0.0) -> PickObject.Result:
        """결과를 채우고 ROS 목표 상태를 정한다: 정상 끝(HOME_REACHED)만 succeeded, 취소는 canceled, 나머지는 aborted.
        판단은 ROS 상태가 아니라 outcome과 pose_reached로 한다."""
        result = PickObject.Result()
        result.outcome = outcome
        result.reason_code = reason_code
        result.message = message
        result.pose_reached = pose_reached
        result.simulated = self.pick.simulated
        result.run_s = float(run_s)
        if outcome == HOME_REACHED:
            goal_handle.succeed()
        elif outcome == "CANCELED":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        line = f"  -> {outcome} {reason_code}{' (모의)' if result.simulated else ''}"
        # rclpy는 한 호출 위치가 두 가지 심각도로 로그를 남기는 것을 거부하므로, 두 수준을 별도 호출로 둔다.
        if outcome == HOME_REACHED:
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)
        return result


def main() -> int:
    """인자를 읽고 이 팔의 action 서버를 띄워 Ctrl+C까지 돈다. 종료해도 토크는 켜 둔다."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="arm-01", choices=("arm-01", "arm-02"))
    robot_id = parser.parse_known_args()[0].robot_id  # 아래의 팔별 기본값이 이 값에 달려 있다
    parser.add_argument("--action", default=None, help=f"action name (default {action_name('arm-01')} for arm-01)")
    add_motion_arguments(parser)
    add_policy_arguments(parser, robot_id)
    # 이 팔의 팔로워 포트 설정(CONTROL_STATION_ARM0N_ / LEFT_ / RIGHT_FOLLOWER_PORT)이 있으면 --port 기본값으로 쓴다.
    if arm_env(robot_id, "FOLLOWER_PORT"):
        parser.set_defaults(port=arm_env(robot_id, "FOLLOWER_PORT"))
    args = parser.parse_args()
    finish_policy_arguments(args)

    pick = PolicyPick(SlotMotion(args), args)
    rclpy.init()
    node = PickActionServer(pick, robot_id, args.action or action_name(robot_id))
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n집기 액션 서버 종료 (토크는 켜 둔 채로 둡니다)", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
