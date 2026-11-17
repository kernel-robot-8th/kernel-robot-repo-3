#!/usr/bin/env python3
"""One arm's PICK_WITH_POLICY as a ROS 2 action server: /arm_01/pick_object (omx_interfaces/action/PickObject).

The same pick as the HTTP worker (omx_lerobot_worker.py): the checks, the ACT policy run and the home alignment are
omx_policy_pick.PolicyPick, so the same rules apply (policy recorded on this arm and board, registry SHA and model,
start from rest, cancel stops in place with torque on). Run one server per arm, in the LeRobot environment with
ROS 2 and the interface package sourced:

    source /opt/ros/jazzy/setup.bash && source cpp/install/setup.bash
    export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_pick_action_server.py --robot-id arm-01"   # real arm
    ~/lerobot-venv/bin/python scripts/omx_pick_action_server.py --robot-id arm-01 --fake-follower    # no arm

How a goal ends:
- one goal at a time; a goal that arrives while the arm is busy is refused by ROS (not accepted)
- checks fail -> aborted, outcome REJECTED; nothing moved
- feedback CHECKING, then STARTING_POLICY right before the policy may move the arm (nothing moves before it),
  RUNNING_POLICY once the policy runs, RETURNING_HOME before the home alignment
- back home -> succeeded, outcome HOME_REACHED_PLACE_NOT_VERIFIED, pose_reached true: the arm cannot see whether
  the object was placed; the control station checks that with its cameras
- cancel -> the policy stops where the arm is -> canceled, outcome CANCELED
- FAILED / RESULT_UNKNOWN (aborted) for a timeout away from rest, a policy error or a failed home move; never retried
--fake-follower and --dry-run run no policy (no camera, no policy server): they wait --simulated-run-s, then do the
home alignment on the fake arm (or only read the real one); every such result carries simulated true.
This does not replace the hardware power cut.
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

# PickEnd.event (the HTTP worker's words) -> action outcome; PLACE_NOT_VERIFIED at home is the expected ending.
HOME_REACHED = "HOME_REACHED_PLACE_NOT_VERIFIED"


def action_name(robot_id: str) -> str:
    return "/" + robot_id.replace("-", "_") + "/pick_object"


def outcome_of(end: PickEnd) -> str:
    if end.event == "RESULT_UNKNOWN" and end.reason_code == "PLACE_NOT_VERIFIED" and end.pose_reached:
        return HOME_REACHED
    return end.event  # FAILED, CANCELED, RESULT_UNKNOWN


class PickActionServer(Node):
    def __init__(self, pick: PolicyPick, robot_id: str, name: str) -> None:
        super().__init__("omx_pick_" + robot_id.replace("-", "_"))
        self.pick = pick
        self.robot_id = robot_id
        self.busy = threading.Lock()
        self.server = ActionServer(
            self, PickObject, name,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f"{name} 대기 · {robot_id} · {pick.motion.describe()} · 정책 {sorted(pick.policies)}")

    def on_goal(self, request: PickObject.Goal) -> GoalResponse:
        if self.busy.locked():
            self.get_logger().warning(f"[{request.command_id}] 팔이 다른 목표를 실행 중이라 받지 않음")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle) -> PickObject.Result:
        request: PickObject.Goal = goal_handle.request
        self.get_logger().info(f"[{request.command_id}] {request.arm_id} {request.object_class} ({request.policy_id})")
        if not self.busy.acquire(blocking=False):
            return self.finish(goal_handle, "REJECTED", "WORKER_BUSY")
        try:
            return self.run(goal_handle, request)
        finally:
            self.busy.release()

    def run(self, goal_handle, request: PickObject.Goal) -> PickObject.Result:
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

        def heartbeat() -> None:  # elapsed time once a second while the policy runs
            while not done.wait(1.0):
                send_feedback()

        feedback("CHECKING")
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
            if reason is None:
                self.pick.prepare()
        except (ConnectionError, OSError, RuntimeError) as error:
            return self.finish(goal_handle, "REJECTED", "FOLLOWER_PORT_UNAVAILABLE", str(error)[:300])
        if reason:
            return self.finish(goal_handle, "REJECTED", reason)
        # The control station's acceptance window may end while the checks run; never start the policy after it.
        if datetime.now(timezone.utc) >= valid_until:
            return self.finish(goal_handle, "REJECTED", "EXPIRED")
        if goal_handle.is_cancel_requested:
            return self.finish(goal_handle, "CANCELED", "CANCELED_BEFORE_START")

        feedback("STARTING_POLICY", f"{policy['label']} 정책 시작")
        ticker = threading.Thread(target=heartbeat, daemon=True)
        ticker.start()
        try:
            result, details = self.pick.run(policy, cancel_requested=lambda: goal_handle.is_cancel_requested,
                                            started=lambda: feedback("RUNNING_POLICY"))
            end = self.pick.end(result, details, returning_home=lambda: feedback("RETURNING_HOME"))
        except Exception as error:  # the arm keeps torque where the policy left it; a person has to look
            self.get_logger().error(f"  정책 실행 오류: {error}")
            end = PickEnd("RESULT_UNKNOWN", "POLICY_RUN_ERROR", str(error)[:300])
        finally:
            done.set()
        return self.finish(goal_handle, outcome_of(end), end.reason_code or "", end.message or "",
                           pose_reached=bool(end.pose_reached), run_s=end.run_s)

    def finish(self, goal_handle, outcome: str, reason_code: str = "", message: str = "", *,
               pose_reached: bool = False, run_s: float = 0.0) -> PickObject.Result:
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
        # rclpy refuses one call site logging at two severities, so the two levels stay separate calls.
        if outcome == HOME_REACHED:
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)
        return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="arm-01", choices=("arm-01", "arm-02"))
    robot_id = parser.parse_known_args()[0].robot_id  # the per-arm defaults below depend on it
    parser.add_argument("--action", default=None, help=f"action name (default {action_name('arm-01')} for arm-01)")
    add_motion_arguments(parser)
    add_policy_arguments(parser, robot_id)
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
