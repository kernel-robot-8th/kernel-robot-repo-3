#!/usr/bin/env python3
"""OMX worker as a ROS 2 action server: /omx/move_to_slot (omx_interfaces/action/MoveToSlot).

Brings the follower to the joint pose taught for a shelf slot, the same motion as the HTTP worker
(omx_slot_motion.py). Run it in the LeRobot environment with ROS 2 and the interface package sourced:

    source /opt/ros/jazzy/setup.bash && source cpp/install/setup.bash
    export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_ros_worker.py"          # real arm
    ~/lerobot-venv/bin/python scripts/omx_ros_worker.py --fake-follower           # no arm, simulated

How a goal ends:
- one goal at a time; a goal that arrives while the arm is moving is refused by ROS (not accepted)
- checks fail -> aborted, outcome REJECTED; nothing moved and no feedback was sent
- the first feedback (phase MOVING) comes right before the goal position is written: nothing moves before it
- reached and stopped -> succeeded, outcome SUCCEEDED, pose_reached true
- cancel -> the arm holds where it is -> canceled, outcome CANCELED
- timeout -> FAILED, motor or cancel-check error -> RESULT_UNKNOWN (both aborted); never retried here
Results from --dry-run or --fake-follower carry simulated true.
This worker only brings the arm to the slot; grasping is done by a LeRobot-trained policy afterwards.
This does not replace the hardware power cut.
"""

from __future__ import annotations

import argparse
import math
import threading
from datetime import datetime, timezone

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from omx_interfaces.action import MoveToSlot
from omx_slot_motion import ARM_JOINTS, Outcome, Rejected, SlotMotion, add_motion_arguments


SHELF_FRAME = "shelf_front"


class OmxActionServer(Node):
    def __init__(self, motion: SlotMotion, action_name: str) -> None:
        super().__init__("omx_worker")
        self.motion = motion
        self.busy = threading.Lock()
        self.server = ActionServer(
            self, MoveToSlot, action_name,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f"{action_name} 대기 · {motion.describe()}")

    def on_goal(self, request: MoveToSlot.Goal) -> GoalResponse:
        if self.busy.locked():
            self.get_logger().warning(f"[{request.command_id}] 팔이 다른 목표를 실행 중이라 받지 않음")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle) -> MoveToSlot.Result:
        request: MoveToSlot.Goal = goal_handle.request
        self.get_logger().info(f"[{request.command_id}] 칸 {request.slot_id}")
        if not self.busy.acquire(blocking=False):
            return self.finish(goal_handle, Outcome("REJECTED", self.motion.simulated, "WORKER_BUSY"))
        try:
            return self.finish(goal_handle, *self.run(goal_handle, request))
        finally:
            self.busy.release()

    def run(self, goal_handle, request: MoveToSlot.Goal) -> tuple[Outcome, dict | None]:
        if request.object_position.header.frame_id not in ("", SHELF_FRAME):
            return Outcome("REJECTED", self.motion.simulated, "FRAME_UNSUPPORTED",
                           f"object_position must be in {SHELF_FRAME}"), None
        if request.valid_until.sec == 0 and request.valid_until.nanosec == 0:
            return Outcome("REJECTED", self.motion.simulated, "VALID_UNTIL_MISSING"), None
        valid_until = datetime.fromtimestamp(request.valid_until.sec + request.valid_until.nanosec / 1e9, timezone.utc)
        # lateral_offset_m is ignored: shoulder-pan correction from camera coordinates was dropped (2026-11-11).

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
                    return Outcome("REJECTED", self.motion.simulated, rejected.reason_code, rejected.message), None
                return self.motion.move(goal, lambda: goal_handle.is_cancel_requested, progress), None
        except (ConnectionError, OSError, RuntimeError) as error:
            # The port may be in use by the policy or teleoperation; nothing was sent to the motors.
            return Outcome("REJECTED", self.motion.simulated, "FOLLOWER_PORT_UNAVAILABLE", str(error)[:300]), None

    def finish(self, goal_handle, outcome: Outcome, correction: dict | None = None) -> MoveToSlot.Result:
        result = MoveToSlot.Result()
        result.outcome = outcome.outcome
        result.pose_reached = outcome.pose_reached
        result.reason_code = outcome.reason_code
        result.message = outcome.message
        result.joint_names = list(outcome.final_ticks)
        result.final_ticks = list(outcome.final_ticks.values())
        result.max_error_ticks = outcome.max_error_ticks
        result.pan_correction_degrees = correction["pan_degrees"] if correction else math.nan
        result.simulated = outcome.simulated
        if outcome.outcome == "SUCCEEDED":
            goal_handle.succeed()
        elif outcome.outcome == "CANCELED":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        line = f"  -> {outcome.outcome} {outcome.reason_code}{' (모의)' if outcome.simulated else ''}"
        # rclpy refuses one call site logging at two severities, so the two levels stay separate calls.
        if outcome.outcome == "SUCCEEDED":
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)
        return result


def main() -> int:
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
