#!/usr/bin/env python3
"""OMX arm worker for the real follower: moves it to the joint pose taught for a slot (MOVE_TO_SLOT), back to
its recorded "home" pose (RETURN_HOME), or brings the tool point to an XYZ position in omx_link0 with the wrist
level (MOVE_TO_POSITION, closed-form IK in omx_kinematics.py; the tick-to-angle mapping is not yet measured).

Runs in the LeRobot environment with serial access:

    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_lerobot_worker.py --server http://127.0.0.1:8000"

It uses LeRobot's Dynamixel bus directly instead of `OMXFollower.send_action`, because connecting the
LeRobot robot rewrites motor settings and enables torque without first aligning the goal to the present
position, and its per-call step limit is not a speed limit. Here the servo's own trapezoidal profile
(Profile_Velocity/Profile_Acceleration) limits speed.

The motion itself and its safety checks live in omx_slot_motion.py, shared with the ROS 2 action worker
(omx_ros_worker.py). This HTTP worker adds:
- moves only after the control station answers ACCEPTED with HTTP 200
- checks the command's cancel flag about once a second while moving
- --release-torque turns arm torque off and exits (support the arm by hand first)
This worker only brings the arm to the slot. Grasping and placing are done afterwards by a LeRobot-trained
policy (another part of the team), so PICK_AND_PLACE commands are rejected (NOT_IMPLEMENTED).
This does not replace the hardware power cut.
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime

from mock_omx_worker import Refused, Worker, request_json
from omx_kinematics import FRAME_ID, Unreachable, inverse, radians_to_ticks, tick_to_radians
from omx_slot_motion import ARM_JOINTS, Outcome, Rejected, SlotMotion, add_motion_arguments


class LeRobotOmxWorker(Worker):
    banner = "실물 OMX 워커 {worker_id}: {url} 대기 (명령을 받으면 팔이 움직입니다)"

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args)
        self.motion = SlotMotion(args)
        self.goal: dict[str, int] = {}
        self.correction: dict | None = None
        print(self.motion.describe(), flush=True)

    # ---- worker hooks ----
    def check(self, command: dict) -> str | None:
        target = command["target"]
        if command["operation"] == "MOVE_TO_POSITION":
            return self.check_position(command)
        if command["operation"] not in ("MOVE_TO_SLOT", "RETURN_HOME"):
            return "NOT_IMPLEMENTED"
        # RETURN_HOME replays this worker's own "home" pose, which belongs to the slot map it was loaded with.
        home = command["operation"] == "RETURN_HOME"
        try:
            self.goal, self.correction = self.motion.plan(
                "home" if home else target.get("pick_slot_id"),
                self.motion.slot_map_version if home else target.get("slot_map_version"),
                None if home else target.get("lateral_offset_m"),
                datetime.fromisoformat(command["valid_until"]),
            )
        except Rejected as rejected:
            return rejected.reason_code
        return None

    def check_position(self, command: dict) -> str | None:
        """IK for the tool point with the wrist level (approach FRONT_HORIZONTAL); wrist roll stays where it is."""
        target = command["target"]
        if target.get("frame_id") != FRAME_ID:
            return "FRAME_UNSUPPORTED"
        if target.get("approach") != "FRONT_HORIZONTAL":
            return "APPROACH_UNSUPPORTED"
        position = target["position"]
        now = tick_to_radians(self.motion.present())
        try:
            angles = inverse((position["x"], position["y"], position["z"]), 0.0, now["wrist_roll"], seed=now)
        except Unreachable as error:
            print(f"  IK 실패: {error}", flush=True)
            return error.reason_code
        self.correction = None
        try:
            self.goal = self.motion.check_goal(radians_to_ticks(angles), datetime.fromisoformat(command["valid_until"]))
        except Rejected as rejected:
            return rejected.reason_code
        return None

    def handle(self, command: dict) -> None:
        target = command["target"]
        if command["operation"] == "MOVE_TO_POSITION":
            where = f"{target.get('frame_id')} {target.get('position')} m"
        elif command["operation"] == "RETURN_HOME":
            where = "home"
        else:
            where = f"{target.get('pick_slot_id')} ({target.get('slot_label')})"
        print(f"[{command['command_id']}] {command['operation']} {where}", flush=True)
        try:
            with self.motion.follower():
                reason = self.check(command)
                try:
                    if reason:
                        self.send(command, "REJECTED", reason_code=reason)
                        return
                    command = self.send(command, "ACCEPTED")  # only an HTTP 200 here allows motion
                    self.move(command, self.goal)
                except Refused as refused:
                    self.motion.hold_here()
                    print(f"  중단: 관제가 거부함 ({refused}). 더 진행하지 않습니다.", flush=True)
        except (ConnectionError, OSError, RuntimeError) as error:
            # The port may be in use by the policy or teleoperation; nothing was sent to the motors.
            print(f"  팔로워 포트를 열 수 없음: {error}", flush=True)
            try:
                self.send(command, "REJECTED", reason_code="FOLLOWER_PORT_UNAVAILABLE", message=str(error)[:300])
            except Refused:
                pass

    def move(self, command: dict, goal: dict[str, int]) -> None:
        command = self.send(command, "PROGRESS", phase="APPROACH_TARGET")
        next_check = time.monotonic() + 1.0

        def cancel_requested() -> bool:
            nonlocal next_check
            if time.monotonic() < next_check:
                return False
            next_check = time.monotonic() + 1.0
            status, body = request_json("GET", f"{self.base}/api/arm/commands/{command['command_id']}", timeout=5)
            return status == 200 and body["command"]["cancel_requested"]

        result = self.motion.move(goal, cancel_requested)
        details = {"goal_ticks": goal, "final_ticks": result.final_ticks, "max_error_ticks": result.max_error_ticks,
                   "lateral_correction": self.correction}
        if result.outcome == "SUCCEEDED":
            self.send(command, "SUCCEEDED", result, pose_reached=True, details=details)
        elif result.outcome == "CANCELED":
            self.send(command, "CANCELED", result, phase="APPROACH_TARGET", details=details)
        else:
            self.send(command, result.outcome, result, reason_code=result.reason_code, message=result.message or None,
                      details=details)

    def send(self, command: dict, event: str, result: Outcome | None = None, **fields: object) -> dict:
        fields["simulated"] = result.simulated if result else self.motion.simulated
        fields = {key: value for key, value in fields.items() if value is not None}
        return super().send(command, event, **fields)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("--robot-id", default="arm-01")
    parser.add_argument("--worker-id", default="omx-lerobot-worker")
    parser.add_argument("--wait-s", type=float, default=20)
    parser.add_argument("--once", action="store_true", help="exit after one command")
    add_motion_arguments(parser)
    parser.add_argument("--release-torque", action="store_true", help="turn arm torque off and exit (hold the arm first)")
    args = parser.parse_args()
    args.frame_id = "omx_link0"
    args.result = "succeeded"
    args.phase_delay = 0.0

    if args.release_torque:
        from record_omx_pose import open_follower

        bus = open_follower(args.port)
        try:
            bus.sync_write("Torque_Enable", {name: 0 for name in ARM_JOINTS}, normalize=False, num_retry=3)
            print("팔 관절 토크를 껐습니다.")
        finally:
            bus.disconnect(disable_torque=False)
        return 0

    worker = LeRobotOmxWorker(args)
    try:
        return worker.run()
    except KeyboardInterrupt:
        print("\n워커 종료 (토크는 켜 둔 채로 둡니다)", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
