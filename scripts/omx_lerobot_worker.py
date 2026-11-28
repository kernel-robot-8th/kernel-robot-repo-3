#!/usr/bin/env python3
"""OMX arm worker for the real follower: moves it to the joint pose taught for a slot (MOVE_TO_SLOT), back to
its recorded "home" pose (RETURN_HOME), or runs the trained ACT policy for an object (PICK_WITH_POLICY).

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
PICK_WITH_POLICY first returns to this arm's recorded home pose, then runs the LeRobot ACT policy trained for the
named object (omx_policy_pick.py, shared with the ROS 2 action server omx_pick_action_server.py) through the policy
server on this PC. It stops the policy once the arm is back at rest and aligns it to the recorded home pose. Whether
the object was grasped and placed is not checked, so the outcome is RESULT_UNKNOWN / PLACE_NOT_VERIFIED.
A policy recorded on one arm (arm_id) runs only in that arm's worker (--robot-id) on that arm's board
(board_serial in config/arms.json). With two arms, run one worker per arm; each takes its port and cameras from
CONTROL_STATION_ARM01_* / ARM02_* (FOLLOWER_PORT, GRIPPER_CAMERA, OVERHEAD_CAMERA) for its --robot-id.
Any other operation is rejected (NOT_IMPLEMENTED).
This does not replace the hardware power cut.
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

from mock_omx_worker import Refused, Worker, request_json
from omx_policy_pick import PolicyPick, add_policy_arguments, arm_env, finish_policy_arguments
from omx_slot_motion import ARM_JOINTS, GRIPPER_CLOSED, GRIPPER_OPEN, Outcome, Rejected, SlotMotion, add_motion_arguments

class LeRobotOmxWorker(Worker):
    banner = "실물 OMX 워커 {worker_id}: {url} 대기 (명령을 받으면 팔이 움직입니다)"

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args)
        self.motion = SlotMotion(args)
        self.goal: dict[str, int] = {}
        self.pick = PolicyPick(self.motion, args)
        print(self.motion.describe(), flush=True)
        print(f"학습 정책: {sorted(self.pick.policies)} · 위 카메라 {args.overhead_camera or '미지정'} · 그리퍼 카메라 "
              f"{args.gripper_camera} · 정책 서버 {args.policy_server}", flush=True)

    # ---- worker hooks ----
    def check(self, command: dict) -> str | None:
        target = command["target"]
        if command["operation"] == "JOG_JOINT":
            try:
                self.goal = self.motion.plan_jog(
                    target.get("joint"), target.get("direction"),
                    datetime.fromisoformat(command["valid_until"]), step_ticks=MANUAL_JOG_STEP_TICKS,
                )
            except Rejected as rejected:
                return rejected.reason_code
            return None
        if command["operation"] == "OPEN_GRIPPER":
            return None
        if command["operation"] not in ("MOVE_TO_SLOT", "RETURN_HOME"):
            return "NOT_IMPLEMENTED"
        # RETURN_HOME replays this worker's own "home" pose, which belongs to the slot map it was loaded with.
        home = command["operation"] == "RETURN_HOME"
        try:
            self.goal = self.motion.recorded_pose("home") if home else self.motion.plan(
                "home" if home else target.get("pick_slot_id"),
                self.motion.slot_map_version if home else target.get("slot_map_version"),
                datetime.fromisoformat(command["valid_until"]),
            )
        except Rejected as rejected:
            return rejected.reason_code
        return None

    # ---- PICK_WITH_POLICY (checks, run and home alignment are omx_policy_pick.PolicyPick) ----
    def handle_policy(self, command: dict) -> None:
        policy_ref = command["target"].get("policy") or {}
        print(f"[{command['command_id']}] PICK_WITH_POLICY {policy_ref.get('label')} ({policy_ref.get('model')})", flush=True)
        try:
            reason, policy = self.pick.check(policy_ref, datetime.fromisoformat(command["valid_until"]))
        except (ConnectionError, OSError, RuntimeError) as error:
            print(f"  팔로워 포트를 열 수 없음: {error}", flush=True)
            reason, policy = "FOLLOWER_PORT_UNAVAILABLE", None
        try:
            if reason:
                self.send(command, "REJECTED", reason_code=reason)
                return
            command = self.send(command, "ACCEPTED")  # only an HTTP 200 here allows motion
        except Refused as refused:
            print(f"  중단: 관제가 거부함 ({refused}). 더 진행하지 않습니다.", flush=True)
            return

        command_id = command["command_id"]

        def cancel_requested() -> bool:
            status, body = request_json("GET", f"{self.base}/api/arm/commands/{command_id}", timeout=5)
            if status != 200:
                raise RuntimeError(f"HTTP {status}")
            return body["command"]["cancel_requested"]

        try:
            command = self.send(command, "PROGRESS", phase="RETURNING_HOME",
                                message="정책 시작 전 저장된 초기자세로 복귀 중")
            home_end = self.pick.return_home_before_policy(cancel_requested)
            if home_end is not None:
                self.send(command, home_end.event, reason_code=home_end.reason_code, message=home_end.message,
                          pose_reached=home_end.pose_reached, phase=home_end.phase, details=home_end.details)
                return
            if cancel_requested():
                self.send(command, "CANCELED", reason_code="OPERATOR_CANCELED",
                          message="초기자세 복귀 후 정책 시작 전에 취소되었습니다.", pose_reached=True,
                          phase="RETURNING_HOME")
                return
            self.pick.prepare()
            if cancel_requested():
                self.send(command, "CANCELED", reason_code="OPERATOR_CANCELED",
                          message="정책 준비 중 취소되어 정책을 시작하지 않았습니다.", pose_reached=True,
                          phase="RETURNING_HOME")
                return
        except Refused as refused:
            print(f"  관제가 초기자세 복귀를 거부함 ({refused}). 정책은 시작하지 않았습니다.", flush=True)
            return
        except Exception as error:
            self.report_policy_error(command, f"초기자세 복귀 또는 정책 준비 실패: {error}")
            return

        try:
            result, policy_details = self.pick.run(
                policy, cancel_requested=cancel_requested,
                started=lambda: self.send(command, "PROGRESS", phase="POLICY_RUNNING"),
            )
        except Refused as refused:
            print(f"  관제가 진행 보고를 거부함 ({refused}). 정책은 멈췄고 팔은 그 자리에서 토크를 유지합니다.", flush=True)
            return
        except Exception as error:  # the arm keeps torque where the policy left it; a person has to look
            self.report_policy_error(command, str(error))
            return
        end = self.pick.end(result, policy_details)
        try:
            self.send(command, end.event, reason_code=end.reason_code, message=end.message,
                      pose_reached=end.pose_reached, phase=end.phase, details=end.details)
        except Refused as refused:
            print(f"  관제가 결과 보고를 거부함 ({refused})", flush=True)

    def report_policy_error(self, command: dict, message: str, policy_details: dict | None = None) -> None:
        print(f"  정책 실행 오류: {message}", flush=True)
        try:
            self.send(command, "RESULT_UNKNOWN", reason_code="POLICY_RUN_ERROR", message=message[:300],
                      details={"policy_run": policy_details} if policy_details else None)
        except Refused:
            pass

    def handle(self, command: dict) -> None:
        if command["operation"] == "PICK_WITH_POLICY":
            self.handle_policy(command)
            return
        if command["operation"] == "OPEN_GRIPPER":
            self.handle_open_gripper(command)
            return
        target = command["target"]
        if command["operation"] == "RETURN_HOME":
            where = "home"
        elif command["operation"] == "JOG_JOINT":
            where = f"{target.get('joint')} {target.get('direction')}"
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

    def handle_open_gripper(self, command: dict) -> None:
        """Open the gripper and report measured position; never infer release from the goal write."""
        accepted = False
        try:
            with self.motion.follower():
                errors = self.motion.bus.sync_read("Hardware_Error_Status", ["gripper"], normalize=False, num_retry=3)
                if int(errors["gripper"]):
                    self.send(command, "REJECTED", reason_code="MOTOR_HARDWARE_ERROR", message=str(errors))
                    return
                start = int(self.motion.bus.sync_read("Present_Position", ["gripper"], normalize=False, num_retry=3)["gripper"])
                command = self.send(command, "ACCEPTED")
                accepted = True
                if command["cancel_requested"]:
                    self.send(command, "CANCELED", phase="OPENING_GRIPPER")
                    return
                command = self.send(command, "PROGRESS", phase="OPENING_GRIPPER")
                final = self.motion.gripper_to(GRIPPER_OPEN, timeout_s=10.0)
                confirmed = final >= GRIPPER_OPEN - 50
                details = {"start_ticks": start, "goal_ticks": GRIPPER_OPEN, "final_ticks": final,
                           "tolerance_ticks": 50, "gripper_open_confirmed": confirmed}
                if confirmed:
                    self.send(command, "SUCCEEDED", details=details)
                else:
                    self.send(command, "FAILED", reason_code="GRIPPER_OPEN_NOT_CONFIRMED", details=details)
        except Refused as refused:
            print(f"  중단: 관제가 거부함 ({refused}). 더 진행하지 않습니다.", flush=True)
        except (ConnectionError, OSError, RuntimeError, KeyError, ValueError) as error:
            print(f"  그리퍼 열기 확인 오류: {error}", flush=True)
            try:
                if accepted:
                    self.send(command, "RESULT_UNKNOWN", reason_code="GRIPPER_OPEN_RESULT_UNKNOWN", message=str(error)[:300])
                else:
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

        is_jog = command["operation"] == "JOG_JOINT"
        if command["operation"] == "RETURN_HOME":
            # Position feedback can settle a few ticks from the commanded goal.
            home_tolerance = 50
            current, error_ticks, stationary = self.motion.pose_error(goal)
            if error_ticks <= home_tolerance and stationary:
                result = Outcome("SUCCEEDED", self.motion.simulated, pose_reached=True,
                                 final_ticks=current, max_error_ticks=error_ticks)
            else:
                previous_timeout = self.motion.settings.move_timeout
                self.motion.settings.move_timeout = max(previous_timeout, 90.0)
                try:
                    result = self.motion.move_staged(goal, cancel_requested, tolerance_ticks=home_tolerance)
                finally:
                    self.motion.settings.move_timeout = previous_timeout
                current, error_ticks, stationary = self.motion.pose_error(goal)
                if result.outcome == "FAILED" and error_ticks == 0 and stationary:
                    result = Outcome("SUCCEEDED", self.motion.simulated, pose_reached=True,
                                    final_ticks=current, max_error_ticks=error_ticks)
            details = {"goal_ticks": goal, "final_ticks": current, "max_error_ticks": error_ticks,
                       "tolerance_ticks": home_tolerance}
            if result.outcome == "SUCCEEDED":
                status, body = request_json(
                    "GET", f"{self.base}/api/arm/commands/{command['command_id']}", timeout=5
                )
                if status != 200:
                    self.send(command, "RESULT_UNKNOWN", reason_code="HOME_CANCEL_STATE_UNKNOWN",
                              message=f"HTTP {status}", pose_reached=True, details=details)
                    return
                if body["command"]["cancel_requested"]:
                    self.send(command, "CANCELED", phase="CLOSING_GRIPPER", pose_reached=True, details=details)
                    return
                command = self.send(command, "PROGRESS", phase="CLOSING_GRIPPER")
                gripper_start = int(self.motion.bus.sync_read(
                    "Present_Position", ["gripper"], normalize=False, num_retry=3
                )["gripper"])
                gripper_final = self.motion.gripper_to(GRIPPER_CLOSED, timeout_s=10.0)
                gripper_closed = gripper_final <= GRIPPER_CLOSED + 50
                details.update({
                    "gripper_start_ticks": gripper_start,
                    "gripper_goal_ticks": GRIPPER_CLOSED,
                    "gripper_final_ticks": gripper_final,
                    "gripper_tolerance_ticks": 50,
                    "gripper_closed_confirmed": gripper_closed,
                })
                if not gripper_closed:
                    self.send(command, "FAILED", reason_code="GRIPPER_CLOSE_NOT_CONFIRMED",
                              message="팔은 home에 도착했지만 그리퍼 닫힘은 확인되지 않았습니다.",
                              pose_reached=True, details=details)
                    return
        else:
            result = self.motion.move(
                goal,
                cancel_requested,
                profile_velocity=2 if is_jog else None,
                profile_acceleration=1 if is_jog else None,
            )
            details = {"goal_ticks": goal, "final_ticks": result.final_ticks,
                       "max_error_ticks": result.max_error_ticks}
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
    robot_id = parser.parse_known_args()[0].robot_id  # the per-arm defaults below depend on it
    parser.add_argument("--worker-id", default="omx-lerobot-worker")
    parser.add_argument("--wait-s", type=float, default=20)
    parser.add_argument("--once", action="store_true", help="exit after one command")
    add_motion_arguments(parser)
    add_policy_arguments(parser, robot_id)
    parser.add_argument("--release-torque", action="store_true", help="turn arm torque off and exit (hold the arm first)")
    if arm_env(robot_id, "FOLLOWER_PORT"):
        parser.set_defaults(port=arm_env(robot_id, "FOLLOWER_PORT"))
    # This arm's pose file (CONTROL_STATION_ARM01_OMX_POSES or _LEFT_OMX_POSES); SlotMotion refuses a pose file
    # recorded on another board, so a wrong default stops instead of moving.
    if arm_env(robot_id, "OMX_POSES"):
        parser.set_defaults(poses=Path(arm_env(robot_id, "OMX_POSES")))
    args = parser.parse_args()
    finish_policy_arguments(args)
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
