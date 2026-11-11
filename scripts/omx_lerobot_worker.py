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
PICK_WITH_POLICY runs the LeRobot ACT policy trained for the named object (omx_act_policy_run.py) through the
policy server on this PC, stops it once the arm is back at rest, then aligns it to the recorded home pose. Whether
the object was grasped and placed is not checked, so the outcome is RESULT_UNKNOWN / PLACE_NOT_VERIFIED.
Any other operation is rejected (NOT_IMPLEMENTED).
This does not replace the hardware power cut.
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
from pathlib import Path

from mock_omx_worker import Refused, Worker, request_json
from omx_act_policy_run import (
    DEFAULT_POLICIES, RestWatcher, check_policy_server, load_policies, policy_cameras, run_policy, wait_devices_free,
)
from omx_slot_motion import ARM_JOINTS, Outcome, Rejected, SlotMotion, add_motion_arguments

DEFAULT_GRIPPER_CAMERA = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0"


class LeRobotOmxWorker(Worker):
    banner = "실물 OMX 워커 {worker_id}: {url} 대기 (명령을 받으면 팔이 움직입니다)"

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args)
        self.motion = SlotMotion(args)
        self.goal: dict[str, int] = {}
        self.policy_version, self.policies = load_policies(args.policies)
        self.camera_devices = {"OVERHEAD": args.overhead_camera, "GRIPPER": args.gripper_camera}
        print(self.motion.describe(), flush=True)
        print(f"학습 정책: {sorted(self.policies)} · 위 카메라 {args.overhead_camera or '미지정'} · 그리퍼 카메라 "
              f"{args.gripper_camera} · 정책 서버 {args.policy_server}", flush=True)

    # ---- worker hooks ----
    def check(self, command: dict) -> str | None:
        target = command["target"]
        if command["operation"] not in ("MOVE_TO_SLOT", "RETURN_HOME"):
            return "NOT_IMPLEMENTED"
        # RETURN_HOME replays this worker's own "home" pose, which belongs to the slot map it was loaded with.
        home = command["operation"] == "RETURN_HOME"
        try:
            self.goal = self.motion.plan(
                "home" if home else target.get("pick_slot_id"),
                self.motion.slot_map_version if home else target.get("slot_map_version"),
                datetime.fromisoformat(command["valid_until"]),
            )
        except Rejected as rejected:
            return rejected.reason_code
        return None

    # ---- PICK_WITH_POLICY ----
    def check_policy(self, command: dict) -> tuple[str | None, dict | None]:
        """(reason to reject, policy). Reads the arm but writes nothing."""
        settings = self.args
        policy_ref = command["target"].get("policy") or {}
        if settings.fake_follower or settings.dry_run:
            return "POLICY_NEEDS_REAL_ARM", None
        if policy_ref.get("registry_version") != self.policy_version:
            return "POLICY_REGISTRY_MISMATCH", None
        policy = self.policies.get(policy_ref.get("policy_id"))
        if policy is None or policy["model"] != policy_ref.get("model"):
            return "POLICY_UNKNOWN", None
        if "home" not in self.motion.poses["poses"]:
            return "POSE_NOT_RECORDED", None
        if datetime.fromisoformat(command["valid_until"]) <= datetime.now(timezone.utc):
            return "EXPIRED", None
        missing = [role for role, path in self.camera_devices.items() if not path or not os.path.exists(path)]
        if missing:
            print(f"  카메라 없음: {missing} {self.camera_devices}", flush=True)
            return "CAMERA_UNAVAILABLE", None
        # The control station closes these cameras when it hands out this command; wait for that.
        busy = wait_devices_free(list(self.camera_devices.values()))
        if busy:
            print(f"  다른 프로그램이 카메라를 쓰는 중: {busy}", flush=True)
            return "CAMERA_BUSY", None
        problem = check_policy_server(settings.policy_server)
        if problem:
            print(f"  {problem}", flush=True)
            return "POLICY_SERVER_UNAVAILABLE", None
        home = self.motion.poses["poses"]["home"]["ticks"]
        with self.motion.follower():
            now = self.motion.present()
        away = max(abs(now[name] - home[name]) for name in ARM_JOINTS)
        print(f"  현재 {now}\n  home과 최대 차이 {away} ticks", flush=True)
        if away > settings.policy_start_ticks:
            if not settings.policy_any_start:
                return "NOT_AT_REST", None  # the policy was trained starting from rest
            print(f"  실험 모드: 쉬는 자세가 아닌 곳에서 정책 시작 (한 스텝 최대 {settings.policy_max_step_ticks} ticks)", flush=True)
        return None, policy

    def handle_policy(self, command: dict) -> None:
        settings = self.args
        policy_ref = command["target"].get("policy") or {}
        print(f"[{command['command_id']}] PICK_WITH_POLICY {policy_ref.get('label')} ({policy_ref.get('model')})", flush=True)
        try:
            reason, policy = self.check_policy(command)
        except (ConnectionError, OSError, RuntimeError) as error:
            print(f"  팔로워 포트를 열 수 없음: {error}", flush=True)
            reason, policy = "FOLLOWER_PORT_UNAVAILABLE", None
        try:
            if reason:
                self.send(command, "REJECTED", reason_code=reason)
                return
            command = self.send(command, "ACCEPTED")  # only an HTTP 200 here allows motion
            with self.motion.follower():
                # Run the policy at the servo speed it was recorded with (power-on default 0 = no profile limit);
                # the slot/home moves of this worker leave a slow profile in RAM.
                self.motion.write("Profile_Velocity", {name: 0 for name in ARM_JOINTS})
                self.motion.write("Profile_Acceleration", {name: 0 for name in ARM_JOINTS})
        except Refused as refused:
            print(f"  중단: 관제가 거부함 ({refused}). 더 진행하지 않습니다.", flush=True)
            return
        except (ConnectionError, OSError, RuntimeError) as error:
            self.report_policy_error(command, f"profile reset failed: {error}")
            return

        home = {name: int(self.motion.poses["poses"]["home"]["ticks"][name]) for name in ARM_JOINTS}
        watcher = RestWatcher(home, settings.policy_depart_ticks, settings.policy_rest_ticks, settings.policy_settle_s)
        command_id = command["command_id"]

        def cancel_requested() -> bool:
            status, body = request_json("GET", f"{self.base}/api/arm/commands/{command_id}", timeout=5)
            if status != 200:
                raise RuntimeError(f"HTTP {status}")
            return body["command"]["cancel_requested"]

        cameras = policy_cameras(policy, self.camera_devices)
        print(f"  정책 실행: {policy['model']} · front={cameras['front']} · wrist={cameras['wrist']}", flush=True)
        try:
            result = run_policy(
                policy, port=settings.port, cameras=cameras, server_address=settings.policy_server, watcher=watcher,
                max_run_s=policy["max_run_s"], cancel_requested=cancel_requested,
                started=lambda: self.send(command, "PROGRESS", phase="APPROACH_TARGET"),
                max_step_ticks=settings.policy_max_step_ticks,
                policy_device=settings.policy_device,
            )
        except Refused as refused:
            print(f"  관제가 진행 보고를 거부함 ({refused}). 정책은 멈췄고 팔은 그 자리에서 토크를 유지합니다.", flush=True)
            return
        except Exception as error:  # the arm keeps torque where the policy left it; a person has to look
            self.report_policy_error(command, str(error))
            return
        policy_details = {"policy_id": policy["policy_id"], "model": policy["model"], **asdict(result)}
        print(f"  정책 종료: {result.stop_reason}, {result.run_s:.1f}s, 행동 {result.actions_sent}개, "
              f"home에서 가장 멀리 {result.farthest_from_home_ticks} ticks", flush=True)
        try:
            if result.stop_reason == "RETURNED_TO_REST":
                self.finish_at_home(command, policy_details)
            elif result.stop_reason == "CANCELED":
                self.send(command, "CANCELED", phase="APPROACH_TARGET", details={"policy_run": policy_details})
            elif result.stop_reason == "NEVER_DEPARTED":
                self.send(command, "FAILED", reason_code="POLICY_DID_NOT_MOVE", details={"policy_run": policy_details})
            elif result.stop_reason == "TIMEOUT":
                # Not back at rest: it may hold the object or be near the shelf, so no automatic move home.
                self.send(command, "FAILED", reason_code="POLICY_TIMEOUT_NOT_AT_REST",
                          message="정책을 멈췄고 팔은 그 자리에서 토크를 유지합니다. 현장 확인 뒤 원상복귀하세요.",
                          details={"policy_run": policy_details})
            else:
                self.send(command, "RESULT_UNKNOWN", reason_code="POLICY_RUN_ERROR", message=result.message[:300] or None,
                          details={"policy_run": policy_details})
        except Refused as refused:
            print(f"  관제가 결과 보고를 거부함 ({refused})", flush=True)

    def finish_at_home(self, command: dict, policy_details: dict) -> None:
        """The policy came back to rest: align to the recorded home pose. The command was accepted in time, so the
        follow-up move is not tied to valid_until."""
        time.sleep(0.5)  # let the servos finish the last goal the policy wrote
        try:
            with self.motion.follower():
                goal = self.motion.plan("home", self.motion.slot_map_version,
                                        datetime.now(timezone.utc) + timedelta(seconds=60))
                now = self.motion.present()
                away = max(abs(goal[name] - now[name]) for name in ARM_JOINTS)
                if away > 2 * self.args.policy_rest_ticks:
                    self.send(command, "FAILED", reason_code="REST_NOT_CONFIRMED",
                              message=f"정책은 쉬는 자세로 돌아왔다고 봤지만 실제 관절이 home에서 {away} ticks 떨어져 있어 움직이지 않았습니다.",
                              details={"policy_run": policy_details, "present_ticks": now})
                    return
                outcome = self.motion.move(goal, lambda: False)
        except Rejected as rejected:
            self.send(command, "FAILED", reason_code=rejected.reason_code, details={"policy_run": policy_details})
            return
        except (ConnectionError, OSError, RuntimeError) as error:
            self.report_policy_error(command, f"home alignment: {error}", policy_details)
            return
        home_details = {"goal_ticks": goal, "final_ticks": outcome.final_ticks, "max_error_ticks": outcome.max_error_ticks}
        details = {"policy_run": policy_details, "home_return": home_details}
        if outcome.outcome == "SUCCEEDED":
            # The arm is home; grasp and placement were not observed, so success cannot be claimed.
            self.send(command, "RESULT_UNKNOWN", reason_code="PLACE_NOT_VERIFIED", pose_reached=True,
                      message="정책 동작 뒤 home 복귀 완료. 집기·적재는 확인하지 않았습니다.", details=details)
        else:
            self.send(command, "FAILED" if outcome.outcome == "FAILED" else "RESULT_UNKNOWN",
                      reason_code=outcome.reason_code or "HOME_RETURN_FAILED", message=outcome.message or None,
                      details=details)

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
        target = command["target"]
        if command["operation"] == "RETURN_HOME":
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
        details = {"goal_ticks": goal, "final_ticks": result.final_ticks, "max_error_ticks": result.max_error_ticks}
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
    policy = parser.add_argument_group("PICK_WITH_POLICY (LeRobot ACT)")
    policy.add_argument("--policies", type=Path, default=DEFAULT_POLICIES, help="object -> trained policy file")
    policy.add_argument("--policy-server", default="127.0.0.1:8080", help="lerobot.async_inference.policy_server address")
    policy.add_argument("--overhead-camera", default=os.environ.get("CONTROL_STATION_ACT_OVERHEAD_CAMERA", ""),
                        help="camera seeing the shelf from above, as in training (env CONTROL_STATION_ACT_OVERHEAD_CAMERA)")
    policy.add_argument("--gripper-camera", default=os.environ.get("CONTROL_STATION_JOINT_CAMERA") or DEFAULT_GRIPPER_CAMERA,
                        help="camera on the gripper (env CONTROL_STATION_JOINT_CAMERA)")
    policy.add_argument("--policy-start-ticks", type=int, default=400,
                        help="refuse to start the policy when a joint is farther than this from home")
    policy.add_argument("--policy-depart-ticks", type=int, default=300, help="a joint this far from home = policy started moving")
    policy.add_argument("--policy-rest-ticks", type=int, default=150, help="all joints this close to home = back at rest")
    policy.add_argument("--policy-settle-s", type=float, default=1.5, help="seconds at rest before stopping the policy")
    policy.add_argument("--policy-device", default="cuda",
                        help="where the policy server runs the model (cpu while another job fills the GPU)")
    policy.add_argument("--policy-any-start", action="store_true",
                        help="EXPERIMENT: also start the policy away from rest (e.g. after MOVE_TO_SLOT); needs a step limit")
    policy.add_argument("--policy-max-step-ticks", type=int, default=None,
                        help="limit each policy goal to present ± this many ticks per 1/30 s step (default: none, "
                             "40 with --policy-any-start)")
    parser.add_argument("--release-torque", action="store_true", help="turn arm torque off and exit (hold the arm first)")
    args = parser.parse_args()
    if args.policy_any_start and args.policy_max_step_ticks is None:
        args.policy_max_step_ticks = 40  # about 1,200 ticks/s at 30 Hz; the first actions from an unseen pose may jump
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
