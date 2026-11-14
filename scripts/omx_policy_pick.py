"""One PICK_WITH_POLICY on one arm: the checks before motion, the ACT policy run and the move back to home.

Shared by the HTTP worker (omx_lerobot_worker.py) and the ROS 2 action server (omx_pick_action_server.py), so both
apply the same rules:
- a policy recorded on one arm (arm_id) runs only on that arm (--robot-id) and on that arm's board (board_serial
  in config/arms.json): otherwise POLICY_WRONG_ARM
- the command's policy registry SHA-256 and model must match this PC's policy file: POLICY_REGISTRY_MISMATCH,
  POLICY_UNKNOWN
- the policy starts only from rest, with free cameras and a reachable policy server; not after valid_until
- a cancel stops the policy where the arm is; torque stays on (the arm must not drop)
- back at rest, the arm is aligned to its recorded home pose. Grasp and placement are not observed, so the best
  ending is RESULT_UNKNOWN / PLACE_NOT_VERIFIED with pose_reached true

Neither transport is known here: the caller says when motion may start (after its own ACCEPTED), how to learn of a
cancel, and what to do on progress. Results use the HTTP worker's event words (RESULT_UNKNOWN, FAILED, CANCELED);
the action server maps them to its outcomes.

A simulated run (--fake-follower: no arm; --dry-run: reads the real arm, writes nothing) runs no policy, opens no
camera and needs no policy server; it only waits, then does the home alignment the same way. Every simulated
result says so.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable

from omx_act_policy_run import (
    DEFAULT_POLICIES, PolicyRunResult, RestWatcher, check_policy_server, load_policies, policy_cameras, run_policy,
    wait_devices_free,
)
from omx_slot_motion import ARM_JOINTS, REPOSITORY_ROOT, Rejected, SlotMotion, board_serial

DEFAULT_GRIPPER_CAMERA = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0"


def arm_env(robot_id: str, name: str) -> str:
    """CONTROL_STATION_ARM01_<name> for arm-01; empty when unset."""
    return os.environ.get("CONTROL_STATION_" + robot_id.replace("-", "").upper() + "_" + name, "").strip()


def arm_board(robot_id: str) -> str | None:
    """This arm's OpenRB-150 board serial from config/arms.json, if recorded."""
    try:
        document = json.loads((REPOSITORY_ROOT / "config" / "arms.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return next((arm.get("board_serial") for arm in document.get("arms", []) if arm.get("arm_id") == robot_id), None)


def add_policy_arguments(parser: argparse.ArgumentParser, robot_id: str) -> None:
    """The PICK_WITH_POLICY settings; camera defaults come from this arm's CONTROL_STATION_ARMxx_* settings."""
    policy = parser.add_argument_group("PICK_WITH_POLICY (LeRobot ACT)")
    policy.add_argument("--policies", type=Path, default=DEFAULT_POLICIES, help="object -> trained policy file")
    policy.add_argument("--policy-server", default="127.0.0.1:8080", help="lerobot.async_inference.policy_server address")
    policy.add_argument("--overhead-camera",
                        default=arm_env(robot_id, "OVERHEAD_CAMERA") or os.environ.get("CONTROL_STATION_ACT_OVERHEAD_CAMERA", ""),
                        help="camera seeing the shelf from above, as in training "
                             "(env CONTROL_STATION_ARM01_OVERHEAD_CAMERA for arm-01, else CONTROL_STATION_ACT_OVERHEAD_CAMERA)")
    policy.add_argument("--gripper-camera",
                        default=arm_env(robot_id, "GRIPPER_CAMERA") or os.environ.get("CONTROL_STATION_JOINT_CAMERA")
                        or DEFAULT_GRIPPER_CAMERA,
                        help="camera on the gripper (env CONTROL_STATION_ARM01_GRIPPER_CAMERA, else CONTROL_STATION_JOINT_CAMERA)")
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
    policy.add_argument("--simulated-run-s", type=float, default=3.0,
                        help="how long a simulated policy run (--fake-follower, --dry-run) pretends to take")


def finish_policy_arguments(args: argparse.Namespace) -> None:
    if args.policy_any_start and args.policy_max_step_ticks is None:
        args.policy_max_step_ticks = 40  # about 1,200 ticks/s at 30 Hz; the first actions from an unseen pose may jump


@dataclass
class PickEnd:
    """How one pick ended, in the HTTP worker's event words."""

    event: str                     # RESULT_UNKNOWN, FAILED, CANCELED
    reason_code: str | None = None
    message: str | None = None
    pose_reached: bool | None = None
    phase: str | None = None
    details: dict[str, Any] | None = None
    run_s: float = 0.0


class PolicyPick:
    def __init__(self, motion: SlotMotion, settings: argparse.Namespace) -> None:
        self.motion = motion
        self.settings = settings
        self.policy_version, self.policies = load_policies(settings.policies)
        self.camera_devices = {"OVERHEAD": settings.overhead_camera, "GRIPPER": settings.gripper_camera}

    @property
    def simulated(self) -> bool:
        return self.motion.simulated

    def check(self, policy_ref: dict, valid_until: datetime, *, allow_simulation: bool = False,
              ) -> tuple[str | None, dict | None]:
        """(reason to refuse, policy). Reads the arm but writes nothing.

        Without `allow_simulation` a simulated arm is refused (POLICY_NEEDS_REAL_ARM), as the HTTP worker always did."""
        settings = self.settings
        if self.simulated and not allow_simulation:
            return "POLICY_NEEDS_REAL_ARM", None
        if policy_ref.get("registry_version") != self.policy_version:
            return "POLICY_REGISTRY_MISMATCH", None
        policy = self.policies.get(policy_ref.get("policy_id"))
        if policy is None or policy["model"] != policy_ref.get("model"):
            return "POLICY_UNKNOWN", None
        # A policy recorded on one arm runs only on that arm: this worker's arm id and the board on its port.
        if policy.get("arm_id") not in (None, settings.robot_id):
            return "POLICY_WRONG_ARM", None
        board = arm_board(settings.robot_id)
        if policy.get("arm_id") and board and not settings.fake_follower and board not in board_serial(settings.port):
            print(f"  {settings.robot_id}의 보드({board})가 아닌 포트입니다: {settings.port}", flush=True)
            return "POLICY_WRONG_ARM", None
        if "home" not in self.motion.poses["poses"]:
            return "POSE_NOT_RECORDED", None
        if valid_until <= datetime.now(timezone.utc):
            return "EXPIRED", None
        if self.simulated:
            return None, policy  # no policy runs: cameras, policy server and the start pose do not matter
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

    def prepare(self) -> None:
        """Run the policy at the servo speed it was recorded with (power-on default 0 = no profile limit); the
        slot/home moves leave a slow profile in RAM. Nothing to do on a simulated arm."""
        if self.simulated:
            return
        with self.motion.follower():
            self.motion.write("Profile_Velocity", {name: 0 for name in ARM_JOINTS})
            self.motion.write("Profile_Acceleration", {name: 0 for name in ARM_JOINTS})

    def run(self, policy: dict, *, cancel_requested: Callable[[], bool],
            started: Callable[[], None] | None = None) -> tuple[PolicyRunResult, dict]:
        """Run the policy until the arm is back at rest, the time limit or a cancel. Exceptions are the caller's:
        the arm then keeps torque where the policy left it and a person has to look."""
        settings = self.settings
        if self.simulated:
            return self._simulate(policy, cancel_requested, started)
        home = {name: int(self.motion.poses["poses"]["home"]["ticks"][name]) for name in ARM_JOINTS}
        watcher = RestWatcher(home, settings.policy_depart_ticks, settings.policy_rest_ticks, settings.policy_settle_s)
        cameras = policy_cameras(policy, self.camera_devices)
        print(f"  정책 실행: {policy['model']} · front={cameras['front']} · wrist={cameras['wrist']}", flush=True)
        result = run_policy(
            policy, port=settings.port, cameras=cameras, server_address=settings.policy_server, watcher=watcher,
            max_run_s=policy["max_run_s"], cancel_requested=cancel_requested, started=started,
            max_step_ticks=settings.policy_max_step_ticks, policy_device=settings.policy_device,
        )
        return result, self._report(policy, result)

    def _simulate(self, policy: dict, cancel_requested: Callable[[], bool],
                  started: Callable[[], None] | None) -> tuple[PolicyRunResult, dict]:
        print(f"  모의 정책 실행 {self.settings.simulated_run_s:g}초: {policy['model']} (팔·카메라·정책 서버 안 씀)", flush=True)
        begin = time.monotonic()
        if started:
            started()
        reason = "RETURNED_TO_REST"
        while time.monotonic() - begin < self.settings.simulated_run_s:
            if cancel_requested():
                reason = "CANCELED"
                break
            time.sleep(0.05)
        result = PolicyRunResult(reason, time.monotonic() - begin, 0, message="simulated")
        return result, {**self._report(policy, result), "simulated": True}

    @staticmethod
    def _report(policy: dict, result: PolicyRunResult) -> dict:
        print(f"  정책 종료: {result.stop_reason}, {result.run_s:.1f}s, 행동 {result.actions_sent}개, "
              f"home에서 가장 멀리 {result.farthest_from_home_ticks} ticks", flush=True)
        return {"policy_id": policy["policy_id"], "model": policy["model"], **asdict(result)}

    def end(self, result: PolicyRunResult, policy_details: dict,
            returning_home: Callable[[], None] | None = None) -> PickEnd:
        """What the policy run ended as. Back at rest, the arm is aligned to home first (`returning_home` is told)."""
        run_s = result.run_s
        if result.stop_reason == "RETURNED_TO_REST":
            if returning_home:
                returning_home()
            end = self.finish_at_home(policy_details)
        elif result.stop_reason == "CANCELED":
            end = PickEnd("CANCELED", phase="APPROACH_TARGET", details={"policy_run": policy_details})
        elif result.stop_reason == "NEVER_DEPARTED":
            end = PickEnd("FAILED", "POLICY_DID_NOT_MOVE", details={"policy_run": policy_details})
        elif result.stop_reason == "TIMEOUT":
            # Not back at rest: it may hold the object or be near the shelf, so no automatic move home.
            end = PickEnd("FAILED", "POLICY_TIMEOUT_NOT_AT_REST",
                          "정책을 멈췄고 팔은 그 자리에서 토크를 유지합니다. 현장 확인 뒤 원상복귀하세요.",
                          details={"policy_run": policy_details})
        else:
            end = PickEnd("RESULT_UNKNOWN", "POLICY_RUN_ERROR", result.message[:300] or None,
                          details={"policy_run": policy_details})
        end.run_s = run_s
        return end

    def finish_at_home(self, policy_details: dict) -> PickEnd:
        """The policy came back to rest: align to the recorded home pose. The command was accepted in time, so the
        follow-up move is not tied to valid_until."""
        time.sleep(0.5)  # let the servos finish the last goal the policy wrote
        try:
            with self.motion.follower():
                goal = self.motion.plan("home", self.motion.slot_map_version,
                                        datetime.now(timezone.utc) + timedelta(seconds=60))
                now = self.motion.present()
                away = max(abs(goal[name] - now[name]) for name in ARM_JOINTS)
                if away > 2 * self.settings.policy_rest_ticks:
                    return PickEnd("FAILED", "REST_NOT_CONFIRMED",
                                   f"정책은 쉬는 자세로 돌아왔다고 봤지만 실제 관절이 home에서 {away} ticks 떨어져 있어 움직이지 않았습니다.",
                                   details={"policy_run": policy_details, "present_ticks": now})
                outcome = self.motion.move(goal, lambda: False)
        except Rejected as rejected:
            return PickEnd("FAILED", rejected.reason_code, details={"policy_run": policy_details})
        except (ConnectionError, OSError, RuntimeError) as error:
            print(f"  정책 실행 오류: home alignment: {error}", flush=True)
            return PickEnd("RESULT_UNKNOWN", "POLICY_RUN_ERROR", f"home alignment: {error}"[:300],
                           details={"policy_run": policy_details})
        home_details = {"goal_ticks": goal, "final_ticks": outcome.final_ticks, "max_error_ticks": outcome.max_error_ticks}
        details = {"policy_run": policy_details, "home_return": home_details}
        if outcome.outcome == "SUCCEEDED":
            # The arm is home; grasp and placement were not observed, so success cannot be claimed.
            return PickEnd("RESULT_UNKNOWN", "PLACE_NOT_VERIFIED", "정책 동작 뒤 home 복귀 완료. 집기·적재는 확인하지 않았습니다.",
                           pose_reached=True, details=details)
        return PickEnd("FAILED" if outcome.outcome == "FAILED" else "RESULT_UNKNOWN",
                       outcome.reason_code or "HOME_RETURN_FAILED", outcome.message or None, details=details)
