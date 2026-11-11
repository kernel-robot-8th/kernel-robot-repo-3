"""Bring the OMX follower to the joint pose taught for a shelf slot. Shared by the HTTP and ROS 2 workers.

Transport-free: the workers decide how commands arrive and how results are reported. LeRobot is imported only
when a real follower is opened, so `--fake-follower` and the tests run without it.

Safety behaviour (same as the HTTP worker had):
- rejects before moving: unknown slot map, pose not recorded, expired command, too-large change
- before enabling torque, writes Goal_Position = Present_Position on the five arm joints (no jump)
- never commands the gripper
- the servo's trapezoidal profile (Profile_Velocity/Profile_Acceleration) limits speed
- on cancel, timeout, or error, writes Goal_Position = Present_Position (hold where it is)
- keeps torque on when the port is released so the arm does not drop
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
# Gripper ticks from the team's recorded demonstrations (JJinJJin2/pick_and_place_v2, RANGE_0_100 on 0..4095):
# larger is more open; the operator opened to about 58-60 (2,400-2,450 ticks) and commanded 46.8 (1,916 ticks) to
# squeeze; 49 (about 2,010 ticks) is the rest/closed reading.
GRIPPER_OPEN = 2450
GRIPPER_CLOSED = 2010
GRIPPER_SQUEEZE = 1916
TICKS_PER_TURN = 4096
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PORT = "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F58AE68650323952582E3120FF05260E-if00"


def board_serial(port: str) -> str:
    return Path(port).name.removeprefix("usb-").removesuffix("-if00") if "/by-id/" in port else port


def add_motion_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--slots", type=Path, default=REPOSITORY_ROOT / "config" / "shelf-slots.json")
    parser.add_argument("--poses", type=Path, default=REPOSITORY_ROOT / "config" / "omx-slot-poses.json")
    # 2026-11-03: 15/10 started and stopped too abruptly on the real arm; 10/2 ramps over about 0.3 s.
    parser.add_argument("--profile-velocity", type=int, default=10, help="servo profile velocity (x0.229 rpm); 10 = about 14 deg/s")
    parser.add_argument(
        "--profile-acceleration", type=int, default=2,
        help="servo profile acceleration (x214.6 rev/min^2); 2 = about 43 deg/s^2, lower is gentler",
    )
    parser.add_argument("--max-delta", type=int, default=1200, help="largest allowed joint change in ticks (4096 = 1 turn)")
    parser.add_argument("--tolerance", type=int, default=15, help="ticks within the goal that count as reached")
    parser.add_argument("--move-timeout", type=float, default=30.0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="read the real motors, but write nothing to them")
    mode.add_argument("--fake-follower", action="store_true",
                      help="no arm: simulated motors in this process; every result is marked simulated")


class Rejected(Exception):
    """Checks failed before anything moved."""

    def __init__(self, reason_code: str, message: str = "") -> None:
        super().__init__(message or reason_code)
        self.reason_code = reason_code
        self.message = message


@dataclass
class Outcome:
    outcome: str  # SUCCEEDED, FAILED, RESULT_UNKNOWN, CANCELED
    simulated: bool
    reason_code: str = ""
    message: str = ""
    pose_reached: bool = False
    final_ticks: dict[str, int] = field(default_factory=dict)
    max_error_ticks: int = -1


class FakeFollowerBus:
    """Stands in for the Dynamixel bus when no arm is connected. Present positions move toward the goal at the
    profile velocity; nothing leaves this process."""

    def __init__(self, start: dict[str, int]) -> None:
        self.positions = {name: float(value) for name, value in start.items()}
        self.goal = dict(self.positions)
        # per joint, like the servos' Profile_Velocity; the gripper is never given a profile here (fast, like power-on 0)
        self.velocity = {name: 100 if name == "gripper" else 10 for name in self.positions}
        self.last = time.monotonic()
        self.is_connected = True

    def _advance(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last
        self.last = now
        for name, goal in self.goal.items():
            step = self.velocity[name] * 0.229 / 60 * TICKS_PER_TURN * elapsed
            here = self.positions[name]
            self.positions[name] = goal if abs(goal - here) <= step else here + math.copysign(step, goal - here)

    def sync_read(self, register: str, motors: list[str] | None = None, **_: Any) -> dict[str, int]:
        self._advance()
        names = motors or list(self.positions)
        if register == "Present_Position":
            return {name: round(self.positions[name]) for name in names}
        if register == "Moving":
            return {name: int(abs(self.goal[name] - self.positions[name]) > 0.5) for name in names}
        if register in ("Homing_Offset", "Drive_Mode"):
            return {name: 0 for name in names}
        raise KeyError(register)

    def sync_write(self, register: str, values: dict[str, int], **_: Any) -> None:
        self._advance()
        if register == "Goal_Position":
            self.goal.update({name: float(value) for name, value in values.items()})
        elif register == "Profile_Velocity":
            self.velocity.update(values)

    def disconnect(self, disable_torque: bool = False) -> None:
        pass  # keeps its state, like the real arm keeps its pose between commands


class SlotMotion:
    def __init__(self, settings: argparse.Namespace) -> None:
        self.settings = settings
        raw = settings.slots.read_bytes()
        self.slot_map_version = hashlib.sha256(raw).hexdigest()
        self.poses = json.loads(settings.poses.read_text(encoding="utf-8"))
        if self.poses.get("slot_map_sha256") != self.slot_map_version:
            raise SystemExit(f"{settings.poses} belongs to a different slot map than {settings.slots}")
        self.simulated = settings.dry_run or settings.fake_follower
        self.fake_bus: FakeFollowerBus | None = None
        if settings.fake_follower:
            # Start away from the recorded poses so a fake move takes a few seconds.
            first = next(iter(self.poses["poses"].values()))["ticks"]
            start = self.poses["poses"].get("home", {}).get("ticks") or {name: first[name] - 200 for name in ARM_JOINTS}
            self.fake_bus = FakeFollowerBus(
                {**{name: int(start[name]) for name in ARM_JOINTS}, "gripper": int(start.get("gripper", GRIPPER_CLOSED))}
            )
        elif self.poses.get("follower_board") != board_serial(settings.port):
            raise SystemExit(f"{settings.poses} was recorded on board {self.poses.get('follower_board')}, not {settings.port}")
        # The port is opened only while handling a command, so the LeRobot policy that grasps next can open the
        # same follower port. Torque stays on when the port is released.
        self.bus: Any = None
        with self.follower():
            if not settings.fake_follower:
                from record_omx_pose import check_calibration

                check_calibration(self.bus)

    def describe(self) -> str:
        mode = " · 가짜 팔로워 (장비 없음, 모의)" if self.settings.fake_follower else " · DRY RUN (모터에 쓰지 않음)" if self.settings.dry_run else ""
        where = "없음" if self.settings.fake_follower else self.settings.port
        return f"팔로워: {where} · 기록된 자세 {sorted(self.poses['poses'])}{mode}"

    # ---- motor helpers ----
    @contextmanager
    def follower(self):
        if self.fake_bus is not None:
            self.bus = self.fake_bus
        else:
            from record_omx_pose import open_follower

            self.bus = open_follower(self.settings.port)
        try:
            yield self.bus
        finally:
            if self.bus.is_connected:
                self.bus.disconnect(disable_torque=False)  # keep holding; the arm must not drop
            self.bus = None

    def present(self) -> dict[str, int]:
        values = self.bus.sync_read("Present_Position", list(ARM_JOINTS), normalize=False, num_retry=3)
        return {name: int(value) for name, value in values.items()}

    def write(self, register: str, values: dict[str, int]) -> None:
        if self.settings.dry_run:
            print(f"  (dry-run) {register} <- {values}", flush=True)
            return
        self.bus.sync_write(register, values, normalize=False, num_retry=3)

    def gripper_to(self, goal: int, timeout_s: float = 3.0) -> int:
        """Open or close the gripper and return the tick where it stopped. The follower's gripper runs in
        current-based position mode, so closing on an object stops at the object instead of crushing it.
        Torque is turned on with the goal first set to the present position, so enabling it cannot jump."""
        def read() -> int:
            return int(self.bus.sync_read("Present_Position", ["gripper"], normalize=False, num_retry=3)["gripper"])

        now = read()
        self.write("Goal_Position", {"gripper": now})
        self.write("Torque_Enable", {"gripper": 1})
        self.write("Goal_Position", {"gripper": int(goal)})
        if self.settings.dry_run:
            return now
        deadline, last, still = time.monotonic() + timeout_s, None, 0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            now = read()
            still = still + 1 if last is not None and abs(now - last) <= 2 else 0
            if still >= 3:  # stopped: at the goal, or blocked by the object
                break
            last = now
        return now

    def hold_here(self) -> None:
        """Stop where the arm is now; the goal becomes the present position."""
        try:
            self.write("Goal_Position", self.present())
        except Exception as error:  # report but keep going: the caller reports the outcome
            print(f"  정지 명령 실패: {error}", flush=True)

    # ---- the two steps a worker runs inside `follower()` ----
    def plan(self, slot_id: str | None, slot_map_version: str | None, valid_until: datetime) -> dict[str, int]:
        """Goal ticks for the slot's taught pose, or raise Rejected."""
        if slot_map_version != self.slot_map_version:
            raise Rejected("SLOT_MAP_MISMATCH")
        pose = self.poses["poses"].get(slot_id)
        if pose is None:
            raise Rejected("POSE_NOT_RECORDED", f"no joint pose recorded for {slot_id}")
        goal = {name: int(pose["ticks"][name]) for name in ARM_JOINTS}
        return self.check_goal(goal, valid_until)

    def check_goal(self, goal: dict[str, int], valid_until: datetime) -> dict[str, int]:
        """The checks every joint goal passes before anything moves: not expired, no joint changes too much."""
        if valid_until <= datetime.now(timezone.utc):
            raise Rejected("EXPIRED")
        now = self.present()
        deltas = {name: abs(goal[name] - now[name]) for name in ARM_JOINTS}
        print(f"  현재 {now}\n  목표 {goal}\n  변화량 {deltas}", flush=True)
        if max(deltas.values()) > self.settings.max_delta:
            raise Rejected("TARGET_TOO_FAR", f"largest joint change {max(deltas.values())} ticks")
        return goal

    def synced_profiles(self, start: dict[str, int], goal: dict[str, int]) -> tuple[dict[str, int], dict[str, int]]:
        """Per-joint profile velocity and acceleration scaled by each joint's travel, so all joints start and arrive
        together and the tool moves close to the straight joint-space line. With one shared profile the joints that
        travel less finish first and the tool can swing far off the line (10 cm measured for a taught pick)."""
        travel = {name: abs(goal[name] - start[name]) for name in ARM_JOINTS}
        longest = max(travel.values()) or 1
        velocity = {name: max(1, round(self.settings.profile_velocity * travel[name] / longest)) for name in ARM_JOINTS}
        acceleration = {name: max(1, round(self.settings.profile_acceleration * travel[name] / longest)) for name in ARM_JOINTS}
        return velocity, acceleration

    def move(self, goal: dict[str, int], cancel_requested: Callable[[], bool],
             progress: Callable[[dict[str, int], int], None] | None = None, *, sync: bool = False) -> Outcome:
        """Move to `goal` and report how it ended. `progress(present, max_error)` is first called just before the
        goal is written (nothing has moved yet), then about every 0.1 s while moving. `sync` scales each joint's
        profile so all joints arrive together (see synced_profiles)."""
        settings = self.settings
        now: dict[str, int] = {}
        try:
            start = self.present()
            self.write("Goal_Position", start)  # align before torque so enabling it cannot jump
            if sync:
                velocity, acceleration = self.synced_profiles(start, goal)
            else:
                velocity = {name: settings.profile_velocity for name in ARM_JOINTS}
                acceleration = {name: settings.profile_acceleration for name in ARM_JOINTS}
            self.write("Profile_Acceleration", acceleration)
            self.write("Profile_Velocity", velocity)
            self.write("Torque_Enable", {name: 1 for name in ARM_JOINTS})
            now = self.present()
            if progress:
                progress(now, max(abs(goal[name] - now[name]) for name in ARM_JOINTS))
            self.write("Goal_Position", goal)
            deadline = time.monotonic() + settings.move_timeout
            while True:
                now = self.present()
                errors = {name: abs(goal[name] - now[name]) for name in ARM_JOINTS}
                if progress:
                    progress(now, max(errors.values()))
                if settings.dry_run or max(errors.values()) <= settings.tolerance:
                    moving = {} if settings.dry_run else self.bus.sync_read("Moving", list(ARM_JOINTS), normalize=False)
                    if not any(moving.values()):
                        if settings.dry_run:
                            # Nothing moved, so success cannot be claimed.
                            return Outcome("RESULT_UNKNOWN", True, "DRY_RUN_NO_MOTION", final_ticks=now)
                        print(f"  도착: 최대 오차 {max(errors.values())} ticks", flush=True)
                        return Outcome("SUCCEEDED", self.simulated, pose_reached=True, final_ticks=now,
                                       max_error_ticks=max(errors.values()))
                if time.monotonic() > deadline:
                    self.hold_here()
                    return Outcome("FAILED", self.simulated, "MOVE_TIMEOUT", final_ticks=now, max_error_ticks=max(errors.values()))
                try:
                    canceled = cancel_requested()
                except Exception as error:  # the arm stops when the cancel flag cannot be read
                    self.hold_here()
                    return Outcome("RESULT_UNKNOWN", self.simulated, "CANCEL_CHECK_FAILED", str(error)[:300], final_ticks=now)
                if canceled:
                    self.hold_here()
                    stopped = self.present()
                    return Outcome("CANCELED", self.simulated, final_ticks=stopped,
                                   max_error_ticks=max(abs(goal[name] - stopped[name]) for name in ARM_JOINTS))
                time.sleep(0.1)
        except Exception as error:
            self.hold_here()
            return Outcome("RESULT_UNKNOWN", self.simulated, "MOTOR_COMM_ERROR", str(error)[:300], final_ticks=now)
