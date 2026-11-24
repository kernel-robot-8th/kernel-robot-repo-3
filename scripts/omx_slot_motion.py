"""OMX 팔로워를 선반 칸에 기록해 둔 관절 자세로 옮긴다. HTTP 워커와 ROS 2 워커가 함께 쓰는 모듈이다.

전송 방식과 무관하다: 명령이 어떻게 들어오고 결과를 어떻게 보고할지는 워커가 정한다
(omx_lerobot_worker.py = HTTP long-poll, omx_ros_worker.py = ROS 2 action, omx_policy_pick.py도 home 정렬에 쓴다).
LeRobot은 실물 팔로워를 열 때만 import하므로 `--fake-follower`와 테스트는 LeRobot 없이 돈다.
직접 실행하는 CLI는 없다. 실물 팔로워를 열면 Dynamixel 모터에 쓰므로 실제로 팔이 움직인다.

안전 동작 (예전 HTTP 워커와 같다):
- 움직이기 전에 거부: 모르는 칸 정의, 기록하지 않은 자세, 만료된 명령, 너무 큰 변화량
- 토크를 켜기 전에 팔 관절 다섯 개에 Goal_Position = Present_Position을 쓴다 (튀지 않게)
- 그리퍼에는 명령하지 않는다 (gripper_to()는 직접 부를 때만 쓰는 별도 기능)
- 서보의 사다리꼴 프로파일(Profile_Velocity/Profile_Acceleration)이 속도를 제한한다
- 취소, 제한 시간 초과, 오류가 나면 Goal_Position = Present_Position을 쓴다 (그 자리에 멈춤)
- 포트를 놓을 때 토크를 켜 둔 채로 둬서 팔이 떨어지지 않게 한다

단위: 관절값은 Dynamixel 위치 틱(1회전 = 4096틱, 1틱 ≈ 0.088°)이다.
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

# 팔 관절 다섯 개. 자세 재생·안전 검사는 이 다섯 개에만 적용하고 그리퍼는 따로 다룬다.
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
# 운영자 조그 한 번의 크기: 20틱 ≈ 1.76°. 한 번에 크게 움직이지 않도록 작게 잡았다.
MANUAL_JOG_STEP_TICKS = 20
# 그리퍼 틱 값은 팀이 녹화한 시연 데이터(JJinJJin2/pick_and_place_v2, 0..4095를 RANGE_0_100으로 표시)에서 가져왔다.
# 값이 클수록 더 벌어진다. 운영자는 약 58-60(2,400-2,450틱)까지 벌렸고, 쥘 때는 46.8(1,916틱)을 명령했다.
# 49(약 2,010틱)는 쉬는/닫힌 상태의 읽은 값이다.
GRIPPER_OPEN = 2450
GRIPPER_CLOSED = 2010
GRIPPER_SQUEEZE = 1916
TICKS_PER_TURN = 4096
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
# 기본 팔로워 보드(OpenRB-150)의 by-id 경로. ttyACM 번호는 꽂는 순서로 바뀌므로 by-id를 쓴다.
DEFAULT_PORT = "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F58AE68650323952582E3120FF05260E-if00"


def board_serial(port: str) -> str:
    """by-id 포트 경로에서 보드 일련번호 부분을 꺼낸다. 자세 파일이 이 보드에서 기록됐는지 대조하는 데 쓴다."""
    return Path(port).name.removeprefix("usb-").removesuffix("-if00") if "/by-id/" in port else port


def add_motion_arguments(parser: argparse.ArgumentParser) -> None:
    """두 워커가 공통으로 받는 이동 관련 CLI 옵션을 붙인다."""
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--slots", type=Path, default=REPOSITORY_ROOT / "config" / "shelf-slots.json")
    parser.add_argument("--poses", type=Path, default=REPOSITORY_ROOT / "config" / "omx-slot-poses.json")
    # 2026-11-03: 처음 쓴 15/10은 실물 팔에서 출발·정지가 너무 급했다. 10/2는 약 0.3초에 걸쳐 가속·감속한다.
    parser.add_argument("--profile-velocity", type=int, default=10, help="servo profile velocity (x0.229 rpm); 10 = about 14 deg/s")
    parser.add_argument(
        "--profile-acceleration", type=int, default=2,
        help="servo profile acceleration (x214.6 rev/min^2); 2 = about 43 deg/s^2, lower is gentler",
    )
    # 1200틱 ≈ 105°. 어느 관절이든 이보다 많이 움직여야 하면 TARGET_TOO_FAR로 거부한다.
    parser.add_argument("--max-delta", type=int, default=1200, help="largest allowed joint change in ticks (4096 = 1 turn)")
    # 15틱 ≈ 1.3°. 다섯 관절이 모두 이 안에 들어오고 Moving이 0이면 도착으로 본다.
    parser.add_argument("--tolerance", type=int, default=15, help="ticks within the goal that count as reached")
    parser.add_argument("--move-timeout", type=float, default=30.0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="read the real motors, but write nothing to them")
    mode.add_argument("--fake-follower", action="store_true",
                      help="no arm: simulated motors in this process; every result is marked simulated")


class Rejected(Exception):
    """아무것도 움직이기 전에 검사에서 걸렸다. reason_code가 관제에 보낼 거부 사유다."""

    def __init__(self, reason_code: str, message: str = "") -> None:
        super().__init__(message or reason_code)
        self.reason_code = reason_code
        self.message = message


@dataclass
class Outcome:
    """이동 한 번의 결과. pose_reached는 측정한 관절값이 허용 오차 안에 들어왔을 때만 True다."""

    outcome: str  # SUCCEEDED, FAILED, RESULT_UNKNOWN, CANCELED
    simulated: bool
    reason_code: str = ""
    message: str = ""
    pose_reached: bool = False
    final_ticks: dict[str, int] = field(default_factory=dict)
    max_error_ticks: int = -1


class FakeFollowerBus:
    """팔이 연결되지 않았을 때 Dynamixel 버스를 대신한다. 현재 위치가 프로파일 속도로 목표를 향해 움직이며,
    이 프로세스 밖으로는 아무것도 나가지 않는다."""

    def __init__(self, start: dict[str, int]) -> None:
        self.positions = {name: float(value) for name, value in start.items()}
        self.goal = dict(self.positions)
        # 관절별 값으로 서보의 Profile_Velocity와 같다. 그리퍼에는 여기서 프로파일을 주지 않는다(전원 직후 0처럼 빠름)
        self.velocity = {name: 100 if name == "gripper" else 10 for name in self.positions}
        self.last = time.monotonic()
        self.is_connected = True

    def _advance(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last
        self.last = now
        for name, goal in self.goal.items():
            # Profile_Velocity 단위는 0.229 rpm. rpm → 초당 회전 수 → 초당 틱으로 바꿔 경과 시간만큼 진행한다.
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
        if register in ("Homing_Offset", "Drive_Mode", "Hardware_Error_Status"):
            return {name: 0 for name in names}
        raise KeyError(register)

    def sync_write(self, register: str, values: dict[str, int], **_: Any) -> None:
        self._advance()
        if register == "Goal_Position":
            self.goal.update({name: float(value) for name, value in values.items()})
        elif register == "Profile_Velocity":
            self.velocity.update(values)

    def disconnect(self, disable_torque: bool = False) -> None:
        pass  # 실물 팔이 명령 사이에 자세를 유지하듯 상태를 그대로 둔다


class SlotMotion:
    """칸 정의·자세 파일을 읽고, 팔로워 포트를 열어 검사(plan)와 이동(move)을 수행한다.

    시작할 때 자세 파일이 이 칸 정의(SHA-256)와 이 보드에서 기록된 것인지, 팔의 보정값이 LeRobot OMX 공장값인지
    확인하고 하나라도 다르면 프로세스를 끝낸다(SystemExit). 다른 팔의 자세로 움직이는 일을 막기 위해서다.
    """

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
            # 가짜 이동이 몇 초는 걸리도록 기록된 자세에서 떨어진 곳(home, 없으면 첫 자세 -200틱)에서 시작한다.
            first = next(iter(self.poses["poses"].values()))["ticks"]
            start = self.poses["poses"].get("home", {}).get("ticks") or {name: first[name] - 200 for name in ARM_JOINTS}
            self.fake_bus = FakeFollowerBus(
                {**{name: int(start[name]) for name in ARM_JOINTS}, "gripper": int(start.get("gripper", GRIPPER_CLOSED))}
            )
        elif self.poses.get("follower_board") != board_serial(settings.port):
            raise SystemExit(f"{settings.poses} was recorded on board {self.poses.get('follower_board')}, not {settings.port}")
        # 포트는 명령을 처리하는 동안만 연다. 그래야 다음에 집기를 맡는 LeRobot 정책이 같은 팔로워 포트를 열 수 있다.
        # 포트를 놓아도 토크는 켜진 채로 남는다.
        self.bus: Any = None
        with self.follower():
            if not settings.fake_follower:
                from record_omx_pose import check_calibration

                check_calibration(self.bus)

    def describe(self) -> str:
        """시작할 때 출력하는 한 줄 요약(포트, 기록된 자세, 모의 여부)."""
        mode = " · 가짜 팔로워 (장비 없음, 모의)" if self.settings.fake_follower else " · DRY RUN (모터에 쓰지 않음)" if self.settings.dry_run else ""
        where = "없음" if self.settings.fake_follower else self.settings.port
        return f"팔로워: {where} · 기록된 자세 {sorted(self.poses['poses'])}{mode}"

    # ---- 모터 도우미 ----
    @contextmanager
    def follower(self):
        """팔로워 포트를 열어 self.bus로 두고, 끝나면 토크를 유지한 채 포트를 닫는다."""
        if self.fake_bus is not None:
            self.bus = self.fake_bus
        else:
            from record_omx_pose import open_follower

            self.bus = open_follower(self.settings.port)
        try:
            yield self.bus
        finally:
            if self.bus.is_connected:
                self.bus.disconnect(disable_torque=False)  # 계속 붙잡는다. 팔이 떨어지면 안 된다
            self.bus = None

    def present(self) -> dict[str, int]:
        """팔 관절 다섯 개의 현재 위치(틱)를 읽는다. 보정 변환 없이 원시값(normalize=False)이다."""
        values = self.bus.sync_read("Present_Position", list(ARM_JOINTS), normalize=False, num_retry=3)
        return {name: int(value) for name, value in values.items()}

    def write(self, register: str, values: dict[str, int]) -> None:
        """레지스터에 쓴다. --dry-run이면 쓰지 않고 쓸 내용만 출력한다. 모든 모터 쓰기는 이 함수를 거친다."""
        if self.settings.dry_run:
            print(f"  (dry-run) {register} <- {values}", flush=True)
            return
        self.bus.sync_write(register, values, normalize=False, num_retry=3)

    def gripper_to(self, goal: int, timeout_s: float = 3.0) -> int:
        """그리퍼를 열거나 닫고 멈춘 위치(틱)를 돌려준다. 팔로워 그리퍼는 전류 기반 위치 제어 모드라서,
        물체를 쥐면 물체를 짓누르지 않고 물체에서 멈춘다.
        목표를 먼저 현재 위치로 맞춘 뒤 토크를 켜므로 켜는 순간 튀지 않는다."""
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
            if still >= 3:  # 0.1초 간격 세 번 연속 2틱 이하 변화 = 멈춤: 목표에 닿았거나 물체에 막혔다
                break
            last = now
        return now

    def hold_here(self) -> None:
        """팔을 지금 자리에 세운다. 목표 위치가 현재 위치가 된다."""
        try:
            self.write("Goal_Position", self.present())
        except Exception as error:  # 출력만 하고 계속한다. 결과 보고는 호출한 쪽이 한다
            print(f"  정지 명령 실패: {error}", flush=True)

    # ---- 워커가 `follower()` 안에서 실행하는 두 단계(plan → move) ----
    def plan(self, slot_id: str | None, slot_map_version: str | None, valid_until: datetime) -> dict[str, int]:
        """칸에 기록한 자세의 목표 틱을 돌려주거나 Rejected를 던진다."""
        if slot_map_version != self.slot_map_version:
            raise Rejected("SLOT_MAP_MISMATCH")
        pose = self.poses["poses"].get(slot_id)
        if pose is None:
            raise Rejected("POSE_NOT_RECORDED", f"no joint pose recorded for {slot_id}")
        goal = {name: int(pose["ticks"][name]) for name in ARM_JOINTS}
        return self.check_goal(goal, valid_until)

    def check_goal(self, goal: dict[str, int], valid_until: datetime) -> dict[str, int]:
        """움직이기 전에 모든 관절 목표가 거치는 검사: 만료되지 않았고, 어느 관절도 너무 많이(--max-delta) 바뀌지 않는다."""
        if valid_until <= datetime.now(timezone.utc):
            raise Rejected("EXPIRED")
        now = self.present()
        deltas = {name: abs(goal[name] - now[name]) for name in ARM_JOINTS}
        print(f"  현재 {now}\n  목표 {goal}\n  변화량 {deltas}", flush=True)
        if max(deltas.values()) > self.settings.max_delta:
            raise Rejected("TARGET_TOO_FAR", f"largest joint change {max(deltas.values())} ticks")
        return goal

    def plan_jog(
        self, joint: str, direction: str, valid_until: datetime, step_ticks: int = MANUAL_JOG_STEP_TICKS
    ) -> dict[str, int]:
        """작은 상대 조그 한 번을 계획한다. 서보 위치 범위(0-4095) 안쪽에 여유를 둔 가드 범위(100-3995틱)를 지킨다.

        하드웨어 오류가 있는 모터가 하나라도 있으면 움직이지 않는다. 이 가드는 충돌 없는 작업 공간을 보장하지 않는다.
        """
        if not isinstance(joint, str) or joint not in ARM_JOINTS:
            raise Rejected("JOINT_NOT_SUPPORTED", joint)
        if not isinstance(direction, str) or direction not in {"POSITIVE", "NEGATIVE"}:
            raise Rejected("DIRECTION_NOT_SUPPORTED", direction)
        errors = self.bus.sync_read("Hardware_Error_Status", list(ARM_JOINTS), normalize=False, num_retry=3)
        if any(int(value) != 0 for value in errors.values()):
            raise Rejected("MOTOR_HARDWARE_ERROR", str(errors))
        start = self.present()
        sign = 1 if direction == "POSITIVE" else -1
        target = start[joint] + sign * step_ticks
        if not 100 <= target <= 3995:
            raise Rejected("JOINT_LIMIT", f"{joint} target {target} is outside the guarded 100–3995 tick range")
        goal = dict(start)
        goal[joint] = target
        return self.check_goal(goal, valid_until)

    def synced_profiles(self, start: dict[str, int], goal: dict[str, int]) -> tuple[dict[str, int], dict[str, int]]:
        """관절마다 이동량에 비례해 프로파일 속도·가속도를 나눠, 모든 관절이 함께 출발해 함께 도착하게 한다.
        그러면 도구 끝이 관절 공간의 직선 경로에 가깝게 움직인다. 모든 관절에 같은 프로파일을 쓰면 이동량이
        적은 관절이 먼저 끝나 도구 끝이 경로에서 크게 벗어날 수 있다(기록한 집기 동작에서 10 cm를 측정했다)."""
        travel = {name: abs(goal[name] - start[name]) for name in ARM_JOINTS}
        longest = max(travel.values()) or 1
        velocity = {name: max(1, round(self.settings.profile_velocity * travel[name] / longest)) for name in ARM_JOINTS}
        acceleration = {name: max(1, round(self.settings.profile_acceleration * travel[name] / longest)) for name in ARM_JOINTS}
        return velocity, acceleration

    def move(self, goal: dict[str, int], cancel_requested: Callable[[], bool],
             progress: Callable[[dict[str, int], int], None] | None = None, *, sync: bool = False,
             profile_velocity: int | None = None, profile_acceleration: int | None = None) -> Outcome:
        """`goal`로 움직이고 어떻게 끝났는지 돌려준다. `progress(present, max_error)`는 목표를 쓰기 직전에 처음 불리고
        (아직 아무것도 움직이지 않음), 이후 움직이는 동안 약 0.1초마다 불린다. `sync`는 관절별 프로파일을 조정해
        모든 관절이 함께 도착하게 한다(synced_profiles 참고).

        쓰는 순서: 목표=현재 → 가속도 → 속도 → 토크 켜기 → 목표. 결과는 SUCCEEDED(허용 오차 안 + Moving 0),
        FAILED/MOVE_TIMEOUT, CANCELED, RESULT_UNKNOWN(통신 오류, 취소 확인 실패, dry-run) 중 하나다.
        """
        settings = self.settings
        now: dict[str, int] = {}
        try:
            start = self.present()
            self.write("Goal_Position", start)  # 토크를 켜기 전에 목표를 현재 위치로 맞춰 켜는 순간 튀지 않게 한다
            if sync:
                velocity, acceleration = self.synced_profiles(start, goal)
            else:
                velocity = {name: profile_velocity or settings.profile_velocity for name in ARM_JOINTS}
                acceleration = {name: profile_acceleration or settings.profile_acceleration for name in ARM_JOINTS}
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
                            # 아무것도 움직이지 않았으므로 성공이라고 할 수 없다.
                            return Outcome("RESULT_UNKNOWN", True, "DRY_RUN_NO_MOTION", final_ticks=now)
                        print(f"  도착: 최대 오차 {max(errors.values())} ticks", flush=True)
                        return Outcome("SUCCEEDED", self.simulated, pose_reached=True, final_ticks=now,
                                       max_error_ticks=max(errors.values()))
                # --move-timeout(기본 30초)을 넘기면 그 자리에 세우고 실패로 끝낸다.
                if time.monotonic() > deadline:
                    self.hold_here()
                    return Outcome("FAILED", self.simulated, "MOVE_TIMEOUT", final_ticks=now, max_error_ticks=max(errors.values()))
                try:
                    canceled = cancel_requested()
                except Exception as error:  # 취소 표시를 읽을 수 없으면 팔을 세운다
                    self.hold_here()
                    return Outcome("RESULT_UNKNOWN", self.simulated, "CANCEL_CHECK_FAILED", str(error)[:300], final_ticks=now)
                if canceled:
                    self.hold_here()
                    stopped = self.present()
                    return Outcome("CANCELED", self.simulated, final_ticks=stopped,
                                   max_error_ticks=max(abs(goal[name] - stopped[name]) for name in ARM_JOINTS))
                time.sleep(0.1)
        except Exception as error:
            # 통신 오류면 팔이 실제로 어디까지 갔는지 모르므로 실패가 아니라 RESULT_UNKNOWN으로 보고한다.
            self.hold_here()
            return Outcome("RESULT_UNKNOWN", self.simulated, "MOTOR_COMM_ERROR", str(error)[:300], final_ticks=now)
