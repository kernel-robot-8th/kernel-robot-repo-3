#!/usr/bin/env python3
"""OMX 팔로워의 현재 관절 위치를 자세 이름으로 기록한다. 읽기만 하고 모터에 절대 쓰지 않는다.

팔로워를 손으로(토크 꺼짐) 또는 리더로 자세에 놓은 뒤 실행한다:

    sg dialout -c "~/lerobot-venv/bin/python scripts/record_omx_pose.py home"
    sg dialout -c "~/lerobot-venv/bin/python scripts/record_omx_pose.py slot_a_2_1"

위치는 팔 관절 다섯 개와 그리퍼의 원시 Dynamixel 틱(1회전 4096, 확장 위치 모드)이다. 파일에는 팔로워 보드
일련번호와 칸 정의 버전도 함께 남겨, 워커가 다른 팔에서 기록했거나 다른 칸 정의용으로 기록한 자세를 거부할 수 있게
한다. 자세 이름은 `home`이거나 사용 중인 칸 ID여야 한다(코드는 place_*, pick_* 이름도 받는다). 이미 있는 이름을
다시 기록하려면 --replace가 필요하다.
결과는 기본 config/omx-slot-poses.json에 임시 파일을 쓴 뒤 바꿔 넣는 방식으로 저장한다.
다른 모듈(omx_slot_motion.py, omx_lerobot_worker.py)이 open_follower()와 check_calibration()을 가져다 쓴다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus

from omx_slot_motion import DEFAULT_PORT, board_serial


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
# 팔로워 관절 이름 → (모터 ID, 모델). xl430은 어깨·팔꿈치, xl330은 손목·그리퍼다.
FOLLOWER_MOTORS = {
    "shoulder_pan": (11, "xl430-w250"),
    "shoulder_lift": (12, "xl430-w250"),
    "elbow_flex": (13, "xl430-w250"),
    "wrist_flex": (14, "xl330-m288"),
    "wrist_roll": (15, "xl330-m288"),
    "gripper": (16, "xl330-m288"),
}
# 틱 값이 같은 뜻이 되려면 팔로워가 이미 갖고 있어야 하는 레지스터 값(LeRobot OMX 공장 보정).
EXPECTED_CALIBRATION = {"Homing_Offset": 0, "Drive_Mode": 0}


def open_follower(port: str) -> DynamixelMotorsBus:
    """팔로워 포트를 LeRobot Dynamixel 버스로 연다. 연결만 하고 모터 설정은 쓰지 않는다."""
    bus = DynamixelMotorsBus(
        port=port,
        motors={name: Motor(motor_id, model, MotorNormMode.RANGE_M100_100) for name, (motor_id, model) in FOLLOWER_MOTORS.items()},
    )
    bus.connect(handshake=True)  # 포트를 열고 모든 모터에 ping한다. 아무것도 쓰지 않는다
    return bus


def check_calibration(bus: DynamixelMotorsBus) -> None:
    """Homing_Offset·Drive_Mode가 공장값(0)인지 읽어 확인하고, 다르면 ValueError를 던진다.

    보정이 다르면 같은 틱 값이 다른 관절 각도를 뜻하므로, 기록한 자세를 재생하면 엉뚱한 곳으로 간다.
    """
    for register, expected in EXPECTED_CALIBRATION.items():
        values = bus.sync_read(register, normalize=False, num_retry=3)
        wrong = {name: value for name, value in values.items() if value != expected}
        if wrong:
            raise ValueError(f"{register} is not the factory value {expected}: {wrong}")


def capture_held_pose(bus: DynamixelMotorsBus, seconds: float) -> dict[str, int] | None:
    """팔이 처음 있던 곳에서 떨어진 채, 어느 관절도 6틱 넘게 움직이지 않은 가장 긴 구간(최소 1초)의 가운데 자세를 고른다.

    토크가 꺼진 팔은 손을 놓으면 처지므로, 손으로 붙잡고 있던 동안의 자세를 골라내기 위한 것이다.
    0.2초 간격으로 읽으므로 6개 간격(j - i >= 5 다음 표본까지) ≈ 1초이고, 시작 자세에서 100틱(약 9°)
    넘게 떨어져야 "옮긴 자세"로 본다. 그런 구간이 없으면 None이다.
    """
    import time

    samples = []
    end = time.monotonic() + seconds
    print(f"{seconds:g}초 동안 읽습니다. 팔을 자세에 놓고 2초 이상 붙잡고 계세요.", flush=True)
    while time.monotonic() < end:
        samples.append(bus.sync_read("Present_Position", normalize=False, num_retry=3))
        time.sleep(0.2)
    start = samples[0]
    best: tuple[int, int] | None = None
    i = 0
    while i < len(samples):
        j = i
        while j + 1 < len(samples) and all(abs(samples[j + 1][k] - samples[i][k]) <= 6 for k in samples[i]):
            j += 1
        away = max(abs(samples[i][k] - start[k]) for k in start if k != "gripper")
        if j - i >= 5 and away > 100 and (best is None or j - i > best[1] - best[0]):
            best = (i, j)
        i = j + 1
    if best is None:
        return None
    print(f"{best[0] * 0.2:.1f}~{best[1] * 0.2:.1f}초 사이에 멈춰 있던 자세를 씁니다.", flush=True)
    return samples[(best[0] + best[1]) // 2]


def main() -> int:
    """이름 검사 → 자세 파일의 보드·칸 정의 버전 대조 → 모터 읽기 → 원자적 저장."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help="'home' or an active slot ID such as slot_a_2_1")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--slots", type=Path, default=REPOSITORY_ROOT / "config" / "shelf-slots.json")
    parser.add_argument("--output", type=Path, default=REPOSITORY_ROOT / "config" / "omx-slot-poses.json")
    parser.add_argument("--replace", action="store_true", help="overwrite an existing pose with this name")
    parser.add_argument(
        "--capture-s", type=float, default=0.0,
        help="read for this many seconds and keep the longest still stretch away from the starting pose "
             "(for a torque-off arm held by hand, which falls back when released)",
    )
    args = parser.parse_args()

    slot_raw = args.slots.read_bytes()
    active = {slot["slot_id"] for slot in json.loads(slot_raw)["slots"] if slot["active"]}
    # "place_*" 자세는 팔이 물체를 내려놓는 곳이다(예: 카트 위의 place_pinky).
    # "pick_*" 자세는 기록해 둔 집기 자세다(예: pick_apple_above, pick_apple_grasp).
    if args.name != "home" and not args.name.startswith(("place_", "pick_")) and args.name not in active:
        print(f"error: name must be 'home', place_<where>, pick_<what> or one of {sorted(active)}", file=sys.stderr)
        return 1
    slot_map_sha = hashlib.sha256(slot_raw).hexdigest()
    document = json.loads(args.output.read_text(encoding="utf-8")) if args.output.exists() else {
        "schema_version": 1,
        "robot": "omx_follower",
        "follower_board": board_serial(args.port),
        "motor_ids": {name: motor_id for name, (motor_id, _) in FOLLOWER_MOTORS.items()},
        "position_unit": "dynamixel_ticks_extended_position",
        "calibration": "lerobot_omx_factory_default (Homing_Offset 0, Drive_Mode 0)",
        "slot_map_sha256": slot_map_sha,
        "poses": {},
    }
    # 기존 파일이 다른 보드나 다른 칸 정의에서 기록됐으면 섞지 않고 멈춘다.
    if document.get("follower_board") != board_serial(args.port):
        print(f"error: {args.output} was recorded on board {document.get('follower_board')}", file=sys.stderr)
        return 1
    if document.get("slot_map_sha256") != slot_map_sha:
        print(f"error: {args.output} belongs to a different slot map version; record into a new file", file=sys.stderr)
        return 1
    if args.name in document["poses"] and not args.replace:
        print(f"error: pose {args.name!r} exists; pass --replace to record it again", file=sys.stderr)
        return 1

    bus = open_follower(args.port)
    try:
        check_calibration(bus)
        ticks = bus.sync_read("Present_Position", normalize=False, num_retry=3)
        torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=3)
        if args.capture_s > 0:
            ticks = capture_held_pose(bus, args.capture_s)
            if ticks is None:
                print("error: no pose was held still away from the starting pose; nothing recorded", file=sys.stderr)
                return 1
    finally:
        bus.disconnect(disable_torque=False)

    document["poses"][args.name] = {
        "recorded_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "ticks": {name: int(value) for name, value in ticks.items()},
        "torque_enabled_while_recording": {name: bool(value) for name, value in torque.items()},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # 같은 폴더에 임시 파일로 다 쓴 뒤 replace로 바꿔 넣어, 중간에 끊겨도 자세 파일이 반쯤 쓰인 채 남지 않게 한다.
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=args.output.parent, delete=False, suffix=".tmp") as handle:
        json.dump(document, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    Path(handle.name).replace(args.output)
    print(f"{args.name}: {document['poses'][args.name]['ticks']} -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
