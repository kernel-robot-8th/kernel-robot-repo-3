#!/usr/bin/env python3
"""Record the OMX follower's current joint positions under a pose name. Reads only; never writes.

Put the follower in place by hand (torque off) or with the leader, then run:

    sg dialout -c "~/lerobot-venv/bin/python scripts/record_omx_pose.py home"
    sg dialout -c "~/lerobot-venv/bin/python scripts/record_omx_pose.py slot_a_2_1"

Positions are raw Dynamixel ticks (4096 per turn, extended position mode) of the five arm joints and the
gripper. The file also keeps the follower board serial and the slot map version, so the worker can refuse
poses recorded on a different arm or for a different slot map. A pose name must be `home` or an active
slot ID; recording an existing name needs --replace.
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
FOLLOWER_MOTORS = {
    "shoulder_pan": (11, "xl430-w250"),
    "shoulder_lift": (12, "xl430-w250"),
    "elbow_flex": (13, "xl430-w250"),
    "wrist_flex": (14, "xl330-m288"),
    "wrist_roll": (15, "xl330-m288"),
    "gripper": (16, "xl330-m288"),
}
# Registers the follower must already hold (LeRobot OMX factory calibration) for ticks to mean the same thing.
EXPECTED_CALIBRATION = {"Homing_Offset": 0, "Drive_Mode": 0}


def open_follower(port: str) -> DynamixelMotorsBus:
    bus = DynamixelMotorsBus(
        port=port,
        motors={name: Motor(motor_id, model, MotorNormMode.RANGE_M100_100) for name, (motor_id, model) in FOLLOWER_MOTORS.items()},
    )
    bus.connect(handshake=True)  # opens the port and pings every motor; writes nothing
    return bus


def check_calibration(bus: DynamixelMotorsBus) -> None:
    for register, expected in EXPECTED_CALIBRATION.items():
        values = bus.sync_read(register, normalize=False, num_retry=3)
        wrong = {name: value for name, value in values.items() if value != expected}
        if wrong:
            raise ValueError(f"{register} is not the factory value {expected}: {wrong}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help="'home' or an active slot ID such as slot_a_2_1")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--slots", type=Path, default=REPOSITORY_ROOT / "config" / "shelf-slots.json")
    parser.add_argument("--output", type=Path, default=REPOSITORY_ROOT / "config" / "omx-slot-poses.json")
    parser.add_argument("--replace", action="store_true", help="overwrite an existing pose with this name")
    args = parser.parse_args()

    slot_raw = args.slots.read_bytes()
    active = {slot["slot_id"] for slot in json.loads(slot_raw)["slots"] if slot["active"]}
    if args.name != "home" and args.name not in active:
        print(f"error: name must be 'home' or one of {sorted(active)}", file=sys.stderr)
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
    finally:
        bus.disconnect(disable_torque=False)

    document["poses"][args.name] = {
        "recorded_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "ticks": {name: int(value) for name, value in ticks.items()},
        "torque_enabled_while_recording": {name: bool(value) for name, value in torque.items()},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=args.output.parent, delete=False, suffix=".tmp") as handle:
        json.dump(document, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    Path(handle.name).replace(args.output)
    print(f"{args.name}: {document['poses'][args.name]['ticks']} -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
