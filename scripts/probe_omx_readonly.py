#!/usr/bin/env python3
"""Read OMX follower motor state without changing configuration or sending motion goals."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from typing import Any

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus


_MOTOR_SPECS = {
    "shoulder_pan": (11, "xl430-w250", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": (12, "xl430-w250", MotorNormMode.RANGE_M100_100),
    "elbow_flex": (13, "xl430-w250", MotorNormMode.RANGE_M100_100),
    "wrist_flex": (14, "xl330-m288", MotorNormMode.RANGE_M100_100),
    "wrist_roll": (15, "xl330-m288", MotorNormMode.RANGE_M100_100),
    "gripper": (16, "xl330-m288", MotorNormMode.RANGE_0_100),
}
_READ_REGISTERS = (
    "Present_Position",
    "Torque_Enable",
    "Hardware_Error_Status",
    "Operating_Mode",
    "Min_Position_Limit",
    "Max_Position_Limit",
    "Present_Input_Voltage",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/omx_follower", help="OMX follower serial port")
    args = parser.parse_args()

    motors = {
        name: Motor(motor_id, model, norm_mode)
        for name, (motor_id, model, norm_mode) in _MOTOR_SPECS.items()
    }
    bus = DynamixelMotorsBus(port=args.port, motors=motors)
    report: dict[str, Any] = {
        "schema_version": 1,
        "observed_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "port": args.port,
        "status": "OBSERVED_ONLY",
        "readiness_state": "UNKNOWN",
        "write_or_motion_command_sent": False,
        "registers": {},
        "read_errors": {},
    }
    try:
        bus.connect(handshake=True)
        for register in _READ_REGISTERS:
            try:
                report["registers"][register] = bus.sync_read(register, normalize=False)
            except Exception as exc:
                report["read_errors"][register] = f"{type(exc).__name__}: {exc}"
    except Exception as exc:
        report["status"] = "PROBE_FAILED"
        report["connect_error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if bus.is_connected:
            try:
                bus.disconnect(disable_torque=False)
                report["port_closed_without_torque_write"] = True
            except Exception as exc:
                report["port_closed_without_torque_write"] = False
                report["disconnect_error"] = f"{type(exc).__name__}: {exc}"

    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(
        "레지스터 관측만 수행했습니다. 이 결과는 READY나 작업 안전성을 증명하지 않으며, "
        "0/4095 기계 범위는 선반 작업 영역이 아닙니다.",
        file=sys.stderr,
    )
    return 0 if report["status"] == "OBSERVED_ONLY" and not report["read_errors"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
