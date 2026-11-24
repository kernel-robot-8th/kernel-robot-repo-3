#!/usr/bin/env python3
"""OMX 팔로워 모터 상태를 설정 변경이나 이동 목표 전송 없이 읽기만 한다.

    ~/lerobot-venv/bin/python scripts/probe_omx_readonly.py --port /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_...-if00

LeRobot Dynamixel 버스로 포트를 열고(모터 ID 11-16에 ping), 현재 위치·토크·하드웨어 오류·동작 모드·위치 한계·
입력 전압 레지스터를 읽어 JSON으로 출력한다. 쓰기나 이동 명령은 보내지 않으며(write_or_motion_command_sent: false),
포트를 닫을 때도 토크를 건드리지 않는다. 팔을 움직이지 않는다.
결과는 관측일 뿐이고 팔이 준비됐다거나(READY) 안전하다는 증명이 아니다. 읽기 오류가 없을 때만 종료 코드 0이다.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from typing import Any

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus


# 관절 이름 → (모터 ID, 모델, LeRobot 정규화 모드). 팔로워는 ID 11-16, 리더는 1-6이다.
# 여기서는 normalize=False로 원시 틱만 읽으므로 정규화 모드는 버스 생성에만 쓰인다.
_MOTOR_SPECS = {
    "shoulder_pan": (11, "xl430-w250", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": (12, "xl430-w250", MotorNormMode.RANGE_M100_100),
    "elbow_flex": (13, "xl430-w250", MotorNormMode.RANGE_M100_100),
    "wrist_flex": (14, "xl330-m288", MotorNormMode.RANGE_M100_100),
    "wrist_roll": (15, "xl330-m288", MotorNormMode.RANGE_M100_100),
    "gripper": (16, "xl330-m288", MotorNormMode.RANGE_0_100),
}
# 읽기만 하는 레지스터 목록
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
    """포트를 열어 레지스터를 읽고 보고서를 출력한다. 레지스터 하나가 실패해도 나머지는 계속 읽는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    # 기본값 /dev/omx_follower는 udev 규칙 이름이다. 자세를 기록한 보드와 다를 수 있으니 by-id 경로를 권한다.
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
                # disable_torque=False: 닫을 때 토크 레지스터에 쓰지 않는다(읽기 전용 약속을 지킨다).
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
