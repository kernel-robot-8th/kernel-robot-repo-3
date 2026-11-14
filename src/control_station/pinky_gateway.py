"""One window onto the Pinky fleet for the operator's Pinky screen.

It reads the fleet dashboard through PinkyFleetClient, whose short status cache already lets the screen, the order
flow and the assignment share one answer, and shapes each Pinky for the screen: connection, readiness and the
sensors it is missing, battery, mission phase and route progress, and the final safety monitor's state. Commands
from the screen (stop, stop all, the SSH password) go through here and each one is kept in a short log with the
dashboard's answer. The password is passed on as is; it is never stored, logged or echoed back.

Without a dashboard URL the gateway shows the mock world's Pinkys, marked simulated, and sends nothing.
"""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable
from datetime import datetime, timezone
from typing import Any

from control_station.flow_view import PHASE_LABELS
from control_station.pinky_fleet import PinkyFleetClient, PinkyFleetError
from control_station.store import PINKY_IDS, PINKY_NAMES

# The live feeds a Pinky needs before it may drive, in the dashboard's names, with the words the screen uses.
SENSOR_LABELS = {
    "scan": "라이다", "odom": "오도메트리", "ultrasonic": "초음파", "ir": "IR", "map": "지도",
    "tf_pose": "위치", "line_control": "주행 제어", "safety_status": "안전 감시기",
}
# The Pinky's final motor gate (pinky_fleet_dashboard/scripts/motion_gate.py): why it lets motion through or not.
SAFETY_LABELS = {
    "allowed": "움직임 허용", "operator_stopped": "작업자 정지 유지", "route_or_traffic_wait": "경로·허가 없음, 정지",
    "sensor_or_command_stale": "센서·명령 끊김, 정지", "collision_stop": "충돌 예상, 정지",
    "invalid_command": "잘못된 명령, 정지",
}
LOG_SIZE = 30


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _number(value: object) -> float | None:
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def shape_pinky(pinky_id: str, robot: dict[str, Any], status: dict[str, Any]) -> dict[str, Any]:
    """One Pinky card: the dashboard's record (`robot`) plus the client's reading of it (`status`)."""
    topics = robot.get("topics") or {}
    route = robot.get("route") or {}
    safety = (topics.get("safety_status") or {}).get("value") or {}
    sensors = {key: bool((topics.get(key) or {}).get("fresh")) for key in SENSOR_LABELS}
    labels = route.get("labels") if isinstance(route.get("labels"), list) else []
    connected = bool(status.get("connected"))
    return {
        "pinky_id": pinky_id,
        "name": PINKY_NAMES[pinky_id],
        "robot_name": robot.get("name"),
        "ip": robot.get("ip"),
        "simulated": False,
        "connected": connected,
        "ready": bool(status.get("ready_for_motion")),
        "readiness_label": status.get("readiness_label"),
        "missing": [SENSOR_LABELS[key] for key, fresh in sensors.items() if not fresh],
        "sensors": sensors,
        "battery_percent": status.get("battery_percent"),
        "battery_volts": status.get("battery_volts"),
        "phase": status.get("phase"),
        "phase_label": "연결 끊김" if not connected else PHASE_LABELS.get(status.get("phase"), status.get("phase")),
        "message": status.get("message") or robot.get("message"),
        "route": {
            "status": route.get("status"),
            "current_label": route.get("current_label"),
            "completed": _number(route.get("completed")),
            "total": len(labels) or None,
        },
        "safety_state": safety.get("state"),
        "safety_label": SAFETY_LABELS.get(safety.get("state"), safety.get("state")),
        "stop_pending": bool(route.get("stop_pending")),
    }


def shape_mock_pinky(pinky_id: str, status: dict[str, Any]) -> dict[str, Any]:
    """A mock world Pinky in the same card shape; every live feed counts as present."""
    return {
        "pinky_id": pinky_id, "name": PINKY_NAMES[pinky_id], "robot_name": None, "ip": None, "simulated": True,
        "connected": True, "ready": bool(status.get("ready_for_motion")), "readiness_label": status.get("readiness_label"),
        "missing": [], "sensors": dict.fromkeys(SENSOR_LABELS, True),
        "battery_percent": status.get("battery_percent"), "battery_volts": None,
        "phase": status.get("phase"), "phase_label": PHASE_LABELS.get(status.get("phase"), status.get("phase")),
        "message": status.get("message"), "route": {"status": None, "current_label": status.get("location"),
                                                    "completed": None, "total": None},
        "safety_state": None, "safety_label": None, "stop_pending": False,
    }


class PinkyGateway:
    def __init__(self, client: PinkyFleetClient | None, mock_status: Callable[[str], dict[str, Any]] | None = None):
        self.client = client
        self.mock_status = mock_status
        self._log: deque[dict[str, Any]] = deque(maxlen=LOG_SIZE)
        self._lock = threading.Lock()

    @property
    def source(self) -> str:
        return "pinky_fleet" if self.client is not None else "mock_world"

    def view(self) -> dict[str, Any]:
        """The Pinky screen: every Pinky card, whether the dashboard answered, and the recent commands."""
        pinkies: list[dict[str, Any]] = []
        error = None
        if self.client is None:
            pinkies = [shape_mock_pinky(pid, self.mock_status(pid)) for pid in PINKY_IDS] if self.mock_status else []
        else:
            try:
                robots = self.client.robots()
                pinkies = [shape_pinky(pid, robots.get(pid) or {}, self.client.fleet_status(pid)) for pid in PINKY_IDS]
            except (PinkyFleetError, OSError) as exc:
                error = str(exc)
        with self._lock:
            log = list(self._log)
        return {"source": self.source, "reachable": error is None, "error": error, "pinkies": pinkies,
                "log": log[::-1], "checked_at": _now()}

    def stop(self, pinky_id: str) -> dict[str, Any]:
        """Stop one Pinky through the dashboard. Accepted means the dashboard took it, not that the Pinky stopped."""
        if pinky_id not in PINKY_IDS:
            raise ValueError(f"unknown Pinky {pinky_id}")
        return self._command("stop", pinky_id, lambda client: client.stop(pinky_id))

    def stop_all(self) -> list[dict[str, Any]]:
        """Stop request to every Pinky, each recorded on its own; one refusal does not hold back the others."""
        return [self.stop(pinky_id) for pinky_id in PINKY_IDS]

    def send_password(self, password: str) -> dict[str, Any]:
        if not isinstance(password, str) or not password or len(password) > 256:
            raise ValueError("비밀번호를 입력하세요")
        return self._command("password", None, lambda client: client.send_password(password))

    def _command(self, action: str, pinky_id: str | None,
                 call: Callable[[PinkyFleetClient], tuple[int, dict[str, Any]]]) -> dict[str, Any]:
        if self.client is None:
            result = {"ok": False, "http_status": None, "simulated": True,
                      "message": "모의 모드라 핑키 관제에 보내지 않았습니다"}
        else:
            try:
                status, answer = call(self.client)
                ok = status in {200, 202} and bool(answer.get("ok", True))
                message = answer.get("message") or answer.get("error") or f"HTTP {status}"
                result = {"ok": ok, "http_status": status, "simulated": False, "message": message}
            except OSError as exc:
                result = {"ok": False, "http_status": None, "simulated": False, "message": str(exc)}
        entry = {"at": _now(), "action": action, "pinky_id": pinky_id, **result}
        with self._lock:
            self._log.append(entry)
        return entry
