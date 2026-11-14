"""One window onto the Pinky fleet for the operator's Pinky screen.

It reads the fleet dashboard through PinkyFleetClient, whose short status cache already lets the screen, the order
flow and the assignment share one answer, and shapes each Pinky for the screen: connection, readiness and the
sensors it is missing, battery, mission phase and route progress, and the final safety monitor's state. Commands
from the screen (stop, stop all, the SSH password) go through here and each one is kept in a short log with the
dashboard's answer. The password is passed on as is; it is never stored, logged or echoed back.

Without a dashboard URL the gateway shows the mock world's Pinkys, marked simulated, and sends nothing.
"""

from __future__ import annotations

import math
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


def _fmt(value: object, digits: int = 2) -> str | None:
    number = _number(value)
    return None if number is None else f"{number:.{digits}f}"


def sensor_details(topics: dict[str, Any]) -> list[dict[str, Any]]:
    """Each live feed with its age and its reading in words (the dashboard's topic values, as of 2026-11-14)."""
    def value(key: str) -> dict[str, Any]:
        return (topics.get(key) or {}).get("value") or {}

    scan, sonar, ir, odom = value("scan"), value("ultrasonic"), value("ir"), value("odom")
    pose, imu, battery, control = value("tf_pose"), value("imu"), value("battery_voltage"), value("line_control")
    yaw = _number(pose.get("yaw_rad"))
    accel = (imu.get("linear_acceleration") or {}).get("z")
    texts = {
        "scan": f"앞 가장 가까운 물체 {_fmt(scan.get('front_min_m'))} m · {scan.get('samples')}점" if scan else None,
        "ultrasonic": f"{_fmt(sonar.get('range_m'))} m (측정 {_fmt(sonar.get('min_m'))}~{_fmt(sonar.get('max_m'), 1)} m)" if sonar else None,
        "ir": " · ".join(str(channel) for channel in ir.get("channels", [])) or None,
        "odom": f"{_fmt(odom.get('linear_mps'))} m/s · 회전 {_fmt(odom.get('angular_rps'))} rad/s" if odom else None,
        "map": "지도 받음" if (topics.get("map") or {}).get("fresh") else None,
        "tf_pose": (f"x {_fmt(pose.get('x_m'))} · y {_fmt(pose.get('y_m'))} m · {math.degrees(yaw):.0f}°"
                    if pose and yaw is not None else None),
        "line_control": control.get("line_algorithm"),
        "safety_status": SAFETY_LABELS.get(value("safety_status").get("state"), value("safety_status").get("state")),
        "imu": f"수직 가속 {_fmt(accel)} m/s²" if accel is not None else None,
        "battery_voltage": f"{_fmt(battery.get('voltage_v'))} V" if battery else None,
    }
    labels = {**SENSOR_LABELS, "imu": "IMU", "battery_voltage": "배터리 전압"}
    return [{"key": key, "label": labels[key], "fresh": bool((topics.get(key) or {}).get("fresh")),
             "age_s": _number((topics.get(key) or {}).get("age_s")), "text": texts[key]} for key in texts]


def shape_pinky(pinky_id: str, robot: dict[str, Any], status: dict[str, Any]) -> dict[str, Any]:
    """One Pinky card: the dashboard's record (`robot`) plus the client's reading of it (`status`)."""
    topics = robot.get("topics") or {}
    route = robot.get("route") or {}
    safety = (topics.get("safety_status") or {}).get("value") or {}
    sensors = {key: bool((topics.get(key) or {}).get("fresh")) for key in SENSOR_LABELS}
    pose = (topics.get("tf_pose") or {}).get("value") or {}
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
        "sensor_details": sensor_details(topics),
        "pose": {key: _number(pose.get(key)) for key in ("x_m", "y_m", "yaw_rad")},  # map frame, for the 3D view
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
        "safety_state": None, "safety_label": None, "stop_pending": False, "sensor_details": [],
        "pose": {"x_m": None, "y_m": None, "yaw_rad": None},
    }


# Fixed points of the cell as the Pinky fleet reports them (traffic_config and parking, 2026-11-14), used for the
# mock map; with a dashboard they are read live.
MOCK_PARKING = {"10": (0.4, 0.4), "11": (0.7, 0.4), "12": (1.0, 0.4)}
MOCK_LANDMARKS = {"dock5": (0.39, 1.64), "junction7": (1.55, 1.24)}
MOCK_GRID = {"width": 44, "height": 51, "resolution_m": 0.05, "origin_x_m": -0.349, "origin_y_m": -0.295}
LANDMARK_LABELS = {"dock5": "5번 적재", "junction7": "교차로 7"}


def _mock_grid() -> dict[str, Any]:
    """An empty room with walls, the size of the real Nav2 map: enough to place things, clearly not the map."""
    width, height = MOCK_GRID["width"], MOCK_GRID["height"]
    cells = [100 if x in (0, width - 1) or y in (0, height - 1) else 0 for y in range(height) for x in range(width)]
    return {**MOCK_GRID, "cells": cells}


def _waypoints(robot: dict[str, Any]) -> list[dict[str, Any]]:
    points = []
    for point in robot.get("waypoints") or []:
        if not isinstance(point, dict):
            continue
        x, y = _number(point.get("x_m", point.get("x"))), _number(point.get("y_m", point.get("y")))
        if x is not None and y is not None:
            points.append({"label": str(point.get("label")), "x_m": x, "y_m": y})
    return points


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

    def map_view(self, zones: list[dict[str, Any]] | None = None) -> dict[str, Any]:
        """Everything to draw the cell from above, in map metres: the Nav2 occupancy grid and keepout mask, the
        parking spots, station 5 and junction 7, recorded waypoints, the receiving zones (placed where their last
        waypoint is known) and each Pinky's live pose."""
        zones = zones or []
        if self.client is None:
            return self._mock_map(zones)
        try:
            robots = self.client.robots()
            extras = self.client.extras()
        except (PinkyFleetError, OSError) as exc:
            return {"source": self.source, "simulated": False, "available": False, "reason": str(exc)}
        grid = next((value for robot in robots.values()
                     if (value := ((robot.get("topics") or {}).get("map") or {}).get("value")) and value.get("cells")), None)
        traffic = next((value.get("traffic_config") for robot in robots.values()
                        if (value := ((robot.get("topics") or {}).get("line_control") or {}).get("value") or {})
                        and value.get("traffic_config")), {}) or {}
        landmarks = [{"key": key, "label": LANDMARK_LABELS[key], "x_m": _number(spot.get("x_m")), "y_m": _number(spot.get("y_m"))}
                     for key, spot in traffic.items() if key in LANDMARK_LABELS and isinstance(spot, dict)]
        waypoints = {pinky_id: _waypoints(robots.get(pinky_id) or {}) for pinky_id in PINKY_IDS}
        cards = {card["pinky_id"]: card for card in self.view()["pinkies"]}
        return {
            "source": self.source, "simulated": False, "available": grid is not None,
            "reason": None if grid is not None else "핑키 관제가 아직 지도를 받지 못했습니다",
            "grid": grid, "keepout": extras.get("keepout"),
            "parking_area": (traffic.get("parking") or {}).get("bounds"),
            "parking": [{"pinky_id": pinky_id, "name": PINKY_NAMES[pinky_id], **(robots.get(pinky_id) or {}).get("parking", {})}
                        for pinky_id in PINKY_IDS if (robots.get(pinky_id) or {}).get("parking")],
            "landmarks": landmarks, "waypoints": waypoints,
            "zones": [self._place_zone(zone, waypoints) for zone in zones],
            "pinkies": [{"pinky_id": pid, "name": card["name"], "connected": card["connected"],
                         "phase_label": card["phase_label"], **card["pose"]} for pid, card in cards.items()],
            "note": None if any(waypoints.values()) else
            "이 PC의 핑키 관제에 웨이포인트가 없습니다(data/waypoints.json). 경로를 달리려면 기록하거나 가져와야 합니다",
        }

    @staticmethod
    def _place_zone(zone: dict[str, Any], waypoints: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
        last = (zone.get("route_labels") or [None])[-1]
        point = next((p for points in waypoints.values() for p in points if p["label"] == last), None)
        return {"zone_id": zone.get("zone_id"), "label": zone.get("label"), "waypoint": last,
                "x_m": point["x_m"] if point else None, "y_m": point["y_m"] if point else None}

    def _mock_map(self, zones: list[dict[str, Any]]) -> dict[str, Any]:
        """The same picture from the mock world: known fixed points, Pinkys placed by their mission phase."""
        cards = {card["pinky_id"]: card for card in self.view()["pinkies"]}
        zone_xy = (1.55, 2.05)  # a stand-in spot for the mock receiving zone, labeled as mock
        def place(pinky_id: str) -> tuple[float, float]:
            phase = (cards.get(pinky_id) or {}).get("phase")
            if phase in {"docking_manual", "docking_anchor", "docked_manual"}:
                return MOCK_LANDMARKS["dock5"]
            if phase == "course_complete":
                return zone_xy
            if phase == "driving":
                return MOCK_LANDMARKS["junction7"]
            return MOCK_PARKING[pinky_id]
        return {
            "source": self.source, "simulated": True, "available": True, "reason": None, "grid": _mock_grid(),
            "keepout": None, "parking_area": [0.2, 1.2, 0.2, 0.6],
            "parking": [{"pinky_id": pid, "name": PINKY_NAMES[pid], "label": f"{PINKY_NAMES[pid]} 주차장",
                         "x_m": x, "y_m": y, "yaw_rad": math.pi} for pid, (x, y) in MOCK_PARKING.items()],
            "landmarks": [{"key": key, "label": LANDMARK_LABELS[key], "x_m": x, "y_m": y} for key, (x, y) in MOCK_LANDMARKS.items()],
            "waypoints": dict.fromkeys(PINKY_IDS, []),
            "zones": [{"zone_id": zone.get("zone_id"), "label": zone.get("label"), "waypoint": None,
                       "x_m": zone_xy[0], "y_m": zone_xy[1]} for zone in zones],
            "pinkies": [{"pinky_id": pid, "name": PINKY_NAMES[pid], "connected": True,
                         "phase_label": (cards.get(pid) or {}).get("phase_label"),
                         "x_m": place(pid)[0], "y_m": place(pid)[1], "yaw_rad": math.pi} for pid in PINKY_IDS],
            "note": "모의 지도입니다. 벽과 위치는 실제 좌표를 흉내 낸 것입니다",
        }

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
