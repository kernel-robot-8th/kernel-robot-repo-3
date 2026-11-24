"""운영자 핑키 화면이 핑키 함대를 보는 하나의 창구.

PinkyFleetClient로 핑키 관제를 읽는다. 그 클라이언트의 짧은 상태 캐시 덕분에 화면, 주문 흐름, 배정이 이미 한
응답을 나눠 쓴다. 여기서는 핑키마다 화면용 모양으로 다듬는다: 연결, 운행 준비와 빠진 센서, 배터리, 임무 단계와
경로 진행, 마지막 안전 감시기의 상태. 화면에서 오는 명령(정지, 전체 정지, SSH 비밀번호 등)은 여기를 거치고, 명령마다
핑키 관제의 답과 함께 짧은 기록에 남는다. 비밀번호는 그대로 넘길 뿐, 저장하거나 기록하거나 되돌려 보이지 않는다.

핑키 관제 URL이 없으면 게이트웨이는 모의 세계의 핑키를 simulated 표시와 함께 보여 주고, 아무것도 보내지 않는다.

위치: app.py가 하나 만들어(server.pinky_gateway) api/pinky.py의 HTTP 경로가 부른다. 위에서 본 지도(map_view),
ROS 모니터(ros_view), 시운전과 운영자 신호(test_mission, signal), 주행준비·위치 잡기 같은 명령도 여기 있다.
명령 기록(_log)은 여러 HTTP 스레드가 함께 쓰므로 `_lock` 아래에서 다룬다.
"""

from __future__ import annotations

import math
import threading
from collections import deque
from collections.abc import Callable
from typing import Any

from control_station import utc_now_iso
from control_station.orders.progress_view import PHASE_LABELS
from control_station.pinky.fleet_client import PinkyFleetClient, PinkyFleetError
from control_station.storage.store import PINKY_IDS, PINKY_NAMES

# 핑키가 주행하기 전에 살아 있어야 하는 실시간 입력. 핑키 관제의 이름과 화면에 쓰는 말.
SENSOR_LABELS = {
    "scan": "라이다", "odom": "오도메트리", "ultrasonic": "초음파", "ir": "IR", "map": "지도",
    "tf_pose": "위치", "line_control": "주행 제어", "safety_status": "안전 감시기",
}
# 핑키의 마지막 모터 관문(pinky_fleet_dashboard/scripts/motion_gate.py)이 움직임을 통과시키거나 막는 이유.
SAFETY_LABELS = {
    "allowed": "움직임 허용", "operator_stopped": "작업자 정지 유지", "route_or_traffic_wait": "경로·허가 없음, 정지",
    "sensor_or_command_stale": "센서·명령 끊김, 정지", "collision_stop": "충돌 예상, 정지",
    "invalid_command": "잘못된 명령, 정지",
}
LOG_SIZE = 30  # 화면 명령 기록을 최근 몇 개까지 남기는지


def _number(value: object) -> float | None:
    """숫자(int/float, bool 제외)면 float, 아니면 None."""
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _fmt(value: object, digits: int = 2) -> str | None:
    """숫자를 소수 `digits`자리 문자열로. 숫자가 아니면 None."""
    number = _number(value)
    return None if number is None else f"{number:.{digits}f}"


def sensor_details(topics: dict[str, Any]) -> list[dict[str, Any]]:
    """실시간 입력마다 나이(age_s, 초)와 값을 말로 옮긴 것(핑키 관제의 토픽 값 형식, 2026-11-14 기준).
    거리는 m, 속도는 m/s·rad/s, 전압은 V다."""
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


# 핑키 관제의 공유 구역을 운영자 말로. 가장 많이 알려 주는 것부터(대기 사유를 고를 때 이 순서로 정렬한다).
ZONE_WORDS = {"dock5": "적재 자리", "dock_queue": "적재 대기 자리", "passage9": "통로", "parking": "주차장 출입",
              "junction7": "교차로"}


def waiting_view(pinky_id: str, robot: dict[str, Any], route: dict[str, Any]) -> dict[str, Any] | None:
    """주행 중인 핑키가 다른 핑키 때문에 서 있는 이유: 요청한 구역과 그 구역을 잡은 핑키. 기다리지 않으면 None."""
    traffic = robot.get("traffic") if isinstance(robot.get("traffic"), dict) else {}
    zones = (traffic.get("waiting") or {}).get(pinky_id) or []
    if not zones or route.get("status") not in {"starting", "running"}:
        return None
    owners = traffic.get("owners") or {}
    zones = sorted(zones, key=lambda zone: list(ZONE_WORDS).index(zone) if zone in ZONE_WORDS else len(ZONE_WORDS))
    holders = [PINKY_NAMES[holder] for zone in zones
               if (holder := (owners.get(zone) or {}).get("robot_id")) in PINKY_NAMES and holder != pinky_id]
    place = ZONE_WORDS.get(zones[0], zones[0])
    by = f" · {', '.join(dict.fromkeys(holders))} 사용 중" if holders else ""
    # 다른 핑키가 구역을 쥐고 있으면, 꺼진 핑키가 구역을 붙잡고 있는 경우일 수 있어 비우는 방법을 덧붙인다.
    if holders and all((owners.get(zone) or {}).get("robot_id") != pinky_id for zone in zones):
        by += " (그 핑키가 꺼져 있으면 치운 뒤 그 줄의 [자리 비우기]를 눌러 주세요)"
    label = {"dock_queue": "앞 차례 대기 중"}.get(zones[0], f"{place} 대기 중")
    return {"zones": zones, "label": label, "text": f"{place}에 자리가 없어 기다리는 중{by}. 비면 바로 들어가요"}


def shape_pinky(pinky_id: str, robot: dict[str, Any], status: dict[str, Any]) -> dict[str, Any]:
    """핑키 카드 한 장: 핑키 관제의 레코드(`robot`)와 클라이언트가 그것을 읽은 결과(`status`, fleet_status)."""
    topics = robot.get("topics") or {}
    route = robot.get("route") or {}
    safety = (topics.get("safety_status") or {}).get("value") or {}
    sensors = {key: bool((topics.get(key) or {}).get("fresh")) for key in SENSOR_LABELS}
    pose = (topics.get("tf_pose") or {}).get("value") or {}
    labels = route.get("labels") if isinstance(route.get("labels"), list) else []
    connected = bool(status.get("connected"))
    waiting = waiting_view(pinky_id, robot, route) if connected else None
    owners = ((robot.get("traffic") or {}).get("owners") or {}) if isinstance(robot.get("traffic"), dict) else {}
    holding = [ZONE_WORDS.get(zone, zone) for zone, owner in owners.items() if (owner or {}).get("robot_id") == pinky_id]
    return {
        "pinky_id": pinky_id,
        # 다른 핑키 때문에 서 있음(적재 자리가 차 있음, 통로를 쓰는 중...): 그렇다고 보여 준다.
        "waiting": waiting,
        # 이 핑키가 쥔 공유 구역. 연결이 끊긴 핑키는 운영자가 경로를 지울 때까지 계속 쥐고 있다.
        "holding": holding,
        "name": PINKY_NAMES[pinky_id],
        "robot_name": robot.get("name"),
        "ip": robot.get("ip"),
        "simulated": bool(status.get("simulated")),
        "connected": connected,
        "ready": bool(status.get("ready_for_motion")),
        "readiness_label": status.get("readiness_label"),
        "missing": [SENSOR_LABELS[key] for key, fresh in sensors.items() if not fresh],
        "sensors": sensors,
        "battery_percent": status.get("battery_percent"),
        "battery_volts": status.get("battery_volts"),
        "phase": status.get("phase"),
        "phase_label": "연결 끊김" if not connected else waiting["label"] if waiting
        else PHASE_LABELS.get(status.get("phase"), status.get("phase")),
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
        "pose": {key: _number(pose.get(key)) for key in ("x_m", "y_m", "yaw_rad")},  # 지도 좌표계(m, rad), 3D 화면용
        # 마지막 라이다 위치 잡기: 핑키를 어디에 놓았는지, AMCL이 받았는지, 고를 수 있는 후보 자리들.
        "localization": robot.get("localization") if isinstance(robot.get("localization"), dict) else None,
        "mission": {"execution": route.get("execution"), "dropoff": route.get("dropoff"),
                    "message": route.get("message"), "return_home": bool(route.get("return_home")),
                    # 멈춘 핑키는 그 전에 명령을 받지 않으면 이 시각(epoch 초)에 스스로 집으로 간다
                    "auto_return_at": _number(route.get("auto_return_at"))} if route else None,
    }


def shape_mock_pinky(pinky_id: str, status: dict[str, Any]) -> dict[str, Any]:
    """모의 세계의 핑키를 같은 카드 모양으로. 실시간 입력은 모두 있는 것으로 친다."""
    return {
        "pinky_id": pinky_id, "name": PINKY_NAMES[pinky_id], "robot_name": None, "ip": None, "simulated": True,
        "connected": True, "ready": bool(status.get("ready_for_motion")), "readiness_label": status.get("readiness_label"),
        "missing": [], "sensors": dict.fromkeys(SENSOR_LABELS, True),
        "battery_percent": status.get("battery_percent"), "battery_volts": None,
        "phase": status.get("phase"), "phase_label": PHASE_LABELS.get(status.get("phase"), status.get("phase")),
        "message": status.get("message"), "route": {"status": None, "current_label": status.get("location"),
                                                    "completed": None, "total": None},
        "safety_state": None, "safety_label": None, "stop_pending": False, "sensor_details": [],
        "pose": {"x_m": None, "y_m": None, "yaw_rad": None}, "localization": None,
    }


# 핑키 함대가 알려 주는 셀의 고정점(traffic_config와 parking, 2026-11-14). 지도 좌표계 m. 모의 지도에 쓰고,
# 핑키 관제가 있으면 실시간으로 읽는다. MOCK_PARKING의 키는 핑키 id, MOCK_GRID는 0.05 m 칸 44 x 51개.
MOCK_PARKING = {"10": (0.4, 0.4), "11": (0.7, 0.4), "12": (1.0, 0.4)}
MOCK_LANDMARKS = {"dock5": (0.39, 1.64), "junction7": (1.55, 1.24)}
MOCK_GRID = {"width": 44, "height": 51, "resolution_m": 0.05, "origin_x_m": -0.349, "origin_y_m": -0.295}
LANDMARK_LABELS = {"dock5": "적재 자리", "junction7": "교차로 7"}


def _mock_grid() -> dict[str, Any]:
    """벽만 있는 빈 방, 실제 Nav2 지도와 같은 크기: 물건을 놓기에는 충분하고, 누가 봐도 실제 지도는 아니다.
    칸 값은 Nav2 점유 격자 관례대로 100 = 막힘, 0 = 빈 곳."""
    width, height = MOCK_GRID["width"], MOCK_GRID["height"]
    cells = [100 if x in (0, width - 1) or y in (0, height - 1) else 0 for y in range(height) for x in range(width)]
    return {**MOCK_GRID, "cells": cells}


def _topic_points(robot: dict[str, Any], key: str) -> list[list[float]]:
    """핑키 관제의 `plan`(Nav2 전역 경로) 또는 `trail`(핑키가 지나온 곳)의 [x, y] 지도 점(m). 잘못된 점은 뺀다."""
    points = (((robot.get("topics") or {}).get(key) or {}).get("value") or {}).get("points") or []
    return [[float(p[0]), float(p[1])] for p in points if isinstance(p, list) and len(p) == 2
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in p)]


def _waypoints(robot: dict[str, Any]) -> list[dict[str, Any]]:
    """핑키 관제가 아는 웨이포인트를 화면용 {id, label, x_m, y_m}로. 좌표가 없는 것은 뺀다."""
    points = []
    for point in robot.get("waypoints") or []:
        if not isinstance(point, dict):
            continue
        x, y = _number(point.get("x_m", point.get("x"))), _number(point.get("y_m", point.get("y")))
        if x is not None and y is not None:
            # 핑키 관제의 id가 POST /api/pinky/{id}/waypoints/{wid}/delete(지우기 버튼)에 필요하다.
            points.append({"id": str(point["id"]) if point.get("id") else None,
                           "label": str(point.get("label")), "x_m": x, "y_m": y})
    return points


class PinkyGateway:
    """핑키 화면의 읽기(view, map_view, ros_view)와 명령(_command를 거쳐 기록됨)을 한곳에 모은다.

    client가 None이면 모의 모드다: mock_status(모의 세계의 fleet_status)로 카드를 만들고 명령은 보내지 않는다.
    """

    def __init__(self, client: PinkyFleetClient | None, mock_status: Callable[[str], dict[str, Any]] | None = None,
                 markers: dict[str, Any] | None = None):
        self.client = client
        self.mock_status = mock_status
        self.markers = markers or {}  # 핑키 id -> PinkyMarker(어떤 마커를 달았는지, 등록됐는지)
        self._log: deque[dict[str, Any]] = deque(maxlen=LOG_SIZE)
        self._lock = threading.Lock()

    @property
    def source(self) -> str:
        """화면에 보일 출처: 실제(또는 가상) 핑키 관제면 pinky_fleet, 아니면 mock_world."""
        return "pinky_fleet" if self.client is not None else "mock_world"

    def view(self) -> dict[str, Any]:
        """핑키 화면: 모든 핑키 카드, 핑키 관제가 답했는지, 최근 명령."""
        pinkies: list[dict[str, Any]] = []
        error = None
        fleet: dict[str, Any] = {}  # 핑키 관제 자신의 상태: SSH 비밀번호를 갖고 있는지, 무엇으로 바쁜지
        if self.client is None:
            pinkies = [shape_mock_pinky(pid, self.mock_status(pid)) for pid in PINKY_IDS] if self.mock_status else []
        else:
            try:
                robots = self.client.robots()
                pinkies = [shape_pinky(pid, robots.get(pid) or {}, self.client.fleet_status(pid)) for pid in PINKY_IDS]
                fleet = self.client.extras()
            except (PinkyFleetError, OSError) as exc:
                error = str(exc)
        for card in pinkies:
            marker = self.markers.get(card["pinky_id"])
            card["marker"] = marker.as_dict() if marker is not None else None
        with self._lock:
            log = list(self._log)
        password_set = fleet.get("password_set")
        return {"source": self.source, "reachable": error is None, "error": error, "pinkies": pinkies,
                # 모르면 None(모의, 답 없음, 또는 그 값을 알려 주지 않는 핑키 관제)
                "password_set": password_set if isinstance(password_set, bool) else None,
                "busy": fleet.get("busy_robot"), "log": log[::-1], "checked_at": utc_now_iso()}

    def map_view(self, zones: list[dict[str, Any]] | None = None) -> dict[str, Any]:
        """셀을 위에서 그리는 데 필요한 모든 것(지도 좌표 m): Nav2 점유 격자와 keepout 마스크, 주차 자리, 5번과
        교차로 7, 기록된 웨이포인트, 수령 구역(마지막 웨이포인트를 알면 그 자리에 놓음), 핑키마다 실시간 자세.
        격자는 핑키 중 누구든 지도를 받은 첫 번째 것에서 가져온다."""
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
            "source": self.source, "simulated": self.client.simulated, "available": grid is not None,
            "reason": None if grid is not None else "핑키 관제가 아직 지도를 받지 못했습니다",
            "grid": grid, "keepout": extras.get("keepout"),
            "parking_area": (traffic.get("parking") or {}).get("bounds"),
            "parking": [{"pinky_id": pinky_id, "name": PINKY_NAMES[pinky_id], **(robots.get(pinky_id) or {}).get("parking", {})}
                        for pinky_id in PINKY_IDS if (robots.get(pinky_id) or {}).get("parking")],
            "landmarks": landmarks, "waypoints": waypoints,
            "zones": [self._place_zone(zone, waypoints) for zone in zones],
            "pinkies": [{"pinky_id": pid, "name": card["name"], "connected": card["connected"],
                         "phase_label": card["phase_label"], **card["pose"],
                         "plan": _topic_points(robots.get(pid) or {}, "plan"),
                         "trail": _topic_points(robots.get(pid) or {}, "trail")} for pid, card in cards.items()],
            "note": None if any(waypoints.values()) else
            "이 PC의 핑키 관제에 웨이포인트가 없습니다(data/waypoints.json). 경로를 달리려면 기록하거나 가져와야 합니다",
        }

    @staticmethod
    def _place_zone(zone: dict[str, Any], waypoints: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
        """수령 구역을 그 경로의 마지막 웨이포인트 자리에 놓는다. 그 웨이포인트를 모르면 좌표는 None."""
        last = (zone.get("route_labels") or [None])[-1]
        point = next((p for points in waypoints.values() for p in points if p["label"] == last), None)
        return {"zone_id": zone.get("zone_id"), "label": zone.get("label"), "waypoint": last,
                "x_m": point["x_m"] if point else None, "y_m": point["y_m"] if point else None}

    def _mock_map(self, zones: list[dict[str, Any]]) -> dict[str, Any]:
        """모의 세계에서 그린 같은 그림: 알려진 고정점, 임무 단계로 자리를 정한 핑키들."""
        cards = {card["pinky_id"]: card for card in self.view()["pinkies"]}
        zone_xy = (1.55, 2.05)  # 모의 수령 구역을 놓을 임시 자리(m, 모의라고 표시됨)
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

    # ROS 모니터에서 빼는 무겁거나 시끄러운 필드: 점유 격자, IR 원본 기록, 점 목록. 길이("N칸")만 보인다.
    _ROS_HEAVY = {"cells", "history", "points"}

    def ros_view(self, pinky_id: str) -> dict[str, Any]:
        """ROS 모니터용: 핑키 관제가 이 핑키에 대해 읽는 모든 토픽(값, 나이, fresh). 지도 격자는 크기만 남긴다.
        그리고 핑키 관제 자신이 이 핑키에 대해 가진 상태(stage, message, error, route, localization)."""
        self._known(pinky_id)
        if self.client is None:
            return {"ok": False, "simulated": True, "error": "모의 모드라 ROS 수신이 없습니다"}
        try:
            robot = self.client.robots().get(pinky_id) or {}
        except (PinkyFleetError, OSError) as exc:
            return {"ok": False, "error": str(exc)}
        topics = {}
        for key, topic in (robot.get("topics") or {}).items():
            value = topic.get("value")
            if isinstance(value, dict):
                value = {k: (f"{len(v)}칸" if k in self._ROS_HEAVY and isinstance(v, list) else v) for k, v in value.items()}
            topics[key] = {"value": value, "age_s": topic.get("age_s"), "fresh": topic.get("fresh")}
        route = robot.get("route") or {}
        return {"ok": True, "pinky_id": pinky_id, "name": PINKY_NAMES[pinky_id], "robot_name": robot.get("name"),
                "domain": robot.get("domain"), "stage": robot.get("stage"), "message": robot.get("message"),
                "error": robot.get("error"), "ssh_reachable": robot.get("ssh_reachable"),
                "operational": robot.get("operational"), "localization": robot.get("localization"),
                "route": {k: route.get(k) for k in ("status", "phase", "current_label", "completed", "message",
                                                    "detail", "execution", "dropoff")} if route else None,
                "topics": topics, "checked_at": utc_now_iso()}

    def nav_log(self, pinky_id: str) -> dict[str, Any]:
        """개발자용 Nav2 로그(핑키 관제가 SSH로 읽음). 기록에 남기지 않는 읽기다."""
        self._known(pinky_id)
        if self.client is None:
            return {"ok": False, "error": "모의 모드라 Nav2 로그가 없습니다"}
        try:
            status, answer = self.client.nav_log(pinky_id)
        except OSError as exc:
            return {"ok": False, "error": str(exc)}
        return {"ok": bool(answer.get("ok")) and status == 200, "output": answer.get("output") or answer.get("error") or ""}

    def stop(self, pinky_id: str) -> dict[str, Any]:
        """핑키 관제를 거쳐 핑키 한 대를 멈춘다. 받음(accepted)은 핑키 관제가 받았다는 뜻이지 핑키가 섰다는 뜻은 아니다."""
        if pinky_id not in PINKY_IDS:
            raise ValueError(f"unknown Pinky {pinky_id}")
        return self._command("stop", pinky_id, lambda client: client.stop(pinky_id))

    MAPS = ("map-proto.yaml", "my_map.yaml", "pinklab.yaml")  # 핑키 관제가 주행준비에 쓸 수 있는 지도들

    def prepare(self, pinky_id: str, map_name: str = "map-proto.yaml") -> dict[str, Any]:
        """주행준비. map-proto.yaml이 keepout 마스크가 있는 지도다."""
        self._known(pinky_id)
        if map_name not in self.MAPS:
            raise ValueError("지도는 " + ", ".join(self.MAPS) + " 중 하나여야 합니다")
        return self._command("prepare", pinky_id, lambda client: client.prepare(pinky_id, map_name))

    def prepare_all(self, map_name: str = "map-proto.yaml") -> dict[str, Any]:
        """아직 준비되지 않은 모든 핑키를 차례로 주행준비한다. 각각 끝에 자기 주차 자리에서 위치를 잡는다."""
        if map_name not in self.MAPS:
            raise ValueError("지도는 " + ", ".join(self.MAPS) + " 중 하나여야 합니다")
        return self._command("prepare_all", None, lambda client: client.prepare_all(map_name))

    def initial_pose(self, pinky_id: str) -> dict[str, Any]:
        """AMCL 자세를 핑키 자신의 주차 자리로 정한다(핑키가 거기 서 있어야 함)."""
        self._known(pinky_id)
        return self._command("initial_pose", pinky_id, lambda client: client.initial_pose(pinky_id))

    def localize(self, pinky_id: str, pose: object = None) -> dict[str, Any]:
        """라이다 위치 잡기. `pose`는 운영자가 고른 후보({x_m, y_m, yaw_rad}) 또는 자동이면 None."""
        self._known(pinky_id)
        if pose is not None:
            values = [_number(pose.get(key)) if isinstance(pose, dict) else None for key in ("x_m", "y_m", "yaw_rad")]
            if None in values or not all(math.isfinite(v) for v in values):
                raise ValueError("고른 후보의 x_m, y_m, yaw_rad가 필요합니다")
            pose = dict(zip(("x_m", "y_m", "yaw_rad"), values))
        return self._command("localize", pinky_id, lambda client: client.localize(pinky_id, pose))

    def set_pose(self, pinky_id: str, pose: object) -> dict[str, Any]:
        """위치 다시 잡기: 운영자가 지도 위에 놓은 핑키 자리 {x_m, y_m, yaw_rad}(yaw = 핑키가 보는 방향)."""
        self._known(pinky_id)
        values = [_number(pose.get(key)) if isinstance(pose, dict) else None for key in ("x_m", "y_m", "yaw_rad")]
        if None in values or not all(math.isfinite(v) for v in values):
            raise ValueError("지정한 위치의 x_m, y_m, yaw_rad가 필요합니다")
        pose = dict(zip(("x_m", "y_m", "yaw_rad"), values))
        return self._command("set_pose", pinky_id, lambda client: client.set_pose(pinky_id, pose))

    def locate(self, pinky_id: str) -> dict[str, Any]:
        """읽기 전용: 지금 라이다가 이 핑키를 놓는 자리들. {ok, match} 또는 {ok: False, error}."""
        self._known(pinky_id)
        if self.client is None:
            return {"ok": False, "simulated": True, "error": "모의 모드라 라이다 위치가 없습니다"}
        try:
            status, answer = self.client.locate(pinky_id)
        except OSError as exc:
            return {"ok": False, "error": str(exc)}
        if status != 200 or not isinstance(answer.get("match"), dict):
            return {"ok": False, "error": answer.get("error") or f"HTTP {status}"}
        return {"ok": True, "match": answer["match"]}

    # 시운전의 운영자 신호와 그 신호를 받는 임무 단계: 5번 도킹(다시 맞추기 포함), 5번 적재 완료, 내려 주는 곳 전달.
    SIGNALS = {"dock_complete": "docking_manual", "redock": "docking_manual", "received": "docked_manual",
               "mission_complete": "course_complete"}

    def test_mission(self, pinky_id: str, dropoff: object, via: object = None) -> dict[str, Any]:
        """주문 없는 시운전: (`via` 경유 →) 5번 적재 → `dropoff` 배달 → 주차장(돌아올 때도 `via`를 거친다).
        웨이포인트 이름은 48자까지, 경유지는 6개까지 받는다."""
        self._known(pinky_id)
        if not isinstance(dropoff, str) or not dropoff.strip() or len(dropoff) > 48:
            raise ValueError("배달 목적지 웨이포인트를 고르세요")
        via = [] if via is None else via
        if not isinstance(via, list) or len(via) > 6 or any(not isinstance(v, str) or not v.strip() or len(v) > 48 for v in via):
            raise ValueError("경유지는 웨이포인트 이름 목록(최대 6개)이어야 합니다")
        labels = [v.strip() for v in via]
        return self._command("test_mission", pinky_id,
                             lambda client: client.start_test_mission(pinky_id, dropoff.strip(), labels))

    def signal(self, pinky_id: str, action: object, offset: object = None) -> dict[str, Any]:
        """핑키의 지금 단계에 맞는 도킹완료 / 적재 완료 / 수령 완료 신호를 핑키 관제의 임무 토큰과 함께 보낸다.

        보내기 직전에 핑키 관제에서 단계와 토큰을 다시 읽고, 지금 단계에 맞지 않는 신호는 보내지 않고 409로 남긴다.
        redock에 `offset`(dict)이 있으면 천장 카메라가 잰 어긋남을 함께 보낸다. 모의 모드에서는 기록만 남는다.
        """
        self._known(pinky_id)
        if action not in self.SIGNALS:
            raise ValueError("신호는 " + ", ".join(self.SIGNALS) + " 중 하나여야 합니다")
        if self.client is None:
            return self._command(action, pinky_id, lambda client: (0, {}))
        def call(client: PinkyFleetClient) -> tuple[int, dict[str, Any]]:
            route = (client.robots().get(pinky_id) or {}).get("route") or {}
            # redock은 dock_complete 뒤(docked_manual)에도 받는다: 적재가 끝나기 전에는 다시 불러들일 수 있다.
            allowed = {self.SIGNALS[action]} | ({"docked_manual"} if action == "redock" else set())
            if route.get("phase") not in allowed:
                return 409, {"error": f"지금 단계({route.get('phase') or '없음'})에서는 보낼 수 없는 신호입니다"}
            try:
                result = (client.mission_complete(pinky_id, route.get("mission_token")) if action == "mission_complete"
                          else client.fleet_confirm(pinky_id, action, route.get("mission_token"), offset)
                          if action == "redock" and isinstance(offset, dict)
                          else client.fleet_confirm(pinky_id, action, route.get("mission_token")))
            except PinkyFleetError as exc:
                return 409 if exc.error_code == "PINKY_SIGNAL_REJECTED" else 502, {"error": str(exc)}
            return 202, {"ok": True, "message": result.get("message") or "신호를 보냈습니다"}
        return self._command(action, pinky_id, call)

    def clear_route(self, pinky_id: str) -> dict[str, Any]:
        """기존 경로 날리기(도는 경로는 먼저 멈춤). 꺼진 핑키가 쥔 공유 구역을 풀 때도 쓴다."""
        self._known(pinky_id)
        return self._command("clear_route", pinky_id, lambda client: client.clear_route(pinky_id))

    def return_home(self, pinky_id: str, via: object = None) -> dict[str, Any]:
        """핑키를 `via`(최대 6개)를 거쳐 자기 주차 자리로 보낸다."""
        self._known(pinky_id)
        via = [] if via is None else via
        if not isinstance(via, list) or len(via) > 6 or any(not isinstance(v, str) or not v.strip() or len(v) > 48 for v in via):
            raise ValueError("경유지는 웨이포인트 이름 목록(최대 6개)이어야 합니다")
        labels = [v.strip() for v in via]
        return self._command("return_home", pinky_id, lambda client: client.return_home(pinky_id, labels))

    def reset(self, pinky_id: str, map_name: str = "map-proto.yaml") -> dict[str, Any]:
        """초기 세팅: 정지 유지, 경로 버리기, 주행준비를 처음부터."""
        self._known(pinky_id)
        if map_name not in self.MAPS:
            raise ValueError("지도는 " + ", ".join(self.MAPS) + " 중 하나여야 합니다")
        return self._command("reset", pinky_id, lambda client: client.reset(pinky_id, map_name))

    def nav_restart(self, pinky_id: str) -> dict[str, Any]:
        """Nav2 다시 띄우기(액션 서버가 없을 때)."""
        self._known(pinky_id)
        return self._command("nav_restart", pinky_id, lambda client: client.nav_restart(pinky_id))

    def record_waypoint(self, pinky_id: str, label: str) -> dict[str, Any]:
        """핑키의 지금 자세를 `label`(1~48자) 웨이포인트로 저장한다."""
        self._known(pinky_id)
        if not isinstance(label, str) or not label.strip() or len(label.strip()) > 48:
            raise ValueError("지점 이름은 1~48자로 입력하세요")
        return self._command("waypoint", pinky_id, lambda client: client.record_waypoint(pinky_id, label.strip()))

    def delete_waypoint(self, pinky_id: str, waypoint_id: str) -> dict[str, Any]:
        """웨이포인트 하나를 지운다. id는 핑키 관제가 준 32자리 소문자 16진수만 받아, 경로에 다른 글자가 끼지 않게 한다."""
        self._known(pinky_id)
        if not isinstance(waypoint_id, str) or len(waypoint_id) != 32 or not all(c in "0123456789abcdef" for c in waypoint_id):
            raise ValueError("웨이포인트 id가 올바르지 않습니다")
        return self._command("waypoint_delete", pinky_id, lambda client: client.delete_waypoint(pinky_id, waypoint_id))

    @staticmethod
    def _known(pinky_id: str) -> None:
        """모르는 핑키 id면 ValueError."""
        if pinky_id not in PINKY_IDS:
            raise ValueError(f"unknown Pinky {pinky_id}")

    def stop_all(self) -> list[dict[str, Any]]:
        """모든 핑키에 정지 요청을 보내고 하나씩 따로 기록한다. 하나가 거절돼도 나머지는 그대로 보낸다."""
        return [self.stop(pinky_id) for pinky_id in PINKY_IDS]

    def send_password(self, password: str) -> dict[str, Any]:
        """SSH 비밀번호를 핑키 관제로 넘긴다. 기록에는 'password' 동작과 핑키 관제의 답만 남고 비밀번호는 남지 않는다."""
        if not isinstance(password, str) or not password or len(password) > 256:
            raise ValueError("비밀번호를 입력하세요")
        return self._command("password", None, lambda client: client.send_password(password))

    def _command(self, action: str, pinky_id: str | None,
                 call: Callable[[PinkyFleetClient], tuple[int, dict[str, Any]]]) -> dict[str, Any]:
        """화면 명령 하나를 보내고 결과를 기록에 남겨 돌려준다. 모의 모드면 보내지 않고 그렇다고 기록한다.

        200/201/202이고 응답의 ok가 거짓이 아니면 성공이다. 연결 실패(OSError)도 실패로 기록한다.
        """
        if self.client is None:
            result = {"ok": False, "http_status": None, "simulated": True,
                      "message": "모의 모드라 핑키 관제에 보내지 않았습니다"}
        else:
            try:
                status, answer = call(self.client)
                ok = status in {200, 201, 202} and bool(answer.get("ok", True))  # 201: 웨이포인트가 저장됨
                message = answer.get("message") or answer.get("error") or f"HTTP {status}"
                result = {"ok": ok, "http_status": status, "simulated": answer.get("simulated") is True,
                          "message": message}
            except OSError as exc:
                result = {"ok": False, "http_status": None, "simulated": False, "message": str(exc)}
        entry = {"at": utc_now_iso(), "action": action, "pinky_id": pinky_id, **result}
        with self._lock:
            self._log.append(entry)
        return entry
