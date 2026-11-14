"""Pinky screen API: a thin HTTP layer over control_station.pinky_gateway.PinkyGateway.

GET  /api/pinky                 every Pinky card, whether the fleet dashboard answered, recent commands
GET  /api/pinky/map             the cell from above in map metres: Nav2 grid, keepout, fixed points, poses
POST /api/pinky/{id}/stop       stop one Pinky (the dashboard latches a stop and cancels its Nav2 goal)
POST /api/pinky/stop-all        stop request to every Pinky
POST /api/pinky/credentials     {password}: passed to the dashboard, never stored or logged here
POST /api/pinky/{id}/ready {map}, /initial-pose, /nav-restart, /waypoints {label}, /waypoints/{wid}/delete
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.store import PINKY_IDS

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def _answer(h: ControlStationHandler, entry: dict[str, Any]) -> None:
    """202 when the dashboard accepted it; otherwise its own refusal status (401 no password, 409 busy) or 502."""
    status = (202 if entry["http_status"] in {None, 202} else entry["http_status"]) if entry["ok"] else (
        entry["http_status"] if entry["http_status"] in {400, 401, 409} else 502)
    if entry.get("simulated"):
        status = 409
    h._json(status, {"command": entry})


def get_pinky(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 화면: 핑키 3대 상태와 최근 명령"""
    h._json(200, h.app.pinky_gateway.view())


def get_map(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 지도: Nav2 격자, 금지 구역, 주차장, 5번, 교차로 7, 웨이포인트, 수령 구역, 핑키 위치(지도 좌표)"""
    zones = [zone.as_dict() for zone in getattr(h.app.order_desk, "zones", {}).values()] if hasattr(h.app, "order_desk") else []
    h._json(200, h.app.pinky_gateway.map_view(zones))


def post_stop(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 한 대 정지 요청"""
    pinky_id = parts[2]
    if pinky_id not in PINKY_IDS:
        h._json_error(404, "PINKY_UNKNOWN", f"핑키 {pinky_id}가 없습니다")
        return
    _answer(h, h.app.pinky_gateway.stop(pinky_id))


def post_action(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """주행준비·위치 재설정·Nav2 복구·웨이포인트 기록과 삭제를 핑키 관제로 전달"""
    gateway, pinky_id, action = h.app.pinky_gateway, parts[2], parts[3]
    try:
        if action == "ready":
            entry = gateway.prepare(pinky_id, body.get("map", "map-proto.yaml"))
        elif action == "initial-pose":
            entry = gateway.initial_pose(pinky_id)
        elif action == "nav-restart":
            entry = gateway.nav_restart(pinky_id)
        elif action == "waypoints" and len(parts) == 4:
            entry = gateway.record_waypoint(pinky_id, body.get("label"))
        else:  # /api/pinky/{id}/waypoints/{waypoint id}/delete
            entry = gateway.delete_waypoint(pinky_id, parts[4])
    except ValueError as exc:
        h._json_error(400, "PINKY_ACTION_INVALID", str(exc))
        return
    _answer(h, entry)


def post_stop_all(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 전체 정지 요청: 핑키마다 따로 보내고 결과를 모두 돌려줌"""
    results = h.app.pinky_gateway.stop_all()
    h._json(202 if all(entry["ok"] for entry in results) else 207, {"commands": results})


def post_credentials(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """SSH 비밀번호를 핑키 관제로 넘김"""
    if set(body) != {"password"}:
        h._json_error(400, "PASSWORD_REQUIRED", "password만 보내 주세요")
        return
    try:
        entry = h.app.pinky_gateway.send_password(body["password"])
    except ValueError as exc:
        h._json_error(400, "PASSWORD_REQUIRED", str(exc))
        return
    if entry["ok"]:
        h._json(200, {"command": entry})
    else:
        _answer(h, entry)
