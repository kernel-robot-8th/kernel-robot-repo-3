"""Pinky screen API: a thin HTTP layer over control_station.pinky_gateway.PinkyGateway.

GET  /api/pinky                 every Pinky card, whether the fleet dashboard answered, recent commands
GET  /api/pinky/map             the cell from above in map metres: Nav2 grid, keepout, fixed points, poses
POST /api/pinky/{id}/stop       stop one Pinky (the dashboard latches a stop and cancels its Nav2 goal)
POST /api/pinky/stop-all        stop request to every Pinky
POST /api/pinky/prepare-all {map}  주행준비 for every Pinky not yet prepared, in turn, each localized at its parking spot
POST /api/pinky/credentials     {password}: passed to the dashboard, never stored or logged here
POST /api/pinky/{id}/ready {map}, /initial-pose, /nav-restart, /waypoints {label}, /waypoints/{wid}/delete
POST /api/pinky/{id}/localize {pose?}  LiDAR localization (automatic, or the picked candidate pose)
POST /api/pinky/{id}/set-pose {pose}  위치 다시 잡기: the pose the operator put on the map (LiDAR polishes it nearby)
GET  /api/pinky/{id}/locate     read only: the LiDAR's candidate places now
POST /api/pinky/{id}/test-mission {dropoff}  시운전 without an order: 5번 적재 → drop-off → 주차장 (Nav2 between stops)
POST /api/pinky/{id}/route-clear   기존 경로 날리기 (stops a running route first)
POST /api/pinky/{id}/return-home {via?}  drive to its own parking spot now
POST /api/pinky/{id}/reset {map}   초기 세팅: hold stopped, drop the route, 주행준비 from the start
POST /api/pinky/{id}/signal {action}  dock_complete | received | mission_complete for the current mission phase
                                      (redock may carry offset {forward_mm, left_mm, dyaw_deg} from the ceiling camera)
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
    h._json(200, {**h.app.pinky_gateway.view(), "dock_auto": bool(getattr(h.app, "dock_auto", False))})


def get_locate(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """라이다 위치 후보 (읽기만, AMCL은 그대로)"""
    try:
        answer = h.app.pinky_gateway.locate(parts[2])
    except ValueError as exc:
        h._json_error(404, "PINKY_UNKNOWN", str(exc))
        return
    if answer["ok"]:
        h._json(200, answer)
    else:
        h._json_error(409, "PINKY_LOCATE_UNAVAILABLE", answer["error"])


def get_ros(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """ROS 모니터: 핑키 관제가 받는 토픽 전부와 상태 (개발자 화면)"""
    try:
        h._json(200, h.app.pinky_gateway.ros_view(parts[2]))
    except ValueError as exc:
        h._json_error(404, "PINKY_UNKNOWN", str(exc))


def get_nav_log(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 Nav2 로그의 오류 줄과 마지막 줄 (개발자 화면, 읽기만)"""
    try:
        h._json(200, h.app.pinky_gateway.nav_log(parts[2]))
    except ValueError as exc:
        h._json_error(404, "PINKY_UNKNOWN", str(exc))


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
        elif action == "localize":
            entry = gateway.localize(pinky_id, body.get("pose"))
        elif action == "set-pose":
            entry = gateway.set_pose(pinky_id, body.get("pose"))
        elif action == "test-mission":
            entry = gateway.test_mission(pinky_id, body.get("dropoff"), body.get("via"))
        elif action == "route-clear":
            entry = gateway.clear_route(pinky_id)
        elif action == "return-home":
            entry = gateway.return_home(pinky_id, body.get("via"))
        elif action == "reset":
            entry = gateway.reset(pinky_id, body.get("map", "map-proto.yaml"))
        elif action == "signal":
            flow = getattr(h.app, "order_flow", None)
            if body.get("action") == "redock" and flow is not None and flow.holds_pinky(pinky_id):
                # An order is loading this Pinky: sending it out from here would leave the arm an empty spot. The
                # order's own flow redocks it when the camera says so.
                h._json_error(409, "PINKY_IN_ORDER", "주문이 이 핑키를 쓰는 중이라 여기서 다시 들어오게 할 수 없어요")
                return
            entry = gateway.signal(pinky_id, body.get("action"), body.get("offset"))
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


def post_dock_handshake(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """적재 자리에 선 핑키를 천장 캠으로 한 번 판정하고, auto면 그 결과대로 다시 맞추기·도킹완료를 보냄"""
    pinky_id = parts[2]
    if pinky_id not in PINKY_IDS:
        h._json_error(404, "PINKY_UNKNOWN", f"핑키 {pinky_id}가 없습니다")
        return
    try:
        repeat = body.get("repeat")
        repeat = repeat if isinstance(repeat, int) and not isinstance(repeat, bool) else 0
        # With the station's own automatic handling on, this call acts too, whatever the screen's box says.
        result = h.app.dock_supervisor.step(pinky_id, body.get("auto") is True or bool(getattr(h.app, "dock_auto", False)), repeat)
    except Exception as exc:  # the fleet dashboard did not answer
        h._json_error(502, "PINKY_FLEET_UNREACHABLE", f"핑키 상태를 읽지 못했습니다: {exc}")
        return
    h._json(200, result)


def post_dock_auto(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """적재 자리 도착 자동 처리 켜기·끄기: {auto: true|false}. 끄면 카메라는 판정만 하고 사람이 누름"""
    if set(body) != {"auto"} or not isinstance(body["auto"], bool):
        h._json_error(400, "PINKY_ACTION_INVALID", "auto(true 또는 false)만 보내 주세요")
        return
    h.app.dock_auto = body["auto"]
    h._json(200, {"dock_auto": h.app.dock_auto})


def post_prepare_all(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """주행준비가 안 된 핑키를 차례로 준비하고 각자 주차장 좌표로 위치 잡기"""
    try:
        entry = h.app.pinky_gateway.prepare_all(body.get("map", "map-proto.yaml"))
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
