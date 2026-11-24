"""핑키 화면 API: control_station.pinky.gateway.PinkyGateway 위의 얇은 HTTP 층.

핑키 세 대를 실제로 움직이는 것은 핑키 팀의 핑키 관제 대시보드(pinky_fleet_dashboard, ROS 2 Nav2)다. 관제는
게이트웨이를 거쳐 그 대시보드에 요청을 전달하고 결과를 기록할 뿐이다. 대시보드 URL이 없으면(모의 모드) 아무것도
보내지 않는다.

GET  /api/pinky                 핑키 카드 전부, 핑키 관제가 응답했는지, 최근 명령
GET  /api/pinky/map             위에서 본 작업장(지도 좌표, m): Nav2 격자, 금지 구역, 고정 지점, 핑키 위치
POST /api/pinky/{id}/stop       핑키 한 대 정지 (대시보드가 정지를 걸어 두고 Nav2 목표를 취소함)
POST /api/pinky/stop-all        모든 핑키에 정지 요청
POST /api/pinky/prepare-all {map}  아직 준비 안 된 핑키를 차례로 주행준비, 각자 주차장 자리에서 위치 잡기
POST /api/pinky/credentials     {password}: 대시보드로 넘길 뿐 여기서는 저장하거나 로그에 남기지 않음
POST /api/pinky/{id}/ready {map}, /initial-pose, /nav-restart, /waypoints {label}, /waypoints/{wid}/delete
POST /api/pinky/{id}/localize {pose?}  라이다 위치 잡기 (자동, 또는 고른 후보 위치)
POST /api/pinky/{id}/set-pose {pose}  위치 다시 잡기: 운영자가 지도에 찍은 위치 (라이다가 그 근처에서 다듬음)
GET  /api/pinky/{id}/locate     읽기 전용: 지금 라이다가 찾은 후보 위치들
POST /api/pinky/{id}/test-mission {dropoff}  주문 없는 시운전: 5번 적재 → 내려놓는 곳 → 주차장 (정류장 사이는 Nav2)
POST /api/pinky/{id}/route-clear   기존 경로 날리기 (실행 중인 경로는 먼저 멈춤)
POST /api/pinky/{id}/return-home {via?}  지금 자기 주차장 자리로 주행
POST /api/pinky/{id}/reset {map}   초기 세팅: 정지 유지, 경로 지우기, 주행준비를 처음부터
POST /api/pinky/{id}/signal {action}  지금 임무 단계에 맞는 dock_complete | received | mission_complete 신호
                                      (redock은 천장 캠의 offset {forward_mm, left_mm, dyaw_deg}를 함께 보낼 수 있음)
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.storage.store import PINKY_IDS

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def _answer(h: ControlStationHandler, entry: dict[str, Any]) -> None:
    """게이트웨이 명령 기록(entry)을 HTTP 응답으로 바꾼다.

    대시보드가 받았으면 202(대시보드가 다른 2xx를 줬으면 그 코드), 거절했으면 대시보드의 거절 코드(400 잘못된 요청,
    401 비밀번호 없음, 409 바쁨)를 그대로, 그 밖의 실패(연결 실패 등)는 502다. 모의 모드라 보내지 않은 명령은 409다.
    """
    status = (202 if entry["http_status"] in {None, 202} else entry["http_status"]) if entry["ok"] else (
        entry["http_status"] if entry["http_status"] in {400, 401, 409} else 502)
    if entry.get("simulated"):
        status = 409
    h._json(status, {"command": entry})


def get_pinky(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 화면: 핑키 3대 상태와 최근 명령, 적재 자리 도착 자동 처리(dock_auto) 켜짐 여부"""
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
    # 수령 구역은 주문 데스크가 갖고 있다. 데스크가 없으면(서버 조립 전 등) 구역 없이 그린다.
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
    """핑키 한 대에 대한 동작을 핑키 관제로 전달한다: 주행준비, 위치 재설정·위치 잡기, 시운전, 경로 지우기,
    주차장 복귀, 초기 세팅, 임무 단계 신호, Nav2 재시작, 웨이포인트 기록과 삭제.

    어느 동작인지는 parts[3](경로의 마지막 조각)로 정한다. 인자 검증은 게이트웨이가 하고 ValueError면 400이다.
    map을 보내지 않으면 기본 지도 map-proto.yaml을 쓴다."""
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
                # 주문이 이 핑키에 물건을 싣는 중이다. 여기서 내보내면 팔 앞의 적재 자리가 비게 된다.
                # 다시 맞추기(redock)는 주문 흐름이 카메라 판정에 따라 직접 보낸다.
                h._json_error(409, "PINKY_IN_ORDER", "주문이 이 핑키를 쓰는 중이라 여기서 다시 들어오게 할 수 없어요")
                return
            entry = gateway.signal(pinky_id, body.get("action"), body.get("offset"))
        elif action == "nav-restart":
            entry = gateway.nav_restart(pinky_id)
        elif action == "waypoints" and len(parts) == 4:
            entry = gateway.record_waypoint(pinky_id, body.get("label"))
        else:  # /api/pinky/{id}/waypoints/{웨이포인트 id}/delete (routes.py가 이 모양만 여기로 보낸다)
            entry = gateway.delete_waypoint(pinky_id, parts[4])
    except ValueError as exc:
        h._json_error(400, "PINKY_ACTION_INVALID", str(exc))
        return
    _answer(h, entry)


def post_dock_handshake(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """적재 자리에 선 핑키를 천장 캠으로 한 번 판정하고, auto면 그 결과대로 다시 맞추기·도킹완료를 보냄.

    본문은 {auto?, repeat?}다. repeat은 진입 반복 시험의 회차 번호이고, 정수가 아니면 0으로 본다.
    판정과 신호 전송은 pinky/docking.py의 DockSupervisor.step이 한다."""
    pinky_id = parts[2]
    if pinky_id not in PINKY_IDS:
        h._json_error(404, "PINKY_UNKNOWN", f"핑키 {pinky_id}가 없습니다")
        return
    try:
        repeat = body.get("repeat")
        repeat = repeat if isinstance(repeat, int) and not isinstance(repeat, bool) else 0
        # 관제 자체의 자동 처리(dock_auto)가 켜져 있으면, 화면의 체크 상자와 관계없이 이 호출도 실제로 신호를 보낸다.
        result = h.app.dock_supervisor.step(pinky_id, body.get("auto") is True or bool(getattr(h.app, "dock_auto", False)), repeat)
    except Exception as exc:  # 핑키 관제 대시보드가 답하지 않음
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
