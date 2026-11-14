"""Pinky screen API: a thin HTTP layer over control_station.pinky_gateway.PinkyGateway.

GET  /api/pinky                 every Pinky card, whether the fleet dashboard answered, recent commands
POST /api/pinky/{id}/stop       stop one Pinky (the dashboard latches a stop and cancels its Nav2 goal)
POST /api/pinky/stop-all        stop request to every Pinky
POST /api/pinky/credentials     {password}: passed to the dashboard, never stored or logged here
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.store import PINKY_IDS

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def _answer(h: ControlStationHandler, entry: dict[str, Any]) -> None:
    """202 when the dashboard accepted it; otherwise its own refusal status (401 no password, 409 busy) or 502."""
    status = 202 if entry["ok"] else (entry["http_status"] if entry["http_status"] in {400, 401, 409} else 502)
    if entry.get("simulated"):
        status = 409
    h._json(status, {"command": entry})


def get_pinky(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 화면: 핑키 3대 상태와 최근 명령"""
    h._json(200, h.app.pinky_gateway.view())


def post_stop(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """핑키 한 대 정지 요청"""
    pinky_id = parts[2]
    if pinky_id not in PINKY_IDS:
        h._json_error(404, "PINKY_UNKNOWN", f"핑키 {pinky_id}가 없습니다")
        return
    _answer(h, h.app.pinky_gateway.stop(pinky_id))


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
