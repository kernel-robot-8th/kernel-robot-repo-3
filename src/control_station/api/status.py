"""서버·로봇 상태, 주차 판정, 전체 정지 API."""

from __future__ import annotations

from typing import TYPE_CHECKING

from control_station.pinky.parking_ros import ParkingObservationError
from control_station.stop_all import stop_everything

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


# ──── 서버·로봇 상태, 주차 판정 ─────────────────────────────────────────────────────────────────────────────────────

def get_health(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """서버 상태와 주차 설정"""
    h._json(
        200,
        {
            "status": "ok",
            "mode": h.app.mode,
            "parking": {
                "camera_id": h.app.parking_camera_id,
                "spot_id": h.app.parking_spot_id,
                "live_tracking": h.app.parking_tracker is not None,
                # every spot followed live: the primary one and each Pinky's own registration of the loading spot
                "live_spots": list(_live_spots(h.app.parking_tracker)),
            },
        },
    )


def _live_spots(tracker: object | None) -> tuple[str, ...]:
    if tracker is None:
        return ()
    return tuple(getattr(tracker, "live_spots", None) or (tracker.spot_id,))


def get_robots(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """로봇 상태"""
    h._json(200, {"robots": h.app.robot_provider.snapshots()})


def get_parking_live(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주차 실시간 상태 (화면 표시용)"""
    spot_id = route.removeprefix("/api/parking-spots/").removesuffix("/live")
    tracker = h.app.parking_tracker
    if spot_id not in _live_spots(tracker):
        h._json_error(
            404,
            "PARKING_TRACKING_NOT_CONFIGURED",
            "이 주차 위치는 실시간 추적 중이 아닙니다 (CONTROL_STATION_PARKING_SPOT, config/pinky-markers.json)",
        )
        return
    h._json(200, {"live": tracker.snapshot() if spot_id == tracker.spot_id else tracker.snapshot(spot_id)})


def get_parking_check(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주차 확정 판정 (요청 이후 프레임)"""
    spot_id = route.removeprefix("/api/parking-spots/").removesuffix("/check")
    if h.app.parking_check is None:
        h._json_error(
            503,
            "PARKING_NODE_UNAVAILABLE",
            "주차 판정 노드가 설정되지 않았습니다 (CONTROL_STATION_PARKING_SPOT)",
        )
        return
    try:
        observation = h.app.parking_check(spot_id)
    except ParkingObservationError as exc:
        h._json_error(exc.http_status, exc.error_code, exc.message)
        return
    h._json(200, {"observation": observation})


# ──── 전체 정지 ───────────────────────────────────────────────────────────────────────────────────────────────

def post_stop_all(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """전체 정지 요청: 팔 명령·시퀀스·정책 실행을 취소하고 핑키마다 정지를 보낸 뒤 결과를 모두 돌려줌.
    202 = 보낸 요청이 모두 접수됨, 207 = 일부가 거절됐거나 보내지 못함(모의 모드 포함). 접수는 멈춤 확인이 아님."""
    result = stop_everything(h.app)
    h._json(202 if result["ok"] else 207, result)
