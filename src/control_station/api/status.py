"""서버 상태, 로봇 상태, 주차 판정"""

from __future__ import annotations

from typing import TYPE_CHECKING

from control_station.parking_ros import ParkingObservationError

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


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
            },
        },
    )


def get_robots(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """로봇 상태"""
    h._json(200, {"robots": h.app.robot_provider.snapshots()})


def get_parking_live(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주차 실시간 상태 (화면 표시용)"""
    spot_id = route.removeprefix("/api/parking-spots/").removesuffix("/live")
    tracker = h.app.parking_tracker
    if tracker is None or tracker.spot_id != spot_id:
        h._json_error(
            404,
            "PARKING_TRACKING_NOT_CONFIGURED",
            "이 주차 위치는 실시간 추적 중이 아닙니다 (CONTROL_STATION_PARKING_SPOT)",
        )
        return
    h._json(200, {"live": tracker.snapshot()})


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
