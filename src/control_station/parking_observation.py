"""Read-only Pinky parking observation over a control-station camera feed.

Follows the "Pinky 주차 정렬 관측" contract draft in docs/spec/interface-contracts.md: only frames captured
after the request are judged, and the result is an observation, never a robot command.
"""

from __future__ import annotations

import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from control_station.parking_check import (
    ParkingJudge,
    ParkingResult,
    ParkingSpotError,
    ParkingStatus,
    load_spot,
    observe_frame,
    pinky_frame_offset,
    sha256_of,
)

OBSERVATION_SCHEMA_VERSION = 1
SPOT_DIR = Path(".control-station") / "parking-spots"
SPOT_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}")
MAX_FRAMES = 15
MIN_FRAMES = 5
COLLECT_SECONDS = 3.0
POLL_SECONDS = 0.01

# (sequence, sha256, BGR pixels, width, height, captured_at, captured_monotonic)
RawFrame = tuple[int, str, bytes, int, int, str, float]
FramesAfter = Callable[[str, int], "list[RawFrame] | None"]


class ParkingObservationError(Exception):
    def __init__(self, http_status: int, error_code: str, message: str) -> None:
        super().__init__(message)
        self.http_status = http_status
        self.error_code = error_code
        self.message = message


def observe_parking(
    spot_id: str,
    feed_camera_id: str | None,
    frames_after: FramesAfter | None,
    repository_root: Path,
    collect_seconds: float = COLLECT_SECONDS,
) -> dict[str, object]:
    if not SPOT_ID_PATTERN.fullmatch(spot_id):
        raise ParkingObservationError(400, "PARKING_SPOT_ID_INVALID", "spot_id must be 1-80 letters, digits, '_', '.' or '-'")
    path = repository_root / SPOT_DIR / f"{spot_id}.json"
    if not path.is_file():
        raise ParkingObservationError(404, "PARKING_SPOT_NOT_FOUND", f"등록된 주차 위치가 없습니다: {spot_id}")
    try:
        registration_sha256 = sha256_of(path)
        spot = load_spot(path, repository_root)
    except (ParkingSpotError, ValueError, KeyError, TypeError, OSError) as exc:
        raise ParkingObservationError(409, "PARKING_SPOT_INVALID", f"주차 위치 파일을 쓸 수 없습니다: {exc}") from exc
    if feed_camera_id is None or frames_after is None:
        raise ParkingObservationError(
            503, "PARKING_CAMERA_UNAVAILABLE", "주차 판정 카메라가 설정되지 않았습니다 (CONTROL_STATION_PARKING_CAMERA)"
        )

    requested_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    requested_monotonic = time.monotonic()
    deadline = requested_monotonic + collect_seconds
    judgment = ParkingJudge(spot.tolerance, MAX_FRAMES)
    result = judgment.result
    last_sequence = 0
    used: list[RawFrame] = []
    while len(judgment.recent) < MAX_FRAMES and time.monotonic() < deadline:
        frames = frames_after(feed_camera_id, last_sequence)
        if frames is None:
            raise ParkingObservationError(503, "PARKING_CAMERA_UNAVAILABLE", f"카메라 feed가 없습니다: {feed_camera_id}")
        for frame in frames:
            sequence, _, pixels, width, height, _, captured_monotonic = frame
            last_sequence = max(last_sequence, sequence)
            if captured_monotonic < requested_monotonic:  # 요청 전에 찍힌 프레임은 쓰지 않는다
                continue
            if (width, height) != (spot.intrinsics.width, spot.intrinsics.height):
                raise ParkingObservationError(
                    503, "PARKING_CAMERA_UNAVAILABLE",
                    f"프레임 해상도 {width}x{height}가 보정 해상도 {spot.intrinsics.width}x{spot.intrinsics.height}와 다릅니다",
                )
            image = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 3)
            result = judgment.add(observe_frame(image, spot))
            used.append(frame)
            if len(judgment.recent) >= MAX_FRAMES:
                break
        time.sleep(POLL_SECONDS)
    if not used:
        raise ParkingObservationError(
            503, "PARKING_CAMERA_UNAVAILABLE", f"{collect_seconds:g}초 안에 새 프레임이 없습니다: {feed_camera_id}"
        )
    if result.offset is not None and len(judgment.recent) < MIN_FRAMES:
        result = ParkingResult(
            ParkingStatus.NOT_FOUND, None, f"only {len(judgment.recent)} usable frames (< {MIN_FRAMES})", result.view_shift_px
        )

    offset: dict[str, object] | None = None
    if result.offset is not None:
        offset = {
            "frame": "REGISTERED_SPOT_MARKER_AXES",
            "dx_mm": round(result.offset.dx_mm, 1),
            "dy_mm": round(result.offset.dy_mm, 1),
            "distance_mm": round(result.offset.distance_mm, 1),
            "dyaw_deg": round(result.offset.dyaw_deg, 2),
            "marker_forward_axis": spot.marker_forward_axis,
        }
        if spot.marker_forward_axis is not None:
            forward, left = pinky_frame_offset(result.offset, spot.marker_forward_axis)
            offset.update(forward_mm=round(forward, 1), left_mm=round(left, 1))
    return {
        "observation_schema_version": OBSERVATION_SCHEMA_VERSION,
        "spot_id": spot.spot_id,
        "marker_id": spot.marker_id,
        "status": result.status.value,
        "reason": result.reason,
        "offset": offset,
        "tolerance": {"position_mm": spot.tolerance.position_mm, "yaw_deg": spot.tolerance.yaw_deg},
        "position_ratio": None if result.position_ratio is None else round(result.position_ratio, 2),
        "yaw_ratio": None if result.yaw_ratio is None else round(result.yaw_ratio, 2),
        "deviation_level": result.deviation_level.value if result.deviation_level else None,
        "view_check": "ON" if spot.anchors else "OFF",
        "view_shift_px": None if result.view_shift_px is None else round(result.view_shift_px, 1),
        "frames_used": len(judgment.recent),
        "frames_examined": len(used),
        "feed_camera_id": feed_camera_id,
        "calibration_camera_id": spot.camera_id,
        "frame_sequence_range": [used[0][0], used[-1][0]],
        "captured_from": used[0][5],
        "captured_until": used[-1][5],
        "requested_at": requested_at,
        "spot_registration_sha256": registration_sha256,
        "intrinsic_calibration_sha256": spot.intrinsics.sha256,
        "opencv_version": cv2.__version__,
        "registration_opencv_version": spot.registration_opencv_version,
        "usable_for_robot_control": False,
    }
