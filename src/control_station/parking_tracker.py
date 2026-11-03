"""Continuous, display-only Pinky parking state from the ceiling camera feed.

Shows the operator how Pinky comes in and settles: NOT_IN_VIEW -> MOVING -> IN_POSITION / OFF_POSITION.
It never gates the arm step. The arrival gate still judges only frames captured after the arrival
result (parking_observation.observe_parking), so a frame where a moving Pinky briefly passes through
the tolerance cannot count as an arrival.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import replace
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path

import numpy as np

from control_station.parking_check import (
    ParkingResult,
    ParkingSpot,
    ParkingSpotError,
    ParkingStatus,
    PlanarOffset,
    judge,
    load_spot,
    median_offset,
    observe_frame,
)
from control_station.parking_observation import SPOT_DIR, FramesAfter, image_overlay, judgment_fields

TRACK_INTERVAL_S = 0.1  # judge at most 10 frames per second; the feed itself runs near 30 fps
STILL_WINDOW_S = 1.0  # Pinky counts as stopped when every frame of the last second agrees within:
# Single raw frames of a parked Pinky scatter about ±1.5 mm and ±0.55° (2026-11-02, OpenCV 4.14);
# a Pinky that is actually driving moves tens of mm within the window.
STILL_POSITION_MM = 5.0
STILL_YAW_DEG = 2.0
MIN_STILL_FRAMES = 5
LOST_AFTER_S = 1.0  # marker unseen this long -> NOT_IN_VIEW
STALE_FEED_S = 2.0  # no new frame this long -> CAMERA_UNAVAILABLE
SPOT_RELOAD_S = 5.0


class LiveState(str, Enum):
    NOT_IN_VIEW = "NOT_IN_VIEW"
    MOVING = "MOVING"
    IN_POSITION = "IN_POSITION"
    OFF_POSITION = "OFF_POSITION"
    AMBIGUOUS = "AMBIGUOUS"
    VIEW_UNVERIFIED = "VIEW_UNVERIFIED"
    CAMERA_UNAVAILABLE = "CAMERA_UNAVAILABLE"
    SPOT_INVALID = "SPOT_INVALID"
    TRACKING_ERROR = "TRACKING_ERROR"  # a software fault in the tracker, not a camera or registration problem


class ParkingTracker:
    def __init__(
        self,
        spot_id: str,
        feed_camera_id: str,
        frames_after: FramesAfter,
        repository_root: Path,
        clock=time.monotonic,  # type: ignore[no-untyped-def]
    ) -> None:
        self.spot_id = spot_id
        self.feed_camera_id = feed_camera_id
        self._frames_after = frames_after
        self._path = repository_root / SPOT_DIR / f"{spot_id}.json"
        self._root = repository_root
        self._clock = clock
        self._spot: ParkingSpot | None = None
        self._source_mtimes: tuple[float, float | None] | None = None
        self._spot_checked = -SPOT_RELOAD_S
        self._last_sequence = 0
        self._started_at: float | None = None
        self._last_frame_at: float | None = None
        self._samples: deque[tuple[float, PlanarOffset]] = deque()
        self._last_seen: float | None = None
        self._marker_corners = None
        self._view_shift_xy = None
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._thread: threading.Thread | None = None
        self._snapshot: dict[str, object] = self._state(LiveState.NOT_IN_VIEW, "아직 프레임을 받지 않았습니다")

    def start(self) -> "ParkingTracker":
        self._thread = threading.Thread(target=self._run, name=f"parking-tracker-{self.spot_id}", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._closed.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return dict(self._snapshot)

    def _run(self) -> None:
        while not self._closed.is_set():
            try:
                self.tick()
            except Exception as exc:  # keep tracking; one bad frame must not stop the display
                self._reset_samples()
                self._publish(self._state(LiveState.TRACKING_ERROR, f"추적 오류: {exc!r}"))
            self._closed.wait(TRACK_INTERVAL_S)

    def tick(self) -> None:
        now = self._clock()
        if self._started_at is None:
            self._started_at = now
        if not self._reload_spot(now):
            return
        spot = self._spot
        assert spot is not None
        frames = self._frames_after(self.feed_camera_id, self._last_sequence)
        if frames is None:
            self._publish(self._state(LiveState.CAMERA_UNAVAILABLE, f"카메라 feed가 없습니다: {self.feed_camera_id}"))
            return
        if not frames:  # the feed may still be opening right after start-up
            if now - (self._started_at if self._last_frame_at is None else self._last_frame_at) > STALE_FEED_S:
                self._publish(self._state(LiveState.CAMERA_UNAVAILABLE, "새 프레임이 들어오지 않습니다"))
            return
        sequence, _, pixels, width, height, captured_at, _ = frames[-1]  # only the newest frame matters for display
        self._last_sequence = sequence
        self._last_frame_at = now
        if (width, height) != (spot.intrinsics.width, spot.intrinsics.height):
            self._publish(self._state(LiveState.CAMERA_UNAVAILABLE, f"프레임 해상도 {width}x{height}가 보정 해상도와 다릅니다"))
            return
        observation = observe_frame(np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 3), spot)
        if observation.blocking_status is not None:
            # Earlier frames no longer describe the current view, so nothing from them may be judged or drawn.
            self._reset_samples()
            state = LiveState.AMBIGUOUS if observation.blocking_status == ParkingStatus.AMBIGUOUS else LiveState.VIEW_UNVERIFIED
            self._publish(self._state(state, observation.reason, observation=observation, sequence=sequence, captured_at=captured_at))
            return
        if observation.offset is not None:  # only judged frames feed the picture
            self._last_seen = now
            self._samples.append((now, observation.offset))
            self._marker_corners = observation.corners
            self._view_shift_xy = observation.view_shift_xy_px
        while self._samples and now - self._samples[0][0] > STILL_WINDOW_S:
            self._samples.popleft()
        if not self._samples or self._last_seen is None or now - self._last_seen > LOST_AFTER_S:
            self._reset_samples()
            state = LiveState.VIEW_UNVERIFIED if observation.skipped_status == ParkingStatus.VIEW_UNVERIFIED else LiveState.NOT_IN_VIEW
            self._publish(self._state(state, observation.reason, sequence=sequence, captured_at=captured_at))
            return
        offsets = [offset for _, offset in self._samples]
        spread = _spread(offsets)
        still = (
            len(offsets) >= MIN_STILL_FRAMES
            and now - self._samples[0][0] >= 0.8 * STILL_WINDOW_S
            and max(o.distance_mm for o in spread) <= STILL_POSITION_MM
            and max(abs(o.dyaw_deg) for o in spread) <= STILL_YAW_DEG
        )
        if still:
            result = judge(median_offset(offsets), spot.tolerance)
            state = LiveState.IN_POSITION if result.status == ParkingStatus.ARRIVED else LiveState.OFF_POSITION
            reason = "1초 이상 멈춰 있음"
        else:
            result = judge(offsets[-1], spot.tolerance)
            state, reason = LiveState.MOVING, "움직이는 중"
        result = replace(result, view_shift_px=observation.view_shift_px)
        self._publish(self._state(state, reason, judged=result, sequence=sequence, captured_at=captured_at))

    def _reset_samples(self) -> None:
        self._samples.clear()
        self._last_seen = None
        self._marker_corners = None
        self._view_shift_xy = None

    def _mtimes(self) -> tuple[float, float | None]:
        intrinsics = None if self._spot is None else self._spot.intrinsics.path
        return self._path.stat().st_mtime, None if intrinsics is None else intrinsics.stat().st_mtime

    def _reload_spot(self, now: float) -> bool:
        if now - self._spot_checked < SPOT_RELOAD_S:  # also while invalid, so a bad file is not re-read every tick
            return self._spot is not None
        self._spot_checked = now
        try:
            # A re-registration, an edited tolerance or a recalibrated camera all reload the spot. A changed
            # calibration then fails the SHA-256 check in load_spot, as the one-shot check does.
            if self._spot is None or self._mtimes() != self._source_mtimes:
                self._spot = load_spot(self._path, self._root)
                self._source_mtimes = self._mtimes()
                self._reset_samples()
        except (ParkingSpotError, ValueError, KeyError, TypeError, OSError) as exc:
            self._spot = None
            self._source_mtimes = None
            self._reset_samples()
            self._publish(self._state(LiveState.SPOT_INVALID, f"주차 위치 파일을 쓸 수 없습니다: {exc}"))
            return False
        return True

    def _state(self, state: LiveState, reason: str, judged=None, observation=None, sequence=None, captured_at=None):  # type: ignore[no-untyped-def]
        spot = self._spot
        payload: dict[str, object] = {
            "state": state.value,
            "reason": reason,
            "spot_id": self.spot_id,
            "feed_camera_id": self.feed_camera_id,
            "frame_sequence": sequence,
            "captured_at": captured_at,
            "updated_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "display_only": True,
            "usable_for_robot_control": False,
        }
        if spot is not None:
            if judged is None:  # same contract fields as a judged state, with no offset
                shift = None if observation is None else observation.view_shift_px
                judged = ParkingResult(ParkingStatus.NOT_FOUND, None, reason, shift)
            payload.update(judgment_fields(judged, spot))
            payload["image_overlay"] = image_overlay(spot, self._marker_corners, self._view_shift_xy)
        return payload

    def _publish(self, payload: dict[str, object]) -> None:
        with self._lock:
            if payload["state"] != self._snapshot.get("state"):
                payload["since"] = payload["updated_at"]
            else:
                payload["since"] = self._snapshot.get("since", payload["updated_at"])
            self._snapshot = payload


def _spread(offsets: list[PlanarOffset]) -> list[PlanarOffset]:
    """Each offset relative to the window median, to tell a stopped Pinky from a moving one."""
    center = median_offset(offsets)
    return [
        PlanarOffset(o.dx_mm - center.dx_mm, o.dy_mm - center.dy_mm, (o.dyaw_deg - center.dyaw_deg + 180.0) % 360.0 - 180.0)
        for o in offsets
    ]
