from __future__ import annotations

import json
import tempfile
import time
import unittest
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from control_station.parking_check import (
    PIXEL_TRANSFORM_ID,
    detect_markers,
    estimate_marker_pose,
    sha256_of,
)
from control_station.parking_observation import (
    MAX_FRAMES,
    SPOT_DIR,
    ParkingObservationError,
    observe_parking,
)

CAMERA = np.array([[980.0, 0.0, 320.0], [0.0, 980.0, 240.0], [0.0, 0.0, 1.0]])
SCENE = {21: (420, 140), 1: (80, 60)}


def render_scene(markers: dict[int, tuple[int, int]], size: int = 60) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    render = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    for marker_id, (x, y) in markers.items():
        canvas[y:y + size, x:x + size] = render(dictionary, marker_id, size)
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def write_spot(root: Path, image: np.ndarray, **extra: object) -> None:
    intrinsics = root / "ceiling_camera.json"
    intrinsics.write_text(json.dumps({
        "camera_id": "ceiling_camera",
        "pixel_transform_id": PIXEL_TRANSFORM_ID,
        "camera_matrix": CAMERA.tolist(),
        "distortion_coefficients": [0.0] * 5,
        "image_size_px": {"width": 640, "height": 480},
    }), encoding="utf-8")
    found = detect_markers(image, "DICT_4X4_50")
    corners = found[21][0]
    plane = estimate_marker_pose(corners, 70.0, CAMERA, np.zeros(5))
    record = {
        "schema_version": 2,
        "spot_id": "dock",
        "camera_id": "ceiling_camera",
        "intrinsic_calibration": {"path": intrinsics.name, "sha256": sha256_of(intrinsics)},
        "aruco_dictionary": "DICT_4X4_50",
        "marker_id": 21,
        "marker_side_mm": 70.0,
        "reference_pose_in_camera": {"rotation": plane.rotation.tolist(), "translation_mm": plane.translation_mm.tolist()},
        "reference_corners_px": corners.tolist(),
        "anchors": [{"marker_id": 1, "center_px": found[1][0].mean(axis=0).tolist()}],
        "view_check": {"mode": "FIXED_MARKERS", "max_anchor_shift_px": 3.0},
        "tolerance": {"position_mm": 50.0, "yaw_deg": 10.0},
        **extra,
    }
    (root / SPOT_DIR).mkdir(parents=True)
    (root / SPOT_DIR / "dock.json").write_text(json.dumps(record), encoding="utf-8")


class FakeFeed:
    """Hands out one new frame per poll, captured 'now', like the camera worker's raw frame ring."""

    def __init__(self, image: np.ndarray, stale_frames: int = 0, limit: int | None = None) -> None:
        self.image = image
        self.sequence = 0
        self.limit = limit
        self.stale: list[tuple] = []
        for _ in range(stale_frames):  # frames already in the ring before the request
            self.stale.append(self._frame(time.monotonic() - 1.0))

    def _frame(self, captured_monotonic: float) -> tuple:
        self.sequence += 1
        height, width = self.image.shape[:2]
        pixels = self.image.tobytes()
        stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        return (self.sequence, "sha", pixels, width, height, stamp, captured_monotonic)

    def __call__(self, camera_id: str, after_sequence: int) -> list[tuple] | None:
        frames = [f for f in self.stale if f[0] > after_sequence]
        if self.limit is None or self.sequence < self.limit:
            frames.append(self._frame(time.monotonic()))
        return frames


class ObserveParkingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.image = render_scene(SCENE)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def observe(self, feed, spot_id: str = "dock", camera_id: str | None = "head_camera"):  # type: ignore[no-untyped-def]
        return observe_parking(spot_id, camera_id, feed, self.root, collect_seconds=1.0)

    def test_arrived_with_contract_fields(self) -> None:
        write_spot(self.root, self.image, marker_forward_axis="-x", opencv_version="4.6.0")
        observation = self.observe(FakeFeed(self.image))
        self.assertEqual(observation["opencv_version"], cv2.__version__)
        self.assertEqual(observation["registration_opencv_version"], "4.6.0")
        self.assertEqual(observation["status"], "ARRIVED")
        self.assertEqual(observation["deviation_level"], "WITHIN")
        self.assertEqual(observation["frames_used"], MAX_FRAMES)
        self.assertEqual(observation["view_check"], "ON")
        self.assertEqual(observation["feed_camera_id"], "head_camera")
        self.assertEqual(observation["calibration_camera_id"], "ceiling_camera")
        self.assertEqual(observation["spot_registration_sha256"], sha256_of(self.root / SPOT_DIR / "dock.json"))
        self.assertFalse(observation["usable_for_robot_control"])
        offset = observation["offset"]
        assert isinstance(offset, dict)
        self.assertIn("forward_mm", offset)
        self.assertAlmostEqual(offset["distance_mm"], 0.0, delta=0.5)

    def test_frames_from_before_the_request_are_ignored(self) -> None:
        write_spot(self.root, self.image)
        feed = FakeFeed(self.image, stale_frames=3)
        observation = self.observe(feed)
        first_used = observation["frame_sequence_range"][0]  # type: ignore[index]
        self.assertGreater(first_used, 3)

    def test_only_stale_frames_means_camera_unavailable(self) -> None:
        write_spot(self.root, self.image)
        with self.assertRaises(ParkingObservationError) as raised:
            self.observe(FakeFeed(self.image, stale_frames=3, limit=3))
        self.assertEqual(raised.exception.error_code, "PARKING_CAMERA_UNAVAILABLE")

    def test_too_few_usable_frames_is_not_judged(self) -> None:
        write_spot(self.root, self.image)
        observation = self.observe(FakeFeed(self.image, limit=3))
        self.assertEqual(observation["status"], "NOT_FOUND")
        self.assertIn("usable frames", observation["reason"])  # type: ignore[operator]
        self.assertIsNone(observation["offset"])

    def test_wrapped_view_is_not_judged(self) -> None:
        write_spot(self.root, self.image)
        observation = self.observe(FakeFeed(np.roll(self.image, 96, axis=1)))
        self.assertEqual(observation["status"], "VIEW_UNVERIFIED")
        self.assertAlmostEqual(observation["view_shift_px"], 96.0, delta=1.0)  # type: ignore[arg-type]
        self.assertIsNone(observation["deviation_level"])

    def test_resolution_mismatch(self) -> None:
        write_spot(self.root, self.image)
        with self.assertRaises(ParkingObservationError) as raised:
            self.observe(FakeFeed(cv2.resize(self.image, (320, 240))))
        self.assertEqual(raised.exception.error_code, "PARKING_CAMERA_UNAVAILABLE")

    def test_camera_not_configured(self) -> None:
        write_spot(self.root, self.image)
        with self.assertRaises(ParkingObservationError) as raised:
            self.observe(None, camera_id=None)
        self.assertEqual((raised.exception.http_status, raised.exception.error_code), (503, "PARKING_CAMERA_UNAVAILABLE"))

    def test_feed_missing(self) -> None:
        write_spot(self.root, self.image)
        with self.assertRaises(ParkingObservationError) as raised:
            self.observe(lambda camera_id, after: None)
        self.assertEqual(raised.exception.error_code, "PARKING_CAMERA_UNAVAILABLE")

    def test_spot_errors(self) -> None:
        for spot_id, status, code in (("../dock", 400, "PARKING_SPOT_ID_INVALID"), ("missing", 404, "PARKING_SPOT_NOT_FOUND")):
            with self.assertRaises(ParkingObservationError) as raised:
                self.observe(FakeFeed(self.image), spot_id=spot_id)
            self.assertEqual((raised.exception.http_status, raised.exception.error_code), (status, code))
        (self.root / SPOT_DIR).mkdir(parents=True)
        (self.root / SPOT_DIR / "old.json").write_text(json.dumps({"schema_version": 1}), encoding="utf-8")
        with self.assertRaises(ParkingObservationError) as raised:
            self.observe(FakeFeed(self.image), spot_id="old")
        self.assertEqual((raised.exception.http_status, raised.exception.error_code), (409, "PARKING_SPOT_INVALID"))


if __name__ == "__main__":
    unittest.main()
