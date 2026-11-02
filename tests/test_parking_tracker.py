from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from control_station.parking_check import PIXEL_TRANSFORM_ID, detect_markers, estimate_marker_pose, sha256_of
from control_station.parking_observation import SPOT_DIR
from control_station.parking_tracker import LiveState, ParkingTracker

CAMERA = np.array([[980.0, 0.0, 320.0], [0.0, 980.0, 240.0], [0.0, 0.0, 1.0]])
PINKY_AT = (420, 140)
ANCHOR_AT = (80, 60)


def render(pinky: tuple[int, int] | None, anchor: bool = True) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    for marker_id, at in ((21, pinky), (1, ANCHOR_AT if anchor else None)):
        if at is not None:
            canvas[at[1]:at[1] + 60, at[0]:at[0] + 60] = draw(dictionary, marker_id, 60)
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def write_spot(root: Path, tolerance_mm: float = 50.0) -> Path:
    intrinsics = root / "ceiling_camera.json"
    intrinsics.write_text(json.dumps({
        "camera_id": "ceiling_camera", "pixel_transform_id": PIXEL_TRANSFORM_ID, "camera_matrix": CAMERA.tolist(),
        "distortion_coefficients": [0.0] * 5, "image_size_px": {"width": 640, "height": 480},
    }), encoding="utf-8")
    found = detect_markers(render(PINKY_AT), "DICT_4X4_50")
    plane = estimate_marker_pose(found[21][0], 70.0, CAMERA, np.zeros(5))
    (root / SPOT_DIR).mkdir(parents=True, exist_ok=True)
    path = root / SPOT_DIR / "dock.json"
    path.write_text(json.dumps({
        "schema_version": 2, "spot_id": "dock", "camera_id": "ceiling_camera",
        "intrinsic_calibration": {"path": intrinsics.name, "sha256": sha256_of(intrinsics)},
        "aruco_dictionary": "DICT_4X4_50", "marker_id": 21, "marker_side_mm": 70.0,
        "reference_pose_in_camera": {"rotation": plane.rotation.tolist(), "translation_mm": plane.translation_mm.tolist()},
        "reference_corners_px": found[21][0].tolist(),
        "anchors": [{"marker_id": 1, "center_px": found[1][0].mean(axis=0).tolist()}],
        "view_check": {"mode": "FIXED_MARKERS"},
        "tolerance": {"position_mm": tolerance_mm, "yaw_deg": 10.0},
        "marker_forward_axis": "-x",
    }), encoding="utf-8")
    return path


class Rig:
    """A tracker on a fake clock whose feed returns whatever image is set, one new frame per tick."""

    def __init__(self, root: Path) -> None:
        self.now = 0.0
        self.sequence = 0
        self.image: np.ndarray | None = render(None)
        self.tracker = ParkingTracker("dock", "head_camera", self.frames_after, root, clock=lambda: self.now)

    def frames_after(self, camera_id: str, after: int):  # type: ignore[no-untyped-def]
        if self.image is None:
            return None
        self.sequence += 1
        height, width = self.image.shape[:2]
        return [(self.sequence, "sha", self.image.tobytes(), width, height, "2026-11-02T00:00:00.000+00:00", self.now)]

    def run(self, image: np.ndarray | None, seconds: float) -> dict:
        self.image = image
        for _ in range(int(round(seconds / 0.1))):
            self.now += 0.1
            self.tracker.tick()
        return self.tracker.snapshot()


class ParkingTrackerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        write_spot(self.root)
        self.rig = Rig(self.root)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_not_in_view_then_moving_then_in_position(self) -> None:
        self.assertEqual(self.rig.run(render(None), 0.5)["state"], LiveState.NOT_IN_VIEW.value)
        for step in range(5):  # driving in, 4 px per frame
            moving = self.rig.run(render((PINKY_AT[0], PINKY_AT[1] + 20 - 4 * step)), 0.1)
        self.assertEqual(moving["state"], LiveState.MOVING.value)
        settled = self.rig.run(render(PINKY_AT), 1.2)
        self.assertEqual(settled["state"], LiveState.IN_POSITION.value)
        self.assertEqual(settled["deviation_level"], "WITHIN")
        self.assertIn("forward_mm", settled["offset"])
        self.assertTrue(settled["display_only"])
        self.assertIsNotNone(settled["image_overlay"]["marker_corners_px"])

    def test_overlay_carries_the_camera_drift(self) -> None:
        drifted = self.rig.run(np.roll(render(PINKY_AT), (5, -2), axis=(0, 1)), 1.2)
        self.assertEqual(drifted["state"], LiveState.IN_POSITION.value)
        np.testing.assert_allclose(drifted["image_overlay"]["view_shift_xy_px"], [-2.0, 5.0], atol=0.3)

    def test_stopped_outside_tolerance_is_off_position(self) -> None:
        stopped = self.rig.run(render((PINKY_AT[0] + 60, PINKY_AT[1])), 1.2)
        self.assertEqual(stopped["state"], LiveState.OFF_POSITION.value)
        self.assertIn(stopped["deviation_level"], ("NEAR", "FAR"))

    def test_since_keeps_the_time_the_state_began(self) -> None:
        first = self.rig.run(render(PINKY_AT), 1.2)
        later = self.rig.run(render(PINKY_AT), 0.5)
        self.assertEqual(first["since"], later["since"])

    def test_wrapped_view(self) -> None:
        state = self.rig.run(np.roll(render(PINKY_AT), 96, axis=1), 0.3)
        self.assertEqual(state["state"], LiveState.VIEW_UNVERIFIED.value)
        self.assertGreater(state["view_shift_px"], 90)

    def test_marker_lost(self) -> None:
        self.rig.run(render(PINKY_AT), 1.2)
        self.assertEqual(self.rig.run(render(None), 1.5)["state"], LiveState.NOT_IN_VIEW.value)

    def test_camera_unavailable(self) -> None:
        self.assertEqual(self.rig.run(None, 0.2)["state"], LiveState.CAMERA_UNAVAILABLE.value)

    def test_no_frames_yet_waits_before_reporting_the_camera(self) -> None:
        self.rig.frames_after = lambda camera_id, after: []  # type: ignore[method-assign]
        self.rig.tracker._frames_after = self.rig.frames_after
        self.assertEqual(self.rig.run(render(PINKY_AT), 1.0)["state"], LiveState.NOT_IN_VIEW.value)
        self.assertEqual(self.rig.run(render(PINKY_AT), 1.5)["state"], LiveState.CAMERA_UNAVAILABLE.value)

    def test_spot_file_change_is_picked_up(self) -> None:
        self.assertEqual(self.rig.run(render((PINKY_AT[0] + 60, PINKY_AT[1])), 1.2)["state"], LiveState.OFF_POSITION.value)
        path = write_spot(self.root, tolerance_mm=500.0)
        stamp = path.stat().st_mtime + 10
        os.utime(path, (stamp, stamp))
        self.assertEqual(self.rig.run(render((PINKY_AT[0] + 60, PINKY_AT[1])), 6.5)["state"], LiveState.IN_POSITION.value)

    def test_spot_missing(self) -> None:
        (self.root / SPOT_DIR / "dock.json").unlink()
        self.assertEqual(Rig(self.root).run(render(PINKY_AT), 0.2)["state"], LiveState.SPOT_INVALID.value)


if __name__ == "__main__":
    unittest.main()
