from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from control_station.shelf_slot_zones import ShelfSlotZones, pixel_to_shelf_plane

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from omx_lateral_correction import TICKS_PER_TURN, load_base, pan_correction  # noqa: E402

CAMERA_MATRIX = np.array([[973.0, 0.0, 203.0], [0.0, 972.6, 277.0], [0.0, 0.0, 1.0]])
DISTORTION = np.array([-0.38, 0.06, 0.0, 0.0, 0.05])


def front_camera(position: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """Camera in front of the shelves, tilted 12 degrees down: image right = -y, image down = -z."""
    tilt = math.radians(12)
    level = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    pitch = np.array([[1, 0, 0], [0, math.cos(tilt), -math.sin(tilt)], [0, math.sin(tilt), math.cos(tilt)]])
    rotation = pitch @ level
    return rotation, (-rotation @ np.array(position)).reshape(3, 1)


class BackProjectionTests(unittest.TestCase):
    def setUp(self) -> None:
        rotation, tvec = front_camera((-1.165, -0.025, 0.383))
        self.projection = {"camera_matrix": CAMERA_MATRIX, "distortion": DISTORTION, "rotation": rotation, "tvec": tvec}
        self.rvec = cv2.Rodrigues(rotation)[0]

    def pixel_of(self, point: tuple[float, float, float]) -> tuple[float, float]:
        pixel, _ = cv2.projectPoints(np.array([point]), self.rvec, self.projection["tvec"], CAMERA_MATRIX, DISTORTION)
        return tuple(pixel.reshape(2))

    def test_pixel_on_a_deck_comes_back_to_its_shelf_point(self) -> None:
        for point in ((0.04, 0.30, 0.083), (0.07, -0.12, 0.083), (0.05, 0.10, 0.251)):
            back = pixel_to_shelf_plane(self.pixel_of(point), self.projection, point[2])
            self.assertAlmostEqual(back["x"], point[0], delta=0.001)
            self.assertAlmostEqual(back["y"], point[1], delta=0.001)

    def test_in_slot_detection_gets_shelf_point_and_offset_from_slot_centre(self) -> None:
        slot = {"slot_id": "slot_a_2_2", "label": "왼쪽 선반 아래 단 우", "polygon_px": [[0, 0], [640, 0], [640, 480], [0, 480]],
                "deck_height_m": 0.083, "marker_m": {"x": 0.0445, "y": 0.105, "z": 0.083}}
        u, v = self.pixel_of((0.0445, 0.145, 0.083))  # 4 cm left of the slot centre line
        self.assertTrue(0 <= u < 640 and 0 <= v < 480)
        detection = {"bbox_xyxy_px": [u - 10, v - 40, u + 10, v]}
        assignment = ShelfSlotZones.assign(detection, [slot], self.projection)
        self.assertEqual(assignment["status"], "IN_SLOT")
        self.assertAlmostEqual(assignment["lateral_offset_m"], 0.04, delta=0.001)
        self.assertIsNone(ShelfSlotZones.assign(detection, [slot]).get("lateral_offset_m"))


class PanCorrectionTests(unittest.TestCase):
    base = {"pan_pivot_shelf_m": {"x": -0.20, "y": 0.30}, "pan_sign": 1}
    marker = {"x": 0.0445, "y": 0.305, "z": 0.083}

    def test_offset_turns_the_shoulder_by_the_angle_seen_from_the_pivot(self) -> None:
        ticks, degrees = pan_correction(self.base, self.marker, 0.04)
        expected = math.degrees(math.atan2(0.345 - 0.30, 0.2445) - math.atan2(0.305 - 0.30, 0.2445))
        self.assertAlmostEqual(degrees, expected, places=6)
        self.assertEqual(ticks, round(math.radians(expected) * TICKS_PER_TURN / (2 * math.pi)))
        self.assertGreater(degrees, 0)  # object to the left (+y) turns toward +y
        self.assertEqual(pan_correction(self.base, self.marker, 0.0), (0, 0.0))

    def test_pan_sign_flips_only_the_motor_direction(self) -> None:
        flipped = {**self.base, "pan_sign": -1}
        ticks, degrees = pan_correction(flipped, self.marker, 0.04)
        self.assertEqual(ticks, -pan_correction(self.base, self.marker, 0.04)[0])
        self.assertGreater(degrees, 0)

    def test_base_file_needs_measured_pivot_and_checked_sign(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "omx-base.json"
            path.write_text(json.dumps({"pan_pivot_shelf_m": {"x": None, "y": 0.3}, "pan_sign": 1}), encoding="utf-8")
            with self.assertRaises(SystemExit):
                load_base(path)
            path.write_text(json.dumps({"pan_pivot_shelf_m": {"x": -0.2, "y": 0.3}, "pan_sign": 0}), encoding="utf-8")
            with self.assertRaises(SystemExit):
                load_base(path)
            path.write_text(json.dumps(self.base), encoding="utf-8")
            self.assertEqual(load_base(path)["pan_sign"], 1)


if __name__ == "__main__":
    unittest.main()
