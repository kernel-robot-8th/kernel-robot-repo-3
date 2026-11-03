from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from calibrate_shelf_pose import build_artifact  # noqa: E402


SLOT_MAP_PATH = REPOSITORY_ROOT / "config" / "shelf-slots.json"
# Pinhole intrinsics in the range of the 640x480 shelf camera; the real draft is not in the repository.
INTRINSICS = {
    "camera_id": "shelf_camera",
    "calibration_id": "test-intrinsics",
    "pixel_transform_id": "opencv-v4l2-capture-no-app-geometry-v1",
    "image_size_px": {"width": 640, "height": 480},
    "camera_matrix": [[973.0, 0.0, 320.0], [0.0, 973.0, 240.0], [0.0, 0.0, 1.0]],
    "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
}


def camera_pose(position: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """Camera in front of the shelves looking into them: image right = -y, image down = -z, optical = +x."""
    rotation = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    tvec = -rotation @ np.array(position)
    return cv2.Rodrigues(rotation)[0], tvec.reshape(3, 1)


class ShelfPoseTests(unittest.TestCase):
    def setUp(self) -> None:
        raw = SLOT_MAP_PATH.read_bytes()
        self.slot_map = json.loads(raw)
        self.slot_sha = hashlib.sha256(raw).hexdigest()
        self.image = np.zeros((480, 640, 3), np.uint8)
        self.keypoints = {item["keypoint_id"]: item for item in self.slot_map["keypoints"]}

    def clicks(self, ids: list[str], position: tuple[float, float, float], noise_px: float = 0.0) -> dict:
        rvec, tvec = camera_pose(position)
        points = np.array([[self.keypoints[i]["point_m"][a] for a in "xyz"] for i in ids])
        pixels, _ = cv2.projectPoints(points, rvec, tvec, np.array(INTRINSICS["camera_matrix"]), np.zeros(5))
        pixels = pixels.reshape(-1, 2) + np.random.default_rng(0).normal(0, noise_px, (len(ids), 2))
        return {
            "schema_version": 1,
            "camera_id": "shelf_camera",
            "source_image_sha256": "img",
            "source_image_size_px": {"width": 640, "height": 480},
            "slot_map_sha256": self.slot_sha,
            "points": [{"keypoint_id": i, "image_px": [float(x), float(y)]} for i, (x, y) in zip(ids, pixels)],
        }

    def run_pose(self, points: dict) -> dict:
        artifact, overlay = build_artifact(
            points, "pts", self.image, "img", INTRINSICS, "intr", self.slot_map, self.slot_sha, 0.10
        )
        self.assertEqual(overlay.shape, self.image.shape)
        return artifact

    def test_recovers_known_camera_position_and_projects_six_slots(self) -> None:
        ids = [
            "seam_top_front", "shelf_a_upper_lip_bottom_right", "right_end_top_front", "shelf_a_upper_lip_top_left",
            "shelf_a_lower_lip_top_left", "shelf_b_lower_lip_bottom_right", "left_end_top_front",
        ]
        artifact = self.run_pose(self.clicks(ids, (-0.9, 0.02, 0.22)))
        position = artifact["pose"]["camera_position_in_shelf_frame_m"]
        self.assertAlmostEqual(position["x"], -0.9, places=3)
        self.assertAlmostEqual(position["y"], 0.02, places=3)
        self.assertAlmostEqual(position["z"], 0.22, places=3)
        self.assertLess(artifact["reprojection_rms_px"], 0.01)
        self.assertEqual(len(artifact["slots"]), 6)
        polygons = {slot["slot_id"]: np.array(slot["polygon_px"]) for slot in artifact["slots"]}
        # Seen from the front, the left shelf is on the image left and the upper tier is above the lower one.
        self.assertLess(polygons["slot_a_2_1"][:, 0].mean(), polygons["slot_b_2_2"][:, 0].mean())
        self.assertLess(polygons["slot_a_1_1"][:, 1].mean(), polygons["slot_a_2_1"][:, 1].mean())
        self.assertEqual(artifact["status"], "DRAFT_ONLY")
        self.assertFalse(artifact["usable_for_robot_control"])

    def test_committed_map_carries_the_measured_lower_deck(self) -> None:
        self.assertEqual(self.slot_map["measured_adjustments"][0]["shift_m"], -0.035)
        lower = next(slot for slot in self.slot_map["slots"] if slot["slot_id"] == "slot_a_2_1")
        self.assertAlmostEqual(lower["deck_height_m"], 0.0828, places=4)
        self.assertAlmostEqual(self.keypoints["shelf_a_lower_lip_top_left"]["point_m"]["z"], 0.10, places=4)
        self.assertAlmostEqual(self.keypoints["shelf_a_upper_lip_top_left"]["point_m"]["z"], 0.269, places=4)

    def test_click_noise_shows_up_as_residuals_not_a_failure(self) -> None:
        ids = [item for item in self.keypoints if not item.endswith("bottom_front")]
        artifact = self.run_pose(self.clicks(ids, (-0.8, -0.05, 0.3), noise_px=1.0))
        self.assertGreater(artifact["reprojection_rms_px"], 0.1)
        position = artifact["pose"]["camera_position_in_shelf_frame_m"]
        self.assertAlmostEqual(position["x"], -0.8, delta=0.03)
        self.assertTrue(all(point["leave_one_out_error_px"] is not None for point in artifact["points"]))

    def test_rejects_too_few_points_or_mismatched_sources(self) -> None:
        few = self.clicks(["seam_top_front", "shelf_a_lower_lip_top_right", "left_end_top_front"], (-0.9, 0.0, 0.2))
        with self.assertRaisesRegex(ValueError, "at least 6"):
            self.run_pose(few)
        points = self.clicks(list(self.keypoints)[:8], (-0.9, 0.0, 0.2))
        with self.assertRaisesRegex(ValueError, "different slot map"):
            self.run_pose({**points, "slot_map_sha256": "other"})
        with self.assertRaisesRegex(ValueError, "different image"):
            self.run_pose({**points, "source_image_sha256": "other"})


if __name__ == "__main__":
    unittest.main()
