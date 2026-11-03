from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from control_station.shelf_slot_map import load_shelf_slot_map
from control_station.shelf_slot_zones import ShelfSlotZones


def camera(**overrides: object) -> dict[str, object]:
    value: dict[str, object] = {
        "id": "shelf_camera",
        "frame_width": 640,
        "frame_height": 480,
        "pixel_transform_id": "opencv-v4l2-capture-no-app-geometry-v1",
        "detections": [
            {"class_name": "ball", "bbox_xyxy_px": [80.0, 150.0, 120.0, 200.0]},  # bottom-center (100, 200)
            {"class_name": "car", "bbox_xyxy_px": [180.0, 150.0, 220.0, 200.0]},  # on the shared edge x=200
            {"class_name": "box", "bbox_xyxy_px": [500.0, 400.0, 540.0, 450.0]},  # below every zone
            # Bottom-center (100, 120): inside the lower slot volume but above its anchor zone,
            # e.g. an object standing on the upper deck.
            {"class_name": "strawberry", "bbox_xyxy_px": [90.0, 100.0, 110.0, 120.0]},
        ],
    }
    value.update(overrides)
    return value


class ShelfSlotZoneTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.slot_map = load_shelf_slot_map()
        self.zones = ShelfSlotZones(self.slot_map, self.directory)

    def write_pose(self, **overrides: object) -> None:
        artifact = {
            "schema_version": 1,
            "status": "DRAFT_ONLY",
            "usable_for_robot_control": False,
            "camera_id": "shelf_camera",
            "pixel_transform_id": "opencv-v4l2-capture-no-app-geometry-v1",
            "image_size_px": {"width": 640, "height": 480},
            "sources": {"slot_map_sha256": self.slot_map.version},
            "reprojection_rms_px": 1.3,
            "pose": {"camera_position_in_shelf_frame_m": {"x": -1.16, "y": -0.02, "z": 0.38}},
            "slots": [
                {
                    "slot_id": "slot_a_2_1",
                    "polygon_px": [[0, 0], [200, 0], [200, 300], [0, 300]],
                    "anchor_polygon_px": [[0, 180], [200, 180], [200, 300], [0, 300]],
                },
                {
                    "slot_id": "slot_a_2_2",
                    "polygon_px": [[200, 0], [400, 0], [400, 300], [200, 300]],
                    "anchor_polygon_px": [[200, 180], [400, 180], [400, 300], [200, 300]],
                },
            ],
        }
        artifact.update(overrides)
        (self.directory / "shelf_camera.json").write_text(json.dumps(artifact), encoding="utf-8")

    def test_unconfigured_camera_is_left_unchanged(self) -> None:
        self.assertEqual(self.zones.annotate(camera()), camera())

    def test_detections_get_the_slot_their_bottom_center_falls_in(self) -> None:
        self.write_pose()
        result = self.zones.annotate(camera())
        self.assertEqual(result["slot_zones"]["status"], "READY")
        self.assertFalse(result["slot_zones"]["used_for_selection"])
        ball, car, box, upper = (item["slot_assignment"] for item in result["detections"])
        self.assertEqual((ball["status"], ball["slot_id"], ball["label"]), ("IN_SLOT", "slot_a_2_1", "왼쪽 선반 아래 단 좌"))
        self.assertEqual(ball["anchor_px"], [100.0, 200.0])
        self.assertEqual(car["status"], "AMBIGUOUS")
        self.assertEqual(sorted(car["candidates"]), ["slot_a_2_1", "slot_a_2_2"])
        self.assertEqual(box["status"], "OUTSIDE")
        self.assertEqual(upper["status"], "OUTSIDE")
        self.assertTrue(all(item["slot_assignment"]["used_for_selection"] is False for item in result["detections"]))

    def test_mismatched_pose_is_reported_instead_of_used(self) -> None:
        self.write_pose(sources={"slot_map_sha256": "0" * 64})
        result = self.zones.annotate(camera())
        self.assertEqual(result["slot_zones"]["status"], "SLOT_MAP_MISMATCH")
        self.assertNotIn("slot_assignment", result["detections"][0])

        self.write_pose(image_size_px={"width": 1280, "height": 720})
        self.assertEqual(self.zones.annotate(camera())["slot_zones"]["status"], "IMAGE_MISMATCH")

        self.write_pose(slots=[{"slot_id": "slot_b_1_1", "polygon_px": [[0, 0], [1, 0], [1, 1]], "anchor_polygon_px": [[0, 0], [1, 0], [1, 1]]}])
        self.assertEqual(self.zones.annotate(camera())["slot_zones"]["status"], "INVALID")

        # Artifacts from before anchor zones existed must be recalibrated, not silently used.
        self.write_pose(slots=[{"slot_id": "slot_a_2_1", "polygon_px": [[0, 0], [9, 0], [9, 9]]}])
        self.assertEqual(self.zones.annotate(camera())["slot_zones"]["status"], "INVALID")


if __name__ == "__main__":
    unittest.main()
