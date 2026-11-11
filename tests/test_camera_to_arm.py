from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

from control_station.arm_commands import ArmCommandStore
from control_station.camera_to_arm import CameraToArmError, load_camera_to_arm
from control_station.shelf_slot_map import load_shelf_slot_map
from control_station.store import TaskStore

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from omx_kinematics import forward, inverse  # noqa: E402

# Numbers measured on 2026-11-07 (front camera shelf position, fingertip in omx_link0) for two carrot touches.
POINTS = [
    {"camera": [0.2475, -0.1564], "arm": [0.230, 0.092, 0.004]},
    {"camera": [0.2547, -0.0472], "arm": [0.255, 0.183, 0.006]},
]


def calibration_file(directory: Path, theta: float, translation: list[float]) -> Path:
    path = directory / "camera-to-arm.json"
    path.write_text(json.dumps({
        "theta_rad": theta, "translation_m": translation, "carrot_touch_z_m": 0.005, "fingertip_offset_m": 0.065,
        "follower_board": "TEST", "points": POINTS,
    }), encoding="utf-8")
    return path


class CameraToArmTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.calibration = load_camera_to_arm(calibration_file(self.directory, -0.2017, [0.0169, 0.2878]))

    def test_measured_points_map_onto_the_touches_within_about_a_centimetre(self) -> None:
        for point in POINTS:
            x, y = self.calibration.to_arm(tuple(point["camera"]))
            self.assertLess(math.dist((x, y), point["arm"][:2]), 0.01)
        self.assertLess(self.calibration.max_fit_error_m, 0.01)

    def test_hover_target_puts_the_fingertip_above_and_slightly_in_front_of_the_object(self) -> None:
        target = self.calibration.hover_target({"x": 0.25, "y": -0.10})
        jaw = target["position"]
        angles = inverse((jaw["x"], jaw["y"], jaw["z"]), math.radians(45))
        points = forward(angles)
        direction = (points[-1] - points[-2]) / math.dist(points[-1], points[-2])
        fingertip = points[-1] + 0.065 * direction
        expected = target["fingertip_arm_m"]
        self.assertLess(math.dist(fingertip, (expected["x"], expected["y"], expected["z"])), 0.002)
        obj = target["object_arm_m"]
        self.assertAlmostEqual(expected["z"] - obj["z"], 0.06, places=3)
        self.assertAlmostEqual(math.hypot(obj["x"], obj["y"]) - math.hypot(expected["x"], expected["y"]), 0.02, places=3)

    def test_pick_target_is_the_object_centre_below_the_touch_height(self) -> None:
        target = self.calibration.object_center_target({"x": 0.2475, "y": -0.1564})
        self.assertEqual(target["approach"], "PITCH_DOWN_45")
        self.assertAlmostEqual(target["position"]["z"], 0.005 - 0.008, places=4)
        self.assertLess(math.dist((target["position"]["x"], target["position"]["y"]), POINTS[0]["arm"][:2]), 0.01)

    def test_pick_command_needs_a_position_and_a_place(self) -> None:
        store_dir = tempfile.TemporaryDirectory()
        self.addCleanup(store_dir.cleanup)
        task_store = TaskStore(str(Path(store_dir.name) / "tasks.sqlite3"))
        self.addCleanup(task_store.close)
        store = ArmCommandStore(task_store.session_factory, slot_map=load_shelf_slot_map())
        target = {"object_class": "carrot", **self.calibration.object_center_target({"x": 0.25, "y": -0.1})}
        request = {"robot_id": "arm-01", "operation": "PICK_AT_POSITION", "valid_for_s": 60, "target": target,
                   "source": {"kind": "OPERATOR_INPUT"}}
        with self.assertRaises(Exception):
            store.create(request)  # no place
        command = store.create({**request, "place": {"slot_id": "CART_1"}})
        self.assertEqual((command["operation"], command["place"]["slot_id"]), ("PICK_AT_POSITION", "CART_1"))

    def test_missing_or_broken_files(self) -> None:
        self.assertIsNone(load_camera_to_arm(""))
        broken = self.directory / "broken.json"
        broken.write_text("{}", encoding="utf-8")
        with self.assertRaises(CameraToArmError):
            load_camera_to_arm(broken)

    def test_position_command_keeps_the_camera_evidence(self) -> None:
        store_dir = tempfile.TemporaryDirectory()
        self.addCleanup(store_dir.cleanup)
        task_store = TaskStore(str(Path(store_dir.name) / "tasks.sqlite3"))
        self.addCleanup(task_store.close)
        store = ArmCommandStore(task_store.session_factory, slot_map=load_shelf_slot_map())
        hover = self.calibration.hover_target({"x": 0.25, "y": -0.10})
        command = store.create(
            {
                "robot_id": "arm-01", "operation": "MOVE_TO_POSITION", "valid_for_s": 60,
                "target": {"object_class": "carrot", **{key: hover[key] for key in ("frame_id", "unit", "position", "approach")}},
                "source": {"kind": "OPERATOR_INPUT"},
            },
            camera_source={"camera_id": "shelf_camera", "slot_id": "slot_b_2_1", "class_name": "carrot",
                           "position_shelf_m": {"x": 0.25, "y": -0.10, "z": 0.0828}},
        )
        self.assertEqual(command["target"]["approach"], "PITCH_DOWN_45")
        self.assertEqual(command["source"]["kind"], "CAMERA")
        self.assertEqual(command["target"]["object_position_shelf_m"]["y"], -0.10)


if __name__ == "__main__":
    unittest.main()
