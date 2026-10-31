from __future__ import annotations

import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "simulation" / "mujoco_omx"))

from omx_mujoco import ARM_JOINTS, CONTROL_JOINTS, OMXSimulation, PICK_PLACE_SCENE  # noqa: E402
from omx_mujoco.camera import CAMERA_NAMES, CameraRig  # noqa: E402
from omx_mujoco.perception import detect_strawberry  # noqa: E402
from omx_mujoco.simulation import GRIPPER_CLOSED_RAD, GRIPPER_OPEN_RAD  # noqa: E402
from omx_mujoco.simulation import REPOSITORY_ROOT  # noqa: E402


class OMXSimulationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.simulation = OMXSimulation(PICK_PLACE_SCENE)

    def test_official_joint_mapping_and_camera_attachments_load(self) -> None:
        self.assertEqual(len(ARM_JOINTS), 5)
        self.assertEqual(len(CONTROL_JOINTS), 6)
        self.assertEqual(set(self.simulation.joint_positions()), set(CONTROL_JOINTS))
        mujoco = self.simulation.mujoco
        wrist_id = mujoco.mj_name2id(
            self.simulation.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera"
        )
        link6_id = mujoco.mj_name2id(
            self.simulation.model, mujoco.mjtObj.mjOBJ_BODY, "link6"
        )
        self.assertGreaterEqual(wrist_id, 0)
        self.assertEqual(int(self.simulation.model.cam_bodyid[wrist_id]), link6_id)
        for camera_name in CAMERA_NAMES:
            camera_id = mujoco.mj_name2id(
                self.simulation.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
            )
            self.assertGreaterEqual(camera_id, 0)

    def test_pick_place_scene_has_no_initial_robot_collision(self) -> None:
        self.assertEqual(
            self.simulation.data.ncon,
            0,
            "the default OMX pose should start clear of the table and objects",
        )

    def test_joint_target_moves_smoothly_and_rejects_out_of_range(self) -> None:
        result = self.simulation.move_joint_targets({"Joint1": 0.2}, duration_s=0.5)
        self.assertAlmostEqual(result["Joint1"], 0.2, delta=0.02)
        with self.assertRaisesRegex(ValueError, "outside model range"):
            self.simulation.set_joint_targets({"Joint2": 99.0})

    def test_gripper_position_and_mimic_joint_move_together(self) -> None:
        self.simulation.move_joint_targets({"Gripper": GRIPPER_OPEN_RAD}, duration_s=0.4)
        state = self.simulation.joint_positions(include_mimic=True)
        self.assertAlmostEqual(state["Gripper"], GRIPPER_OPEN_RAD, delta=0.03)
        self.assertAlmostEqual(state["Gripper_mimic"], -GRIPPER_OPEN_RAD, delta=0.03)
        self.simulation.move_joint_targets({"Gripper": GRIPPER_CLOSED_RAD}, duration_s=0.4)
        self.assertAlmostEqual(
            self.simulation.joint_positions()["Gripper"], GRIPPER_CLOSED_RAD, delta=0.03
        )

    def test_dynamic_objects_settle_under_gravity_on_table(self) -> None:
        self.simulation.step(1000)
        mujoco = self.simulation.mujoco
        cube_id = mujoco.mj_name2id(
            self.simulation.model, mujoco.mjtObj.mjOBJ_BODY, "pick_place_cube"
        )
        box_id = mujoco.mj_name2id(
            self.simulation.model, mujoco.mjtObj.mjOBJ_BODY, "pick_place_box"
        )
        cube_z = float(self.simulation.data.xpos[cube_id, 2])
        box_z = float(self.simulation.data.xpos[box_id, 2])
        self.assertAlmostEqual(cube_z, 0.325, delta=0.01)
        self.assertAlmostEqual(box_z, 0.335, delta=0.01)

    def test_fixed_and_wrist_rgb_camera_render(self) -> None:
        with CameraRig(self.simulation, width=96, height=72) as cameras:
            for camera_name in CAMERA_NAMES:
                frame = cameras.render_rgb(camera_name)
                self.assertEqual(frame.shape, (72, 96, 3))
                self.assertEqual(str(frame.dtype), "uint8")
                self.assertGreater(int(frame.max()), int(frame.min()))


class StrawberrySimulationTests(unittest.TestCase):
    def test_default_photo_match_scene_keeps_static_strawberry_and_two_shelves(self) -> None:
        simulation = OMXSimulation()
        self.assertEqual(simulation.scene_path.name, "shelf_photo_match_scene.xml")
        for geom_name in ("photo_match_lower_deck", "photo_match_upper_deck"):
            geom_id = simulation.mujoco.mj_name2id(
                simulation.model, simulation.mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            self.assertGreaterEqual(geom_id, 0)
        strawberry_id = simulation.mujoco.mj_name2id(
            simulation.model,
            simulation.mujoco.mjtObj.mjOBJ_BODY,
            "photo_match_strawberry",
        )
        self.assertEqual(int(simulation.model.body_jntnum[strawberry_id]), 0)

    def setUp(self) -> None:
        scene = (
            REPOSITORY_ROOT
            / "simulation"
            / "mujoco_omx"
            / "robotis_omx"
            / "strawberry_pick_scene.xml"
        )
        self.simulation = OMXSimulation(scene)

    def test_fixed_camera_rgbd_localizes_dynamic_strawberry(self) -> None:
        self.simulation.step(500)
        observation = detect_strawberry(self.simulation)
        self.assertIsNotNone(observation)
        strawberry_id = self.simulation.mujoco.mj_name2id(
            self.simulation.model,
            self.simulation.mujoco.mjtObj.mjOBJ_BODY,
            "photo_match_strawberry",
        )
        actual = self.simulation.data.xpos[strawberry_id]
        estimated = observation.estimated_center_world_m
        error_m = sum((float(actual[i]) - estimated[i]) ** 2 for i in range(3)) ** 0.5
        self.assertGreater(observation.red_pixel_count, 12)
        self.assertGreater(observation.median_depth_m, 0.0)
        self.assertLess(error_m, 0.01, f"RGB-D center estimate error was {error_m:.4f} m")

    def test_dynamic_strawberry_remains_on_second_shelf(self) -> None:
        self.simulation.step(2000)
        strawberry_id = self.simulation.mujoco.mj_name2id(
            self.simulation.model,
            self.simulation.mujoco.mjtObj.mjOBJ_BODY,
            "photo_match_strawberry",
        )
        position = self.simulation.data.xpos[strawberry_id]
        self.assertAlmostEqual(float(position[0]), 0.10, delta=0.005)
        self.assertAlmostEqual(float(position[1]), 0.21, delta=0.005)
        self.assertAlmostEqual(float(position[2]), 0.219, delta=0.005)


if __name__ == "__main__":
    unittest.main()
