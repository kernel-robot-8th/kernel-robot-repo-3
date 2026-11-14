"""Each arm's devices on this PC: per-arm settings, and the older single-arm settings for the arm they name."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from control_station.arm_devices import arm_devices, env_prefix, legacy_arm_id
from control_station.arms import ArmConfigError, load_arms

LEFT_SERIAL = "F58AE68650323952582E3120FF05260E"
RIGHT_SERIAL = "A9F9354350323952582E3120FF070727"
ARMS = (
    {"arm_id": "arm-01", "label": "왼쪽 팔", "board_serial": LEFT_SERIAL},
    {"arm_id": "arm-02", "label": "오른쪽 팔", "board_serial": RIGHT_SERIAL},
)
RIGHT_PORT = f"/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_{RIGHT_SERIAL}-if00"


class ArmDevicesTest(unittest.TestCase):
    def test_env_names(self) -> None:
        self.assertEqual(env_prefix("arm-02"), "CONTROL_STATION_ARM02_")

    def test_single_arm_settings_belong_to_the_arm_whose_board_they_name(self) -> None:
        env = {"CONTROL_STATION_FOLLOWER_PORT": RIGHT_PORT, "CONTROL_STATION_JOINT_CAMERA": "/dev/joint",
               "CONTROL_STATION_ACT_OVERHEAD_CAMERA": "/dev/top"}
        self.assertEqual(legacy_arm_id(ARMS, env), "arm-02")
        devices = arm_devices(ARMS, env)
        self.assertEqual((devices["arm-02"].source, devices["arm-02"].cameras), ("legacy", {"OVERHEAD": "/dev/top", "GRIPPER": "/dev/joint"}))
        self.assertTrue(devices["arm-02"].ready)
        left = devices["arm-01"]
        self.assertEqual((left.source, left.ready), ("none", False))
        self.assertEqual(left.missing, ["CONTROL_STATION_ARM01_FOLLOWER_PORT", "CONTROL_STATION_ARM01_GRIPPER_CAMERA",
                                        "CONTROL_STATION_ARM01_OVERHEAD_CAMERA"])

    def test_a_port_without_a_board_serial_names_no_arm(self) -> None:
        env = {"CONTROL_STATION_POLICY_ROBOT_PORT": "/dev/omx_follower", "CONTROL_STATION_JOINT_CAMERA": "/dev/joint"}
        self.assertIsNone(legacy_arm_id(ARMS, env))
        self.assertEqual({devices.source for devices in arm_devices(ARMS, env).values()}, {"none"})

    def test_per_arm_settings_replace_the_single_arm_ones(self) -> None:
        env = {"CONTROL_STATION_FOLLOWER_PORT": RIGHT_PORT, "CONTROL_STATION_JOINT_CAMERA": "/dev/joint",
               "CONTROL_STATION_ARM02_GRIPPER_CAMERA": "/dev/right_gripper", "CONTROL_STATION_ARM02_ROBOT_ID": "right"}
        right = arm_devices(ARMS, env)["arm-02"]
        self.assertEqual((right.source, right.follower_port, right.robot_id), ("arm", None, "right"))
        self.assertEqual(right.cameras, {"GRIPPER": "/dev/right_gripper", "OVERHEAD": None})
        self.assertEqual(right.missing, ["CONTROL_STATION_ARM02_FOLLOWER_PORT", "CONTROL_STATION_ARM02_OVERHEAD_CAMERA"])

    def test_arm_file_takes_an_optional_board_serial(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "arms.json"
            arm = {"arm_id": "arm-01", "label": "왼쪽 팔", "shelf_id": "shelf_a", "stock_camera_id": "shelf_camera"}
            path.write_text(json.dumps({"schema_version": 1, "arms": [arm]}))
            self.assertIsNone(load_arms(path).arms[0]["board_serial"])
            path.write_text(json.dumps({"schema_version": 1, "arms": [{**arm, "board_serial": "f58a"}]}))
            with self.assertRaises(ArmConfigError):
                load_arms(path)

    def test_repository_arms_name_both_boards(self) -> None:
        arms = {arm["arm_id"]: arm["board_serial"] for arm in load_arms().arms}
        self.assertEqual(arms, {"arm-01": LEFT_SERIAL, "arm-02": RIGHT_SERIAL})


if __name__ == "__main__":
    unittest.main()
