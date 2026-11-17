"""The policy client command built by the supervisor (no process is started)."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from control_station.arms import ArmRegistry
from control_station.policy_run_supervisor import PolicyRunSupervisor

LEFT_SERIAL = "F58AE68650323952582E3120FF05260E"
RIGHT_SERIAL = "A9F9354350323952582E3120FF070727"
ARMS = ArmRegistry((
    {"arm_id": "arm-01", "label": "왼쪽 팔", "shelf_id": "shelf_a", "stock_camera_id": "shelf_camera", "board_serial": LEFT_SERIAL},
    {"arm_id": "arm-02", "label": "오른쪽 팔", "shelf_id": "shelf_b", "stock_camera_id": "shelf_camera", "board_serial": RIGHT_SERIAL},
))


def policy(policy_id: str, object_class: str, arm_id: str | None, front: str = "OVERHEAD") -> dict:
    return {
        "policy_id": policy_id, "object_class": object_class, "label": object_class, "aliases": [], "model": f"team/{policy_id}",
        "policy_type": "act", "task": "Pick", "cameras": {"front": front, "wrist": "GRIPPER" if front == "OVERHEAD" else "OVERHEAD"},
        "arm_id": arm_id, "max_run_s": 30,
    }


class PolicyCommandTest(unittest.TestCase):
    def supervisor(self, **env: str) -> PolicyRunSupervisor:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        policies = Path(directory.name) / "policies.json"
        policies.write_text(json.dumps({"schema_version": 1, "policies": [
            policy("any_act", "carrot", None),
            policy("left_act", "apple", "arm-01", front="GRIPPER"),
            policy("right_act", "tissue", "arm-02"),
        ]}))
        settings = {
            "CONTROL_STATION_LEROBOT_DIR": directory.name,
            "CONTROL_STATION_LEROBOT_PYTHON": sys.executable,
            "CONTROL_STATION_ACT_POLICIES": str(policies),
            "CONTROL_STATION_POLICY_OVERHEAD_CAMERA": "/dev/overhead",
            "CONTROL_STATION_POLICY_GRIPPER_CAMERA": "/dev/gripper",
            "CONTROL_STATION_POLICY_MAX_ACTION_STEPS": "300",
            **env,
        }
        cleared = {name: "" for name in os.environ if name.startswith("CONTROL_STATION_")}
        with mock.patch.dict(os.environ, {**cleared, **settings}):
            return PolicyRunSupervisor(disabled_reason="테스트에서는 정책을 실행하지 않습니다", arms=ARMS)

    @staticmethod
    def argument(command: list[str], name: str) -> str:
        return next(part for part in command if part.startswith(f"--{name}=")).split("=", 1)[1]

    def test_profiles_come_from_the_policy_file_with_their_arm(self) -> None:
        profiles = self.supervisor().policy_profiles
        self.assertEqual({key: profile["arm_id"] for key, profile in profiles.items()},
                         {"any_act": None, "left_act": "arm-01", "right_act": "arm-02"})

    def test_client_keeps_torque_and_exits_without_a_plot_window(self) -> None:
        supervisor = self.supervisor()
        command = supervisor._command(supervisor.policy_profiles["any_act"])
        self.assertIn("--robot.disable_torque_on_disconnect=false", command)
        self.assertIn("--debug_visualize_queue_size=false", command)

    def test_a_policy_without_arm_runs_on_the_single_arm_settings(self) -> None:
        supervisor = self.supervisor(CONTROL_STATION_POLICY_ROBOT_PORT="/dev/omx_single")
        command = supervisor._command(supervisor.policy_profiles["any_act"])
        self.assertEqual(self.argument(command, "robot.port"), "/dev/omx_single")
        cameras = self.argument(command, "robot.cameras")
        self.assertLess(cameras.index("/dev/overhead"), cameras.index("/dev/gripper"))  # front = overhead

    def test_an_arm_bound_policy_runs_on_its_own_arm(self) -> None:
        supervisor = self.supervisor(
            CONTROL_STATION_ARM01_FOLLOWER_PORT="/dev/left_port", CONTROL_STATION_ARM01_ROBOT_ID="left_arm",
            CONTROL_STATION_ARM01_GRIPPER_CAMERA="/dev/left_gripper", CONTROL_STATION_ARM01_OVERHEAD_CAMERA="/dev/left_top",
        )
        command = supervisor._command(supervisor.policy_profiles["left_act"])
        self.assertEqual((self.argument(command, "robot.port"), self.argument(command, "robot.id")), ("/dev/left_port", "left_arm"))
        cameras = self.argument(command, "robot.cameras")
        self.assertLess(cameras.index("/dev/left_gripper"), cameras.index("/dev/left_top"))  # recorded front = gripper
        self.assertNotIn("/dev/overhead", cameras)
        self.assertEqual(supervisor.devices_for("left_act"), ["/dev/left_top", "/dev/left_gripper"])

    def test_an_arm_bound_policy_never_borrows_another_arms_port(self) -> None:
        # The single-arm port is the right arm's board: it does not count for the left arm's policy.
        supervisor = self.supervisor(
            CONTROL_STATION_FOLLOWER_PORT=f"/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_{RIGHT_SERIAL}-if00")
        with self.assertRaisesRegex(RuntimeError, "CONTROL_STATION_ARM01_FOLLOWER_PORT"):
            supervisor._command(supervisor.policy_profiles["left_act"])
        self.assertEqual(supervisor.devices_for("left_act"), [])
        # ... but it does for the right arm, which is the board it names.
        command = supervisor._command(supervisor.policy_profiles["right_act"])
        self.assertIn(RIGHT_SERIAL, self.argument(command, "robot.port"))
        arms = {arm["arm_id"]: arm for arm in supervisor.snapshot()["arms"]}
        self.assertEqual((arms["arm-01"]["ready"], arms["arm-02"]["source"]), (False, "legacy"))


if __name__ == "__main__":
    unittest.main()
