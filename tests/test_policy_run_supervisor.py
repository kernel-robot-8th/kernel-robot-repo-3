"""The policy client command built by the supervisor (no process is started)."""

from __future__ import annotations

import os
import sys
import tempfile
import unittest
from unittest import mock

from control_station.policy_run_supervisor import PolicyRunSupervisor


class PolicyCommandTest(unittest.TestCase):
    def supervisor(self) -> PolicyRunSupervisor:
        lerobot_dir = tempfile.TemporaryDirectory()
        self.addCleanup(lerobot_dir.cleanup)
        env = {
            "CONTROL_STATION_LEROBOT_DIR": lerobot_dir.name,
            "CONTROL_STATION_LEROBOT_PYTHON": sys.executable,
            "CONTROL_STATION_POLICY_OVERHEAD_CAMERA": "/dev/overhead",
            "CONTROL_STATION_POLICY_GRIPPER_CAMERA": "/dev/gripper",
            "CONTROL_STATION_POLICY_MAX_ACTION_STEPS": "300",
        }
        with mock.patch.dict(os.environ, env):
            return PolicyRunSupervisor(disabled_reason="테스트에서는 정책을 실행하지 않습니다")

    def test_profiles_come_from_the_order_flow_policy_list(self) -> None:
        profiles = self.supervisor().policy_profiles
        self.assertEqual(profiles["apple_act_20261109"]["model"], "neverlish/omx_act_record-apple-20261109-1")
        self.assertEqual(set(profiles), {"carrot_act_v2", "tissue_act_v1", "apple_act_20261109"})

    def test_client_keeps_torque_and_exits_without_a_plot_window(self) -> None:
        supervisor = self.supervisor()
        command = supervisor._command(supervisor.policy_profiles["carrot_act_v2"])
        self.assertIn("--robot.disable_torque_on_disconnect=false", command)
        self.assertIn("--debug_visualize_queue_size=false", command)

    def test_each_policy_gets_its_recorded_camera_layout(self) -> None:
        supervisor = self.supervisor()

        def cameras(policy_id: str) -> str:
            command = supervisor._command(supervisor.policy_profiles[policy_id])
            return next(part for part in command if part.startswith("--robot.cameras="))

        # carrot was recorded with front = overhead; apple with the two swapped (front = gripper).
        carrot, apple = cameras("carrot_act_v2"), cameras("apple_act_20261109")
        self.assertLess(carrot.index("/dev/overhead"), carrot.index("/dev/gripper"))
        self.assertLess(apple.index("/dev/gripper"), apple.index("/dev/overhead"))


if __name__ == "__main__":
    unittest.main()
