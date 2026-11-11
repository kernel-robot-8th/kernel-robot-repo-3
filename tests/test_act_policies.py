from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

from control_station.act_policies import ActPolicyError, load_act_policies

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from omx_act_policy_run import DEFAULT_POLICIES, RestWatcher, load_policies, policy_cameras  # noqa: E402

HOME = {"shoulder_pan": 2033, "shoulder_lift": 748, "elbow_flex": 3175, "wrist_flex": 2945, "wrist_roll": 2025}


def shifted(**deltas: int) -> dict[str, int]:
    return {name: value + deltas.get(name, 0) for name, value in HOME.items()}


class ActPolicyRegistryTests(unittest.TestCase):
    def test_committed_registry_maps_korean_and_detector_names(self) -> None:
        registry = load_act_policies()
        self.assertEqual(registry.resolve("당근")["model"], "JJinJJin2/omx_act_policy_v2")
        self.assertEqual(registry.resolve("Carrot")["policy_id"], "carrot_act_v2")
        self.assertEqual(registry.resolve("물티슈")["object_class"], "tissue")
        self.assertIsNone(registry.resolve("ball"))

    def test_worker_and_control_station_see_the_same_registry_version(self) -> None:
        version, policies = load_policies(DEFAULT_POLICIES)
        self.assertEqual(version, load_act_policies().version)
        self.assertEqual(sorted(policies), ["carrot_act_v2", "tissue_act_v1"])

    def test_bad_registries_are_refused(self) -> None:
        good = json.loads(DEFAULT_POLICIES.read_text(encoding="utf-8"))
        carrot = good["policies"][0]
        cases = {
            "same name in two policies": [carrot, {**carrot, "policy_id": "other"}],
            "front twice": [{**carrot, "cameras": {"front": "OVERHEAD", "wrist": "OVERHEAD"}}],
            "model not a repo id": [{**carrot, "model": "omx_act_policy_v2"}],
        }
        with tempfile.TemporaryDirectory() as directory:
            for name, policies in cases.items():
                path = Path(directory) / "policies.json"
                path.write_text(json.dumps({"schema_version": 1, "policies": policies}), encoding="utf-8")
                with self.subTest(name), self.assertRaises(ActPolicyError):
                    load_act_policies(path)


class PolicyRunTests(unittest.TestCase):
    def test_cameras_follow_the_roles_each_policy_was_trained_with(self) -> None:
        _, policies = load_policies(DEFAULT_POLICIES)
        devices = {"OVERHEAD": "/dev/overhead", "GRIPPER": "/dev/gripper"}
        self.assertEqual(policy_cameras(policies["carrot_act_v2"], devices), {"front": "/dev/overhead", "wrist": "/dev/gripper"})
        # The tissue dataset was recorded with the two camera names swapped.
        self.assertEqual(policy_cameras(policies["tissue_act_v1"], devices), {"front": "/dev/gripper", "wrist": "/dev/overhead"})

    def test_stops_only_after_leaving_rest_and_settling_back(self) -> None:
        watcher = RestWatcher(HOME, depart_ticks=300, rest_ticks=150, settle_s=1.5)
        self.assertFalse(watcher.update(0.0, HOME))  # still at rest before the policy moved: not done
        self.assertFalse(watcher.update(5.0, HOME))
        self.assertFalse(watcher.update(6.0, shifted(shoulder_lift=900)))  # reaching for the shelf
        self.assertTrue(watcher.departed)
        self.assertFalse(watcher.update(10.0, shifted(shoulder_lift=100)))  # back near rest, settling starts
        self.assertFalse(watcher.update(11.0, shifted(shoulder_lift=60)))
        self.assertTrue(watcher.update(11.6, shifted(wrist_roll=74)))
        self.assertEqual(watcher.farthest, 900)

    def test_leaving_rest_again_restarts_the_settle_time(self) -> None:
        watcher = RestWatcher(HOME, depart_ticks=300, rest_ticks=150, settle_s=1.5)
        watcher.update(0.0, shifted(elbow_flex=-600))
        watcher.update(1.0, HOME)
        watcher.update(2.0, shifted(elbow_flex=-200))  # passing by rest on the way back out
        self.assertFalse(watcher.update(2.6, HOME))
        self.assertTrue(watcher.update(4.2, HOME))


if __name__ == "__main__":
    unittest.main()
