from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

from control_station import REPO_ROOT
from control_station.arms.policies import ActPolicyError, load_act_policies

sys.path.insert(0, str(REPO_ROOT / "scripts"))

from omx_act_policy_run import DEFAULT_POLICIES, RestWatcher, load_policies, policy_cameras  # noqa: E402

from support import TEST_POLICIES_PATH  # noqa: E402

HOME = {"shoulder_pan": 2033, "shoulder_lift": 748, "elbow_flex": 3175, "wrist_flex": 2945, "wrist_roll": 2025}


def shifted(**deltas: int) -> dict[str, int]:
    return {name: value + deltas.get(name, 0) for name, value in HOME.items()}


class ActPolicyRegistryTests(unittest.TestCase):
    # The first two read the committed config/omx-act-policies.json. They name no policy id or model, so swapping a
    # model or an object there does not break them.
    def test_committed_registry_maps_korean_and_detector_names(self) -> None:
        registry = load_act_policies()
        for policy in registry.policies.values():
            arm_id = policy["arm_id"]
            for name in (policy["label"], policy["object_class"].title(), *policy["aliases"]):
                with self.subTest(policy=policy["policy_id"], name=name):
                    self.assertEqual(registry.resolve(name, arm_id)["policy_id"], policy["policy_id"])
        self.assertIsNone(registry.resolve("ball"))

    def test_worker_and_control_station_see_the_same_registry_version(self) -> None:
        version, policies = load_policies(DEFAULT_POLICIES)
        registry = load_act_policies()
        self.assertEqual(version, registry.version)
        self.assertEqual(sorted(policies), sorted(registry.policies))

    def test_bad_registries_are_refused(self) -> None:
        good = json.loads(TEST_POLICIES_PATH.read_text(encoding="utf-8"))
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
        _, policies = load_policies(TEST_POLICIES_PATH)
        devices = {"OVERHEAD": "/dev/overhead", "GRIPPER": "/dev/gripper"}
        self.assertEqual(policy_cameras(policies["carrot_act_v2"], devices), {"front": "/dev/overhead", "wrist": "/dev/gripper"})
        # The tissue dataset was recorded with the two camera names swapped.
        self.assertEqual(policy_cameras(policies["tissue_act_v1"], devices), {"front": "/dev/gripper", "wrist": "/dev/overhead"})

    def test_stops_only_after_leaving_rest_and_three_home_samples(self) -> None:
        watcher = RestWatcher(HOME, depart_ticks=300, rest_ticks=150)
        self.assertFalse(watcher.update(1, HOME))  # still at rest before the policy moved: not done
        self.assertFalse(watcher.update(2, HOME))
        self.assertFalse(watcher.update(3, shifted(shoulder_lift=900)))  # reaching for the shelf
        self.assertTrue(watcher.departed)
        self.assertFalse(watcher.update(4, shifted(shoulder_lift=100)))  # first home sample
        self.assertFalse(watcher.update(5, shifted(shoulder_lift=60)))
        self.assertTrue(watcher.update(6, shifted(wrist_roll=74)))
        self.assertEqual(watcher.farthest, 900)

    def test_leaving_home_tolerance_resets_consecutive_sample_count(self) -> None:
        watcher = RestWatcher(HOME, depart_ticks=300, rest_ticks=150)
        watcher.update(1, shifted(elbow_flex=-600))
        watcher.update(2, HOME)
        watcher.update(3, shifted(elbow_flex=-200))  # passing by rest on the way back out
        self.assertFalse(watcher.update(4, HOME))
        self.assertFalse(watcher.update(5, HOME))
        self.assertTrue(watcher.update(6, HOME))


if __name__ == "__main__":
    unittest.main()
