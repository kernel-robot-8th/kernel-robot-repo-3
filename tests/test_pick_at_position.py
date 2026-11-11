from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from omx_kinematics import forward, tick_to_radians  # noqa: E402
from omx_pick_at_position import FINGER_M, PickAtPosition, plan_pick  # noqa: E402
from omx_slot_motion import ARM_JOINTS, SlotMotion, add_motion_arguments  # noqa: E402

HOME = {"shoulder_pan": 2033, "shoulder_lift": 748, "elbow_flex": 3175, "wrist_flex": 2945, "wrist_roll": 2025}
CARROT = (0.23, 0.09, -0.003)  # object centre measured on 2026-11-07 (omx_link0)


def fingertip(ticks: dict[str, int]) -> tuple[float, float, float]:
    points = forward(tick_to_radians(ticks))
    direction = (points[-1] - points[-2]) / math.dist(points[-1], points[-2])
    return tuple(points[-1] + FINGER_M * direction)


class PlanTests(unittest.TestCase):
    def test_fingertip_goes_above_then_onto_then_above_the_object(self) -> None:
        plan = plan_pick(CARROT, HOME)
        for name in ("hover", "grasp", "lift"):
            # ticks are rounded, so the reached fingertip is within a few millimetres of the planned one
            self.assertLess(math.dist(fingertip(plan.poses[name]), plan.fingertips[name]), 0.004, name)
        self.assertAlmostEqual(plan.fingertips["hover"][2] - CARROT[2], 0.06, places=3)
        self.assertAlmostEqual(plan.fingertips["grasp"][2], CARROT[2], places=3)
        self.assertGreater(math.hypot(*plan.fingertips["grasp"][:2]), math.hypot(*CARROT[:2]))

    def test_out_of_reach_is_refused(self) -> None:
        with self.assertRaises(Exception):
            plan_pick((0.60, 0.0, 0.0), HOME)


class FakeArmPickTests(unittest.TestCase):
    def motion(self, place: bool) -> SlotMotion:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        slots = REPOSITORY_ROOT / "config" / "shelf-slots.json"
        import hashlib
        poses = {
            "schema_version": 1, "follower_board": "FAKE", "slot_map_sha256": hashlib.sha256(slots.read_bytes()).hexdigest(),
            "poses": {"home": {"ticks": {**HOME, "gripper": 2010}}},
        }
        if place:
            poses["poses"]["place_pinky"] = {"ticks": {**HOME, "shoulder_pan": 1700, "shoulder_lift": 1300, "gripper": 2010}}
        path = Path(directory.name) / "poses.json"
        path.write_text(json.dumps(poses), encoding="utf-8")
        parser = argparse.ArgumentParser()
        add_motion_arguments(parser)
        return SlotMotion(parser.parse_args(["--fake-follower", "--poses", str(path), "--profile-velocity", "400",
                                             "--max-delta", "1500"]))

    def command(self) -> dict:
        return {
            "valid_until": (datetime.now(timezone.utc) + timedelta(seconds=60)).isoformat(),
            "target": {"frame_id": "omx_link0", "approach": "PITCH_DOWN_45",
                       "position": dict(zip("xyz", CARROT))},
        }

    def run_pick(self, stop_after: str, place: bool) -> tuple[str | None, list[tuple[str, dict]], SlotMotion]:
        motion = self.motion(place)
        picker = PickAtPosition(motion, stop_after, 1500)
        sent: list[tuple[str, dict]] = []
        with motion.follower():
            reason, plan = picker.check(self.command())
            if reason is None:
                picker.run(plan, lambda event, result=None, **fields: sent.append((event, fields)) or {}, lambda: False)
        return reason, sent, motion

    def test_stop_after_lift_holds_the_object_up(self) -> None:
        reason, sent, motion = self.run_pick("lift", place=False)
        self.assertIsNone(reason)
        self.assertEqual([fields.get("phase") for event, fields in sent if event == "PROGRESS"],
                         ["APPROACH_TARGET", "GRASP_TARGET", "LIFT_TARGET"])
        self.assertEqual((sent[-1][0], sent[-1][1]["reason_code"]), ("RESULT_UNKNOWN", "STOPPED_AFTER_LIFT"))
        self.assertLess(motion.fake_bus.positions["gripper"], 1950)  # closed (fake arm: nothing to stop it)

    def test_full_pick_places_opens_and_goes_home_without_claiming_success(self) -> None:
        reason, sent, motion = self.run_pick("place", place=True)
        self.assertIsNone(reason)
        self.assertEqual([fields.get("phase") for event, fields in sent if event == "PROGRESS"],
                         ["APPROACH_TARGET", "GRASP_TARGET", "LIFT_TARGET", "MOVE_TO_DESTINATION", "PLACE_TARGET"])
        event, fields = sent[-1]
        self.assertEqual((event, fields["reason_code"], fields["pose_reached"]), ("RESULT_UNKNOWN", "PLACE_NOT_VERIFIED", True))
        self.assertGreater(motion.fake_bus.positions["gripper"], 2400)  # released
        self.assertTrue(all(abs(motion.fake_bus.positions[j] - HOME[j]) <= 15 for j in ARM_JOINTS))

    def test_full_pick_without_a_place_pose_is_refused_before_moving(self) -> None:
        reason, sent, _ = self.run_pick("place", place=False)
        self.assertEqual((reason, sent), ("POSE_NOT_RECORDED", []))


if __name__ == "__main__":
    unittest.main()
