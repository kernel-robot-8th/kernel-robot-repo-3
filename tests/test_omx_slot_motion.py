from __future__ import annotations

import argparse
import sys
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from omx_slot_motion import ARM_JOINTS, Rejected, SlotMotion, add_motion_arguments  # noqa: E402


def fake_motion(*options: str) -> SlotMotion:
    parser = argparse.ArgumentParser()
    add_motion_arguments(parser)
    # The fake arm starts at the recorded "home" pose, which is re-taught on site; a wide --max-delta keeps these
    # tests independent of how far home is from the slot (later options override it).
    return SlotMotion(parser.parse_args(["--fake-follower", "--profile-velocity", "200", "--max-delta", "4096", *options]))


def soon() -> datetime:
    return datetime.now(timezone.utc) + timedelta(seconds=30)


class SlotMotionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.motion = fake_motion()
        self.version = self.motion.slot_map_version

    def plan(self, slot_id: str = "slot_a_2_1", **overrides: object) -> tuple[dict[str, int], dict | None]:
        arguments = {"slot_map_version": self.version, "lateral_offset_m": None, "valid_until": soon(), **overrides}
        with self.motion.follower():
            return self.motion.plan(slot_id, **arguments)

    def test_checks_reject_before_anything_moves(self) -> None:
        cases = {
            "SLOT_MAP_MISMATCH": {"slot_map_version": "other"},
            "POSE_NOT_RECORDED": {"slot_id": "slot_a_2_2"},
            "EXPIRED": {"valid_until": datetime.now(timezone.utc) - timedelta(seconds=1)},
        }
        start = self.motion.fake_bus.sync_read("Present_Position", list(ARM_JOINTS))
        for reason, overrides in cases.items():
            with self.subTest(reason), self.assertRaises(Rejected) as raised:
                self.plan(**overrides)
            self.assertEqual(raised.exception.reason_code, reason)
        far = fake_motion("--max-delta", "100")
        with far.follower(), self.assertRaises(Rejected) as raised:
            far.plan("slot_a_2_1", far.slot_map_version, None, soon())
        self.assertEqual(raised.exception.reason_code, "TARGET_TOO_FAR")
        self.assertEqual(self.motion.fake_bus.sync_read("Present_Position", list(ARM_JOINTS)), start)

    def test_reaches_the_recorded_pose_and_reports_it_as_simulated(self) -> None:
        goal, correction = self.plan()
        self.assertIsNone(correction)
        reports: list[dict[str, int]] = []
        with self.motion.follower():
            outcome = self.motion.move(goal, lambda: False, lambda present, _error: reports.append(present))
        self.assertEqual(outcome.outcome, "SUCCEEDED")
        self.assertTrue(outcome.pose_reached and outcome.simulated)
        self.assertLessEqual(outcome.max_error_ticks, 15)
        self.assertNotEqual(reports[0], goal)  # the first report comes before the goal is written

    def test_cancel_holds_the_arm_where_it_is(self) -> None:
        motion = fake_motion("--profile-velocity", "1")  # about 16 ticks/s: still far from the goal
        with motion.follower():
            goal, _ = motion.plan("slot_a_2_1", motion.slot_map_version, None, soon())
            outcome = motion.move(goal, lambda: True)
        self.assertEqual(outcome.outcome, "CANCELED")
        self.assertFalse(outcome.pose_reached)
        self.assertGreater(outcome.max_error_ticks, 15)
        self.assertEqual(motion.fake_bus.goal, {name: float(value) for name, value in outcome.final_ticks.items()})

    def test_timeout_fails_without_claiming_the_pose(self) -> None:
        motion = fake_motion("--profile-velocity", "1", "--move-timeout", "0.2")
        with motion.follower():
            goal, _ = motion.plan("slot_a_2_1", motion.slot_map_version, None, soon())
            outcome = motion.move(goal, lambda: False)
        self.assertEqual((outcome.outcome, outcome.reason_code, outcome.pose_reached), ("FAILED", "MOVE_TIMEOUT", False))

    def test_cancel_check_error_stops_with_unknown_result(self) -> None:
        motion = fake_motion("--profile-velocity", "1")

        def broken() -> bool:
            raise ConnectionError("control station unreachable")

        with motion.follower():
            goal, _ = motion.plan("slot_a_2_1", motion.slot_map_version, None, soon())
            outcome = motion.move(goal, broken)
        self.assertEqual((outcome.outcome, outcome.reason_code), ("RESULT_UNKNOWN", "CANCEL_CHECK_FAILED"))


if __name__ == "__main__":
    unittest.main()
