from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from omx_kinematics import (  # noqa: E402
    ARM_JOINTS,
    Unreachable,
    forward,
    inverse,
    radians_to_ticks,
    tick_to_radians,
    tool_pitch,
)

ZERO = dict.fromkeys(ARM_JOINTS, 2048)


class ForwardKinematicsTests(unittest.TestCase):
    def test_zero_ticks_give_the_mjcf_zero_pose(self) -> None:
        angles = tick_to_radians(ZERO)
        self.assertTrue(all(angle == 0 for angle in angles.values()))
        # Upper arm straight up, forearm and wrist straight forward (sum of the MJCF offsets).
        np.testing.assert_allclose(
            forward(angles)[-1], [-0.01125 + 0.0415 + 0.162 + 0.0287 + 0.0295, 0.0, 0.034 + 0.0635 + 0.11315]
        )

    def test_a_quarter_turn_of_the_pan_points_the_arm_left(self) -> None:
        angles = tick_to_radians({**ZERO, "shoulder_pan": 2048 + 1024})
        self.assertAlmostEqual(angles["shoulder_pan"], math.pi / 2)
        x, y, _ = forward(angles)[-1]
        self.assertAlmostEqual(x, -0.01125)
        self.assertGreater(y, 0.2)

    def test_lifting_the_shoulder_tips_the_arm_forward_and_down(self) -> None:
        upright = forward(tick_to_radians(ZERO))[2]
        tipped = forward(tick_to_radians({**ZERO, "shoulder_lift": 2048 + 256}))[2]
        self.assertGreater(tipped[0], upright[0])
        self.assertLess(tipped[2], upright[2])


class InverseKinematicsTests(unittest.TestCase):
    # The two taught slot poses (ticks recorded on 2026-11-04) and a horizontal reach.
    POSES = (
        {"shoulder_pan": 2118, "shoulder_lift": 2153, "elbow_flex": 2601, "wrist_flex": 1790, "wrist_roll": 2071},
        {"shoulder_pan": 1939, "shoulder_lift": 1967, "elbow_flex": 2329, "wrist_flex": 1671, "wrist_roll": 2056},
        {"shoulder_pan": 2048, "shoulder_lift": 2300, "elbow_flex": 2048, "wrist_flex": 1796, "wrist_roll": 2048},
    )

    def test_inverse_of_forward_returns_the_same_joints(self) -> None:
        for ticks in self.POSES:
            with self.subTest(ticks=ticks):
                angles = tick_to_radians(ticks)
                solved = inverse(forward(angles)[-1], tool_pitch(angles), angles["wrist_roll"], seed=angles)
                for name in ARM_JOINTS:
                    self.assertAlmostEqual(solved[name], angles[name], places=9)
                self.assertEqual(radians_to_ticks(solved), ticks)

    def test_solution_reaches_the_target_at_the_requested_pitch(self) -> None:
        target = (0.20, 0.05, 0.12)
        angles = inverse(target, math.radians(20))
        np.testing.assert_allclose(forward(angles)[-1], target, atol=1e-9)
        self.assertAlmostEqual(tool_pitch(angles), math.radians(20))

    def test_points_out_of_reach_or_range_are_refused(self) -> None:
        for target, reason in (((0.60, 0.0, 0.10), "OUT_OF_REACH"), ((-0.01125, 0.0, 0.2), "OUT_OF_REACH")):
            with self.subTest(target=target), self.assertRaises(Unreachable) as raised:
                inverse(target)
            self.assertEqual(raised.exception.reason_code, reason)
        with self.assertRaises(Unreachable) as raised:  # horizontal just in front of the shoulder folds past the range
            inverse((0.0, 0.0, 0.10))
        self.assertEqual(raised.exception.reason_code, "JOINT_LIMIT")


if __name__ == "__main__":
    unittest.main()
