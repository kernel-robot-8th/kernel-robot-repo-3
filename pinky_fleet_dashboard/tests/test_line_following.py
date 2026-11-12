import json
import math
import tempfile
import unittest
from pathlib import Path

from scripts.line_following import (LineAssist, capture_profile, default_profile,
                                    normalize, swept_clear, validate_profile)
from scripts.ir_profiles import IRProfiles


def calibrated():
    p = default_profile()
    p.update(floor=[3000., 2800., 2600.], line=[300., 400., 500.], enabled=True)
    return p


class LineTests(unittest.TestCase):
    def decide(self, raw, **overrides):
        args = dict(linear=.048, angular=0., raw=raw, profile=calibrated(),
                    now=1., ir_fresh=True, scan_fresh=True, scan_points=[])
        args.update(overrides)
        return LineAssist().decide(**args)

    def test_center_keeps_straight_speed(self):
        d = self.decide([3000, 400, 2600])
        self.assertEqual(d.state, 'centered')
        self.assertEqual(d.angular, 0.)
        self.assertEqual(d.linear, .048)

    def test_center_reference_suppresses_minor_nav_steering(self):
        d = self.decide([3000, 400, 2600], angular=.08)
        self.assertEqual((d.state, d.angular), ('centered', 0.))

    def test_two_active_sensors_at_corners_are_valid(self):
        for raw in ([300, 400, 2600], [3000, 400, 500]):
            d = self.decide(raw, angular=.08)
            self.assertEqual((d.state, d.angular, d.linear), ('nav2_corner', .08, .048))
        d = self.decide([300, 2800, 500])
        self.assertEqual(d.state, 'nav2_ambiguous')

    def test_center_hysteresis_avoids_chatter(self):
        assist = LineAssist()
        def decide(raw, now):
            return assist.decide(.048, .0, raw, calibrated(), now, True, True, [])
        self.assertEqual(decide([3000, 400, 2600], 1.).state, 'centered')
        # Center strength 0.4 remains centered after a strong center sample.
        self.assertEqual(decide([3000, 1840, 2600], 1.05).state, 'centered')
        # Center strength 0.3 drops the center latch.
        self.assertEqual(decide([3000, 2080, 2600], 1.10).state, 'nav2_line_lost')
        self.assertEqual(decide([3000, 1840, 2600], 1.15).state, 'nav2_line_lost')
        self.assertEqual(decide([3000, 400, 2600], 1.20).state, 'centered')

    def test_left_and_right_signs_and_speed_bounds(self):
        left, right = self.decide([300, 2800, 2600]), self.decide([3000, 2800, 500])
        self.assertGreater(left.angular, 0)
        self.assertLess(right.angular, 0)
        self.assertLess(left.linear, .048)
        self.assertLessEqual(abs(left.angular), .15)

    def test_default_disabled_does_not_change_nav(self):
        d = self.decide([], profile=default_profile(), scan_fresh=False, ir_fresh=False)
        self.assertEqual((d.linear, d.angular, d.state), (.048, 0., 'disabled'))

    def test_missing_calibration_or_stale_input_stops(self):
        invalid = default_profile(); invalid['enabled'] = True
        for kw in (dict(profile=invalid), dict(ir_fresh=False), dict(scan_fresh=False)):
            d = self.decide([3000, 400, 2600], **kw)
            self.assertEqual((d.linear, d.angular), (0., 0.))
        for raw in ([1, 2], [math.nan, 2, 3], [5000, 2, 3]):
            self.assertEqual(self.decide(raw).linear, 0.)

    def test_goal_rotation_reverse_and_lidar_avoidance_unchanged(self):
        for v, w in ((0., .2), (-.03, 0.), (.02, .18)):
            d = self.decide([300, 2800, 2600], linear=v, angular=w)
            self.assertEqual((d.linear, d.angular), (v, w))
        d = self.decide([300, 2800, 2600], scan_points=[(.2, .05)])
        self.assertEqual(d.state, 'nav2_avoidance')
        self.assertEqual((d.linear, d.angular), (.048, 0.))

    def test_node_and_lost_line_hand_back_to_nav_without_blind_motion(self):
        for raw, state in (([300, 400, 500], 'marker_unclassified'), ([3000, 2800, 2600], 'nav2_line_lost')):
            d = self.decide(raw)
            self.assertEqual((d.linear, d.angular, d.state), (.048, 0., state))

    def test_collision_on_corrected_path_stops(self):
        # Lateral obstacle outside the Nav2 bypass cone but inside robot footprint.
        d = self.decide([3000, 400, 2600], scan_points=[(.01, .065)])
        self.assertEqual((d.linear, d.angular, d.state), (0., 0., 'lidar_stop'))
        self.assertTrue(swept_clear([(.5, .5)], .048, .15))
        self.assertFalse(swept_clear([(.09, 0)], .048, 0.))

    def test_opposite_polarity_and_channel_mapping(self):
        p = calibrated(); p['floor'], p['line'] = p['line'], p['floor']
        self.assertEqual(normalize([3000, 400, 500], p), [1., 0., 0.])
        p['channel_order'] = [2, 1, 0]
        self.assertEqual(normalize([3000, 400, 500], p), [0., 0., 1.])

    def test_capture_requires_stopped_stable_multi_sample_data(self):
        p = default_profile()
        p = capture_profile(p, [[3000, 2800, 2600]]*12, 'floor')
        for ch, value in enumerate((300, 400, 500)):
            sample = [3000, 2800, 2600]; sample[ch] = value
            p = capture_profile(p, [sample]*12, 'line', ch)
        self.assertIsNone(validate_profile(p))
        self.assertEqual(p['floor'], [3000, 2800, 2600])
        with self.assertRaises(ValueError): capture_profile(p, [[1, 2, 3]], 'floor')
        with self.assertRaises(ValueError): capture_profile(p, [[0, 0, 0], [4095, 4095, 4095]]*6, 'floor')
        p['enabled'] = True
        with self.assertRaises(ValueError): capture_profile(p, [[1000]*3]*12, 'floor')

    def test_low_contrast_or_invalid_noise_cannot_enable(self):
        p = calibrated(); p['line'][1] = p['floor'][1]-20
        self.assertIsNotNone(validate_profile(p))
        p = calibrated(); p['floor_noise'][1] = math.nan
        self.assertIsNotNone(validate_profile(p))

    def test_atomic_storage_robot_isolation_and_restart(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'profiles.json'
            store = IRProfiles(path, ['10', '12'])
            store.save('10', calibrated())
            store = IRProfiles(path, ['10', '12'])
            self.assertTrue(store.get('10')['enabled'])
            self.assertFalse(store.get('12')['enabled'])
            self.assertEqual(json.loads(path.read_text())['10']['floor'], [3000, 2800, 2600])


if __name__ == '__main__':
    unittest.main()
