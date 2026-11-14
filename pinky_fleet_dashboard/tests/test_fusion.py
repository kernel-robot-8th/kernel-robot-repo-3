"""No hardware: LiDAR + ultrasonic fusion in Nav2's height-cell costmap, and IR kept as a reading only."""
import math
import unittest
from unittest.mock import patch

import app
from scripts.install_nav2_runtime import add_ultrasonic_fusion
from scripts.ultrasonic_cloud import RangeFilter


class RangeFilterTests(unittest.TestCase):
    def test_a_steady_echo_is_marked_after_agreeing_readings(self):
        f = RangeFilter({})
        self.assertEqual(f.update(0.30)[0], 'unsure')
        self.assertEqual(f.update(0.31)[0], 'unsure')
        kind, distance = f.update(0.30)
        self.assertEqual(kind, 'obstacle')
        self.assertAlmostEqual(distance, 0.30)

    def test_a_single_jump_is_outvoted_by_the_median(self):
        f = RangeFilter({})
        for value in (0.40, 0.40):
            f.update(value)
        self.assertEqual(f.update(0.20), ('obstacle', 0.40))  # one odd reading (12번 0.60→0.77 같은) does not move it

    def test_a_real_change_needs_new_agreeing_readings(self):
        f = RangeFilter({'window': 3})
        for value in (0.50, 0.50, 0.50):
            f.update(value)
        self.assertEqual(f.update(0.20)[0], 'obstacle')  # median still 0.50
        self.assertEqual(f.update(0.20)[0], 'unsure')    # median moved to 0.20: agreement starts again

    def test_far_echoes_only_clear_and_no_echo_clears_to_the_limit(self):
        f = RangeFilter({})
        for value in (0.70, 0.70, 0.70):
            kind, distance = f.update(value)
        self.assertEqual((kind, distance), ('free', 0.70))  # beyond mark_max_range_m: may be the floor
        self.assertEqual(f.update(math.inf), ('free', 0.8))
        self.assertEqual(f.update(math.nan), ('unsure', None))

    def test_the_arc_spans_the_cone(self):
        points = RangeFilter({'arc_points': 3}).arc(0.5, 0.26)
        self.assertEqual(len(points), 3)
        self.assertAlmostEqual(points[1][0], 0.5)
        self.assertAlmostEqual(points[0][1], -points[2][1])


def pinky_params():
    """The two costmap layers as pinky_navigation's nav2_params.yaml has them (checked 2026-11-14)."""
    scan = {'topic': '/scan', 'data_type': 'LaserScan', 'marking': True, 'clearing': True}
    return {
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'plugins': ['voxel_layer', 'inflation_layer'],
            'voxel_layer': {'plugin': 'nav2_costmap_2d::VoxelLayer', 'origin_z': 0.0, 'z_resolution': 0.05,
                            'z_voxels': 16, 'observation_sources': 'scan', 'scan': dict(scan)}}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
            'obstacle_layer': {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'observation_sources': 'scan',
                               'scan': dict(scan)}}}},
    }


class Nav2FusionParamsTests(unittest.TestCase):
    def test_both_costmaps_get_height_cells_with_the_scan_and_both_ultrasonic_clouds(self):
        parsed = pinky_params()
        add_ultrasonic_fusion(parsed)
        for name, layer in (('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')):
            section = parsed[name][name]['ros__parameters'][layer]
            self.assertEqual(section['plugin'], 'nav2_costmap_2d::VoxelLayer')
            self.assertEqual(section['observation_sources'].split(), ['scan', 'ultrasonic_mark', 'ultrasonic_clear'])
            self.assertTrue(section['ultrasonic_mark']['marking'])
            self.assertFalse(section['ultrasonic_mark']['clearing'])
            self.assertTrue(section['ultrasonic_clear']['clearing'])
            self.assertFalse(section['ultrasonic_clear']['marking'])
            self.assertEqual(section['scan']['topic'], '/scan')  # the LiDAR source stays as it was
        again = pinky_params()
        add_ultrasonic_fusion(again)
        add_ultrasonic_fusion(again)  # applying twice adds nothing twice
        self.assertEqual(again['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['observation_sources'].split().count('ultrasonic_mark'), 1)


class IrReadingOnlyTests(unittest.TestCase):
    def test_ir_does_not_steer_while_navigation_says_so_even_with_a_calibrated_profile(self):
        profiles = type('P', (), {'get': lambda self, rid: {'enabled': True}})()
        with patch.multiple(app, IR_PROFILES=profiles, NAVIGATION={'ir_steering': False}):
            self.assertFalse(app._ir_steers('10'))
        with patch.multiple(app, IR_PROFILES=profiles, NAVIGATION={'ir_steering': True}):
            self.assertTrue(app._ir_steers('10'))

    def test_the_shipped_settings_are_nav2_with_fusion_and_no_detour(self):
        self.assertEqual((app.NAVIGATION['ir_steering'], app.NAVIGATION['ultrasonic_fusion'], app.NAVIGATION['ultrasonic_detour']),
                         (False, True, False))


if __name__ == '__main__':
    unittest.main()


class LocalizedBeforeReadyTests(unittest.TestCase):
    def test_a_pose_from_before_localization_does_not_make_the_pinky_ready(self):
        from scripts.fleet_ops import operational_status, REQUIRED
        topics = {key: {'fresh': True, 'value': {}} for key in REQUIRED}
        topics['line_control']['value'] = {'dock_protocol': 'sponge_v1', 'line_algorithm': 'fleet_control_v4'}
        topics['safety_status']['value'] = {'version': 'final_gate_v1'}
        robot = {'stage': 'prepared', 'ssh_reachable': True, 'nav_started_at': 100.0}
        self.assertEqual(operational_status(robot, topics)['state'], 'unlocalized')  # AMCL's default origin pose
        robot['localized_at'] = 90.0  # set before this Nav2 start: stale
        self.assertEqual(operational_status(robot, topics)['state'], 'unlocalized')
        robot['localized_at'] = 101.0
        self.assertEqual(operational_status(robot, topics)['state'], 'ready')
