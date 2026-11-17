"""No hardware: the fused low layer in Nav2's height-cell costmap, and IR kept as a reading only.

The fusion itself (scripts/fusion_grid.py) is tested in test_fusion_grid.py."""
import unittest
from unittest.mock import patch

import app
from scripts.install_nav2_runtime import add_ultrasonic_fusion
from scripts.fusion_grid import validated


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
    def test_the_ultrasonic_goes_to_the_local_costmap_only_by_default(self):
        parsed = pinky_params()
        add_ultrasonic_fusion(parsed)
        local = parsed['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
        self.assertEqual(local['observation_sources'].split(), ['scan', 'fusion_low_mark', 'fusion_low_clear'])
        self.assertFalse(local['publish_voxel_map'])
        self.assertEqual(parsed['global_costmap'], pinky_params()['global_costmap'])  # a wrong mark is not kept there

    def test_with_global_both_costmaps_get_height_cells_with_the_scan_and_both_ultrasonic_clouds(self):
        parsed = pinky_params()
        add_ultrasonic_fusion(parsed, include_global=True)
        for name, layer in (('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')):
            section = parsed[name][name]['ros__parameters'][layer]
            self.assertEqual(section['plugin'], 'nav2_costmap_2d::VoxelLayer')
            self.assertEqual(section['observation_sources'].split(), ['scan', 'fusion_low_mark', 'fusion_low_clear'])
            self.assertEqual((section['fusion_low_mark']['topic'], section['fusion_low_clear']['topic']),
                             ('/fusion/low_marks', '/fusion/low_clear'))  # scripts/fusion_grid.py publishes these
            self.assertTrue(section['fusion_low_mark']['marking'])
            self.assertFalse(section['fusion_low_mark']['clearing'])
            self.assertTrue(section['fusion_low_clear']['clearing'])
            self.assertFalse(section['fusion_low_clear']['marking'])
            # Both stay in the lowest 5 cm voxel row: the LiDAR plane (about 12.5 cm) is never cleared by them
            self.assertLessEqual(section['fusion_low_clear']['max_obstacle_height'], 0.10)
            self.assertEqual(section['scan']['topic'], '/scan')  # the LiDAR source stays as it was
            self.assertFalse(section['publish_voxel_map'])
        again = pinky_params()
        add_ultrasonic_fusion(again)
        add_ultrasonic_fusion(again)  # applying twice adds nothing twice
        self.assertEqual(again['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['observation_sources'].split().count('fusion_low_mark'), 1)


class IrReadingOnlyTests(unittest.TestCase):
    def test_ir_does_not_steer_while_navigation_says_so_even_with_a_calibrated_profile(self):
        profiles = type('P', (), {'get': lambda self, rid: {'enabled': True}})()
        with patch.multiple(app, IR_PROFILES=profiles, NAVIGATION={'ir_steering': False}):
            self.assertFalse(app._ir_steers('10'))
        with patch.multiple(app, IR_PROFILES=profiles, NAVIGATION={'ir_steering': True}):
            self.assertTrue(app._ir_steers('10'))

    def test_the_shipped_settings_are_nav2_with_lidar_costmap_first_and_no_detour(self):
        # 2026-11-16: LiDAR-only costmap first; the ultrasonic joins once that runs steadily.
        self.assertEqual((app.NAVIGATION['ir_steering'], app.NAVIGATION['ultrasonic_fusion'], app.NAVIGATION['ultrasonic_detour']),
                         (False, False, False))

    def test_the_shipped_fusion_grid_settings_are_valid(self):
        validated(app.NAVIGATION['fusion_grid'])  # the robot node would refuse to start otherwise


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


class StatusDisplayTests(unittest.TestCase):
    def test_the_screen_follows_the_lamp_colour_and_says_no_link_when_the_dashboard_is_quiet(self):
        from scripts.status_display import screen_state, render
        self.assertEqual(screen_state(('blue', 'on'), 10.0, 11.0), ('blue', 'READY'))
        self.assertEqual(screen_state(('green', 'on'), 10.0, 11.0), ('green', 'DRIVING'))
        self.assertEqual(screen_state(('green', 'on'), 10.0, 20.0), ('orange', 'NO LINK'))
        self.assertEqual(screen_state(None, 0.0, 1.0), ('orange', 'NO LINK'))
        image = render((320, 240), 'P2', '12', 'blue', 'READY')
        self.assertEqual(image.size, (320, 240))
        self.assertEqual(image.getpixel((5, 5)), (0, 90, 255))
