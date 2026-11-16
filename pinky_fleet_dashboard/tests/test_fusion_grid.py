"""No hardware: the LiDAR + ultrasonic occupancy grid (scripts/fusion_grid.py) and its offline evaluation."""
import math
import unittest

import numpy as np

from scripts.fusion_grid import FusionGrid, validated, to_local, ring
from scripts import fusion_eval
from scripts.fusion_eval import compare, noise, noise_stats, suggested_sigmas, StaticFrames

ROBOT = (0.0, 0.0, 0.0)


def value_at(grid, layer, x, y):
    i = int(math.floor((y - grid.origin[1]) / grid.res))
    j = int(math.floor((x - grid.origin[0]) / grid.res))
    return float(getattr(grid, layer)[i, j])


def beams(*pairs, count=360):
    """A scan of `count` beams over a full turn; pairs = (angle, range) set, every other beam has no return."""
    angles = -math.pi + np.arange(count) * 2 * math.pi / count
    ranges = np.full(count, np.inf)
    for angle, distance in pairs:
        ranges[int(np.argmin(np.abs(angles - angle)))] = distance
    return angles, ranges


class LidarModelTests(unittest.TestCase):
    def test_a_beam_frees_the_cells_it_passes_and_occupies_its_end_in_the_lidar_layer_only(self):
        grid = FusionGrid({'lidar_beam_stride': 1})
        angles, ranges = beams((0.0, 0.6))
        grid.update_scan(ROBOT, angles, ranges, 0.05, 3.0, now=0.0)
        self.assertGreaterEqual(value_at(grid, 'high', 0.6, 0.0), grid.l['occupied'])
        self.assertLess(value_at(grid, 'high', 0.3, 0.0), 0)
        self.assertFalse(grid.low.any())  # the LiDAR says nothing about what lies below its plane

    def test_a_lidar_ray_passing_over_a_low_obstacle_does_not_erase_it(self):
        grid = FusionGrid({'lidar_beam_stride': 1})
        for k in range(4):
            grid.update_range(ROBOT, 0.4, 0.26, now=k * 0.1)
        self.assertGreaterEqual(value_at(grid, 'low', 0.4, 0.0), grid.l['occupied'])
        before = value_at(grid, 'low', 0.4, 0.0)
        angles, ranges = beams((0.0, 0.9))
        for k in range(10):
            grid.update_scan(ROBOT, angles, ranges, 0.05, 3.0, now=0.3)  # same time: no fading in between
        self.assertEqual(value_at(grid, 'low', 0.4, 0.0), before)
        self.assertTrue(grid.low_obstacle_cells()[int((0.0 - grid.origin[1]) / grid.res),
                                                   int((0.4 - grid.origin[0]) / grid.res)])


class UltrasonicModelTests(unittest.TestCase):
    def lidar_sees_free_ahead(self, grid):
        angles = np.linspace(-0.4, 0.4, 81)
        grid.update_scan(ROBOT, angles, np.full(81, 0.9), 0.05, 3.0, now=0.0)

    def test_one_echo_is_not_an_obstacle_but_consistent_echoes_are(self):
        grid = FusionGrid()
        self.lidar_sees_free_ahead(grid)
        self.assertEqual(grid.update_range(ROBOT, 0.4, 0.26, now=0.0), 'low_obstacle')
        self.assertFalse(grid.low_obstacle_cells().any())  # a stray echo alone never blocks the path
        for k in range(1, 3):
            grid.update_range(ROBOT, 0.4, 0.26, now=k * 0.1)
        self.assertTrue(grid.low_obstacle_cells().any())
        pts = grid.points(grid.low_obstacle_cells())
        self.assertTrue(np.all(np.abs(np.hypot(pts[:, 0], pts[:, 1]) - 0.4) <= 0.04))  # on the arc

    def test_an_echo_the_lidar_explains_goes_to_the_lidar_obstacle_not_the_whole_arc(self):
        grid = FusionGrid({'lidar_beam_stride': 1})
        angles, ranges = beams((0.08, 0.5), (0.0, 0.9), (-0.08, 0.9), count=720)
        for k in range(2):
            grid.update_scan(ROBOT, angles, ranges, 0.05, 3.0, now=0.0)
        self.assertEqual(grid.update_range(ROBOT, 0.5, 0.26, now=0.0), 'lidar_obstacle')
        self.assertGreater(value_at(grid, 'low', 0.5 * math.cos(0.08), 0.5 * math.sin(0.08)), 0)
        self.assertLessEqual(value_at(grid, 'low', 0.5 * math.cos(-0.08), 0.5 * math.sin(-0.08)), 0)
        self.assertFalse(grid.low_obstacle_cells().any())  # it is the obstacle the LiDAR sees, not a low one

    def test_an_echo_where_the_lidar_has_not_looked_is_unresolved(self):
        self.assertEqual(FusionGrid().update_range(ROBOT, 0.4, 0.26, now=0.0), 'unresolved')

    def test_far_echoes_and_no_echo_only_clear_the_cone(self):
        grid = FusionGrid()
        self.assertEqual(grid.update_range(ROBOT, 0.7, 0.26, now=0.0), 'far')  # may be the floor
        self.assertEqual(grid.update_range(ROBOT, math.inf, 0.26, now=0.0), 'free')
        self.assertEqual(grid.update_range(ROBOT, math.nan, 0.26, now=0.0), 'unsure')
        self.assertFalse((grid.low > 0).any())
        self.assertLess(value_at(grid, 'low', 0.3, 0.0), 0)
        self.assertEqual(value_at(grid, 'low', 0.0, 0.5), 0)  # outside the cone: unknown

    def test_off_and_without_association(self):
        self.assertEqual(FusionGrid({'use_ultrasonic': False}).update_range(ROBOT, 0.4, 0.26, now=0.0), 'off')
        self.assertEqual(FusionGrid({'associate': False}).update_range(ROBOT, 0.4, 0.26, now=0.0), 'echo')


class GridKeepingTests(unittest.TestCase):
    def test_evidence_fades_and_the_lidar_layer_fades_faster(self):
        grid = FusionGrid({'lidar_beam_stride': 1})
        angles, ranges = beams((0.0, 0.6))
        grid.update_scan(ROBOT, angles, ranges, 0.05, 3.0, now=0.0)
        for k in range(5):
            grid.update_range(ROBOT, 0.4, 0.26, now=0.0)
        grid.decay(10.0)
        self.assertLess(value_at(grid, 'high', 0.6, 0.0), grid.l['occupied'])
        self.assertTrue(grid.low_occupied().any())  # a low obstacle is remembered while the robot goes around
        grid.decay(120.0)
        self.assertFalse(grid.low_occupied().any())

    def test_recentering_keeps_marks_where_they_are_in_the_world(self):
        grid = FusionGrid({'lidar_beam_stride': 1})
        angles, ranges = beams((0.0, 0.6))
        grid.update_scan(ROBOT, angles, ranges, 0.05, 3.0, now=0.0)
        grid.recenter(0.3, -0.1)
        np.testing.assert_allclose(grid.center(), [0.3, -0.1], atol=grid.res / 2)
        self.assertGreaterEqual(value_at(grid, 'high', 0.6, 0.0), grid.l['occupied'])
        grid.recenter(5.0, 5.0)  # far away: all unknown
        self.assertFalse(grid.high.any())

    def test_occupancy_grid_data_is_unknown_until_observed(self):
        grid = FusionGrid({'lidar_beam_stride': 1})
        self.assertTrue(np.all(grid.occupancy_data() == -1))
        angles, ranges = beams((0.0, 0.6))
        grid.update_scan(ROBOT, angles, ranges, 0.05, 3.0, now=0.0)
        data = grid.occupancy_data().reshape(grid.n, grid.n)
        i, j = int((0.0 - grid.origin[1]) / grid.res), int((0.6 - grid.origin[0]) / grid.res)
        self.assertGreaterEqual(data[i, j], 70)
        self.assertLess(data[i, int((0.3 - grid.origin[0]) / grid.res)], 50)

    def test_settings_are_checked(self):
        for bad in ({'nope': 1}, {'us_hit_p': 0.4}, {'us_free_p': 0.6}, {'associate': 1},
                    {'us_mark_max_range_m': 0.9}, {'size_m': 0.2}, {'clamp_occupied_p': 0.6}):
            with self.assertRaises(ValueError, msg=bad):
                validated(bad)

    def test_local_points_and_clearing_ring(self):
        np.testing.assert_allclose(to_local(np.array([[1.0, 1.0]]), (1.0, 0.0, math.pi / 2)), [[1.0, 0.0]], atol=1e-9)
        points = ring(0.98, 0.037)
        self.assertEqual(len(points), 180)
        self.assertTrue(all(abs(math.hypot(x, y) - 0.98) < 1e-9 and z == 0.037 for x, y, z in points))


# -- offline evaluation on a synthetic recording -----------------------------------------------------------------
ROOM = (-1.0, 1.5, -0.8, 0.8)  # x min, x max, y min, y max: walls the LiDAR sees


def ray_to_room(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    hits = []
    if c > 1e-9: hits.append((ROOM[1] - x) / c)
    if c < -1e-9: hits.append((ROOM[0] - x) / c)
    if s > 1e-9: hits.append((ROOM[3] - y) / s)
    if s < -1e-9: hits.append((ROOM[2] - y) / s)
    return min(hits)


def recording(box=True, spikes=(), seconds=6.0, speed=0.1, seed=0):
    """Robot drives straight ahead from the origin. The low box (face at x = 0.95) is under the LiDAR plane."""
    rng = np.random.default_rng(seed)
    records = [dict(type='tf_static', parent='base_footprint', child='laser', x=0.0, y=0.0, z=0.125, yaw=0.0),
               dict(type='tf_static', parent='base_footprint', child='us_link', x=0.05, y=0.0, z=0.037, yaw=0.0)]
    angles = -math.pi + np.arange(360) * 2 * math.pi / 360
    for k in range(int(seconds * 20) + 1):
        t = k / 20
        x = speed * t
        records.append(dict(type='odom', t=t, x=x, y=0.0, yaw=0.0))
        if k % 2 == 0:
            ranges = [ray_to_room(x, 0.0, a) + float(rng.normal(0, 0.005)) for a in angles]
            records.append(dict(type='scan', t=t + 0.001, frame='laser', angle_min=-math.pi,
                                angle_increment=2 * math.pi / 360, range_min=0.05, range_max=3.0, ranges=ranges))
        else:
            face = (0.95 if box else ROOM[1]) - x - 0.05
            value = face + float(rng.normal(0, 0.01))
            if any(abs(t - spike) < 1e-6 for spike in spikes):
                value = 0.3  # a stray echo (crosstalk, a cable): nothing is there
            records.append(dict(type='range', t=t, frame='us_link', range=value, fov=0.26, min=0.02, max=2.0))
    return records


class OfflineComparisonTests(unittest.TestCase):
    def test_the_low_box_is_found_only_with_the_ultrasonic_and_the_fused_grid_finds_it_cleanly(self):
        results = compare(recording(box=True), box=(1.0, 0.0))
        self.assertFalse(results['lidar_only']['detected'])  # under the LiDAR plane: invisible to it
        self.assertTrue(results['fused']['detected'])
        self.assertGreater(results['fused']['robot_to_box_at_detection_m'], 0.5)  # found well before reaching it
        self.assertLessEqual(results['fused']['false_positive_s'], results['ultrasonic_binary']['false_positive_s'])
        self.assertIn('low_obstacle', results['fused']['echo_counts'])

    def test_a_stray_echo_makes_a_false_obstacle_only_without_accumulation(self):
        results = compare(recording(box=False, spikes=(1.05, 2.55, 4.05)))
        self.assertGreater(results['ultrasonic_binary']['false_positive_s'], 0)
        self.assertEqual(results['fused']['false_positive_s'], 0)
        self.assertEqual(results['fused']['max_false_cells'], 0)
        self.assertIsNone(results['fused']['detected'])  # an empty run has no box to find

    def test_the_recorded_mounting_chain_is_composed(self):
        frames = StaticFrames([dict(type='tf_static', parent='base_footprint', child='base_link', x=0.1, y=0.0, z=0.02, yaw=math.pi / 2),
                               dict(type='tf_static', parent='base_link', child='laser', x=0.05, y=0.0, z=0.1, yaw=0.0)])
        x, y, yaw, z = frames.in_base('laser')
        self.assertAlmostEqual(x, 0.1)
        self.assertAlmostEqual(y, 0.05)
        self.assertAlmostEqual(yaw, math.pi / 2)
        self.assertAlmostEqual(z, 0.12)
        self.assertIsNone(frames.in_base('camera'))


class NoiseMeasurementTests(unittest.TestCase):
    def test_noise_statistics_and_the_suggested_sigmas(self):
        stats = noise_stats([0.40, 0.42, None, 0.41, math.inf], expected=0.40)
        self.assertEqual((stats['n'], stats['valid'], stats['dropout']), (5, 3, 0.4))
        self.assertAlmostEqual(stats['mean_m'], 0.41)
        self.assertAlmostEqual(stats['std_m'], 0.01)
        self.assertAlmostEqual(stats['bias_m'], 0.01)
        rows = [dict(lidar=dict(std_m=0.004), ultrasonic=dict(std_m=0.012)),
                dict(lidar=dict(std_m=0.002), ultrasonic=dict(std_m=0.02))]
        self.assertEqual(suggested_sigmas(rows), {'lidar_sigma_m': 0.005, 'us_sigma_m': 0.02})

    def test_noise_from_a_still_recording_uses_the_forward_beam_and_the_mountings(self):
        records = recording(box=True, speed=0.0, seconds=3.0)
        result = noise(records, distance=0.95)
        self.assertAlmostEqual(result['ultrasonic']['mean_m'], 0.90, delta=0.01)  # sensor 5 cm ahead of the centre
        self.assertAlmostEqual(result['ultrasonic']['bias_m'], 0.0, delta=0.01)
        self.assertAlmostEqual(result['lidar']['mean_m'], 1.5, delta=0.01)  # the LiDAR looks over the box at the wall
        self.assertLess(result['lidar']['std_m'], 0.01)

    def test_the_command_line_reads_recordings(self):
        import json, tempfile, io, contextlib
        with tempfile.TemporaryDirectory() as tmp:
            path = f'{tmp}/run.jsonl'
            with open(path, 'w') as handle:
                for record in recording(box=True):
                    handle.write(json.dumps(record) + '\n')
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(fusion_eval.main(['compare', path, '--box', '1.0,0.0', '--json', f'{tmp}/r.json']), 0)
            self.assertIn('fused', out.getvalue())
            with open(f'{tmp}/r.json') as handle:
                self.assertTrue(json.load(handle)['fused']['detected'])


if __name__ == '__main__':
    unittest.main()
