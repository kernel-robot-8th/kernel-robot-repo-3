"""No hardware: automatic localization at the parking spot and 주행준비 for every Pinky in turn."""
import json
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace as NS
from unittest.mock import patch

import app


def telemetry_with(**entries):
    data = {'10': {key: {'value': None, 'received_at': None} for key in ('amcl_pose', 'tf_pose')}}
    data['10'].update(entries)
    return NS(data=data, changed=threading.Condition())


class WaitForPoseTests(unittest.TestCase):
    home = app.PARKING['10']

    def wait(self, fake, since):
        return app.Telemetry.wait_for_pose_near(fake, '10', since, self.home['x_m'], self.home['y_m'],
                                                self.home['yaw_rad'], .05)

    def pose(self, dx=0., dyaw=0.):
        return {'x_m': self.home['x_m'] + dx, 'y_m': self.home['y_m'], 'yaw_rad': self.home['yaw_rad'] + dyaw,
                'frame_id': 'map'}

    def test_amcl_pose_after_the_send_near_parking_confirms(self):
        since = time.time()
        fake = telemetry_with(amcl_pose={'value': self.pose(.05), 'received_at': since + .5})
        self.assertEqual(self.wait(fake, since), 'amcl_pose')

    def test_map_tf_pose_also_confirms(self):
        since = time.time()
        fake = telemetry_with(tf_pose={'value': self.pose(dyaw=-.1), 'received_at': since + .5})
        self.assertEqual(self.wait(fake, since), 'tf_pose')

    def test_a_pose_from_before_the_send_does_not_count(self):
        since = time.time()
        fake = telemetry_with(amcl_pose={'value': self.pose(), 'received_at': since - 1})
        self.assertIsNone(self.wait(fake, since))

    def test_a_pose_away_from_parking_or_turned_does_not_count(self):
        since = time.time()
        far = telemetry_with(amcl_pose={'value': self.pose(.3), 'received_at': since + .5})
        turned = telemetry_with(amcl_pose={'value': self.pose(dyaw=1.), 'received_at': since + .5})
        self.assertIsNone(self.wait(far, since))
        self.assertIsNone(self.wait(turned, since))


class LocalizeAtParkingTests(unittest.TestCase):
    def monitor(self, linear=0., confirmed='amcl_pose'):
        class Monitor:
            def snapshot(self):
                return {'10': {'odom': {'fresh': True, 'value': {'linear_mps': linear, 'angular_rps': 0.}}}}

            def wait_for_pose_near(self, *args):
                return confirmed
        return Monitor()

    def test_sends_the_parking_pose_and_waits_for_amcl(self):
        sent = []
        ssh = lambda rid, command: sent.append(command) or (True, 'Published AMCL initial pose at x=0.4')
        with patch.multiple(app, TELEMETRY=self.monitor(), _ssh_run=ssh):
            self.assertEqual(app._localize_at_parking('10'), (True, 'amcl_pose'))
        home = app.PARKING['10']
        self.assertIn(f"{home['x_m']} {home['y_m']} {home['yaw_rad']}", sent[0])

    def test_a_moving_pinky_gets_no_pose(self):
        sent = []
        with patch.multiple(app, TELEMETRY=self.monitor(linear=.1), _ssh_run=lambda *a: sent.append(a) or (True, '')):
            ok, why = app._localize_at_parking('10')
        self.assertFalse(ok)
        self.assertIn('움직이는 중', why)
        self.assertEqual(sent, [])

    def test_no_pose_back_from_amcl_is_a_failure(self):
        ssh = lambda rid, command: (True, 'Published AMCL initial pose')
        with patch.multiple(app, TELEMETRY=self.monitor(confirmed=None), _ssh_run=ssh):
            ok, why = app._localize_at_parking('10')
        self.assertFalse(ok)
        self.assertIn('돌아오지 않았습니다', why)


class PrepareAllTests(unittest.TestCase):
    def serve(self, state):
        submitted = []
        workers = NS(submit=lambda fn, *args: submitted.append((fn, args)))
        server = ThreadingHTTPServer(('127.0.0.1', 0), app.Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)  # cleanups run last-in first: shut down, then close
        patcher = patch.multiple(app, STATE=state, WORKERS=workers)
        patcher.start()
        self.addCleanup(patcher.stop)

        def post(body):
            request = urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/prepare-all',
                                             data=json.dumps(body).encode(), method='POST',
                                             headers={'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(request) as response:
                    return response.status, json.load(response)
            except urllib.error.HTTPError as error:
                return error.code, json.load(error)
        return post, submitted

    def test_needs_the_password(self):
        post, submitted = self.serve(app.FleetState())
        self.assertEqual(post({})[0], 401)
        self.assertEqual(submitted, [])

    def test_prepares_the_unprepared_reachable_pinkys_in_one_job(self):
        state = app.FleetState()
        state.password = 'test-only'
        state.robots['10'].update(stage='prepared', ssh_reachable=True)
        state.robots['11'].update(stage='idle', ssh_reachable=True)
        state.robots['12'].update(stage='fault', ssh_reachable=False)
        post, submitted = self.serve(state)
        status, answer = post({'map': 'map-proto.yaml'})
        self.assertEqual(status, 202)
        self.assertEqual(answer['robots'], ['11'])
        self.assertEqual(answer['skipped'], ['10 이미 준비됨', '12 연결 안 됨'])
        self.assertEqual(state.busy_robot, 'all')
        self.assertEqual(submitted, [(app._prepare_robots_in_turn, (['11'], 'map-proto.yaml'))])
        self.assertEqual(post({})[0], 409)  # busy until the job ends

    def test_nothing_to_prepare_is_refused(self):
        state = app.FleetState()
        state.password = 'test-only'
        for robot in state.robots.values():
            robot.update(stage='prepared', ssh_reachable=True)
        post, submitted = self.serve(state)
        status, answer = post({})
        self.assertEqual(status, 409)
        self.assertIn('준비할 핑키가 없습니다', answer['error'])
        self.assertIsNone(state.busy_robot)

    def test_the_job_prepares_each_in_turn_and_frees_the_lock(self):
        state = app.FleetState()
        state.busy_robot = 'all'
        prepared = []

        def prepare(robot_id, map_name, release_busy=True):
            # _prepare_robot records its own failure as a fault and returns; the next robot still goes.
            prepared.append((robot_id, release_busy))
            state.robots[robot_id]['stage'] = 'fault' if robot_id == '11' else 'prepared'
        with patch.multiple(app, STATE=state, _prepare_robot=prepare):
            app._prepare_robots_in_turn(['10', '11', '12'], 'map-proto.yaml')
        self.assertIsNone(state.busy_robot)
        self.assertEqual(prepared, [('10', False), ('11', False), ('12', False)])


if __name__ == '__main__':
    unittest.main()


# ---- LiDAR localization (scripts/scan_localizer.py) on synthetic rooms ----

import math as _math

from scripts.scan_localizer import locate


def room(extra_walls=()):
    """A 44 x 51 cell room (0.05 m) like map-proto: walls round the edge, plus `extra_walls` cells (col, row)."""
    width, height = 44, 51
    cells = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            if col in (0, width - 1) or row in (0, height - 1) or (col, row) in extra_walls:
                cells[row * width + col] = 100
    return dict(width=width, height=height, resolution_m=0.05, origin_x_m=-0.349, origin_y_m=-0.295,
                origin_yaw_rad=0.0, cells=cells)


def cast(grid, x, y, yaw, laser=(-0.017, 0.0, _math.pi), beams=720):
    """A LaserScan from a laser mounted at `laser` on a robot at (x, y, yaw), by stepping each ray to a wall."""
    lx, ly, lyaw = laser
    ox, oy = x + _math.cos(yaw) * lx - _math.sin(yaw) * ly, y + _math.sin(yaw) * lx + _math.cos(yaw) * ly
    ranges = []
    for i in range(beams):
        angle = -_math.pi + i * 2 * _math.pi / beams
        heading, distance = yaw + lyaw + angle, 0.0
        while distance < 4.0:
            distance += 0.005
            col = int((ox + distance * _math.cos(heading) - grid['origin_x_m']) / 0.05)
            row = int((oy + distance * _math.sin(heading) - grid['origin_y_m']) / 0.05)
            if not (0 <= col < grid['width'] and 0 <= row < grid['height']) or grid['cells'][row * grid['width'] + col] == 100:
                break
        ranges.append(distance)
    return dict(angle_min=-_math.pi, angle_increment=2 * _math.pi / beams, range_min=0.05, range_max=40.0, ranges=ranges)


class ScanLocalizerTests(unittest.TestCase):
    def assertPose(self, match, x, y, yaw):
        self.assertTrue(match.ok, match.reason)
        self.assertLess(_math.hypot(match.x_m - x, match.y_m - y), 0.04)
        self.assertLess(abs(_math.atan2(_math.sin(match.yaw_rad - yaw), _math.cos(match.yaw_rad - yaw))), _math.radians(4))

    def test_finds_the_pose_in_a_room_with_a_landmark(self):
        grid = room(extra_walls={(col, 30) for col in range(1, 25)})  # a wall from the left side, halfway up
        truth = (0.53, 0.38, _math.radians(-3))
        self.assertPose(locate(grid, cast(grid, *truth), (-0.017, 0.0, _math.pi)), *truth)

    def test_a_mirror_image_room_needs_the_parking_hint_and_takes_the_heading_from_the_scan(self):
        grid = room()  # a bare rectangle: (x, y, yaw) and its half-turn about the centre fit the same
        truth = (0.53, 0.38, _math.radians(-3))
        scan = cast(grid, *truth)
        laser = (-0.017, 0.0, _math.pi)
        refused = locate(grid, scan, laser)
        self.assertFalse(refused.ok)
        self.assertIn('후보 중 실제 자리를 골라', refused.reason)
        self.assertEqual(len(refused.candidates), 2)  # the place and its half-turn, for the operator
        match = locate(grid, scan, laser, known_places=[(0.4, 0.4, None)])  # a known place near the true pose
        self.assertPose(match, *truth)
        self.assertIn('서 있을 자리', match.reason)
        # A slot that expects the robot to face pi does not take a robot the scan shows facing east.
        self.assertFalse(locate(grid, scan, laser, known_places=[(0.4, 0.4, _math.pi)]).ok)

    def test_a_scan_from_elsewhere_does_not_fit(self):
        grid = room()
        scan = cast(room(extra_walls={(c, r) for c in range(5, 40) for r in range(20, 23)}), 0.5, 0.5, 0.0)
        scan['ranges'] = [r * 0.5 for r in scan['ranges']]  # a smaller room than the map
        self.assertFalse(locate(grid, scan, (-0.017, 0.0, _math.pi), known_places=[(0.4, 0.4, None)]).ok)


class LidarLocalizeTests(unittest.TestCase):
    def test_frame_in_base_walks_the_static_mounts(self):
        fake = NS(base_frames={'10': 'base_footprint'}, lock=threading.RLock(), transforms={'10': {
            'rplidar_link': dict(parent='rplidar_mount', x=0., y=0., yaw=_math.pi, static=True),
            'rplidar_mount': dict(parent='base_link', x=-0.017, y=0., yaw=0., static=True),
            'base_link': dict(parent='base_footprint', x=0., y=0., yaw=0., static=True)}})
        x, y, yaw = app.Telemetry.frame_in_base(fake, '10', 'rplidar_link')
        self.assertAlmostEqual(x, -0.017)
        self.assertAlmostEqual(abs(yaw), _math.pi)
        self.assertIsNone(app.Telemetry.frame_in_base(fake, '10', 'unknown_link'))

    def monitor(self, grid, scan, subscribed=True, confirmed='amcl_pose'):
        sent = []

        class Monitor:
            base_frames = {'10': 'base_footprint'}

            def snapshot(self):
                return {'10': {'odom': {'fresh': True, 'value': {'linear_mps': 0., 'angular_rps': 0.}},
                               'map': {'fresh': True, 'value': grid}}}

            def scan_after(self, robot_id, after, timeout):
                return dict(scan, frame_id='rplidar_link', received_at=after + 0.1)

            def frame_in_base(self, robot_id, frame):
                return (-0.017, 0.0, _math.pi)

            def publish_initial_pose(self, robot_id, x, y, yaw):
                sent.append((x, y, yaw))
                return subscribed

            def wait_for_pose_near(self, *args):
                return confirmed
        return Monitor(), sent

    def test_finds_the_robot_and_sends_that_pose_to_amcl_without_ssh(self):
        grid = room(extra_walls={(col, 30) for col in range(1, 25)})
        monitor, sent = self.monitor(grid, cast(grid, 0.53, 0.38, 0.0))
        state = app.FleetState()
        no_ssh = lambda *a: self.fail('LiDAR localization must not use SSH')
        with patch.multiple(app, TELEMETRY=monitor, STATE=state, _ssh_run=no_ssh):
            ok, detail = app._localize_with_lidar('10')
        self.assertTrue(ok, detail)
        self.assertEqual(len(sent), 1)
        self.assertLess(_math.hypot(sent[0][0] - 0.53, sent[0][1] - 0.38), 0.04)
        self.assertTrue(state.robots['10']['localization']['applied'])

    def test_no_amcl_means_no_pose(self):
        grid = room(extra_walls={(col, 30) for col in range(1, 25)})
        monitor, _ = self.monitor(grid, cast(grid, 0.53, 0.38, 0.0), subscribed=False)
        with patch.multiple(app, TELEMETRY=monitor, STATE=app.FleetState()):
            ok, detail = app._localize_with_lidar('10')
        self.assertFalse(ok)
        self.assertIn('주행준비', detail)

    def test_localize_endpoint_runs_without_the_password(self):
        state = app.FleetState()  # no password
        submitted = []
        server = ThreadingHTTPServer(('127.0.0.1', 0), app.Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with patch.multiple(app, STATE=state, WORKERS=NS(submit=lambda fn, *a: submitted.append((fn, a)))):
            request = urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/robots/11/localize',
                                             data=b'{}', method='POST', headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request) as response:
                self.assertEqual(response.status, 202)
        self.assertEqual(submitted, [(app._relocalize, ('11', None))])
        self.assertEqual(state.busy_robot, '11')


class OperatorChoiceTests(LidarLocalizeTests):
    def test_a_mirror_image_room_waits_for_the_operator_then_takes_the_picked_place(self):
        grid = room()  # bare rectangle: two places fit
        monitor, sent = self.monitor(grid, cast(grid, 0.53, 0.38, 0.0))
        state = app.FleetState()
        with patch.multiple(app, TELEMETRY=monitor, STATE=state, TRAFFIC=NS(zones={})):
            ok, detail = app._localize_with_lidar('10')
            self.assertFalse(ok)
            self.assertIn('후보 중 실제 자리를 골라', detail)
            candidates = state.robots['10']['localization']['candidates']
            self.assertEqual(len(candidates), 2)
            self.assertEqual(sent, [])  # nothing reaches AMCL until someone picks
            mirror = next(c for c in candidates if c['y_m'] > 1.0)
            ok, detail = app._localize_with_lidar('10', (mirror['x_m'], mirror['y_m'], mirror['yaw_rad']))
        self.assertTrue(ok, detail)
        self.assertEqual(len(sent), 1)
        self.assertGreater(sent[0][1], 1.0)


class IndicatorTests(unittest.TestCase):
    def test_colours_follow_the_pinky_state(self):
        from scripts.fleet_ops import indicator
        ready = {'ready_for_motion': True}
        self.assertEqual(indicator({'stage': 'base_starting'}, None, {}), {'color': 'orange', 'mode': 'on'})
        self.assertEqual(indicator({'stage': 'prepared'}, None, {'ready_for_motion': False})['color'], 'orange')  # no pose yet
        self.assertEqual(indicator({'stage': 'prepared'}, None, ready)['color'], 'blue')
        self.assertEqual(indicator({'stage': 'prepared'}, {'status': 'running'}, ready)['color'], 'green')
        self.assertEqual(indicator({'stage': 'prepared'}, {'status': 'running', 'stop_pending': True}, ready)['color'], 'red')
        self.assertEqual(indicator({'stage': 'fault'}, None, {})['color'], 'red')
