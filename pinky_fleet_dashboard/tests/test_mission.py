"""No hardware: the free test mission (start → 5 → drop-off → own parking) and its start checks."""
import json
import math
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace as NS
from unittest.mock import patch

import app
from scripts.route_geometry import build_mission_route

MAP = 'map-proto.yaml'


def wp(label, x, y, yaw=0.0):
    return dict(label=label, map=MAP, frame_id='map', x_m=x, y_m=y, yaw_rad=yaw)


class MissionRouteTests(unittest.TestCase):
    def test_only_the_stops_and_a_dock_exit_along_the_docks_own_heading(self):
        dock = wp('5', 0.39, 1.64, math.radians(-171.9))
        route = build_mission_route([wp('주차장', 1.0, 0.45, 0.0), dock, wp('수령 A', 1.4, 2.0), wp('주차장', 1.0, 0.4, math.pi)])
        self.assertEqual([p['label'] for p in route], ['주차장', '5 정면', '5', '도킹 탈출', '수령 A', '주차장'])
        self.assertTrue(all(p['anchor'] for p in route))  # nothing in between for the robot to follow
        square, five, exit_ = route[1], route[2], route[3]
        # squares up 25 cm in front of 5 along the dock's heading, then drives straight in
        self.assertAlmostEqual(math.hypot(square['x_m'] - 0.39, square['y_m'] - 1.64), 0.25)
        self.assertAlmostEqual(square['arrival_yaw_rad'], math.radians(-171.9))
        self.assertAlmostEqual(five['yaw_rad'], math.radians(-171.9))  # not the parking→5 line (117°)
        self.assertAlmostEqual(math.hypot(exit_['x_m'] - 0.39, exit_['y_m'] - 1.64), 0.20)
        self.assertTrue(exit_['dock_exit'] and exit_['reverse'])


class MissionEndpointTests(unittest.TestCase):
    NAVIGATION = {'ir_steering': False}  # no passage points, no queue spot, the reverse exit: the plain mission

    def post(self, state, body):
        server = ThreadingHTTPServer(('127.0.0.1', 0), app.Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        submitted = []
        with patch.multiple(app, STATE=state, WORKERS=NS(submit=lambda fn, *a: submitted.append((fn, a))),
                            _route_start_problem=lambda rid: None, NAVIGATION=dict(self.NAVIGATION),
                            TELEMETRY=NS(snapshot=lambda: {'12': {'tf_pose': {'value': {'x_m': 1.0, 'y_m': 0.45, 'yaw_rad': 0.1}}}}),
                            TRAFFIC=NS(zones={'parking': {'bounds': [0.2, 1.2, 0.2, 0.6]}})):
            request = urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/robots/12/mission',
                                             data=json.dumps(body).encode(), method='POST',
                                             headers={'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(request) as response:
                    return response.status, json.load(response), submitted
            except urllib.error.HTTPError as error:
                return error.code, json.load(error), submitted

    def state(self):
        state = app.FleetState()
        state.robots['12']['map'] = MAP
        state.waypoints = {'10': [wp('수령 A', 1.4, 2.0)], '11': [], '12': [wp('5', 0.39, 1.64, math.pi)]}
        return state

    def test_starts_from_where_it_stands_using_a_drop_off_recorded_by_another_pinky(self):
        state = self.state()
        status, answer, submitted = self.post(state, {'dropoff': '수령 A'})
        self.assertEqual(status, 202, answer)
        goals = submitted[0][1][1]
        self.assertEqual([g['label'] for g in goals], ['주차장', '5 정면', '5', '도킹 탈출', '수령 A', '주차장'])
        self.assertAlmostEqual(goals[0]['parking_yaw_rad'], 0.1)  # leaves facing as it stands
        self.assertTrue(goals[4]['course_end'] and goals[-1]['parking_return'])  # 수령 A, after the square-up point
        self.assertEqual(state.routes['12']['execution'], 'free_mission')

    def test_an_unknown_drop_off_or_station_5_is_refused(self):
        self.assertEqual(self.post(self.state(), {'dropoff': '없는 곳'})[0], 400)
        self.assertEqual(self.post(self.state(), {'dropoff': '5'})[0], 400)


if __name__ == '__main__':
    unittest.main()


class KeepNav2Tests(unittest.TestCase):
    def run_start(self, hashes):
        sent = []
        answers = iter(hashes)

        def ssh(robot_id, command, *args, **kwargs):
            sent.append(command)
            if 'sha256sum' in command:
                return True, next(answers)
            if 'install_nav2_runtime.py' in command:
                return True, 'NAV2_RUNTIME_READY'
            if 'NAV_PROCESSES_CLEARED' in command or 'pinky_navigation bringup_launch.xml' in command and 'kill' in command:
                return True, 'NAV_PROCESSES_CLEARED'
            return True, 'STARTED SAFETY_STOPPED NAV_PROCESSES_CLEARED'
        with patch.multiple(app, _ssh_run=ssh, STATE=app.FleetState(), NAVIGATION={'ultrasonic_fusion': False}):
            result = app._start_navigation_with_safety('12', '/m.yaml', 'map-proto.yaml', keep_nav2=True)
        return result, sent

    def test_unchanged_settings_keep_the_running_nav2(self):
        (ok, output), sent = self.run_start(['abc', 'abc'])
        self.assertEqual((ok, output), (True, 'NAV2_KEPT'))
        self.assertFalse(any('bringup_launch.xml map:=' in c for c in sent))  # no new launch

    def test_changed_settings_restart_nav2(self):
        (ok, output), sent = self.run_start(['abc', 'def'])
        self.assertTrue(any('bringup_launch.xml map:=' in c for c in sent))


class ClockSyncTests(unittest.TestCase):
    def test_a_wrong_clock_is_set_with_the_password_on_stdin_only(self):
        import time as _time
        calls = []

        def ssh(robot_id, command, *args, **kwargs):
            calls.append((command, kwargs.get('send_password', False)))
            if command.startswith('date +%s'):
                return True, f'{_time.time() - 8_000_000:.3f}'  # three months behind
            if 'sudo -S' in command:
                return True, f'{_time.time():.3f}'
            return True, 'NAV_PROCESSES_CLEARED'
        state = app.FleetState(); state.password = 'secret-pw'
        with patch.multiple(app, _ssh_run=ssh, STATE=state):
            changed, note = app._sync_robot_clock('10')
        self.assertTrue(changed, note)
        sudo = [c for c in calls if 'sudo -S' in c[0]][0]
        self.assertTrue(sudo[1])                      # password goes to stdin
        self.assertNotIn('secret-pw', sudo[0])        # never in the command
        self.assertTrue(any('NAV_PROCESSES_CLEARED' in c[0] for c in calls))  # Nav2 stopped before the jump

    def test_a_close_clock_is_left_alone(self):
        import time as _time
        with patch.multiple(app, _ssh_run=lambda rid, cmd, *a, **k: (True, f'{_time.time() + 0.2:.3f}'), STATE=app.FleetState()):
            changed, note = app._sync_robot_clock('10')
        self.assertFalse(changed)
        self.assertIn('그대로', note)


class ViaTests(MissionEndpointTests):
    def test_via_waypoints_go_out_and_come_back_in_reverse(self):
        state = self.state()
        state.waypoints['11'] = [wp('9', 1.55, 0.77), wp('7', 1.55, 1.24)]
        status, answer, submitted = self.post(state, {'dropoff': '수령 A', 'via': ['9', '7']})
        self.assertEqual(status, 202, answer)
        goals = submitted[0][1][1]
        self.assertEqual([g['label'] for g in goals], ['주차장', '9', '7', '5 정면', '5', '도킹 탈출', '수령 A', '7', '9', '주차장'])
        self.assertEqual([g['label'] for g in goals if g.get('course_end')], ['수령 A'])
        self.assertEqual(self.post(self.state(), {'dropoff': '수령 A', 'via': ['없음']})[0], 400)


class Nav2BringupRetryTests(unittest.TestCase):
    LOG = ("=== PINKY_FLEET_START navigation 1 ===\n[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.\n"
           "=== PINKY_FLEET_START navigation 2 ===\n[lifecycle_manager_navigation]: Configuring planner_server\n")

    def test_only_the_latest_launch_counts_as_aborted(self):
        from scripts.nav_readiness import bringup_abort
        self.assertIsNone(bringup_abort(self.LOG))  # the abort belongs to the launch before
        aborted = self.LOG + ("[lifecycle_manager_navigation]: Failed to change state for node: planner_server. Exception: "
                              "planner_server/get_state service client: async_send_request failed.\n"
                              "[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.\n")
        self.assertIn('planner_server', bringup_abort(aborted))

    def test_the_managers_that_gave_up_are_named(self):
        from scripts.nav_readiness import aborted_managers
        log = self.LOG + (
            "[component_container_isolated-1] [ERROR] [1.0] [lifecycle_manager_localization]: Failed to change state for node: amcl. "
            "Exception: amcl/get_state service client: async_send_request failed.\n"
            "[component_container_isolated-1] [ERROR] [1.1] [lifecycle_manager_localization]: Failed to bring up all requested nodes. Aborting bringup.\n"
            "[component_container_isolated-1] [INFO] [1.2] [lifecycle_manager_navigation]: Configuring bt_navigator\n")
        managers = aborted_managers(log)
        self.assertEqual(list(managers), ['lifecycle_manager_localization'])  # navigation went on
        self.assertIn('amcl', managers['lifecycle_manager_localization'])

    def test_an_aborted_bringup_is_started_once_more(self):
        answers = iter([(False, 'NAV_BRINGUP_ABORTED {} · Failed to change state for node: planner_server'),
                        (True, 'NAV_ACTION_READY · lifecycle active')])
        state, calls = app.FleetState(), []
        with patch.multiple(app, STATE=state, _nav_action_ready=lambda rid: next(answers),
                            _stop_existing_navigation=lambda rid: calls.append('stop') or (True, 'NAV_PROCESSES_CLEARED'),
                            _start_navigation_with_safety=lambda rid, path, name: calls.append('start') or (True, 'STARTED'),
                            _topic_ready=lambda rid, topics, timeout: (True, [])):
            ready, detail = app._nav_ready_with_retry('10', '/home/pinky/map-proto.yaml', MAP)
        self.assertTrue(ready)
        self.assertEqual(calls, ['stop', 'start'])

    def test_other_failures_are_not_retried(self):
        state, calls = app.FleetState(), []
        with patch.multiple(app, STATE=state, _nav_action_ready=lambda rid: (False, 'NAV_ACTION_MISSING_OR_INACTIVE {}'),
                            _stop_existing_navigation=lambda rid: calls.append('stop') or (True, 'NAV_PROCESSES_CLEARED')):
            ready, _ = app._nav_ready_with_retry('10', '/home/pinky/map-proto.yaml', MAP)
        self.assertFalse(ready)
        self.assertEqual(calls, [])


class StopLeavesReadyTests(unittest.TestCase):
    def test_a_confirmed_stop_clears_the_route_and_the_plan_but_keeps_the_robot_prepared(self):
        state = app.FleetState()
        state.robots['10'].update(stage='prepared', localized_at=1.0)
        route = {'status': 'cancel_requested', 'labels': ['출발', '5'], 'run_id': 'r1', 'stop_pending': True}
        state.routes['10'] = route
        forgotten = []
        here = {'x_m': 1.2, 'y_m': 0.6, 'yaw_rad': 0.5}
        telemetry = NS(forget=lambda rid, key: forgotten.append((rid, key)),
                       snapshot=lambda: {'10': {'tf_pose': {'value': here, 'fresh': True}}})
        with patch.multiple(app, STATE=state, TELEMETRY=telemetry, NAVIGATION={'auto_return_after_stop_s': 60},
                            _ssh_run=lambda rid, cmd, **kw: (True, 'STOP_LATCHED {"nav_cancel_service_replied": true}')):
            app._cancel_waypoint_route('10')
        self.assertEqual(route['status'], 'cancelled')
        self.assertTrue(route['ready_again'])
        self.assertGreater(route['auto_return_at'], app.time.time() + 50)  # home after a minute without a command
        self.assertEqual(route['stopped_pose'], here)
        self.assertFalse(route['stop_pending'])
        self.assertEqual(forgotten, [('10', 'plan')])
        self.assertEqual(state.robots['10']['stage'], 'prepared')  # no new 주행준비 needed

    def test_a_new_run_wipes_the_line_of_the_run_before(self):
        state = app.FleetState()
        forgotten = []
        telemetry = NS(forget=lambda rid, key: forgotten.append((rid, key)))
        with patch.multiple(app, STATE=state, TELEMETRY=telemetry):
            app._run_waypoint_route('10', [])  # no run waiting: it returns at once, after the wipe
        self.assertEqual(forgotten, [('10', 'trail'), ('10', 'plan')])

    def test_forget_trail_empties_the_line(self):
        monitor = app.Telemetry.__new__(app.Telemetry)
        import threading as _threading
        from collections import deque
        monitor.lock = _threading.Lock()
        monitor.trails = {'10': deque([(0.0, 0.0), (0.1, 0.0)])}
        monitor.data = {'10': {}}
        monitor.forget('10', 'trail')
        self.assertEqual(list(monitor.trails['10']), [])


class BackOffTests(unittest.TestCase):
    def test_backs_off_after_standing_then_stops_when_the_front_is_clear(self):
        from scripts.mission_behaviors import BackOff
        b = BackOff(distance_m=.12, clear_m=.15, hold_s=1.0)
        self.assertIsNone(b.update(True, .04, (0., 0., 0.), 0.0, True))          # just stopped: wait first
        self.assertIsNone(b.update(True, .04, (0., 0., 0.), 0.5, True))
        m = b.update(True, .04, (0., 0., 0.), 1.1, True)
        self.assertLess(m.linear, 0)                                               # reversing straight
        self.assertEqual((m.angular, m.state), (0., 'ultrasonic_back_off'))
        self.assertLess(b.update(False, .10, (-.06, 0., 0.), 2.6, True).linear, 0)  # goes on past the 8 cm line
        self.assertIsNone(b.update(False, .16, (-.10, 0., 0.), 3.5, True))          # 15 cm in front: enough
        self.assertFalse(b.active)

    def test_never_reverses_into_something_and_gives_up_after_three_tries(self):
        from scripts.mission_behaviors import BackOff
        b = BackOff(hold_s=0.0)
        self.assertIsNone(b.update(True, .04, (0., 0., 0.), 0.0, False))           # LiDAR: not clear behind
        now = 1.0
        for _ in range(3):
            self.assertIsNotNone(b.update(True, .04, (0., 0., 0.), now, True))
            self.assertIsNone(b.update(True, .04, (-.13, 0., 0.), now + 1, True))  # the 12 cm is done
            now += 2
        self.assertIsNone(b.update(True, .04, (0., 0., 0.), now, True))            # the fourth time it waits


class ShippedCourseTests(MissionEndpointTests):
    test_starts_from_where_it_stands_using_a_drop_off_recorded_by_another_pinky = None  # the plain mission's test

    NAVIGATION = {'ir_steering': False, 'dock_exit_reverse': False, 'dock_queue': {'waypoint': '4'},
                  'wall_passages': [{'x_m': 1.57, 'wall_y_m': 1.035, 'approach_m': 0.24}]}

    def test_through_the_passage_to_the_queue_spot_then_in_and_out_forward(self):
        state = self.state()
        state.waypoints['11'] = [wp('4', 1.55, 1.64)]
        status, answer, submitted = self.post(state, {'dropoff': '수령 A'})
        self.assertEqual(status, 202, answer)
        goals = submitted[0][1][1]
        self.assertEqual([g['label'] for g in goals],
                         ['주차장', '통로 아래', '통로 위', '4', '5 정면', '5', '수령 A', '통로 위', '통로 아래', '주차장'])
        passing = {g['label']: g.get('pass_radius_m') for g in goals}
        self.assertEqual((passing['통로 아래'], passing['통로 위'], passing['4'], passing['5 정면'], passing['5']),
                         (0.10, 0.10, 0.06, 0.05, None))  # the passage points, the queue spot, and the square-up point (it turns itself)
        self.assertFalse(any(g.get('dock_exit') for g in goals))  # out of station 5 forward, no backing out


class CameraGuidedRedockTests(unittest.TestCase):
    """2026-11-17: redock carries the ceiling camera's offset; the Pinky comes in corrected by it."""
    DOCK = dict(label='5', x_m=.339, y_m=1.5226, yaw_rad=math.pi)

    def test_the_spot_is_where_the_camera_says_it_is_from_where_the_pinky_stands(self):
        from scripts.route_geometry import corrected_dock
        # Heading -x (yaw pi). The camera: 30 mm too far forward, 45 mm to the right, turned 6 degrees left.
        pose = (self.DOCK['x_m'], self.DOCK['y_m'], math.pi)
        square, spot = corrected_dock(pose, dict(forward_mm=30., left_mm=-45., dyaw_deg=6.), self.DOCK, .25)
        yaw = math.pi-math.radians(6.)
        self.assertAlmostEqual(spot['yaw_rad'], yaw)
        # Standing on the corrected spot and applying the same miss again puts the Pinky's body on the real spot.
        self.assertAlmostEqual(spot['x_m']+(.030*math.cos(yaw)+.045*math.sin(yaw)), pose[0], places=6)
        self.assertAlmostEqual(spot['y_m']+(.030*math.sin(yaw)-.045*math.cos(yaw)), pose[1], places=6)
        self.assertAlmostEqual(math.hypot(square['x_m']-spot['x_m'], square['y_m']-spot['y_m']), .25)
        self.assertAlmostEqual(math.atan2(spot['y_m']-square['y_m'], spot['x_m']-square['x_m']), yaw)  # straight in
        self.assertEqual(spot['label'], '5')

    def test_an_offset_that_is_missing_wrong_or_too_large_is_not_used(self):
        from scripts.route_geometry import clean_dock_offset, corrected_dock
        pose = (self.DOCK['x_m'], self.DOCK['y_m'], math.pi)
        for offset in (None, {}, dict(forward_mm=10., left_mm=5.), dict(forward_mm='1', left_mm=0., dyaw_deg=0.),
                       dict(forward_mm=True, left_mm=0., dyaw_deg=0.), dict(forward_mm=float('nan'), left_mm=0., dyaw_deg=0.),
                       dict(forward_mm=200., left_mm=0., dyaw_deg=0.), dict(forward_mm=0., left_mm=0., dyaw_deg=40.)):
            self.assertIsNone(clean_dock_offset(offset), offset)
            self.assertIsNone(corrected_dock(pose, offset, self.DOCK, .25), offset)
        # A usable offset, but the Pinky itself believes it is far from the waypoint: the corrected spot would be too.
        self.assertIsNone(corrected_dock((self.DOCK['x_m']+.20, self.DOCK['y_m'], math.pi),
                                         dict(forward_mm=10., left_mm=0., dyaw_deg=0.), self.DOCK, .25))
        self.assertIsNone(corrected_dock(None, dict(forward_mm=10., left_mm=0., dyaw_deg=0.), self.DOCK, .25))

    def test_the_dashboard_corrects_from_the_pinkys_fresh_pose_and_its_station_5(self):
        telemetry = type('T', (), {'snapshot': lambda self: {'12': {'tf_pose': {
            'fresh': True, 'value': {'x_m': .345, 'y_m': 1.520, 'yaw_rad': 3.10}}}}})()
        offset = dict(forward_mm=20., left_mm=-30., dyaw_deg=4.)
        with patch.object(app, 'TELEMETRY', telemetry), patch.object(app, '_shared_waypoint', lambda rid, label: dict(self.DOCK)):
            correction = app._dock_correction('12', offset)
            self.assertEqual(set(correction), {'square', 'dock'})
            self.assertAlmostEqual(correction['dock']['yaw_rad'], 3.10-math.radians(4.), places=4)
            self.assertLess(math.hypot(correction['dock']['x_m']-.345, correction['dock']['y_m']-1.520), .04)
            self.assertIsNone(app._dock_correction('12', dict(offset, forward_mm=500.)))
        stale = type('T', (), {'snapshot': lambda self: {'12': {'tf_pose': {'fresh': False, 'value': {'x_m': .345, 'y_m': 1.52, 'yaw_rad': 3.1}}}}})()
        with patch.object(app, 'TELEMETRY', stale), patch.object(app, '_shared_waypoint', lambda rid, label: dict(self.DOCK)):
            self.assertIsNone(app._dock_correction('12', offset))  # no fresh pose: come in again at the waypoint

    def test_the_runner_takes_the_corrected_points_only_when_they_are_whole(self):
        import ast
        tree = ast.parse((app.ROOT/'scripts/navigate_route.py').read_text())
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'guided_dock']
        ns = {'math': math}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), 'guided_dock', 'exec'), ns)
        point = dict(x_m=.3, y_m=1.5, yaw_rad=3.1)
        self.assertEqual(ns['guided_dock'](dict(square=point, dock=point)), [point, point])
        for bad in (None, {}, dict(square=point), dict(square=point, dock=dict(point, x_m='a')),
                    dict(square=point, dock=dict(point, yaw_rad=float('inf'))), dict(square=point, dock=[1, 2, 3])):
            self.assertIsNone(ns['guided_dock'](bad), bad)
