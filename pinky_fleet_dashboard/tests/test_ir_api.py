import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import app
from scripts.ir_profiles import IRProfiles
from scripts.line_following import default_profile


class TelemetryFake:
    def __init__(self):
        self.lock = threading.RLock()
        self.ir_history = {'10': []}
        self.control = {'version': 1, 'enabled': False, 'revision': 'old'}
    def snapshot(self):
        return {'10': {'line_control': {'fresh': True, 'value': dict(self.control)}}}


class IRApiTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.store = IRProfiles(Path(self.directory.name)/'profiles.json', ['10', '11', '12'])
        self.state = app.FleetState()
        self.state.robots['10']['stage'] = 'prepared'
        self.state.password = 'test-only'
        self.telemetry = TelemetryFake()
        self.patches = [patch.object(app, 'IR_PROFILES', self.store),
                        patch.object(app, 'STATE', self.state),
                        patch.object(app, 'TELEMETRY', self.telemetry),
                        patch.object(app, '_ir_profile_install_command', lambda profile: dict(profile)),
                        patch.object(app, '_ssh_run', self.fake_ssh)]
        for p in self.patches: p.start()
    def tearDown(self):
        for p in reversed(self.patches): p.stop()
        self.directory.cleanup()
    def fake_ssh(self, robot, command):
        self.assertEqual(robot, '10')
        self.telemetry.control.update(command)
        self.telemetry.control['version'] = 1
        return True, 'IR_PROFILE_SAVED'
    def call(self, **body):
        handler = app.Handler.__new__(app.Handler)
        handler.path = '/api/robots/10/ir'
        handler._body = lambda: body
        captured = []
        handler._json = lambda code, payload: captured.append((code, payload))
        handler.do_POST()
        self.assertEqual(len(captured), 1)
        return captured[0]
    def samples(self, values):
        self.telemetry.ir_history['10'] = [(time.time(), values)]*12
    def calibrate(self):
        self.samples([3000, 2800, 2600]); self.assertEqual(self.call(action='floor')[0], 200)
        for i, value in enumerate((300, 400, 500)):
            raw = [3000, 2800, 2600]; raw[i] = value
            self.samples(raw); self.assertEqual(self.call(action='line', channel=i)[0], 200)
    def test_calibration_enable_ack_disable_and_no_auto_enable(self):
        self.calibrate()
        self.assertFalse(self.store.get('10')['enabled'])
        self.assertEqual(self.call(action='enable')[0], 200)
        self.assertTrue(self.store.get('10')['enabled'])
        self.assertEqual(self.call(action='floor')[0], 409)
        self.assertEqual(self.call(action='disable')[0], 200)
        self.assertFalse(self.store.get('10')['enabled'])
        self.assertIsNone(self.state.busy_robot)
    def test_uncalibrated_or_old_guard_cannot_enable(self):
        self.assertEqual(self.call(action='enable')[0], 409)
        self.calibrate(); self.telemetry.control['version'] = 0
        self.assertEqual(self.call(action='enable')[0], 409)
    def test_live_capture_requires_enough_samples_and_valid_channel(self):
        self.assertEqual(self.call(action='floor')[0], 400)
        self.assertEqual(self.call(action='line', channel=True)[0], 400)
        self.assertEqual(self.call(action='line', channel=4)[0], 400)
    def test_active_route_and_other_busy_robot_block_reconfiguration(self):
        self.state.routes['10'] = {'status': 'running'}
        self.assertEqual(self.call(action='floor')[0], 409)
        self.state.routes['10'] = None; self.state.busy_robot = '12'
        self.assertEqual(self.call(action='floor')[0], 409)
    def test_remote_enabled_blocks_capture_even_if_local_profile_disabled(self):
        self.telemetry.control['enabled'] = True
        self.samples([3000, 2800, 2600])
        self.assertEqual(self.call(action='floor')[0], 409)
    def test_failed_ssh_does_not_mark_profile_enabled(self):
        self.calibrate()
        with patch.object(app, '_ssh_run', return_value=(False, 'No route to host')):
            self.assertEqual(self.call(action='enable')[0], 502)
        self.assertFalse(self.store.get('10')['enabled'])
        self.assertIsNone(self.state.busy_robot)


if __name__ == '__main__': unittest.main()
