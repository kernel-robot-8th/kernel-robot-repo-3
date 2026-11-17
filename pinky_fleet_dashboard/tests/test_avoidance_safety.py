"""Offline regressions: no ROS graph, SSH, or hardware is used."""
import ast
import io
import json
import math
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import app
from scripts.line_following import swept_clear
from scripts.mission_behaviors import Crossing, UltrasonicDetour


class DetourTests(unittest.TestCase):
    def test_detour_measures_ten_cm_offset_before_passing(self):
        detour = UltrasonicDetour()
        pose, now, max_lateral = (0., 0., 0.), 0., 0.
        detour.start(pose, .10, [], now)
        visited = set()
        for _ in range(1200):
            # A narrow beam loses sight of the original low obstacle on turning.
            reading = .10 if abs(pose[2]) < .02 and pose[0] < .01 and abs(pose[1]) < .01 else None
            motion = detour.update(pose, reading, [], now, swept_clear)
            visited.add(detour.phase)
            if detour.phase == 'passing':
                self.assertGreaterEqual(pose[1], .10)
            v, w, dt = motion.linear, motion.angular, .05
            yaw = pose[2]
            if abs(w) > 1e-9:
                x = pose[0]+v/w*(math.sin(yaw+w*dt)-math.sin(yaw))
                y = pose[1]+v/w*(-math.cos(yaw+w*dt)+math.cos(yaw))
            else:
                x, y = pose[0]+v*math.cos(yaw)*dt, pose[1]+v*math.sin(yaw)*dt
            pose = x, y, yaw+w*dt
            max_lateral = max(max_lateral, pose[1])
            now += dt
            if detour.phase in {'idle', 'fault'}:
                break
        self.assertEqual(detour.phase, 'idle')
        self.assertTrue({'stopped','turning','offset','aligning','passing'} <= visited)
        self.assertGreaterEqual(max_lateral, .10)
        self.assertGreaterEqual(pose[0], .22)
        self.assertLess(abs(pose[2]), .04)

    def test_all_motion_phases_respect_lidar_and_footprint(self):
        for phase in ['turning', 'offset', 'aligning', 'passing']:
            detour = UltrasonicDetour(); detour.start((0.,0.,0.), .1, [], 0.)
            detour.phase = phase
            pose = (0.,0.,.5) if phase != 'passing' else (0.,.1,0.)
            # Point beside the wheel, outside the ultrasound's forward ray.
            motion = detour.update(pose, None, [(.01,.065)], 1., swept_clear)
            self.assertEqual((motion.linear,motion.angular), (0.,0.))
            self.assertEqual(motion.state, 'ultrasonic_clearance_wait')

    def test_known_low_obstacle_blocks_motion_even_when_lidar_cannot_see_it(self):
        detour = UltrasonicDetour(); detour.start((0.,0.,0.), .05, [], 0.)
        detour.phase = 'turning'
        motion = detour.update((0.,0.,0.), None, [], 1., swept_clear)
        self.assertEqual((motion.linear,motion.angular),(0.,0.))
        self.assertEqual(motion.state,'ultrasonic_clearance_wait')

    def test_chooses_less_occupied_side_and_has_finite_budget(self):
        detour = UltrasonicDetour();detour.start((0.,0.,0.), .1, [(.1,.2)], 0.)
        self.assertEqual(detour.direction,-1.)
        motion=detour.update((0.,0.,0.), None, [], 61., swept_clear)
        self.assertEqual((motion.linear,motion.angular,motion.state),(0.,0.,'ultrasonic_fault'))


class GuardTests(unittest.TestCase):
    def methods(self):
        tree=ast.parse((app.ROOT/'scripts/ultrasonic_safety.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='UltrasonicSafety')
        selected=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in {'tick','avoidance_tick'}]
        class CachePath:
            def __init__(self, name):self.name=name
            def exists(self):return False
            def read_text(self):return json.dumps({'monotonic':time.monotonic()-.01,'token':'route'})
        ns=dict(time=time,json=json,Path=CachePath,UltrasonicDetour=UltrasonicDetour,swept_clear=swept_clear)
        exec(compile(ast.Module(body=selected,type_ignores=[]),'guard_test','exec'),ns)
        return ns

    def test_no_lidar_or_odometry_no_detour_motion(self):
        methods=self.methods()
        for pose,scan_at in [(None,time.monotonic()),((0.,0.,0.),0.)]:
            sent=[]
            fake=SimpleNamespace(mission_token=None,fresh_pose=lambda frame:pose,last_scan_at=scan_at,
                line_state='',send=lambda v=0.,w=0.:sent.append((v,w)))
            methods['avoidance_tick'](fake,time.monotonic(),{'token':'route'})
            self.assertEqual(sent,[(0.,0.)])
            self.assertEqual(fake.line_state,'ultrasonic_geometry_unavailable')

    def test_context_expiry_stops_live_detour(self):
        methods=self.methods();sent=[]
        class MissingPath:
            def __init__(self,name):pass
            def exists(self):return False
            def read_text(self):raise FileNotFoundError
        methods['Path']=MissingPath
        fake=SimpleNamespace(sensor_fresh=lambda now:True,sensor_valid=True,geometry={},state='offset',detour=UltrasonicDetour(),
            send=lambda v=0.,w=0.:sent.append((v,w)))
        methods['tick'](fake)
        self.assertEqual(sent,[(0.,0.)]);self.assertEqual(fake.state,'idle')


class StopRaceTests(unittest.TestCase):
    def test_cancellation_stays_pending_until_stop_worker_ack(self):
        state=app.FleetState()
        state.routes['10']={'status':'starting','run_id':'old','message':'starting'}
        entered,release=threading.Event(),threading.Event()
        def ssh(*args,**kwargs):entered.set();release.wait(3);return True,'STOP_LATCHED'
        process=SimpleNamespace(stdout=io.StringIO('ROUTE_CANCELLED\n'),wait=lambda:130)
        worker=threading.Thread(target=app._cancel_waypoint_route,args=('10',))
        def launch(*args,**kwargs):
            state.routes['10'].update(status='cancel_requested',stop_pending=True)
            worker.start()
            self.assertTrue(entered.wait(1))
            return process
        with patch.multiple(app,STATE=state,_ssh_run=ssh,_ssh_askpass_env=lambda:(None,{}),_route_remote_command=lambda *a:'offline'),patch.object(app.subprocess,'Popen',side_effect=launch):
            try:
                app._run_waypoint_route('10',[{'label':'주차장'}])
                self.assertEqual(state.routes['10']['status'],'cancel_requested')
                self.assertTrue(state.routes['10']['stop_pending'])
            finally:release.set();worker.join()
        self.assertEqual(state.routes['10']['status'],'cancelled')
        self.assertFalse(state.routes['10']['stop_pending'])

    def test_cancelled_queued_start_never_opens_ssh(self):
        state=app.FleetState();state.routes['10']={'status':'cancelled','run_id':'old'}
        with patch.object(app,'STATE',state),patch.object(app.subprocess,'Popen') as popen:
            app._run_waypoint_route('10',[{'label':'주차장'}])
            popen.assert_not_called()

    def test_unconfirmed_stop_keeps_restart_blocked_and_allows_retry(self):
        state=app.FleetState();state.routes['10']={'status':'cancel_requested','run_id':'old','stop_pending':True}
        with patch.multiple(app,STATE=state,_ssh_run=lambda *a,**kw:(False,'offline')):
            app._cancel_waypoint_route('10')
        self.assertEqual(state.routes['10']['status'],'cancel_requested')
        self.assertFalse(state.routes['10']['stop_pending'])


if __name__=='__main__':unittest.main()
