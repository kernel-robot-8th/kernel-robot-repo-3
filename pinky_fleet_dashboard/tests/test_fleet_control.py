import base64,gzip,json,math,re,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from scripts.motion_gate import geometry,range_arc,swept_footprint,gate,LowObstacleMemory
from scripts.mission_behaviors import UltrasonicDetour
from scripts.traffic import TrafficCoordinator,LeaseReceiver,lease_allows,required_zones
from scripts.fleet_ops import operational_status,REQUIRED
from scripts.runtime_bundle import install_command

class GeometryTests(unittest.TestCase):
    def test_real_mount_offset_and_fov_are_used(self):
        points=range_arc(.1,geometry({'ultrasound_x_m':.05,'ultrasound_y_m':.01}))
        self.assertAlmostEqual(points[4][0],.15);self.assertAlmostEqual(points[4][1],.01)
        self.assertNotEqual(points[0],points[-1])
    def test_footprint_includes_wheels_on_turn(self):
        self.assertFalse(swept_footprint([(.01,.065)],0.,.25))
        self.assertFalse(swept_footprint([(.10,0.)],.04,0.))
        self.assertTrue(swept_footprint([(.4,.4)],.03,.1))
    def test_bad_geometry_is_rejected(self):
        for g in [{'half_width_m':.01},{'padding_m':-.01},{'ultrasound_x_m':math.nan}]:
            with self.assertRaises(ValueError):geometry(g)
    def test_memory_tracks_odom_and_clears_visible_free_beam(self):
        memory=LowObstacleMemory();memory.update((0,0,0),.1,1.)
        self.assertGreater(len(memory.points),1)
        self.assertAlmostEqual(max(x for x,y in memory.local((.05,0,0))),.05)
        memory.update((0,0,0),None,2.)
        self.assertEqual(memory.points,[])
    def test_final_gate_stops_for_each_invalid_input(self):
        args=dict(fresh=True,stop=False,context_valid=True,points=[])
        self.assertEqual(gate(.05,.1,**args),(.05,.1,'allowed'))
        for override in [dict(fresh=False),dict(stop=True),dict(context_valid=False),dict(points=[(.01,.065)])]:
            self.assertEqual(gate(.05,.1,**{**args,**override})[:2],(0.,0.))
        self.assertEqual(gate(math.nan,0.,**args)[:2],(0.,0.))

class RejoinTests(unittest.TestCase):
    def test_full_detour_returns_to_line_then_requires_fresh_center_samples(self):
        robot=UltrasonicDetour();robot.start((0.,0.,0.),.1,[],0.,require_line=True)
        pose=[0.,0.,0.];seen=set();max_y=0.;now=0.
        for _ in range(1600):
            center=abs(pose[1])<.01
            reading=.1 if abs(pose[2])<.02 and pose[0]<.01 and abs(pose[1])<.01 else None
            m=robot.update(pose,reading,[],now,lambda p,v,w:swept_footprint(p,v,w),line_center=center,line_sample_at=now)
            seen.add(robot.phase);max_y=max(max_y,pose[1])
            if robot.phase in {'idle','fault'}:break
            dt=.05;v,w=m.linear,m.angular;yaw=pose[2]
            if abs(w)>1e-6:
                pose[0]+=v/w*(math.sin(yaw+w*dt)-math.sin(yaw));pose[1]+=v/w*(math.cos(yaw)-math.cos(yaw+w*dt))
            else:pose[0]+=v*math.cos(yaw)*dt;pose[1]+=v*math.sin(yaw)*dt
            pose[2]+=w*dt;now+=dt
        self.assertEqual(robot.phase,'idle')
        self.assertTrue({'return_turn','returning','line_align','line_confirm'}<=seen)
        self.assertGreaterEqual(max_y,.1);self.assertLess(abs(pose[1]),.02);self.assertLess(abs(pose[2]),.04)
    def test_one_frozen_ir_sample_cannot_release_rejoin(self):
        d=UltrasonicDetour();d.start((0.,0.,0.),.1,[],0.,require_line=True);d.phase='line_confirm'
        for now in (1.,1.2,1.4,1.6):d.update((.3,0.,0.),None,[],now,lambda *a:True,line_center=True,line_sample_at=1.)
        self.assertEqual(d.phase,'line_confirm')
        for now in (1.8,2.):d.update((.3,0.,0.),None,[],now,lambda *a:True,line_center=True,line_sample_at=now)
        self.assertEqual(d.phase,'idle')
    def test_rejoin_has_deadline_if_no_line(self):
        d=UltrasonicDetour();d.start((0.,0.,0.),.1,[],0.,require_line=True);d.phase='line_confirm'
        self.assertEqual(d.update((.3,0.,0.),None,[],61.,lambda *a:True).state,'ultrasonic_fault')

class TrafficTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'owners.json'
        self.zones={'parking':{},'junction7':dict(x_m=1.,y_m=1.,request_radius_m=.4),'dock5':dict(x_m=2.,y_m=2.)}
        self.c=TrafficCoordinator(self.path,self.zones)
    def tearDown(self):self.tmp.cleanup()
    def obs(self,token='a',zones=None,pose=(.65,1.),fresh=True,**kw):
        return dict(token=token,zones=['junction7'] if zones is None else zones,pose=pose,fresh=fresh,**kw)
    def test_two_robots_cannot_own_same_junction(self):
        g=self.c.update({'10':self.obs(),'11':self.obs('b')})
        self.assertEqual(g['10'],['junction7']);self.assertEqual(g['11'],[])
        self.assertEqual(self.c.snapshot()['waiting']['11'],['junction7'])
    def test_missing_owner_stays_reserved_across_restart(self):
        self.c.update({'10':self.obs()});self.c=TrafficCoordinator(self.path,self.zones)
        g=self.c.update({'10':self.obs(fresh=False),'11':self.obs('b')})
        self.assertEqual(g['11'],[]);self.assertEqual(self.c.owners['junction7']['robot_id'],'10')
    def test_fresh_exit_releases_to_waiter(self):
        self.c.update({'10':self.obs(),'11':self.obs('b')})
        g=self.c.update({'10':self.obs(zones=[],pose=(0,0)),'11':self.obs('b')})
        self.assertEqual(g['11'],['junction7'])
    def test_explicit_release_requires_fresh_exit(self):
        self.c.update({'10':self.obs()})
        with self.assertRaises(ValueError):self.c.release('junction7','10','a',self.obs(fresh=False))
        with self.assertRaises(ValueError):self.c.release('junction7','10','a',self.obs(pose=(1,1)))
        self.c.release('junction7','10','a',self.obs(pose=(0,0),zones=[]))
        self.assertFalse(self.c.owners)
    def test_completed_parking_releases_corridor(self):
        self.c.update({'10':self.obs(zones=['parking'],pose=(.4,.4))})
        self.c.update({'10':self.obs(zones=[],pose=(.4,.4),parked=True)})
        self.assertFalse(self.c.owners)
    def test_corrupt_state_never_grants_new_permission(self):
        self.path.write_text('{bad');c=TrafficCoordinator(self.path,self.zones)
        with self.assertRaises(ValueError):c.update({'10':self.obs()})
    def test_failed_write_rolls_back_ownership(self):
        with patch.object(self.c,'save',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.c.update({'10':self.obs()})
        self.assertFalse(self.c.owners)
    def test_range_gate_requests_before_entry(self):
        self.assertEqual(required_zones({'transit':True},(0,0),self.zones),['parking'])
        self.assertEqual(required_zones({},(.65,1.),self.zones),['junction7'])
    def test_expiry_wrong_route_and_reordered_packets_stop(self):
        r=LeaseReceiver();data=dict(token='a',zones=['junction7'],boot='pc1',sequence=1)
        self.assertTrue(r.receive(data,10.));self.assertTrue(lease_allows(r.lease,'a',['junction7'],10.1))
        self.assertFalse(lease_allows(r.lease,'b',['junction7'],10.1))
        self.assertFalse(lease_allows(r.lease,'a',['junction7'],12.))
        self.assertFalse(r.receive(data,12.))
        self.assertTrue(r.receive({**data,'boot':'pc2'},12.))
        self.assertFalse(r.receive({**data,'sequence':99},12.1))

class OperationsTests(unittest.TestCase):
    def test_preparation_history_is_not_live_readiness(self):
        robot=dict(stage='prepared',ssh_reachable=False)
        s=operational_status(robot,{})
        self.assertEqual(s['state'],'disconnected');self.assertTrue(s['last_preparation_ok']);self.assertFalse(s['ready_for_motion'])
    def test_ready_requires_both_new_processes_and_all_fresh_topics(self):
        topics={k:dict(fresh=True,value={}) for k in REQUIRED}
        topics['line_control']['value']={'line_algorithm':'fleet_control_v4'}
        topics['safety_status']['value']={'version':'final_gate_v1'}
        r=dict(stage='prepared',ssh_reachable=True)
        self.assertTrue(operational_status(r,topics)['ready_for_motion'])
        topics['safety_status']['fresh']=False
        self.assertFalse(operational_status(r,topics)['ready_for_motion'])
    def test_bundle_small_complete_and_not_plaintext_password(self):
        root=Path(__file__).resolve().parent.parent
        command=install_command(root,dict(geometry=geometry(),traffic_zones={}),{'enabled':False})
        self.assertLess(len(command.encode()),100_000)
        import shlex
        program=shlex.split(command)[2]
        encoded=re.search(r"base64.b64decode\('([^']+)'\)",program).group(1)
        files=json.loads(gzip.decompress(base64.b64decode(encoded)))
        self.assertTrue({'nav2/velocity_guard.py','nav2/motion_gate.py','nav2/runtime_config.json','nav2/traffic.py'}<=files.keys())
        for name,text in files.items():
            if name.endswith('.py'):compile(text,name,'exec')


class FinalGuardWiringTests(unittest.TestCase):
    def test_final_process_itself_stops_stale_collision_and_permission_loss(self):
        import ast
        from types import SimpleNamespace as NS
        root=Path(__file__).resolve().parent.parent
        cls=next(n for n in ast.parse((root/'scripts/velocity_guard.py').read_text()).body if isinstance(n,ast.ClassDef))
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
        context={'token':'a','monotonic':99.9}
        class Cache:
            def __init__(self,name=''):self.name=name
            def __truediv__(self,name):return Cache(name)
            def exists(self):return False
            def read_text(self):return json.dumps(context)
        class Twist:
            def __init__(self):self.linear=NS(x=0.);self.angular=NS(z=0.)
        ns=dict(time=NS(monotonic=lambda:100.),CACHE=Cache(),json=json,Twist=Twist,
                UltrasonicSafety=NS(fresh_pose=lambda self,frame:(0.,0.,0.)),required_zones=required_zones,
                lease_allows=lease_allows,gate=gate)
        exec(compile(ast.Module(body=[tick],type_ignores=[]),'final_guard_wiring','exec'),ns)
        for condition in ['allowed','stale','collision','expired']:
            sent=[];memory=LowObstacleMemory();cmd=Twist();cmd.linear.x=.03
            fake=NS(shutting_down=False,runtime={'robot_id':'10','traffic_zones':{}},config=geometry(),memory=memory,
                    lease_receiver=NS(lease=dict(token='a',zones=[],expires_monotonic=99. if condition=='expired' else 101.)),
                    command=cmd,command_at=99. if condition=='stale' else 99.99,last_scan_at=99.99,last_sensor_at=99.99,
                    valid_range=True,range_seq=0,memory_seq=0,scan_points=[(.01,.065)] if condition=='collision' else [],
                    range_m=None,publisher=NS(publish=lambda m:sent.append((m.linear.x,m.angular.z))),
                    journal=NS(record=lambda *a,**kw:None),last_status=100.)
            ns['tick'](fake)
            self.assertEqual(sent[-1],(.03,0.) if condition=='allowed' else (0.,0.))

class RecoveryRunnerTests(unittest.TestCase):
    def test_nav2_is_cancelled_while_heartbeat_continues_then_resumed(self):
        import ast
        from types import SimpleNamespace as NS
        root=Path(__file__).resolve().parent.parent
        tree=ast.parse((root/'scripts/navigate_route.py').read_text())
        nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in {'detour_wait','RouteFailure'}]
        events=[];heartbeats=[];cancelled=[];phases=iter(['returning','line_confirm','idle'])
        node=NS(active_goal=NS(cancel_goal_async=lambda:cancelled.append(True)),interrupted=False,pose=lambda *a:(0,0,0))
        ns=dict(time=__import__('time'),json=json,rclpy=NS(ok=lambda:True,spin_once=lambda *a,**k:None),
                mission_feedback=lambda token:{'avoidance':{'phase':next(phases)}},print=lambda value,**kw:events.append(value))
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'recovery_runner','exec'),ns)
        ns['detour_wait'](node,lambda pose:heartbeats.append(pose),'a')
        self.assertEqual(cancelled,[True]);self.assertEqual(len(heartbeats),3)
        self.assertIn('recovering',events[0]);self.assertIn('driving',events[-1]);self.assertIsNone(node.active_goal)


class JournalTests(unittest.TestCase):
    def test_invalid_numeric_input_is_logged_without_breaking_control(self):
        from scripts.control_journal import ControlJournal
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'trace.jsonl';journal=ControlJournal(path)
            journal.record('command',state='invalid_command',requested=[math.nan,math.inf])
            journal.close()
            record=json.loads(path.read_text())
            self.assertEqual(record['requested'],[None,None])

class ReadinessChainTests(unittest.TestCase):
    def test_stale_map_to_odom_edge_cannot_look_like_fresh_map_pose(self):
        import app,threading,time
        from types import SimpleNamespace as NS
        now=time.time()
        fake=NS(base_frames={'10':'base_footprint'},transforms={'10':{
            'base_footprint':dict(parent='odom',x=0.,y=0.,yaw=0.,received_at=now,static=False),
            'odom':dict(parent='map',x=0.,y=0.,yaw=0.,received_at=now-30,static=False)}},data={'10':{}})
        app.Telemetry._update_tf_pose_unlocked(fake,'10',now)
        self.assertEqual(fake.data['10']['tf_pose']['received_at'],now-30)
    def test_reservation_does_not_release_on_invalid_localization(self):
        import app
        topics={'tf_pose':dict(fresh=True,age_s=.1,value={'x_m':0.,'y_m':0.}),
                'line_control':dict(fresh=True,age_s=.1,value={'traffic_request':{'token':'a','zones':[]}}),
                'safety_status':dict(fresh=True,value={'sensors_fresh':False})}
        self.assertFalse(app._traffic_observations({'10':topics})['10']['fresh'])

if __name__=='__main__':unittest.main()
