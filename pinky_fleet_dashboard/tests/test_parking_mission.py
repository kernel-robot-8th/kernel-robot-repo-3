"""No hardware motion: real HTTP concurrency and generated parking mission contracts."""
import concurrent.futures
import json
import threading
import unittest
import urllib.request
import urllib.error
from unittest.mock import patch
from http.server import ThreadingHTTPServer
import app

class ParkingMissionTest(unittest.TestCase):
    def test_three_independent_missions_and_completion_gate(self):
        state=app.FleetState()
        state.password='test-only'
        for rid in app.ROBOTS:
            state.robots[rid]['stage']='prepared'
        telemetry={rid:{key:{'fresh':True,'value':{'recovery_policy':'parking_mission_v2','line_algorithm':'fleet_control_v4','version':'final_gate_v1','traffic_config':app.TRAFFIC.zones}} for key in
                       ('map','tf_pose','scan','odom','ultrasonic','ir','line_control','safety_status')} for rid in app.ROBOTS}
        class Monitor:
            def snapshot(self): return telemetry
        class Profiles:
            def get(self,rid): return app.default_profile() # uncalibrated IR disabled must be supported
        submitted=[]
        class Workers:
            def submit(self,fn,*args): submitted.append((fn,args))
        sent=[]
        def ssh(rid,command):sent.append((rid,command));return True,'saved'
        with patch.multiple(app,STATE=state,TELEMETRY=Monitor(),IR_PROFILES=Profiles(),WORKERS=Workers(),_ssh_run=ssh):
            server=ThreadingHTTPServer(('127.0.0.1',0),app.Handler)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def post(rid,path,body):
                req=urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/robots/{rid}/{path}',
                    data=json.dumps(body).encode(),headers={'Content-Type':'application/json'},method='POST')
                try:
                    with urllib.request.urlopen(req) as r:return r.status,json.load(r)
                except urllib.error.HTTPError as e:return e.code,json.load(e)
            try:
                with concurrent.futures.ThreadPoolExecutor() as pool:
                    results=list(pool.map(lambda rid:post(rid,'route',{'labels':['1','2','3','7','4','5','6','7','9','8','1']}),app.ROBOTS))
                self.assertEqual([r[0] for r in results],[202]*3)
                self.assertIsNone(state.busy_robot)
                self.assertEqual(len(submitted),3)
                for _,(rid,goals) in submitted:
                    self.assertEqual(goals[0]['label'],'주차장')
                    self.assertEqual(goals[-1]['label'],'주차장')
                    self.assertEqual(goals[0]['x_m'],app.PARKING[rid]['x_m'])
                    self.assertEqual(goals[-1]['y_m'],app.PARKING[rid]['y_m'])
                    self.assertEqual(goals[-1]['yaw_rad'],app.PARKING[rid]['yaw_rad'])
                    self.assertEqual(sum(bool(p.get('course_end')) for p in goals),1)
                    self.assertTrue(goals[-1]['parking_return'])
                    self.assertTrue(all(not p['use_ir'] for p in goals))
                self.assertEqual(post('10','route',{'labels':['1','2']})[0],409)
                self.assertEqual(post('10','mission-complete',{'token':'old'})[0],409)
                self.assertEqual(post('10','route/cancel',{})[0],202)
                self.assertEqual(state.routes['10']['status'],'cancel_requested')
                self.assertEqual(state.routes['11']['status'],'starting')
                self.assertEqual(state.routes['12']['status'],'starting')
                for rid in app.ROBOTS: state.routes[rid].update(status='running',phase='course_complete',mission_token='token-'+rid)
                with concurrent.futures.ThreadPoolExecutor() as pool:
                    results=list(pool.map(lambda rid:post(rid,'mission-complete',{'token':'token-'+rid}),app.ROBOTS))
                self.assertEqual([r[0] for r in results],[202]*3)
                self.assertEqual({rid for rid,_ in sent},set(app.ROBOTS))
                self.assertEqual(post('11','mission-complete',{'token':'token-10'})[0],409)
                state.routes['10']['phase']='returning'
                self.assertEqual(post('10','mission-complete',{'token':'token-10'})[0],409)
            finally: server.shutdown();server.server_close();thread.join()


class RunnerFlowTest(unittest.TestCase):
    def test_parallel_runner_wait_then_return(self):
        import ast,base64,contextlib,io,math,tempfile,time,types,uuid
        source=(app.ROOT/'scripts/navigate_route.py').read_text()
        tree=ast.parse(source)
        selected=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ('run','angle','checkpoint','RouteFailure','mission_feedback','detour_wait')]
        code=compile(ast.Module(body=selected,type_ignores=[]),'navigate_route_flow','exec')
        def exercise(rid):
            with tempfile.TemporaryDirectory() as directory:
                cache=app.Path(directory);events=[];sent=[];wait_ticks=0
                home=app.PARKING[rid]
                points=[dict(label='주차장',x_m=home['x_m'],y_m=home['y_m'],yaw_rad=home['yaw_rad'],parking_yaw_rad=home['yaw_rad'],anchor=True,use_ir=False),
                        dict(label='1',x_m=0.,y_m=0.,yaw_rad=0.,anchor=True,use_ir=False,course_end=True),
                        dict(label='주차장',x_m=home['x_m'],y_m=home['y_m'],yaw_rad=home['yaw_rad'],anchor=True,use_ir=False,parking_return=True)]
                ns={'math':math,'time':time,'uuid':uuid,'json':json,'CACHE':cache,'GoalStatus':types.SimpleNamespace(STATUS_SUCCEEDED=4)}
                class Future:
                    def __init__(self,value):self.value=value
                    def done(self):return True
                    def result(self):return self.value
                    def exception(self):return None
                class PoseStamped:
                    def __init__(self):self.header=types.SimpleNamespace();self.pose=types.SimpleNamespace(position=types.SimpleNamespace(),orientation=types.SimpleNamespace())
                ns['PoseStamped']=PoseStamped
                ns['NavigateToPose']=types.SimpleNamespace(Goal=lambda:types.SimpleNamespace())
                class Node:
                    interrupted=False;active_goal=None;run_id=None
                    def __init__(self):self.position=(home['x_m'],home['y_m'],home['yaw_rad']);self.client=self
                    def wait_for_server(self,**kw):return True
                    def get_publishers_info_by_topic(self,*a):return [types.SimpleNamespace(node_name='pinky_final_guard')]
                    def pose(self,*a):return self.position
                    def cancel_active(self):self.active_goal=None
                    def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:None))
                    def send_goal_async(self,goal):
                        p=goal.pose.pose
                        sent.append((p.position.x,p.position.y,p.orientation.z,p.orientation.w))
                        if len(sent)==2:
                            assert any(e.get('phase')=='course_complete' for e in events)
                            assert wait_ticks>=2
                        self.position=(p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w))
                        handle=types.SimpleNamespace(accepted=True,get_result_async=lambda:Future(types.SimpleNamespace(status=4)))
                        return Future(handle)
                def atomic(name,data):
                    (cache/name).write_text(json.dumps(data))
                def spin(*a,**kw):
                    nonlocal wait_ticks
                    path=cache/'route_context.json'
                    if path.exists():
                        ctx=json.loads(path.read_text())
                        if ctx.get('mission_wait'):
                            wait_ticks+=1
                            if wait_ticks>=2:atomic('mission_complete.json',dict(token=ctx['token'],monotonic=time.monotonic()))
                ns['atomic_json']=atomic
                ns['rclpy']=types.SimpleNamespace(ok=lambda:True,spin_once=spin,spin_until_future_complete=lambda *a,**kw:None)
                def capture(line,*a,**kw):
                    if line.startswith('MISSION_STATUS '):events.append(json.loads(line.split(' ',1)[1]))
                ns['print']=capture
                exec(code,ns)
                ns['run'](Node(),points)
                self.assertEqual([e['phase'] for e in events],['course_complete','returning'])
                self.assertEqual(len(sent),2)
                self.assertEqual(sent[-1][:2],(home['x_m'],home['y_m']))
                self.assertAlmostEqual(sent[-1][2],math.sin(home['yaw_rad']/2))
                self.assertFalse((cache/'route_start.json').exists())
                return rid
        with concurrent.futures.ThreadPoolExecutor() as pool:
            self.assertEqual(set(pool.map(exercise,app.ROBOTS)),set(app.ROBOTS))

if __name__=="__main__": unittest.main()
