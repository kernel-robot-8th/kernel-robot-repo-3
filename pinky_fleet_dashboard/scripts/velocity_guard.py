#!/usr/bin/env python3
"""독립된 20 Hz 최종 속도 감시기(pinky_final_guard). 평소에는 이 노드만 /cmd_vel을 발행한다.

실행 위치: 로봇 (app.py가 `velocity_guard` 단계로 띄움). 경로: /cmd_vel_nav2 → ultrasonic_safety → /cmd_vel_selected → 이 노드.
발행자가 둘이면 한쪽 정지가 덮인다(실제 경쟁 사고). 행동 선택과 프로세스를 나눠, 그쪽이 멈춰도 명령이 0.3초 끊기면 선다.
예외: stop_robot.py는 0 속도를 직접 낸다. 정지 조건 순서는 motion_gate.gate. 자세한 설명: pinky_fleet_dashboard/README.md
"""
import json,math,time,signal,threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Range
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
if __package__:
    from .ultrasonic_safety import UltrasonicSafety
    from .motion_gate import geometry,gate,LowObstacleMemory
    from .traffic import required_zones,lease_allows,LeaseReceiver
    from .control_journal import ControlJournal
else:
    from ultrasonic_safety import UltrasonicSafety
    from motion_gate import geometry,gate,LowObstacleMemory
    from traffic import required_zones,lease_allows,LeaseReceiver
    from control_journal import ControlJournal

CACHE=Path('/home/pinky/.cache/pinky-fleet')

class FinalVelocityGuard(Node):
    def __init__(self):
        super().__init__('pinky_final_guard')
        self.runtime=json.loads((CACHE/'nav2/runtime_config.json').read_text())
        # memory: 초음파가 본 낮은 장애물 기억(1~90 s, 5~25 cm로 제한).
        self.config=geometry(self.runtime['geometry']);self.sensor_pose_source='configured' if self.config['sensor_pose_measured'] else 'unmeasured_default';_nav=self.runtime.get('navigation') or {};self.memory=LowObstacleMemory(self.config,
            keep_s=min(90.,max(1.,float(_nav.get('ultrasonic_memory_s',90.)))),mark_m=min(.25,max(.05,float(_nav.get('ultrasonic_mark_m',.25)))))
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.scan_points=[];self.last_scan_at=0.;self.last_sensor_at=0.;self.valid_range=False;self.range_m=None
        self.command=Twist();self.command_at=0.;self.range_seq=0;self.memory_seq=-1
        self.publisher=self.create_publisher(Twist,'/cmd_vel',1)
        self.status=self.create_publisher(String,'/safety/status',10)
        self.journal=ControlJournal(CACHE/'logs/final-guard.jsonl')
        self.create_subscription(Twist,'/cmd_vel_selected',self.on_command,1)
        # on_scan은 UltrasonicSafety의 메서드를 빌려 쓴다.
        self.create_subscription(LaserScan,'/scan',lambda msg:UltrasonicSafety.on_scan(self,msg),qos_profile_sensor_data)
        self.create_subscription(Range,'/us_sensor/range',self.on_range,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
        # 관제 구역 허가(0.25초마다, best effort).
        self.lease_receiver=LeaseReceiver()
        self.create_subscription(String,'/fleet/lease',self.on_lease,qos_profile_sensor_data)
        self.yield_at=-100.
        self.create_subscription(String,'/fleet/yield',self.on_yield,qos_profile_sensor_data)
        self.shutting_down=False
        self.last_status=0.
    def on_lease(self,msg):
        try:self.lease_receiver.receive(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):pass
    def on_yield(self,msg):
        """양보 명령 받은 시각만 기억한다(후진은 행동 선택 노드가 한다)."""
        try:
            if isinstance(json.loads(msg.data).get('id'),str):self.yield_at=time.monotonic()
        except (ValueError,TypeError,AttributeError):pass
    def on_command(self,msg):self.command,self.command_at=msg,time.monotonic()
    def on_range(self,msg):
        """초음파 값. stamp 나이 -0.2~0.6 s 밖이면 버린다. +inf는 반사 없음(유효), NaN은 무효."""
        age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        self.range_timestamped=bool(msg.header.stamp.sec or msg.header.stamp.nanosec)
        if self.range_timestamped and not -.2<=age<=.6:return
        v=float(msg.range);self.range_m=v if math.isfinite(v) else None
        self.valid_range=(math.isinf(v) and v>0) or (math.isfinite(v) and msg.min_range<=v<=msg.max_range)
        self.last_sensor_at=time.monotonic();self.range_seq+=1
        # 장착 위치를 실측하지 않았으면 Range 프레임 TF로 채운다.
        if not self.config.get('sensor_pose_measured') and msg.header.frame_id:
            try:
                tf=self.tf_buffer.lookup_transform('base_footprint',msg.header.frame_id.lstrip('/'),Time()).transform
                q=tf.rotation
                self.config.update(ultrasound_x_m=tf.translation.x,ultrasound_y_m=tf.translation.y,
                                   ultrasound_yaw_rad=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                if 0<msg.field_of_view<=math.pi:self.config['ultrasound_fov_rad']=float(msg.field_of_view)
                self.memory.config=geometry(self.config)
                self.sensor_pose_source='tf'
            except (TransformException,ValueError):pass
    def tick(self):
        """20 Hz: 정지 조건을 motion_gate.gate에 넘기고 결과를 매 틱 /cmd_vel로 낸다."""
        if self.shutting_down:
            self.publisher.publish(Twist());return
        now=time.monotonic();odom=UltrasonicSafety.fresh_pose(self,'odom');pose=UltrasonicSafety.fresh_pose(self,'map')
        # route_context.json heartbeat가 1.5초 넘게 낡았거나 mission_wait이면 live=False로 멈춘다.
        context={};lease={}
        try:context=json.loads((CACHE/'route_context.json').read_text())
        except (OSError,ValueError):pass
        if not isinstance(context,dict):context={}
        try:live=0<=now-float(context.get('monotonic',-100))<1.5 and not context.get('mission_wait')
        except (ValueError,TypeError):live=False
        zones=required_zones(context,pose,self.runtime['traffic_zones']) if pose else []
        traffic=lease_allows(self.lease_receiver.lease,context.get('token'),zones,now)
        fresh=(now-self.command_at<.30 and now-self.last_scan_at<.6 and now-self.last_sensor_at<.6
               and self.valid_range and odom is not None and pose is not None)
        if odom and self.valid_range and now-self.last_sensor_at<.6 and self.memory_seq!=self.range_seq:
            self.memory.update(odom,self.range_m,now);self.memory_seq=self.range_seq
        # 5번 진입(dock_entry) 때는 초음파 기억을 뺀다(낮은 적재대가 가까운 게 정상).
        docking_in=bool(context.get('dock_entry')) and live
        scan=self.scan_points
        if docking_in:
            # 적재 자리 진입(2026-11-16): 차체 폭(+1 cm) 안 정면의 라이다 점은 거리와 상관없이 뺀다.
            width=self.config['half_width_m']+.01
            scan=[(px,py) for px,py in scan if not (px>0 and abs(py)<=width)]
        obstacles=scan+(self.memory.local(odom) if odom and not docking_in else [])
        # 정지 거리 guard_front_reach_m(2~20 cm). 없으면 1초 주행 거리.
        reach=((getattr(self,'runtime',None) or {}).get('navigation') or {}).get('guard_front_reach_m')
        reach=min(.20,max(.02,float(reach))) if isinstance(reach,(int,float)) and not isinstance(reach,bool) else None
        # 양보(yielding.py, 2026-11-24): 구역 허가가 없어도 1초 안의 순수 후진은 허용(swept 검사는 그대로).
        giving_way=(0<=now-getattr(self,'yield_at',-100.)<=1.0 and self.command.linear.x<0 and abs(self.command.angular.z)<=.01)
        v,w,state=gate(self.command.linear.x,self.command.angular.z,fresh=fresh,
                       stop=(CACHE/'stop_latch.json').exists(),context_valid=live and (traffic or giving_way),
                       points=obstacles,config=self.config,reach_m=reach)
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.publisher.publish(msg)
        self.journal.record('final_velocity',state=state,robot_id=self.runtime['robot_id'],run_id=context.get('token'),
                            requested=[self.command.linear.x,self.command.angular.z],applied=[v,w],odom=odom,
                            ultrasound=self.range_m,geometry=self.config,traffic_zones=zones)
        if now-self.last_status>.2:
            self.last_status=now
            self.status.publish(String(data=json.dumps(dict(version='final_gate_v1',state=state,applied=[v,w],
                        traffic=traffic,sensors_fresh=fresh,geometry=self.config,sensor_pose_source=self.sensor_pose_source,range_timestamped=getattr(self,'range_timestamped',False)))))


def main():
    """노드를 돌린다. SIGTERM·SIGINT 때 0 속도를 세 번 보내고 끝낸다."""
    rclpy.init();node=FinalVelocityGuard();stopped=threading.Event()
    def stop(*args):node.shutting_down=True;stopped.set()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        while rclpy.ok() and not stopped.is_set():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.shutting_down=True
        if rclpy.ok():
            for _ in range(3):node.publisher.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
