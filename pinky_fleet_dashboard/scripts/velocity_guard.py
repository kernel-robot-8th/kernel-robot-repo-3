#!/usr/bin/env python3
"""독립된 20 Hz 최종 속도 감시기(final guard). 평소에는 이 노드만 /cmd_vel을 발행한다.

실행 위치: 로봇 (주행준비 때 runtime_bundle로 올라가고, app.py가 `velocity_guard` 단계로 따로 띄움)
사용처: 노드 이름 `pinky_final_guard`. navigate_route.py는 출발 전에 /cmd_vel 발행자가 이 노드 하나뿐인지 확인한다.

모터 명령 경로: Nav2 `/cmd_vel_nav2` → 행동 선택(ultrasonic_safety.py) `/cmd_vel_selected` → 이 노드 → `/cmd_vel`.
발행자를 하나로 두는 까닭: 둘이 번갈아 쓰면 모터는 마지막에 온 명령을 따르므로 한쪽의 정지가 다른 쪽 명령에 덮인다
(예전에 실제로 두 발행자가 경쟁했다). 행동 선택과 프로세스를 나눠 두어 그쪽이 죽거나 멈춰도 여기서 명령이 0.3초
넘게 끊긴 것을 보고 멈춘다. 예외는 작업자 정지(stop_robot.py)로, 0 속도를 /cmd_vel에 직접 잠깐 낸다.

매 틱 정지 조건은 motion_gate.gate 순서를 따른다: 명령이 숫자가 아님 → 작업자 정지(`stop_latch.json`) →
경로 heartbeat(1.5초)·구역 허가 없음 → 명령(0.3초)·라이다·초음파(0.6초)·odom·map TF가 오래됨 → 앞으로 갈 자리에
장애물(swept footprint). 통과하면 선속도 ±0.12 m/s, 각속도 ±0.30 rad/s로 자른다.
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
# PC 테스트처럼 패키지로 import되면 상대 import, 로봇에서 파일로 바로 실행되면 같은 폴더(nav2/)의 모듈을 쓴다.
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

# 로봇 런타임 폴더. navigate_route.py·stop_robot.py·app.py와 여기 JSON 파일로 상태를 주고받는다.
CACHE=Path('/home/pinky/.cache/pinky-fleet')

class FinalVelocityGuard(Node):
    """/cmd_vel_selected를 받아 정지 조건을 검사한 뒤 /cmd_vel로 내보내는 최종 감시 노드."""
    def __init__(self):
        super().__init__('pinky_final_guard')
        self.runtime=json.loads((CACHE/'nav2/runtime_config.json').read_text())
        # config: 차체·초음파 장착 치수(motion_gate.geometry가 검증). sensor_pose_source: 초음파 장착 위치의 출처
        # ('configured' 실측값, 'unmeasured_default' 기본값, on_range에서 TF로 찾으면 'tf').
        # memory: 초음파가 본 낮은 장애물을 odom 좌표로 기억한다. 기억 시간 1~90 s, 기억할 거리 5~25 cm로 제한한다.
        self.config=geometry(self.runtime['geometry']);self.sensor_pose_source='configured' if self.config['sensor_pose_measured'] else 'unmeasured_default';_nav=self.runtime.get('navigation') or {};self.memory=LowObstacleMemory(self.config,
            keep_s=min(90.,max(1.,float(_nav.get('ultrasonic_memory_s',90.)))),mark_m=min(.25,max(.05,float(_nav.get('ultrasonic_mark_m',.25)))))
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.scan_points=[];self.last_scan_at=0.;self.last_sensor_at=0.;self.valid_range=False;self.range_m=None
        self.command=Twist();self.command_at=0.;self.range_seq=0;self.memory_seq=-1
        # 큐 깊이 1: 밀린 옛 속도 명령은 쓸모가 없고 가장 최근 것만 의미가 있다.
        self.publisher=self.create_publisher(Twist,'/cmd_vel',1)
        self.status=self.create_publisher(String,'/safety/status',10)
        self.journal=ControlJournal(CACHE/'logs/final-guard.jsonl')
        self.create_subscription(Twist,'/cmd_vel_selected',self.on_command,1)
        # 센서 토픽은 발행자에 맞춰 sensor_data QoS(best effort). on_scan은 UltrasonicSafety의 메서드를 이 노드를 self로
        # 넘겨 빌려 쓴다: 라이다 점을 base_footprint 좌표로 바꿔 self.scan_points·last_scan_at에 둔다.
        self.create_subscription(LaserScan,'/scan',lambda msg:UltrasonicSafety.on_scan(self,msg),qos_profile_sensor_data)
        self.create_subscription(Range,'/us_sensor/range',self.on_range,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)  # 20 Hz
        # 관제가 0.25초마다 보내는 공유 구역 허가(best effort). 하나 잃어도 곧 다음 것이 온다.
        self.lease_receiver=LeaseReceiver()
        self.create_subscription(String,'/fleet/lease',self.on_lease,qos_profile_sensor_data)
        self.yield_at=-100.  # 마지막 양보 명령을 받은 시각(monotonic). -100은 받은 적 없음
        self.create_subscription(String,'/fleet/yield',self.on_yield,qos_profile_sensor_data)
        self.shutting_down=False
        self.last_status=0.
    def on_lease(self,msg):
        """허가 JSON을 LeaseReceiver에 넘긴다. 중복·역순 heartbeat는 거기서 버린다."""
        try:self.lease_receiver.receive(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):pass
    def on_yield(self,msg):
        """관제의 양보 명령(id가 있는 JSON)을 받은 시각만 기억한다. 실제 후진은 행동 선택 노드가 한다."""
        try:
            if isinstance(json.loads(msg.data).get('id'),str):self.yield_at=time.monotonic()
        except (ValueError,TypeError,AttributeError):pass
    # 행동 선택 노드가 낸 속도와 받은 시각. 0.3초 넘게 새 명령이 없으면 tick이 멈춘다.
    def on_command(self,msg):self.command,self.command_at=msg,time.monotonic()
    def on_range(self,msg):
        """초음파 값을 받는다. stamp가 있는데 나이가 -0.2~0.6 s 밖이면 버린다(stamp가 0인 드라이버는 받은 시각으로만
        검사한다). +inf는 반사 없음이라 유효, NaN은 무효로 본다."""
        age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        self.range_timestamped=bool(msg.header.stamp.sec or msg.header.stamp.nanosec)
        if self.range_timestamped and not -.2<=age<=.6:return
        v=float(msg.range);self.range_m=v if math.isfinite(v) else None
        self.valid_range=(math.isinf(v) and v>0) or (math.isfinite(v) and msg.min_range<=v<=msg.max_range)
        self.last_sensor_at=time.monotonic();self.range_seq+=1
        # 장착 위치를 실측하지 않았으면 Range 프레임의 TF로 장착 위치·방향·시야각을 채운다. yaw는 쿼터니언의 z축 회전.
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
        """20 Hz: 정지 조건을 모아 motion_gate.gate에 넘기고 그 결과를 /cmd_vel로 낸다. 어떤 경우에도 매 틱 발행한다."""
        if self.shutting_down:
            self.publisher.publish(Twist());return
        now=time.monotonic();odom=UltrasonicSafety.fresh_pose(self,'odom');pose=UltrasonicSafety.fresh_pose(self,'map')
        # odom·map 자세도 UltrasonicSafety.fresh_pose를 빌린다: TF가 0.6초 안이면 (x, y, yaw), 아니면 None.
        # route_context.json은 navigate_route.py가 경로를 도는 동안 계속 고쳐 쓰는 heartbeat다. 1.5초 넘게
        # 바뀌지 않았거나 mission_wait(관제 신호를 기다림)이면 live=False로 멈춘다.
        context={};lease={}
        try:context=json.loads((CACHE/'route_context.json').read_text())
        except (OSError,ValueError):pass
        if not isinstance(context,dict):context={}
        try:live=0<=now-float(context.get('monotonic',-100))<1.5 and not context.get('mission_wait')
        except (ValueError,TypeError):live=False
        # 지금 위치·구간으로 들어가야 할 공유 구역과, 관제의 허가가 이 경로 토큰에 그 구역을 모두 주는지.
        zones=required_zones(context,pose,self.runtime['traffic_zones']) if pose else []
        traffic=lease_allows(self.lease_receiver.lease,context.get('token'),zones,now)
        fresh=(now-self.command_at<.30 and now-self.last_scan_at<.6 and now-self.last_sensor_at<.6
               and self.valid_range and odom is not None and pose is not None)
        # 새 초음파 값이 들어왔을 때만(range_seq가 바뀜) 기억을 갱신한다.
        if odom and self.valid_range and now-self.last_sensor_at<.6 and self.memory_seq!=self.range_seq:
            self.memory.update(odom,self.range_m,now);self.memory_seq=self.range_seq
        # 5번 적재 자리로 들어갈 때(dock_entry)는 초음파 기억을 빼고 본다: 거기서는 낮은 적재대가 가까이 있는 것이 정상이다.
        docking_in=bool(context.get('dock_entry')) and live
        scan=self.scan_points
        if docking_in:
            # 적재 자리는 바로 앞에 있다(2026-11-16): 차체 폭 옆에 있는 것만 장애물로 친다.
            # 안전 가정: 이때 차체 폭(+1 cm) 안 정면의 라이다 점은 거리와 상관없이 모두 빠진다.
            width=self.config['half_width_m']+.01
            scan=[(px,py) for px,py in scan if not (px>0 and abs(py)<=width)]
        obstacles=scan+(self.memory.local(odom) if odom and not docking_in else [])
        # 차체 앞 얼마 거리의 장애물에서 핑키를 세울지(navigation.json guard_front_reach_m, 2~20 cm로 제한).
        # 설정이 없으면 1초 동안 갈 거리를 본다.
        reach=((getattr(self,'runtime',None) or {}).get('navigation') or {}).get('guard_front_reach_m')
        reach=min(.20,max(.02,float(reach))) if isinstance(reach,(int,float)) and not isinstance(reach,bool) else None
        # 관제의 말에 따른 양보(scripts/yielding.py, 2026-11-24): 이 핑키가 기다리는 구역의 허가가 아직 없어도 곧은
        # 후진은 통과시킨다(그 구역 가장자리에서 구역을 바라보고 서 있으므로 후진하면 구역에서 멀어진다). 순수한
        # 후진만, 1초 안의 새 명령이 있을 때만 허용하고, swept footprint 검사는 평소처럼 한다.
        giving_way=(0<=now-getattr(self,'yield_at',-100.)<=1.0 and self.command.linear.x<0 and abs(self.command.angular.z)<=.01)
        v,w,state=gate(self.command.linear.x,self.command.angular.z,fresh=fresh,
                       stop=(CACHE/'stop_latch.json').exists(),context_valid=live and (traffic or giving_way),
                       points=obstacles,config=self.config,reach_m=reach)
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.publisher.publish(msg)
        # final-guard.jsonl: 요청 속도, 적용 속도, 정지 이유(state). 같은 state 반복은 ControlJournal이 5 Hz로 줄인다.
        self.journal.record('final_velocity',state=state,robot_id=self.runtime['robot_id'],run_id=context.get('token'),
                            requested=[self.command.linear.x,self.command.angular.z],applied=[v,w],odom=odom,
                            ultrasound=self.range_m,geometry=self.config,traffic_zones=zones)
        # /safety/status는 5 Hz로 낸다. app.py가 safety_status로 구독해 화면에 보여 준다.
        if now-self.last_status>.2:
            self.last_status=now
            self.status.publish(String(data=json.dumps(dict(version='final_gate_v1',state=state,applied=[v,w],
                        traffic=traffic,sensors_fresh=fresh,geometry=self.config,sensor_pose_source=self.sensor_pose_source,range_timestamped=getattr(self,'range_timestamped',False)))))


def main():
    """노드를 돌린다. SIGTERM·SIGINT를 직접 받아 shutting_down을 켜고, 끝나기 전에 0 속도를 세 번 보내 모터가
    마지막 명령으로 계속 달리지 않게 한다."""
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
