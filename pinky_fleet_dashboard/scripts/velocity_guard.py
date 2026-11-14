#!/usr/bin/env python3
"""Independent 20Hz final gate. Only this node normally publishes /cmd_vel."""
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
        self.config=geometry(self.runtime['geometry']);self.sensor_pose_source='configured' if self.config['sensor_pose_measured'] else 'unmeasured_default';self.memory=LowObstacleMemory(self.config)
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.scan_points=[];self.last_scan_at=0.;self.last_sensor_at=0.;self.valid_range=False;self.range_m=None
        self.command=Twist();self.command_at=0.;self.range_seq=0;self.memory_seq=-1
        self.publisher=self.create_publisher(Twist,'/cmd_vel',1)
        self.status=self.create_publisher(String,'/safety/status',10)
        self.journal=ControlJournal(CACHE/'logs/final-guard.jsonl')
        self.create_subscription(Twist,'/cmd_vel_selected',self.on_command,1)
        self.create_subscription(LaserScan,'/scan',lambda msg:UltrasonicSafety.on_scan(self,msg),qos_profile_sensor_data)
        self.create_subscription(Range,'/us_sensor/range',self.on_range,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
        self.lease_receiver=LeaseReceiver()
        self.create_subscription(String,'/fleet/lease',self.on_lease,qos_profile_sensor_data)
        self.shutting_down=False
        self.last_status=0.
    def on_lease(self,msg):
        try:self.lease_receiver.receive(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):pass
    def on_command(self,msg):self.command,self.command_at=msg,time.monotonic()
    def on_range(self,msg):
        age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        self.range_timestamped=bool(msg.header.stamp.sec or msg.header.stamp.nanosec)
        if self.range_timestamped and not -.2<=age<=.6:return
        v=float(msg.range);self.range_m=v if math.isfinite(v) else None
        self.valid_range=(math.isinf(v) and v>0) or (math.isfinite(v) and msg.min_range<=v<=msg.max_range)
        self.last_sensor_at=time.monotonic();self.range_seq+=1
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
        if self.shutting_down:
            self.publisher.publish(Twist());return
        now=time.monotonic();odom=UltrasonicSafety.fresh_pose(self,'odom');pose=UltrasonicSafety.fresh_pose(self,'map')
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
        obstacles=self.scan_points+(self.memory.local(odom) if odom else [])
        v,w,state=gate(self.command.linear.x,self.command.angular.z,fresh=fresh,
                       stop=(CACHE/'stop_latch.json').exists(),context_valid=live and traffic,
                       points=obstacles,config=self.config)
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.publisher.publish(msg)
        self.journal.record('final_velocity',state=state,robot_id=self.runtime['robot_id'],run_id=context.get('token'),
                            requested=[self.command.linear.x,self.command.angular.z],applied=[v,w],odom=odom,
                            ultrasound=self.range_m,geometry=self.config,traffic_zones=zones)
        if now-self.last_status>.2:
            self.last_status=now
            self.status.publish(String(data=json.dumps(dict(version='final_gate_v1',state=state,applied=[v,w],
                        traffic=traffic,sensors_fresh=fresh,geometry=self.config,sensor_pose_source=self.sensor_pose_source,range_timestamped=getattr(self,'range_timestamped',False)))))


def main():
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
