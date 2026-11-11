"""ROS adapter smoke checks on isolated, localhost-only domain 83 (no robot)."""
import os
os.environ['ROS_DOMAIN_ID'] = '83'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
import math
import time
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
from scripts.ultrasonic_safety import UltrasonicSafety
from scripts.line_following import default_profile


class Recorder:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


rclpy.init()
node = UltrasonicSafety()
try:
    node.publisher = Recorder()
    node.line_status_publisher = Recorder()
    node.ir_profile = default_profile()
    node.ir_profile.update(enabled=True, floor=[3000., 2800., 2600.], line=[300., 400., 500.])
    node.ir_raw = [300, 2800, 2600]
    node.last_ir_at = node.last_scan_at = time.monotonic()
    node.scan_points = []
    us = Range(); us.min_range = .02; us.max_range = 3.; us.range = .8
    node.on_range(us)
    command = Twist(); command.linear.x = .048; node.on_command(command)
    node.tick()
    assert node.publisher.messages[-1].angular.z > 0
    assert 0 < node.publisher.messages[-1].linear.x <= .048
    node.last_ir_at = time.monotonic()-2
    node.tick()
    assert node.publisher.messages[-1].linear.x == 0
    node.last_ir_at = time.monotonic()
    node.state = 'turning'; node.last_command_at = time.monotonic()-2
    node.tick()
    assert node.state == 'idle'
    assert node.publisher.messages[-1].linear.x == node.publisher.messages[-1].angular.z == 0
    # A nearby object at rest must not initiate an unsolicited recovery.
    us.range = .05; node.on_range(us)
    assert node.state == 'idle'
    node.on_command(command); node.on_range(us); node.tick()
    assert node.publisher.messages[-1].linear.x == 0
    assert node.publisher.messages[-1].angular.z == 0
    # Verify scan points are transformed from the scanner to the robot base.
    q = SimpleNamespace(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4))
    t = SimpleNamespace(x=.1, y=.2)
    transform = SimpleNamespace(transform=SimpleNamespace(rotation=q, translation=t))
    scan = LaserScan(); scan.header.frame_id = 'laser'; scan.range_min=.02;scan.range_max=3.;scan.ranges=[1.]
    with patch.object(node.tf_buffer, 'lookup_transform', return_value=transform): node.on_scan(scan)
    assert abs(node.scan_points[0][0]-.1)<1e-6
    assert abs(node.scan_points[0][1]-1.2)<1e-6
    node.publish_line_status()
    status = json.loads(node.line_status_publisher.messages[-1].data)
    assert status['version']==1
    print('PASS: ROS adapter, left steering sign, stale stop, no-command stop, ultrasound priority, scan TF, status; isolated domain 83')
finally:
    node.destroy_node()
    rclpy.shutdown()
