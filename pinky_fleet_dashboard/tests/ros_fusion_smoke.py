"""ROS smoke check of the fusion node (scripts/fusion_grid.py) on isolated, localhost-only domain 83 (no robot).

    source /opt/ros/jazzy/setup.bash && PYTHONPATH=".:$PYTHONPATH" python3 tests/ros_fusion_smoke.py

Starts the node as the robot would, feeds it TF, a LiDAR that sees a wall 0.9 m ahead and an ultrasonic that keeps
echoing at 0.4 m (a box under the LiDAR plane), and checks what Nav2 and the dashboard would receive."""
import os
os.environ['ROS_DOMAIN_ID'] = '83'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
node_process = subprocess.Popen([sys.executable, str(ROOT / 'scripts/fusion_grid.py')], env=os.environ.copy())
rclpy.init()
node = rclpy.create_node('fusion_smoke')
received = {}
node.create_subscription(String, '/fusion/status', lambda m: received.__setitem__('status', json.loads(m.data)), 10)
node.create_subscription(PointCloud2, '/fusion/low_marks', lambda m: received.__setitem__('marks', m), qos_profile_sensor_data)
node.create_subscription(PointCloud2, '/fusion/low_clear', lambda m: received.__setitem__('clear', m), qos_profile_sensor_data)
node.create_subscription(OccupancyGrid, '/fusion/grid', lambda m: received.__setitem__('grid', m), 1)
scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
range_pub = node.create_publisher(Range, '/us_sensor/range', qos_profile_sensor_data)
static = StaticTransformBroadcaster(node)


def transform(parent, child, x, z):
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.z = x, z
    t.transform.rotation.w = 1.0
    return t


static.sendTransform([transform('odom', 'base_footprint', 0.0, 0.0), transform('base_footprint', 'laser', 0.0, 0.125),
                      transform('base_footprint', 'us_link', 0.05, 0.037)])
try:
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser', node.get_clock().now().to_msg()
        scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max = -math.pi, 2 * math.pi / 360, 0.05, 3.0
        scan.ranges = [0.9 / math.cos(-math.pi + k * 2 * math.pi / 360) if math.cos(-math.pi + k * 2 * math.pi / 360) > 0.7
                       else math.inf for k in range(360)]
        scan_pub.publish(scan)
        echo = Range()
        echo.header.frame_id, echo.header.stamp = 'us_link', node.get_clock().now().to_msg()
        echo.field_of_view, echo.min_range, echo.max_range, echo.range = 0.26, 0.02, 2.0, 0.35
        range_pub.publish(echo)
        rclpy.spin_once(node, timeout_sec=0.1)
        status = received.get('status') or {}
        if status.get('low_obstacle_cells') and 'grid' in received and received.get('marks') is not None \
                and received['marks'].width:
            break
    status = received.get('status')
    assert status, 'no /fusion/status: the node did not start'
    assert status['low_obstacle_cells'] > 0, status
    assert status['last_echo']['kind'] == 'low_obstacle', status
    assert 0.35 <= status['nearest_low_obstacle_m'] <= 0.45, status
    marks = list(point_cloud2.read_points(received['marks'], field_names=('x', 'y', 'z')))
    assert received['marks'].header.frame_id == 'base_footprint'
    assert all(0.33 <= math.hypot(p[0], p[1]) <= 0.47 and p[2] <= 0.045 for p in marks), marks[:5]
    ring = list(point_cloud2.read_points(received['clear'], field_names=('x', 'y', 'z')))
    assert len(ring) == 180 and all(p[2] <= 0.045 for p in ring)
    grid = received['grid']
    assert grid.header.frame_id == 'odom' and grid.info.width == grid.info.height == 100
    print('fusion smoke ok:', json.dumps({k: status[k] for k in ('low_obstacle_cells', 'nearest_low_obstacle_m', 'timing')}))
finally:
    node.destroy_node()
    rclpy.shutdown()
    node_process.terminate()
    node_process.wait(timeout=5)
