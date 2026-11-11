#!/usr/bin/env python3
"""Check live base samples without the ROS CLI discovery daemon."""
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('pinky_base_topic_probe')
    received = set()
    subscriptions = [
        node.create_subscription(LaserScan, '/scan', lambda _: received.add('SCAN'), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', lambda _: received.add('ODOM'), qos_profile_sensor_data),
    ]
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and len(received) < 2:
            rclpy.spin_once(node, timeout_sec=.1)
        for name in ('SCAN', 'ODOM'):
            print(f'{name}_SAMPLE_' + ('OK' if name in received else 'MISSING'), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
