#!/usr/bin/env python3
"""Seed AMCL at the map origin after the operator physically places the robot."""

import time
import sys
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node


def main() -> int:
    x,y,yaw=map(float,sys.argv[1:4])
    if not all(math.isfinite(v) for v in (x,y,yaw)): return 2
    rclpy.init(args=[])
    node = Node("pinky_dashboard_initial_pose")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
    deadline = time.monotonic() + 15.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() < 1:
        node.get_logger().error("AMCL is not subscribed to /initialpose")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    message = PoseWithCovarianceStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "map"
    message.pose.pose.position.x, message.pose.pose.position.y = x,y
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
    message.pose.covariance[0] = 0.04
    message.pose.covariance[7] = 0.04
    message.pose.covariance[35] = 0.03
    for _ in range(3):
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f"Published AMCL initial pose at x={x}, y={y}, yaw={yaw} in map")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
