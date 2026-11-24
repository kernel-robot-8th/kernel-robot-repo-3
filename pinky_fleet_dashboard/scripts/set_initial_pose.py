#!/usr/bin/env python3
"""운영자가 로봇을 실제로 그 자리에 놓은 뒤 AMCL에 초기 위치를 넣어 준다.

실행 위치: 로봇 (app._initial_pose_command가 ~/.cache/pinky-fleet/set_initial_pose.py로 올리고 바로 실행한다)
사용처: app._localize_at_parking — 핑키가 자기 주차 칸에 서 있다고 보고 그 칸의 좌표(x, y, yaw)를 인자로 넘긴다.

원래는 지도 원점 (0, 0, 0°)에 놓고 쓰던 스크립트라 이름이 그렇지만, 지금은 인자로 받은 map 좌표를 쓴다.
로봇을 움직이지 않으며 /initialpose를 세 번 보낸다. 이후 AMCL이 라이다로 추정을 고칠 수 있다.
인자: x_m y_m yaw_rad (단위 m, m, rad). 값이 유한하지 않거나 AMCL이 구독하지 않으면 종료 코드 2.
app.py는 출력의 "Published AMCL initial pose"를 보고 성공을 판단하므로 그 로그 문구를 바꾸면 안 된다.
"""

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
    # args=[]: 위치 인자(x y yaw)를 ROS 인자로 해석하지 않게 한다.
    rclpy.init(args=[])
    node = Node("pinky_dashboard_initial_pose")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    # VOLATILE이라 구독자가 붙기 전에 보낸 메시지는 사라진다. get_subscription_count()는 QoS가 맞아 연결된
    # 구독만 세므로, AMCL이 실제로 받을 수 있게 될 때까지 최대 15초 기다린 뒤 보낸다.
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
    # yaw만 있는 2D 자세를 쿼터니언으로: z=sin(yaw/2), w=cos(yaw/2).
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
    # 6x6 공분산의 대각: [0]=x 분산, [7]=y 분산 (0.04 m² → 표준편차 0.2 m), [35]=yaw 분산 (0.03 rad² → 약 0.17 rad).
    # 사람이 칸에 세운 오차만큼만 퍼뜨려 AMCL 파티클이 그 근처에서 시작하게 한다.
    message.pose.covariance[0] = 0.04
    message.pose.covariance[7] = 0.04
    message.pose.covariance[35] = 0.03
    # BEST_EFFORT는 유실될 수 있으므로 같은 자세를 세 번 보낸다.
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
