#!/usr/bin/env python3
"""로봇 내부에서 /scan·/odom이 실제로 들어오는지 ROS CLI 디스커버리 데몬 없이 확인한다.

실행 위치: 로봇 (app._base_diagnostics_command가 base64로 실어 python3 -c로 exec한다. 파일로 남지 않는다)
사용처: 주행준비 중 PC 관제가 /scan·/odom을 받지 못했을 때 app.py가 원인을 가리려고 부른다.

`ros2 topic echo` 같은 CLI는 ros2 daemon의 디스커버리 캐시에 기대므로 Wi-Fi 재연결 뒤 엉뚱한 결과를 낼 수 있다.
그래서 rclpy 노드를 직접 만들어 8초 안에 샘플이 하나라도 오는지 본다. 결과는 줄마다
`SCAN_SAMPLE_OK|MISSING`, `ODOM_SAMPLE_OK|MISSING`으로 찍고, app.py는 둘 다 OK인데 PC가 못 받으면
"로봇 쪽은 정상, PC 쪽 DDS 수신이 끊김"으로 판정한다.
"""
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('pinky_base_topic_probe')
    received = set()
    # 센서 데이터 QoS(BEST_EFFORT) 구독은 RELIABLE·BEST_EFFORT 발행자 어느 쪽과도 매칭되므로 발행 QoS와 상관없이 받는다.
    subscriptions = [
        node.create_subscription(LaserScan, '/scan', lambda _: received.add('SCAN'), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', lambda _: received.add('ODOM'), qos_profile_sensor_data),
    ]
    try:
        # 8초: 라이다가 막 켜졌어도 첫 스캔이 올 만큼의 여유. 두 토픽을 다 받으면 바로 끝낸다.
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
