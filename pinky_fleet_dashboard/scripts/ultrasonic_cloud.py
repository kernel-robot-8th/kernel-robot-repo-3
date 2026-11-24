#!/usr/bin/env python3
"""(사용 안 함) 로봇 쪽: 전면 초음파 Range를 Nav2 VoxelLayer용 점으로 바꾸던 이전 라이다 + 초음파 퓨전.

실행 위치: 지금은 어디서도 실행되지 않는다(저장소에 남은 죽은 코드).
사용처: 없음. 2026-11-16 scripts/fusion_grid.py가 이 파일을 대신했다. runtime_bundle에 들어 있지 않고, 어느 코드도
  import하지 않으며, 테스트도 없다. app.py는 예전 설치에서 남아 도는 같은 이름의 프로세스를 주행준비 때 멈추기만
  한다(app.py의 _stop_script_command("ultrasonic_cloud")).

라이다는 바닥에서 약 12.5 cm 높이의 평면 하나를 보고, 초음파는 바닥에서 약 3.7 cm 높이로 앞을 보므로 라이다가
넘겨 보는 낮은 물체를 본다. costmap은 둘 다 5 cm 높이 칸에 두고, 기둥 안의 어느 칸이든 점유면 막힌 것으로 보므로
낮은 상자가 Nav2가 돌아가는 장애물이 된다.

Range는 "이 원뿔 안, 이 거리에 무언가 있다"만 말하므로, 확인된 반사는 시야 전체에 걸친 호를 표시한다(안전한 쪽).
센서 좌표계로 점군 두 개를 낸다:
  /us_sensor/cloud_mark   mark_max_range_m 안의 확인된 반사의 호 (marking 전용)
  /us_sensor/cloud_clear  측정 거리의 호, 반사가 없으면 clear_max_range_m의 호 (clearing 전용)
값은 최근 `window`개의 중앙값 둘레 agree_m 안에서 agree_count번 연속 맞아야 인정한다. 한 번 튀는 값을 막는다.
먼 반사는 표시하지 않는다: 3.7 cm 높이에서는 원뿔이 바닥에 닿는다.
설정: /home/pinky/.cache/pinky-fleet/nav2/runtime_config.json -> navigation.ultrasonic_cloud.
"""
from __future__ import annotations

import json
import math
import statistics
from collections import deque
from pathlib import Path

# 단위: m, rad. window·agree_count·arc_points는 개수다.
DEFAULTS = dict(min_range_m=0.03, mark_max_range_m=0.6, clear_max_range_m=0.8, window=5, agree_m=0.03,
                agree_count=3, arc_points=7, fov_rad=0.26)


class RangeFilter:
    """최근 값들의 중앙값을, 충분히 맞으면 확인한다. ROS가 없어 따로 테스트할 수 있다."""

    def __init__(self, settings: dict):
        self.s = {**DEFAULTS, **(settings or {})}
        self.recent: deque[float] = deque(maxlen=int(self.s["window"]))
        self.agreeing = 0
        self.last_median: float | None = None

    def update(self, value: float) -> tuple[str, float | None]:
        """표시할 확인된 반사면 ('obstacle', d), d까지 비울 때 ('free', d), 그 밖에는 ('unsure', None)."""
        if math.isnan(value):
            self.recent.clear()
            self.agreeing = 0
            return "unsure", None
        if math.isinf(value) or value > self.s["clear_max_range_m"]:
            self.recent.clear()
            self.agreeing = 0
            return "free", self.s["clear_max_range_m"]
        if value < self.s["min_range_m"]:
            return "unsure", None
        self.recent.append(value)
        median = statistics.median(self.recent)
        close = self.last_median is not None and abs(median - self.last_median) <= self.s["agree_m"]
        self.agreeing = self.agreeing + 1 if close else 1
        self.last_median = median
        if self.agreeing < self.s["agree_count"]:
            return "unsure", None
        if median <= self.s["mark_max_range_m"]:
            return "obstacle", median
        return "free", median

    def arc(self, distance: float, fov: float | None) -> list[tuple[float, float, float]]:
        """센서 좌표계(x가 앞)에서 `distance` 거리의 원뿔을 가로지르는 점들."""
        width = fov if fov and math.isfinite(fov) and 0 < fov < math.pi else self.s["fov_rad"]
        count = max(1, int(self.s["arc_points"]))
        angles = [0.0] if count == 1 else [-width / 2 + i * width / (count - 1) for i in range(count)]
        return [(distance * math.cos(a), distance * math.sin(a), 0.0) for a in angles]


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Range
    from sensor_msgs_py import point_cloud2

    try:
        runtime = json.loads(Path("/home/pinky/.cache/pinky-fleet/nav2/runtime_config.json").read_text())
        settings = (runtime.get("navigation") or {}).get("ultrasonic_cloud") or {}
    except (OSError, ValueError):
        settings = {}

    class UltrasonicCloud(Node):
        def __init__(self):
            super().__init__("pinky_ultrasonic_cloud")
            self.filter = RangeFilter(settings)
            self.mark = self.create_publisher(PointCloud2, "/us_sensor/cloud_mark", qos_profile_sensor_data)
            self.clear = self.create_publisher(PointCloud2, "/us_sensor/cloud_clear", qos_profile_sensor_data)
            self.create_subscription(Range, "/us_sensor/range", self.on_range, qos_profile_sensor_data)

        def on_range(self, msg):
            kind, distance = self.filter.update(float(msg.range))
            if kind == "unsure":
                return
            header = msg.header
            if header.stamp.sec == 0 and header.stamp.nanosec == 0:  # stamp를 0으로 두는 드라이버
                header.stamp = self.get_clock().now().to_msg()
            points = self.filter.arc(distance, float(msg.field_of_view))
            self.clear.publish(point_cloud2.create_cloud_xyz32(header, points))
            if kind == "obstacle":
                self.mark.publish(point_cloud2.create_cloud_xyz32(header, points))

    rclpy.init()
    node = UltrasonicCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
