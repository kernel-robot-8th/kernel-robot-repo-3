#!/usr/bin/env python3
"""Robot side: turn the front ultrasonic Range into points for Nav2's VoxelLayer (LiDAR + ultrasonic fusion).

The LiDAR sees one plane about 12.5 cm above the floor; the ultrasonic looks forward about 3.7 cm above it, so
it sees low things the LiDAR passes over. The costmap keeps both in 5 cm height cells and treats a column as
blocked when any cell in it is occupied, so a low box becomes an obstacle Nav2 plans around.

A Range only says "something within this cone at this distance", so a confirmed echo marks the whole arc across
the field of view (the safe side). Two clouds go out in the sensor frame:
  /us_sensor/cloud_mark   the arc of a confirmed echo within mark_max_range_m (marking only)
  /us_sensor/cloud_clear  the arc at the measured distance, or at clear_max_range_m with no echo (clearing only)
A reading counts only after agree_count readings in a row agree within agree_m around the median of the last
`window`, which keeps single jumps out. Far echoes are not marked: from 3.7 cm high the cone meets the floor.
Settings: /home/pinky/.cache/pinky-fleet/nav2/runtime_config.json -> navigation.ultrasonic_cloud.
"""
from __future__ import annotations

import json
import math
import statistics
from collections import deque
from pathlib import Path

DEFAULTS = dict(min_range_m=0.03, mark_max_range_m=0.6, clear_max_range_m=0.8, window=5, agree_m=0.03,
                agree_count=3, arc_points=7, fov_rad=0.26)


class RangeFilter:
    """Median of the last readings, confirmed once enough agree. No ROS here, so it can be tested on its own."""

    def __init__(self, settings: dict):
        self.s = {**DEFAULTS, **(settings or {})}
        self.recent: deque[float] = deque(maxlen=int(self.s["window"]))
        self.agreeing = 0
        self.last_median: float | None = None

    def update(self, value: float) -> tuple[str, float | None]:
        """('obstacle', d) for a confirmed echo to mark, ('free', d) to clear up to d, or ('unsure', None)."""
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
        """Points across the cone at `distance` in the sensor frame (x forward)."""
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
            if header.stamp.sec == 0 and header.stamp.nanosec == 0:  # drivers that leave the stamp at zero
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
