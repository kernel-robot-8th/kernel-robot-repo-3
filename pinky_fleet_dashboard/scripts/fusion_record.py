#!/usr/bin/env python3
"""Record what the fusion needs, for the noise measurement and the offline comparison (scripts/fusion_eval.py).

On the PC through the domain bridge (topics under /pinky_<id>/):
    source /opt/ros/jazzy/setup.bash
    python3 scripts/fusion_record.py --ns /pinky_10 --seconds 20 --out logs/fusion/box60.jsonl --note "low box 0.6 m"
On the Pinky itself: --ns ''.

One JSON object per line: meta, tf_static (sensor mountings), odom (robot pose), scan, range. "t" is the time the
message arrived here, so all topics share one clock; stamps are kept as they came.
Only reads topics: it never commands the robot.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def finite_or_none(values):
    """JSON has no inf/nan: +inf (no return) stays as 1e9, anything else non-finite becomes null."""
    return [1e9 if math.isinf(v) and v > 0 else (v if math.isfinite(v) else None) for v in values]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ns', default='/pinky_10', help="topic prefix, '' on the robot itself")
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--out', required=True)
    parser.add_argument('--note', default='')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Range
    from tf2_msgs.msg import TFMessage

    ns = args.ns.rstrip('/')
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    handle = out.open('w', encoding='utf-8')
    counts = dict(scan=0, range=0, odom=0, tf_static=0)

    def write(record):
        handle.write(json.dumps(record) + '\n')
        counts[record['type']] = counts.get(record['type'], 0) + 1

    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    class Recorder(Node):
        def __init__(self):
            super().__init__('pinky_fusion_record')
            latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(TFMessage, f'{ns}/tf_static', self.on_static, latched)
            self.create_subscription(Odometry, f'{ns}/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, f'{ns}/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Range, f'{ns}/us_sensor/range', self.on_range, qos_profile_sensor_data)
            self.seen_static = set()

        def on_static(self, msg):
            for tf in msg.transforms:
                key = (tf.header.frame_id, tf.child_frame_id)
                if key in self.seen_static:
                    continue
                self.seen_static.add(key)
                t = tf.transform.translation
                write(dict(type='tf_static', parent=tf.header.frame_id, child=tf.child_frame_id,
                           x=t.x, y=t.y, z=t.z, yaw=yaw_of(tf.transform.rotation)))

        def on_odom(self, msg):
            p = msg.pose.pose
            write(dict(type='odom', t=time.time(), stamp=stamp(msg.header), x=p.position.x, y=p.position.y,
                       yaw=yaw_of(p.orientation)))

        def on_scan(self, msg):
            write(dict(type='scan', t=time.time(), stamp=stamp(msg.header), frame=msg.header.frame_id,
                       angle_min=msg.angle_min, angle_increment=msg.angle_increment, range_min=msg.range_min,
                       range_max=msg.range_max, ranges=finite_or_none(msg.ranges)))

        def on_range(self, msg):
            write(dict(type='range', t=time.time(), stamp=stamp(msg.header), frame=msg.header.frame_id,
                       range=finite_or_none([msg.range])[0], fov=msg.field_of_view, min=msg.min_range,
                       max=msg.max_range))

    write(dict(type='meta', ns=ns, note=args.note, started=time.strftime('%Y-%m-%dT%H:%M:%S%z')))
    rclpy.init()
    node = Recorder()
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        handle.close()
    print(f'{out}: ' + ', '.join(f'{k} {v}' for k, v in counts.items()))
    missing = [k for k in ('scan', 'range', 'odom', 'tf_static') if not counts.get(k)]
    if missing:
        print('nothing received for: ' + ', '.join(missing) + ' (bridge running? --ns right?)')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
