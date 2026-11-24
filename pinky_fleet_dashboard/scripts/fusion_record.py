#!/usr/bin/env python3
"""퓨전에 필요한 토픽을 녹화한다: 노이즈 측정과 오프라인 비교(scripts/fusion_eval.py)에 쓴다.

실행 위치: 오프라인 도구 (사람이 직접 실행). PC에서 도메인 브리지를 거치거나 핑키에서 직접 돌린다.
사용처: 만든 JSONL을 scripts/fusion_eval.py가 읽는다. app.py나 주행준비는 이 파일을 쓰지 않는다.

PC에서 도메인 브리지를 거쳐(토픽이 /pinky_<id>/ 아래에 있음):
    source /opt/ros/jazzy/setup.bash
    python3 scripts/fusion_record.py --ns /pinky_10 --seconds 20 --out logs/fusion/box60.jsonl --note "low box 0.6 m"
핑키 자체에서는: --ns ''.

한 줄에 JSON 객체 하나: meta, tf_static(센서 장착 위치), odom(로봇 자세), scan, range. "t"는 메시지가 여기
도착한 시각이라 모든 토픽이 시계 하나를 공유한다. stamp는 들어온 그대로 둔다.
토픽을 읽기만 한다: 로봇에 명령을 내리지 않는다.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path


# 쿼터니언 q에서 z축 회전(yaw, rad)만 뽑는다.
def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def finite_or_none(values):
    """JSON에는 inf/nan이 없다: +inf(반사 없음)는 1e9로 남기고, 나머지 유한하지 않은 값은 null로 바꾼다."""
    return [1e9 if math.isinf(v) and v > 0 else (v if math.isfinite(v) else None) for v in values]


def main(argv=None) -> int:
    """--seconds 동안 녹화해 --out에 쓴다. 받지 못한 토픽이 하나라도 있으면 1을 돌려준다."""
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
            # /tf_static은 발행자가 TRANSIENT_LOCAL(latched)로 한 번만 보낸다: 같은 QoS로 받아야 늦게 붙어도
            # 장착 위치를 받는다. 센서 토픽은 발행자에 맞춰 best-effort(sensor_data)다.
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
