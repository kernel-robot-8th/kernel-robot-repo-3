#!/usr/bin/env python3
"""Offline evaluation of the LiDAR + ultrasonic fusion on recorded runs (scripts/fusion_record.py makes the logs).

Two jobs:
  noise    Sensor noise from runs standing still in front of a box: mean, standard deviation and dropouts per
           sensor and distance. The largest standard deviation becomes lidar_sigma_m / us_sigma_m.
             python3 scripts/fusion_eval.py noise --run 0.20=logs/fusion/d20.jsonl --run 0.40=logs/fusion/d40.jsonl
  compare  The same recording replayed through three methods (an ablation):
             lidar_only         LiDAR alone: what Nav2 had before
             ultrasonic_binary  every reading overwrites its cells: an echo marks its whole arc at once, the
                                next reading decides again (the classic binary sonar marking, as Nav2's own
                                marking/clearing does it: no accumulation, no LiDAR association)
             fused              FusionGrid as it runs on the robot
           With --box x,y (the box centre in the robot's start frame, x forward, y left, metres) it reports whether
           and when the low box was found and how far the robot was from it then; without --box the run is an empty
           path and every low obstacle is a false one.
             python3 scripts/fusion_eval.py compare logs/fusion/box60.jsonl --box 0.60,0.0
             python3 scripts/fusion_eval.py compare logs/fusion/empty.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

if __package__:
    from .fusion_grid import FusionGrid
else:
    from fusion_grid import FusionGrid

ROOT = Path(__file__).resolve().parents[1]
BASE = 'base_footprint'
METHODS = {
    'lidar_only': {'use_ultrasonic': False},
    'ultrasonic_binary': {'associate': False, 'us_accumulate': False},
    'fused': {},
}


def load(path) -> list[dict]:
    with open(path, encoding='utf-8') as handle:
        return [json.loads(line) for line in handle if line.strip()]


def reading(value) -> float:
    """A recorded range back to a float: null -> nan, the 1e9 that stands for +inf (no return) -> inf."""
    if value is None:
        return math.nan
    return math.inf if value >= 1e9 else float(value)


def compose(a, b):
    """Pose b given in the frame of pose a, into a's parent frame. Poses are (x, y, yaw)."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], yaw + b[2]


class StaticFrames:
    """Sensor mountings from the recorded static TF (URDF), as (x, y, yaw, z) in base_footprint."""

    def __init__(self, records):
        self.parent = {r['child'].lstrip('/'): r for r in records if r.get('type') == 'tf_static'}

    def in_base(self, frame: str):
        frame = (frame or '').lstrip('/')
        if frame == BASE:
            return 0.0, 0.0, 0.0, 0.0
        chain = []
        while frame != BASE:
            link = self.parent.get(frame)
            if link is None or len(chain) > 20:
                return None
            chain.append(link)
            frame = link['parent'].lstrip('/')
        pose, z = (0.0, 0.0, 0.0), 0.0
        for link in reversed(chain):
            pose = compose(pose, (link['x'], link['y'], link['yaw']))
            z += link.get('z', 0.0)
        return pose[0], pose[1], pose[2], z


class OdomTrack:
    def __init__(self, records):
        rows = sorted((r['t'], r['x'], r['y'], r['yaw']) for r in records if r.get('type') == 'odom')
        self.t = np.array([row[0] for row in rows])
        self.x = np.array([row[1] for row in rows])
        self.y = np.array([row[2] for row in rows])
        self.yaw = np.unwrap(np.array([row[3] for row in rows])) if rows else np.array([])

    def at(self, t: float, slack: float = 0.5):
        if not len(self.t) or t < self.t[0] - slack or t > self.t[-1] + slack:
            return None
        return (float(np.interp(t, self.t, self.x)), float(np.interp(t, self.t, self.y)),
                float(np.interp(t, self.t, self.yaw)))

    def start(self):
        return (float(self.x[0]), float(self.y[0]), float(self.yaw[0])) if len(self.t) else None


def ultrasonic_mount(frames: StaticFrames, frame: str, geometry: dict):
    mount = frames.in_base(frame) if frame else None
    if mount is not None:
        return mount
    return geometry.get('ultrasound_x_m', 0.0), geometry.get('ultrasound_y_m', 0.0), \
        geometry.get('ultrasound_yaw_rad', 0.0), 0.037


def replay(records, settings=None, geometry=None):
    """Feed the recording to a FusionGrid in time order; yields (t, robot pose in odom, grid) after each reading."""
    grid = FusionGrid(settings)
    frames, odom, geometry = StaticFrames(records), OdomTrack(records), geometry or {}
    for rec in sorted((r for r in records if r.get('type') in ('scan', 'range')), key=lambda r: r['t']):
        base = odom.at(rec['t'])
        if base is None:
            continue
        grid.recenter(base[0], base[1])
        if rec['type'] == 'scan':
            mount = frames.in_base(rec['frame'])
            if mount is None:
                continue
            angles = rec['angle_min'] + np.arange(len(rec['ranges'])) * rec['angle_increment']
            ranges = np.array([reading(v) for v in rec['ranges']], float)
            grid.update_scan(compose(base, mount[:3]), angles, ranges, rec['range_min'], rec['range_max'], rec['t'])
        else:
            mount = ultrasonic_mount(frames, rec.get('frame'), geometry)
            grid.update_range(compose(base, mount[:3]), reading(rec['range']), rec.get('fov'), rec['t'])
        yield rec['t'], base, grid


def evaluate(records, settings=None, box=None, box_size=0.10, geometry=None) -> dict:
    """Low-obstacle detection over one recording. box = (x, y) in the robot's start frame, None for an empty path."""
    odom = OdomTrack(records)
    start = odom.start()
    if start is None:
        raise ValueError('the recording has no odometry')
    target = compose(start, (box[0], box[1], 0.0))[:2] if box else None
    match = box_size / 2 + 0.05
    result = dict(detected=False if box else None, first_detection_s=None, robot_to_box_at_detection_m=None,
                  closest_robot_to_box_m=None, false_positive_s=0.0, max_false_cells=0)
    first_t = last_t = None
    grid = None
    for t, base, grid in replay(records, settings, geometry):
        first_t = t if first_t is None else first_t
        dt = 0.0 if last_t is None else t - last_t
        last_t = t
        pts = grid.points(grid.low_obstacle_cells())
        false = len(pts)
        if target is not None:
            to_box = math.hypot(base[0] - target[0], base[1] - target[1])
            closest = result['closest_robot_to_box_m']
            result['closest_robot_to_box_m'] = to_box if closest is None else min(closest, to_box)
            near = np.hypot(pts[:, 0] - target[0], pts[:, 1] - target[1]) <= match if len(pts) else np.zeros(0, bool)
            false = int((~near).sum())
            if near.any() and not result['detected']:
                result.update(detected=True, first_detection_s=round(t - first_t, 2),
                              robot_to_box_at_detection_m=round(to_box, 3))
        if false:
            result['false_positive_s'] += dt
        result['max_false_cells'] = max(result['max_false_cells'], false)
    if grid is None:
        raise ValueError('nothing to replay: no scan or ultrasonic reading overlaps the odometry')
    result['false_positive_s'] = round(result['false_positive_s'], 2)
    if result['closest_robot_to_box_m'] is not None:
        result['closest_robot_to_box_m'] = round(result['closest_robot_to_box_m'], 3)
    result['duration_s'] = round(last_t - first_t, 2)
    result['echo_counts'] = {k: v for k, v in grid.echo_counts.items() if v}
    return result


def compare(records, base_settings=None, box=None, box_size=0.10, geometry=None) -> dict:
    return {name: evaluate(records, {**(base_settings or {}), **preset}, box, box_size, geometry)
            for name, preset in METHODS.items()}


def noise_stats(values, expected=None) -> dict:
    """Mean, standard deviation, share of dropouts (None/non-finite) and bias against the expected distance."""
    good = [v for v in values if v is not None and math.isfinite(v)]
    result = dict(n=len(values), valid=len(good), dropout=round(1 - len(good) / len(values), 3) if values else None,
                  mean_m=None, std_m=None, bias_m=None)
    if len(good) >= 2:
        result.update(mean_m=round(statistics.fmean(good), 4), std_m=round(statistics.stdev(good), 4))
        if expected is not None:
            result['bias_m'] = round(result['mean_m'] - expected, 4)
    return result


def noise(records, distance=None, geometry=None) -> dict:
    """distance = robot centre (base_footprint) to the box face, straight ahead; None if not measured."""
    frames = StaticFrames(records)
    geometry = geometry or {}
    us_values, us_mount = [], None
    for rec in (r for r in records if r.get('type') == 'range'):
        us_mount = us_mount or ultrasonic_mount(frames, rec.get('frame'), geometry)
        v = rec['range']
        us_values.append(v if v is not None and math.isfinite(v) and rec.get('min', 0) <= v <= rec.get('max', 1e9) else None)
    lidar_values, lidar_mount = [], None
    for rec in (r for r in records if r.get('type') == 'scan'):
        mount = frames.in_base(rec['frame'])
        if mount is None:
            continue
        lidar_mount = mount
        angles = rec['angle_min'] + np.arange(len(rec['ranges'])) * rec['angle_increment'] + mount[2]
        k = int(np.argmin(np.abs(np.arctan2(np.sin(angles), np.cos(angles)))))  # the beam pointing straight ahead
        v = rec['ranges'][k]
        lidar_values.append(v if v is not None and math.isfinite(v) and rec['range_min'] <= v <= rec['range_max'] else None)
    expect = (lambda mount: None if distance is None or mount is None else distance - mount[0])
    return dict(ultrasonic=noise_stats(us_values, expect(us_mount)), lidar=noise_stats(lidar_values, expect(lidar_mount)))


def suggested_sigmas(rows: list[dict]) -> dict:
    """The largest standard deviation over all distances, so the model is never more confident than the sensor."""
    out = {}
    for sensor, key in (('lidar', 'lidar_sigma_m'), ('ultrasonic', 'us_sigma_m')):
        stds = [row[sensor]['std_m'] for row in rows if row[sensor]['std_m'] is not None]
        if stds:
            out[key] = round(max(max(stds), 0.005), 4)
    return out


def default_geometry() -> dict:
    try:
        return json.loads((ROOT / 'config/robot_geometry.json').read_text())['default']
    except (OSError, ValueError, KeyError):
        return {}


def _print_compare(results: dict, box) -> None:
    head = f"{'method':<19}{'found':>7}{'after s':>9}{'robot-box m':>13}{'false s':>9}{'max false cells':>17}"
    print(head)
    print('-' * len(head))
    for name, r in results.items():
        found = '-' if r['detected'] is None else ('yes' if r['detected'] else 'no')
        fmt = (lambda v: '-' if v is None else str(v))
        print(f"{name:<19}{found:>7}{fmt(r['first_detection_s']):>9}{fmt(r['robot_to_box_at_detection_m']):>13}"
              f"{r['false_positive_s']:>9}{r['max_false_cells']:>17}")
    if box:
        print(f"closest the robot came to the box: {results['fused']['closest_robot_to_box_m']} m")
    print('fused echo classes:', results['fused']['echo_counts'])


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p_noise = sub.add_parser('noise', help='sensor noise from runs standing still in front of a box')
    p_noise.add_argument('--run', action='append', required=True, metavar='DISTANCE=LOG',
                         help='robot centre to box face in metres = recording; DISTANCE may be "-" if not measured')
    p_cmp = sub.add_parser('compare', help='lidar_only vs ultrasonic_binary vs fused on one recording')
    p_cmp.add_argument('log')
    p_cmp.add_argument('--box', help='x,y of the low box centre in the start frame (m); leave out for an empty run')
    p_cmp.add_argument('--box-size', type=float, default=0.10)
    p_cmp.add_argument('--settings', help='JSON file with fusion_grid settings to try')
    for p in (p_noise, p_cmp):
        p.add_argument('--json', help='also write the result here')
    args = parser.parse_args(argv)
    geometry = default_geometry()
    if args.command == 'noise':
        rows = []
        for item in args.run:
            distance, _, path = item.partition('=')
            row = dict(distance_m=None if distance == '-' else float(distance), log=path,
                       **noise(load(path), None if distance == '-' else float(distance), geometry))
            rows.append(row)
            for sensor in ('ultrasonic', 'lidar'):
                s = row[sensor]
                print(f"{str(row['distance_m']):>6} m  {sensor:<10} n={s['n']:<5} mean={s['mean_m']}  std={s['std_m']}"
                      f"  dropout={s['dropout']}  bias={s['bias_m']}")
        output = dict(runs=rows, suggested=suggested_sigmas(rows))
        print('suggested navigation.fusion_grid:', json.dumps(output['suggested']))
    else:
        settings = json.loads(Path(args.settings).read_text()) if args.settings else {}
        box = tuple(float(v) for v in args.box.split(',')) if args.box else None
        if box is not None and len(box) != 2:
            parser.error('--box takes x,y')
        output = compare(load(args.log), settings, box, args.box_size, geometry)
        _print_compare(output, box)
    if args.json:
        Path(args.json).write_text(json.dumps(output, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
