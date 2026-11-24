#!/usr/bin/env python3
"""녹화한 주행으로 라이다 + 초음파 퓨전을 오프라인 평가한다(로그는 scripts/fusion_record.py가 만든다).

실행 위치: 오프라인 도구 (사람이 직접 실행, PC). ROS 없이 돈다.
사용처: 사람이 퓨전 설정(navigation.fusion_grid)을 정하고 비교할 때. 로봇 FusionGrid 코드를 그대로 재생한다.

할 일 두 가지:
  noise    상자 앞에 서 있는 녹화로 센서 노이즈를 잰다: 센서·거리별 평균, 표준편차, 결측 비율.
           가장 큰 표준편차가 lidar_sigma_m / us_sigma_m이 된다.
             python3 scripts/fusion_eval.py noise --run 0.20=logs/fusion/d20.jsonl --run 0.40=logs/fusion/d40.jsonl
  compare  같은 녹화를 세 방법으로 재생한다(ablation, 한 요소씩 빼 보는 비교):
             lidar_only         라이다만: 퓨전 전에 Nav2가 가졌던 것
             ultrasonic_binary  읽을 때마다 칸을 덮어쓴다: 반사 한 번이 호 전체를 바로 표시하고 다음 값이 다시
                                정한다(Nav2 자체 marking/clearing처럼 하는 고전 이진 소나 표시: 누적 없음,
                                라이다 연관 없음)
             fused              로봇에서 도는 그대로의 FusionGrid
           --box x,y(로봇 출발 좌표계의 상자 중심, x 앞, y 왼쪽, m)를 주면 낮은 상자를 찾았는지, 언제 찾았는지,
           그때 로봇이 얼마나 떨어져 있었는지 보고한다. --box가 없으면 빈 길 주행이고 낮은 장애물은 모두 오검출이다.
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

# PC 패키지(scripts.fusion_eval)로 불리면 상대 import, 스크립트로 직접 실행하면 같은 폴더에서 import한다.
if __package__:
    from .fusion_grid import FusionGrid
else:
    from fusion_grid import FusionGrid

ROOT = Path(__file__).resolve().parents[1]
BASE = 'base_footprint'
# compare가 돌리는 방법별 FusionGrid 설정 덮어쓰기(fusion_grid.DEFAULTS의 비교용 스위치).
METHODS = {
    'lidar_only': {'use_ultrasonic': False},
    'ultrasonic_binary': {'associate': False, 'us_accumulate': False},
    'fused': {},
}


def load(path) -> list[dict]:
    """JSONL 녹화를 레코드 목록으로 읽는다."""
    with open(path, encoding='utf-8') as handle:
        return [json.loads(line) for line in handle if line.strip()]


def reading(value) -> float:
    """녹화된 거리 값을 float로 되돌린다: null -> nan, +inf(반사 없음)를 뜻하는 1e9 -> inf."""
    if value is None:
        return math.nan
    return math.inf if value >= 1e9 else float(value)


def compose(a, b):
    """자세 a의 좌표계로 주어진 자세 b를 a의 부모 좌표계로 옮긴다. 자세는 (x, y, yaw)다."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], yaw + b[2]


class StaticFrames:
    """녹화된 static TF(URDF)에서 얻은 센서 장착 위치. base_footprint 기준 (x, y, yaw, z)."""

    def __init__(self, records):
        self.parent = {r['child'].lstrip('/'): r for r in records if r.get('type') == 'tf_static'}

    def in_base(self, frame: str):
        """frame에서 부모를 따라 base_footprint까지 올라가며 합성한다. 사슬이 끊기거나 20단을 넘으면 None."""
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
    """녹화된 odom 자세를 시간으로 선형 보간한다. yaw는 unwrap해 -pi/pi 경계에서 튀지 않게 한다."""
    def __init__(self, records):
        rows = sorted((r['t'], r['x'], r['y'], r['yaw']) for r in records if r.get('type') == 'odom')
        self.t = np.array([row[0] for row in rows])
        self.x = np.array([row[1] for row in rows])
        self.y = np.array([row[2] for row in rows])
        self.yaw = np.unwrap(np.array([row[3] for row in rows])) if rows else np.array([])

    # 녹화 구간 앞뒤 slack초(0.5 s)까지만 보간한다. 그 밖이면 None(자세를 모름).
    def at(self, t: float, slack: float = 0.5):
        if not len(self.t) or t < self.t[0] - slack or t > self.t[-1] + slack:
            return None
        return (float(np.interp(t, self.t, self.x)), float(np.interp(t, self.t, self.y)),
                float(np.interp(t, self.t, self.yaw)))

    def start(self):
        return (float(self.x[0]), float(self.y[0]), float(self.yaw[0])) if len(self.t) else None


def ultrasonic_mount(frames: StaticFrames, frame: str, geometry: dict):
    """초음파 장착 위치: TF에 있으면 그것, 없으면 robot_geometry.json 값과 높이 3.7 cm(fusion_grid와 같은 대체)."""
    mount = frames.in_base(frame) if frame else None
    if mount is not None:
        return mount
    return geometry.get('ultrasound_x_m', 0.0), geometry.get('ultrasound_y_m', 0.0), \
        geometry.get('ultrasound_yaw_rad', 0.0), 0.037


def replay(records, settings=None, geometry=None):
    """녹화를 시간 순서로 FusionGrid에 넣는다. 값 하나를 넣을 때마다 (t, odom의 로봇 자세, 격자)를 낸다."""
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
    """녹화 하나에서 낮은 장애물 검출을 평가한다. box = 로봇 출발 좌표계의 (x, y), 빈 길이면 None."""
    odom = OdomTrack(records)
    start = odom.start()
    if start is None:
        raise ValueError('the recording has no odometry')
    target = compose(start, (box[0], box[1], 0.0))[:2] if box else None
    # 상자 반 폭 + 5 cm 안의 낮은 장애물 칸은 상자를 찾은 것, 그 밖은 오검출로 센다
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
    """METHODS의 방법마다 evaluate한 결과 {방법 이름: 결과}."""
    return {name: evaluate(records, {**(base_settings or {}), **preset}, box, box_size, geometry)
            for name, preset in METHODS.items()}


def noise_stats(values, expected=None) -> dict:
    """평균, 표준편차, 결측(None/유한하지 않음) 비율, 기대 거리 대비 편향(bias)."""
    good = [v for v in values if v is not None and math.isfinite(v)]
    result = dict(n=len(values), valid=len(good), dropout=round(1 - len(good) / len(values), 3) if values else None,
                  mean_m=None, std_m=None, bias_m=None)
    if len(good) >= 2:
        result.update(mean_m=round(statistics.fmean(good), 4), std_m=round(statistics.stdev(good), 4))
        if expected is not None:
            result['bias_m'] = round(result['mean_m'] - expected, 4)
    return result


def noise(records, distance=None, geometry=None) -> dict:
    """distance = 로봇 중심(base_footprint)에서 바로 앞 상자 면까지(m). 재지 않았으면 None.
    센서별 기대 거리는 distance에서 센서 장착 x를 뺀 값이다. 라이다는 정면 빔 하나만 본다."""
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
        k = int(np.argmin(np.abs(np.arctan2(np.sin(angles), np.cos(angles)))))  # 바로 앞을 향한 빔
        v = rec['ranges'][k]
        lidar_values.append(v if v is not None and math.isfinite(v) and rec['range_min'] <= v <= rec['range_max'] else None)
    expect = (lambda mount: None if distance is None or mount is None else distance - mount[0])
    return dict(ultrasonic=noise_stats(us_values, expect(us_mount)), lidar=noise_stats(lidar_values, expect(lidar_mount)))


def suggested_sigmas(rows: list[dict]) -> dict:
    """모든 거리 중 가장 큰 표준편차(최소 5 mm). 모델이 센서보다 더 확신하는 일이 없게 한다."""
    out = {}
    for sensor, key in (('lidar', 'lidar_sigma_m'), ('ultrasonic', 'us_sigma_m')):
        stds = [row[sensor]['std_m'] for row in rows if row[sensor]['std_m'] is not None]
        if stds:
            out[key] = round(max(max(stds), 0.005), 4)
    return out


def default_geometry() -> dict:
    """config/robot_geometry.json의 default 치수. 읽지 못하면 빈 dict."""
    try:
        return json.loads((ROOT / 'config/robot_geometry.json').read_text())['default']
    except (OSError, ValueError, KeyError):
        return {}


def _print_compare(results: dict, box) -> None:
    """compare 결과를 방법별 한 줄 표로 출력한다."""
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
