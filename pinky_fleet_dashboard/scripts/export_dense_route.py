#!/usr/bin/env python3
"""설정된 실측 기준점(anchor) 경로를 촘촘한 경로로 펼쳐 CSV로 내보낸다. 원본 CSV는 바꾸지 않는다.

실행 위치: 오프라인 도구 (사람이 PC에서 직접 실행: python3 scripts/export_dense_route.py).
사용처: 없음(확인용). route_geometry.build_dense_route의 결과를 눈으로 보거나 다른 도구에 넣을 때 쓴다.

핵심: data/waypoints.json의 10번·12번 웨이포인트를 build_dense_route로 약 10 cm 간격 점들로 펼쳐
~/Downloads/pinky_<번호>_route_10cm.csv에 쓴다. 엑셀에서 한글이 깨지지 않게 utf-8-sig(BOM)로 쓴다.
스크립트로 실행하면 scripts/ 폴더가 sys.path 맨 앞에 들어가므로 `from route_geometry import`가 동작한다.
"""
import csv
import json
from pathlib import Path
from route_geometry import build_dense_route

root = Path(__file__).resolve().parents[1]
anchors = json.loads((root / 'data/waypoints.json').read_text())
for robot in ('10', '12'):
    points = build_dense_route(anchors[robot])
    path = Path.home() / 'Downloads' / f'pinky_{robot}_route_10cm.csv'
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['seq', 'name', 'map', 'frame', 'x_m', 'y_m', 'yaw_rad', 'anchor'])
        for seq, p in enumerate(points, 1):
            writer.writerow([seq, p['label'], p['map'], p['frame_id'], p['x_m'], p['y_m'], p['yaw_rad'], p['anchor']])
    print(f'{path}: {len(points)} checkpoints')
