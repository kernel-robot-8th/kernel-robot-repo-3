#!/usr/bin/env python3
"""Export the configured measured-anchor route without changing its source CSV."""
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
