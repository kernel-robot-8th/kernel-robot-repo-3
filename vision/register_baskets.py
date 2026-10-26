"""상단캠 화면에 칸(바구니) 영역을 다각형으로 등록한다.

실행: cd vision && python3 register_baskets.py --camera top_a --names L_top L_bottom C_top ...
마우스 왼쪽 클릭: 꼭짓점 추가 / Enter: 이 칸 완료, 다음 칸 / z: 마지막 점 취소 / r: 이 칸 다시
s: 저장 (모든 칸 완료 후) / q: 저장 없이 종료
왜곡을 편 프레임에 그린다. yolo_debug.py도 같은 프레임에서 검출하므로 둘을 맞춰 둔다.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from vision_omx import calibration as calib
from vision_omx.baskets import load_latest_baskets, save_baskets
from vision_omx.camera import open_camera

HERE = Path(__file__).resolve().parent
GREEN, RED, WHITE, YELLOW, CYAN = (0, 200, 0), (0, 0, 255), (255, 255, 255), (0, 220, 255), (255, 200, 0)


def draw_text(frame, lines, origin=(10, 25)):
    x, y = origin
    for text, color in lines:
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        y += 24


def draw_region(frame, name, polygon, color, closed=True):
    if not polygon:
        return
    points = np.asarray(polygon, dtype=np.int32)
    cv2.polylines(frame, [points], closed, color, 2)
    for point in points:
        cv2.circle(frame, tuple(int(v) for v in point), 4, color, -1)
    if closed:
        u, v = points.mean(axis=0).astype(int)
        cv2.putText(frame, name, (u - 30, v), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(frame, name, (u - 30, v), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(HERE / 'config/poc.yaml'))
    parser.add_argument('--camera', help='config의 cameras 중 하나 (기본: 첫 번째)')
    parser.add_argument('--names', nargs='+', required=True, help='등록할 칸 이름, 등록 순서대로')
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text())
    calib_dir = HERE / config['calibration_dir']
    camera_name = args.camera or next(iter(config['cameras']))
    if camera_name not in config['cameras']:
        raise SystemExit(f'카메라 {camera_name}가 config에 없습니다. 있는 것: {list(config["cameras"])}')

    intrinsics = calib.load_latest_intrinsics(calib_dir, camera_name)
    undistort = calib.Undistorter(intrinsics) if intrinsics else None
    previous = load_latest_baskets(calib_dir, camera_name)
    cap = open_camera(config['cameras'][camera_name])

    regions = {}
    current = []
    window = f'register baskets - {camera_name}'
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, lambda event, u, v, *_: current.append([u, v])
                         if event == cv2.EVENT_LBUTTONDOWN and len(regions) < len(args.names) else None)

    while True:
        ok, frame = cap.read()
        if not ok:
            print('프레임을 읽지 못했습니다.')
            break
        if undistort:
            frame = undistort(frame)

        if previous and not regions and not current:
            # 새로 그리기 전에는 이전 등록을 흐리게 보여준다
            for name, polygon in previous.regions.items():
                draw_region(frame, name, polygon, (128, 128, 128))
        for name, polygon in regions.items():
            draw_region(frame, name, polygon, GREEN)
        done = len(regions) == len(args.names)
        if not done:
            draw_region(frame, args.names[len(regions)], current, YELLOW, closed=False)

        lines = [(f'[{camera_name}] intrinsic: {intrinsics.version if intrinsics else "NONE"}',
                  WHITE if intrinsics else YELLOW)]
        if previous:
            lines.append((f'previous: {previous.version} (gray)', WHITE))
        if done:
            lines.append(('all regions done -> [s] save  [q] quit', GREEN))
        else:
            lines.append((f'now: {args.names[len(regions)]} ({len(regions) + 1}/{len(args.names)})  '
                          f'points {len(current)}', CYAN))
            lines.append(('[click] point  [Enter] next  [z] undo  [r] redo  [q] quit', YELLOW))
        draw_text(frame, lines)
        cv2.imshow(window, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            print('저장하지 않고 종료합니다.')
            break
        if key == ord('z') and current:
            current.pop()
        if key == ord('r'):
            current.clear()
        if key in (13, 10) and not done:
            if len(current) < 3:
                print('꼭짓점이 3개 이상이어야 합니다.')
                continue
            regions[args.names[len(regions)]] = list(current)
            current.clear()
        if key == ord('s'):
            if not done:
                print(f'아직 등록하지 않은 칸이 있습니다: {args.names[len(regions):]}')
                continue
            _, path = save_baskets(regions, calib_dir, camera_name,
                                   intrinsic_version=intrinsics.version if intrinsics else '')
            print(f'저장: {path}')
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
