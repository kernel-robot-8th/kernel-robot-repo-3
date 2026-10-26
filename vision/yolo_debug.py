"""YOLO 물체 인식 디버그 창.

실행: cd vision && python3 yolo_debug.py [--camera top_a] [--model models/best_top_a.pt]
키: u 왜곡 펴기 on/off / +,- 신뢰도 기준 / s 원본 프레임 저장 / q 종료
칸 영역(register_baskets.py)이 있으면 bbox 중심이 든 칸을, homography 보정이 있으면 작업대 좌표(mm)도 표시한다.
"""
import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from vision_omx import calibration as calib
from vision_omx.baskets import lateral_position, load_latest_baskets, locate, position_label
from vision_omx.camera import open_camera
from vision_omx.geometry import pixel_to_table
from vision_omx.yolo_detector import YoloDetector

HERE = Path(__file__).resolve().parent
GREEN, RED, WHITE, YELLOW, GRAY = (0, 200, 0), (0, 0, 255), (255, 255, 255), (0, 220, 255), (160, 160, 160)
CLASS_COLORS = [(60, 60, 255), (255, 160, 0), (0, 200, 255), (200, 0, 200), (0, 200, 0)]


def draw_text(frame, lines, origin=(10, 25)):
    x, y = origin
    for text, color in lines:
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        y += 24


def draw_detection(frame, det):
    color = CLASS_COLORS[det.class_id % len(CLASS_COLORS)]
    x1, y1, x2, y2 = (int(v) for v in det.bbox_xyxy)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    label = f'{det.name} {det.confidence:.2f}'
    cv2.putText(frame, label, (x1, max(y1 - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(frame, label, (x1, max(y1 - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    center = (int(det.center_uv[0]), int(det.center_uv[1]))
    cv2.drawMarker(frame, center, color, cv2.MARKER_CROSS, 20, 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(HERE / 'config/poc.yaml'))
    parser.add_argument('--camera', help='config의 cameras 중 하나 (기본: 첫 번째)')
    parser.add_argument('--model', help='가중치 경로 (기본: config의 yolo.model_path)')
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text())
    yolo_cfg = config['yolo']
    calib_dir = HERE / config['calibration_dir']
    rec_dir = HERE / config['recordings_dir']
    camera_name = args.camera or next(iter(config['cameras']))
    if camera_name not in config['cameras']:
        raise SystemExit(f'카메라 {camera_name}가 config에 없습니다. 있는 것: {list(config["cameras"])}')

    model_path = Path(args.model or HERE / yolo_cfg['model_path'])
    if not model_path.exists():
        raise SystemExit(f'가중치 {model_path}가 없습니다. models/에 넣거나 --model로 지정하세요.')
    detector = YoloDetector(model_path, yolo_cfg['min_confidence'], yolo_cfg.get('imgsz', 640))

    calibration = calib.load_latest(calib_dir, camera_name)
    baskets = load_latest_baskets(calib_dir, camera_name)
    intrinsics = calib.load_latest_intrinsics(calib_dir, camera_name)
    undistorter = calib.Undistorter(intrinsics) if intrinsics else None
    undistort = undistorter is not None

    cap = open_camera(config['cameras'][camera_name])

    window = f'YOLO - {camera_name}'
    fps, last = 0.0, cv2.getTickCount()
    while True:
        ok, frame = cap.read()
        if not ok:
            print('프레임을 읽지 못했습니다.')
            break
        if undistort:
            frame = undistorter(frame)
        raw = frame.copy()  # s로 저장할 그림 없는 프레임 (학습 데이터용)

        # 칸 영역·homography는 그때 쓴 프레임(왜곡 폄 여부와 intrinsic 버전)과 같을 때만 쓴다
        frame_version = intrinsics.version if undistort else ''
        use_baskets = baskets is not None and baskets.intrinsic_version == frame_version
        to_table = calibration is not None and calibration.intrinsic_version == frame_version

        if use_baskets:
            for name, polygon in baskets.regions.items():
                points = np.asarray(polygon, dtype=np.int32)
                cv2.polylines(frame, [points], True, GRAY, 1)
                cv2.putText(frame, name, tuple(int(v) for v in points.min(axis=0) + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, GRAY, 1)

        detections = detector.detect(raw)
        for det in detections:
            draw_detection(frame, det)

        now = cv2.getTickCount()
        fps = 0.9 * fps + 0.1 * cv2.getTickFrequency() / max(now - last, 1)
        last = now
        lines = [(f'[{camera_name}] {model_path.name} {fps:4.1f} fps  conf >= {detector.min_confidence:.2f}', WHITE),
                 (f'undistort: {"on" if undistort else "off"}'
                  + ('' if intrinsics else ' (intrinsic NONE)'), WHITE if intrinsics else YELLOW),
                 (f'baskets: {baskets.version}' if baskets else 'baskets: NONE (register_baskets.py)',
                  GREEN if use_baskets else YELLOW)]
        if baskets and not use_baskets:
            lines.append(('baskets made with other undistort/intrinsic -> re-register', RED))
        if calibration and not to_table:
            lines.append(('homography made with other undistort/intrinsic -> table mm off', YELLOW))
        for det in detections:
            text = f'{det.name} {det.confidence:.2f} px ({det.center_uv[0]:.0f}, {det.center_uv[1]:.0f})'
            if use_baskets:
                basket = locate(baskets.regions, det.center_uv)
                if basket:
                    pos = lateral_position(baskets.regions[basket], det.center_uv)
                    text += f'  {basket} {pos:.2f} ({position_label(pos)})'
                else:
                    text += '  basket -'
            if to_table:
                tx, ty = pixel_to_table(calibration.homography, det.center_uv)
                text += f'  table ({tx:.1f}, {ty:.1f}) mm'
            lines.append((text, GREEN))
        if not detections:
            lines.append(('no detection', RED))
        lines.append(('[u] undistort  [+/-] conf  [s] save  [q] quit', YELLOW))
        draw_text(frame, lines)
        cv2.imshow(window, frame)

        key = cv2.waitKey(1) & 0xFF
        # 창 X 버튼으로 닫아도 끝낸다. 안 그러면 카메라를 계속 붙잡고 있는다
        if key == ord('q') or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord('u') and undistorter:
            undistort = not undistort
        if key in (ord('+'), ord('=')):
            detector.min_confidence = min(detector.min_confidence + 0.05, 0.95)
        if key == ord('-'):
            detector.min_confidence = max(detector.min_confidence - 0.05, 0.05)
        if key == ord('s'):
            rec_dir.mkdir(parents=True, exist_ok=True)
            path = rec_dir / f'{camera_name}_yolo_{datetime.now():%Y%m%d_%H%M%S}.png'
            cv2.imwrite(str(path), raw)
            print(f'저장: {path}')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
