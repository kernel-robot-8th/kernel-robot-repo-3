"""체크보드로 상단캠 렌즈 왜곡(intrinsic)을 보정한다.

실행: cd vision && python3 calib_intrinsics.py --camera top_a --cols 8 --rows 6 --square-mm 25
  --cols/--rows 는 칸 수가 아니라 **안쪽 모서리 수**다 (10x7 칸 보드 → 9x6).
키: Space 현재 프레임 담기(보드가 잡혔을 때만) / c 보정·저장 / u 왜곡 편 화면 보기 / q 종료
    d 보드 크기 찾기(안 잡힐 때) / s 원본 프레임 저장

보드를 화면 가운데·네 모서리·가장자리에 두루, 기울기도 바꿔 가며 15장 이상 담는다.
가장자리를 안 찍으면 그쪽 왜곡이 안 잡힌다. 결과 RMS가 0.5 px 이하면 양호.
"""
import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from vision_omx import calibration as calib
from vision_omx.camera import open_camera

HERE = Path(__file__).resolve().parent
GREEN, RED, YELLOW = (0, 200, 0), (0, 0, 255), (0, 220, 255)
MIN_SHOTS = 10
SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def find_board(gray, board_size):
    """체크보드 안쪽 모서리 픽셀. 못 찾으면 None.

    SB 검출기가 흐림·반사·작은 보드에 더 강하다. 실패하면 고전 검출기를 한 번 더 쓴다.
    """
    found, corners = cv2.findChessboardCornersSB(gray, board_size, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if found:
        return corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not found:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX)


def guess_board_sizes(gray):
    """현재 화면에서 잡히는 (가로, 세로) 안쪽 모서리 수를 모두 찾는다. 몇 초 걸린다."""
    sizes = []
    for cols in range(3, 15):
        for rows in range(3, cols + 1):
            found, _ = cv2.findChessboardCornersSB(gray, (cols, rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                sizes.append((cols, rows))
    return sizes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(HERE / 'config/poc.yaml'))
    parser.add_argument('--camera', help='config의 cameras 중 하나 (기본: 첫 번째)')
    parser.add_argument('--cols', type=int, required=True, help='가로 안쪽 모서리 수')
    parser.add_argument('--rows', type=int, required=True, help='세로 안쪽 모서리 수')
    parser.add_argument('--square-mm', type=float, required=True, help='칸 한 변(mm), 자로 잰 값')
    parser.add_argument('--k3', action='store_true', help='k3도 푼다. 광각 렌즈에서 가장자리까지 고르게 찍었을 때만')
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text())
    calib_dir = HERE / config['calibration_dir']
    camera_name = args.camera or next(iter(config['cameras']))
    if camera_name not in config['cameras']:
        raise SystemExit(f'카메라 {camera_name}가 config에 없습니다. 있는 것: {list(config["cameras"])}')
    board_size = (args.cols, args.rows)

    cam = config['cameras'][camera_name]
    cap = open_camera(cam)

    shots = []          # 담은 프레임의 모서리 픽셀
    coverage = None     # 담은 보드가 덮은 영역 (어디를 더 찍어야 하는지 보여 준다)
    undistort = None    # 보정 후 u 키로 켜는 왜곡 펴기
    show_undistorted = False

    window = f'intrinsic calib - {camera_name}'
    while True:
        ok, frame = cap.read()
        if not ok:
            print('프레임을 읽지 못했습니다.')
            break
        height, width = frame.shape[:2]
        if coverage is None:
            coverage = np.zeros((height, width), dtype=np.uint8)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_board(gray, board_size)

        view = undistort(frame) if (show_undistorted and undistort) else frame.copy()
        tint = np.zeros_like(view)
        tint[coverage > 0] = (0, 120, 0)
        view = cv2.addWeighted(view, 1.0, tint, 0.35, 0)
        if corners is not None and not show_undistorted:
            cv2.drawChessboardCorners(view, board_size, corners, True)

        status = 'board OK' if corners is not None else 'board not found'
        lines = [(f'[{camera_name}] {width}x{height}  board {args.cols}x{args.rows}  '
                  f'shots {len(shots)} (min {MIN_SHOTS})', GREEN),
                 (status, GREEN if corners is not None else RED),
                 ('[Space] add  [c] calibrate  [u] undistort view  [q] quit', YELLOW),
                 ('[d] guess board size  [s] save frame', YELLOW)]
        if show_undistorted:
            lines.insert(0, ('UNDISTORTED VIEW', YELLOW))
        y = 25
        for text, color in lines:
            cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            y += 24
        cv2.imshow(window, view)

        key = cv2.waitKey(1) & 0xFF
        # 창 X 버튼으로 닫아도 끝낸다. 안 그러면 카메라를 계속 붙잡고 있는다
        if key == ord('q') or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord(' '):
            if corners is None:
                print('보드가 잡히지 않았습니다. --cols/--rows가 안쪽 모서리 수인지 확인하세요.')
            else:
                shots.append(corners)
                cv2.fillConvexPoly(coverage, cv2.convexHull(corners.reshape(-1, 2).astype(np.int32)), 255)
                print(f'{len(shots)}장째 담음')
        if key == ord('c'):
            if len(shots) < MIN_SHOTS:
                print(f'{MIN_SHOTS}장 이상 담아야 합니다. 지금 {len(shots)}장.')
                continue
            camera_matrix, dist_coeffs, rms = calib.compute_intrinsics(
                shots, board_size, args.square_mm, (width, height), fix_k3=not args.k3)
            covered = float(np.count_nonzero(coverage)) / coverage.size
            intrinsics, path = calib.save_intrinsics(
                camera_matrix, dist_coeffs, (width, height), calib_dir, camera_name,
                extra={'rms_px': round(float(rms), 4), 'shots': len(shots),
                       'coverage': round(covered, 3), 'fix_k3': not args.k3,
                       'board': {'cols': args.cols, 'rows': args.rows, 'square_mm': args.square_mm}})
            undistort = calib.Undistorter(intrinsics)
            # 다시 찍지 않고 옵션만 바꿔 재계산할 수 있게 모서리 점을 남긴다
            np.save(path.with_name(f'{intrinsics.version}_points.npy'), np.array(shots))
            print(f'저장: {path}')
            print(f'  RMS {rms:.3f} px ({"양호" if rms <= 0.5 else "높음: 흔들린 사진을 빼고 다시"}), '
                  f'화면 덮은 비율 {covered:.0%}'
                  f'{"" if covered >= 0.8 else " (부족: 비어 있는 가장자리를 더 담으세요)"}')
            print(f'  fx={camera_matrix[0, 0]:.1f} fy={camera_matrix[1, 1]:.1f} '
                  f'cx={camera_matrix[0, 2]:.1f} cy={camera_matrix[1, 2]:.1f}')
            print(f'  dist={np.round(dist_coeffs, 4).tolist()}')
            print('  u 키로 왜곡 편 화면을 확인하세요 (직선이 곧게 보여야 함).')
        if key == ord('d'):
            print('보드 크기 찾는 중... (보드를 움직이지 마세요)')
            sizes = guess_board_sizes(gray)
            if not sizes:
                print('  어떤 크기로도 안 잡힙니다. 보드 전체가 화면에 들어오는지, 반사·흐림이 없는지 확인하세요. s로 저장해 보내 주세요.')
            else:
                # 가장 큰 크기가 실제 보드다. 작은 것은 보드 일부가 잡힌 것
                cols, rows = max(sizes, key=lambda size: size[0] * size[1])
                print(f'  잡히는 크기: {sizes}')
                print(f'  → --cols {cols} --rows {rows} 로 다시 실행하세요.')
        if key == ord('s'):
            rec_dir = HERE / config['recordings_dir']
            rec_dir.mkdir(parents=True, exist_ok=True)
            path = rec_dir / f'{camera_name}_board_{datetime.now():%Y%m%d_%H%M%S}.png'
            cv2.imwrite(str(path), frame)
            print(f'저장: {path}')
        if key == ord('u') and undistort:
            show_undistorted = not show_undistorted

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
