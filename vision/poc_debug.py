"""카메라 → OMX 좌표 PoC 디버그 창.

실행: cd vision && python3 poc_debug.py [--camera top_a] [--config config/poc.yaml]
키: c 보정 / Space 전송(지금은 출력만) / s 프레임 저장 / q 종료
"""
import argparse
from datetime import datetime
from pathlib import Path

import cv2
import yaml

from vision_omx import calibration as calib
from vision_omx.camera import open_camera
from vision_omx.detector import MarkerDetector
from vision_omx.geometry import build_pick_target
from vision_omx.types import Failure

HERE = Path(__file__).resolve().parent
GREEN, RED, WHITE, YELLOW = (0, 200, 0), (0, 0, 255), (255, 255, 255), (0, 220, 255)


def send_to_omx(target):
    # TODO: OMX 연결 후 어댑터 호출로 교체. 지금은 좌표만 출력한다.
    print(f'[SEND] {target.object_name}#{target.object_id} -> '
          f'{target.frame_id} x={target.x:.3f} y={target.y:.3f} z={target.z:.3f} m '
          f'(calib {target.calibration_version})')


def draw_text(frame, lines, origin=(10, 25)):
    x, y = origin
    for text, color in lines:
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        y += 24


def calibrate(centers, references, checks, calib_dir, camera_name, intrinsic_version):
    """보이는 기준 마커로 보정하고 오차를 출력한다. 실패하면 None."""
    homography = calib.compute_homography(centers, references)
    if homography is None:
        print(f'기준 마커가 4개 이상 보여야 보정할 수 있습니다. 보이는 것: {sorted(set(centers) & set(references))}')
        return None
    residuals = calib.reprojection_errors(homography, centers, references)
    check_errors = calib.reprojection_errors(homography, centers, checks)
    extra = {'reference_ids': sorted(residuals),
             'residual_max_mm': round(max(residuals.values()), 2),
             'check_errors_mm': {i: round(e, 2) for i, e in check_errors.items()}}
    calibration, path = calib.save(homography, calib_dir, camera_name, extra=extra,
                                   intrinsic_version=intrinsic_version)
    print(f'보정 저장: {path}')
    for marker_id, err in residuals.items():
        print(f'  기준 #{marker_id}: 잔차 {err:.1f} mm')
    for marker_id, err in check_errors.items():
        print(f'  검증 #{marker_id}: 오차 {err:.1f} mm')
    if not check_errors:
        print('  검증 마커가 안 보여 실제 오차는 확인하지 못했습니다.')
    return calibration


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(HERE / 'config/poc.yaml'))
    parser.add_argument('--camera', help='config의 cameras 중 하나 (기본: 첫 번째)')
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text())
    calib_dir = HERE / config['calibration_dir']
    rec_dir = HERE / config['recordings_dir']
    references = config['reference_markers_mm']
    checks = config.get('check_markers_mm') or {}
    camera_name = args.camera or next(iter(config['cameras']))
    if camera_name not in config['cameras']:
        raise SystemExit(f'카메라 {camera_name}가 config에 없습니다. 있는 것: {list(config["cameras"])}')

    detector = MarkerDetector(config['aruco']['dictionary'])
    calibration = calib.load_latest(calib_dir, camera_name)
    intrinsics = calib.load_latest_intrinsics(calib_dir, camera_name)
    undistort = calib.Undistorter(intrinsics) if intrinsics else None
    intrinsic_version = intrinsics.version if intrinsics else ''

    cam = config['cameras'][camera_name]
    cap = open_camera(cam)

    window = f'vision_omx PoC - {camera_name}'
    while True:
        ok, frame = cap.read()
        if not ok:
            print('프레임을 읽지 못했습니다.')
            break
        if undistort:
            frame = undistort(frame)

        centers, corners, ids = detector.detect(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        calib_text = f'calib: {calibration.version}' if calibration else 'calib: NONE (press c)'
        visible_refs = sorted(set(references) & set(centers))
        lines = [(f'[{camera_name}] {calib_text}', GREEN if calibration else RED),
                 (f'intrinsic: {intrinsic_version or "NONE (calib_intrinsics.py)"}',
                  WHITE if intrinsics else YELLOW),
                 (f'refs {len(visible_refs)}/{len(references)} {visible_refs} (need 4+)',
                  WHITE if len(visible_refs) >= 4 else RED)]
        if calibration and calibration.intrinsic_version != intrinsic_version:
            # 왜곡 펴기가 달라지면 픽셀 좌표가 바뀌므로 homography를 다시 만들어야 한다
            lines.append(('homography made with other intrinsic -> press c', RED))
        if calibration and visible_refs:
            # 보정 후 카메라가 밀리면 기준 마커 잔차가 커진다
            drift = calib.reprojection_errors(calibration.homography, centers, references)
            worst_id = max(drift, key=drift.get)
            ok = drift[worst_id] <= config['drift_warn_mm']
            lines.append((f'ref err max {drift[worst_id]:.1f} mm (#{worst_id})'
                          + ('' if ok else '  -> recalibrate?'), GREEN if ok else RED))
        if calibration:
            for marker_id, err in calib.reprojection_errors(calibration.homography, centers, checks).items():
                lines.append((f'check #{marker_id}: {err:.1f} mm', GREEN if err <= 5 else RED))

        targets = []
        for marker_id, uv in centers.items():
            if marker_id in references or marker_id in checks:
                continue
            result = build_pick_target(marker_id, uv, calibration, config)
            center = (int(uv[0]), int(uv[1]))
            cv2.drawMarker(frame, center, YELLOW, cv2.MARKER_CROSS, 30, 2)
            if isinstance(result, Failure):
                lines.append((f'#{marker_id}: {result.code.value} {result.detail}', RED))
            else:
                targets.append(result)
                tx, ty = result.table_xy_mm
                lines.append((f'#{marker_id} {result.object_name}: table ({tx:.1f}, {ty:.1f}) mm', WHITE))
                lines.append((f'    OMX ({result.x:.3f}, {result.y:.3f}, {result.z:.3f}) m', GREEN))

        lines.append(('[c] calib  [Space] send  [s] save  [q] quit', YELLOW))
        draw_text(frame, lines)
        cv2.imshow(window, frame)

        key = cv2.waitKey(1) & 0xFF
        # 창 X 버튼으로 닫아도 끝낸다. 안 그러면 카메라를 계속 붙잡고 있는다
        if key == ord('q') or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord('c'):
            calibration = calibrate(centers, references, checks, calib_dir, camera_name,
                                    intrinsic_version) or calibration
        if key == ord(' '):
            if not targets:
                print('보낼 목표가 없습니다.')
            for target in targets:
                send_to_omx(target)
        if key == ord('s'):
            rec_dir.mkdir(parents=True, exist_ok=True)
            path = rec_dir / f'{camera_name}_{datetime.now():%Y%m%d_%H%M%S}.png'
            cv2.imwrite(str(path), frame)
            print(f'저장: {path}')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
