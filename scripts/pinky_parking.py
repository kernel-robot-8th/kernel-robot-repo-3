#!/usr/bin/env python3
"""Register Pinky's parking spot and check whether Pinky stopped on it, using the ceiling camera.

register: Pinky를 올바른 주차 위치에 세워 두고 Space를 누르면 마커 자세를 기준으로 저장한다.
check:    저장한 기준과 지금 마커 자세를 비교해 ARRIVED / MISALIGNED / NOT_FOUND를 판정한다.
결과는 관측 초안일 뿐이며 Pinky나 로봇팔에 명령을 보내지 않는다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from control_station.parking_check import (  # noqa: E402
    MarkerPose,
    ParkingResult,
    ParkingStatus,
    ParkingTolerance,
    PlanarOffset,
    detect_markers,
    estimate_marker_pose,
    judge,
    marker_object_points,
    median_offset,
    planar_offset,
)

SCHEMA_VERSION = 1
PIXEL_TRANSFORM_ID = "opencv-v4l2-capture-no-app-geometry-v1"
DEFAULT_SPOT_DIR = REPOSITORY_ROOT / ".control-station" / "parking-spots"
# 등록 중 마커가 이보다 흔들리면 Pinky가 움직였거나 가려진 것으로 보고 저장하지 않는다
MAX_REGISTER_JITTER_MM = 5.0
MAX_REGISTER_JITTER_DEG = 3.0
DEFAULT_INTRINSICS = REPOSITORY_ROOT / ".control-station" / "camera-calibrations" / "ceiling_camera.json"
STATUS_COLORS = {
    ParkingStatus.ARRIVED: (0, 200, 0),
    ParkingStatus.MISALIGNED: (0, 160, 255),
    ParkingStatus.NOT_FOUND: (0, 0, 255),
    ParkingStatus.AMBIGUOUS: (0, 0, 255),
}


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_intrinsics(path: Path, camera_id: str) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("camera_id") != camera_id:
        raise SystemExit(f"내부 보정 파일의 camera_id가 {data.get('camera_id')!r}입니다 ({camera_id!r} 필요): {path}")
    if data.get("pixel_transform_id") != PIXEL_TRANSFORM_ID:
        raise SystemExit(f"내부 보정 파일의 pixel_transform_id가 다릅니다: {data.get('pixel_transform_id')!r}")
    return {
        "camera_matrix": np.array(data["camera_matrix"], dtype=np.float64),
        "distortion": np.array(data["distortion_coefficients"], dtype=np.float64),
        "width": int(data["image_size_px"]["width"]),
        "height": int(data["image_size_px"]["height"]),
        "calibration_id": data.get("calibration_id"),
        "path": path,
        "sha256": sha256_of(path),
    }


def open_camera(device: str, width: int, height: int) -> cv2.VideoCapture:
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not camera.isOpened():
        raise SystemExit(f"카메라를 열 수 없습니다: {device} (다른 프로그램이 쓰는지 fuser -v 로 확인)")
    return camera


def read_frame(camera: cv2.VideoCapture, width: int, height: int) -> np.ndarray | None:
    ok, frame = camera.read()
    if not ok:
        return None
    if frame.shape[:2] != (height, width):
        # 다른 해상도로 돌면 내부 보정값이 맞지 않는다
        raise SystemExit(f"프레임 해상도 {frame.shape[1]}x{frame.shape[0]}가 보정 해상도 {width}x{height}와 다릅니다")
    return frame


def observe(
    frame: np.ndarray, args: argparse.Namespace, intrinsics: dict
) -> tuple[MarkerPose | None, np.ndarray | None, str, bool]:
    """Return (pose, corners, reason, ambiguous). Pose is None when the marker is missing, duplicated or unusable."""
    found = detect_markers(frame, args.dictionary).get(args.marker_id, [])
    if not found:
        return None, None, "marker not visible", False
    if len(found) > 1:
        return None, None, f"marker {args.marker_id} seen {len(found)} times", True
    try:
        pose = estimate_marker_pose(found[0], args.marker_side_mm, intrinsics["camera_matrix"], intrinsics["distortion"])
    except ValueError:
        return None, found[0], "marker pose estimation failed", False
    return pose, found[0], "", False


def draw_pose(view: np.ndarray, pose: MarkerPose | None, corners: np.ndarray | None, intrinsics: dict, side_mm: float) -> None:
    if corners is not None:
        cv2.polylines(view, [corners.astype(np.int32)], True, (255, 200, 0), 2)
    if pose is not None:
        rvec, _ = cv2.Rodrigues(pose.rotation)
        cv2.drawFrameAxes(view, intrinsics["camera_matrix"], intrinsics["distortion"], rvec, pose.translation_mm, side_mm)


def draw_reference(view: np.ndarray, reference: MarkerPose, intrinsics: dict, side_mm: float) -> None:
    """Draw where the marker should be (the registered spot) as a white outline with a front tick."""
    rvec, _ = cv2.Rodrigues(reference.rotation)
    points = np.vstack([marker_object_points(side_mm), [[0.0, side_mm, 0.0]]])
    projected, _ = cv2.projectPoints(points, rvec, reference.translation_mm, intrinsics["camera_matrix"], intrinsics["distortion"])
    projected = projected.reshape(-1, 2).astype(np.int32)
    cv2.polylines(view, [projected[:4]], True, (255, 255, 255), 1, cv2.LINE_AA)
    top_mid = ((projected[0] + projected[1]) / 2).astype(np.int32)
    cv2.line(view, tuple(int(v) for v in top_mid), tuple(int(v) for v in projected[4]), (255, 255, 255), 1, cv2.LINE_AA)


def draw_lines(view: np.ndarray, lines: list[tuple[str, tuple[int, int, int]]]) -> None:
    for row, (text, color) in enumerate(lines):
        y = 26 + 24 * row
        cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)


def pose_to_json(pose: MarkerPose) -> dict:
    return {"rotation": pose.rotation.tolist(), "translation_mm": pose.translation_mm.tolist()}


def pose_from_json(data: dict) -> MarkerPose:
    return MarkerPose(np.array(data["rotation"], dtype=np.float64), np.array(data["translation_mm"], dtype=np.float64))


def pick_reference(collected: list[MarkerPose]) -> tuple[MarkerPose, float, float]:
    """Pick the frame nearest the median position, and report how far the others spread from it."""
    translations = np.array([p.translation_mm for p in collected])
    center = np.median(translations, axis=0)
    reference = collected[int(np.argmin(np.linalg.norm(translations - center, axis=1)))]
    spread = [planar_offset(reference, p) for p in collected]
    return reference, max(o.distance_mm for o in spread), max(abs(o.dyaw_deg) for o in spread)


def register(args: argparse.Namespace) -> int:
    intrinsics = load_intrinsics(args.intrinsics, args.camera_id)
    output = args.spot_dir / f"{args.spot_id}.json"
    if output.exists():
        raise SystemExit(f"이미 등록된 주차 위치가 있습니다: {output} (다른 --spot-id를 쓰거나 기존 파일을 직접 옮기세요)")
    camera = open_camera(args.camera, intrinsics["width"], intrinsics["height"])
    window = f"register parking spot {args.spot_id}"
    collected: list[MarkerPose] = []
    collecting = False
    warning = ""
    print("Pinky를 올바른 주차 위치에 세운 뒤 Space. q: 종료", flush=True)
    try:
        while True:
            frame = read_frame(camera, intrinsics["width"], intrinsics["height"])
            if frame is None:
                continue
            pose, corners, reason, _ = observe(frame, args, intrinsics)
            view = frame.copy()
            draw_pose(view, pose, corners, intrinsics, args.marker_side_mm)
            if collecting and pose is not None:
                collected.append(pose)
            lines = [(f"marker {args.marker_id} ({args.dictionary}, {args.marker_side_mm:g} mm)", (255, 255, 255))]
            if pose is None:
                lines.append((f"NOT FOUND: {reason}", (0, 0, 255)))
            else:
                distance = float(np.linalg.norm(pose.translation_mm))
                lines.append((f"found, {distance:.0f} mm from camera", (0, 200, 0)))
            if collecting:
                lines.append((f"registering... {len(collected)}/{args.frames}", (0, 220, 255)))
            else:
                lines.append(("[Space] register this spot  [q] quit", (0, 220, 255)))
            if warning:
                lines.append((warning, (0, 0, 255)))
            draw_lines(view, lines)
            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("저장하지 않고 종료합니다.", flush=True)
                return 1
            if key == ord(" ") and not collecting:
                collecting, collected, warning = True, [], ""
            if collecting and len(collected) >= args.frames:
                reference, jitter_mm, jitter_deg = pick_reference(collected)
                if jitter_mm <= MAX_REGISTER_JITTER_MM and jitter_deg <= MAX_REGISTER_JITTER_DEG:
                    break
                collecting = False
                warning = (f"moved while registering ({jitter_mm:.1f} mm / {jitter_deg:.1f} deg) "
                           "- keep Pinky still, press Space again")
                print(f"등록 중 마커가 움직였습니다 ({jitter_mm:.1f} mm / {jitter_deg:.1f}°). 다시 Space", flush=True)
    finally:
        camera.release()
        cv2.destroyAllWindows()
    record = {
        "schema_version": SCHEMA_VERSION,
        "spot_id": args.spot_id,
        "camera_id": args.camera_id,
        "pixel_transform_id": PIXEL_TRANSFORM_ID,
        "image_size_px": {"width": intrinsics["width"], "height": intrinsics["height"]},
        "intrinsic_calibration": {
            "path": os.path.relpath(intrinsics["path"], REPOSITORY_ROOT),
            "sha256": intrinsics["sha256"],
            "calibration_id": intrinsics["calibration_id"],
        },
        "aruco_dictionary": args.dictionary,
        "marker_id": args.marker_id,
        "marker_side_mm": args.marker_side_mm,
        "reference_pose_in_camera": pose_to_json(reference),
        "reference_frame_count": len(collected),
        "reference_jitter": {"position_mm": round(jitter_mm, 2), "yaw_deg": round(jitter_deg, 3)},
        "tolerance": {"position_mm": args.tolerance_mm, "yaw_deg": args.tolerance_deg},
        "axes": "dx +: 마커 오른쪽 변 방향, dy +: 마커 윗변(화살표) 방향, dyaw +: 천장 카메라에서 볼 때 반시계",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "DRAFT_ONLY",
        "review_required": True,
        "usable_for_robot_control": False,
    }
    args.spot_dir.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as handle:  # 기존 파일은 덮어쓰지 않는다
        json.dump(record, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(f"주차 위치 저장: {output}", flush=True)
    print(f"기준 프레임 {len(collected)}장, 흔들림 최대 {jitter_mm:.1f} mm / {jitter_deg:.2f}°", flush=True)
    return 0


def load_spot(args: argparse.Namespace) -> tuple[dict, dict, MarkerPose]:
    path = args.spot_dir / f"{args.spot_id}.json"
    if not path.is_file():
        raise SystemExit(f"등록된 주차 위치가 없습니다: {path} (먼저 register)")
    spot = json.loads(path.read_text(encoding="utf-8"))
    if spot.get("schema_version") != SCHEMA_VERSION:
        raise SystemExit(f"지원하지 않는 주차 위치 schema: {spot.get('schema_version')}")
    intrinsics_path = REPOSITORY_ROOT / spot["intrinsic_calibration"]["path"]
    intrinsics = load_intrinsics(intrinsics_path, spot["camera_id"])
    if intrinsics["sha256"] != spot["intrinsic_calibration"]["sha256"]:
        raise SystemExit("등록 이후 내부 보정 파일이 바뀌었습니다. 주차 위치를 다시 등록하세요.")
    args.dictionary = spot["aruco_dictionary"]
    args.marker_id = spot["marker_id"]
    args.marker_side_mm = spot["marker_side_mm"]
    return spot, intrinsics, pose_from_json(spot["reference_pose_in_camera"])


def check(args: argparse.Namespace) -> int:
    spot, intrinsics, reference = load_spot(args)
    tolerance = ParkingTolerance(
        args.tolerance_mm if args.tolerance_mm is not None else spot["tolerance"]["position_mm"],
        args.tolerance_deg if args.tolerance_deg is not None else spot["tolerance"]["yaw_deg"],
    )
    camera = open_camera(args.camera, intrinsics["width"], intrinsics["height"])
    window = f"parking check {args.spot_id}"
    recent: deque[PlanarOffset] = deque(maxlen=args.frames)
    last_status = None
    started = time.monotonic()
    misses = 0
    result = judge(None, tolerance, "no frame yet")
    try:
        while True:
            frame = read_frame(camera, intrinsics["width"], intrinsics["height"])
            if frame is None:
                continue
            pose, corners, reason, ambiguous = observe(frame, args, intrinsics)
            if pose is None:
                misses += 1
                if misses >= args.frames:  # 연속으로 못 찾으면 이전 값을 버린다
                    recent.clear()
            else:
                misses = 0
                recent.append(planar_offset(reference, pose))
            if ambiguous:  # 같은 ID가 둘이면 어느 쪽이 Pinky인지 모른다
                recent.clear()
                result = ParkingResult(ParkingStatus.AMBIGUOUS, None, reason)
            elif recent:
                result = judge(median_offset(list(recent)), tolerance)
            else:
                result = judge(None, tolerance, reason)

            if args.once:
                if len(recent) >= args.frames or time.monotonic() - started > args.timeout:
                    break
                continue

            if result.status != last_status:
                print(json.dumps({"spot_id": args.spot_id, **result.to_dict()}, ensure_ascii=False), flush=True)
                last_status = result.status
            view = frame.copy()
            draw_reference(view, reference, intrinsics, args.marker_side_mm)
            draw_pose(view, pose, corners, intrinsics, args.marker_side_mm)
            color = STATUS_COLORS[result.status]
            lines = [(f"{result.status.value}", color)]
            if result.offset is not None:
                o = result.offset
                lines.append((f"dx {o.dx_mm:+.0f} mm  dy {o.dy_mm:+.0f} mm  dyaw {o.dyaw_deg:+.1f} deg", color))
            lines.append((result.reason, color))
            lines.append((f"tolerance {tolerance.position_mm:g} mm / {tolerance.yaw_deg:g} deg   white = registered spot   [q] quit", (220, 220, 220)))
            draw_lines(view, lines)
            cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1), color, 6)
            cv2.imshow(window, view)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

    if args.once:
        print(json.dumps({"spot_id": args.spot_id, "frames": len(recent), **result.to_dict()}, ensure_ascii=False))
    return {ParkingStatus.ARRIVED: 0, ParkingStatus.MISALIGNED: 1}.get(result.status, 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("register", "check"))
    parser.add_argument("--spot-id", required=True, help="주차 위치 이름 (예: pinky_dock_1)")
    parser.add_argument("--camera", default=os.environ.get("CONTROL_STATION_CEILING_CAMERA"),
                        help="천장 카메라 장치 경로. 기본값은 환경 변수 CONTROL_STATION_CEILING_CAMERA")
    parser.add_argument("--camera-id", default="ceiling_camera")
    parser.add_argument("--intrinsics", type=Path, default=DEFAULT_INTRINSICS, help="천장 카메라 내부 보정 JSON")
    parser.add_argument("--spot-dir", type=Path, default=DEFAULT_SPOT_DIR)
    parser.add_argument("--dictionary", default="DICT_4X4_50", help="register: ArUco 사전")
    parser.add_argument("--marker-id", type=int, help="register: Pinky 마커 ID")
    parser.add_argument("--marker-side-mm", type=float, help="register: 인쇄된 마커 검은 테두리 한 변 길이(mm)")
    parser.add_argument("--tolerance-mm", type=float, help="허용 위치 오차 (register 기본 20)")
    parser.add_argument("--tolerance-deg", type=float, help="허용 방향 오차 (register 기본 10)")
    parser.add_argument("--frames", type=int, default=15, help="기준 등록·판정에 모을 프레임 수")
    parser.add_argument("--once", action="store_true", help="check: 창 없이 한 번 판정하고 JSON 출력 (종료 코드 0 도착, 1 어긋남, 2 못 찾음)")
    parser.add_argument("--timeout", type=float, default=3.0, help="check --once: 최대 대기 초")
    args = parser.parse_args()
    if not args.camera:
        parser.error("--camera 또는 CONTROL_STATION_CEILING_CAMERA가 필요합니다 (같은 모델 카메라가 여럿이면 /dev/v4l/by-path 경로)")
    if args.command == "register":
        if args.marker_id is None or args.marker_side_mm is None:
            parser.error("register에는 --marker-id와 --marker-side-mm가 필요합니다")
        args.tolerance_mm = 20.0 if args.tolerance_mm is None else args.tolerance_mm
        args.tolerance_deg = 10.0 if args.tolerance_deg is None else args.tolerance_deg
        return register(args)
    return check(args)


if __name__ == "__main__":
    raise SystemExit(main())
