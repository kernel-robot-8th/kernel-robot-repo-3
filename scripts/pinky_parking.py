#!/usr/bin/env python3
"""Register Pinky's parking spot and check whether Pinky stopped on it, using the ceiling camera.

register: Pinky를 올바른 주차 위치에 세워 두고 Space를 누르면 마커 위치와 바닥 고정 마커 위치를 기준으로 저장한다.
check:    저장한 기준과 지금 마커 위치를 비교해 ARRIVED / MISALIGNED / NOT_FOUND / AMBIGUOUS / VIEW_UNVERIFIED를 판정한다.
결과는 관측 초안일 뿐이며 Pinky나 로봇팔에 명령을 보내지 않는다.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from control_station.parking_check import (  # noqa: E402
    MARKER_FORWARD_AXES,
    MAX_VIEW_SHIFT_PX,
    PIXEL_TRANSFORM_ID,
    SCHEMA_VERSION,
    Intrinsics,
    ParkingJudge,
    ParkingSpot,
    ParkingSpotError,
    ParkingStatus,
    ParkingTolerance,
    detect_markers,
    estimate_marker_pose,
    load_intrinsics,
    load_spot,
    observe_frame,
    ParkingResult,
    pinky_frame_offset,
    plane_offset,
    project_to_plane,
)

DEFAULT_SPOT_DIR = REPOSITORY_ROOT / ".control-station" / "parking-spots"
# 등록 중 마커가 이보다 흔들리면 Pinky가 움직였거나 가려진 것으로 보고 저장하지 않는다
MAX_REGISTER_JITTER_MM = 5.0
MAX_REGISTER_JITTER_DEG = 3.0
# 등록 중 이보다 흔들린 고정 마커는 시야 확인 기준으로 쓰지 않는다
MAX_ANCHOR_JITTER_PX = 1.0
DEFAULT_INTRINSICS = REPOSITORY_ROOT / ".control-station" / "camera-calibrations" / "ceiling_camera.json"
STATUS_COLORS = {
    ParkingStatus.ARRIVED: (0, 200, 0),
    ParkingStatus.MISALIGNED: (0, 160, 255),
    ParkingStatus.NOT_FOUND: (0, 0, 255),
    ParkingStatus.AMBIGUOUS: (0, 0, 255),
    ParkingStatus.VIEW_UNVERIFIED: (0, 0, 255),
}


class RegistrationRejected(Exception):
    pass


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


def draw_markers(view: np.ndarray, found: dict[int, list[np.ndarray]], marker_id: int) -> None:
    for found_id, corners_list in found.items():
        color = (255, 200, 0) if found_id == marker_id else (160, 160, 160)
        for corners in corners_list:
            cv2.polylines(view, [corners.astype(np.int32)], True, color, 2)


def draw_registered(view: np.ndarray, spot: ParkingSpot) -> None:
    """Draw where the marker should be (white outline with a tick to its top edge) and the fixed markers' registered centers."""
    corners = spot.reference_corners_px
    cv2.polylines(view, [corners.astype(np.int32)], True, (255, 255, 255), 1, cv2.LINE_AA)
    center = corners.mean(axis=0)
    top_mid = (corners[0] + corners[1]) / 2
    cv2.line(view, tuple(int(v) for v in center), tuple(int(v) for v in top_mid), (255, 255, 255), 1, cv2.LINE_AA)
    for anchor in spot.anchors.values():
        cv2.circle(view, tuple(int(v) for v in anchor), 6, (255, 255, 255), 1, cv2.LINE_AA)


def result_payload(result: ParkingResult, spot: ParkingSpot) -> dict[str, object]:
    """Judgment plus Pinky-relative forward/left when the marker mounting direction is recorded."""
    payload = result.to_dict()
    if result.offset is not None and spot.marker_forward_axis is not None:
        forward, left = pinky_frame_offset(result.offset, spot.marker_forward_axis)
        payload.update(forward_mm=round(forward, 1), left_mm=round(left, 1))
    return payload


def draw_lines(view: np.ndarray, lines: list[tuple[str, tuple[int, int, int]]]) -> None:
    for row, (text, color) in enumerate(lines):
        y = 26 + 24 * row
        cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)


def pick_anchors(
    collected: list[tuple[np.ndarray, dict[int, list[np.ndarray]]]], marker_id: int, anchor_ids: list[int] | None
) -> dict[int, np.ndarray]:
    """Fixed markers seen once in every collected frame and steady within MAX_ANCHOR_JITTER_PX."""
    candidates = anchor_ids or sorted({i for _, found in collected for i in found if i != marker_id})
    anchors: dict[int, np.ndarray] = {}
    for anchor_id in candidates:
        centers = [found[anchor_id][0].mean(axis=0) for _, found in collected if len(found.get(anchor_id, [])) == 1]
        if len(centers) == len(collected):
            center = np.median(centers, axis=0)
            if max(float(np.linalg.norm(c - center)) for c in centers) <= MAX_ANCHOR_JITTER_PX:
                anchors[anchor_id] = center
                continue
        if anchor_ids:
            raise RegistrationRejected(f"fixed marker {anchor_id} was not seen steadily in every frame")
        print(f"마커 {anchor_id}는 모든 프레임에서 고정돼 보이지 않아 시야 확인 기준에서 뺍니다.", flush=True)
    if not anchors:
        raise RegistrationRejected("no fixed marker seen steadily - the camera view cannot be verified later")
    return anchors


def build_registration(
    collected: list[tuple[np.ndarray, dict[int, list[np.ndarray]]]], args: argparse.Namespace, intrinsics: Intrinsics
) -> dict:
    corners = np.median(np.stack([c for c, _ in collected]), axis=0)
    try:
        plane = estimate_marker_pose(corners, args.marker_side_mm, intrinsics.camera_matrix, intrinsics.distortion)
    except ValueError as error:
        raise RegistrationRejected(str(error)) from error
    reference = project_to_plane(corners, plane, intrinsics.camera_matrix, intrinsics.distortion)
    spread = [
        plane_offset(reference, project_to_plane(c, plane, intrinsics.camera_matrix, intrinsics.distortion))
        for c, _ in collected
    ]
    jitter_mm = max(o.distance_mm for o in spread)
    jitter_deg = max(abs(o.dyaw_deg) for o in spread)
    if jitter_mm > MAX_REGISTER_JITTER_MM or jitter_deg > MAX_REGISTER_JITTER_DEG:
        raise RegistrationRejected(f"moved while registering ({jitter_mm:.1f} mm / {jitter_deg:.1f} deg) - keep Pinky still")
    anchors = {} if args.no_anchors else pick_anchors(collected, args.marker_id, args.anchor_ids)
    view_check = {"mode": "OFF"} if args.no_anchors else {"mode": "FIXED_MARKERS", "max_anchor_shift_px": MAX_VIEW_SHIFT_PX}
    return {
        "schema_version": SCHEMA_VERSION,
        "spot_id": args.spot_id,
        "camera_id": args.camera_id,
        "pixel_transform_id": PIXEL_TRANSFORM_ID,
        "image_size_px": {"width": intrinsics.width, "height": intrinsics.height},
        "intrinsic_calibration": {
            "path": os.path.relpath(intrinsics.path, REPOSITORY_ROOT),
            "sha256": intrinsics.sha256,
            "calibration_id": intrinsics.calibration_id,
        },
        "aruco_dictionary": args.dictionary,
        "marker_id": args.marker_id,
        "marker_side_mm": args.marker_side_mm,
        "marker_forward_axis": args.marker_forward_axis,
        "position_method": "corner rays intersected with the registered marker plane",
        "reference_pose_in_camera": {"rotation": plane.rotation.tolist(), "translation_mm": plane.translation_mm.tolist()},
        "reference_corners_px": corners.tolist(),
        "reference_marker_side_on_plane_mm": round(reference.side_mm, 2),
        "reference_frame_count": len(collected),
        "reference_jitter": {"position_mm": round(jitter_mm, 2), "yaw_deg": round(jitter_deg, 3)},
        "anchors": [{"marker_id": i, "center_px": [round(float(v), 2) for v in c]} for i, c in anchors.items()],
        "view_check": view_check,
        "tolerance": {"position_mm": args.tolerance_mm, "yaw_deg": args.tolerance_deg},
        "axes": "dx +: 마커 오른쪽 변 방향, dy +: 마커 윗변(화살표) 방향, dyaw +: 천장 카메라에서 볼 때 반시계",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "DRAFT_ONLY",
        "review_required": True,
        "usable_for_robot_control": False,
    }


def register(args: argparse.Namespace) -> int:
    intrinsics = load_intrinsics(args.intrinsics, args.camera_id)
    output = args.spot_dir / f"{args.spot_id}.json"
    if output.exists():
        raise SystemExit(f"이미 등록된 주차 위치가 있습니다: {output} (다른 --spot-id를 쓰거나 기존 파일을 직접 옮기세요)")
    camera = open_camera(args.camera, intrinsics.width, intrinsics.height)
    window = f"register parking spot {args.spot_id}"
    collected: list[tuple[np.ndarray, dict[int, list[np.ndarray]]]] = []
    collecting = False
    warning = ""
    print("Pinky를 올바른 주차 위치에 세운 뒤 Space. q: 종료", flush=True)
    print("화면 왼쪽 가장자리에 세로 이음선이 보이면 영상이 밀린 상태이니 종료 후 다시 실행하세요.", flush=True)
    try:
        while True:
            frame = read_frame(camera, intrinsics.width, intrinsics.height)
            if frame is None:
                continue
            found = detect_markers(frame, args.dictionary)
            markers = found.get(args.marker_id, [])
            view = frame.copy()
            draw_markers(view, found, args.marker_id)
            if collecting and len(markers) == 1:
                collected.append((markers[0], found))
            others = sorted(i for i in found if i != args.marker_id)
            lines = [(f"marker {args.marker_id} ({args.dictionary}, {args.marker_side_mm:g} mm)", (255, 255, 255))]
            if len(markers) == 1:
                lines.append((f"found; fixed markers {others}", (0, 200, 0)))
            else:
                lines.append((f"NOT FOUND (seen {len(markers)} times)", (0, 0, 255)))
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
                try:
                    record = build_registration(collected, args, intrinsics)
                    break
                except RegistrationRejected as error:
                    collecting, warning = False, f"{error} - press Space again"
                    print(f"등록하지 않았습니다: {error}. 다시 Space", flush=True)
    finally:
        camera.release()
        cv2.destroyAllWindows()
    args.spot_dir.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as handle:  # 기존 파일은 덮어쓰지 않는다
        json.dump(record, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(f"주차 위치 저장: {output}", flush=True)
    jitter = record["reference_jitter"]
    print(f"기준 프레임 {len(collected)}장, 흔들림 최대 {jitter['position_mm']:.1f} mm / {jitter['yaw_deg']:.2f}°", flush=True)
    if record["anchors"]:
        print(f"시야 확인 고정 마커: {[a['marker_id'] for a in record['anchors']]}", flush=True)
    else:
        print("시야 확인 꺼짐: 영상 밀림이나 카메라 이동을 감지하지 않습니다.", flush=True)
    return 0


def check(args: argparse.Namespace) -> int:
    try:
        spot = load_spot(args.spot_dir / f"{args.spot_id}.json", REPOSITORY_ROOT)
    except ParkingSpotError as error:
        raise SystemExit(str(error)) from error
    tolerance = ParkingTolerance(
        args.tolerance_mm if args.tolerance_mm is not None else spot.tolerance.position_mm,
        args.tolerance_deg if args.tolerance_deg is not None else spot.tolerance.yaw_deg,
    )
    view_check = "ON" if spot.anchors else "OFF"
    camera = open_camera(args.camera, spot.intrinsics.width, spot.intrinsics.height)
    window = f"parking check {args.spot_id}"
    judgment = ParkingJudge(tolerance, args.frames)
    result = judgment.result
    last_status = None
    started = time.monotonic()
    try:
        while True:
            frame = read_frame(camera, spot.intrinsics.width, spot.intrinsics.height)
            if frame is None:
                continue
            observation = observe_frame(frame, spot)
            result = judgment.add(observation)

            if args.once:
                if len(judgment.recent) >= args.frames or time.monotonic() - started > args.timeout:
                    break
                continue

            if result.status != last_status:
                print(json.dumps({"spot_id": args.spot_id, "view_check": view_check, **result_payload(result, spot)},
                                 ensure_ascii=False), flush=True)
                last_status = result.status
            view = frame.copy()
            draw_registered(view, spot)
            if observation.corners is not None:
                cv2.polylines(view, [observation.corners.astype(np.int32)], True, (255, 200, 0), 2)
            color = STATUS_COLORS[result.status]
            lines = [(f"{result.status.value}", color)]
            if result.offset is not None:
                o = result.offset
                lines.append((f"dx {o.dx_mm:+.0f} mm  dy {o.dy_mm:+.0f} mm  dyaw {o.dyaw_deg:+.1f} deg  "
                              f"{result.deviation_level.value if result.deviation_level else ''}", color))
                if spot.marker_forward_axis is not None:
                    forward, left = pinky_frame_offset(o, spot.marker_forward_axis)
                    lines.append((f"Pinky forward {forward:+.0f} mm  left {left:+.0f} mm", color))
            lines.append((result.reason, color))
            lines.append((f"tolerance {tolerance.position_mm:g} mm / {tolerance.yaw_deg:g} deg   view check {view_check}   "
                          "white = registered   [q] quit", (220, 220, 220)))
            draw_lines(view, lines)
            cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1), color, 6)
            cv2.imshow(window, view)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

    if args.once:
        print(json.dumps(
            {"spot_id": args.spot_id, "frames": len(judgment.recent), "view_check": view_check, **result_payload(result, spot)},
            ensure_ascii=False,
        ))
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
    parser.add_argument("--anchor-ids", type=lambda text: [int(v) for v in text.split(",")],
                        help="register: 시야 확인에 쓸 바닥 고정 마커 ID (예: 1,3,4). 기본은 등록 중 고정돼 보인 모든 다른 마커")
    parser.add_argument("--marker-forward-axis", choices=sorted(MARKER_FORWARD_AXES),
                        help="register: Pinky 앞쪽을 향하는 마커 축 (+x: 마커 오른쪽 변, +y: 윗변). 주면 판정에 Pinky 기준 앞/왼쪽 mm를 함께 출력")
    parser.add_argument("--no-anchors", action="store_true",
                        help="register: 고정 마커 없이 Pinky 마커만으로 판정 (영상 밀림·카메라 이동을 감지하지 않음)")
    parser.add_argument("--tolerance-mm", type=float, help="허용 위치 오차 (register 기본 30)")
    parser.add_argument("--tolerance-deg", type=float, help="허용 방향 오차 (register 기본 5)")
    parser.add_argument("--frames", type=int, default=15, help="기준 등록·판정에 모을 프레임 수")
    parser.add_argument("--once", action="store_true",
                        help="check: 창 없이 한 번 판정하고 JSON 출력 (종료 코드 0 도착, 1 어긋남, 2 판정 불가)")
    parser.add_argument("--timeout", type=float, default=3.0, help="check --once: 최대 대기 초")
    args = parser.parse_args()
    if not args.camera:
        parser.error("--camera 또는 CONTROL_STATION_CEILING_CAMERA가 필요합니다 (같은 모델 카메라가 여럿이면 /dev/v4l/by-path 경로)")
    if args.command == "register":
        if args.marker_id is None or args.marker_side_mm is None:
            parser.error("register에는 --marker-id와 --marker-side-mm가 필요합니다")
        if args.anchor_ids and args.marker_id in args.anchor_ids:
            parser.error("--anchor-ids에 Pinky 마커 ID를 넣을 수 없습니다")
        if args.anchor_ids and args.no_anchors:
            parser.error("--anchor-ids와 --no-anchors는 함께 쓸 수 없습니다")
        args.tolerance_mm = 30.0 if args.tolerance_mm is None else args.tolerance_mm
        args.tolerance_deg = 5.0 if args.tolerance_deg is None else args.tolerance_deg
        return register(args)
    try:
        return check(args)
    except SystemExit as error:
        if not isinstance(error.code, str):
            raise
        # 등록 파일·카메라 문제로 판정하지 못한 경우도 1(어긋남)이 아닌 2(판정 불가)로 끝낸다
        print(error.code, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
