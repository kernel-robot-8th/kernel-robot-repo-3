#!/usr/bin/env python3
"""정면 카메라가 선반 기준으로 어디에 있는지 추정하고, 칸 영역을 카메라 화면에 투영한다.

입력: 운영자가 찍은 선반 기준점(scripts/shelf_pose_editor.html), 그 기준점을 찍은 바로 그 원본 프레임,
카메라 내부 보정 초안, config/shelf-slots.json. solvePnP(SQPnP 후 Levenberg-Marquardt 보정)로
`shelf_front` 좌표계에서 카메라 자세를 구한다. 사용하는 칸마다 공간(바닥판 영역부터 위 단까지 높이)을
화면 다각형으로 투영해, 검출 박스의 아래 가운데 점을 칸에 대응시킬 수 있게 한다.

출력은 DRAFT_ONLY 검토용 산출물과 오버레이 PNG다. 잔차는 보고만 하고 판정하지 않는다:
자동 합격 기준은 없으며, 이 결과가 로봇 동작을 허가하지도 않는다.

사용법 (저장소 루트에서):
    .venv/bin/python scripts/calibrate_shelf_pose.py shelf-pose-points.json frame.png

기본 입력은 .control-station/camera-calibrations/shelf_camera.json(내부 보정)과 config/shelf-slots.json이고,
출력은 .control-station/camera-calibrations/<camera_id>-shelf-pose-<id>.json과 같은 이름의 .overlay.png다.
같은 이름의 파일이 있으면 덮어쓰지 않는다. 파일만 읽고 쓰며 카메라·로봇·관제 API에는 연결하지 않는다.
검토한 결과를 .control-station/shelf-pose-calibrations/<camera_id>.json으로 복사하면 관제가 칸 표시에 쓴다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from calibration_artifacts import (
    MAX_CALIBRATION_IMAGE_BYTES,
    ensure_distinct_output_path,
    install_file_exclusive,
    read_regular_file_bounded,
    unique_json_object,
    validate_intrinsic_draft,
)


# 기준점 최소 개수. solvePnP는 이론상 4점이면 되지만, 잘못 찍은 점 하나가 결과를 끌고 가지 않도록
# 여유를 두고, 점 하나를 뺀 재계산(leave-one-out)에도 5점이 남게 한다.
MIN_POINTS = 6
MAX_JSON_BYTES = 1_000_000  # 기준점·내부 보정·칸 정의 JSON 하나의 최대 크기(바이트)
# 위가 열린 칸(왼쪽 선반 위 단)에 그릴 높이(m). 측정값이 아니라 표시·대응용 가정값이다.
DEFAULT_OPEN_SLOT_HEIGHT_M = 0.10
# 칸별 오버레이 색. OpenCV 이미지 순서(B, G, R)다.
COLORS = [(66, 135, 245), (48, 209, 88), (255, 159, 10), (191, 90, 242), (255, 69, 58), (100, 210, 255)]


def _load_json(path: Path, label: str) -> tuple[dict[str, Any], str]:
    """JSON 객체 파일을 읽어 (내용, 파일 bytes의 SHA-256)을 돌려준다. 중복 키는 거부한다.

    해시는 결과 파일에 입력 출처로 남겨, 나중에 어떤 입력으로 만든 결과인지 대조할 수 있게 한다.
    """
    raw = read_regular_file_bounded(path, MAX_JSON_BYTES, label)
    payload = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_json_object)
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return payload, hashlib.sha256(raw).hexdigest()


def _point(value: object, label: str) -> tuple[float, float]:
    """[x, y] 픽셀 좌표를 검사해 float 쌍으로 돌려준다. bool은 int의 하위형이라 따로 거부한다."""
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value)
        or not all(math.isfinite(float(item)) for item in value)
    ):
        raise ValueError(f"{label} must be two finite numbers")
    return float(value[0]), float(value[1])


def solve_pose(
    object_points: np.ndarray, image_points: np.ndarray, camera_matrix: np.ndarray, distortion: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """3D 기준점(선반 좌표, m)과 클릭한 픽셀로 카메라 자세 (rvec, tvec)를 구한다.

    SQPnP는 초기값 없이 전역 해를 찾고, 이어서 Levenberg-Marquardt로 재투영 오차를 더 줄인다.
    rvec/tvec은 선반 좌표를 카메라 좌표로 옮기는 변환이다.
    """
    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion, flags=cv2.SOLVEPNP_SQPNP
    )
    if not ok:
        raise ValueError("solvePnP did not find a pose")
    rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, camera_matrix, distortion, rvec, tvec)
    return rvec, tvec


def slot_volume_corners(slot: dict[str, Any], open_height: float, top: float | None = None) -> np.ndarray:
    """칸 바닥판 영역을 바닥판부터 `top`까지 세운 상자의 꼭짓점 8개를 돌려준다(기본: 칸 전체 공간).

    칸 전체 높이는 clear_height_m(위 단까지)이고, 위가 열린 칸(null)은 open_height를 쓴다.
    """
    footprint = slot["footprint_m"]
    bottom = slot["deck_height_m"]
    if top is None:
        top = bottom + (slot["clear_height_m"] if slot["clear_height_m"] is not None else open_height)
    return np.array(
        [
            [x, y, z]
            for x in (footprint["x_min"], footprint["x_max"])
            for y in (footprint["y_min"], footprint["y_max"])
            for z in (bottom, top)
        ],
        dtype=np.float64,
    )


def build_artifact(
    points_doc: dict[str, Any],
    points_sha: str,
    image: np.ndarray,
    image_sha: str,
    intrinsics: dict[str, Any],
    intrinsics_sha: str,
    slot_map: dict[str, Any],
    slot_map_sha: str,
    open_height: float,
    anchor_margin: float = 0.02,
) -> tuple[dict[str, Any], np.ndarray]:
    """입력을 대조하고 카메라 자세를 구해 결과 JSON(dict)과 오버레이 이미지를 만든다. 파일은 쓰지 않는다.

    기준점 JSON이 같은 프레임(SHA-256)·같은 칸 정의(SHA-256)·같은 카메라 ID·같은 해상도로 만들어졌는지
    먼저 확인한다. 하나라도 다르면 찍은 픽셀과 3D 좌표의 짝이 맞지 않으므로 계산하지 않는다.
    """
    if points_doc.get("schema_version") != 1:
        raise ValueError("points JSON must be schema_version 1")
    if points_doc.get("source_image_sha256") != image_sha:
        raise ValueError("points JSON was made on a different image (source_image_sha256 mismatch)")
    if points_doc.get("slot_map_sha256") != slot_map_sha:
        raise ValueError("points JSON was made with a different slot map (slot_map_sha256 mismatch)")
    if points_doc.get("camera_id") != intrinsics["camera_id"]:
        raise ValueError("points JSON camera_id does not match the intrinsic calibration camera_id")
    height, width = image.shape[:2]
    size = intrinsics["image_size_px"]
    if points_doc.get("source_image_size_px") != {"width": width, "height": height} or size != {
        "width": width,
        "height": height,
    }:
        raise ValueError("image size must match both the points JSON and the intrinsic calibration")

    keypoints = {item["keypoint_id"]: item for item in slot_map.get("keypoints", [])}
    clicked = points_doc.get("points")
    if not isinstance(clicked, list) or len(clicked) < MIN_POINTS:
        raise ValueError(f"at least {MIN_POINTS} clicked keypoints are required")
    ids = [item.get("keypoint_id") for item in clicked]
    if len(set(ids)) != len(ids) or any(keypoint_id not in keypoints for keypoint_id in ids):
        raise ValueError("clicked keypoints must be unique keypoint_id values from the slot map")
    object_points = np.array(
        [[keypoints[i]["point_m"][axis] for axis in ("x", "y", "z")] for i in ids], dtype=np.float64
    )
    image_points = np.array(
        [_point(item.get("image_px"), f"{item.get('keypoint_id')}.image_px") for item in clicked],
        dtype=np.float64,
    )
    # 중심을 뺀 3D 점들의 rank가 2 미만이면 모두 한 직선 위에 있다. 그런 점으로는 직선 둘레 회전이
    # 정해지지 않으므로 거부한다(tol 1e-3 = 약 1 mm 수준의 편차는 무시).
    if np.linalg.matrix_rank(object_points - object_points.mean(axis=0), tol=1e-3) < 2:
        raise ValueError("clicked keypoints are collinear; pick corners on different edges")

    camera_matrix = np.array(intrinsics["camera_matrix"], dtype=np.float64)
    distortion = np.array(intrinsics["distortion_coefficients"], dtype=np.float64).reshape(-1)
    rvec, tvec = solve_pose(object_points, image_points, camera_matrix, distortion)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    residuals = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)  # 점별 재투영 오차(px)
    rotation, _ = cv2.Rodrigues(rvec)
    # 카메라 중심의 선반 좌표: x_cam = R·x_shelf + t 에서 x_cam = 0 을 풀면 x_shelf = -Rᵀ·t 이다.
    camera_in_shelf = (-rotation.T @ tvec).reshape(3)

    # Leave-one-out: 점 하나씩 빼고 다시 풀어 그 점을 예측해, 다른 점들과 맞지 않는 점을 드러낸다.
    # 자기 자신이 포함된 재투영 오차는 잘못 찍은 점도 작게 나올 수 있어, 빼고 예측한 오차가 더 믿을 만하다.
    leave_one_out: list[float | None] = []
    for index in range(len(ids)):
        keep = np.arange(len(ids)) != index
        if keep.sum() < MIN_POINTS - 1:
            leave_one_out.append(None)
            continue
        try:
            r, t = solve_pose(object_points[keep], image_points[keep], camera_matrix, distortion)
        except (ValueError, cv2.error):
            leave_one_out.append(None)
            continue
        predicted, _ = cv2.projectPoints(object_points[index : index + 1], r, t, camera_matrix, distortion)
        leave_one_out.append(round(float(np.linalg.norm(predicted.reshape(2) - image_points[index])), 2))

    overlay = image.copy()
    slots_out = []
    active = [slot for slot in slot_map["slots"] if slot["active"]]  # 사용 칸 6개만 투영
    def projected_hull(corners: np.ndarray) -> np.ndarray:
        """3D 상자 꼭짓점을 화면에 투영해 볼록 껍질 다각형(px)을 돌려준다.

        카메라 뒤(z ≤ 0)에 있는 꼭짓점은 투영 결과가 뒤집혀 엉뚱한 다각형이 되므로 거부한다.
        오류 메시지의 slot은 아래 반복문의 현재 칸이다.
        """
        camera_side = (rotation @ corners.T + tvec).T
        if np.any(camera_side[:, 2] <= 0):
            raise ValueError(f"{slot['slot_id']} projects behind the camera; check the clicked points")
        pixels, _ = cv2.projectPoints(corners, rvec, tvec, camera_matrix, distortion)
        return cv2.convexHull(pixels.reshape(-1, 2).astype(np.float32)).reshape(-1, 2)

    for index, slot in enumerate(active):
        if not isinstance(slot.get("lip_top_height_m"), (int, float)):
            raise ValueError(f"{slot['slot_id']} has no lip_top_height_m; regenerate the slot map")
        corners = slot_volume_corners(slot, open_height)
        hull = projected_hull(corners)
        # 검출의 아래 가운데 점이 나타날 수 있는 곳: 바닥판부터 앞 턱 바로 위까지.
        # 턱 뒤에 선 물건은 바닥이 턱 윗선이나 그보다 조금 위에 보인다.
        # 칸 전체 공간으로 판정하면 아래 칸 영역이 위 단 바로 밑까지 올라가 위 단 물건의 바닥점이 걸리므로,
        # 판정은 이 얇은 영역(anchor)으로 하고 칸 전체(hull)는 참고로만 그린다.
        anchor_top = slot["lip_top_height_m"] + anchor_margin
        anchor = projected_hull(slot_volume_corners(slot, open_height, top=anchor_top))
        deck_px = projected_hull(slot_volume_corners(slot, open_height, top=slot["deck_height_m"]))
        color = COLORS[index % len(COLORS)]
        # 판정 영역은 35% 반투명으로 채우고, 칸 전체는 가는 선, 판정 영역 경계는 굵은 선으로 그린다.
        layer = overlay.copy()
        cv2.fillPoly(layer, [np.round(anchor).astype(np.int32)], color)
        overlay = cv2.addWeighted(layer, 0.35, overlay, 0.65, 0)
        cv2.polylines(overlay, [np.round(hull).astype(np.int32)], True, color, 1, cv2.LINE_AA)
        cv2.polylines(overlay, [np.round(anchor).astype(np.int32)], True, color, 2, cv2.LINE_AA)
        center = anchor.mean(axis=0)
        # 같은 글자를 흰색 굵게 한 번, 칸 색 가늘게 한 번 써서 배경과 상관없이 읽히게 한다.
        cv2.putText(overlay, slot["slot_id"], (int(center[0]) - 34, int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, slot["slot_id"], (int(center[0]) - 34, int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
        slots_out.append(
            {
                "slot_id": slot["slot_id"],
                "label": slot["label"],
                "volume_height_m": round(float(corners[:, 2].max() - corners[:, 2].min()), 4),
                "volume_height_source": "clear_height" if slot["clear_height_m"] is not None else "open_slot_height",
                "polygon_px": [[round(float(x), 1), round(float(y), 1)] for x, y in hull],
                "anchor_top_height_m": round(float(anchor_top), 4),
                "anchor_polygon_px": [[round(float(x), 1), round(float(y), 1)] for x, y in anchor],
                "deck_polygon_px": [[round(float(x), 1), round(float(y), 1)] for x, y in deck_px],
            }
        )
    # 검토용 표시: 초록 십자 = 운영자가 찍은 점, 빨간 원 = 구한 자세로 다시 투영한 위치. 겹쳐야 한다.
    for (x, y), (px, py) in zip(image_points, projected.reshape(-1, 2)):
        cv2.drawMarker(overlay, (int(round(x)), int(round(y))), (80, 255, 80), cv2.MARKER_CROSS, 10, 1, cv2.LINE_AA)
        cv2.circle(overlay, (int(round(px)), int(round(py))), 3, (60, 60, 255), 1, cv2.LINE_AA)

    artifact = {
        "schema_version": 1,
        "pose_id": str(uuid.uuid4()),
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "DRAFT_ONLY",
        "review_required": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "camera_id": intrinsics["camera_id"],
        "pixel_transform_id": intrinsics["pixel_transform_id"],
        "image_size_px": {"width": width, "height": height},
        "frame_id": slot_map["frame"]["frame_id"],
        "sources": {
            "points_json_sha256": points_sha,
            "source_image_sha256": image_sha,
            "intrinsic_calibration_id": intrinsics["calibration_id"],
            "intrinsic_file_sha256": intrinsics_sha,
            "slot_map_sha256": slot_map_sha,
        },
        "solver": {"method": "SOLVEPNP_SQPNP + solvePnPRefineLM", "opencv_version": cv2.__version__},
        # 자세와 함께 보관해, 이 파일 하나만으로 픽셀을 선반 좌표의 점으로 되돌릴 수 있게 한다.
        "intrinsics": {
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": distortion.tolist(),
        },
        "pose": {
            "rvec": [round(float(v), 6) for v in rvec.reshape(3)],
            "tvec_m": [round(float(v), 5) for v in tvec.reshape(3)],
            "camera_position_in_shelf_frame_m": {
                axis: round(float(value), 4) for axis, value in zip(("x", "y", "z"), camera_in_shelf)
            },
        },
        "points": [
            {
                "keypoint_id": keypoint_id,
                "image_px": [round(float(v), 2) for v in image_points[index]],
                "reprojection_error_px": round(float(residuals[index]), 2),
                "leave_one_out_error_px": leave_one_out[index],
            }
            for index, keypoint_id in enumerate(ids)
        ],
        "reprojection_rms_px": round(float(np.sqrt(np.mean(residuals**2))), 3),
        "open_slot_height_m": open_height,
        "anchor_margin_m": anchor_margin,
        "slots": slots_out,
    }
    return artifact, overlay


def _write_exclusive(path: Path, data: bytes) -> None:
    """임시 파일에 쓴 뒤 install_file_exclusive로 옮긴다. 같은 이름이 있으면 덮어쓰지 않고 실패한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", delete=False) as handle:
            handle.write(data)
            temporary = Path(handle.name)
        install_file_exclusive(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main() -> int:
    """인수를 검사하고 입력을 읽어 결과 JSON과 오버레이 PNG를 저장한 뒤 점별 오차를 출력한다. 성공 0, 오류 1."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("points", type=Path, help="shelf-pose-points.json from shelf_pose_editor.html")
    parser.add_argument("image", type=Path, help="the same source frame the points were clicked on")
    parser.add_argument(
        "--intrinsics", type=Path, default=Path(".control-station/camera-calibrations/shelf_camera.json")
    )
    parser.add_argument("--slots", type=Path, default=Path("config/shelf-slots.json"))
    parser.add_argument(
        "--open-slot-height",
        type=float,
        default=DEFAULT_OPEN_SLOT_HEIGHT_M,
        help="height (m) drawn for slots with an open top; display/matching assumption, not a measurement",
    )
    parser.add_argument(
        "--anchor-margin",
        type=float,
        default=0.02,
        help="height (m) above each front lip still counted as the slot for a detection's bottom-center (assumption)",
    )
    parser.add_argument("--output", type=Path, help="artifact JSON path (default: managed calibration directory)")
    args = parser.parse_args()
    try:
        # 가정값의 허용 범위: 판정 여유 0–10 cm, 열린 칸 높이 0 초과–50 cm. 그 밖은 오타로 본다.
        if not math.isfinite(args.anchor_margin) or not 0 <= args.anchor_margin <= 0.1:
            raise ValueError("--anchor-margin must be in [0, 0.1] m")
        if not math.isfinite(args.open_slot_height) or not 0 < args.open_slot_height <= 0.5:
            raise ValueError("--open-slot-height must be in (0, 0.5] m")
        points_doc, points_sha = _load_json(args.points, "points JSON")
        intrinsics, intrinsics_sha = _load_json(args.intrinsics, "intrinsic calibration")
        validate_intrinsic_draft(intrinsics)  # 내부 보정 파일 형식이 만든 쪽 계약과 맞는지 확인
        slot_map, slot_map_sha = _load_json(args.slots, "slot map")
        image_bytes = read_regular_file_bounded(args.image, MAX_CALIBRATION_IMAGE_BYTES, "source image")
        image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("source image could not be decoded")
        artifact, overlay = build_artifact(
            points_doc, points_sha, image, hashlib.sha256(image_bytes).hexdigest(),
            intrinsics, intrinsics_sha, slot_map, slot_map_sha, args.open_slot_height, args.anchor_margin,
        )
        output = args.output or Path(".control-station/camera-calibrations") / (
            f"{artifact['camera_id']}-shelf-pose-{artifact['pose_id'][:8]}.json"
        )
        # 출력이 입력 파일(경로 alias·hard link 포함)과 같으면 거부해 입력을 덮어쓰지 않게 한다.
        ensure_distinct_output_path(output, [args.points, args.image, args.intrinsics, args.slots])
        ok, png = cv2.imencode(".png", overlay)
        if not ok:
            raise ValueError("overlay PNG could not be encoded")
        overlay_path = output.with_suffix(".overlay.png")
        artifact["overlay_png"] = overlay_path.name
        _write_exclusive(output, (json.dumps(artifact, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
        _write_exclusive(overlay_path, png.tobytes())
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    position = artifact["pose"]["camera_position_in_shelf_frame_m"]
    # shelf_front 좌표의 x는 선반 안쪽이 +이므로, 선반 앞에 있는 카메라는 x가 음수다. 그래서 부호를 바꿔
    # "앞으로 n cm"로 보여 준다. y는 정면에서 볼 때 왼쪽이 +, z는 선반 밑면 기준 높이다.
    print(f"재투영 RMS {artifact['reprojection_rms_px']} px, 기준점 {len(artifact['points'])}개")
    for point in artifact["points"]:
        print(f"  {point['keypoint_id']:32s} 오차 {point['reprojection_error_px']:5.2f} px  "
              f"빼고 예측 {point['leave_one_out_error_px'] if point['leave_one_out_error_px'] is not None else '-'} px")
    print(
        f"카메라 위치 (선반 기준): 앞으로 {-position['x']*100:.1f} cm, 왼쪽 {position['y']*100:.1f} cm, "
        f"높이 {position['z']*100:.1f} cm"
    )
    print(f"저장: {output}\n오버레이: {overlay_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
