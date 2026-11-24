#!/usr/bin/env python3
"""저장된 체커보드 사진으로 카메라 내부 파라미터를 추정한다. 로봇에는 절대 연결하지 않는다.

사용법:
    python scripts/calibrate_camera_intrinsics.py <사진 폴더> --camera-id head_camera
        --pixel-transform-id opencv-v4l2-capture-no-app-geometry-v1
        --columns 8 --rows 6 --square-size 25 --unit mm [--output 경로]

카메라 한 대에서 같은 해상도로 찍은 체커보드 사진 폴더를 읽어 OpenCV로 camera matrix와 왜곡 계수를 계산하고,
.control-station/camera-calibrations/{camera_id}.json에 검토용 초안(DRAFT_ONLY, schema v2)으로 저장한다.
--columns/--rows는 칸 수가 아니라 내부 교차점 수다(팀 보드 9×7칸 25 mm → 8×6).

파일만 읽고 JSON 하나를 쓸 뿐, 카메라 장치를 열거나 로봇·관제 API와 통신하지 않는다. 결과는 렌즈 내부 값뿐이며
카메라 장착 위치(extrinsic)나 팔 좌표 변환은 담지 않는다. 결과 파일은 천장 카메라 주차 판정(cpp/parking_vision)과
정면 카메라 선반 위치 보정(scripts/calibrate_shelf_pose.py)이 읽는다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from calibration_artifacts import (
    MAX_CALIBRATION_IMAGE_BYTES,
    ensure_distinct_output_path,
    install_file_exclusive,
    read_regular_file_bounded,
    validate_intrinsic_draft,
)


# camera_id는 기본 출력 파일 이름이 되므로 경로 구분자 없이 영문자·숫자·_·.·-만 허용한다.
_CAMERA_ID = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
# 계산을 시작할 최소 사진 수. 품질 보증이 아니라 하한일 뿐이며, 실제로는 각도·거리·화면 위치를 바꿔
# 더 많이 찍는다(2026-10-31 천장 카메라 35장, 정면 카메라 23장).
_MINIMUM_VIEWS = 6


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_dir", type=Path, help="체커보드 캡처 이미지가 있는 폴더")
    parser.add_argument("--camera-id", required=True, help="카메라 설정 ID")
    parser.add_argument(
        "--pixel-transform-id",
        required=True,
        help="카메라 관측 API의 pixel_transform_id 값",
    )
    parser.add_argument("--columns", type=int, required=True, help="체커보드 내부 코너 열 수")
    parser.add_argument("--rows", type=int, required=True, help="체커보드 내부 코너 행 수")
    parser.add_argument("--square-size", type=float, required=True, help="체커보드 한 칸의 실제 길이")
    parser.add_argument("--unit", choices=("mm", "m"), required=True, help="한 칸 길이 단위")
    parser.add_argument(
        "--output",
        type=Path,
        help="결과 JSON 경로 (기본: .control-station/camera-calibrations/{camera_id}.json)",
    )
    return parser.parse_args()


def _write_json(path: Path, payload: dict[str, object]) -> None:
    """같은 폴더의 임시 파일에 다 쓴 뒤 install_file_exclusive로 옮긴다.

    출력 파일이 이미 있으면 덮어쓰지 않고 실패하며, 중간에 실패해도 반쯤 쓴 JSON이 남지 않는다.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            json.dump(payload, temporary_file, ensure_ascii=False, indent=2, allow_nan=False)
            temporary_file.write("\n")
        install_file_exclusive(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def main() -> int:
    """입력을 검사하고 사진마다 코너를 찾아 보정한 뒤 초안 JSON을 저장한다. 성공 0, 입력·계산 오류 2."""
    args = _parse_args()
    if not _CAMERA_ID.fullmatch(args.camera_id):
        print("camera-id는 영문자, 숫자, _, ., -만 포함할 수 있습니다.", file=sys.stderr)
        return 2
    if not _CAMERA_ID.fullmatch(args.pixel_transform_id):
        print("pixel-transform-id 형식이 올바르지 않습니다.", file=sys.stderr)
        return 2
    if args.columns < 2 or args.rows < 2:
        print("체커보드 내부 코너 열/행 수는 각각 2 이상이어야 합니다.", file=sys.stderr)
        return 2
    if not math.isfinite(args.square_size) or args.square_size <= 0:
        print("square-size는 유한한 양수여야 합니다.", file=sys.stderr)
        return 2
    if not args.image_dir.is_dir():
        print(f"이미지 폴더가 없습니다: {args.image_dir}", file=sys.stderr)
        return 2

    # symlink도 목록에 넣는다. 보정에 쓰지는 않지만 read_regular_file_bounded가 거부한 이유를
    # rejected_images에 남기도록 하기 위해서다.
    image_paths = sorted(
        path
        for path in args.image_dir.iterdir()
        if (path.is_file() or path.is_symlink()) and path.suffix.lower() in _IMAGE_SUFFIXES
    )
    if not image_paths:
        print("지원하는 이미지 파일을 찾지 못했습니다.", file=sys.stderr)
        return 2

    # 체커보드 내부 코너의 실제 3D 좌표. 보드 평면을 z=0으로 두고 (열, 행) 격자에 한 칸 길이를 곱한다.
    # 단위는 --unit 그대로(mm 또는 m)이며 내부 파라미터 결과에는 영향이 없다.
    board_size = (args.columns, args.rows)
    board_points = np.zeros((args.rows * args.columns, 3), dtype=np.float32)
    board_points[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2)
    board_points *= args.square_size
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    accepted_files: list[str] = []
    accepted_sha256: list[str] = []
    rejected_files: list[dict[str, str]] = []
    image_size: tuple[int, int] | None = None

    for image_path in image_paths:
        # 파일당 최대 64 MiB, 일반 파일만 읽는다. 해시와 decode를 같은 bytes로 해서 기록한 SHA-256이
        # 실제로 계산에 쓴 내용과 일치하게 한다.
        try:
            image_bytes = read_regular_file_bounded(
                image_path, MAX_CALIBRATION_IMAGE_BYTES, "checkerboard image"
            )
        except (OSError, ValueError) as exc:
            rejected_files.append({"file": image_path.name, "reason": str(exc)})
            continue
        image_digest = hashlib.sha256(image_bytes).hexdigest()
        # 바이트가 같은 복제 사진은 새 시점이 아니므로 view 수에 넣지 않는다.
        if image_digest in accepted_sha256:
            rejected_files.append({"file": image_path.name, "reason": "이미지 내용이 다른 승인 이미지와 동일함"})
            continue
        try:
            image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        except cv2.error:
            image = None
        if image is None:
            rejected_files.append({"file": image_path.name, "reason": "이미지를 읽을 수 없음"})
            continue
        # 내부 파라미터는 해상도마다 다르므로, 처음 승인된 사진의 해상도와 다른 사진은 버린다.
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is not None and current_size != image_size:
            rejected_files.append({"file": image_path.name, "reason": "이미지 해상도가 서로 다름"})
            continue
        # SB(sector-based) 검출기는 기본 검출기보다 잡음에 강하고 서브픽셀 정확도의 코너를 바로 돌려준다.
        found, corners = cv2.findChessboardCornersSB(image, board_size)
        if not found or corners is None:
            rejected_files.append({"file": image_path.name, "reason": "체커보드 코너를 찾지 못함"})
            continue
        image_size = current_size
        object_points.append(board_points.copy())
        image_points.append(corners.astype(np.float32))
        accepted_files.append(image_path.name)
        accepted_sha256.append(image_digest)

    if len(accepted_files) < _MINIMUM_VIEWS or image_size is None:
        print(
            f"보정에는 같은 해상도에서 코너를 찾은 이미지가 최소 {_MINIMUM_VIEWS}장 필요합니다. "
            f"현재 유효 이미지: {len(accepted_files)}장",
            file=sys.stderr,
        )
        return 2

    try:
        rms, camera_matrix, distortion, rotations, translations = cv2.calibrateCamera(
            object_points, image_points, image_size, None, None
        )
    except cv2.error as exc:
        print(f"OpenCV 내부 파라미터 계산 실패: {exc}", file=sys.stderr)
        return 2
    # 사진별 재투영 오차(px): 추정한 내부 값과 그 사진의 보드 자세로 3D 코너를 다시 투영해 검출 코너와의
    # L2 거리를 RMS로 낸다. 특정 사진만 크면 그 사진의 코너 검출이 잘못됐을 가능성이 있어 검토에 쓴다.
    per_image_errors: list[dict[str, object]] = []
    for image_name, image_digest, object_view, image_view, rotation, translation in zip(
        accepted_files, accepted_sha256, object_points, image_points, rotations, translations
    ):
        projected, _ = cv2.projectPoints(
            object_view, rotation, translation, camera_matrix, distortion
        )
        error_px = cv2.norm(image_view, projected, cv2.NORM_L2) / math.sqrt(len(projected))
        per_image_errors.append(
            {
                "file": image_name,
                "sha256": image_digest,
                "reprojection_error_px": float(error_px),
            }
        )

    # 기본 출력 경로는 현재 작업 디렉터리 기준이므로 저장소 루트에서 실행한다.
    # status DRAFT_ONLY, usable_for_robot_control false, review_required true: 사람이 검토하기 전의 초안이며
    # 로봇 제어 경로가 읽는 값이 아니라는 표시다.
    output_path = args.output or Path(".control-station/camera-calibrations") / f"{args.camera_id}.json"
    payload: dict[str, object] = {
        "schema_version": 2,
        "calibration_id": str(uuid.uuid4()),
        "created_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "status": "DRAFT_ONLY",
        "calibration_scope": "CAMERA_INTRINSICS_ONLY",
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "camera_id": args.camera_id,
        "pixel_transform_id": args.pixel_transform_id,
        "opencv_version": cv2.__version__,
        "image_size_px": {"width": image_size[0], "height": image_size[1]},
        "checkerboard": {
            "inner_corner_columns": args.columns,
            "inner_corner_rows": args.rows,
            "square_size": args.square_size,
            "unit": args.unit,
        },
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "rms_reprojection_error_px": float(rms),
        "calibrated_view_count": len(accepted_files),
        "per_image_reprojection_errors": per_image_errors,
        "source_images": accepted_files,
        "rejected_images": rejected_files,
        "review_required": True,
    }
    try:
        # 출력 경로가 입력 사진 중 하나(경로 alias·hard link 포함)와 같으면 거부한다.
        ensure_distinct_output_path(output_path, image_paths)
        # 검토용 산출물을 쓰기 전에, 이 결과를 읽는 쪽이 쓰는 검증기로 먼저 확인해
        # 만드는 쪽과 읽는 쪽의 형식 약속이 어긋나지 않게 한다.
        validate_intrinsic_draft(payload)
        _write_json(output_path, payload)
    except (OSError, TypeError, ValueError, OverflowError) as exc:
        print(f"보정 초안을 저장하지 못했습니다: {exc}", file=sys.stderr)
        return 2

    print(f"카메라 내부 파라미터 초안 저장: {output_path}")
    print(f"유효 이미지: {len(accepted_files)}장 · RMS 재투영 오차: {rms:.4f}px")
    print("DRAFT_ONLY · camera-to-robot 변환/IK 없음 · 로봇 명령을 보내지 않았습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
