#!/usr/bin/env python3
"""Estimate camera intrinsics from saved chessboard images; never connects to a robot."""

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


_CAMERA_ID = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
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

    image_paths = sorted(
        path
        for path in args.image_dir.iterdir()
        if (path.is_file() or path.is_symlink()) and path.suffix.lower() in _IMAGE_SUFFIXES
    )
    if not image_paths:
        print("지원하는 이미지 파일을 찾지 못했습니다.", file=sys.stderr)
        return 2

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
        try:
            image_bytes = read_regular_file_bounded(
                image_path, MAX_CALIBRATION_IMAGE_BYTES, "checkerboard image"
            )
        except (OSError, ValueError) as exc:
            rejected_files.append({"file": image_path.name, "reason": str(exc)})
            continue
        image_digest = hashlib.sha256(image_bytes).hexdigest()
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
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is not None and current_size != image_size:
            rejected_files.append({"file": image_path.name, "reason": "이미지 해상도가 서로 다름"})
            continue
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
        ensure_distinct_output_path(output_path, image_paths)
        # Keep the producer contract aligned with the validators used by downstream
        # calibration consumers before writing a review artifact.
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
