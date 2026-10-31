#!/usr/bin/env python3
"""Fit and review a fixed shelf-plane homography from labeled image correspondences."""

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
from typing import Any

import cv2
import numpy as np

from calibration_artifacts import (
    MAX_CALIBRATION_IMAGE_BYTES,
    MAX_TCP_CALIBRATION_BYTES,
    ensure_distinct_output_path,
    install_file_exclusive,
    read_regular_file_bounded,
    unique_json_object,
    validate_intrinsic_draft,
    validate_plane_sample_sources,
    validate_plane_source_provenance,
)


_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")


def _finite_point(value: object, field: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field}는 숫자 2개를 가진 배열이어야 합니다.")
    if any(isinstance(number, bool) or not isinstance(number, (int, float)) for number in value):
        raise ValueError(f"{field}의 값은 숫자여야 합니다.")
    try:
        point = float(value[0]), float(value[1])
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{field}의 값은 유한한 숫자여야 합니다.") from exc
    if not all(math.isfinite(number) for number in point):
        raise ValueError(f"{field}의 값은 유한해야 합니다.")
    return point


def _write_json(path: Path, payload: dict[str, Any]) -> None:
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


def _load_intrinsics(path: Path) -> dict[str, Any]:
    raw = read_regular_file_bounded(path, MAX_TCP_CALIBRATION_BYTES, "intrinsic calibration")
    payload = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_json_object)
    if not isinstance(payload, dict):
        raise ValueError("내부 보정 파일의 최상위 값은 객체여야 합니다.")
    validate_intrinsic_draft(payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("correspondences", type=Path, help="대응점 입력 JSON")
    parser.add_argument("intrinsics", type=Path, help="내부 파라미터 DRAFT_ONLY JSON")
    parser.add_argument("--output", type=Path, help="결과 JSON 경로")
    args = parser.parse_args()

    try:
        source_bytes = read_regular_file_bounded(
            args.correspondences, MAX_TCP_CALIBRATION_BYTES, "correspondence JSON"
        )
        source_sha256 = hashlib.sha256(source_bytes).hexdigest()
        source = json.loads(
            source_bytes.decode("utf-8"), object_pairs_hook=unique_json_object
        )
        if not isinstance(source, dict):
            raise ValueError("대응점 입력의 최상위 값은 객체여야 합니다.")
        if set(source) != {
            "schema_version", "camera_id", "pixel_transform_id", "plane_frame", "image_size_px", "samples"
        }:
            raise ValueError("대응점 입력에는 schema v1 필드만 포함되어야 합니다.")
        source_schema_version = source.get("schema_version")
        if (
            isinstance(source_schema_version, bool)
            or not isinstance(source_schema_version, int)
            or source_schema_version != 1
        ):
            raise ValueError("지원하지 않는 대응점 입력 schema_version입니다.")
        intrinsic = _load_intrinsics(args.intrinsics)

        camera_id = source.get("camera_id")
        pixel_transform_id = source.get("pixel_transform_id")
        plane = source.get("plane_frame")
        image_size = source.get("image_size_px")
        samples = source.get("samples")
        if not isinstance(camera_id, str) or not _IDENTIFIER.fullmatch(camera_id):
            raise ValueError("camera_id 형식이 올바르지 않습니다.")
        if camera_id != intrinsic.get("camera_id"):
            raise ValueError("대응점과 내부 보정의 camera_id가 다릅니다.")
        if (
            not isinstance(pixel_transform_id, str)
            or not _IDENTIFIER.fullmatch(pixel_transform_id)
            or pixel_transform_id != intrinsic.get("pixel_transform_id")
        ):
            raise ValueError("대응점 pixel_transform_id와 내부 보정 값이 없거나 일치하지 않습니다.")
        if not isinstance(plane, dict):
            raise ValueError("plane_frame 객체가 필요합니다.")
        if set(plane) != {"frame_id", "unit", "axis_definition"}:
            raise ValueError("plane_frame에는 frame_id, unit, axis_definition만 포함되어야 합니다.")
        frame_id, unit, axis_definition = (
            plane.get("frame_id"), plane.get("unit"), plane.get("axis_definition")
        )
        if not isinstance(frame_id, str) or not _IDENTIFIER.fullmatch(frame_id):
            raise ValueError("plane_frame.frame_id 형식이 올바르지 않습니다.")
        if unit not in {"m", "mm"}:
            raise ValueError("plane_frame.unit은 m 또는 mm여야 합니다.")
        if (
            not isinstance(axis_definition, str)
            or not axis_definition.strip()
            or len(axis_definition) > 200
        ):
            raise ValueError("plane_frame.axis_definition에 축 방향 설명(1~200자)이 필요합니다.")
        if not isinstance(image_size, dict):
            raise ValueError("image_size_px 객체가 필요합니다.")
        width, height = image_size.get("width"), image_size.get("height")
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value <= 0
            for value in (width, height)
        ):
            raise ValueError("image_size_px.width/height는 양의 정수여야 합니다.")
        if image_size != intrinsic.get("image_size_px"):
            raise ValueError("대응점 해상도와 내부 보정 해상도가 다릅니다.")
        if not isinstance(samples, list):
            raise ValueError("samples 배열이 필요합니다.")

        camera_matrix = np.asarray(intrinsic.get("camera_matrix"), dtype=np.float64)
        distortion = np.asarray(
            intrinsic.get("distortion_coefficients"), dtype=np.float64
        ).reshape(-1, 1)
        if camera_matrix.shape != (3, 3) or not np.isfinite(camera_matrix).all():
            raise ValueError("camera_matrix는 유한한 3×3 행렬이어야 합니다.")
        if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
            raise ValueError("camera_matrix의 초점 거리 항목은 양수여야 합니다.")
        if distortion.size not in {4, 5, 8, 12, 14} or not np.isfinite(distortion).all():
            raise ValueError("왜곡 계수는 유한한 OpenCV 계수 4/5/8/12/14개여야 합니다.")
        intrinsic_id = intrinsic.get("calibration_id")
        if not isinstance(intrinsic_id, str) or not intrinsic_id:
            raise ValueError("내부 보정 calibration_id가 필요합니다.")

        fit_samples: list[dict[str, Any]] = []
        validation_samples: list[dict[str, Any]] = []
        seen_image_pixels: set[tuple[str, float, float]] = set()
        seen_plane_points: set[tuple[float, float]] = set()
        correspondence_root = args.correspondences.resolve().parent
        source_image_hashes: dict[str, str] = {}
        for index, sample in enumerate(samples):
            if not isinstance(sample, dict):
                raise ValueError(f"samples[{index}]는 객체여야 합니다.")
            if set(sample) != {"source_image", "image_px", "plane_xy", "role"}:
                raise ValueError(f"samples[{index}]에는 source_image/image_px/plane_xy/role만 허용됩니다.")
            raw_pixel = _finite_point(sample.get("image_px"), f"samples[{index}].image_px")
            plane_xy = _finite_point(sample.get("plane_xy"), f"samples[{index}].plane_xy")
            if not (0 <= raw_pixel[0] < width and 0 <= raw_pixel[1] < height):
                raise ValueError(f"samples[{index}].image_px가 원본 이미지 범위를 벗어났습니다.")
            role = sample.get("role")
            if role not in {"fit", "validation"}:
                raise ValueError(f"samples[{index}].role은 fit 또는 validation이어야 합니다.")
            source_image = sample.get("source_image")
            if not isinstance(source_image, str) or not source_image.strip() or len(source_image) > 255:
                raise ValueError(f"samples[{index}].source_image에는 원본 이미지 식별자(1~255자)가 필요합니다.")
            source_image_path = Path(source_image.strip())
            if source_image_path.is_absolute() or "\\" in source_image or ".." in source_image_path.parts:
                raise ValueError(f"samples[{index}].source_image는 입력 파일 폴더 안의 상대 경로여야 합니다.")
            source_component = correspondence_root
            for part in source_image_path.parts:
                source_component = source_component / part
                if source_component.is_symlink():
                    raise ValueError(f"samples[{index}].source_image 경로는 symbolic link를 포함할 수 없습니다.")
            resolved_source_image = (correspondence_root / source_image_path).resolve()
            if correspondence_root not in resolved_source_image.parents or not resolved_source_image.is_file():
                raise ValueError(f"samples[{index}].source_image 파일이 입력 폴더 안에 없습니다.")
            normalized_source_image = resolved_source_image.relative_to(correspondence_root).as_posix()
            if normalized_source_image not in source_image_hashes:
                source_image_bytes = read_regular_file_bounded(
                    resolved_source_image,
                    MAX_CALIBRATION_IMAGE_BYTES,
                    "correspondence source image",
                )
                decoded_source = cv2.imdecode(
                    np.frombuffer(source_image_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED
                )
                if decoded_source is None or decoded_source.ndim < 2:
                    raise ValueError(
                        f"samples[{index}].source_image를 이미지로 decode할 수 없습니다."
                    )
                if (int(decoded_source.shape[1]), int(decoded_source.shape[0])) != (width, height):
                    raise ValueError(
                        f"samples[{index}].source_image 해상도가 대응점/intrinsic 해상도와 다릅니다."
                    )
                source_image_hashes[normalized_source_image] = hashlib.sha256(
                    source_image_bytes
                ).hexdigest()
            image_pixel_key = (normalized_source_image, *raw_pixel)
            if image_pixel_key in seen_image_pixels or plane_xy in seen_plane_points:
                raise ValueError(
                    f"samples[{index}]는 같은 원본 이미지 안의 중복 pixel 또는 중복 plane 좌표를 가집니다."
                )
            seen_image_pixels.add(image_pixel_key)
            seen_plane_points.add(plane_xy)
            row = {
                "source_image": normalized_source_image,
                "source_image_sha256": source_image_hashes[normalized_source_image],
                "image_px": list(raw_pixel),
                "plane_xy": list(plane_xy),
                "role": role,
            }
            (fit_samples if role == "fit" else validation_samples).append(row)
        if len(fit_samples) < 4:
            raise ValueError("변환 계산용 fit 대응점이 최소 4개 필요합니다.")
        if len(validation_samples) < 2:
            raise ValueError("독립 오차 확인용 validation 대응점이 최소 2개 필요합니다.")

        all_samples = fit_samples + validation_samples
        raw_pixels = np.asarray([sample["image_px"] for sample in all_samples], dtype=np.float64)
        undistorted = cv2.undistortPoints(
            raw_pixels.reshape(-1, 1, 2), camera_matrix, distortion, P=camera_matrix
        ).reshape(-1, 2)
        for sample, pixel in zip(all_samples, undistorted):
            sample["undistorted_image_px"] = pixel.tolist()

        fit_pixels = np.asarray(
            [sample["undistorted_image_px"] for sample in fit_samples], dtype=np.float64
        )
        fit_plane = np.asarray([sample["plane_xy"] for sample in fit_samples], dtype=np.float64)
        homography, _ = cv2.findHomography(fit_pixels, fit_plane, method=0)
        if (
            homography is None
            or not np.isfinite(homography).all()
            or np.linalg.matrix_rank(homography) < 3
        ):
            raise ValueError("대응점으로 유효한 평면 변환을 계산하지 못했습니다.")

        undistorted_points = np.asarray(
            [sample["undistorted_image_px"] for sample in all_samples], dtype=np.float64
        ).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(undistorted_points, homography).reshape(-1, 2)
        errors = np.linalg.norm(
            projected
            - np.asarray([sample["plane_xy"] for sample in all_samples], dtype=np.float64),
            axis=1,
        )
        fit_count = len(fit_samples)
        validation_errors = errors[fit_count:]
        for sample, point, error in zip(all_samples, projected, errors):
            sample["projected_plane_xy"] = point.tolist()
            sample["residual"] = float(error)
            sample["residual_unit"] = unit

        calibration_id = str(uuid.uuid4())
        output_path = args.output or (
            Path(".control-station/camera-calibrations") / f"{camera_id}-plane-{calibration_id}.json"
        )
        ensure_distinct_output_path(
            output_path,
            [
                args.correspondences,
                args.intrinsics,
                *(correspondence_root / name for name in source_image_hashes),
            ],
        )
        payload: dict[str, Any] = {
            "schema_version": 4,
            "calibration_id": calibration_id,
            "created_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "status": "DRAFT_ONLY",
            "calibration_scope": "CAMERA_TO_FIXED_PLANE_ONLY",
            "usable_for_robot_control": False,
            "arm_command_sent": False,
            "camera_id": camera_id,
            "pixel_transform_id": pixel_transform_id,
            "source_correspondence_schema_version": source_schema_version,
            "source_correspondence_file": args.correspondences.name,
            "source_correspondence_sha256": source_sha256,
            "intrinsic_calibration_id": intrinsic_id,
            "image_size_px": {"width": width, "height": height},
            "plane_frame": {
                "frame_id": frame_id,
                "unit": unit,
                "axis_definition": axis_definition.strip(),
            },
            "input_pixel_domain": "DISTORTED_IMAGE_PIXELS_UNDISTORTED_WITH_REFERENCED_INTRINSICS",
            "homography_undistorted_image_px_to_plane_xy": homography.tolist(),
            "fit_point_count": fit_count,
            "validation_point_count": len(validation_samples),
            "fit_rms_residual": float(math.sqrt(float(np.mean(errors[:fit_count] ** 2)))),
            "validation_rms_residual": float(math.sqrt(float(np.mean(validation_errors ** 2)))),
            "residual_unit": unit,
            "samples": all_samples,
            "review_required": True,
        }
        validate_plane_source_provenance(payload)
        validate_plane_sample_sources(payload)
        _write_json(output_path, payload)
    except (
        OSError,
        RuntimeError,
        json.JSONDecodeError,
        TypeError,
        ValueError,
        cv2.error,
    ) as exc:
        print(f"평면 보정 초안을 만들지 못했습니다: {exc}", file=sys.stderr)
        return 2

    print(f"고정 평면 보정 초안 저장: {output_path}")
    print(
        f"fit {len(fit_samples)}점 · validation {len(validation_samples)}점 · "
        f"validation RMS {payload['validation_rms_residual']:.4f} {unit}"
    )
    print("DRAFT_ONLY · OMX frame/IK/작업 영역 없음 · 로봇 명령을 보내지 않았습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
