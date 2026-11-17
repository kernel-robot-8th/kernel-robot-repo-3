"""Shared validation for offline, review-only camera calibration JSON artifacts."""

from __future__ import annotations

import math
import os
import re
from pathlib import Path
from typing import Any, Iterable


MAX_CALIBRATION_IMAGE_BYTES = 64 * 1024 * 1024


def unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """Build a JSON object only when each member name occurs once."""
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"JSON object contains a duplicate key: {key}")
        value[key] = item
    return value


def read_regular_file_bounded(path: Path, maximum_bytes: int, label: str) -> bytes:
    """Read a non-symlink regular file with a hard byte limit."""
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular non-symlink file")
    try:
        if path.stat().st_size > maximum_bytes:
            raise ValueError(f"{label} exceeds the {maximum_bytes}-byte limit")
        with path.open("rb") as source:
            payload = source.read(maximum_bytes + 1)
    except OSError as exc:
        raise ValueError(f"{label} cannot be read") from exc
    if len(payload) > maximum_bytes:
        raise ValueError(f"{label} exceeds the {maximum_bytes}-byte limit")
    return payload


def install_file_exclusive(temporary_path: Path, output_path: Path) -> None:
    """Atomically publish a completed same-directory temporary file without replacement."""
    try:
        os.link(temporary_path, output_path)
    except FileExistsError as exc:
        raise ValueError(f"output already exists; refusing to overwrite: {output_path}") from exc


def ensure_distinct_output_path(output_path: Path, input_paths: Iterable[Path]) -> None:
    """Reject an output path that resolves to any input artifact or source image."""
    try:
        resolved_output = output_path.expanduser().resolve(strict=False)
    except (OSError, RuntimeError) as exc:
        raise ValueError("calibration output path cannot be resolved safely") from exc
    for input_path in input_paths:
        try:
            resolved_input = input_path.expanduser().resolve(strict=False)
        except (OSError, RuntimeError) as exc:
            raise ValueError("calibration input path cannot be normalized safely") from exc
        try:
            same_file = (
                output_path.exists()
                and input_path.exists()
                and os.path.samefile(output_path, input_path)
            )
        except OSError as exc:
            raise ValueError("calibration output/input identity cannot be checked safely") from exc
        if resolved_output == resolved_input or same_file:
            raise ValueError(
                "calibration output must not overwrite an input artifact or source image"
            )


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    return number if math.isfinite(number) else None


def validate_intrinsic_draft(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate intrinsic artifact schema, provenance, geometry fields, and RMS metadata."""
    schema_version = payload.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 2
    ):
        raise ValueError("unsupported intrinsic calibration schema_version")
    if (
        payload.get("status") != "DRAFT_ONLY"
        or payload.get("calibration_scope") != "CAMERA_INTRINSICS_ONLY"
    ):
        raise ValueError("intrinsic calibration must be a CAMERA_INTRINSICS_ONLY draft")
    if (
        payload.get("review_required") is not True
        or payload.get("usable_for_robot_control") is not False
        or payload.get("arm_command_sent") is not False
    ):
        raise ValueError("intrinsic calibration must require review and be unusable for robot control")
    for key in ("calibration_id", "camera_id"):
        if not isinstance(payload.get(key), str) or not payload[key].strip():
            raise ValueError(f"intrinsic calibration {key} is missing")
    pixel_transform_id = payload.get("pixel_transform_id")
    if not isinstance(pixel_transform_id, str) or not re.fullmatch(
        r"[A-Za-z0-9_.-]{1,80}", pixel_transform_id
    ):
        raise ValueError("intrinsic calibration pixel_transform_id is missing or invalid")

    image_size = payload.get("image_size_px")
    if not isinstance(image_size, dict) or any(
        isinstance(image_size.get(key), bool)
        or not isinstance(image_size.get(key), int)
        or image_size[key] <= 0
        for key in ("width", "height")
    ):
        raise ValueError("intrinsic calibration image_size_px is invalid")

    camera_matrix = payload.get("camera_matrix")
    if (
        not isinstance(camera_matrix, list)
        or len(camera_matrix) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in camera_matrix)
        or any(
            _finite_number(value) is None
            for row in camera_matrix
            for value in row
        )
    ):
        raise ValueError("intrinsic calibration camera_matrix must be a finite 3x3 numeric array")
    matrix_values = [[float(value) for value in row] for row in camera_matrix]
    if (
        matrix_values[0][0] <= 0
        or matrix_values[1][1] <= 0
        or not math.isclose(matrix_values[2][0], 0.0, rel_tol=0.0, abs_tol=1e-9)
        or not math.isclose(matrix_values[2][1], 0.0, rel_tol=0.0, abs_tol=1e-9)
        or not math.isclose(matrix_values[2][2], 1.0, rel_tol=0.0, abs_tol=1e-9)
    ):
        raise ValueError("intrinsic calibration camera_matrix has invalid focal or homogeneous row")
    distortion = payload.get("distortion_coefficients")
    if (
        not isinstance(distortion, list)
        or len(distortion) not in {4, 5, 8, 12, 14}
        or any(_finite_number(value) is None for value in distortion)
    ):
        raise ValueError("intrinsic calibration distortion_coefficients are invalid")
    checkerboard = payload.get("checkerboard")
    if not isinstance(checkerboard, dict):
        raise ValueError("intrinsic calibration checkerboard metadata is missing")
    corner_columns = checkerboard.get("inner_corner_columns")
    corner_rows = checkerboard.get("inner_corner_rows")
    square_size = _finite_number(checkerboard.get("square_size"))
    if (
        isinstance(corner_columns, bool)
        or not isinstance(corner_columns, int)
        or corner_columns < 2
        or isinstance(corner_rows, bool)
        or not isinstance(corner_rows, int)
        or corner_rows < 2
        or square_size is None
        or square_size <= 0
        or checkerboard.get("unit") not in {"m", "mm"}
    ):
        raise ValueError("intrinsic calibration checkerboard dimensions/size/unit are invalid")

    view_count = payload.get("calibrated_view_count")
    source_images = payload.get("source_images")
    image_errors = payload.get("per_image_reprojection_errors")
    rms = payload.get("rms_reprojection_error_px")
    rms_value = _finite_number(rms)
    if (
        isinstance(view_count, bool)
        or not isinstance(view_count, int)
        or view_count < 6
        or not isinstance(source_images, list)
        or len(source_images) != view_count
        or any(not isinstance(name, str) or not name.strip() for name in source_images)
        or len(set(source_images)) != len(source_images)
        or not isinstance(image_errors, list)
        or len(image_errors) != view_count
        or rms_value is None
        or rms_value < 0
    ):
        raise ValueError("intrinsic calibration view/source/reprojection metadata is missing or invalid")

    per_view_rms: list[float] = []
    seen_image_hashes: set[str] = set()
    for index, (source_image, image_error) in enumerate(zip(source_images, image_errors)):
        if (
            not isinstance(image_error, dict)
            or image_error.get("file") != source_image
            or not isinstance(image_error.get("sha256"), str)
            or not re.fullmatch(r"[0-9a-f]{64}", image_error["sha256"])
        ):
            raise ValueError(f"intrinsic calibration per-image record {index} mismatches its source image")
        image_hash = image_error["sha256"]
        if image_hash in seen_image_hashes:
            raise ValueError("intrinsic calibration reuses identical source image bytes across views")
        seen_image_hashes.add(image_hash)
        error_px = image_error.get("reprojection_error_px")
        error_value = _finite_number(error_px)
        if error_value is None or error_value < 0:
            raise ValueError(f"intrinsic calibration per-image RMS {index} is invalid")
        per_view_rms.append(error_value)

    scale = max(per_view_rms)
    recomputed_rms = (
        0.0
        if scale == 0
        else scale * math.sqrt(math.fsum((error / scale) ** 2 for error in per_view_rms) / view_count)
    )
    if not math.isclose(rms_value, recomputed_rms, rel_tol=1e-6, abs_tol=1e-9):
        raise ValueError("intrinsic calibration RMS does not match per-image reprojection records")
    return {
        "view_count": view_count,
        "rms_reprojection_error_px": rms_value,
        "checkerboard": {
            "inner_corner_columns": corner_columns,
            "inner_corner_rows": corner_rows,
            "square_size": square_size,
            "unit": checkerboard["unit"],
        },
        "source_images": source_images,
        "per_image_reprojection_errors": image_errors,
    }
