"""Shared validation for offline, review-only camera calibration JSON artifacts."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from datetime import datetime
from pathlib import PurePosixPath
from pathlib import Path
from typing import Any, Iterable


MAX_TCP_CALIBRATION_BYTES = 1_000_000
MAX_TCP_EVIDENCE_BYTES = 100_000_000
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


def event_step_link_matches(
    task: dict[str, Any], event: dict[str, Any], expected_step_type: str
) -> bool:
    """Validate a present workflow-step link; accept legacy events without one."""
    event_step_id = event.get("step_id")
    if event_step_id is None:
        return True
    steps = task.get("steps")
    if not isinstance(steps, list):
        return False
    matches = [
        step
        for step in steps
        if isinstance(step, dict) and step.get("step_type") == expected_step_type
    ]
    return (
        len(matches) == 1
        and matches[0].get("id") == event_step_id
        and matches[0].get("status") == "PENDING"
    )


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


def validate_plane_source_provenance(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate versioned correspondence-source identity stored in a plane draft."""
    schema_version = payload.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 4
    ):
        raise ValueError("unsupported plane calibration schema_version")
    source_schema_version = payload.get("source_correspondence_schema_version")
    if (
        isinstance(source_schema_version, bool)
        or not isinstance(source_schema_version, int)
        or source_schema_version != 1
    ):
        raise ValueError("unsupported correspondence source schema version")
    source_name = payload.get("source_correspondence_file")
    if (
        not isinstance(source_name, str)
        or not source_name.strip()
        or len(source_name) > 255
        or "/" in source_name
        or "\\" in source_name
    ):
        raise ValueError("plane calibration correspondence source filename is invalid")
    source_sha256 = payload.get("source_correspondence_sha256")
    if not isinstance(source_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", source_sha256):
        raise ValueError("plane calibration correspondence source SHA-256 is invalid")
    return {
        "source_correspondence_schema_version": source_schema_version,
        "source_correspondence_file": source_name,
        "source_correspondence_sha256": source_sha256,
    }


def validate_plane_sample_sources(payload: dict[str, Any]) -> list[dict[str, str]]:
    """Validate relative source-image paths and their recorded file hashes."""
    samples = payload.get("samples")
    if not isinstance(samples, list):
        raise ValueError("plane calibration samples are missing")
    source_hashes: dict[str, str] = {}
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"plane calibration samples[{index}] must be an object")
        source_image = sample.get("source_image")
        source_hash = sample.get("source_image_sha256")
        if (
            not isinstance(source_image, str)
            or not source_image.strip()
            or len(source_image) > 255
            or source_image.startswith("/")
            or "\\" in source_image
            or ".." in source_image.split("/")
        ):
            raise ValueError(f"plane calibration samples[{index}].source_image is invalid")
        path = PurePosixPath(source_image)
        if path.is_absolute() or not path.parts or ":" in path.parts[0]:
            raise ValueError(f"plane calibration samples[{index}].source_image must be relative")
        if not isinstance(source_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", source_hash):
            raise ValueError(f"plane calibration samples[{index}].source_image_sha256 is invalid")
        normalized_path = path.as_posix()
        previous_hash = source_hashes.setdefault(normalized_path, source_hash)
        if previous_hash != source_hash:
            raise ValueError("plane calibration assigns different hashes to the same source image")
    return [
        {"source_image": name, "sha256": digest}
        for name, digest in source_hashes.items()
    ]


def validate_link6_tcp_transform_draft(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate a reviewed-frame, draft-only transform artifact for offline IK."""
    def require_keys(value: dict[str, Any], expected: set[str], label: str) -> None:
        if set(value) != expected:
            raise ValueError(f"TCP transform {label} has missing or unexpected fields")

    require_keys(
        payload,
        {
            "schema_version",
            "kind",
            "status",
            "calibration_scope",
            "calibration_id",
            "review_required",
            "usable_for_robot_control",
            "arm_command_sent",
            "robot_model",
            "frame_mapping",
            "tcp_transform",
            "measurement",
        },
        "artifact",
    )
    schema_version = payload.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
    ):
        raise ValueError("unsupported TCP transform schema_version")
    if (
        payload.get("kind") != "omx_link6_tcp_transform"
        or payload.get("status") != "DRAFT_ONLY"
        or payload.get("calibration_scope") != "LINK6_TO_TCP"
    ):
        raise ValueError("artifact must be a DRAFT_ONLY LINK6_TO_TCP transform")
    if (
        payload.get("review_required") is not True
        or payload.get("usable_for_robot_control") is not False
        or payload.get("arm_command_sent") is not False
    ):
        raise ValueError("TCP transform draft must require review and be unusable for robot control")

    calibration_id = payload.get("calibration_id")
    if not isinstance(calibration_id, str) or not re.fullmatch(
        r"[A-Za-z0-9_.-]{1,80}", calibration_id
    ):
        raise ValueError("TCP transform calibration_id is missing or invalid")

    robot_model = payload.get("robot_model")
    if not isinstance(robot_model, dict):
        raise ValueError("TCP transform robot_model name/revision is missing")
    require_keys(robot_model, {"name", "revision"}, "robot_model")
    if any(
        not isinstance(robot_model.get(key), str) or not robot_model[key].strip()
        for key in ("name", "revision")
    ):
        raise ValueError("TCP transform robot_model name/revision is missing")

    frame_mapping = payload.get("frame_mapping")
    if isinstance(frame_mapping, dict):
        require_keys(
            frame_mapping,
            {
                "physical_parent_frame",
                "simulator_frame",
                "review_status",
                "evidence_reference",
                "evidence_sha256",
            },
            "frame_mapping",
        )
    if (
        not isinstance(frame_mapping, dict)
        or not isinstance(frame_mapping.get("physical_parent_frame"), str)
        or not frame_mapping["physical_parent_frame"].strip()
        or frame_mapping.get("simulator_frame") != "link6"
        or frame_mapping.get("review_status") != "REVIEWED"
    ):
        raise ValueError("TCP transform physical-parent to MuJoCo link6 mapping is not reviewed")
    mapping_reference = frame_mapping.get("evidence_reference")
    mapping_sha256 = frame_mapping.get("evidence_sha256")
    if (
        not isinstance(mapping_reference, str)
        or len(mapping_reference) > 255
        or not re.fullmatch(r"[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*", mapping_reference)
        or any(part in {".", ".."} for part in mapping_reference.split("/"))
        or not isinstance(mapping_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", mapping_sha256)
    ):
        raise ValueError("TCP transform frame-mapping evidence reference/hash is invalid")

    transform = payload.get("tcp_transform")
    if not isinstance(transform, dict):
        raise ValueError("TCP transform values are missing")
    require_keys(transform, {"translation_link6_m", "orientation_link6_wxyz"}, "tcp_transform")
    translation = transform.get("translation_link6_m")
    quaternion = transform.get("orientation_link6_wxyz")
    if (
        not isinstance(translation, list)
        or len(translation) != 3
        or any(_finite_number(value) is None for value in translation)
    ):
        raise ValueError("TCP translation_link6_m must contain three finite numbers")
    if (
        not isinstance(quaternion, list)
        or len(quaternion) != 4
        or any(_finite_number(value) is None for value in quaternion)
    ):
        raise ValueError("TCP orientation_link6_wxyz must contain four finite numbers")
    normalized_translation = [float(value) for value in translation]
    normalized_quaternion = [float(value) for value in quaternion]
    quaternion_norm = math.hypot(*normalized_quaternion)
    if not math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("TCP transform quaternion must be normalized")

    measurement = payload.get("measurement")
    if not isinstance(measurement, dict):
        raise ValueError("TCP transform measurement record is missing")
    require_keys(
        measurement,
        {
            "method",
            "measured_at",
            "measured_by",
            "source_reference",
            "source_sha256",
            "translation_uncertainty_m",
            "orientation_uncertainty_rad",
        },
        "measurement",
    )
    for key in ("method", "measured_at", "measured_by", "source_reference"):
        if not isinstance(measurement.get(key), str) or not measurement[key].strip():
            raise ValueError(f"TCP transform measurement.{key} is missing")
    measured_at = measurement["measured_at"]
    normalized_measured_at = (
        measured_at[:-1] + "+00:00" if measured_at.endswith("Z") else measured_at
    )
    try:
        measured_at_value = datetime.fromisoformat(normalized_measured_at)
    except ValueError as exc:
        raise ValueError("TCP transform measurement.measured_at must be ISO 8601") from exc
    if measured_at_value.tzinfo is None or measured_at_value.utcoffset() is None:
        raise ValueError("TCP transform measurement.measured_at must include a timezone")
    source_sha256 = measurement.get("source_sha256")
    source_reference = measurement.get("source_reference")
    if (
        not isinstance(source_reference, str)
        or len(source_reference) > 255
        or not re.fullmatch(r"[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*", source_reference)
        or any(part in {".", ".."} for part in source_reference.split("/"))
        or not isinstance(source_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", source_sha256)
    ):
        raise ValueError("TCP transform measurement source reference/hash is invalid")
    translation_uncertainty = _finite_number(measurement.get("translation_uncertainty_m"))
    orientation_uncertainty = _finite_number(measurement.get("orientation_uncertainty_rad"))
    if (
        translation_uncertainty is None
        or translation_uncertainty < 0
        or orientation_uncertainty is None
        or orientation_uncertainty < 0
    ):
        raise ValueError("TCP transform measurement uncertainty values must be finite and nonnegative")

    return {
        "calibration_id": calibration_id,
        "robot_model": {"name": robot_model["name"], "revision": robot_model["revision"]},
        "frame_mapping": {
            "physical_parent_frame": frame_mapping["physical_parent_frame"],
            "simulator_frame": "link6",
            "review_status": "REVIEWED",
            "evidence_reference": mapping_reference,
            "evidence_sha256": mapping_sha256,
        },
        "tcp_transform": {
            "translation_link6_m": normalized_translation,
            "orientation_link6_wxyz": normalized_quaternion,
        },
        "measurement": {
            "method": measurement["method"],
            "measured_at": measurement["measured_at"],
            "measured_by": measurement["measured_by"],
            "source_reference": source_reference,
            "source_sha256": source_sha256,
            "translation_uncertainty_m": translation_uncertainty,
            "orientation_uncertainty_rad": orientation_uncertainty,
        },
    }


def load_link6_tcp_transform_artifact(
    path: Path, repository_root: Path
) -> dict[str, Any]:
    """Load a managed draft artifact and verify its referenced evidence file hashes."""
    root = repository_root.resolve(strict=True)
    control_directory = root / ".control-station"
    managed_directory = control_directory / "omx-calibrations"
    lexical_path = Path(os.path.abspath(path.expanduser()))
    if (
        control_directory.is_symlink()
        or managed_directory.is_symlink()
        or lexical_path.parent != managed_directory
    ):
        raise ValueError(
            "TCP transform artifact must be a direct file in the managed directory, "
            "without symbolic-link path components"
        )
    if lexical_path.is_symlink():
        raise ValueError("TCP transform artifact cannot be a symbolic link")
    try:
        resolved_directory = managed_directory.resolve(strict=True)
        resolved_path = lexical_path.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise ValueError("TCP transform artifact path cannot be resolved") from exc
    if (
        resolved_directory != managed_directory
        or resolved_path.parent != resolved_directory
        or resolved_path.suffix.lower() != ".json"
        or not resolved_path.is_file()
        or resolved_path.stat().st_size > MAX_TCP_CALIBRATION_BYTES
    ):
        raise ValueError(
            "TCP transform artifact must be a JSON file directly under "
            "REPOSITORY_ROOT/.control-station/omx-calibrations"
        )
    raw_artifact = resolved_path.read_bytes()
    if len(raw_artifact) > MAX_TCP_CALIBRATION_BYTES:
        raise ValueError("TCP transform artifact exceeds 1 MB")
    try:
        payload = json.loads(raw_artifact, object_pairs_hook=unique_json_object)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise ValueError("TCP transform artifact must contain valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("TCP transform artifact must contain a JSON object")
    validated = validate_link6_tcp_transform_draft(payload)

    verified_evidence: dict[str, dict[str, str]] = {}
    for key, reference_key, hash_key in (
        ("frame_mapping", "evidence_reference", "evidence_sha256"),
        ("measurement", "source_reference", "source_sha256"),
    ):
        record = validated[key]
        relative_name = record[reference_key]
        evidence_path = resolved_directory.joinpath(*PurePosixPath(relative_name).parts)
        current = resolved_directory
        for component in PurePosixPath(relative_name).parts:
            current = current / component
            if current.is_symlink():
                raise ValueError(f"TCP calibration evidence cannot be a symbolic link: {relative_name}")
        try:
            resolved_evidence = evidence_path.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise ValueError(f"TCP calibration evidence is missing: {relative_name}") from exc
        if (
            resolved_directory not in resolved_evidence.parents
            or not resolved_evidence.is_file()
            or resolved_evidence.stat().st_size > MAX_TCP_EVIDENCE_BYTES
        ):
            raise ValueError(f"TCP calibration evidence path is invalid: {relative_name}")
        digest = hashlib.sha256()
        evidence_size = 0
        with resolved_evidence.open("rb") as evidence_file:
            for block in iter(lambda: evidence_file.read(1024 * 1024), b""):
                evidence_size += len(block)
                if evidence_size > MAX_TCP_EVIDENCE_BYTES:
                    raise ValueError(f"TCP calibration evidence exceeds 100 MB: {relative_name}")
                digest.update(block)
        actual_sha256 = digest.hexdigest()
        if actual_sha256 != record[hash_key]:
            raise ValueError(f"TCP calibration evidence SHA-256 mismatch: {relative_name}")
        verified_evidence[key] = {
            "path": resolved_evidence.relative_to(root).as_posix(),
            "sha256": actual_sha256,
        }

    return {
        "validated": validated,
        "artifact": {
            "path": resolved_path.relative_to(root).as_posix(),
            "sha256": hashlib.sha256(raw_artifact).hexdigest(),
        },
        "verified_evidence": verified_evidence,
    }


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
