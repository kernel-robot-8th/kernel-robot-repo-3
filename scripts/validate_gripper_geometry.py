#!/usr/bin/env python3
"""Validate measured OMX finger geometry and its TCP/source provenance."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import sys
from typing import Any

from calibration_artifacts import load_link6_tcp_transform_artifact, unique_json_object


REPO_ROOT = Path(__file__).resolve().parents[1]
CONTROL_DIRECTORY = REPO_ROOT / ".control-station"
CALIBRATION_DIRECTORY = CONTROL_DIRECTORY / "omx-calibrations"
MAX_ARTIFACT_BYTES = 1_000_000
MAX_EVIDENCE_BYTES = 100_000_000
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
SAFE_ID_PATTERN = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
SAFE_RELATIVE_PATH_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*$")


class GeometryArtifactError(ValueError):
    """Raised when a measurement artifact or referenced source is invalid."""


def _finite_number(value: object, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GeometryArtifactError(f"{label} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise GeometryArtifactError(f"{label} must be a finite number") from exc
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        raise GeometryArtifactError(f"{label} is outside its allowed numeric range")
    return number


def _vector3(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise GeometryArtifactError(f"{label} must contain three finite coordinates")
    return tuple(_finite_number(component, label) for component in value)  # type: ignore[return-value]


def _validate_contact_polygon(
    value: object, side: str
) -> list[tuple[float, float, float]]:
    if not isinstance(value, list) or len(value) < 3 or len(value) > 64:
        raise GeometryArtifactError(
            f"{side} contact patch must contain 3 to 64 measured points"
        )
    points = [
        _vector3(point, f"{side} contact patch vertex")
        for point in value
    ]
    if len(set(points)) < 3:
        raise GeometryArtifactError(
            f"{side} contact patch needs at least 3 distinct points"
        )

    origin = points[0]
    edges = [
        (point[0] - origin[0], point[1] - origin[1], point[2] - origin[2])
        for point in points[1:]
    ]
    largest_cross_norm = 0.0
    for index, first in enumerate(edges):
        for second in edges[index + 1 :]:
            cross = (
                first[1] * second[2] - first[2] * second[1],
                first[2] * second[0] - first[0] * second[2],
                first[0] * second[1] - first[1] * second[0],
            )
            if not all(math.isfinite(component) for component in cross):
                raise GeometryArtifactError(f"{side} contact patch area overflows numeric range")
            largest_cross_norm = max(largest_cross_norm, math.hypot(*cross))
    if largest_cross_norm <= 1e-12:
        raise GeometryArtifactError(
            f"{side} contact patch vertices are collinear or degenerate"
        )
    doubled_area = [0.0, 0.0, 0.0]
    for current, following in zip(points, points[1:] + points[:1]):
        cross = (
            current[1] * following[2] - current[2] * following[1],
            current[2] * following[0] - current[0] * following[2],
            current[0] * following[1] - current[1] * following[0],
        )
        if not all(math.isfinite(component) for component in cross):
            raise GeometryArtifactError(f"{side} contact polygon area overflows numeric range")
        for axis in range(3):
            doubled_area[axis] += cross[axis]
    if not all(math.isfinite(component) for component in doubled_area):
        raise GeometryArtifactError(f"{side} contact polygon area overflows numeric range")
    if math.hypot(*doubled_area) <= 2e-12:
        raise GeometryArtifactError(
            f"{side} contact patch polygon boundary has zero or degenerate area"
        )
    return points


def _safe_managed_file(
    relative_name: object,
    label: str,
    *,
    maximum_bytes: int = MAX_EVIDENCE_BYTES,
) -> tuple[Path, str, int]:
    if (
        not isinstance(relative_name, str)
        or len(relative_name) > 255
        or not SAFE_RELATIVE_PATH_PATTERN.fullmatch(relative_name)
        or any(part in {".", ".."} for part in relative_name.split("/"))
    ):
        raise GeometryArtifactError(f"{label} path must use safe relative ASCII components")
    if CONTROL_DIRECTORY.is_symlink() or CALIBRATION_DIRECTORY.is_symlink():
        raise GeometryArtifactError("calibration directories cannot be symbolic links")
    base = CALIBRATION_DIRECTORY.resolve(strict=True)
    lexical = base.joinpath(*PurePosixPath(relative_name).parts)
    current = base
    for component in PurePosixPath(relative_name).parts:
        current = current / component
        if current.is_symlink():
            raise GeometryArtifactError(f"{label} cannot use symbolic-link path components")
    resolved = lexical.resolve(strict=True)
    if base not in resolved.parents or not resolved.is_file():
        raise GeometryArtifactError(f"{label} must be a regular file under {base}")
    size_bytes = resolved.stat().st_size
    if size_bytes > maximum_bytes:
        raise GeometryArtifactError(f"{label} exceeds its {maximum_bytes}-byte size limit")
    digest = hashlib.sha256()
    read_bytes = 0
    with resolved.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            read_bytes += len(block)
            if read_bytes > maximum_bytes:
                raise GeometryArtifactError(
                    f"{label} exceeds its {maximum_bytes}-byte size limit"
                )
            digest.update(block)
    return resolved, digest.hexdigest(), size_bytes


def _timestamp_with_timezone(value: object) -> None:
    if not isinstance(value, str) or not value:
        raise GeometryArtifactError("measurement.measured_at must be an ISO 8601 timestamp")
    normalized = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise GeometryArtifactError("measurement.measured_at is invalid") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise GeometryArtifactError("measurement.measured_at must include a timezone")


def _validate(
    payload: object, artifact_path: Path, artifact_sha256: str
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise GeometryArtifactError("geometry artifact must contain a JSON object")
    expected = {
        "schema_version", "kind", "status", "review_required",
        "usable_for_robot_control", "arm_command_sent", "geometry_id",
        "robot_model", "tcp", "tool_geometry", "measurement",
    }
    if set(payload) != expected:
        raise GeometryArtifactError("geometry artifact has missing or unexpected fields")
    if (
        isinstance(payload["schema_version"], bool)
        or not isinstance(payload["schema_version"], int)
        or payload["schema_version"] != 1
        or payload["kind"] != "omx_gripper_geometry_measurement"
        or payload["status"] != "DRAFT_ONLY"
        or payload["review_required"] is not True
        or payload["usable_for_robot_control"] is not False
        or payload["arm_command_sent"] is not False
    ):
        raise GeometryArtifactError("artifact must be a version 1 review-only measurement")
    for key in ("geometry_id",):
        if not isinstance(payload[key], str) or not SAFE_ID_PATTERN.fullmatch(payload[key]):
            raise GeometryArtifactError(f"{key} is missing or invalid")

    robot = payload["robot_model"]
    if not isinstance(robot, dict) or set(robot) != {"name", "revision"}:
        raise GeometryArtifactError("robot_model must contain name and revision")
    if any(not isinstance(robot[key], str) or not robot[key].strip() for key in robot):
        raise GeometryArtifactError("robot_model name and revision must be non-empty")

    tcp = payload["tcp"]
    if not isinstance(tcp, dict) or set(tcp) != {"frame_id", "calibration_id", "artifact"}:
        raise GeometryArtifactError("tcp must reference a frame and measured TCP artifact")
    for key in ("frame_id", "calibration_id"):
        if not isinstance(tcp[key], str) or not tcp[key].strip():
            raise GeometryArtifactError(f"tcp.{key} must be non-empty")
    if not SAFE_ID_PATTERN.fullmatch(tcp["calibration_id"]):
        raise GeometryArtifactError("tcp.calibration_id is invalid")
    tcp_artifact = tcp["artifact"]
    if (
        not isinstance(tcp_artifact, dict)
        or set(tcp_artifact) != {"path", "sha256"}
        or not isinstance(tcp_artifact["path"], str)
        or not isinstance(tcp_artifact["sha256"], str)
        or not SHA256_PATTERN.fullmatch(tcp_artifact["sha256"])
    ):
        raise GeometryArtifactError("tcp.artifact path/hash reference is invalid")
    tcp_path, tcp_sha256, tcp_size = _safe_managed_file(
        tcp_artifact["path"],
        "TCP calibration artifact",
        maximum_bytes=MAX_ARTIFACT_BYTES,
    )
    if tcp_sha256 != tcp_artifact["sha256"]:
        raise GeometryArtifactError("TCP calibration artifact SHA-256 mismatch")
    loaded_tcp = load_link6_tcp_transform_artifact(tcp_path, REPO_ROOT)
    if loaded_tcp["artifact"]["sha256"] != tcp_artifact["sha256"]:
        raise GeometryArtifactError("TCP calibration artifact SHA-256 mismatch")
    calibration = loaded_tcp["validated"]
    if calibration["calibration_id"] != tcp["calibration_id"]:
        raise GeometryArtifactError("TCP calibration ID does not match the referenced artifact")
    if calibration["robot_model"] != robot:
        raise GeometryArtifactError("robot model does not match the TCP calibration artifact")

    geometry = payload["tool_geometry"]
    geometry_keys = {
        "coordinate_frame_id", "closing_axis_tcp_unit", "approach_axis_tcp_unit",
        "measured_gap_range_m", "finger_contact_patches_tcp_m",
    }
    if not isinstance(geometry, dict) or set(geometry) != geometry_keys:
        raise GeometryArtifactError("tool_geometry fields do not match the supported contract")
    if geometry["coordinate_frame_id"] != tcp["frame_id"]:
        raise GeometryArtifactError("tool geometry coordinate frame does not match tcp.frame_id")
    closing = _vector3(geometry["closing_axis_tcp_unit"], "closing_axis_tcp_unit")
    approach = _vector3(geometry["approach_axis_tcp_unit"], "approach_axis_tcp_unit")
    for label, axis in (("closing_axis_tcp_unit", closing), ("approach_axis_tcp_unit", approach)):
        if not math.isclose(math.hypot(*axis), 1.0, rel_tol=0.0, abs_tol=1e-5):
            raise GeometryArtifactError(f"{label} must be a measured unit vector")
    if not math.isclose(sum(a * b for a, b in zip(closing, approach)), 0.0, rel_tol=0.0, abs_tol=1e-5):
        raise GeometryArtifactError("closing and approach axes must be orthogonal")

    gap = geometry["measured_gap_range_m"]
    if not isinstance(gap, dict) or set(gap) != {"minimum_m", "maximum_m"}:
        raise GeometryArtifactError("measured_gap_range_m must contain minimum_m and maximum_m")
    minimum_gap = _finite_number(gap["minimum_m"], "minimum_m", minimum=0.0)
    maximum_gap = _finite_number(gap["maximum_m"], "maximum_m", minimum=0.0)
    if maximum_gap < minimum_gap:
        raise GeometryArtifactError("maximum measured gap cannot be smaller than minimum gap")

    patches = geometry["finger_contact_patches_tcp_m"]
    if not isinstance(patches, dict) or set(patches) != {"left", "right"}:
        raise GeometryArtifactError("finger_contact_patches_tcp_m must contain left and right polygons")
    patch_points: dict[str, list[tuple[float, float, float]]] = {}
    for side in ("left", "right"):
        patch_points[side] = _validate_contact_polygon(patches[side], side)

    measurement = payload["measurement"]
    measurement_keys = {
        "method", "measured_at", "measured_by", "evidence_path", "evidence_sha256",
        "axis_uncertainty_rad", "gap_uncertainty_m", "contact_point_uncertainty_m",
    }
    if not isinstance(measurement, dict) or set(measurement) != measurement_keys:
        raise GeometryArtifactError("measurement fields do not match the supported contract")
    for key in ("method", "measured_by"):
        if not isinstance(measurement[key], str) or not measurement[key].strip():
            raise GeometryArtifactError(f"measurement.{key} must be non-empty")
    _timestamp_with_timezone(measurement["measured_at"])
    if (
        not isinstance(measurement["evidence_sha256"], str)
        or not SHA256_PATTERN.fullmatch(measurement["evidence_sha256"])
    ):
        raise GeometryArtifactError("measurement.evidence_sha256 is invalid")
    evidence_path, evidence_sha256, evidence_size = _safe_managed_file(
        measurement["evidence_path"], "gripper geometry evidence"
    )
    if evidence_sha256 != measurement["evidence_sha256"]:
        raise GeometryArtifactError("gripper geometry evidence SHA-256 mismatch")
    uncertainties = {
        key: _finite_number(measurement[key], key, minimum=0.0)
        for key in (
            "axis_uncertainty_rad", "gap_uncertainty_m", "contact_point_uncertainty_m"
        )
    }

    artifact = artifact_path.resolve(strict=True)
    return {
        "validation_status": "MEASUREMENT_CONTRACT_AND_SOURCE_HASHES_MATCH",
        "geometry_id": payload["geometry_id"],
        "status": "DRAFT_ONLY",
        "robot_model": robot,
        "tcp": {
            **tcp,
            "verified_artifact": loaded_tcp["artifact"],
            "physical_accuracy_verified": False,
        },
        "tool_geometry": {
            "coordinate_frame_id": geometry["coordinate_frame_id"],
            "closing_axis_tcp_unit": list(closing),
            "approach_axis_tcp_unit": list(approach),
            "measured_gap_range_m": {
                "minimum_m": minimum_gap,
                "maximum_m": maximum_gap,
            },
            "finger_contact_patches_tcp_m": {
                side: [list(point) for point in points]
                for side, points in patch_points.items()
            },
        },
        "measurement": {
            **measurement,
            **uncertainties,
            "verified_evidence": {
                "path": evidence_path.relative_to(REPO_ROOT).as_posix(),
                "sha256": evidence_sha256,
                "size_bytes": evidence_size,
            },
        },
        "artifact": {
            "path": artifact.relative_to(REPO_ROOT).as_posix(),
            "sha256": artifact_sha256,
        },
        "review_required": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "physical_accuracy_verified": False,
        "grasp_verified": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    args = parser.parse_args()
    artifact_path = args.artifact
    if not artifact_path.is_absolute():
        artifact_path = (Path.cwd() / artifact_path).absolute()
    if (
        CONTROL_DIRECTORY.is_symlink()
        or CALIBRATION_DIRECTORY.is_symlink()
        or artifact_path.parent != CALIBRATION_DIRECTORY
        or artifact_path.is_symlink()
        or artifact_path.suffix.lower() != ".json"
    ):
        print(
            "gripper geometry artifact must be a direct non-symlink JSON "
            "under the managed calibration directory",
            file=sys.stderr,
        )
        return 2
    try:
        resolved_directory = CALIBRATION_DIRECTORY.resolve(strict=True)
        resolved_artifact = artifact_path.resolve(strict=True)
        if resolved_artifact.parent != resolved_directory or not resolved_artifact.is_file():
            raise GeometryArtifactError("artifact path is invalid")
        with resolved_artifact.open("rb") as source:
            raw = source.read(MAX_ARTIFACT_BYTES + 1)
        if len(raw) > MAX_ARTIFACT_BYTES:
            raise GeometryArtifactError("artifact exceeds 1 MB")
        payload = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_json_object)
        result = _validate(
            payload,
            resolved_artifact,
            hashlib.sha256(raw).hexdigest(),
        )
    except (OSError, RuntimeError, UnicodeError, ValueError, RecursionError) as exc:
        print(f"gripper geometry artifact rejected: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
