#!/usr/bin/env python3
"""Create a review-only link6-to-TCP transform artifact from operator measurements."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any

from calibration_artifacts import (
    ensure_distinct_output_path,
    validate_link6_tcp_transform_draft,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_DIRECTORY = REPO_ROOT / ".control-station" / "omx-calibrations"
MAX_EVIDENCE_BYTES = 100_000_000


def _parse_xyz(value: str) -> tuple[float, float, float]:
    try:
        parts = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be x,y,z in meters") from exc
    if len(parts) != 3 or not all(math.isfinite(part) for part in parts):
        raise argparse.ArgumentTypeError("value must contain three finite meter values x,y,z")
    return parts


def _parse_unit_quaternion(value: str) -> tuple[float, float, float, float]:
    try:
        parts = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("quaternion must be w,x,y,z") from exc
    if len(parts) != 4 or not all(math.isfinite(part) for part in parts):
        raise argparse.ArgumentTypeError("quaternion must contain four finite values w,x,y,z")
    norm = math.hypot(*parts)
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise argparse.ArgumentTypeError("measured quaternion must already be normalized")
    return parts  # type: ignore[return-value]


def _managed_evidence(path: Path, label: str) -> tuple[str, str, Path]:
    if CALIBRATION_DIRECTORY.is_symlink() or (REPO_ROOT / ".control-station").is_symlink():
        raise ValueError("calibration directory cannot be a symbolic link")
    try:
        base = CALIBRATION_DIRECTORY.resolve(strict=True)
        lexical_path = Path(os.path.abspath(path.expanduser()))
        lexical_relative = lexical_path.relative_to(base)
        current = base
        for component in lexical_relative.parts:
            current = current / component
            if current.is_symlink():
                raise ValueError(f"{label} evidence cannot use symbolic links")
        resolved = path.expanduser().resolve(strict=True)
    except ValueError as exc:
        if str(exc).startswith(f"{label} evidence"):
            raise
        raise ValueError(f"{label} evidence must be under {CALIBRATION_DIRECTORY}") from exc
    except (OSError, RuntimeError) as exc:
        raise ValueError(f"{label} evidence cannot be resolved") from exc
    if base not in resolved.parents or not resolved.is_file():
        raise ValueError(f"{label} evidence must be a file under {base}")
    digest = hashlib.sha256()
    size_bytes = 0
    with resolved.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            size_bytes += len(block)
            if size_bytes > MAX_EVIDENCE_BYTES:
                raise ValueError(f"{label} evidence exceeds 100 MB")
            digest.update(block)
    relative_name = resolved.relative_to(base).as_posix()
    if (
        len(relative_name) > 255
        or not re.fullmatch(r"[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*", relative_name)
        or any(part in {".", ".."} for part in relative_name.split("/"))
    ):
        raise ValueError(f"{label} evidence filename must use safe ASCII path components")
    return relative_name, digest.hexdigest(), resolved


def _timestamp_with_timezone(value: str) -> str:
    normalized = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("measured-at must be an ISO 8601 timestamp with timezone") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise argparse.ArgumentTypeError("measured-at must include a timezone")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_exclusive_json(path: Path, payload: dict[str, Any]) -> None:
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.stem}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            json.dump(payload, temporary_file, ensure_ascii=False, indent=2, allow_nan=False)
            temporary_file.write("\n")
            temporary_path = Path(temporary_file.name)
        os.link(temporary_path, path)
    except FileExistsError as exc:
        raise ValueError("output artifact already exists; refusing to overwrite") from exc
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-id", required=True, help="stable ID for this draft artifact")
    parser.add_argument("--robot-model-name", required=True, help="measured physical robot/gripper variant")
    parser.add_argument("--robot-model-revision", required=True, help="physical model/config revision")
    parser.add_argument(
        "--physical-parent-frame", required=True,
        help="physical frame whose reviewed correspondence to MuJoCo link6 is documented",
    )
    parser.add_argument(
        "--tcp-translation-link6-m", type=_parse_xyz, required=True,
        help="measured TCP translation already expressed in MuJoCo link6 coordinates (x,y,z meters)",
    )
    parser.add_argument(
        "--tcp-quaternion-link6-wxyz", type=_parse_unit_quaternion, required=True,
        help="normalized measured TCP orientation already expressed relative to MuJoCo link6 (w,x,y,z)",
    )
    parser.add_argument(
        "--mapping-evidence", type=Path, required=True,
        help="review evidence file under .control-station/omx-calibrations/",
    )
    parser.add_argument(
        "--measurement-evidence", type=Path, required=True,
        help="TCP measurement source file under .control-station/omx-calibrations/",
    )
    parser.add_argument("--method", required=True, help="measurement method and instrument")
    parser.add_argument(
        "--measured-at", type=_timestamp_with_timezone, required=True,
        help="measurement timestamp in ISO 8601 format with timezone",
    )
    parser.add_argument("--measured-by", required=True, help="operator or record owner")
    parser.add_argument(
        "--translation-uncertainty-m", type=float, required=True,
        help="translation uncertainty in meters",
    )
    parser.add_argument(
        "--orientation-uncertainty-rad", type=float, required=True,
        help="orientation uncertainty in radians",
    )
    parser.add_argument(
        "--frame-mapping-reviewed",
        action="store_true",
        help="record that the physical-parent to MuJoCo link6 evidence was reviewed",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if not args.frame_mapping_reviewed:
        parser.error("--frame-mapping-reviewed is required after operator review of the mapping evidence")
    for option, value in (
        ("--translation-uncertainty-m", args.translation_uncertainty_m),
        ("--orientation-uncertainty-rad", args.orientation_uncertainty_rad),
    ):
        if not math.isfinite(value) or value < 0:
            parser.error(f"{option} must be finite and nonnegative")
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,80}", args.calibration_id):
        parser.error("--calibration-id must use 1 to 80 ASCII letters, digits, dot, underscore, or dash")
    for option, value in (
        ("--robot-model-name", args.robot_model_name),
        ("--robot-model-revision", args.robot_model_revision),
        ("--physical-parent-frame", args.physical_parent_frame),
        ("--method", args.method),
        ("--measured-by", args.measured_by),
    ):
        if not value.strip():
            parser.error(f"{option} must not be empty")

    if CALIBRATION_DIRECTORY.is_symlink() or (REPO_ROOT / ".control-station").is_symlink():
        parser.error("calibration output directory cannot be a symbolic link")
    CALIBRATION_DIRECTORY.mkdir(parents=True, exist_ok=True)
    output_path = Path(os.path.abspath(args.output.expanduser()))
    if output_path.parent != CALIBRATION_DIRECTORY or output_path.is_symlink():
        parser.error("output must be a direct file path in the managed calibration directory")
    try:
        resolved_directory = CALIBRATION_DIRECTORY.resolve(strict=True)
        resolved_output = output_path.resolve(strict=False)
    except (OSError, RuntimeError) as exc:
        parser.error(f"output path cannot be resolved: {exc}")
    if (
        resolved_directory != CALIBRATION_DIRECTORY
        or resolved_output.parent != resolved_directory
        or resolved_output.suffix.lower() != ".json"
    ):
        parser.error("--output must be a .json file directly under .control-station/omx-calibrations")

    try:
        mapping_reference, mapping_sha256, mapping_path = _managed_evidence(
            args.mapping_evidence, "frame-mapping"
        )
        measurement_reference, measurement_sha256, measurement_path = _managed_evidence(
            args.measurement_evidence, "measurement"
        )
        ensure_distinct_output_path(resolved_output, (mapping_path, measurement_path))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    payload = {
        "schema_version": 1,
        "kind": "omx_link6_tcp_transform",
        "status": "DRAFT_ONLY",
        "calibration_scope": "LINK6_TO_TCP",
        "calibration_id": args.calibration_id,
        "review_required": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "robot_model": {
            "name": args.robot_model_name,
            "revision": args.robot_model_revision,
        },
        "frame_mapping": {
            "physical_parent_frame": args.physical_parent_frame,
            "simulator_frame": "link6",
            "review_status": "REVIEWED",
            "evidence_reference": mapping_reference,
            "evidence_sha256": mapping_sha256,
        },
        "tcp_transform": {
            "translation_link6_m": list(args.tcp_translation_link6_m),
            "orientation_link6_wxyz": list(args.tcp_quaternion_link6_wxyz),
        },
        "measurement": {
            "method": args.method,
            "measured_at": args.measured_at,
            "measured_by": args.measured_by,
            "source_reference": measurement_reference,
            "source_sha256": measurement_sha256,
            "translation_uncertainty_m": args.translation_uncertainty_m,
            "orientation_uncertainty_rad": args.orientation_uncertainty_rad,
        },
    }
    try:
        validate_link6_tcp_transform_draft(payload)
        _write_exclusive_json(resolved_output, payload)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    print(
        json.dumps(
            {
                "calibration_id": args.calibration_id,
                "status": "DRAFT_ONLY",
                "artifact": resolved_output.relative_to(REPO_ROOT).as_posix(),
                "artifact_sha256": _sha256(resolved_output),
                "usable_for_robot_control": False,
                "arm_command_sent": False,
            },
            ensure_ascii=False,
            allow_nan=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
