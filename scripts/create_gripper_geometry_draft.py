#!/usr/bin/env python3
"""Create a review-only gripper geometry artifact from operator measurements."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any

from calibration_artifacts import load_link6_tcp_transform_artifact, unique_json_object
from validate_gripper_geometry import (
    CALIBRATION_DIRECTORY,
    CONTROL_DIRECTORY,
    MAX_ARTIFACT_BYTES,
    _safe_managed_file,
    _validate,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def _write_exclusive_json(path: Path, payload: dict[str, Any]) -> bytes:
    raw = (json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode(
        "utf-8"
    )
    if len(raw) > MAX_ARTIFACT_BYTES:
        raise ValueError("generated artifact exceeds 1 MB")
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=path.parent,
            prefix=f".{path.stem}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_file.write(raw)
            temporary_path = Path(temporary_file.name)
        os.link(temporary_path, path)
    except FileExistsError as exc:
        raise ValueError("output artifact already exists; refusing to overwrite") from exc
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return raw


def _load_measurement_record(path: Path) -> tuple[dict[str, Any], str, str]:
    if CONTROL_DIRECTORY.is_symlink() or CALIBRATION_DIRECTORY.is_symlink():
        raise ValueError("calibration directories cannot be symbolic links")
    lexical_path = Path(os.path.abspath(path.expanduser()))
    try:
        relative_path = lexical_path.relative_to(CALIBRATION_DIRECTORY)
    except ValueError as exc:
        raise ValueError(
            f"measurement record must be under {CALIBRATION_DIRECTORY}"
        ) from exc
    relative_name = relative_path.as_posix()
    resolved, digest, size = _safe_managed_file(
        relative_name, "gripper measurement record", maximum_bytes=MAX_ARTIFACT_BYTES
    )
    if resolved.suffix.lower() != ".json":
        raise ValueError("measurement record must be a JSON file")
    with resolved.open("rb") as source:
        raw = source.read(MAX_ARTIFACT_BYTES + 1)
    if len(raw) > MAX_ARTIFACT_BYTES:
        raise ValueError("measurement record exceeds 1 MB")
    if len(raw) != size or hashlib.sha256(raw).hexdigest() != digest:
        raise ValueError("measurement record changed while it was being read")
    try:
        payload = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_json_object)
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ValueError("measurement record must contain valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("measurement record must contain a JSON object")
    return payload, relative_name, digest


def _validate_measurement_record(payload: dict[str, Any]) -> dict[str, Any]:
    expected = {
        "schema_version",
        "kind",
        "geometry_id",
        "robot_model",
        "tcp",
        "tool_geometry",
        "measurement",
    }
    if set(payload) != expected:
        raise ValueError("measurement record has missing or unexpected fields")
    if (
        isinstance(payload["schema_version"], bool)
        or not isinstance(payload["schema_version"], int)
        or payload["schema_version"] != 1
        or payload["kind"] != "omx_gripper_geometry_measurement_input"
    ):
        raise ValueError("unsupported gripper measurement record version or kind")
    measurement = payload["measurement"]
    if not isinstance(measurement, dict) or set(measurement) != {
        "method",
        "measured_at",
        "measured_by",
        "axis_uncertainty_rad",
        "gap_uncertainty_m",
        "contact_point_uncertainty_m",
    }:
        raise ValueError("measurement record measurement fields do not match the input contract")
    tcp = payload["tcp"]
    if not isinstance(tcp, dict) or set(tcp) != {
        "frame_id", "calibration_id", "artifact_path"
    }:
        raise ValueError("measurement record tcp must include frame, ID and artifact path")
    if not isinstance(tcp["artifact_path"], str):
        raise ValueError("tcp.artifact_path must be a managed relative path")
    if "/" in tcp["artifact_path"]:
        raise ValueError("TCP calibration artifact must be directly under the managed directory")
    tcp_path, tcp_digest, tcp_size = _safe_managed_file(
        tcp["artifact_path"], "TCP calibration artifact", maximum_bytes=MAX_ARTIFACT_BYTES
    )
    if tcp_path.suffix.lower() != ".json" or tcp_size > MAX_ARTIFACT_BYTES:
        raise ValueError("TCP calibration artifact must be a JSON file under 1 MB")
    loaded_tcp = load_link6_tcp_transform_artifact(tcp_path, REPO_ROOT)
    tcp_validated = loaded_tcp["validated"]
    robot = payload["robot_model"]
    if tcp_validated["robot_model"] != robot:
        raise ValueError("robot model/revision does not match the TCP calibration")
    if tcp_validated["calibration_id"] != tcp["calibration_id"]:
        raise ValueError("TCP calibration ID does not match the referenced artifact")
    return {
        "frame_id": tcp["frame_id"],
        "calibration_id": tcp["calibration_id"],
        "artifact": {
            "path": tcp["artifact_path"],
            "sha256": tcp_digest,
        },
    }


def _ensure_output_path(output: Path, inputs: list[Path]) -> Path:
    if CONTROL_DIRECTORY.is_symlink() or CALIBRATION_DIRECTORY.is_symlink():
        raise ValueError("calibration directories cannot be symbolic links")
    if not CALIBRATION_DIRECTORY.is_dir():
        raise ValueError("managed calibration directory must exist before creating a draft")
    resolved_directory = CALIBRATION_DIRECTORY.resolve(strict=True)
    lexical = Path(os.path.abspath(output.expanduser()))
    if lexical.parent != CALIBRATION_DIRECTORY or lexical.is_symlink():
        raise ValueError("output must be a direct non-symlink file in the managed directory")
    resolved_output = lexical.resolve(strict=False)
    if resolved_output.parent != resolved_directory or resolved_output.suffix.lower() != ".json":
        raise ValueError("output must be a .json directly under the managed calibration directory")
    for source in inputs:
        try:
            if resolved_output == source.resolve(strict=True) or (
                lexical.exists() and source.exists() and os.path.samefile(lexical, source)
            ):
                raise ValueError("output must be distinct from every measurement input")
        except (OSError, RuntimeError) as exc:
            raise ValueError("input/output file identity cannot be checked safely") from exc
    return resolved_output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--measurement-record", type=Path, required=True,
        help="operator measurement JSON under .control-station/omx-calibrations/",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        record, record_reference, record_sha256 = _load_measurement_record(
            args.measurement_record
        )
        tcp_reference = _validate_measurement_record(record)
        artifact = {
            "schema_version": 1,
            "kind": "omx_gripper_geometry_measurement",
            "status": "DRAFT_ONLY",
            "review_required": True,
            "usable_for_robot_control": False,
            "arm_command_sent": False,
            "geometry_id": record["geometry_id"],
            "robot_model": record["robot_model"],
            "tcp": tcp_reference,
            "tool_geometry": record["tool_geometry"],
            "measurement": {
                **record["measurement"],
                "evidence_path": record_reference,
                "evidence_sha256": record_sha256,
            },
        }
        output_path = _ensure_output_path(
            args.output,
            [
                args.measurement_record.expanduser(),
                CALIBRATION_DIRECTORY / tcp_reference["artifact"]["path"],
            ],
        )
        # Validate the assembled payload and all referenced source hashes before publishing it.
        _validate(artifact, args.measurement_record.resolve(strict=True), record_sha256)
        artifact_bytes = _write_exclusive_json(output_path, artifact)
    except (OSError, RuntimeError, UnicodeError, ValueError, RecursionError) as exc:
        parser.error(str(exc))

    print(json.dumps({
        "geometry_id": artifact["geometry_id"],
        "status": "DRAFT_ONLY",
        "artifact": output_path.relative_to(REPO_ROOT).as_posix(),
        "artifact_sha256": hashlib.sha256(artifact_bytes).hexdigest(),
        "usable_for_robot_control": False,
        "arm_command_sent": False,
    }, ensure_ascii=False, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
