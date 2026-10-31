#!/usr/bin/env python3
"""Validate an offline OMX joint-pose JSON file without connecting to a robot."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from control_station.omx_pose import OmxJointPose, PoseValidationError  # noqa: E402


_MAX_POSE_FILE_BYTES = 64 * 1024


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON object key: {key}")
        value[key] = item
    return value


def _read_pose_mapping(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise PoseValidationError("pose file must be a regular non-symlink file")
    try:
        if path.stat().st_size > _MAX_POSE_FILE_BYTES:
            raise PoseValidationError("pose file exceeds the 64 KiB limit")
        with path.open("rb") as pose_file:
            raw = pose_file.read(_MAX_POSE_FILE_BYTES + 1)
    except OSError as exc:
        raise PoseValidationError("pose file cannot be read") from exc
    if len(raw) > _MAX_POSE_FILE_BYTES:
        raise PoseValidationError("pose file exceeds the 64 KiB limit")
    try:
        payload = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise PoseValidationError("pose file must be valid UTF-8 JSON with unique keys") from exc
    if not isinstance(payload, dict):
        raise PoseValidationError("top-level JSON value must be an object")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose_file", type=Path, help="Path to an OMX joint-pose JSON file")
    args = parser.parse_args()

    try:
        payload = _read_pose_mapping(args.pose_file)
        pose = OmxJointPose.from_mapping(payload)
    except PoseValidationError as exc:
        print(f"Invalid OMX pose: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(pose.to_mapping(), indent=2))
    print("Valid input only; no robot was connected and no command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
