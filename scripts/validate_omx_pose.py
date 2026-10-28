#!/usr/bin/env python3
"""Validate an offline OMX joint-pose JSON file without connecting to a robot."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from control_station.omx_pose import OmxJointPose, PoseValidationError  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose_file", type=Path, help="Path to an OMX joint-pose JSON file")
    args = parser.parse_args()

    try:
        with args.pose_file.open(encoding="utf-8") as pose_file:
            payload = json.load(pose_file)
        if not isinstance(payload, dict):
            raise PoseValidationError("top-level JSON value must be an object")
        pose = OmxJointPose.from_mapping(payload)
    except (OSError, json.JSONDecodeError, PoseValidationError) as exc:
        print(f"Invalid OMX pose: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(pose.to_mapping(), indent=2))
    print("Valid input only; no robot was connected and no command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
