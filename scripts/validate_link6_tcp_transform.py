#!/usr/bin/env python3
"""Validate a draft TCP transform artifact and its local source-file hashes."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from calibration_artifacts import load_link6_tcp_transform_artifact


REPO_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    args = parser.parse_args()
    try:
        loaded = load_link6_tcp_transform_artifact(args.artifact, REPO_ROOT)
    except (OSError, ValueError) as exc:
        print(f"TCP transform artifact rejected: {exc}", file=sys.stderr)
        return 2

    calibration = loaded["validated"]
    result = {
        "validation_status": "DRAFT_CONTRACT_AND_SOURCE_HASHES_MATCH",
        "calibration_id": calibration["calibration_id"],
        "status": "DRAFT_ONLY",
        "artifact": loaded["artifact"],
        "robot_model": calibration["robot_model"],
        "frame_mapping": {
            **calibration["frame_mapping"],
            "verified_evidence_file": loaded["verified_evidence"]["frame_mapping"],
        },
        "tcp_transform": calibration["tcp_transform"],
        "measurement": {
            **calibration["measurement"],
            "verified_source_file": loaded["verified_evidence"]["measurement"],
        },
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "physical_accuracy_verified": False,
        "robot_readiness_verified": False,
    }
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
