#!/usr/bin/env python3
"""Open or close the simulated OMX gripper using bounded position control."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "simulation" / "mujoco_omx"))

from omx_mujoco import OMXSimulation  # noqa: E402
from omx_mujoco.simulation import GRIPPER_CLOSED_RAD, GRIPPER_OPEN_RAD  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("open", "close"))
    parser.add_argument("--duration", type=float, default=0.8, help="interpolation duration in seconds")
    parser.add_argument("--scene", type=Path, help="optional MJCF scene path")
    args = parser.parse_args()
    target = GRIPPER_OPEN_RAD if args.action == "open" else GRIPPER_CLOSED_RAD
    try:
        simulation = OMXSimulation(args.scene) if args.scene else OMXSimulation()
        result = simulation.move_joint_targets({"Gripper": target}, args.duration)
        mimic = simulation.joint_positions(include_mimic=True)["Gripper_mimic"]
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"Gripper command rejected: {exc}", file=sys.stderr)
        return 2
    print(f"Gripper {args.action}: actuator={result['Gripper']:.4f} rad, mimic={mimic:.4f} rad")
    print("Simulation only; jaw aperture and object holding have not been calibrated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
