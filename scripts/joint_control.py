#!/usr/bin/env python3
"""Read or smoothly move OMX simulation joints; targets are radians."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "simulation" / "mujoco_omx"))

from omx_mujoco import OMXSimulation  # noqa: E402


def parse_target(value: str) -> tuple[str, float]:
    name, separator, raw_value = value.partition("=")
    try:
        target = float(raw_value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("target must be JOINT=RADIANS") from exc
    if not separator or not math.isfinite(target):
        raise argparse.ArgumentTypeError("target must be JOINT=finite_RADIANS")
    return name, target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", action="append", type=parse_target, default=[], metavar="NAME=RAD")
    parser.add_argument("--duration", type=float, default=1.0, help="interpolation duration in seconds")
    parser.add_argument("--scene", type=Path, help="optional MJCF scene path")
    args = parser.parse_args()
    if len(dict(args.joint)) != len(args.joint):
        parser.error("a joint may be specified only once")
    try:
        simulation = OMXSimulation(args.scene) if args.scene else OMXSimulation()
        if args.joint:
            start = simulation.joint_positions()
            final = simulation.move_joint_targets(dict(args.joint), args.duration)
            for name, target in dict(args.joint).items():
                print(f"{name}: {start[name]:+.4f} -> target {target:+.4f} -> simulated {final[name]:+.4f} rad")
        else:
            for name, value in simulation.joint_positions().items():
                lower, upper = simulation.joint_limits(name)
                print(f"{name}: {value:+.4f} rad · limits [{lower:+.4f}, {upper:+.4f}] rad")
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"Joint control rejected: {exc}", file=sys.stderr)
        return 2
    print("Simulation only; no physical robot command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
