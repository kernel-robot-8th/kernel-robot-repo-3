#!/usr/bin/env python3
"""Open the simulation-only ROBOTIS OMX MuJoCo scene."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "simulation" / "mujoco_omx"
sys.path.insert(0, str(PACKAGE_ROOT))

from omx_mujoco import OMXSimulation  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="optional MJCF scene path")
    args = parser.parse_args()
    try:
        simulation = OMXSimulation(args.scene) if args.scene else OMXSimulation()
        import mujoco.viewer
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"Could not start OMX MuJoCo simulation: {exc}", file=sys.stderr)
        return 2

    print(f"Loaded {simulation.scene_path}")
    print(f"MuJoCo {simulation.mujoco.__version__}; timestep={simulation.model.opt.timestep:.4f} s")
    for name, value in simulation.joint_positions().items():
        lower, upper = simulation.joint_limits(name)
        print(f"{name}: {value:+.4f} rad · limits [{lower:+.4f}, {upper:+.4f}] rad")
    print("Simulation only. No ROS node, camera device, serial port, or physical robot is opened.")
    mujoco.viewer.launch(simulation.model, simulation.data, show_left_ui=True, show_right_ui=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
