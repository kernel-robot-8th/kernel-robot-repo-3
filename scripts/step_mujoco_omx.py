#!/usr/bin/env python3
"""Apply explicit radian targets to the MuJoCo-only OMX model."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"
JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Gripper")


def _parse_target(value: str) -> tuple[str, float]:
    name, separator, raw_target = value.partition("=")
    if not separator or name not in JOINTS:
        raise argparse.ArgumentTypeError(
            f"target must use one of {', '.join(JOINTS)}=radians"
        )
    try:
        target = float(raw_target)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("target must be a finite radian value") from exc
    if not math.isfinite(target):
        raise argparse.ArgumentTypeError("target must be a finite radian value")
    return name, target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target",
        action="append",
        type=_parse_target,
        required=True,
        metavar="JOINT=RAD",
        help="Explicit MuJoCo target in radians; may be repeated",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=500,
        help="Simulation steps to run (1–100000; default: 500)",
    )
    args = parser.parse_args()
    if not 1 <= args.steps <= 100_000:
        parser.error("--steps must be between 1 and 100000")
    target_map = dict(args.target)
    if len(target_map) != len(args.target):
        parser.error("each joint may be specified only once")

    try:
        import mujoco
    except ImportError:
        print(
            "MuJoCo is not installed. Install the optional dependency with "
            "python -m pip install -r requirements-mujoco.txt",
            file=sys.stderr,
        )
        return 2

    try:
        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    except (OSError, ValueError) as exc:
        print(f"Could not load the MuJoCo OMX scene: {exc}", file=sys.stderr)
        return 2

    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)

    initial: dict[str, float] = {}
    for name, target in target_map.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if joint_id < 0 or actuator_id < 0:
            print(f"Required MuJoCo joint/actuator is missing: {name}", file=sys.stderr)
            return 2
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            print(f"{name} is not a scalar hinge joint in this model", file=sys.stderr)
            return 2
        if not model.actuator_ctrllimited[actuator_id]:
            print(f"{name} actuator has no bounded control range; refusing target", file=sys.stderr)
            return 2
        lower, upper = model.actuator_ctrlrange[actuator_id]
        if target < lower or target > upper:
            print(
                f"{name} target {target} rad is outside this model's actuator range "
                f"[{lower}, {upper}] rad",
                file=sys.stderr,
            )
            return 2
        qpos_address = model.jnt_qposadr[joint_id]
        initial[name] = float(data.qpos[qpos_address])
        data.ctrl[actuator_id] = target

    for _ in range(args.steps):
        mujoco.mj_step(model, data)

    simulated_seconds = args.steps * float(model.opt.timestep)
    print(
        f"MuJoCo {mujoco.__version__} · simulated {args.steps} steps "
        f"({simulated_seconds:.3f} s)"
    )
    for name, target in target_map.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_address = model.jnt_qposadr[joint_id]
        final = float(data.qpos[qpos_address])
        if not math.isfinite(final):
            print(f"{name} simulation state is non-finite; discarding readback", file=sys.stderr)
            return 2
        print(
            f"{name}: initial={initial[name]:.5f} rad, target={target:.5f} rad, "
            f"final={final:.5f} rad"
        )
    print("Simulation only. No ROS node, camera, serial port, or physical robot was opened.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
