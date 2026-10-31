#!/usr/bin/env python3
"""Launch the photo-match OMX scene in MuJoCo's native interactive Simulate GUI."""

from __future__ import annotations

import math
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = (
    REPO_ROOT
    / "simulation"
    / "mujoco_omx"
    / "robotis_omx"
    / "shelf_photo_match_scene.xml"
)
RAW_POSE = {
    "Joint1": 1732,
    "Joint2": 729,
    "Joint3": 2910,
    "Joint4": 2942,
    "Joint5": 1934,
    "Gripper": 2403,
}
ARM_JOINTS = tuple(RAW_POSE)[:-1]
# Visual convention for this scene: the robot's local +X axis is its front.
# The base faces inward normal to the shelf front; Joint1 turns the arm right.
PHOTO_FIT_JOINT1 = math.radians(-65.0)
SHELF_INWARD_YAW = math.radians(90.0)
BASE_OFFSET = (-0.03, -0.03, 0.0)  # 3 cm left and backward from the previous pose.


def _observed_joint_value(model, mujoco, name: str) -> tuple[float, str | None]:  # type: ignore[no-untyped-def]
    raw_radians = (RAW_POSE[name] % 4096) * (2.0 * math.pi / 4096)
    wrapped = (raw_radians + math.pi) % (2.0 * math.pi) - math.pi
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    lower, upper = (float(value) for value in model.jnt_range[joint_id])
    value = min(upper, max(lower, wrapped))
    note = f"{name} nominal {wrapped:.4f} rad clipped to {value:.4f} rad model limit" if value != wrapped else None
    return value, note


def main() -> int:
    try:
        import mujoco
        from mujoco import viewer
    except ImportError:
        print("MuJoCo is missing. Install requirements-mujoco.txt in the dedicated environment.", file=sys.stderr)
        return 2

    # Aim the base normal to the shelf front, not at the strawberry's position.
    # Local +X is treated as robot front; the shelf inward direction is world +Y.
    spec = mujoco.MjSpec.from_file(str(SCENE_PATH))
    base_yaw = SHELF_INWARD_YAW
    spec.body("link0").pos = list(BASE_OFFSET)
    spec.body("link0").quat = [
        math.cos(base_yaw / 2.0),
        0.0,
        0.0,
        math.sin(base_yaw / 2.0),
    ]
    model = spec.compile()
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    mapping_notes: list[str] = []
    for name in ARM_JOINTS:
        value, note = _observed_joint_value(model, mujoco, name)
        if name == "Joint1":
            value = PHOTO_FIT_JOINT1
            note = "Joint1 set to -65° as a visual pose fit: arm right of base-forward; not encoder-calibrated"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = value
        data.ctrl[actuator_id] = value
        if note:
            mapping_notes.append(note)

    gripper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper")
    gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Gripper")
    gripper_value = RAW_POSE["Gripper"] / 4095 * math.radians(100)
    gripper_value = min(float(model.jnt_range[gripper_id, 1]), max(float(model.jnt_range[gripper_id, 0]), gripper_value))
    data.qpos[int(model.jnt_qposadr[gripper_id])] = gripper_value
    data.ctrl[gripper_actuator_id] = gripper_value
    mujoco.mj_forward(model, data)

    print("Launching MuJoCo Simulate GUI with the photo-match two-tier scene.", flush=True)
    print("Use MuJoCo's run/pause controls and Actuators panel to manipulate the arm in this window.", flush=True)
    print("Simulation only: actual camera calibration and encoder zero/sign/scale are unknown.", flush=True)
    print(f"Photo-fit base yaw: {math.degrees(base_yaw):.1f}° normal to the shelf front; local +X faces inward.", flush=True)
    print(f"Base translated {BASE_OFFSET[0]:.2f} m left and {BASE_OFFSET[1]:.2f} m backward from the prior simulator pose.", flush=True)
    print("Provisional mapping: raw position × 2π/4096; gripper mapped linearly to 0–100 degrees.", flush=True)
    for note in mapping_notes:
        print(f"Mapping note: {note}", flush=True)
    print("No physical robot, serial port, or camera device is opened.", flush=True)
    viewer.launch(model, data, show_left_ui=True, show_right_ui=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
