#!/usr/bin/env python3
"""Run repeatable RGB-D center-height strawberry grasp trials in MuJoCo only."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from importlib.metadata import version

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "mujoco_omx"))

from omx_mujoco.camera import CameraRig, save_rgb_png  # noqa: E402
from omx_mujoco.perception import detect_strawberry  # noqa: E402
from omx_mujoco.simulation import (  # noqa: E402
    ARM_JOINTS,
    GRIPPER_OPEN_RAD,
    OMXSimulation,
)


SCENE = ROOT / "simulation/mujoco_omx/robotis_omx/strawberry_pick_scene.xml"
OUTPUT = ROOT / "docs/implementation/assets/strawberry-repeatability-20261031.json"
SEED_Q = np.array([-0.4594, 0.4384, -1.2268, 1.5459, -0.3329])
TIP_NAMES = ("link6_fingertip_contact_proxy", "link7_fingertip_contact_proxy")
FRUIT_BODY = "photo_match_strawberry"
GRASP_CONTACT_ANGLE = 0.476
TIP_LEVEL_ERROR_MAX_MM = 5.0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def name_id(sim: OMXSimulation, object_type: object, name: str) -> int:
    result = sim.mujoco.mj_name2id(sim.model, object_type, name)
    if result < 0:
        raise RuntimeError(f"MuJoCo model is missing {name}")
    return int(result)


def center_height_ik(
    sim: OMXSimulation,
    target: np.ndarray,
    *,
    seed_q: np.ndarray | None = None,
    reference_yaw: float | None = None,
    gripper_angle: float = GRIPPER_OPEN_RAD,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Fit the open fingertip midpoint to target and keep fingertip centers level."""
    model, data, mujoco = sim.model, sim.data, sim.mujoco
    joint_ids = [sim._joint_ids[name] for name in ARM_JOINTS]
    qadr = [int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids]
    qlo = np.array([model.jnt_range[joint_id, 0] for joint_id in joint_ids])
    qhi = np.array([model.jnt_range[joint_id, 1] for joint_id in joint_ids])
    tips = [name_id(sim, mujoco.mjtObj.mjOBJ_GEOM, name) for name in TIP_NAMES]
    mimic_joint = name_id(sim, mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic")

    def fingertip_positions(q: np.ndarray) -> np.ndarray:
        for address, value in zip(qadr, q):
            data.qpos[address] = value
        gripper_position = model.jnt_qposadr[sim._joint_ids["Gripper"]]
        data.qpos[gripper_position] = gripper_angle
        data.qpos[model.jnt_qposadr[mimic_joint]] = -gripper_angle
        mujoco.mj_forward(model, data)
        return data.geom_xpos[tips].copy()

    seed = SEED_Q.copy() if seed_q is None else np.asarray(seed_q, dtype=np.float64).copy()
    seed_tips = fingertip_positions(seed)
    seed_axis = seed_tips[1] - seed_tips[0]
    target_yaw = (
        math.atan2(seed_axis[1], seed_axis[0])
        if reference_yaw is None
        else float(reference_yaw)
    )

    def residual(q: np.ndarray) -> np.ndarray:
        points = fingertip_positions(q)
        midpoint = points.mean(axis=0)
        axis = points[1] - points[0]
        yaw_error = math.atan2(axis[1], axis[0]) - target_yaw
        return np.r_[
            (midpoint - target) * 10.0,
            axis[2] * 10.0,
            math.sin(yaw_error) * 0.4,
            (q - seed) * 0.02,
        ]

    q = seed.copy()
    for _ in range(250):
        base = residual(q)
        jacobian = np.column_stack(
            [
                (residual(q + np.eye(5)[index] * 1e-5)
                 - residual(q - np.eye(5)[index] * 1e-5))
                / 2e-5
                for index in range(5)
            ]
        )
        delta = np.linalg.solve(
            jacobian.T @ jacobian + np.eye(5) * 1e-4,
            -jacobian.T @ base,
        )
        old_norm = float(np.linalg.norm(base))
        scale = 1.0
        while scale > 1e-4:
            candidate = np.clip(q + scale * delta, qlo + 1e-4, qhi - 1e-4)
            if np.linalg.norm(residual(candidate)) < old_norm:
                q = candidate
                break
            scale *= 0.5
        if np.linalg.norm(scale * delta) < 1e-8:
            break

    points = fingertip_positions(q)
    midpoint_error = float(np.linalg.norm(points.mean(axis=0) - target))
    return q, points, midpoint_error


def shelf_path_clear(
    sim: OMXSimulation,
    target_q: np.ndarray,
    fruit_offset: np.ndarray,
    *,
    start_q: np.ndarray | None = None,
) -> bool:
    model, data, mujoco = sim.model, sim.data, sim.mujoco
    joint_ids = [sim._joint_ids[name] for name in ARM_JOINTS]
    qadr = [int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids]
    if start_q is None:
        start_q = np.array([sim.joint_positions()[name] for name in ARM_JOINTS])
    free_joint = name_id(sim, mujoco.mjtObj.mjOBJ_JOINT, f"{FRUIT_BODY}_free")
    free_address = int(model.jnt_qposadr[free_joint])
    gripper_address = model.jnt_qposadr[sim._joint_ids["Gripper"]]
    mimic_joint = name_id(sim, mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic")
    mimic_address = model.jnt_qposadr[mimic_joint]
    clear = True
    for blend in np.linspace(0.0, 1.0, 101):
        for address, value in zip(qadr, start_q * (1.0 - blend) + target_q * blend):
            data.qpos[address] = value
        data.qpos[gripper_address] = GRIPPER_OPEN_RAD
        data.qpos[mimic_address] = -GRIPPER_OPEN_RAD
        data.qpos[free_address] = 0.10 + fruit_offset[0]
        data.qpos[free_address + 1] = 0.21 + fruit_offset[1]
        mujoco.mj_forward(model, data)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            first = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
            ) or ""
            second = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
            ) or ""
            first_body = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY,
                int(model.geom_bodyid[contact.geom1]),
            ) or "world"
            second_body = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY,
                int(model.geom_bodyid[contact.geom2]),
            ) or "world"
            geom_pair = first + second
            body_pair = first_body + second_body
            if "deck" in geom_pair and "link" in body_pair:
                clear = False
                break
            if "strawberry" in body_pair and "link" in body_pair:
                clear = False
                break
        if not clear:
            break
    return clear


def contacts_with_fruit(sim: OMXSimulation) -> set[str]:
    contacts: set[str] = set()
    fruit_id = sim.mujoco.mj_name2id(
        sim.model, sim.mujoco.mjtObj.mjOBJ_BODY, FRUIT_BODY
    )
    for index in range(sim.data.ncon):
        contact = sim.data.contact[index]
        first_body = int(sim.model.geom_bodyid[contact.geom1])
        second_body = int(sim.model.geom_bodyid[contact.geom2])
        if first_body == fruit_id:
            other_geom = contact.geom2
            other_body = second_body
        elif second_body == fruit_id:
            other_geom = contact.geom1
            other_body = first_body
        else:
            continue
        other_geom_name = sim.mujoco.mj_id2name(
            sim.model, sim.mujoco.mjtObj.mjOBJ_GEOM, other_geom
        )
        other_body_name = sim.mujoco.mj_id2name(
            sim.model, sim.mujoco.mjtObj.mjOBJ_BODY, other_body
        )
        contacts.add(other_geom_name or other_body_name or "world")
    return contacts


def run_trial(index: int, offset: np.ndarray, save_frame: bool) -> dict[str, object]:
    sim = OMXSimulation(SCENE, gripper_contact_proxy=True)
    model, data, mujoco = sim.model, sim.data, sim.mujoco
    fruit_id = name_id(sim, mujoco.mjtObj.mjOBJ_BODY, FRUIT_BODY)
    home_q = np.array([sim.joint_positions()[name] for name in ARM_JOINTS])
    free_joint = name_id(sim, mujoco.mjtObj.mjOBJ_JOINT, f"{FRUIT_BODY}_free")
    free_address = int(model.jnt_qposadr[free_joint])
    data.qpos[free_address] += float(offset[0])
    data.qpos[free_address + 1] += float(offset[1])
    mujoco.mj_forward(model, data)
    sim.step(120)
    initial_center = data.xpos[fruit_id].copy()

    observation = detect_strawberry(sim)
    if observation is None:
        return {"trial": index, "offset_m": offset.tolist(), "success": False,
                "failure": "camera_detection_failed"}
    target = np.asarray(observation.estimated_center_world_m, dtype=np.float64)
    q, fingertip_positions, midpoint_error = center_height_ik(
        sim, target, gripper_angle=GRASP_CONTACT_ANGLE
    )
    fingertip_axis = fingertip_positions[1] - fingertip_positions[0]
    clear = shelf_path_clear(sim, q, offset, start_q=home_q)
    if not clear:
        return {
            "trial": index,
            "offset_m": offset.tolist(),
            "camera_estimated_center_m": target.tolist(),
            "fingertip_midpoint_error_mm": midpoint_error * 1000.0,
            "shelf_path_clear": False,
            "success": False,
            "failure_reasons": ["direct_home_to_grasp_path_collision"],
        }

    # Reset after static path probing; preserve the same target offset.
    sim.reset()
    data.qpos[free_address] += float(offset[0])
    data.qpos[free_address + 1] += float(offset[1])
    mujoco.mj_forward(model, data)
    sim.step(120)
    initial_center = data.xpos[fruit_id].copy()
    sim.move_joint_targets({"Gripper": GRIPPER_OPEN_RAD}, 0.4)
    sim.move_joint_targets(dict(zip(ARM_JOINTS, q.tolist())), 2.5)
    pregrasp_center = data.xpos[fruit_id].copy()
    pregrasp_contacts = contacts_with_fruit(sim)
    pregrasp_drift_mm = float(np.linalg.norm(pregrasp_center - initial_center) * 1000.0)
    pregrasp_contact_free = not any(
        ("link" in contact_name) or contact_name == ""
        for contact_name in pregrasp_contacts
    )

    gripper_start = sim.joint_positions()["Gripper"]
    close_angle = gripper_start
    both_tips = False
    physics_per_close_step = 8
    for _ in range(math.ceil(gripper_start / 0.01)):
        close_angle = max(0.0, close_angle - 0.01)
        sim.set_joint_targets({"Gripper": close_angle})
        sim.step(physics_per_close_step)
        if set(TIP_NAMES).issubset(contacts_with_fruit(sim)):
            both_tips = True
            break

    link6_id = name_id(sim, mujoco.mjtObj.mjOBJ_BODY, "link6")
    lift_target = data.xpos[link6_id].copy()
    lift_target[2] += 0.055
    arm_q = np.array([sim.joint_positions()[name] for name in ARM_JOINTS])
    arm_ids = [sim._joint_ids[name] for name in ARM_JOINTS]
    arm_lower = np.array([model.jnt_range[joint_id, 0] for joint_id in arm_ids])
    arm_upper = np.array([model.jnt_range[joint_id, 1] for joint_id in arm_ids])
    for _ in range(180):
        error = lift_target - data.xpos[link6_id].copy()
        jacobian_position = np.zeros((3, model.nv))
        jacobian_rotation = np.zeros((3, model.nv))
        mujoco.mj_jacBody(
            model, data, jacobian_position, jacobian_rotation, link6_id
        )
        arm_jacobian = jacobian_position[
            :, [model.jnt_dofadr[joint_id] for joint_id in arm_ids]
        ]
        delta = arm_jacobian.T @ np.linalg.solve(
            arm_jacobian @ arm_jacobian.T + np.eye(3) * 1e-3, error
        )
        if np.linalg.norm(delta) < 1e-7:
            break
        arm_q = np.clip(arm_q + 0.1 * delta, arm_lower + 1e-4, arm_upper - 1e-4)
        sim.set_joint_targets(dict(zip(ARM_JOINTS, arm_q.tolist())))
        sim.step(8)

    lifted_center = data.xpos[fruit_id].copy()
    sim.step(math.ceil(1.0 / float(model.opt.timestep)))
    held_center = data.xpos[fruit_id].copy()
    contacts = contacts_with_fruit(sim)
    both_tips = both_tips and set(TIP_NAMES).issubset(contacts)
    exclusive_tip_contacts = contacts == set(TIP_NAMES)
    lift_mm = float((held_center[2] - initial_center[2]) * 1000.0)
    hold_drift_mm = float(np.linalg.norm(held_center - lifted_center) * 1000.0)
    midpoint_error_mm = midpoint_error * 1000.0
    tip_level_error_mm = abs(float(fingertip_axis[2])) * 1000.0
    success = bool(
        clear
        and midpoint_error_mm <= 3.0
        and tip_level_error_mm <= TIP_LEVEL_ERROR_MAX_MM
        and pregrasp_contact_free
        and pregrasp_drift_mm <= 3.0
        and both_tips
        and exclusive_tip_contacts
        and lift_mm >= 30.0
        and hold_drift_mm <= 10.0
    )

    if save_frame:
        frame_path = ROOT / "docs/implementation/assets" / f"strawberry-repeatability-{index:02d}.png"
        with CameraRig(sim) as cameras:
            save_rgb_png(frame_path, cameras.render_rgb("fixed_camera"))
    return {
        "trial": index,
        "offset_m": offset.tolist(),
        "camera_estimated_center_m": target.tolist(),
        "camera_center_error_to_scene_mm": float(
            np.linalg.norm(target - initial_center) * 1000.0
        ),
        "fingertip_midpoint_error_mm": midpoint_error_mm,
        "fingertip_height_difference_mm": tip_level_error_mm,
        "shelf_path_clear": clear,
        "pregrasp_contact_free": pregrasp_contact_free,
        "pregrasp_object_drift_mm": pregrasp_drift_mm,
        "pregrasp_object_contact_geoms": sorted(pregrasp_contacts),
        "both_fingertips_contact": both_tips,
        "gripper_angle_at_first_dual_contact_rad": close_angle if both_tips else None,
        "exclusive_fingertip_contacts": exclusive_tip_contacts,
        "fruit_contact_geoms": sorted(contacts),
        "object_lift_from_start_mm": lift_mm,
        "object_drift_during_1s_hold_mm": hold_drift_mm,
        "success": success,
        "failure_reasons": [
            reason
            for condition, reason in (
                (clear, "shelf_path_collision"),
                (midpoint_error_mm <= 3.0, "midpoint_alignment"),
                (tip_level_error_mm <= TIP_LEVEL_ERROR_MAX_MM, "fingertips_not_level"),
                (pregrasp_contact_free, "object_touched_before_grasp"),
                (pregrasp_drift_mm <= 3.0, "object_displaced_during_approach"),
                (both_tips, "missing_two_fingertip_contacts"),
                (exclusive_tip_contacts, "unintended_wrist_or_shelf_contact"),
                (lift_mm >= 30.0, "insufficient_lift"),
                (hold_drift_mm <= 10.0, "object_slipped_during_hold"),
            )
            if not condition
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20261031)
    parser.add_argument("--save-frames", action="store_true")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    if not 1 <= args.trials <= 100:
        parser.error("--trials must be between 1 and 100")

    rng = np.random.default_rng(args.seed)
    offsets = rng.uniform(-0.006, 0.006, size=(args.trials, 2))
    trials: list[dict[str, object]] = []
    started = time.monotonic()
    for index, offset in enumerate(offsets, start=1):
        try:
            result = run_trial(index, offset, args.save_frames)
        except Exception as exc:  # continue the batch and retain failure details
            result = {
                "trial": index,
                "offset_m": offset.tolist(),
                "success": False,
                "failure_reasons": [f"{type(exc).__name__}: {exc}"],
            }
        trials.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)

    successful = sum(bool(trial.get("success")) for trial in trials)
    lift_values = [
        float(trial["object_lift_from_start_mm"])
        for trial in trials
        if "object_lift_from_start_mm" in trial
    ]
    drift_values = [
        float(trial["object_drift_during_1s_hold_mm"])
        for trial in trials
        if "object_drift_during_1s_hold_mm" in trial
    ]
    summary = {
        "schema_version": 1,
        "simulation_only": True,
        "model_scene": str(SCENE.relative_to(ROOT)),
        "software_versions": {
            "python": sys.version.split()[0],
            "mujoco": version("mujoco"),
            "numpy": np.__version__,
        },
        "sha256": {
            str(path.relative_to(ROOT)): sha256_file(path)
            for path in (
                Path(__file__).resolve(),
                SCENE,
                SCENE.parent / "shelf_photo_match_scene.xml",
                SCENE.parent / "omx.xml",
                ROOT / "requirements-mujoco.txt",
            )
        },
        "gripper_contact_proxy": True,
        "scenario": "RGB-D center-height grasp; seeded x/y offsets uniform in +/-6 mm",
        "seed": args.seed,
        "trials_requested": args.trials,
        "successes": successful,
        "success_rate": successful / args.trials,
        "lift_mm_min_max_mean": (
            {
                "min": min(lift_values),
                "max": max(lift_values),
                "mean": float(np.mean(lift_values)),
            }
            if lift_values else None
        ),
        "hold_drift_mm_min_max_mean": (
            {
                "min": min(drift_values),
                "max": max(drift_values),
                "mean": float(np.mean(drift_values)),
            }
            if drift_values else None
        ),
        "elapsed_seconds": time.monotonic() - started,
        "criteria": {
            "no_static_shelf_contact_on_101_joint_waypoints": True,
            "fingertip_midpoint_error_mm_max": 3.0,
            "fingertip_height_difference_mm_max": TIP_LEVEL_ERROR_MAX_MM,
            "both_fingertip_proxy_contacts_after_hold": True,
            "no_unintended_object_contacts_after_hold": True,
            "object_lift_mm_min": 30.0,
            "object_drift_during_1s_hold_mm_max": 10.0,
        },
        "trials": trials,
        "limitations": [
            "Synthetic MuJoCo camera and scene only.",
            "Fingertip contacts use approximate 6 mm spheres; official collision mesh grasp is not validated.",
            "No real robot or physical strawberry was used.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in (
        "trials_requested", "successes", "success_rate",
        "lift_mm_min_max_mean", "hold_drift_mm_min_max_mean", "elapsed_seconds"
    )}, ensure_ascii=False, indent=2))
    print(f"wrote: {args.output}")
    return 0 if successful == args.trials else 1


if __name__ == "__main__":
    raise SystemExit(main())
