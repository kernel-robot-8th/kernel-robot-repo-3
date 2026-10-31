#!/usr/bin/env python3
"""Evaluate simulated strawberry transport and rear-floor release with explicit grasp modes."""

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
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "simulation" / "mujoco_omx"))

from omx_mujoco.camera import CameraRig, save_rgb_png  # noqa: E402
from omx_mujoco.perception import detect_strawberry  # noqa: E402
from omx_mujoco.simulation import (  # noqa: E402
    ARM_JOINTS,
    GRIPPER_OPEN_RAD,
    OMXSimulation,
)
from scripts.run_mujoco_strawberry_repeatability import (  # noqa: E402
    GRASP_CONTACT_ANGLE,
    TIP_NAMES,
    center_height_ik,
    contacts_with_fruit,
    name_id,
    shelf_path_clear,
)


SCENE = ROOT / "simulation/mujoco_omx/robotis_omx/strawberry_pick_scene.xml"
OUTPUT = ROOT / "docs/implementation/assets/strawberry-place-behind-results.json"
FRUIT_BODY = "photo_match_strawberry"
PICK_LIFT_M = 0.055
CARRY_WAYPOINT_OBJECT_CENTERS = (
    np.array([0.10, 0.04, 0.36]),   # clear the front lip of the shelf
    np.array([0.10, -0.20, 0.36]),  # move outside the shelf before turning behind
)
PLACE_IK_SEED = np.array([2.380958, 0.627422, -0.560600, 1.600972, 2.815710])
DEFAULT_PLACE_TARGETS = (
    (-0.20, -0.22),
    (-0.18, -0.20),
    (-0.22, -0.18),
    (-0.16, -0.22),
    (-0.22, -0.24),
)
ACTIVE_VIEWERS: list[object] = []
ACTIVE_SIMULATIONS: list[OMXSimulation] = []


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def set_arm_q(sim: OMXSimulation, q: np.ndarray) -> None:
    for name, value in zip(ARM_JOINTS, q):
        joint_id = sim._joint_ids[name]
        sim.data.qpos[sim.model.jnt_qposadr[joint_id]] = value


def finger_midpoint(sim: OMXSimulation) -> np.ndarray:
    tip_ids = [name_id(sim, sim.mujoco.mjtObj.mjOBJ_GEOM, name) for name in TIP_NAMES]
    return sim.data.geom_xpos[tip_ids].mean(axis=0).copy()


def quaternion_multiply_wxyz(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = (float(value) for value in first)
    bw, bx, by, bz = (float(value) for value in second)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def activate_simulation_grasp_weld(
    sim: OMXSimulation, link6_id: int, fruit_id: int
) -> np.ndarray:
    """Attach the fruit at the measured grasp pose; this is not friction validation."""
    if sim._grasp_weld_id < 0:
        raise RuntimeError("Simulation grasp weld was not enabled in the model")
    model, data = sim.model, sim.data
    equality_id = sim._grasp_weld_id
    rotation = data.xmat[link6_id].reshape(3, 3)
    relative_position = rotation.T @ (data.xpos[fruit_id] - data.xpos[link6_id])
    link_quaternion = data.xquat[link6_id].copy()
    fruit_quaternion = data.xquat[fruit_id].copy()
    inverse_link_quaternion = link_quaternion.copy()
    inverse_link_quaternion[1:] *= -1.0
    relative_quaternion = quaternion_multiply_wxyz(
        inverse_link_quaternion, fruit_quaternion
    )
    # Weld data stores the two local anchors followed by relative orientation.
    model.eq_data[equality_id, 0:3] = 0.0
    model.eq_data[equality_id, 3:6] = relative_position
    model.eq_data[equality_id, 6:10] = relative_quaternion
    data.eq_active[equality_id] = True
    sim.mujoco.mj_forward(model, data)
    return relative_position


def body_contact_pairs(sim: OMXSimulation) -> list[tuple[str, str, float]]:
    pairs: list[tuple[str, str, float]] = []
    model, data, mujoco = sim.model, sim.data, sim.mujoco
    for index in range(data.ncon):
        contact = data.contact[index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2)
        body1 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom1])
        )
        body2 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom2])
        )
        pairs.append((name1 or body1 or "world", name2 or body2 or "world", float(contact.dist)))
    return pairs


def static_contact_failure(
    sim: OMXSimulation,
    q: np.ndarray,
    object_center: np.ndarray,
    object_quaternion: np.ndarray,
    *,
    gripper_angle: float = GRASP_CONTACT_ANGLE,
    allow_object_floor: bool = False,
) -> list[tuple[str, str, float]]:
    model, data, mujoco = sim.model, sim.data, sim.mujoco
    fruit_id = name_id(sim, mujoco.mjtObj.mjOBJ_BODY, FRUIT_BODY)
    free_joint = name_id(sim, mujoco.mjtObj.mjOBJ_JOINT, f"{FRUIT_BODY}_free")
    free_address = int(model.jnt_qposadr[free_joint])
    gripper_address = int(model.jnt_qposadr[sim._joint_ids["Gripper"]])
    mimic_joint = name_id(sim, mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic")
    mimic_address = int(model.jnt_qposadr[mimic_joint])
    set_arm_q(sim, q)
    data.qpos[gripper_address] = gripper_angle
    data.qpos[mimic_address] = -gripper_angle
    mujoco.mj_forward(model, data)
    midpoint = finger_midpoint(sim)
    data.qpos[free_address : free_address + 3] = midpoint + (
        object_center - midpoint
    )
    data.qpos[free_address + 3 : free_address + 7] = object_quaternion
    mujoco.mj_forward(model, data)

    failures: list[tuple[str, str, float]] = []
    allowed_tip_names = set(TIP_NAMES)
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1) or ""
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2) or ""
        body1 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom1])
        ) or "world"
        body2 = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom2])
        ) or "world"
        names, bodies = {name1, name2}, {body1, body2}

        if FRUIT_BODY in bodies:
            other_name = name2 if body1 == FRUIT_BODY else name1
            other_body = body2 if body1 == FRUIT_BODY else body1
            if other_name in allowed_tip_names:
                continue
            if allow_object_floor and other_name == "photo_match_floor":
                continue
            failures.append((name1 or body1, name2 or body2, float(contact.dist)))
            continue

        # The robot base is expected to rest on the scene floor.
        if "world" in bodies and "link0" in bodies and "photo_match_floor" in names:
            continue
        is_robot_contact = any(body.startswith("link") for body in bodies)
        if not is_robot_contact:
            continue
        if any("photo_match_" in name for name in names):
            failures.append((name1 or body1, name2 or body2, float(contact.dist)))
        elif body1.startswith("link") and body2.startswith("link"):
            failures.append((name1 or body1, name2 or body2, float(contact.dist)))
    return failures


def nearest_joint_equivalent(
    value: float, reference: float, lower: float, upper: float
) -> float:
    candidates = [
        value + 2.0 * math.pi * turns
        for turns in range(-2, 3)
        if lower <= value + 2.0 * math.pi * turns <= upper
    ]
    if not candidates:
        raise ValueError("No equivalent revolute joint angle is inside the model limits")
    return min(candidates, key=lambda candidate: abs(candidate - reference))


def plan_carry_path(
    sim: OMXSimulation,
    q_start: np.ndarray,
    tcp_start: np.ndarray,
    object_offset: np.ndarray,
    object_quaternion: np.ndarray,
    finger_yaw: float,
    place_target: np.ndarray,
    contact_angle: float,
    waypoint_spacing_mm: float,
) -> tuple[list[np.ndarray], np.ndarray, dict[str, object]]:
    """Plan preflighted Cartesian shelf exit, front staging, and rear floor approach."""
    model = sim.model
    q_path: list[np.ndarray] = []
    q_previous = q_start.copy()
    tcp_previous = tcp_start.copy()
    all_ik_errors_mm: list[float] = []
    object_waypoints = (*CARRY_WAYPOINT_OBJECT_CENTERS, place_target)

    for segment_index, object_waypoint in enumerate(object_waypoints[:-1]):
        tcp_target = object_waypoint - object_offset
        count = max(
            2,
            math.ceil(
                np.linalg.norm(tcp_target - tcp_previous)
                / (waypoint_spacing_mm / 1000.0)
            ),
        )
        for fraction in np.linspace(0.0, 1.0, count + 1)[1:]:
            tcp = tcp_previous * (1.0 - fraction) + tcp_target * fraction
            q, _tips, residual = center_height_ik(
                sim,
                tcp,
                seed_q=q_previous,
                reference_yaw=finger_yaw,
                gripper_angle=contact_angle,
            )
            error_mm = residual * 1000.0
            all_ik_errors_mm.append(error_mm)
            if error_mm > 8.0:
                raise RuntimeError(
                    f"Cartesian carry IK failed in segment {segment_index} at "
                    f"{fraction:.3f}: residual {error_mm:.2f} mm"
                )
            failure = static_contact_failure(
                sim,
                q,
                tcp + object_offset,
                object_quaternion,
                gripper_angle=contact_angle,
            )
            if failure:
                raise RuntimeError(
                    f"Static collision in carry segment {segment_index}: {failure[0]}"
                )
            q_path.append(q.copy())
            q_previous = q
        tcp_previous = tcp_target

    # The rear floor target has several IK branches. This branch was selected
    # from a search of contact-free endpoint candidates, then wrapped to the
    # shortest valid Joint1 turn from the front staging pose.
    tcp_target = place_target - object_offset
    q_goal, _tips, goal_error = center_height_ik(
        sim,
        tcp_target,
        seed_q=PLACE_IK_SEED,
        reference_yaw=finger_yaw,
        gripper_angle=contact_angle,
    )
    q_goal[0] = nearest_joint_equivalent(
        float(q_goal[0]),
        float(q_previous[0]),
        float(model.jnt_range[sim._joint_ids["Joint1"], 0]),
        float(model.jnt_range[sim._joint_ids["Joint1"], 1]),
    )
    q_goal[4] = nearest_joint_equivalent(
        float(q_goal[4]),
        float(q_previous[4]),
        float(model.jnt_range[sim._joint_ids["Joint5"], 0]),
        float(model.jnt_range[sim._joint_ids["Joint5"], 1]),
    )
    if goal_error * 1000.0 > 5.0:
        raise RuntimeError(f"Rear floor IK residual {goal_error * 1000.0:.2f} mm")

    joint_path = np.linspace(q_previous, q_goal, 301)
    floor_geom = name_id(sim, sim.mujoco.mjtObj.mjOBJ_GEOM, "photo_match_floor")
    for index, q in enumerate(joint_path):
        alpha = index / (len(joint_path) - 1)
        # The floor is only an allowed object contact at the terminal pose.
        failure = static_contact_failure(
            sim,
            q,
            place_target,
            object_quaternion,
            gripper_angle=contact_angle,
            allow_object_floor=alpha >= 0.998,
        )
        if failure:
            raise RuntimeError(f"Static collision on rear turn at {alpha:.3f}: {failure[0]}")
        if index % 3 == 0 or index == len(joint_path) - 1:
            q_path.append(q.copy())

    return q_path, q_goal, {
        "cartesian_waypoints_object_center_m": [
            point.tolist() for point in CARRY_WAYPOINT_OBJECT_CENTERS
        ],
        "place_target_object_center_m": place_target.tolist(),
        "ik_error_mm_max": max(all_ik_errors_mm + [goal_error * 1000.0]),
        "planned_joint_samples": len(q_path),
        "place_joint_goal_rad": dict(zip(ARM_JOINTS, q_goal.tolist())),
        "static_collision_preflight": "clear",
        "joint1_turn_rad": float(q_goal[0] - q_previous[0]),
    }


def capture(sim: OMXSimulation, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with CameraRig(sim) as cameras:
        save_rgb_png(path, cameras.render_rgb("fixed_camera"))


def capture_placement_overview(sim: OMXSimulation, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mujoco = sim.mujoco
    renderer = mujoco.Renderer(sim.model, width=800, height=600)
    try:
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = [-0.02, -0.04, 0.18]
        camera.distance = 0.92
        camera.azimuth = 145.0
        camera.elevation = -18.0
        renderer.update_scene(sim.data, camera=camera)
        save_rgb_png(path, np.asarray(renderer.render()).copy())
    finally:
        renderer.close()


def execute_trial(
    index: int,
    fruit_offset_xy: np.ndarray,
    place_target_xy: tuple[float, float],
    save_frames: bool,
    steps_per_waypoint: int,
    waypoint_spacing_mm: float,
    preload_rad: float,
    proxy_radius_m: float,
    proxy_friction: float,
    grasp_model: str,
    show_viewer: bool,
) -> dict[str, object]:
    sim = OMXSimulation(
        SCENE,
        gripper_contact_proxy=True,
        gripper_contact_proxy_radius_m=proxy_radius_m,
        grasp_weld=grasp_model == "weld",
    )
    model, data, mujoco = sim.model, sim.data, sim.mujoco
    if show_viewer:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(
            model, data, show_left_ui=True, show_right_ui=True
        )
        viewer.cam.lookat[:] = [-0.02, -0.04, 0.18]
        viewer.cam.distance = 0.92
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -18.0
        sim.attach_viewer(viewer)
        ACTIVE_VIEWERS.append(viewer)
        ACTIVE_SIMULATIONS.append(sim)
    for geom_name in TIP_NAMES:
        proxy_id = name_id(sim, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        model.geom_friction[proxy_id, 0] = proxy_friction
    fruit_id = name_id(sim, mujoco.mjtObj.mjOBJ_BODY, FRUIT_BODY)
    free_joint = name_id(sim, mujoco.mjtObj.mjOBJ_JOINT, f"{FRUIT_BODY}_free")
    free_address = int(model.jnt_qposadr[free_joint])
    home_q = np.array([sim.joint_positions()[name] for name in ARM_JOINTS])
    data.qpos[free_address] += float(fruit_offset_xy[0])
    data.qpos[free_address + 1] += float(fruit_offset_xy[1])
    mujoco.mj_forward(model, data)
    sim.step(120)
    observation = detect_strawberry(sim)
    if observation is None:
        raise RuntimeError("Fixed-camera RGB-D did not detect the strawberry")
    target_center = np.asarray(observation.estimated_center_world_m, dtype=np.float64)
    q_pick, _tips, pick_error = center_height_ik(
        sim, target_center, gripper_angle=GRASP_CONTACT_ANGLE
    )
    if pick_error * 1000.0 > 3.0:
        raise RuntimeError(f"Pick midpoint IK residual {pick_error * 1000.0:.2f} mm")
    if not shelf_path_clear(
        sim, q_pick, fruit_offset_xy, start_q=home_q
    ):
        raise RuntimeError("Home-to-pick path collides with the shelf or strawberry")

    # Restore a known start state before physical simulation of the pick.
    sim.reset()
    data.qpos[free_address] += float(fruit_offset_xy[0])
    data.qpos[free_address + 1] += float(fruit_offset_xy[1])
    mujoco.mj_forward(model, data)
    sim.step(120)
    fruit_center_start = data.xpos[fruit_id].copy()
    sim.move_joint_targets({"Gripper": GRIPPER_OPEN_RAD}, 0.4)
    sim.move_joint_targets(dict(zip(ARM_JOINTS, q_pick.tolist())), 2.5)
    pregrasp_center = data.xpos[fruit_id].copy()
    pregrasp_contacts = contacts_with_fruit(sim)
    pregrasp_drift_mm = float(np.linalg.norm(pregrasp_center - fruit_center_start) * 1000.0)
    if any(name.startswith("link") or name == "" for name in pregrasp_contacts):
        raise RuntimeError(f"Object was touched before the grasp: {sorted(pregrasp_contacts)}")
    if pregrasp_drift_mm > 3.0:
        raise RuntimeError(f"Approach displaced the strawberry by {pregrasp_drift_mm:.2f} mm")

    gripper_angle = sim.joint_positions()["Gripper"]
    dual_contact = False
    for _ in range(math.ceil(gripper_angle / 0.01)):
        gripper_angle = max(0.0, gripper_angle - 0.01)
        sim.set_joint_targets({"Gripper": gripper_angle})
        sim.step(8)
        if set(TIP_NAMES).issubset(contacts_with_fruit(sim)):
            dual_contact = True
            break
    if not dual_contact:
        raise RuntimeError("Both fingertip contacts were not acquired")

    # Add a small, bounded closing preload after both pads first touch. This
    # improves the contact margin before the longer carry motion.
    preload_angle = max(0.0, gripper_angle - preload_rad)
    pre_preload_center = data.xpos[fruit_id].copy()
    while gripper_angle - preload_angle > 1e-6:
        gripper_angle = max(preload_angle, gripper_angle - 0.005)
        sim.set_joint_targets({"Gripper": gripper_angle})
        sim.step(12)
        if not set(TIP_NAMES).issubset(contacts_with_fruit(sim)):
            raise RuntimeError("Dual fingertip contact was lost during preload")
    preload_object_shift_mm = float(
        np.linalg.norm(data.xpos[fruit_id] - pre_preload_center) * 1000.0
    )

    link6_id = name_id(sim, mujoco.mjtObj.mjOBJ_BODY, "link6")
    grasp_weld_relative_position = None
    if grasp_model == "weld":
        grasp_weld_relative_position = activate_simulation_grasp_weld(
            sim, link6_id, fruit_id
        )
    lift_target = data.xpos[link6_id].copy()
    lift_target[2] += PICK_LIFT_M
    arm_ids = [sim._joint_ids[name] for name in ARM_JOINTS]
    lower = np.array([model.jnt_range[joint_id, 0] for joint_id in arm_ids])
    upper = np.array([model.jnt_range[joint_id, 1] for joint_id in arm_ids])
    arm_q = np.array([sim.joint_positions()[name] for name in ARM_JOINTS])
    for _ in range(180):
        error = lift_target - data.xpos[link6_id].copy()
        jacobian_position = np.zeros((3, model.nv))
        jacobian_rotation = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacobian_position, jacobian_rotation, link6_id)
        jacobian = jacobian_position[:, [model.jnt_dofadr[joint_id] for joint_id in arm_ids]]
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + np.eye(3) * 1e-3, error
        )
        if np.linalg.norm(delta) < 1e-7:
            break
        arm_q = np.clip(arm_q + 0.1 * delta, lower + 1e-4, upper - 1e-4)
        sim.set_joint_targets(dict(zip(ARM_JOINTS, arm_q.tolist())))
        sim.set_joint_targets({"Gripper": gripper_angle})
        sim.step(8)
    sim.step(math.ceil(1.0 / float(model.opt.timestep)))
    pick_hold_contacts = contacts_with_fruit(sim)
    if pick_hold_contacts != set(TIP_NAMES):
        raise RuntimeError(f"Unexpected contact after pick lift: {sorted(pick_hold_contacts)}")

    tip_midpoint = finger_midpoint(sim)
    held_center = data.xpos[fruit_id].copy()
    object_offset = held_center - tip_midpoint
    object_quaternion = data.qpos[free_address + 3 : free_address + 7].copy()
    tip_ids = [name_id(sim, mujoco.mjtObj.mjOBJ_GEOM, name) for name in TIP_NAMES]
    finger_axis = data.geom_xpos[tip_ids[1]] - data.geom_xpos[tip_ids[0]]
    finger_yaw = math.atan2(finger_axis[1], finger_axis[0])
    q_carry_start = np.array([sim.joint_positions()[name] for name in ARM_JOINTS])
    place_target = np.array([place_target_xy[0], place_target_xy[1], 0.037])

    # Static IK/contact preflight writes qpos for collision checks. Restore the
    # physically simulated grasp state before executing its planned trajectory.
    state_snapshot = {
        "qpos": data.qpos.copy(),
        "qvel": data.qvel.copy(),
        "ctrl": data.ctrl.copy(),
        "act": data.act.copy(),
        "eq_active": data.eq_active.copy(),
        "time": float(data.time),
    }
    q_path, q_goal, plan = plan_carry_path(
        sim,
        q_carry_start,
        tip_midpoint,
        object_offset,
        object_quaternion,
        finger_yaw,
        place_target,
        gripper_angle,
        waypoint_spacing_mm,
    )
    data.qpos[:] = state_snapshot["qpos"]
    data.qvel[:] = state_snapshot["qvel"]
    data.ctrl[:] = state_snapshot["ctrl"]
    data.act[:] = state_snapshot["act"]
    data.eq_active[:] = state_snapshot["eq_active"]
    data.time = state_snapshot["time"]
    mujoco.mj_forward(model, data)
    if save_frames:
        capture(
            sim,
            ROOT / "docs/implementation/assets" / f"place-behind-trial-{index:02d}-grasped.png",
        )

    # Leave enough simulated time for the position actuators to track each
    # short waypoint without dragging the strawberry out of the contact pair.
    step_count = max(1, round(steps_per_waypoint * waypoint_spacing_mm / 5.0))
    for path_index, target_q in enumerate(q_path):
        sim.set_joint_targets(dict(zip(ARM_JOINTS, target_q.tolist())))
        sim.set_joint_targets({"Gripper": gripper_angle})
        sim.step(step_count)
        contacts = contacts_with_fruit(sim)
        if grasp_model == "contact" and not set(TIP_NAMES).issubset(contacts):
            if save_frames:
                capture(
                    sim,
                    ROOT / "docs/implementation/assets" / f"place-behind-trial-{index:02d}-contact-loss.png",
                )
            object_pairs = [
                [name1, name2, distance]
                for name1, name2, distance in body_contact_pairs(sim)
                if FRUIT_BODY in name1 + name2
                or "photo_match_strawberry_body" in name1 + name2
            ]
            raise RuntimeError(
                f"Lost fingertip contact during carry at sample {path_index}: "
                f"contacts={sorted(contacts)}, object_xyz={data.xpos[fruit_id].tolist()}, "
                f"object_contacts={object_pairs}, "
                f"actual_q={[sim.joint_positions()[name] for name in ARM_JOINTS]}, "
                f"target_q={target_q.tolist()}"
            )
        if grasp_model == "weld":
            rotation = data.xmat[link6_id].reshape(3, 3)
            actual_relative_position = rotation.T @ (
                data.xpos[fruit_id] - data.xpos[link6_id]
            )
            weld_error_mm = float(
                np.linalg.norm(actual_relative_position - grasp_weld_relative_position)
                * 1000.0
            )
            if weld_error_mm > 10.0:
                raise RuntimeError(
                    f"Simulation grasp weld drift exceeded 10 mm at sample {path_index}: "
                    f"{weld_error_mm:.2f} mm"
                )
        expected_contacts = set(TIP_NAMES)
        if path_index >= len(q_path) - 4:
            expected_contacts.add("photo_match_floor")
        unexpected = contacts - expected_contacts
        if unexpected:
            raise RuntimeError(
                f"Unexpected object contact during carry at sample {path_index}: "
                f"{sorted(unexpected)}"
            )

    pre_release_center = data.xpos[fruit_id].copy()
    if np.linalg.norm(pre_release_center[:2] - place_target[:2]) > 0.015:
        raise RuntimeError(
            f"Before release, strawberry is {np.linalg.norm(pre_release_center[:2] - place_target[:2])*1000:.1f} mm from target"
        )
    if not 0.025 <= pre_release_center[2] <= 0.060:
        raise RuntimeError(f"Before release, strawberry height is {pre_release_center[2]:.4f} m")
    if save_frames:
        capture(
            sim,
            ROOT / "docs/implementation/assets" / f"place-behind-trial-{index:02d}-rear-floor.png",
        )

    if grasp_model == "weld":
        data.eq_active[sim._grasp_weld_id] = False
        mujoco.mj_forward(model, data)
    sim.move_joint_targets({"Gripper": GRIPPER_OPEN_RAD}, 1.0)
    sim.step(math.ceil(1.0 / float(model.opt.timestep)))
    released_center = data.xpos[fruit_id].copy()
    released_contacts = contacts_with_fruit(sim)
    floor_contact = any(
        "photo_match_floor" in name1 + name2
        for name1, name2, _distance in body_contact_pairs(sim)
        if FRUIT_BODY in name1 + name2 or "photo_match_strawberry_body" in name1 + name2
    )
    release_position_error_mm = float(
        np.linalg.norm(released_center[:2] - place_target[:2]) * 1000.0
    )
    success = bool(
        release_position_error_mm <= 15.0
        and 0.025 <= released_center[2] <= 0.055
        and not (released_contacts & set(TIP_NAMES))
        and floor_contact
    )
    if save_frames:
        capture(
            sim,
            ROOT / "docs/implementation/assets" / f"place-behind-trial-{index:02d}-released.png",
        )
        capture_placement_overview(
            sim,
            ROOT / "docs/implementation/assets" / f"place-behind-trial-{index:02d}-released-overview.png",
        )
    return {
        "trial": index,
        "fruit_offset_xy_m": fruit_offset_xy.tolist(),
        "camera_estimated_pick_center_m": target_center.tolist(),
        "pick_ik_error_mm": pick_error * 1000.0,
        "pregrasp_object_drift_mm": pregrasp_drift_mm,
        "pick_dual_contact": dual_contact,
        "gripper_contact_proxy_radius_m": proxy_radius_m,
        "gripper_contact_proxy_friction": proxy_friction,
        "grasp_model": grasp_model,
        "simulation_grasp_weld_relative_position_m": (
            None if grasp_weld_relative_position is None
            else grasp_weld_relative_position.tolist()
        ),
        "gripper_angle_after_preload_rad": gripper_angle,
        "preload_object_shift_mm": preload_object_shift_mm,
        "pick_hold_contacts": sorted(pick_hold_contacts),
        "held_object_offset_from_fingertip_midpoint_m": object_offset.tolist(),
        "carry_plan": plan,
        "place_target_m": place_target.tolist(),
        "object_before_release_m": pre_release_center.tolist(),
        "object_after_release_m": released_center.tolist(),
        "released_object_contacts": sorted(released_contacts),
        "floor_contact_after_release": floor_contact,
        "release_xy_error_mm": release_position_error_mm,
        "success": success,
        "failure_reasons": [] if success else ["floor_release_verification_failed"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed", type=int, default=20261031)
    parser.add_argument("--save-frames", action="store_true")
    parser.add_argument("--steps-per-waypoint", type=int, default=100)
    parser.add_argument("--waypoint-spacing-mm", type=float, default=5.0)
    parser.add_argument("--preload-rad", type=float, default=0.03)
    parser.add_argument("--proxy-radius-mm", type=float, default=6.0)
    parser.add_argument("--proxy-friction", type=float, default=2.0)
    parser.add_argument("--grasp-model", choices=("contact", "weld"), default="contact")
    parser.add_argument("--viewer", action="store_true", help="play one trial in a MuJoCo GUI window")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials must be positive")
    if args.viewer and args.trials != 1:
        parser.error("--viewer requires --trials 1")
    if args.steps_per_waypoint < 1:
        parser.error("--steps-per-waypoint must be positive")
    if not 0.5 <= args.waypoint_spacing_mm <= 20.0:
        parser.error("--waypoint-spacing-mm must be between 0.5 and 20 mm")
    if not 0.0 <= args.preload_rad <= 0.30:
        parser.error("--preload-rad must be between 0 and 0.30 radians")
    if not 2.0 <= args.proxy_radius_mm <= 20.0:
        parser.error("--proxy-radius-mm must be between 2 and 20 mm")
    if not 0.1 <= args.proxy_friction <= 50.0:
        parser.error("--proxy-friction must be between 0.1 and 50")

    rng = np.random.default_rng(args.seed)
    trials: list[dict[str, object]] = []
    started = time.monotonic()
    for index in range(args.trials):
        pick_offset = rng.uniform(-0.006, 0.006, size=2)
        place_xy = DEFAULT_PLACE_TARGETS[index % len(DEFAULT_PLACE_TARGETS)]
        try:
            result = execute_trial(
                index + 1, pick_offset, place_xy, args.save_frames,
                args.steps_per_waypoint, args.waypoint_spacing_mm,
                args.preload_rad,
                args.proxy_radius_mm / 1000.0,
                args.proxy_friction,
                args.grasp_model,
                args.viewer,
            )
        except Exception as exc:
            result = {
                "trial": index + 1,
                "fruit_offset_xy_m": pick_offset.tolist(),
                "place_target_xy_m": list(place_xy),
                "success": False,
                "failure_reasons": [f"{type(exc).__name__}: {exc}"],
            }
        trials.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)

    summary = {
        "schema_version": 1,
        "simulation_only": True,
        "scene": str(SCENE.relative_to(ROOT)),
        "seed": args.seed,
        "software_versions": {
            "python": sys.version.split()[0],
            "mujoco": version("mujoco"),
            "numpy": np.__version__,
        },
        "sha256": {
            str(path.relative_to(ROOT)): sha256_file(path)
            for path in (
                Path(__file__).resolve(),
                ROOT / "scripts/run_mujoco_strawberry_repeatability.py",
                SCENE,
                SCENE.parent / "shelf_photo_match_scene.xml",
                SCENE.parent / "omx.xml",
                ROOT / "requirements-mujoco.txt",
            )
        },
        "gripper_model": {
            "contact_proxy_radius_mm": args.proxy_radius_mm,
            "closing_preload_rad": args.preload_rad,
        },
        "steps_per_waypoint": args.steps_per_waypoint,
        "waypoint_spacing_mm": args.waypoint_spacing_mm,
        "proxy_contact_friction": args.proxy_friction,
        "grasp_model": args.grasp_model,
        "trials_requested": args.trials,
        "successes": sum(bool(trial.get("success")) for trial in trials),
        "success_rate": sum(bool(trial.get("success")) for trial in trials) / args.trials,
        "elapsed_seconds": time.monotonic() - started,
        "trials": trials,
        "limitations": [
            "Synthetic fixed camera and illustrative shelf/strawberry scene only.",
            "Contact proxies are approximate; official collision meshes are not a validated grasp model.",
            "In weld mode the fruit is attached after dual-tip contact; this validates transport/place, not frictional grasp retention.",
            "The rear floor place is not validated on real hardware.",
        ],
    }
    output_path = args.output
    if args.viewer and output_path == OUTPUT:
        output_path = ROOT / "docs/implementation/assets/strawberry-place-behind-viewer-result.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in (
        "trials_requested", "successes", "success_rate", "elapsed_seconds"
    )}, ensure_ascii=False, indent=2))
    print(f"wrote: {output_path}")
    if ACTIVE_VIEWERS:
        print("MuJoCo replay is complete. Close the viewer window to exit.", flush=True)
        while any(viewer.is_running() for viewer in ACTIVE_VIEWERS):
            for viewer in ACTIVE_VIEWERS:
                if viewer.is_running():
                    viewer.sync()
            time.sleep(0.05)
    return 0 if summary["successes"] == args.trials else 1


if __name__ == "__main__":
    raise SystemExit(main())
