#!/usr/bin/env python3
"""Inspect the ROBOTIS OMX MuJoCo scene or open its simulation viewer."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"
JOINT_MAP = (
    ("shoulder_pan.pos", "Joint1"),
    ("shoulder_lift.pos", "Joint2"),
    ("elbow_flex.pos", "Joint3"),
    ("wrist_flex.pos", "Joint4"),
    ("wrist_roll.pos", "Joint5"),
    ("gripper.pos", "Gripper"),
)
GRIPPER_ANGLE_SAMPLES_DEG = (0.0, 25.0, 50.0, 75.0, 100.0)


def inspect_gripper_mesh_distance(model, data, mujoco) -> list[dict[str, object]]:  # type: ignore[no-untyped-def]
    """Measure the two modeled gripper collision meshes; this is not jaw aperture."""
    import numpy as np

    joint_ids = {}
    for name in ("Gripper", "Gripper_mimic"):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f"Required gripper hinge joint is missing: {name}")
        joint_ids[name] = int(model.jnt_qposadr[joint_id])

    collision_geoms = {}
    for body_name in ("link6", "link7"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise ValueError(f"Required gripper body is missing: {body_name}")
        geom_ids = [
            geom_id
            for geom_id in range(model.ngeom)
            if int(model.geom_bodyid[geom_id]) == body_id
            and (int(model.geom_contype[geom_id]) != 0 or int(model.geom_conaffinity[geom_id]) != 0)
        ]
        if len(geom_ids) != 1:
            raise ValueError(
                f"Expected one collision geom on {body_name}; found {len(geom_ids)}"
            )
        collision_geoms[body_name] = geom_ids[0]

    samples = []
    for angle_degrees in GRIPPER_ANGLE_SAMPLES_DEG:
        angle_radians = math.radians(angle_degrees)
        data.qpos[joint_ids["Gripper"]] = angle_radians
        data.qpos[joint_ids["Gripper_mimic"]] = -angle_radians
        mujoco.mj_forward(model, data)
        geom_link6 = collision_geoms["link6"]
        geom_link7 = collision_geoms["link7"]
        nearest_points = np.empty(6, dtype=np.float64)
        signed_distance_m = mujoco.mj_geomDistance(
            model, data, geom_link6, geom_link7, 1.0, nearest_points
        )
        mesh_ids = [int(model.geom_dataid[geom_link6]), int(model.geom_dataid[geom_link7])]
        mesh_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
            for mesh_id in mesh_ids
        ]
        samples.append(
            {
                "gripper_angle_degrees": angle_degrees,
                "gripper_mimic_angle_degrees": -angle_degrees,
                "collision_meshes": mesh_names,
                "signed_geom_distance_m": float(signed_distance_m),
                "nearest_points_world_m": [nearest_points[:3].tolist(), nearest_points[3:].tolist()],
                "contact_count_after_mj_forward": int(data.ncon),
            }
        )
    return samples


def _parse_joint_center(value: str) -> dict[str, float]:
    expected = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5"}
    parsed: dict[str, float] = {}
    try:
        for item in value.split(","):
            name, raw_value = item.split("=", maxsplit=1)
            name = name.strip()
            if name not in expected or name in parsed:
                raise ValueError
            joint_value = float(raw_value.strip())
            if not math.isfinite(joint_value):
                raise ValueError
            parsed[name] = joint_value
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "local center must list each Joint1–Joint5 once as Joint1=RAD,..."
        ) from exc
    if set(parsed) != expected:
        raise argparse.ArgumentTypeError(
            "local center must list each Joint1–Joint5 once as Joint1=RAD,..."
        )
    return parsed


def _parse_box_half_size(value: str) -> tuple[float, float, float]:
    try:
        values = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("box half-size must be X,Y,Z in meters") from exc
    if len(values) != 3 or not all(math.isfinite(item) and item > 0 for item in values):
        raise argparse.ArgumentTypeError("box half-size must contain three positive finite meter values")
    return values


def _parse_joint_order(value: str) -> tuple[str, ...]:
    expected = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5"}
    order = tuple(name.strip() for name in value.split(","))
    if len(order) != len(expected) or set(order) != expected:
        raise argparse.ArgumentTypeError(
            "joint order must list Joint1–Joint5 exactly once, separated by commas"
        )
    return order


def load_scene_model(mujoco, object_half_size_m: tuple[float, float, float] | None = None):  # type: ignore[no-untyped-def]
    if object_half_size_m is None:
        return mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    spec = mujoco.MjSpec.from_file(str(SCENE_PATH))
    object_geom = spec.geom("placeholder_pick_object_geom")
    if object_geom is None or object_geom.type != mujoco.mjtGeom.mjGEOM_BOX:
        raise ValueError("The scene does not contain the expected placeholder object box geom")
    object_geom.size = object_half_size_m
    return spec.compile()


def _place_target_on_shelf(model, data, mujoco, target_body: int, target_geom: int):  # type: ignore[no-untyped-def]
    shelf_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "placeholder_shelf_deck")
    free_joint = next(
        (
            joint_id
            for joint_id in range(model.njnt)
            if int(model.jnt_bodyid[joint_id]) == target_body
            and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        ),
        -1,
    )
    if shelf_geom < 0 or model.geom_type[shelf_geom] != mujoco.mjtGeom.mjGEOM_BOX:
        raise ValueError("Shelf size override requires the placeholder shelf deck box")
    if free_joint < 0:
        raise ValueError("Placeholder object needs a freejoint to adjust its static shelf pose")
    shelf_top_z = float(model.geom_pos[shelf_geom, 2] + model.geom_size[shelf_geom, 2])
    free_qpos = int(model.jnt_qposadr[free_joint])
    # A 0.5 mm overlap makes the zero-velocity support contact observable to mj_forward.
    data.qpos[free_qpos + 2] = shelf_top_z + float(model.geom_size[target_geom, 2]) - 0.0005
    return shelf_geom, shelf_top_z


def _is_expected_shelf_support_contact(
    model,
    contact,
    target_body: int,
    target_geom: int,
    shelf_geom: int,
    shelf_top_z: float,
) -> bool:  # type: ignore[no-untyped-def]
    geom1, geom2 = int(contact.geom1), int(contact.geom2)
    body1 = int(model.geom_bodyid[geom1])
    body2 = int(model.geom_bodyid[geom2])
    return (
        target_body in (body1, body2)
        and {geom1, geom2} == {target_geom, shelf_geom}
        and abs(float(contact.pos[2]) - shelf_top_z) <= 0.002
        and abs(float(contact.frame[2])) >= 0.95
        and float(contact.dist) >= -0.002
    )


def sample_static_gripper_contacts(
    model,
    data,
    mujoco,
    sample_count: int,
    seed: int,
    local_center_joints_rad: dict[str, float] | None = None,
    local_center_gripper_rad: float | None = None,
    local_joint_sigma_rad: float | None = None,
    local_gripper_sigma_rad: float | None = None,
    rest_target_on_shelf: bool = False,
) -> dict[str, object]:  # type: ignore[no-untyped-def]
    """Sample static model joint states; report geometry observations, not grasps."""
    import numpy as np

    arm_names = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")
    joint_positions = []
    for name in arm_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if (
            joint_id < 0
            or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE
            or not model.jnt_limited[joint_id]
        ):
            raise ValueError(f"Required bounded arm hinge joint is missing: {name}")
        joint_positions.append((name, int(model.jnt_qposadr[joint_id]), model.jnt_range[joint_id].copy()))

    gripper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper")
    mimic_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic")
    target_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "placeholder_pick_object")
    target_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "placeholder_pick_object_geom"
    )
    jaw_bodies = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in ("link6", "link7")
    }
    link6_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    if min(gripper_id, mimic_id, target_body, target_geom, *jaw_bodies) < 0:
        raise ValueError("Required gripper joints, jaw bodies, or placeholder target are missing")
    if (
        model.geom_type[target_geom] != mujoco.mjtGeom.mjGEOM_BOX
        or int(model.geom_bodyid[target_geom]) != target_body
    ):
        raise ValueError("Placeholder target must remain the expected box geom on its freejoint body")
    if not model.jnt_limited[gripper_id] or not model.jnt_limited[mimic_id]:
        raise ValueError("Gripper joints must have bounded model ranges")

    local_values = (
        local_center_joints_rad,
        local_center_gripper_rad,
        local_joint_sigma_rad,
        local_gripper_sigma_rad,
    )
    if any(value is not None for value in local_values) and not all(
        value is not None for value in local_values
    ):
        raise ValueError("Local sampling requires joint center, gripper center, and both sigmas")
    if local_center_joints_rad is not None:
        for name, _, limits in joint_positions:
            center = local_center_joints_rad[name]
            if not float(limits[0]) <= center <= float(limits[1]):
                raise ValueError(f"Local center for {name} is outside the MJCF joint range")
        if not float(model.jnt_range[gripper_id, 0]) <= local_center_gripper_rad <= float(
            model.jnt_range[gripper_id, 1]
        ):
            raise ValueError("Local gripper center is outside the MJCF joint range")
        if local_joint_sigma_rad <= 0 or not math.isfinite(local_joint_sigma_rad):
            raise ValueError("Local joint sigma must be finite and positive")
        if local_gripper_sigma_rad <= 0 or not math.isfinite(local_gripper_sigma_rad):
            raise ValueError("Local gripper sigma must be finite and positive")

    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
    shelf_geom = -1
    shelf_top_z: float | None = None
    if rest_target_on_shelf:
        shelf_geom, shelf_top_z = _place_target_on_shelf(
            model, data, mujoco, target_body, target_geom
        )
    random = np.random.default_rng(seed)
    target_contact_count = 0
    bilateral_contact_count = 0
    jaw_only_no_other_count = 0
    opposing_jaw_contact_count = 0
    shelf_support_sample_count = 0
    jaw_only_candidates = []
    opposing_candidates = []
    for sample_index in range(sample_count):
        sampled_arm = {}
        for name, qpos_address, limits in joint_positions:
            if local_center_joints_rad is None:
                joint_value = random.uniform(float(limits[0]), float(limits[1]))
            else:
                joint_value = float(
                    np.clip(
                        random.normal(local_center_joints_rad[name], local_joint_sigma_rad),
                        float(limits[0]),
                        float(limits[1]),
                    )
                )
            data.qpos[qpos_address] = joint_value
            sampled_arm[name] = joint_value
        if local_center_gripper_rad is None:
            gripper_angle = random.uniform(
                float(model.jnt_range[gripper_id, 0]), float(model.jnt_range[gripper_id, 1])
            )
        else:
            gripper_angle = float(
                np.clip(
                    random.normal(local_center_gripper_rad, local_gripper_sigma_rad),
                    float(model.jnt_range[gripper_id, 0]),
                    float(model.jnt_range[gripper_id, 1]),
                )
            )
        data.qpos[int(model.jnt_qposadr[gripper_id])] = gripper_angle
        data.qpos[int(model.jnt_qposadr[mimic_id])] = -gripper_angle
        mujoco.mj_forward(model, data)

        target_bodies = set()
        target_distances = []
        support_contacts_this_sample = 0
        jaw_normals: dict[int, list[object]] = {body_id: [] for body_id in jaw_bodies}
        jaw_contact_points: dict[int, list[object]] = {body_id: [] for body_id in jaw_bodies}
        non_target_contacts = []
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            body1 = int(model.geom_bodyid[contact.geom1])
            body2 = int(model.geom_bodyid[contact.geom2])
            if target_body in (body1, body2):
                if rest_target_on_shelf and _is_expected_shelf_support_contact(
                    model, contact, target_body, target_geom, shelf_geom, shelf_top_z
                ):
                    support_contacts_this_sample += 1
                    continue
                other_body = body2 if body1 == target_body else body1
                target_bodies.add(other_body)
                target_distances.append(float(contact.dist))
                if other_body in jaw_bodies:
                    # MuJoCo's contact-frame normal points from geom1 to geom2.
                    normal = np.asarray(contact.frame[:3], dtype=float)
                    if body1 != target_body:
                        normal = -normal
                    jaw_normals[other_body].append(normal)
                    jaw_contact_points[other_body].append(np.asarray(contact.pos, dtype=float))
            else:
                non_target_contacts.append((body1, body2))

        if support_contacts_this_sample:
            shelf_support_sample_count += 1
        if target_distances:
            target_contact_count += 1
        if jaw_bodies.issubset(target_bodies):
            bilateral_contact_count += 1
        if target_bodies == jaw_bodies and target_distances and not non_target_contacts:
            jaw_only_no_other_count += 1
            mean_jaw_normals = {}
            for body_id, normals in jaw_normals.items():
                if normals:
                    mean_normal = np.mean(np.stack(normals), axis=0)
                    mean_normal /= max(float(np.linalg.norm(mean_normal)), 1e-12)
                    mean_jaw_normals[
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                    ] = mean_normal
            normal_dot = float(
                np.dot(mean_jaw_normals["link6"], mean_jaw_normals["link7"])
            )
            object_position_world = np.asarray(data.xpos[target_body], dtype=float).copy()
            object_quaternion_world = np.asarray(data.xquat[target_body], dtype=float).copy()
            link6_position_world = np.asarray(data.xpos[link6_body], dtype=float).copy()
            link6_quaternion_world = np.asarray(data.xquat[link6_body], dtype=float).copy()
            object_rotation_world = np.asarray(data.xmat[target_body], dtype=float).reshape(3, 3)
            link6_position_object = object_rotation_world.T @ (
                link6_position_world - object_position_world
            )
            ow, ox, oy, oz = (float(value) for value in object_quaternion_world)
            lw, lx, ly, lz = (float(value) for value in link6_quaternion_world)
            link6_quaternion_object = np.asarray(
                (
                    ow * lw + ox * lx + oy * ly + oz * lz,
                    ow * lx - ox * lw - oy * lz + oz * ly,
                    ow * ly + ox * lz - oy * lw - oz * lx,
                    ow * lz - ox * ly + oy * lx - oz * lw,
                ),
                dtype=float,
            )
            mean_contact_points = {
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id): np.mean(
                    np.stack(points), axis=0
                ).tolist()
                for body_id, points in jaw_contact_points.items()
                if points
            }
            candidate = {
                "sample_index": sample_index,
                "arm_joint_positions_rad": sampled_arm,
                "gripper_rad": float(gripper_angle),
                "gripper_mimic_rad": float(-gripper_angle),
                "target_contact_count": len(target_distances),
                "target_contact_distances_m": target_distances,
                "worst_absolute_target_contact_distance_m": max(
                    abs(distance) for distance in target_distances
                ),
                "mean_target_to_jaw_contact_normals": {
                    name: normal.tolist() for name, normal in mean_jaw_normals.items()
                },
                "mean_target_to_jaw_contact_positions_world_m": mean_contact_points,
                "placeholder_object_pose_world": {
                    "position_m": object_position_world.tolist(),
                    "quaternion_wxyz": object_quaternion_world.tolist(),
                },
                "link6_body_pose_world": {
                    "position_m": link6_position_world.tolist(),
                    "quaternion_wxyz": link6_quaternion_world.tolist(),
                },
                "link6_body_pose_in_placeholder_object_frame": {
                    "position_m": link6_position_object.tolist(),
                    "quaternion_wxyz": link6_quaternion_object.tolist(),
                },
                "pose_scope": (
                    "static MuJoCo body origins; link6 is not a measured physical TCP; "
                    "contact is not grasp verification"
                ),
                "mean_jaw_normal_dot": normal_dot,
                "opposing_contact_normals": normal_dot <= -0.5,
            }
            jaw_only_candidates.append(candidate)
            if normal_dot <= -0.5:
                opposing_jaw_contact_count += 1
                opposing_candidates.append(candidate)

    jaw_only_candidates.sort(
        key=lambda candidate: (
            candidate["worst_absolute_target_contact_distance_m"],
            candidate["target_contact_count"],
        )
    )
    opposing_candidates.sort(
        key=lambda candidate: (
            candidate["worst_absolute_target_contact_distance_m"],
            candidate["target_contact_count"],
        )
    )
    object_pose_sampling = (
        "placeholder freejoint x/y/orientation retained; z adjusted to shelf-support contact"
        if rest_target_on_shelf
        else "placeholder freejoint held at home keyframe"
    )
    if local_center_joints_rad is None:
        sampling = f"independent uniform qpos within model ranges; {object_pose_sampling}"
    else:
        sampling = (
            "Gaussian qpos perturbation around supplied simulation-only center, clipped to MJCF ranges; "
            f"{object_pose_sampling}"
        )
    result = {
        "samples": sample_count,
        "seed": seed,
        "sampling": sampling,
        "collision_check": "mj_forward only; no simulation stepping or actuator target",
        "placeholder_object_half_size_m": model.geom_size[target_geom, :3].tolist(),
        "target_pose_assumption": (
            "freejoint x/y/orientation retained; box bottom overlaps placeholder shelf top by 0.5 mm for static contact observation"
            if rest_target_on_shelf
            else "placeholder object freejoint held at home keyframe pose"
        ),
        "target_contact_samples": target_contact_count,
        "expected_shelf_support_contact_samples": shelf_support_sample_count,
        "both_gripper_bodies_touch_target_samples": bilateral_contact_count,
        "both_gripper_bodies_only_without_other_contacts": jaw_only_no_other_count,
        "opposing_jaw_normal_samples": opposing_jaw_contact_count,
        "opposition_criterion": "dot(mean target-to-jaw contact normals) <= -0.5; geometry heuristic only",
        "best_jaw_only_contact_samples": jaw_only_candidates[:3],
        "best_opposing_jaw_contact_samples": opposing_candidates[:3],
        "verified_grasp": False,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
    }
    if local_center_joints_rad is not None:
        result["local_search"] = {
            "center_joint_positions_rad": local_center_joints_rad,
            "center_gripper_rad": local_center_gripper_rad,
            "joint_sigma_rad": local_joint_sigma_rad,
            "gripper_sigma_rad": local_gripper_sigma_rad,
            "range_handling": "sampled values clipped to MJCF joint ranges",
        }
    return result


def inspect_home_to_joint_path(
    model,
    data,
    mujoco,
    goal_joints_rad: dict[str, float],
    gripper_rad: float,
    path_segments: int,
    rest_target_on_shelf: bool,
    joint_order: tuple[str, ...] | None = None,
) -> dict[str, object]:  # type: ignore[no-untyped-def]
    """Inspect discrete joint-linear MuJoCo qpos samples; not a trajectory planner."""
    import numpy as np

    arm_names = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")
    qpos_addresses = {}
    for name in arm_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if (
            joint_id < 0
            or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE
            or not model.jnt_limited[joint_id]
        ):
            raise ValueError(f"Required bounded arm hinge joint is missing: {name}")
        if name not in goal_joints_rad:
            raise ValueError(f"Path goal is missing {name}")
        lower, upper = model.jnt_range[joint_id]
        if not float(lower) <= goal_joints_rad[name] <= float(upper):
            raise ValueError(f"Path goal for {name} is outside the MJCF range")
        qpos_addresses[name] = int(model.jnt_qposadr[joint_id])

    gripper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper")
    mimic_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic")
    target_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "placeholder_pick_object")
    target_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "placeholder_pick_object_geom"
    )
    jaw_bodies = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in ("link6", "link7")
    }
    if min(gripper_id, mimic_id, target_body, target_geom, *jaw_bodies) < 0:
        raise ValueError("Required gripper, jaw, or placeholder object model entries are missing")
    if not model.jnt_limited[gripper_id] or not model.jnt_limited[mimic_id]:
        raise ValueError("Gripper joints must have bounded model ranges")
    if not float(model.jnt_range[gripper_id, 0]) <= gripper_rad <= float(
        model.jnt_range[gripper_id, 1]
    ):
        raise ValueError("Path gripper goal is outside the MJCF range")
    if not float(model.jnt_range[mimic_id, 0]) <= -gripper_rad <= float(
        model.jnt_range[mimic_id, 1]
    ):
        raise ValueError("Mimic gripper goal is outside the MJCF range")

    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key < 0:
        raise ValueError("The scene needs a home keyframe to audit a home-to-goal path")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    shelf_geom = -1
    shelf_top_z = None
    if rest_target_on_shelf:
        shelf_geom, shelf_top_z = _place_target_on_shelf(
            model, data, mujoco, target_body, target_geom
        )
    start = np.array([data.qpos[qpos_addresses[name]] for name in arm_names])
    goal = np.array([goal_joints_rad[name] for name in arm_names])
    qpos_samples = [start.copy()]
    if joint_order is None:
        qpos_samples.extend(
            start + (goal - start) * (segment / path_segments)
            for segment in range(1, path_segments + 1)
        )
        interpolation = "simultaneous_joint_linear"
    else:
        current = start.copy()
        for name in joint_order:
            joint_index = arm_names.index(name)
            joint_start = float(current[joint_index])
            joint_goal = float(goal[joint_index])
            for segment in range(1, path_segments + 1):
                current[joint_index] = joint_start + (joint_goal - joint_start) * (
                    segment / path_segments
                )
                qpos_samples.append(current.copy())
        interpolation = "one_joint_at_a_time_in_supplied_order"
    total_segments = len(qpos_samples) - 1
    gripper_qpos = int(model.jnt_qposadr[gripper_id])
    mimic_qpos = int(model.jnt_qposadr[mimic_id])
    first_target_contact = None
    first_other_collision = None
    endpoint_contacts = []
    target_contact_samples = 0
    other_collision_samples = 0
    support_contact_count = 0
    max_target_penetration_m = 0.0
    max_other_penetration_m = 0.0
    endpoint_jaw_normals: dict[int, list[object]] = {body_id: [] for body_id in jaw_bodies}

    for sample_index, arm_sample in enumerate(qpos_samples):
        fraction = sample_index / total_segments
        for name, value in zip(arm_names, arm_sample):
            data.qpos[qpos_addresses[name]] = float(value)
        data.qpos[gripper_qpos] = gripper_rad
        data.qpos[mimic_qpos] = -gripper_rad
        mujoco.mj_forward(model, data)
        target_events = []
        other_events = []
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            body1, body2 = int(model.geom_bodyid[geom1]), int(model.geom_bodyid[geom2])
            if rest_target_on_shelf and _is_expected_shelf_support_contact(
                model, contact, target_body, target_geom, shelf_geom, shelf_top_z
            ):
                support_contact_count += 1
                continue
            record = {
                "geom1": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1),
                "geom2": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2),
                "body1": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body1),
                "body2": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body2),
                "distance_m": float(contact.dist),
            }
            if target_body in (body1, body2):
                target_events.append(record)
                max_target_penetration_m = max(max_target_penetration_m, -float(contact.dist))
                other_body = body2 if body1 == target_body else body1
                if sample_index == total_segments and other_body in jaw_bodies:
                    normal = np.asarray(contact.frame[:3], dtype=float)
                    if body1 != target_body:
                        normal = -normal
                    endpoint_jaw_normals[other_body].append(normal)
            else:
                other_events.append(record)
                max_other_penetration_m = max(max_other_penetration_m, -float(contact.dist))
        if target_events:
            target_contact_samples += 1
            if first_target_contact is None:
                first_target_contact = {
                    "sample_index": sample_index,
                    "path_fraction": fraction,
                    "contacts": target_events[:4],
                }
        if other_events:
            other_collision_samples += 1
            if first_other_collision is None:
                first_other_collision = {
                    "sample_index": sample_index,
                    "path_fraction": fraction,
                    "contacts": other_events[:4],
                }
        if sample_index == total_segments:
            endpoint_contacts = target_events + other_events

    endpoint_normal_dot = None
    if all(endpoint_jaw_normals.values()):
        mean_normals = []
        for body_id in (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in ("link6", "link7")):
            mean_normal = np.mean(np.stack(endpoint_jaw_normals[body_id]), axis=0)
            mean_normal /= max(float(np.linalg.norm(mean_normal)), 1e-12)
            mean_normals.append(mean_normal)
        endpoint_normal_dot = float(np.dot(mean_normals[0], mean_normals[1]))

    return {
        "path": "home_keyframe to supplied joint goal; joint-linear interpolation",
        "interpolation": interpolation,
        "path_joint_order": list(joint_order) if joint_order is not None else None,
        "segments_per_joint": path_segments if joint_order is not None else None,
        "path_segments": total_segments,
        "sampled_qpos_points": len(qpos_samples),
        "arm_joint_start_rad": {name: float(value) for name, value in zip(arm_names, start)},
        "arm_joint_goal_rad": goal_joints_rad,
        "gripper_rad_held_constant": gripper_rad,
        "expected_shelf_support_contact_count": support_contact_count,
        "target_contact_sample_count": target_contact_samples,
        "other_environment_or_self_collision_sample_count": other_collision_samples,
        "first_target_contact": first_target_contact,
        "first_other_collision": first_other_collision,
        "endpoint_non_support_contacts": endpoint_contacts,
        "endpoint_jaw_mean_normal_dot": endpoint_normal_dot,
        "endpoint_opposing_normal_heuristic": (
            endpoint_normal_dot is not None and endpoint_normal_dot <= -0.5
        ),
        "max_target_penetration_m": max_target_penetration_m,
        "max_other_collision_penetration_m": max_other_penetration_m,
        "no_non_support_object_contact_before_endpoint": (
            first_target_contact is None
            or first_target_contact["sample_index"] == total_segments
        ),
        "no_sampled_environment_or_self_collision": other_collision_samples == 0,
        "continuous_collision_checked": False,
        "trajectory_planned": False,
        "verified_grasp": False,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="Open the MuJoCo viewer for this simulation scene")
    parser.add_argument(
        "--gripper-mesh-distance",
        action="store_true",
        help="Report static collision-mesh signed distances across modeled gripper angles",
    )
    parser.add_argument(
        "--sample-static-gripper-contacts",
        type=int,
        default=0,
        metavar="COUNT",
        help="Sample bounded static arm/gripper qpos states against the placeholder object (1–1000000)",
    )
    parser.add_argument(
        "--local-center-joints",
        type=_parse_joint_center,
        metavar="JOINTS",
        help="Gaussian local-search center; provide each of Joint1–Joint5 once as NAME=RAD",
    )
    parser.add_argument(
        "--local-center-gripper-rad",
        type=float,
        metavar="RADIANS",
        help="Gripper center in MuJoCo radians for local Gaussian sampling",
    )
    parser.add_argument(
        "--local-joint-sigma-rad",
        type=float,
        metavar="RADIANS",
        help="Gaussian standard deviation shared by the five arm joints",
    )
    parser.add_argument(
        "--local-gripper-sigma-rad",
        type=float,
        metavar="RADIANS",
        help="Gaussian standard deviation for Gripper",
    )
    parser.add_argument(
        "--placeholder-object-half-size-m",
        type=_parse_box_half_size,
        metavar="X,Y,Z",
        help="recompile the scene with these placeholder box half-sizes in meters for simulation-only inspection",
    )
    parser.add_argument(
        "--audit-home-path",
        action="store_true",
        help="inspect discrete joint-linear qpos samples from the home keyframe to a supplied goal",
    )
    parser.add_argument(
        "--path-goal-joints",
        type=_parse_joint_center,
        metavar="JOINTS",
        help="path endpoint for all five arm joints, as NAME=RAD pairs",
    )
    parser.add_argument(
        "--path-gripper-rad",
        type=float,
        metavar="RADIANS",
        help="hold this MuJoCo gripper angle across all path samples",
    )
    parser.add_argument(
        "--path-segments",
        type=int,
        default=100,
        metavar="COUNT",
        help="number of discrete joint-linear path segments (2–2000)",
    )
    parser.add_argument(
        "--path-joint-order",
        type=_parse_joint_order,
        metavar="ORDER",
        help="move one arm joint at a time in this order instead of moving all joints simultaneously",
    )
    parser.add_argument("--random-seed", type=int, default=20261031)
    args = parser.parse_args()
    if not 0 <= args.sample_static_gripper_contacts <= 1_000_000:
        parser.error("--sample-static-gripper-contacts must be between 0 and 1000000")
    local_options = (
        args.local_center_joints,
        args.local_center_gripper_rad,
        args.local_joint_sigma_rad,
        args.local_gripper_sigma_rad,
    )
    if any(value is not None for value in local_options) and not all(
        value is not None for value in local_options
    ):
        parser.error(
            "local sampling requires --local-center-joints, --local-center-gripper-rad, "
            "--local-joint-sigma-rad, and --local-gripper-sigma-rad together"
        )
    if any(value is not None for value in local_options) and not args.sample_static_gripper_contacts:
        parser.error("local sampling requires --sample-static-gripper-contacts COUNT")
    path_options = (args.path_goal_joints, args.path_gripper_rad)
    if args.audit_home_path and not all(value is not None for value in path_options):
        parser.error("--audit-home-path requires --path-goal-joints and --path-gripper-rad")
    if not args.audit_home_path and any(value is not None for value in path_options):
        parser.error("--path-goal-joints and --path-gripper-rad require --audit-home-path")
    if not args.audit_home_path and args.path_joint_order is not None:
        parser.error("--path-joint-order requires --audit-home-path")
    if not 2 <= args.path_segments <= 2000:
        parser.error("--path-segments must be between 2 and 2000")

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
        model = load_scene_model(mujoco, args.placeholder_object_half_size_m)
    except (ValueError, RuntimeError) as exc:
        print(f"Could not load requested simulation scene variant: {exc}", file=sys.stderr)
        return 2
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)

    print(f"MuJoCo {mujoco.__version__}")
    print(f"Scene: {SCENE_PATH.relative_to(REPO_ROOT)}")
    print(f"Model dimensions: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    if args.placeholder_object_half_size_m is not None:
        print(
            "Placeholder object box half-size override m="
            f"{args.placeholder_object_half_size_m} (recompiled model; static contact survey only)"
        )
    print("Joint mapping (semantic only; no real-unit conversion applied):")
    for action_name, sim_joint_name in JOINT_MAP:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, sim_joint_name)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, sim_joint_name)
        if joint_id < 0 or actuator_id < 0:
            print(f"  {action_name} -> {sim_joint_name}: model entry missing", file=sys.stderr)
            return 2
        limits = model.jnt_range[joint_id] if model.jnt_limited[joint_id] else None
        print(f"  {action_name} -> {sim_joint_name}: simulation range rad={limits}")

    if args.gripper_mesh_distance:
        try:
            samples = inspect_gripper_mesh_distance(model, data, mujoco)
        except ValueError as exc:
            print(f"Could not inspect modeled gripper geometry: {exc}", file=sys.stderr)
            return 2
        print("Static gripper collision-mesh distance (not jaw opening width):")
        for sample in samples:
            print(
                "  gripper={:.0f}° mimic={:.0f}° distance={:.6f} m contacts={}".format(
                    sample["gripper_angle_degrees"],
                    sample["gripper_mimic_angle_degrees"],
                    sample["signed_geom_distance_m"],
                    sample["contact_count_after_mj_forward"],
                )
            )
        print("  No object was placed between the jaws; no simulation step was performed.")

    if args.sample_static_gripper_contacts:
        try:
            result = sample_static_gripper_contacts(
                model,
                data,
                mujoco,
                args.sample_static_gripper_contacts,
                args.random_seed,
                local_center_joints_rad=args.local_center_joints,
                local_center_gripper_rad=args.local_center_gripper_rad,
                local_joint_sigma_rad=args.local_joint_sigma_rad,
                local_gripper_sigma_rad=args.local_gripper_sigma_rad,
                rest_target_on_shelf=args.placeholder_object_half_size_m is not None,
            )
        except ValueError as exc:
            print(f"Could not sample modeled gripper contacts: {exc}", file=sys.stderr)
            return 2
        print("Static gripper/object qpos sampling; contact geometry is not grasp verification:")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if result["opposing_jaw_normal_samples"] == 0:
            print("  No sampled jaw-only contacts met the opposing-normal geometry heuristic.")

    if args.audit_home_path:
        try:
            result = inspect_home_to_joint_path(
                model,
                data,
                mujoco,
                args.path_goal_joints,
                args.path_gripper_rad,
                args.path_segments,
                rest_target_on_shelf=args.placeholder_object_half_size_m is not None,
                joint_order=args.path_joint_order,
            )
        except ValueError as exc:
            print(f"Could not inspect the requested joint path: {exc}", file=sys.stderr)
            return 2
        print("Static joint-linear path inspection; not trajectory planning or grasp verification:")
        print(json.dumps(result, ensure_ascii=False, indent=2))

    print("Simulation only. No camera or robot device was opened; no physical command was sent.")
    if args.viewer:
        from mujoco import viewer

        viewer.launch(model, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
