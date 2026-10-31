#!/usr/bin/env python3
"""Audit one explicit OMX pick candidate in the placeholder MuJoCo scene only."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import uuid
from datetime import datetime, timezone

import mujoco
import numpy as np
from calibration_artifacts import install_file_exclusive
from mujoco_model_sources import model_source_artifacts


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"
ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")
MAX_IK_CANDIDATE_BYTES = 1_000_000


def parse_joint_goals(value: str) -> dict[str, float]:
    result: dict[str, float] = {}
    try:
        for field in value.split(","):
            name, raw = field.split("=", maxsplit=1)
            name = name.strip()
            number = float(raw.strip())
            if name not in ARM_JOINTS or name in result or not math.isfinite(number):
                raise ValueError
            result[name] = number
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "provide each arm joint once as Joint1=RAD,...,Joint5=RAD"
        ) from exc
    if set(result) != set(ARM_JOINTS):
        raise argparse.ArgumentTypeError("all five arm joints are required")
    return result


def name_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    identifier = mujoco.mj_name2id(model, object_type, name)
    if identifier < 0:
        raise ValueError(f"required MuJoCo model item is missing: {name}")
    return int(identifier)


def unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    value: dict[str, object] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"IK candidate contains a duplicate JSON key: {key}")
        value[key] = item
    return value


def is_finite_json_number(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(float(value))
    except OverflowError:
        return False


def quaternion_rotation_matrix_wxyz(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(component) for component in quaternion)
    return np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        ),
        dtype=float,
    )


def relative_quaternion_wxyz(parent: np.ndarray, child: np.ndarray) -> np.ndarray:
    pw, px, py, pz = (float(component) for component in parent)
    cw, cx, cy, cz = (float(component) for component in child)
    return np.asarray(
        (
            pw * cw + px * cx + py * cy + pz * cz,
            pw * cx - px * cw - py * cz + pz * cy,
            pw * cy + px * cz - py * cw - pz * cx,
            pw * cz - px * cy + py * cx - pz * cw,
        ),
        dtype=float,
    )


def load_ik_candidate(
    path: Path,
    *,
    expected_model_sources: list[dict[str, object]],
    expected_mujoco_version: str,
) -> tuple[dict[str, object], str]:
    """Load only a matching, converged, discrete-contact-free offline IK candidate."""
    if path.is_symlink() or not path.is_file():
        raise ValueError("IK candidate must be a regular non-symlink JSON file")
    if path.stat().st_size > MAX_IK_CANDIDATE_BYTES:
        raise ValueError("IK candidate exceeds the 1 MB limit")
    with path.open("rb") as candidate_file:
        raw = candidate_file.read(MAX_IK_CANDIDATE_BYTES + 1)
    if len(raw) > MAX_IK_CANDIDATE_BYTES:
        raise ValueError("IK candidate exceeds the 1 MB limit")
    try:
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_json_object)
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ValueError("IK candidate must be valid UTF-8 JSON with unique keys") from exc
    if not isinstance(value, dict):
        raise ValueError("IK candidate must contain a JSON object")
    if (
        isinstance(value.get("schema_version"), bool)
        or not isinstance(value.get("schema_version"), int)
        or value.get("schema_version") != 2
        or value.get("kind") != "mujoco_ik_candidate"
        or value.get("simulation_only") is not True
        or value.get("usable_for_robot_control") is not False
        or value.get("arm_command_sent") is not False
        or value.get("model") != "simulation/mujoco_omx/robotis_omx/shelf_pick_scene.xml"
        or value.get("pose_frame") not in ("link6", "tcp")
    ):
        raise ValueError("IK candidate is not a version 2 MuJoCo-only OMX candidate")
    if value.get("mujoco_version") != expected_mujoco_version:
        raise ValueError("IK candidate MuJoCo version does not match the current runtime")
    if value.get("model_source_files") != expected_model_sources:
        raise ValueError("IK candidate model source hashes do not match the current scene")
    if (
        value.get("converged") is not True
        or value.get("position_converged") is not True
        or value.get("pose_converged") is not True
        or value.get("candidate_status")
        not in (
            "POSITION_CONVERGED_ONLY; PATH_AND_GRASP_UNVERIFIED",
            "POSE_CONVERGED_ONLY; PATH_AND_GRASP_UNVERIFIED",
        )
    ):
        raise ValueError("IK candidate must be converged with no recorded pose/path contacts")
    contacts = value.get("contacts")
    contact_keys = (
        "start_pose",
        "final_pose",
        "final_shelf",
        "final_target_object",
        "joint_linear_path",
    )
    if not isinstance(contacts, dict) or any(
        not isinstance(contacts.get(key), list) or contacts[key] for key in contact_keys
    ):
        raise ValueError("IK candidate must have empty start/final/path contact lists")
    path_sampling = value.get("path_sampling")
    segments = path_sampling.get("segments") if isinstance(path_sampling, dict) else None
    intermediate_samples = (
        path_sampling.get("intermediate_samples") if isinstance(path_sampling, dict) else None
    )
    max_joint_step = (
        path_sampling.get("max_joint_step_rad") if isinstance(path_sampling, dict) else None
    )
    requested_max_joint_step = (
        path_sampling.get("requested_max_joint_step_rad")
        if isinstance(path_sampling, dict)
        else None
    )
    if (
        not isinstance(path_sampling, dict)
        or path_sampling.get("method") != "discrete_joint_linear_interpolation"
        or path_sampling.get("motion_planned") is not False
        or path_sampling.get("continuous_collision_checked") is not False
        or isinstance(segments, bool)
        or not isinstance(segments, int)
        or segments < 2
        or isinstance(intermediate_samples, bool)
        or not isinstance(intermediate_samples, int)
        or intermediate_samples != segments - 1
        or not is_finite_json_number(max_joint_step)
        or max_joint_step < 0
        or not is_finite_json_number(requested_max_joint_step)
        or not 0.005 <= requested_max_joint_step <= 0.5
        or path_sampling.get("first_contact_sample") is not None
    ):
        raise ValueError("IK candidate path-sampling status is unsupported")
    joints = value.get("joint_candidate_rad")
    if not isinstance(joints, dict) or set(joints) != set(ARM_JOINTS):
        raise ValueError("IK candidate must include all five arm joints")
    for name, target in joints.items():
        if isinstance(target, bool) or not isinstance(target, (int, float)):
            raise ValueError(f"IK candidate has an invalid value for {name}")
        try:
            if not math.isfinite(float(target)):
                raise ValueError(f"IK candidate has a non-finite value for {name}")
        except OverflowError as exc:
            raise ValueError(f"IK candidate has an invalid value for {name}") from exc
    return value, hashlib.sha256(raw).hexdigest()


def world_orientation_error(
    target_wxyz: np.ndarray, current_wxyz: np.ndarray
) -> np.ndarray:
    """Shortest world-frame rotation vector from current to target quaternion."""
    tw, tx, ty, tz = (float(value) for value in target_wxyz)
    cw, cx, cy, cz = (float(value) for value in current_wxyz)
    error_w = tw * cw + tx * cx + ty * cy + tz * cz
    error_xyz = np.asarray(
        (
            cw * tx - tw * cx - ty * cz + tz * cy,
            cw * ty - tw * cy - tz * cx + tx * cz,
            cw * tz - tw * cz - tx * cy + ty * cx,
        ),
        dtype=float,
    )
    if error_w < 0:
        error_w = -error_w
        error_xyz = -error_xyz
    vector_norm = float(np.linalg.norm(error_xyz))
    if vector_norm < 1e-12:
        return 2.0 * error_xyz
    angle = 2.0 * math.atan2(vector_norm, max(0.0, error_w))
    return error_xyz * (angle / vector_norm)


def publish_audit_artifact(payload: dict[str, object]) -> Path:
    """Atomically save one non-overwriting audit record under the managed directory."""
    control_directory = REPO_ROOT / ".control-station"
    output_directory = control_directory / "mujoco-dynamics-audits"
    for directory in (control_directory, output_directory):
        if directory.is_symlink():
            raise ValueError("MuJoCo audit artifact directory cannot be a symbolic link")
    output_directory.mkdir(parents=True, exist_ok=True)
    if output_directory.resolve(strict=True) != output_directory:
        raise ValueError("MuJoCo audit artifact directory resolves outside its managed path")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    audit_id = str(payload["audit_id"])
    output_path = output_directory / f"audit-{stamp}-{audit_id[:8]}.json"
    payload["artifact_path"] = output_path.relative_to(REPO_ROOT).as_posix()
    serialized = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".audit-", suffix=".tmp", dir=output_directory
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(serialized)
            output.flush()
            os.fsync(output.fileno())
        install_file_exclusive(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--goal-joints", type=parse_joint_goals)
    target_group.add_argument(
        "--candidate-json",
        type=Path,
        help="hash-verified contact-free IK candidate JSON from solve_mujoco_omx_position_ik.py",
    )
    parser.add_argument("--joint-order", default="Joint3,Joint1,Joint4,Joint5,Joint2")
    parser.add_argument("--gripper-open-rad", type=float, required=True)
    parser.add_argument("--gripper-close-rad", type=float, required=True)
    parser.add_argument("--approach-segments-per-joint", type=int, default=1000)
    parser.add_argument("--close-steps", type=int, default=1000)
    parser.add_argument("--settle-steps", type=int, default=1000)
    parser.add_argument("--lift-height-m", type=float, default=0.03)
    parser.add_argument("--lift-steps", type=int, default=1500)
    parser.add_argument(
        "--lift-orientation-scale-m-per-rad",
        type=float,
        default=0.0,
        help="weight link6 orientation error against position error; zero selects position-only IK",
    )
    parser.add_argument(
        "--save-artifact",
        action="store_true",
        help="save a non-overwriting JSON record under .control-station/mujoco-dynamics-audits/",
    )
    args = parser.parse_args()

    order = tuple(name.strip() for name in args.joint_order.split(","))
    if len(order) != len(ARM_JOINTS) or set(order) != set(ARM_JOINTS):
        parser.error("--joint-order must list Joint1–Joint5 exactly once")
    for name, value, lower in (
        ("--gripper-open-rad", args.gripper_open_rad, 0),
        ("--gripper-close-rad", args.gripper_close_rad, 0),
        ("--lift-height-m", args.lift_height_m, 0),
        ("--lift-orientation-scale-m-per-rad", args.lift_orientation_scale_m_per_rad, 0),
    ):
        if not math.isfinite(value) or value < lower:
            parser.error(f"{name} must be a finite non-negative value")
    for name in ("approach_segments_per_joint", "close_steps", "settle_steps", "lift_steps"):
        if getattr(args, name) < 1:
            parser.error(f"--{name.replace('_', '-')} must be positive")

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    home = name_id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home)

    joint_ids = {
        name: name_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINTS
    }
    qpos_ids = {name: int(model.jnt_qposadr[joint_ids[name]]) for name in ARM_JOINTS}
    dof_ids = np.asarray([int(model.jnt_dofadr[joint_ids[name]]) for name in ARM_JOINTS])
    model_sources = model_source_artifacts()
    candidate_record: dict[str, object] | None = None
    candidate_sha256: str | None = None
    if args.candidate_json is not None:
        try:
            candidate_record, candidate_sha256 = load_ik_candidate(
                args.candidate_json,
                expected_model_sources=model_sources,
                expected_mujoco_version=str(mujoco.__version__),
            )
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
        candidate_joints = candidate_record["joint_candidate_rad"]
        assert isinstance(candidate_joints, dict)
        goal_joints = {name: float(candidate_joints[name]) for name in ARM_JOINTS}
    else:
        goal_joints = args.goal_joints
    assert goal_joints is not None
    actuator_ids = {
        name: name_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in (*ARM_JOINTS, "Gripper")
    }
    for name, target in goal_joints.items():
        if model.jnt_limited[joint_ids[name]] and not (
            model.jnt_range[joint_ids[name], 0] <= target <= model.jnt_range[joint_ids[name], 1]
        ):
            parser.error(f"{name} goal is outside the MuJoCo model range")

    gripper_id = name_id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper")
    gripper_qpos = int(model.jnt_qposadr[gripper_id])
    gripper_mimic_id = name_id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic")
    gripper_mimic_qpos = int(model.jnt_qposadr[gripper_mimic_id])
    gripper_actuator = actuator_ids["Gripper"]
    if model.actuator_ctrllimited[gripper_actuator]:
        low, high = model.actuator_ctrlrange[gripper_actuator]
        for label, value in (
            ("--gripper-open-rad", args.gripper_open_rad),
            ("--gripper-close-rad", args.gripper_close_rad),
        ):
            if not low <= value <= high:
                parser.error(f"{label} is outside the MuJoCo actuator control range")
    object_body = name_id(model, mujoco.mjtObj.mjOBJ_BODY, "placeholder_pick_object")
    object_body_name = "placeholder_pick_object"
    object_geom = name_id(model, mujoco.mjtObj.mjOBJ_GEOM, "placeholder_pick_object_geom")
    shelf_geom = name_id(model, mujoco.mjtObj.mjOBJ_GEOM, "placeholder_shelf_deck")
    free_joint = next(
        (
            joint_id
            for joint_id in range(model.njnt)
            if int(model.jnt_bodyid[joint_id]) == object_body
            and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        ),
        -1,
    )
    if free_joint < 0 or model.geom_type[shelf_geom] != mujoco.mjtGeom.mjGEOM_BOX:
        raise ValueError("placeholder object freejoint or shelf box is missing")
    object_qpos = int(model.jnt_qposadr[free_joint])
    shelf_top_z = float(model.geom_pos[shelf_geom, 2] + model.geom_size[shelf_geom, 2])
    data.qpos[object_qpos + 2] = (
        shelf_top_z + float(model.geom_size[object_geom, 2]) - 0.0005
    )
    mujoco.mj_forward(model, data)
    placeholder_object_center_world = data.qpos[object_qpos : object_qpos + 3].copy()

    contact_metrics: dict[str, dict[str, object]] = {}

    def observe_contacts(stage: str) -> None:
        metrics = contact_metrics.setdefault(
            stage,
            {
                "sampled_steps": 0,
                "object_shelf_support_steps": 0,
                "object_jaw_contact_steps": 0,
                "both_jaw_contact_steps": 0,
                "object_other_contact_steps": 0,
                "other_robot_or_environment_contact_steps": 0,
                "contact_pair_contact_counts": Counter(),
                "jaw_normal_dots": [],
                "max_object_jaw_penetration_m": 0.0,
                "jaw_contact_force_observations": {"link6": 0, "link7": 0},
                "jaw_normal_force_sum_n": {"link6": 0.0, "link7": 0.0},
                "jaw_normal_force_max_n": {"link6": 0.0, "link7": 0.0},
                "jaw_tangential_force_sum_n": {"link6": 0.0, "link7": 0.0},
                "jaw_tangential_force_max_n": {"link6": 0.0, "link7": 0.0},
            },
        )
        metrics["sampled_steps"] += 1
        target_contacts: set[str] = set()
        categories: set[str] = set()
        jaw_normals: dict[str, list[np.ndarray]] = {"link6": [], "link7": []}
        pair_counts = metrics["contact_pair_contact_counts"]
        assert isinstance(pair_counts, Counter)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            body_names = tuple(
                mujoco.mj_id2name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.geom_bodyid[g]),
                )
                for g in geom_ids
            )
            geom_names = tuple(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                or str(geom_id)
                for geom_id in geom_ids
            )
            pair = " <-> ".join(
                f"{body or 'unknown'}/{geom}" for body, geom in zip(body_names, geom_names)
            )
            pair_counts[pair] += 1
            if object_body_name in body_names:
                other_body = next(
                    (body for body in body_names if body != object_body_name), "unknown"
                )
                if shelf_geom in geom_ids:
                    categories.add("object_shelf_support")
                elif other_body in ("link6", "link7"):
                    categories.add("object_jaw")
                    target_contacts.add(other_body)
                    normal = np.asarray(contact.frame[:3], dtype=float)
                    if body_names[0] != object_body_name:
                        normal = -normal
                    jaw_normals[other_body].append(normal)
                    metrics["max_object_jaw_penetration_m"] = max(
                        float(metrics["max_object_jaw_penetration_m"]),
                        max(0.0, -float(contact.dist)),
                    )
                    contact_force = np.zeros(6, dtype=float)
                    mujoco.mj_contactForce(model, data, contact_index, contact_force)
                    normal_force_n = abs(float(contact_force[0]))
                    tangential_force_n = float(np.linalg.norm(contact_force[1:3]))
                    metrics["jaw_contact_force_observations"][other_body] += 1
                    metrics["jaw_normal_force_sum_n"][other_body] += normal_force_n
                    metrics["jaw_normal_force_max_n"][other_body] = max(
                        metrics["jaw_normal_force_max_n"][other_body], normal_force_n
                    )
                    metrics["jaw_tangential_force_sum_n"][other_body] += tangential_force_n
                    metrics["jaw_tangential_force_max_n"][other_body] = max(
                        metrics["jaw_tangential_force_max_n"][other_body], tangential_force_n
                    )
                else:
                    categories.add("object_other")
            else:
                categories.add("other_robot_or_environment")
        if "object_shelf_support" in categories:
            metrics["object_shelf_support_steps"] += 1
        if "object_jaw" in categories:
            metrics["object_jaw_contact_steps"] += 1
        if target_contacts == {"link6", "link7"}:
            metrics["both_jaw_contact_steps"] += 1
            mean_normals = []
            for jaw in ("link6", "link7"):
                mean_normal = np.mean(np.stack(jaw_normals[jaw]), axis=0)
                mean_normals.append(mean_normal / max(float(np.linalg.norm(mean_normal)), 1e-12))
            metrics["jaw_normal_dots"].append(float(np.dot(mean_normals[0], mean_normals[1])))
        if "object_other" in categories:
            metrics["object_other_contact_steps"] += 1
        if "other_robot_or_environment" in categories:
            metrics["other_robot_or_environment_contact_steps"] += 1

    def contact_metrics_json() -> dict[str, dict[str, object]]:
        output: dict[str, dict[str, object]] = {}
        for stage, metrics in contact_metrics.items():
            pair_counts = metrics["contact_pair_contact_counts"]
            assert isinstance(pair_counts, Counter)
            jaw_normal_dots = metrics["jaw_normal_dots"]
            assert isinstance(jaw_normal_dots, list)
            jaw_force_summary = {}
            for jaw in ("link6", "link7"):
                observations = metrics["jaw_contact_force_observations"][jaw]
                jaw_force_summary[jaw] = {
                    "contact_observations": observations,
                    "mean_normal_force_n_per_contact_observation": (
                        metrics["jaw_normal_force_sum_n"][jaw] / observations
                        if observations
                        else None
                    ),
                    "max_normal_force_n": (
                        metrics["jaw_normal_force_max_n"][jaw] if observations else None
                    ),
                    "mean_tangential_force_n_per_contact_observation": (
                        metrics["jaw_tangential_force_sum_n"][jaw] / observations
                        if observations
                        else None
                    ),
                    "max_tangential_force_n": (
                        metrics["jaw_tangential_force_max_n"][jaw] if observations else None
                    ),
                }
            output[stage] = {
                **{
                    key: value
                    for key, value in metrics.items()
                    if key not in ("contact_pair_contact_counts", "jaw_normal_dots")
                },
                "bilateral_jaw_normal_dot_samples": len(jaw_normal_dots),
                "bilateral_jaw_normal_dot_min": min(jaw_normal_dots) if jaw_normal_dots else None,
                "bilateral_jaw_normal_dot_mean": (
                    float(np.mean(jaw_normal_dots)) if jaw_normal_dots else None
                ),
                "bilateral_jaw_normal_dot_max": max(jaw_normal_dots) if jaw_normal_dots else None,
                "opposing_jaw_normal_steps": sum(dot <= -0.5 for dot in jaw_normal_dots),
                "jaw_contact_force_summary": jaw_force_summary,
                "contact_pair_contact_count": dict(sorted(pair_counts.items())),
            }
        return output

    def set_targets(arm_targets: dict[str, float], gripper_target: float) -> None:
        for joint_name, target in arm_targets.items():
            data.ctrl[actuator_ids[joint_name]] = target
        data.ctrl[actuator_ids["Gripper"]] = gripper_target

    current = {name: float(data.qpos[qpos_ids[name]]) for name in ARM_JOINTS}
    for joint_name in order:
        start = current[joint_name]
        end = goal_joints[joint_name]
        for index in range(1, args.approach_segments_per_joint + 1):
            current[joint_name] = start + (end - start) * index / args.approach_segments_per_joint
            set_targets(current, args.gripper_open_rad)
            mujoco.mj_step(model, data)
            observe_contacts("approach_path")
    for _ in range(args.settle_steps):
        set_targets(goal_joints, args.gripper_open_rad)
        mujoco.mj_step(model, data)
        observe_contacts("approach_settle")

    approach_actual_joints = {
        name: float(data.qpos[qpos_ids[name]]) for name in ARM_JOINTS
    }
    close_start = {name: float(data.qpos[qpos_ids[name]]) for name in ARM_JOINTS}
    object_z_before_close = float(data.qpos[object_qpos + 2])
    for index in range(1, args.close_steps + 1):
        fraction = index / args.close_steps
        grip_target = args.gripper_open_rad + (
            args.gripper_close_rad - args.gripper_open_rad
        ) * fraction
        set_targets(close_start, grip_target)
        mujoco.mj_step(model, data)
        observe_contacts("close_path")
    for _ in range(args.settle_steps):
        set_targets(close_start, args.gripper_close_rad)
        mujoco.mj_step(model, data)
        observe_contacts("close_settle")
    close_actual_joints = {
        name: float(data.qpos[qpos_ids[name]]) for name in ARM_JOINTS
    }
    close_state = {
        "arm_actuator_targets_rad": close_start,
        "arm_joint_positions_rad": close_actual_joints,
        "arm_joint_delta_from_approach_rad": {
            name: close_actual_joints[name] - close_start[name] for name in ARM_JOINTS
        },
        "gripper_actuator_target_rad": args.gripper_close_rad,
        "gripper_joint_position_rad": float(data.qpos[gripper_qpos]),
        "gripper_mimic_joint_position_rad": float(data.qpos[gripper_mimic_qpos]),
    }

    def object_contact_summary(*, include_contact_details: bool = False) -> dict[str, object]:
        pairs: dict[str, int] = {}
        jaw_bodies: set[str] = set()
        contact_details: list[dict[str, object]] = []
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            body_names = tuple(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g]))
                for g in geom_ids
            )
            if object_body_name in body_names:
                other = next((body for body in body_names if body != object_body_name), "unknown")
                pairs[other or "world"] = pairs.get(other or "world", 0) + 1
                if other in ("link6", "link7"):
                    jaw_bodies.add(other)
                if include_contact_details:
                    normal_object_to_other = np.asarray(contact.frame[:3], dtype=float).copy()
                    if body_names[0] != object_body_name:
                        normal_object_to_other *= -1.0
                    object_geom_name = mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_GEOM,
                        geom_ids[body_names.index(object_body_name)],
                    )
                    other_geom_id = geom_ids[1 - body_names.index(object_body_name)]
                    other_geom_name = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_GEOM, other_geom_id
                    )
                    contact_details.append(
                        {
                            "other_body": other or "world",
                            "object_geom": object_geom_name
                            or f"unnamed_geom:{geom_ids[body_names.index(object_body_name)]}",
                            "other_geom": other_geom_name or f"unnamed_geom:{other_geom_id}",
                            "distance_m": float(contact.dist),
                            "position_world_m": np.asarray(contact.pos, dtype=float).tolist(),
                            "normal_object_to_other_world": normal_object_to_other.tolist(),
                        }
                    )
        summary = {
            "contact_count_by_other_body": pairs,
            "both_jaws_contacting": jaw_bodies == {"link6", "link7"},
        }
        if include_contact_details:
            summary["object_contact_details"] = contact_details
        return summary

    close_contacts = object_contact_summary(include_contact_details=True)
    object_z_before_lift = float(data.qpos[object_qpos + 2])
    link6 = name_id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    object_pose_world_position = data.xpos[object_body].copy()
    object_pose_world_quaternion = data.xquat[object_body].copy()
    link6_pose_world_position = data.xpos[link6].copy()
    link6_pose_world_quaternion = data.xquat[link6].copy()
    object_rotation_world = quaternion_rotation_matrix_wxyz(object_pose_world_quaternion)
    link6_pose_object_position = object_rotation_world.T @ (
        link6_pose_world_position - object_pose_world_position
    )
    object_quaternion_conjugate = object_pose_world_quaternion.copy()
    object_quaternion_conjugate[1:] *= -1
    link6_pose_object_quaternion = relative_quaternion_wxyz(
        object_quaternion_conjugate, link6_pose_world_quaternion
    )
    close_pose_observation = {
        "parent_body": "placeholder_pick_object",
        "child_body": "link6",
        "object_pose_world": {
            "position_m": object_pose_world_position.tolist(),
            "quaternion_wxyz": object_pose_world_quaternion.tolist(),
        },
        "link6_pose_world": {
            "position_m": link6_pose_world_position.tolist(),
            "quaternion_wxyz": link6_pose_world_quaternion.tolist(),
        },
        "link6_pose_object": {
            "position_m": link6_pose_object_position.tolist(),
            "quaternion_wxyz": link6_pose_object_quaternion.tolist(),
        },
        "scope": (
            "MuJoCo body-origin transform at close-settle state; placeholder object and link6 only; "
            "not a physical TCP measurement or verified grasp transform"
        ),
    }

    # Solve link6-origin position IK on a scratch state; live physics state remains untouched.
    ik_data = mujoco.MjData(model)
    mujoco.mj_copyData(ik_data, model, data)
    target_position = ik_data.xpos[link6].copy()
    target_position[2] += args.lift_height_m
    target_orientation = ik_data.xquat[link6].copy()
    jacobian_position = np.zeros((3, model.nv), dtype=float)
    jacobian_rotation = np.zeros((3, model.nv), dtype=float)
    residual_m = math.inf
    orientation_residual_rad = 0.0
    iterations = 0
    best_score = math.inf
    best_qpos = ik_data.qpos.copy()
    for iterations in range(1, 101):
        mujoco.mj_forward(model, ik_data)
        error = target_position - ik_data.xpos[link6]
        residual_m = float(np.linalg.norm(error))
        orientation_error = world_orientation_error(target_orientation, ik_data.xquat[link6])
        orientation_residual_rad = float(np.linalg.norm(orientation_error))
        scale = args.lift_orientation_scale_m_per_rad
        score = math.hypot(residual_m, scale * orientation_residual_rad)
        if score < best_score:
            best_score = score
            best_qpos = ik_data.qpos.copy()
        if residual_m < 0.0003 and (
            scale == 0 or orientation_residual_rad < math.radians(1)
        ):
            break
        mujoco.mj_jacBody(model, ik_data, jacobian_position, jacobian_rotation, link6)
        arm_jacobian_position = jacobian_position[:, dof_ids]
        if scale > 0:
            jacobian = np.vstack(
                (arm_jacobian_position, jacobian_rotation[:, dof_ids] * scale)
            )
            task_error = np.concatenate((error, orientation_error * scale))
            damping = 0.02**2 * np.eye(jacobian.shape[0])
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping, task_error
            )
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > 0.2:
                delta *= 0.2 / delta_norm
        else:
            damping = 0.0025 * np.eye(3)
            delta = arm_jacobian_position.T @ np.linalg.solve(
                arm_jacobian_position @ arm_jacobian_position.T + damping, error
            )
        velocity = np.zeros(model.nv, dtype=float)
        velocity[dof_ids] = delta
        mujoco.mj_integratePos(model, ik_data.qpos, velocity, 1.0)
        for joint_name in ARM_JOINTS:
            joint_id = joint_ids[joint_name]
            if model.jnt_limited[joint_id]:
                address = qpos_ids[joint_name]
                ik_data.qpos[address] = np.clip(
                    ik_data.qpos[address], *model.jnt_range[joint_id]
                )
    ik_data.qpos[:] = best_qpos
    lift_targets = {name: float(ik_data.qpos[qpos_ids[name]]) for name in ARM_JOINTS}
    mujoco.mj_forward(model, ik_data)
    residual_m = float(np.linalg.norm(target_position - ik_data.xpos[link6]))
    orientation_residual_rad = float(
        np.linalg.norm(world_orientation_error(target_orientation, ik_data.xquat[link6]))
    )

    min_object_z = object_z_before_lift
    max_object_z = object_z_before_lift
    both_jaw_contact_steps = 0
    for index in range(1, args.lift_steps + 1):
        fraction = index / args.lift_steps
        targets = {
            name: close_start[name] + (lift_targets[name] - close_start[name]) * fraction
            for name in ARM_JOINTS
        }
        set_targets(targets, args.gripper_close_rad)
        mujoco.mj_step(model, data)
        observe_contacts("lift_path")
        object_z = float(data.qpos[object_qpos + 2])
        min_object_z = min(min_object_z, object_z)
        max_object_z = max(max_object_z, object_z)
        if object_contact_summary()["both_jaws_contacting"]:
            both_jaw_contact_steps += 1
    for _ in range(args.settle_steps):
        set_targets(lift_targets, args.gripper_close_rad)
        mujoco.mj_step(model, data)
        observe_contacts("post_lift_settle")

    result = {
        "schema_version": 8,
        "kind": "mujoco_pick_dynamics_audit",
        "audit_id": uuid.uuid4().hex,
        "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "simulation_only": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "candidate_verified_grasp": False,
        "scene": str(SCENE_PATH.relative_to(REPO_ROOT)),
        "mujoco_version": str(mujoco.__version__),
        "model_source_files": model_sources,
        "model_dimensions": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "physics_timestep_s": float(model.opt.timestep),
        },
        "placeholder_geometry": {
            "object_box_half_size_m": model.geom_size[object_geom].tolist(),
            "object_center_world_m": placeholder_object_center_world.tolist(),
            "shelf_top_z_m": shelf_top_z,
        },
        "model_joint_goals_rad": goal_joints,
        "joint_goal_source": (
            "hash_verified_offline_ik_candidate"
            if candidate_record is not None
            else "explicit_simulation_joint_values"
        ),
        "ik_candidate_input": (
            {
                "path": args.candidate_json.as_posix(),
                "sha256": candidate_sha256,
                "candidate_status": candidate_record.get("candidate_status")
                if candidate_record is not None
                else None,
                "target": {
                    "frame_id": candidate_record["frame_id"],
                    "body": candidate_record["body"],
                    "position_m": candidate_record["target"]["position_m"],
                    "orientation_quaternion_wxyz": candidate_record["target"][
                        "orientation_quaternion_wxyz"
                    ],
                },
                "achieved_position_m": candidate_record["achieved"]["position_m"],
                "position_residual_m": candidate_record["residual"]["position_m"],
                "target_to_object_center_distance_m": float(
                    np.linalg.norm(
                        np.asarray(candidate_record["target"]["position_m"], dtype=float)
                        - placeholder_object_center_world
                    )
                ),
                "distance_scope": (
                    "Euclidean world distance from requested link6 body origin to initial "
                    "placeholder box center; not TCP offset or grasp clearance"
                ),
            }
            if args.candidate_json is not None and candidate_record is not None
            else None
        ),
        "joint_order": order,
        "gripper_open_target_rad": args.gripper_open_rad,
        "gripper_close_target_rad": args.gripper_close_rad,
        "step_counts": {
            "approach_segments_per_joint": args.approach_segments_per_joint,
            "close_steps": args.close_steps,
            "settle_steps_each_stage": args.settle_steps,
            "lift_steps": args.lift_steps,
        },
        "contact_observations": contact_metrics_json(),
        "approach_actual_joint_error_rad": {
            name: approach_actual_joints[name] - goal_joints[name]
            for name in ARM_JOINTS
        },
        "object_z_before_close_m": object_z_before_close,
        "object_z_after_close_m": object_z_before_lift,
        "close_contact_observation": close_contacts,
        "close_state": close_state,
        "close_pose_observation": close_pose_observation,
        "lift_ik": {
            "method": (
                "weighted_position_and_orientation"
                if args.lift_orientation_scale_m_per_rad > 0
                else "position_only"
            ),
            "requested_height_m": args.lift_height_m,
            "orientation_scale_m_per_rad": args.lift_orientation_scale_m_per_rad,
            "iterations": iterations,
            "position_residual_m": residual_m,
            "orientation_residual_rad": orientation_residual_rad,
            "joint_delta_rad": {
                name: lift_targets[name] - close_start[name] for name in ARM_JOINTS
            },
        },
        "lift_observation": {
            "object_z_min_m": min_object_z,
            "object_z_max_m": max_object_z,
            "object_z_final_m": float(data.qpos[object_qpos + 2]),
            "object_z_change_final_m": float(data.qpos[object_qpos + 2] - object_z_before_lift),
            "object_bottom_clearance_max_above_shelf_m": (
                max_object_z - float(model.geom_size[object_geom, 2]) - shelf_top_z
            ),
            "object_bottom_clearance_final_above_shelf_m": (
                float(data.qpos[object_qpos + 2])
                - float(model.geom_size[object_geom, 2])
                - shelf_top_z
            ),
            "both_jaw_contact_steps": both_jaw_contact_steps,
            "lift_steps": args.lift_steps,
            "final_object_contacts": object_contact_summary(),
        },
        "continuous_collision_checked": False,
        "contact_sampling": "one contact snapshot after each MuJoCo mj_step; not continuous collision checking",
        "note": "MuJoCo placeholder geometry and actuator dynamics only; no ROS or hardware command.",
    }
    artifact_error: Exception | None = None
    if args.save_artifact:
        try:
            publish_audit_artifact(result)
        except (OSError, ValueError) as exc:
            result.pop("artifact_path", None)
            artifact_error = exc
    print(json.dumps(result, indent=2, sort_keys=True))
    if artifact_error is not None:
        print(f"Could not save MuJoCo audit artifact: {artifact_error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"MuJoCo pick dynamics audit failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
