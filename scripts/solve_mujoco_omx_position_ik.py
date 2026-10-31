#!/usr/bin/env python3
"""Solve a MuJoCo-only position or optional pose IK candidate for OMX link6."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

from calibration_artifacts import load_link6_tcp_transform_artifact
from mujoco_model_sources import model_source_artifacts


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"
ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")
BODY_NAME = "link6"
MAX_START_CANDIDATE_BYTES = 1_000_000


def _parse_xyz(value: str) -> tuple[float, float, float]:
    try:
        parts = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be x,y,z in meters") from exc
    if len(parts) != 3 or not all(math.isfinite(part) for part in parts):
        raise argparse.ArgumentTypeError("value must contain three finite meter values x,y,z")
    return parts


def _parse_quaternion(value: str) -> tuple[float, float, float, float]:
    try:
        parts = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("target quaternion must be w,x,y,z") from exc
    if len(parts) != 4 or not all(math.isfinite(part) for part in parts):
        raise argparse.ArgumentTypeError("target quaternion must contain four finite values w,x,y,z")
    scale = max(abs(part) for part in parts)
    if scale == 0:
        raise argparse.ArgumentTypeError("target quaternion must have nonzero norm")
    scaled = tuple(part / scale for part in parts)
    scaled_norm = math.hypot(*scaled)
    if scale < 1e-12 / scaled_norm:
        raise argparse.ArgumentTypeError("target quaternion norm must be at least 1e-12")
    return tuple(part / scaled_norm for part in scaled)  # type: ignore[return-value]


def _orientation_error(
    target: tuple[float, float, float, float],
    current: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Return the shortest world-frame rotation vector from current to target."""
    tw, tx, ty, tz = target
    cw, cx, cy, cz = current
    error_w = tw * cw + tx * cx + ty * cy + tz * cz
    error_x = cw * tx - tw * cx - ty * cz + tz * cy
    error_y = cw * ty - tw * cy - tz * cx + tx * cz
    error_z = cw * tz - tw * cz - tx * cy + ty * cx
    if error_w < 0:
        error_w, error_x, error_y, error_z = (
            -error_w,
            -error_x,
            -error_y,
            -error_z,
        )
    vector_norm = math.sqrt(error_x * error_x + error_y * error_y + error_z * error_z)
    if vector_norm < 1e-12:
        return (2.0 * error_x, 2.0 * error_y, 2.0 * error_z)
    angle = 2.0 * math.atan2(vector_norm, max(0.0, error_w))
    scale = angle / vector_norm
    return (error_x * scale, error_y * scale, error_z * scale)


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a 3D vector by a normalized scalar-first quaternion."""
    axis_x, axis_y, axis_z = quaternion[1:]
    first = (
        axis_y * vector[2] - axis_z * vector[1],
        axis_z * vector[0] - axis_x * vector[2],
        axis_x * vector[1] - axis_y * vector[0],
    )
    second = (
        axis_y * first[2] - axis_z * first[1],
        axis_z * first[0] - axis_x * first[2],
        axis_x * first[1] - axis_y * first[0],
    )
    return tuple(
        vector[index] + 2.0 * (quaternion[0] * first[index] + second[index])
        for index in range(3)
    )


def _contact_pairs(model, data, mujoco) -> set[str]:
    pairs: set[str] = set()
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        labels = []
        for geom_id in (contact.geom1, contact.geom2):
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            body_id = int(model.geom_bodyid[geom_id])
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            labels.append(f"{body_name or body_id}/{geom_name or geom_id}")
        pairs.add(" <-> ".join(labels))
    return pairs


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    value: dict[str, object] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"start candidate contains a duplicate JSON key: {key}")
        value[key] = item
    return value


def _load_start_candidate(path: Path) -> tuple[dict[str, object], str]:
    if path.is_symlink() or not path.is_file():
        raise ValueError("start candidate must be a regular non-symlink JSON file")
    try:
        if path.stat().st_size > MAX_START_CANDIDATE_BYTES:
            raise ValueError("start candidate JSON exceeds the 1 MB limit")
        with path.open("rb") as source:
            raw = source.read(MAX_START_CANDIDATE_BYTES + 1)
    except OSError as exc:
        raise ValueError("start candidate JSON cannot be read") from exc
    if len(raw) > MAX_START_CANDIDATE_BYTES:
        raise ValueError("start candidate JSON exceeds the 1 MB limit")
    try:
        candidate = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ValueError("start candidate JSON is invalid") from exc
    if not isinstance(candidate, dict):
        raise ValueError("start candidate JSON must contain an object")
    if (
        isinstance(candidate.get("schema_version"), bool)
        or not isinstance(candidate.get("schema_version"), int)
        or candidate.get("schema_version") != 2
        or candidate.get("kind") != "mujoco_ik_candidate"
        or candidate.get("simulation_only") is not True
        or candidate.get("usable_for_robot_control") is not False
        or candidate.get("arm_command_sent") is not False
        or candidate.get("model") != "simulation/mujoco_omx/robotis_omx/shelf_pick_scene.xml"
    ):
        raise ValueError("start candidate is not a version 2 MuJoCo-only OMX IK candidate")
    return candidate, hashlib.sha256(raw).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=_parse_xyz, required=True, metavar="X,Y,Z")
    parser.add_argument(
        "--target-frame",
        choices=("link6", "tcp"),
        default="link6",
        help="frame of the requested world position/orientation",
    )
    parser.add_argument(
        "--target-quat",
        type=_parse_quaternion,
        metavar="W,X,Y,Z",
        help="optional requested-frame orientation in MuJoCo world-frame quaternion order",
    )
    parser.add_argument(
        "--tcp-offset-link6-m",
        type=_parse_xyz,
        metavar="X,Y,Z",
        help="TCP origin translation expressed in MuJoCo link6 coordinates",
    )
    parser.add_argument(
        "--tcp-quat-link6-wxyz",
        type=_parse_quaternion,
        metavar="W,X,Y,Z",
        help="TCP orientation expressed relative to MuJoCo link6, scalar-first order",
    )
    parser.add_argument(
        "--tcp-transform-json",
        type=Path,
        help="review-only TCP transform artifact under .control-station/omx-calibrations",
    )
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--tolerance-mm", type=float, default=2.0)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=3.0)
    parser.add_argument("--orientation-scale-m", type=float, default=0.1)
    parser.add_argument(
        "--format",
        choices=("text", "json"),
        default="text",
        help="output format; JSON is a simulation-only candidate record",
    )
    parser.add_argument(
        "--path-samples",
        type=int,
        default=21,
        help="minimum number of joint-linear path segments",
    )
    parser.add_argument(
        "--max-path-joint-step-rad",
        type=float,
        default=0.05,
        help="maximum absolute joint change between discrete path samples",
    )
    parser.add_argument(
        "--start-joints-json",
        type=Path,
        help="use a validated, contact-free prior IK candidate as the simulation start state",
    )
    args = parser.parse_args()
    has_tcp_translation = args.tcp_offset_link6_m is not None
    has_tcp_orientation = args.tcp_quat_link6_wxyz is not None
    tcp_calibration_artifact = None
    if args.target_frame == "tcp":
        if args.tcp_transform_json is not None:
            if has_tcp_translation or has_tcp_orientation:
                parser.error(
                    "--tcp-transform-json cannot be combined with inline TCP transform options"
                )
            try:
                tcp_calibration_artifact = load_link6_tcp_transform_artifact(
                    args.tcp_transform_json, REPO_ROOT
                )
            except (OSError, ValueError) as exc:
                parser.error(str(exc))
            calibration = tcp_calibration_artifact["validated"]
            transform = calibration["tcp_transform"]
            args.tcp_offset_link6_m = tuple(transform["translation_link6_m"])
            args.tcp_quat_link6_wxyz = tuple(transform["orientation_link6_wxyz"])
        elif not has_tcp_translation or not has_tcp_orientation:
            parser.error(
                "--target-frame tcp requires --tcp-transform-json or both inline "
                "TCP transform options"
            )
        if args.target_quat is None:
            parser.error("--target-frame tcp requires --target-quat for a full TCP pose target")
    elif has_tcp_translation or has_tcp_orientation or args.tcp_transform_json is not None:
        parser.error("TCP transform options require --target-frame tcp")
    if not 1 <= args.iterations <= 2_000:
        parser.error("--iterations must be between 1 and 2000")
    if not math.isfinite(args.tolerance_mm) or not 0.1 <= args.tolerance_mm <= 50:
        parser.error("--tolerance-mm must be between 0.1 and 50")
    if (
        not math.isfinite(args.orientation_tolerance_deg)
        or not 0.1 <= args.orientation_tolerance_deg <= 90
    ):
        parser.error("--orientation-tolerance-deg must be between 0.1 and 90")
    if not math.isfinite(args.orientation_scale_m) or not 0.01 <= args.orientation_scale_m <= 1:
        parser.error("--orientation-scale-m must be between 0.01 and 1")
    if not 2 <= args.path_samples <= 501:
        parser.error("--path-samples must be between 2 and 501")
    if (
        not math.isfinite(args.max_path_joint_step_rad)
        or not 0.005 <= args.max_path_joint_step_rad <= 0.5
    ):
        parser.error("--max-path-joint-step-rad must be between 0.005 and 0.5 radians")

    try:
        import mujoco
        import numpy as np
    except ImportError:
        print(
            "MuJoCo/NumPy is not installed. Install the optional dependency with "
            "python -m pip install -r requirements-mujoco.txt",
            file=sys.stderr,
        )
        return 2

    try:
        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    except (OSError, ValueError) as exc:
        print(f"Could not load the MuJoCo OMX scene: {exc}", file=sys.stderr)
        return 2

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BODY_NAME)
    if body_id < 0:
        print(f"Required MuJoCo body is missing: {BODY_NAME}", file=sys.stderr)
        return 2

    joint_ids: list[int] = []
    qpos_addresses: list[int] = []
    dof_addresses: list[int] = []
    limits: list[tuple[float, float]] = []
    for name in ARM_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            print(f"Required hinge joint is missing or unsupported: {name}", file=sys.stderr)
            return 2
        if not model.jnt_limited[joint_id]:
            print(f"Joint {name} has no bounded range; refusing IK candidate", file=sys.stderr)
            return 2
        joint_ids.append(joint_id)
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        dof_addresses.append(int(model.jnt_dofadr[joint_id]))
        lower, upper = model.jnt_range[joint_id]
        limits.append((float(lower), float(upper)))

    model_sources: list[dict[str, object]] = []
    if args.format == "json" or args.start_joints_json is not None:
        try:
            model_sources = model_source_artifacts()
        except OSError as exc:
            print(f"Could not hash MuJoCo model source files: {exc}", file=sys.stderr)
            return 2

    start_candidate = None
    start_candidate_sha256 = None
    if args.start_joints_json is not None:
        try:
            start_candidate, start_candidate_sha256 = _load_start_candidate(
                args.start_joints_json
            )
        except (OSError, ValueError) as exc:
            print(f"Invalid start candidate: {exc}", file=sys.stderr)
            return 2
        if start_candidate.get("mujoco_version") != str(mujoco.__version__):
            print("Start candidate MuJoCo version does not match the current runtime", file=sys.stderr)
            return 2
        if start_candidate.get("model_source_files") != model_sources:
            print("Start candidate model source hashes do not match the current scene", file=sys.stderr)
            return 2
        previous_start_state = start_candidate.get("start_state")
        previous_candidate_hash = start_candidate.get("start_candidate_sha256")
        if previous_start_state not in {"home_keyframe", "model_default", "provided_candidate"}:
            print("Start candidate start_state is invalid", file=sys.stderr)
            return 2
        if previous_start_state == "provided_candidate":
            if (
                not isinstance(previous_candidate_hash, str)
                or len(previous_candidate_hash) != 64
                or any(character not in "0123456789abcdef" for character in previous_candidate_hash)
            ):
                print("Start candidate source hash is missing or invalid", file=sys.stderr)
                return 2
        elif previous_candidate_hash is not None:
            print("Home/model-default candidate must not claim a prior candidate hash", file=sys.stderr)
            return 2
        if start_candidate.get("converged") is not True:
            print("Start candidate did not converge", file=sys.stderr)
            return 2
        if (
            start_candidate.get("position_converged") is not True
            or start_candidate.get("pose_converged") is not True
            or start_candidate.get("pose_frame") not in {"link6", "tcp"}
            or start_candidate.get("candidate_status")
            not in {
                "POSITION_CONVERGED_ONLY; PATH_AND_GRASP_UNVERIFIED",
                "POSE_CONVERGED_ONLY; PATH_AND_GRASP_UNVERIFIED",
            }
        ):
            print("Start candidate convergence status is inconsistent or unsupported", file=sys.stderr)
            return 2
        contacts = start_candidate.get("contacts")
        if not isinstance(contacts, dict) or any(
            not isinstance(contacts.get(key), list) or contacts[key]
            for key in ("start_pose", "final_pose", "final_shelf", "final_target_object", "joint_linear_path")
        ):
            print("Start candidate must have no recorded start, final, or discrete-path contacts", file=sys.stderr)
            return 2
        path_sampling = start_candidate.get("path_sampling")
        if (
            not isinstance(path_sampling, dict)
            or path_sampling.get("motion_planned") is not False
            or path_sampling.get("continuous_collision_checked") is not False
        ):
            print("Start candidate path-sampling contract is invalid", file=sys.stderr)
            return 2
        candidate_joints = start_candidate.get("joint_candidate_rad")
        if not isinstance(candidate_joints, dict) or set(candidate_joints) != set(ARM_JOINTS):
            print("Start candidate must contain all five arm joint positions", file=sys.stderr)
            return 2

    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
    start_state = "home_keyframe" if home_key >= 0 else "model_default"
    if start_candidate is not None:
        candidate_joints = start_candidate["joint_candidate_rad"]
        assert isinstance(candidate_joints, dict)
        for name, qpos_address, (lower, upper) in zip(ARM_JOINTS, qpos_addresses, limits):
            value = candidate_joints.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                print(f"Start candidate has an invalid/out-of-range position for {name}", file=sys.stderr)
                return 2
            try:
                numeric_value = float(value)
            except (OverflowError, ValueError):
                print(f"Start candidate has an invalid/out-of-range position for {name}", file=sys.stderr)
                return 2
            if not math.isfinite(numeric_value) or not lower <= numeric_value <= upper:
                print(f"Start candidate has an invalid/out-of-range position for {name}", file=sys.stderr)
                return 2
            data.qpos[qpos_address] = numeric_value
        start_state = "provided_candidate"
    start_qpos = data.qpos.copy()
    start_joint_positions = {
        name: float(start_qpos[qpos_address])
        for name, qpos_address in zip(ARM_JOINTS, qpos_addresses)
    }
    mujoco.mj_forward(model, data)
    start_contact_pairs = _contact_pairs(model, data, mujoco)
    target = np.asarray(args.target, dtype=np.float64)
    tcp_mode = args.target_frame == "tcp"
    tcp_offset = np.asarray(args.tcp_offset_link6_m or (0.0, 0.0, 0.0), dtype=np.float64)
    tcp_quaternion = args.tcp_quat_link6_wxyz or (1.0, 0.0, 0.0, 0.0)
    tolerance_m = args.tolerance_mm / 1000.0
    target_quaternion = args.target_quat
    orientation_tolerance_rad = math.radians(args.orientation_tolerance_deg)
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    damping = 0.02
    max_joint_step = 0.20
    best_error = math.inf
    best_qpos = data.qpos.copy()

    iterations_used = 0
    for iteration in range(args.iterations):
        iterations_used = iteration + 1
        mujoco.mj_forward(model, data)
        link6_position = data.xpos[body_id].copy()
        link6_quaternion = tuple(float(value) for value in data.xquat[body_id])
        current = link6_position.copy()
        current_quaternion = link6_quaternion
        tcp_world_offset = np.zeros(3, dtype=np.float64)
        if tcp_mode:
            tcp_world_offset = np.asarray(
                _rotate_vector(link6_quaternion, tuple(float(value) for value in tcp_offset)),
                dtype=np.float64,
            )
            current += tcp_world_offset
            current_quaternion = _quaternion_multiply(link6_quaternion, tcp_quaternion)
        position_error = target - current
        position_error_norm = float(np.linalg.norm(position_error))
        orientation_error = (
            np.asarray(
                _orientation_error(target_quaternion, current_quaternion),
                dtype=np.float64,
            )
            if target_quaternion is not None
            else np.empty(0, dtype=np.float64)
        )
        orientation_error_norm = (
            float(np.linalg.norm(orientation_error)) if target_quaternion is not None else 0.0
        )
        if not math.isfinite(position_error_norm) or not math.isfinite(orientation_error_norm):
            print("IK produced a non-finite residual", file=sys.stderr)
            return 3
        pose_error_score = math.hypot(
            position_error_norm,
            orientation_error_norm * args.orientation_scale_m,
        )
        if pose_error_score < best_error:
            best_error = pose_error_score
            best_qpos = data.qpos.copy()
        if (
            position_error_norm <= tolerance_m
            and (
                target_quaternion is None
                or orientation_error_norm <= orientation_tolerance_rad
            )
        ):
            break

        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        arm_jacobian_position = jacp[:, dof_addresses]
        if tcp_mode:
            arm_jacobian_position = arm_jacobian_position + np.cross(
                jacr[:, dof_addresses].T, tcp_world_offset
            ).T
        if target_quaternion is None:
            arm_jacobian = arm_jacobian_position
            error = position_error
        else:
            arm_jacobian_rotation = jacr[:, dof_addresses]
            arm_jacobian = np.vstack(
                (
                    arm_jacobian_position,
                    arm_jacobian_rotation * args.orientation_scale_m,
                )
            )
            error = np.concatenate(
                (position_error, orientation_error * args.orientation_scale_m)
            )
        system = arm_jacobian @ arm_jacobian.T + (damping**2) * np.eye(arm_jacobian.shape[0])
        try:
            delta = arm_jacobian.T @ np.linalg.solve(system, error)
        except np.linalg.LinAlgError:
            print("IK Jacobian solve failed", file=sys.stderr)
            return 3
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > max_joint_step:
            delta *= max_joint_step / delta_norm

        for qpos_address, step, (lower, upper) in zip(qpos_addresses, delta, limits):
            data.qpos[qpos_address] = min(
                upper, max(lower, float(data.qpos[qpos_address] + step))
            )

    data.qpos[:] = best_qpos
    mujoco.mj_forward(model, data)
    final_link6_position = data.xpos[body_id].copy()
    final_link6_quaternion = tuple(float(value) for value in data.xquat[body_id])
    final_position = final_link6_position.copy()
    final_quaternion = final_link6_quaternion
    if tcp_mode:
        final_position += np.asarray(
            _rotate_vector(final_link6_quaternion, tuple(float(value) for value in tcp_offset)),
            dtype=np.float64,
        )
        final_quaternion = _quaternion_multiply(final_link6_quaternion, tcp_quaternion)
    final_error = float(np.linalg.norm(target - final_position))
    if not np.isfinite(final_position).all() or not math.isfinite(final_error):
        print("IK produced a non-finite result", file=sys.stderr)
        return 3
    final_orientation_error_rad = (
        float(
            np.linalg.norm(
                _orientation_error(target_quaternion, final_quaternion)
            )
        )
        if target_quaternion is not None
        else None
    )
    position_converged = final_error <= tolerance_m
    orientation_converged = (
        target_quaternion is None
        or (
            final_orientation_error_rad is not None
            and final_orientation_error_rad <= orientation_tolerance_rad
        )
    )
    pose_converged = position_converged and orientation_converged
    final_qpos = data.qpos.copy()
    max_joint_delta = max(
        abs(float(final_qpos[address] - start_qpos[address]))
        for address in qpos_addresses
    )
    path_segments = max(
        args.path_samples,
        math.ceil(max_joint_delta / args.max_path_joint_step_rad),
    )
    path_step_rad = max_joint_delta / path_segments
    contact_pairs = _contact_pairs(model, data, mujoco)
    shelf_contact_pairs: set[str] = set()
    object_contact_pairs: set[str] = set()
    for pair in contact_pairs:
        geom_names = []
        for label in pair.split(" <-> "):
            geom_names.append(label.rsplit("/", 1)[-1])
        if any(name and name.startswith("placeholder_shelf_") for name in geom_names):
            shelf_contact_pairs.add(pair)
        if "placeholder_pick_object_geom" in geom_names:
            object_contact_pairs.add(pair)

    path_contact_pairs: set[str] = set()
    first_path_contact_sample: int | None = None
    for sample_index in range(1, path_segments):
        fraction = sample_index / path_segments
        data.qpos[:] = start_qpos
        for qpos_address, start_value, target_value in zip(
            qpos_addresses,
            (start_qpos[address] for address in qpos_addresses),
            (final_qpos[address] for address in qpos_addresses),
        ):
            data.qpos[qpos_address] = start_value + fraction * (target_value - start_value)
        mujoco.mj_forward(model, data)
        sample_contacts = _contact_pairs(model, data, mujoco)
        if sample_contacts and first_path_contact_sample is None:
            first_path_contact_sample = sample_index
        path_contact_pairs.update(sample_contacts)

    data.qpos[:] = final_qpos
    mujoco.mj_forward(model, data)
    mode_prefix = "POSE" if target_quaternion is not None else "POSITION"
    if not pose_converged:
        candidate_status = (
            "POSE_POSITION_DID_NOT_CONVERGE"
            if not position_converged and target_quaternion is not None
            else "POSE_ORIENTATION_DID_NOT_CONVERGE"
            if target_quaternion is not None
            else "POSITION_DID_NOT_CONVERGE"
        )
        exit_code = 3
    elif start_contact_pairs:
        candidate_status = (
            f"{mode_prefix}_CONVERGED_BUT_START_POSE_HAS_UNVERIFIED_CONTACTS"
        )
        exit_code = 6
    elif contact_pairs:
        candidate_status = (
            f"{mode_prefix}_CONVERGED_BUT_FINAL_POSE_HAS_UNVERIFIED_CONTACTS"
        )
        exit_code = 4
    elif path_contact_pairs:
        candidate_status = f"{mode_prefix}_CONVERGED_BUT_DISCRETE_PATH_HAS_CONTACTS"
        exit_code = 5
    else:
        candidate_status = (
            ("POSE_CONVERGED_ONLY" if target_quaternion is not None else "POSITION_CONVERGED_ONLY")
            + "; PATH_AND_GRASP_UNVERIFIED"
        )
        exit_code = 0

    joint_candidate = {
        name: float(data.qpos[qpos_address])
        for name, qpos_address in zip(ARM_JOINTS, qpos_addresses)
    }
    tcp_calibration_record = None
    if tcp_calibration_artifact is not None:
        calibration = tcp_calibration_artifact["validated"]
        evidence = tcp_calibration_artifact["verified_evidence"]
        tcp_calibration_record = {
            "calibration_id": calibration["calibration_id"],
            "status": "DRAFT_ONLY",
            "artifact": tcp_calibration_artifact["artifact"],
            "robot_model": calibration["robot_model"],
            "frame_mapping": {
                **calibration["frame_mapping"],
                "evidence": evidence["frame_mapping"],
            },
            "measurement": {
                **calibration["measurement"],
                "source": evidence["measurement"],
            },
        }
    result = {
        "schema_version": 2,
        "kind": "mujoco_ik_candidate",
        "simulation_only": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "model": "simulation/mujoco_omx/robotis_omx/shelf_pick_scene.xml",
        "model_source_files": model_sources,
        "mujoco_version": str(mujoco.__version__),
        "start_state": start_state,
        "start_candidate_sha256": start_candidate_sha256,
        "start_joint_rad": start_joint_positions,
        "pose_frame": "tcp" if tcp_mode else "link6",
        "tcp_transform": {
            "translation_link6_m": [float(value) for value in tcp_offset],
            "orientation_link6_wxyz": list(tcp_quaternion),
            "source": "operator_supplied_transform",
        }
        if tcp_mode
        else None,
        "tcp_calibration": tcp_calibration_record,
        "solver": {
            "algorithm": "damped_least_squares",
            "max_iterations": args.iterations,
            "iterations_used": iterations_used,
            "position_tolerance_mm": args.tolerance_mm,
            "orientation_tolerance_deg": args.orientation_tolerance_deg
            if target_quaternion is not None
            else None,
            "orientation_scale_m_per_rad": args.orientation_scale_m
            if target_quaternion is not None
            else None,
            "minimum_path_segments": args.path_samples,
            "maximum_joint_step_rad": args.max_path_joint_step_rad,
        },
        "frame_id": "world",
        "body": "link6",
        "body_reference": "body_origin",
        "target": {
            "position_m": [float(value) for value in target],
            "orientation_quaternion_wxyz": list(target_quaternion)
            if target_quaternion is not None
            else None,
        },
        "achieved": {
            "position_m": [float(value) for value in final_position],
            "orientation_quaternion_wxyz": list(final_quaternion),
        },
        "link6_body_pose": {
            "position_m": [float(value) for value in final_link6_position],
            "orientation_quaternion_wxyz": list(final_link6_quaternion),
        },
        "residual": {
            "position_m": final_error,
            "orientation_rad": final_orientation_error_rad,
        },
        "tolerance": {
            "position_m": tolerance_m,
            "orientation_rad": orientation_tolerance_rad
            if target_quaternion is not None
            else None,
        },
        "converged": pose_converged,
        "position_converged": position_converged,
        "orientation_converged": orientation_converged
        if target_quaternion is not None
        else None,
        "pose_converged": pose_converged,
        "iterations": iterations_used,
        "candidate_status": candidate_status,
        "joint_candidate_rad": joint_candidate,
        "contacts": {
            "start_pose": sorted(start_contact_pairs),
            "final_pose": sorted(contact_pairs),
            "final_shelf": sorted(shelf_contact_pairs),
            "final_target_object": sorted(object_contact_pairs),
            "joint_linear_path": sorted(path_contact_pairs),
        },
        "path_sampling": {
            "method": "discrete_joint_linear_interpolation",
            "segments": path_segments,
            "intermediate_samples": path_segments - 1,
            "max_joint_step_rad": path_step_rad,
            "requested_max_joint_step_rad": args.max_path_joint_step_rad,
            "first_contact_sample": first_path_contact_sample,
            "continuous_collision_checked": False,
            "motion_planned": False,
        },
        "limitations": [
            (
                "TCP transform is operator-supplied and has not been verified against a physical model frame"
                if tcp_mode
                else "link6 body origin is not a measured gripper TCP"
            ),
            "contact is not grasp verification",
            "simulation candidates are not validated hardware commands",
            "camera observations, ROS, and automated calibration are not used",
        ],
    }
    if args.format == "json":
        print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
        return exit_code

    candidate_label = "pose IK" if target_quaternion is not None else "position IK"
    print(
        f"MuJoCo {candidate_label} candidate · body=link6 · "
        f"target_frame={args.target_frame} · frame=world"
    )
    print(f"target_xyz_m={target.tolist()}")
    print(f"achieved_{args.target_frame}_xyz_m={final_position.tolist()}")
    if tcp_mode:
        print(f"link6_body_xyz_m={final_link6_position.tolist()}")
    print(f"position_error_mm={final_error * 1000.0:.3f}")
    if target_quaternion is not None:
        print(f"target_quat_wxyz={list(target_quaternion)}")
        print(f"achieved_{args.target_frame}_quat_wxyz={list(final_quaternion)}")
        print(
            f"orientation_error_deg={math.degrees(final_orientation_error_rad or 0.0):.3f} · "
            f"tolerance_deg={args.orientation_tolerance_deg:.3f}"
        )
    print(f"converged={pose_converged} · iterations={iterations_used}")
    print("start_pose_contacts=" + ("; ".join(sorted(start_contact_pairs)) if start_contact_pairs else "none"))
    print("final_pose_contacts=" + ("; ".join(sorted(contact_pairs)) if contact_pairs else "none"))
    print("final_shelf_contacts=" + ("; ".join(sorted(shelf_contact_pairs)) if shelf_contact_pairs else "none"))
    print("final_target_object_contacts=" + ("; ".join(sorted(object_contact_pairs)) if object_contact_pairs else "none"))
    print(
        f"joint_linear_path_segments={path_segments} · maximum_joint_step_rad={path_step_rad:.6f} · "
        f"intermediate_samples={path_segments - 1} · first_contact_sample={first_path_contact_sample}"
    )
    print("joint_linear_path_contacts=" + ("; ".join(sorted(path_contact_pairs)) if path_contact_pairs else "none"))
    print("joint_candidate_rad=" + ", ".join(
        f"{name}={joint_candidate[name]:.5f}" for name in ARM_JOINTS
    ))
    if target_quaternion is None:
        print(
            "Position only. The path check linearly interpolates joint positions at discrete samples "
            "with a bounded maximum joint step; it is not continuous collision checking or a motion planner. "
            "No tool orientation, grasp check, ROS, camera, serial port, or physical command."
        )
    elif tcp_mode:
        print(
            "TCP pose uses the supplied link6-to-TCP transform, which is not verified against the physical tool. "
            "Path sampling is discrete and is not a motion planner. No grasp check, ROS, camera, serial port, "
            "or physical command."
        )
    else:
        print(
            "Link6 body pose only; link6 is not a measured gripper TCP. The path check is discrete "
            "and is not a motion planner. No grasp check, ROS, camera, serial port, or physical command."
        )
    print(f"candidate_status={candidate_status}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
