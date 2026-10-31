#!/usr/bin/env python3
"""Verify a saved MuJoCo pick-dynamics audit record without starting MuJoCo or ROS."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import sys
from typing import Any

from mujoco_model_sources import model_source_artifacts


REPO_ROOT = Path(__file__).resolve().parents[1]
MANAGED_DIRECTORY = REPO_ROOT / ".control-station" / "mujoco-dynamics-audits"
MAX_AUDIT_BYTES = 1_000_000
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")
CONTACT_STAGES = (
    "approach_path",
    "approach_settle",
    "close_path",
    "close_settle",
    "lift_path",
    "post_lift_settle",
)


class AuditVerificationError(ValueError):
    """Raised when a saved audit does not satisfy its review contract."""


def unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise AuditVerificationError(f"audit JSON contains a duplicate key: {key}")
        value[key] = item
    return value


def _finite_number(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(float(value))
    except OverflowError:
        return False


def _non_negative_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _finite_vector(value: object, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(_finite_number(component) for component in value)
    )


def _rotation_matrix_wxyz(quaternion: list[float]) -> tuple[tuple[float, ...], ...]:
    w, x, y, z = quaternion
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _relative_quaternion_wxyz(parent: list[float], child: list[float]) -> tuple[float, ...]:
    pw, px, py, pz = parent
    cw, cx, cy, cz = child
    return (
        pw * cw + px * cx + py * cy + pz * cz,
        pw * cx - px * cw - py * cz + pz * cy,
        pw * cy + px * cz - py * cw - pz * cx,
        pw * cz - px * cy + py * cx - pz * cw,
    )


def _regular_audit_bytes(path_value: str) -> tuple[Path, bytes]:
    requested = Path(path_value).expanduser()
    if not requested.is_absolute():
        requested = (Path.cwd() / requested).absolute()
    if requested.parent != MANAGED_DIRECTORY:
        raise AuditVerificationError(
            "audit must be a direct file under .control-station/mujoco-dynamics-audits"
        )
    for checked_path in (
        REPO_ROOT / ".control-station",
        MANAGED_DIRECTORY,
        requested,
    ):
        if checked_path.is_symlink():
            raise AuditVerificationError("audit path cannot contain symbolic links")
    try:
        if MANAGED_DIRECTORY.resolve(strict=True) != MANAGED_DIRECTORY:
            raise AuditVerificationError("managed audit directory resolves outside its repository path")
        if not requested.is_file() or requested.stat().st_size > MAX_AUDIT_BYTES:
            raise AuditVerificationError("audit must be a regular file no larger than 1 MB")
        with requested.open("rb") as source:
            raw = source.read(MAX_AUDIT_BYTES + 1)
    except OSError as exc:
        raise AuditVerificationError("audit file cannot be read") from exc
    if len(raw) > MAX_AUDIT_BYTES:
        raise AuditVerificationError("audit exceeds the 1 MB limit")
    return requested, raw


def _validate_sources(value: object) -> tuple[list[dict[str, Any]], bool]:
    if not isinstance(value, list) or len(value) < 2:
        raise AuditVerificationError("model_source_files must contain scene and OMX model sources")
    normalized: list[dict[str, Any]] = []
    for item in value:
        if not isinstance(item, dict):
            raise AuditVerificationError("model source entries must be JSON objects")
        path = item.get("path")
        if not isinstance(path, str) or not path:
            raise AuditVerificationError("model source path is missing")
        relative = PurePosixPath(path)
        if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
            raise AuditVerificationError("model source path must be repository-relative")
        if (
            item.get("path_base") != "REPOSITORY_ROOT"
            or item.get("hash_algorithm") != "sha256"
            or not isinstance(item.get("sha256"), str)
            or not SHA256_PATTERN.fullmatch(item["sha256"])
            or not _non_negative_integer(item.get("size_bytes"))
        ):
            raise AuditVerificationError("model source hash descriptor is invalid")
        normalized.append(item)
    current_sources = model_source_artifacts()
    return normalized, normalized == current_sources


def verify_audit(path_value: str) -> dict[str, object]:
    path, raw = _regular_audit_bytes(path_value)
    try:
        audit = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_json_object)
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise AuditVerificationError("audit must be valid UTF-8 JSON with unique keys") from exc
    if not isinstance(audit, dict):
        raise AuditVerificationError("audit JSON must contain an object")
    if (
        isinstance(audit.get("schema_version"), bool)
        or not isinstance(audit.get("schema_version"), int)
        or audit.get("schema_version") not in (1, 2, 3, 4, 5, 6, 7, 8)
        or audit.get("kind") != "mujoco_pick_dynamics_audit"
    ):
        raise AuditVerificationError("unsupported dynamics audit schema")
    schema_version = audit["schema_version"]
    if (
        not isinstance(audit.get("audit_id"), str)
        or not re.fullmatch(r"[0-9a-f]{32}", audit["audit_id"])
        or audit.get("artifact_path") != path.relative_to(REPO_ROOT).as_posix()
    ):
        raise AuditVerificationError("audit ID or stored artifact path does not match the file")
    created_at = audit.get("created_at")
    if not isinstance(created_at, str):
        raise AuditVerificationError("audit created_at is missing")
    try:
        parsed_created_at = datetime.fromisoformat(created_at.replace("Z", "+00:00"))
    except ValueError as exc:
        raise AuditVerificationError("audit created_at is not an ISO-8601 timestamp") from exc
    if parsed_created_at.tzinfo is None:
        raise AuditVerificationError("audit created_at must include a timezone")

    expected_flags = {
        "simulation_only": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "candidate_verified_grasp": False,
        "continuous_collision_checked": False,
    }
    if any(audit.get(key) is not expected for key, expected in expected_flags.items()):
        raise AuditVerificationError("audit scope/safety flags do not match the review contract")
    if audit.get("scene") != "simulation/mujoco_omx/robotis_omx/shelf_pick_scene.xml":
        raise AuditVerificationError("audit scene path is unsupported")
    if not isinstance(audit.get("mujoco_version"), str) or not audit["mujoco_version"]:
        raise AuditVerificationError("MuJoCo version is missing")
    if (
        audit.get("contact_sampling")
        != "one contact snapshot after each MuJoCo mj_step; not continuous collision checking"
    ):
        raise AuditVerificationError("contact sampling scope is missing or overstates coverage")

    sources, sources_match_current = _validate_sources(audit.get("model_source_files"))
    dimensions = audit.get("model_dimensions")
    if not isinstance(dimensions, dict) or any(
        not _non_negative_integer(dimensions.get(key)) for key in ("nq", "nv", "nu")
    ) or not _finite_number(dimensions.get("physics_timestep_s")) or dimensions["physics_timestep_s"] <= 0:
        raise AuditVerificationError("model_dimensions is invalid")

    joints = audit.get("model_joint_goals_rad")
    if not isinstance(joints, dict) or set(joints) != set(ARM_JOINTS) or any(
        not _finite_number(joints.get(name)) for name in ARM_JOINTS
    ):
        raise AuditVerificationError("model_joint_goals_rad is invalid")
    if audit.get("joint_goal_source") not in (
        "explicit_simulation_joint_values",
        "hash_verified_offline_ik_candidate",
    ):
        raise AuditVerificationError("joint_goal_source is unsupported")

    candidate = audit.get("ik_candidate_input")
    if candidate is not None:
        if (
            not isinstance(candidate, dict)
            or not isinstance(candidate.get("path"), str)
            or not candidate["path"]
            or not isinstance(candidate.get("sha256"), str)
            or not SHA256_PATTERN.fullmatch(candidate["sha256"])
            or candidate.get("candidate_status")
            not in (
                "POSITION_CONVERGED_ONLY; PATH_AND_GRASP_UNVERIFIED",
                "POSE_CONVERGED_ONLY; PATH_AND_GRASP_UNVERIFIED",
            )
            or audit.get("joint_goal_source") != "hash_verified_offline_ik_candidate"
        ):
            raise AuditVerificationError("ik_candidate_input descriptor is invalid")
        if schema_version >= 2:
            target = candidate.get("target")
            orientation = target.get("orientation_quaternion_wxyz") if isinstance(target, dict) else None
            if (
                not isinstance(target, dict)
                or target.get("frame_id") != "world"
                or target.get("body") != "link6"
                or not _finite_vector(target.get("position_m"), 3)
                or (
                    orientation is not None
                    and not _finite_vector(orientation, 4)
                )
                or not _finite_vector(candidate.get("achieved_position_m"), 3)
                or not _finite_number(candidate.get("position_residual_m"))
                or candidate["position_residual_m"] < 0
            ):
                raise AuditVerificationError("candidate target provenance is invalid")
            object_center = audit.get("placeholder_geometry", {}).get("object_center_world_m")
            if not _finite_vector(object_center, 3):
                raise AuditVerificationError("candidate audit is missing its object-center source")
            if schema_version >= 3:
                if (
                    not _finite_number(candidate.get("target_to_object_center_distance_m"))
                    or candidate["target_to_object_center_distance_m"] < 0
                    or candidate.get("distance_scope")
                    != (
                        "Euclidean world distance from requested link6 body origin to initial "
                        "placeholder box center; not TCP offset or grasp clearance"
                    )
                ):
                    raise AuditVerificationError("candidate/object-center distance provenance is invalid")
                target_distance = math.dist(target["position_m"], object_center)
                if not math.isclose(
                    candidate["target_to_object_center_distance_m"],
                    target_distance,
                    rel_tol=0.0,
                    abs_tol=1e-9,
                ):
                    raise AuditVerificationError("candidate/object-center distance does not match its poses")
    elif audit.get("joint_goal_source") != "explicit_simulation_joint_values":
        raise AuditVerificationError("candidate-based goal source is missing its input descriptor")

    geometry = audit.get("placeholder_geometry")
    if schema_version >= 2 and (
        not isinstance(geometry, dict)
        or not _finite_vector(geometry.get("object_center_world_m"), 3)
    ):
        raise AuditVerificationError("schema v2 placeholder object center is invalid")

    steps = audit.get("step_counts")
    if not isinstance(steps, dict) or any(
        not isinstance(steps.get(key), int)
        or isinstance(steps.get(key), bool)
        or steps[key] < 1
        for key in (
            "approach_segments_per_joint",
            "close_steps",
            "settle_steps_each_stage",
            "lift_steps",
        )
    ):
        raise AuditVerificationError("step_counts is invalid")

    observations = audit.get("contact_observations")
    if not isinstance(observations, dict) or set(observations) != set(CONTACT_STAGES):
        raise AuditVerificationError("contact_observations stage set is invalid")
    metric_keys = (
        "sampled_steps",
        "object_shelf_support_steps",
        "object_jaw_contact_steps",
        "both_jaw_contact_steps",
        "object_other_contact_steps",
        "other_robot_or_environment_contact_steps",
    )
    for stage, metrics in observations.items():
        if not isinstance(metrics, dict) or any(
            not _non_negative_integer(metrics.get(key)) for key in metric_keys
        ):
            raise AuditVerificationError(f"contact metrics are invalid for stage {stage}")
        if metrics["sampled_steps"] < 1:
            raise AuditVerificationError(f"stage {stage} has no observed physics steps")
        if (
            metrics["object_shelf_support_steps"] > metrics["sampled_steps"]
            or metrics["object_jaw_contact_steps"] > metrics["sampled_steps"]
            or metrics["both_jaw_contact_steps"] > metrics["object_jaw_contact_steps"]
            or metrics["object_other_contact_steps"] > metrics["sampled_steps"]
            or metrics["other_robot_or_environment_contact_steps"] > metrics["sampled_steps"]
        ):
            raise AuditVerificationError(f"contact step counts are inconsistent for stage {stage}")
        pair_counts = metrics.get("contact_pair_contact_count")
        if not isinstance(pair_counts, dict) or any(
            not isinstance(pair, str) or not _non_negative_integer(count)
            for pair, count in pair_counts.items()
        ):
            raise AuditVerificationError(f"contact pair counts are invalid for stage {stage}")
        if schema_version >= 4:
            normal_samples = metrics.get("bilateral_jaw_normal_dot_samples")
            opposing_steps = metrics.get("opposing_jaw_normal_steps")
            penetration = metrics.get("max_object_jaw_penetration_m")
            dot_values = [
                metrics.get("bilateral_jaw_normal_dot_min"),
                metrics.get("bilateral_jaw_normal_dot_mean"),
                metrics.get("bilateral_jaw_normal_dot_max"),
            ]
            if (
                not _non_negative_integer(normal_samples)
                or normal_samples > metrics["both_jaw_contact_steps"]
                or not _non_negative_integer(opposing_steps)
                or opposing_steps > normal_samples
                or not _finite_number(penetration)
                or penetration < 0
            ):
                raise AuditVerificationError(f"jaw normal summary is invalid for stage {stage}")
            if normal_samples == 0:
                if any(value is not None for value in dot_values):
                    raise AuditVerificationError(f"empty jaw normal summary is invalid for stage {stage}")
            elif (
                any(not _finite_number(value) or not -1.000001 <= value <= 1.000001 for value in dot_values)
                or dot_values[0] > dot_values[1]
                or dot_values[1] > dot_values[2]
            ):
                raise AuditVerificationError(f"jaw normal dot range is invalid for stage {stage}")
        if schema_version >= 6:
            force_summary = metrics.get("jaw_contact_force_summary")
            if not isinstance(force_summary, dict) or set(force_summary) != {"link6", "link7"}:
                raise AuditVerificationError(f"jaw force summary is invalid for stage {stage}")
            for jaw, values in force_summary.items():
                if not isinstance(values, dict) or not _non_negative_integer(
                    values.get("contact_observations")
                ):
                    raise AuditVerificationError(f"jaw force observations are invalid for {stage}/{jaw}")
                force_fields = (
                    "mean_normal_force_n_per_contact_observation",
                    "max_normal_force_n",
                    "mean_tangential_force_n_per_contact_observation",
                    "max_tangential_force_n",
                )
                measurements = [values.get(key) for key in force_fields]
                if values["contact_observations"] == 0:
                    if any(value is not None for value in measurements):
                        raise AuditVerificationError(f"empty jaw force values are invalid for {stage}/{jaw}")
                elif (
                    any(not _finite_number(value) or value < 0 for value in measurements)
                    or measurements[0] > measurements[1]
                    or measurements[2] > measurements[3]
                ):
                    raise AuditVerificationError(f"jaw force values are invalid for {stage}/{jaw}")

    close_observation = audit.get("close_contact_observation")
    if (
        not isinstance(close_observation, dict)
        or not isinstance(close_observation.get("both_jaws_contacting"), bool)
        or not isinstance(close_observation.get("contact_count_by_other_body"), dict)
        or any(
            not isinstance(body, str)
            or not body
            or not _non_negative_integer(count)
            for body, count in close_observation["contact_count_by_other_body"].items()
        )
    ):
        raise AuditVerificationError("close_contact_observation is invalid")

    if schema_version >= 8:
        details = close_observation.get("object_contact_details")
        if (
            not isinstance(details, list)
            or len(details)
            != sum(close_observation["contact_count_by_other_body"].values())
        ):
            raise AuditVerificationError("close-state object contact details do not match contact counts")
        detail_counts: dict[str, int] = {}
        for detail in details:
            if (
                not isinstance(detail, dict)
                or not isinstance(detail.get("other_body"), str)
                or not detail["other_body"]
                or not isinstance(detail.get("object_geom"), str)
                or not detail["object_geom"]
                or not isinstance(detail.get("other_geom"), str)
                or not detail["other_geom"]
                or not _finite_number(detail.get("distance_m"))
                or not _finite_vector(detail.get("position_world_m"), 3)
                or not _finite_vector(detail.get("normal_object_to_other_world"), 3)
            ):
                raise AuditVerificationError("close-state object contact detail is invalid")
            normal = detail["normal_object_to_other_world"]
            normal_length = math.sqrt(sum(value * value for value in normal))
            if not math.isclose(normal_length, 1.0, rel_tol=0.0, abs_tol=1e-5):
                raise AuditVerificationError("close-state object contact normal is not unit length")
            body = detail["other_body"]
            detail_counts[body] = detail_counts.get(body, 0) + 1
        if detail_counts != close_observation["contact_count_by_other_body"]:
            raise AuditVerificationError("close-state object contact details disagree with body counts")

    if schema_version >= 7:
        close_state = audit.get("close_state")
        if not isinstance(close_state, dict):
            raise AuditVerificationError("close_state is missing")
        target_joints = close_state.get("arm_actuator_targets_rad")
        actual_joints = close_state.get("arm_joint_positions_rad")
        joint_deltas = close_state.get("arm_joint_delta_from_approach_rad")
        if any(
            not isinstance(values, dict)
            or set(values) != set(ARM_JOINTS)
            or any(not _finite_number(values.get(joint)) for joint in ARM_JOINTS)
            for values in (target_joints, actual_joints, joint_deltas)
        ):
            raise AuditVerificationError("close_state arm joint mappings are invalid")
        if any(
            not math.isclose(
                joint_deltas[joint],
                actual_joints[joint] - target_joints[joint],
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            for joint in ARM_JOINTS
        ):
            raise AuditVerificationError("close_state arm joint deltas do not match targets and positions")
        if (
            not _finite_number(close_state.get("gripper_actuator_target_rad"))
            or not math.isclose(
                close_state["gripper_actuator_target_rad"],
                audit.get("gripper_close_target_rad"),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            or not _finite_number(close_state.get("gripper_joint_position_rad"))
            or not _finite_number(close_state.get("gripper_mimic_joint_position_rad"))
        ):
            raise AuditVerificationError("close_state gripper target or actual positions are invalid")

    if schema_version >= 5:
        close_pose = audit.get("close_pose_observation")
        scope = (
            "MuJoCo body-origin transform at close-settle state; placeholder object and link6 only; "
            "not a physical TCP measurement or verified grasp transform"
        )
        pose_keys = ("object_pose_world", "link6_pose_world", "link6_pose_object")
        if (
            not isinstance(close_pose, dict)
            or close_pose.get("parent_body") != "placeholder_pick_object"
            or close_pose.get("child_body") != "link6"
            or close_pose.get("scope") != scope
            or any(not isinstance(close_pose.get(key), dict) for key in pose_keys)
        ):
            raise AuditVerificationError("close_pose_observation is invalid")
        poses = {key: close_pose[key] for key in pose_keys}
        for pose_name, pose in poses.items():
            if (
                not _finite_vector(pose.get("position_m"), 3)
                or not _finite_vector(pose.get("quaternion_wxyz"), 4)
                or not math.isclose(
                    math.sqrt(sum(component * component for component in pose["quaternion_wxyz"])),
                    1.0,
                    rel_tol=0.0,
                    abs_tol=1e-6,
                )
            ):
                raise AuditVerificationError(f"close pose is invalid: {pose_name}")
        parent = poses["object_pose_world"]
        child = poses["link6_pose_world"]
        relative = poses["link6_pose_object"]
        rotation = _rotation_matrix_wxyz(parent["quaternion_wxyz"])
        delta = [
            child["position_m"][index] - parent["position_m"][index]
            for index in range(3)
        ]
        expected_position = [
            sum(rotation[row][column] * delta[row] for row in range(3))
            for column in range(3)
        ]
        if any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-8)
            for actual, expected in zip(relative["position_m"], expected_position)
        ):
            raise AuditVerificationError("close relative position does not match world poses")
        parent_quaternion = parent["quaternion_wxyz"]
        parent_conjugate = [parent_quaternion[0], *(-component for component in parent_quaternion[1:])]
        expected_quaternion = _relative_quaternion_wxyz(
            parent_conjugate, child["quaternion_wxyz"]
        )
        quaternion_error = min(
            math.sqrt(sum((actual - expected) ** 2 for actual, expected in zip(relative["quaternion_wxyz"], expected_quaternion))),
            math.sqrt(sum((actual + expected) ** 2 for actual, expected in zip(relative["quaternion_wxyz"], expected_quaternion))),
        )
        if quaternion_error > 1e-8:
            raise AuditVerificationError("close relative orientation does not match world poses")

    lift_ik = audit.get("lift_ik")
    if (
        not isinstance(lift_ik, dict)
        or lift_ik.get("method") not in ("position_only", "weighted_position_and_orientation")
        or not _non_negative_integer(lift_ik.get("iterations"))
        or lift_ik["iterations"] < 1
        or not isinstance(lift_ik.get("joint_delta_rad"), dict)
        or set(lift_ik["joint_delta_rad"]) != set(ARM_JOINTS)
        or any(not _finite_number(lift_ik["joint_delta_rad"].get(name)) for name in ARM_JOINTS)
        or any(
            not _finite_number(lift_ik.get(key)) or lift_ik[key] < 0
            for key in (
                "orientation_residual_rad",
                "orientation_scale_m_per_rad",
                "position_residual_m",
            )
        )
        or not _finite_number(lift_ik.get("requested_height_m"))
        or lift_ik["requested_height_m"] <= 0
    ):
        raise AuditVerificationError("lift_ik is invalid")

    lift = audit.get("lift_observation")
    if (
        not isinstance(lift, dict)
        or lift.get("lift_steps") != steps["lift_steps"]
        or not _non_negative_integer(lift.get("both_jaw_contact_steps"))
        or lift["both_jaw_contact_steps"] > lift["lift_steps"]
        or not isinstance(lift.get("final_object_contacts"), dict)
        or not isinstance(lift["final_object_contacts"].get("both_jaws_contacting"), bool)
        or not isinstance(lift["final_object_contacts"].get("contact_count_by_other_body"), dict)
        or any(
            not _finite_number(lift.get(key))
            for key in (
                "object_bottom_clearance_final_above_shelf_m",
                "object_bottom_clearance_max_above_shelf_m",
                "object_z_change_final_m",
                "object_z_final_m",
                "object_z_max_m",
                "object_z_min_m",
            )
        )
    ):
        raise AuditVerificationError("lift_observation is invalid")

    if any(
        not _finite_number(audit.get(key))
        for key in ("gripper_open_target_rad", "gripper_close_target_rad", "object_z_after_close_m", "object_z_before_close_m")
    ):
        raise AuditVerificationError("gripper targets or close-stage object heights are invalid")

    audit_sha256 = hashlib.sha256(raw).hexdigest()
    return {
        "valid": True,
        "audit_id": audit["audit_id"],
        "artifact_path": path.relative_to(REPO_ROOT).as_posix(),
        "audit_sha256": audit_sha256,
        "source_files_match_current_model": sources_match_current,
        "source_file_count": len(sources),
        "simulation_only": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "candidate_verified_grasp": False,
        "report": (
            "AUDIT_VALID_CURRENT_MODEL"
            if sources_match_current
            else "AUDIT_VALID_BUT_SOURCE_FILES_DIFFER_FROM_CURRENT_MODEL"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audit_json", help="saved JSON directly under the managed audit directory")
    args = parser.parse_args()
    try:
        print(json.dumps(verify_audit(args.audit_json), indent=2, sort_keys=True))
        return 0
    except (OSError, AuditVerificationError, ValueError, RecursionError) as exc:
        print(f"MuJoCo audit verification failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
