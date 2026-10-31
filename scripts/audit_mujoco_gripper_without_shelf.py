#!/usr/bin/env python3
"""Probe one saved close posture with placeholder shelf collisions disabled."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import mujoco

from mujoco_model_sources import model_source_artifacts
from verify_mujoco_pick_dynamics_audit import verify_audit


REPO_ROOT = Path(__file__).resolve().parents[1]
AUDIT_DIRECTORY = REPO_ROOT / ".control-station" / "mujoco-dynamics-audits"
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"
ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audit_json", type=Path, help="verified managed dynamics audit")
    args = parser.parse_args()

    audit_path = args.audit_json
    if not audit_path.is_absolute():
        audit_path = (Path.cwd() / audit_path).absolute()
    try:
        verified = verify_audit(str(audit_path))
    except (OSError, ValueError) as exc:
        parser.error(f"input must be a valid managed dynamics audit: {exc}")
    if verified["source_files_match_current_model"] is not True:
        parser.error("input audit model sources do not match the current checkout")

    raw = audit_path.read_bytes()
    source_audit_sha256 = hashlib.sha256(raw).hexdigest()
    if source_audit_sha256 != verified["audit_sha256"]:
        parser.error("input audit changed after verification")
    try:
        audit = json.loads(raw.decode("utf-8"))
    except (UnicodeError, ValueError) as exc:
        parser.error(f"input audit is not readable JSON: {exc}")

    model_sources = model_source_artifacts()
    if audit.get("model_source_files") != model_sources:
        parser.error("input audit model source inventory does not match the current checkout")

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_id < 0:
        parser.error("MuJoCo model does not contain the required home keyframe")
    mujoco.mj_resetDataKeyframe(model, data, home_id)

    disabled_shelf_geoms: list[str] = []
    for geom_id in range(model.ngeom):
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if geom_name.startswith("placeholder_shelf_"):
            disabled_shelf_geoms.append(geom_name)
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0
    if not disabled_shelf_geoms:
        parser.error("no placeholder shelf collision geometries were found")

    joint_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in (*ARM_JOINTS, "Gripper", "Gripper_mimic")
    }
    if any(joint_id < 0 for joint_id in joint_ids.values()):
        parser.error("MuJoCo model is missing a required OMX joint")
    qpos_ids = {
        name: int(model.jnt_qposadr[joint_ids[name]]) for name in joint_ids
    }
    dof_ids = {
        name: int(model.jnt_dofadr[joint_ids[name]]) for name in joint_ids
    }
    actuator_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in (*ARM_JOINTS, "Gripper")
    }
    if any(actuator_id < 0 for actuator_id in actuator_ids.values()):
        parser.error("MuJoCo model is missing a required OMX actuator")

    object_body = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "placeholder_pick_object"
    )
    free_joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "placeholder_pick_object_free"
    )
    if min(object_body, free_joint) < 0:
        parser.error("MuJoCo model is missing the placeholder object")
    object_qpos = int(model.jnt_qposadr[free_joint])
    object_dof = int(model.jnt_dofadr[free_joint])

    close_state = audit.get("close_state")
    initial_center = audit.get("placeholder_geometry", {}).get(
        "object_center_world_m"
    )
    if (
        not isinstance(close_state, dict)
        or not isinstance(initial_center, list)
        or len(initial_center) != 3
    ):
        parser.error("input audit lacks close state or initial object center")
    arm_positions = close_state.get("arm_joint_positions_rad")
    if not isinstance(arm_positions, dict) or set(arm_positions) != set(ARM_JOINTS):
        parser.error("input audit close arm positions are invalid")
    open_target = audit.get("gripper_open_target_rad")
    close_target = audit.get("gripper_close_target_rad")
    step_counts = audit.get("step_counts")
    if not isinstance(open_target, (int, float)) or not isinstance(close_target, (int, float)):
        parser.error("input audit gripper targets are invalid")
    if not isinstance(step_counts, dict):
        parser.error("input audit step counts are invalid")
    close_steps = step_counts.get("close_steps")
    settle_steps = step_counts.get("settle_steps_each_stage")
    if (
        isinstance(close_steps, bool)
        or not isinstance(close_steps, int)
        or close_steps < 1
        or isinstance(settle_steps, bool)
        or not isinstance(settle_steps, int)
        or settle_steps < 1
    ):
        parser.error("input audit close/settle step counts are invalid")

    for name in ARM_JOINTS:
        position = float(arm_positions[name])
        data.qpos[qpos_ids[name]] = position
        data.qvel[dof_ids[name]] = 0.0
        data.ctrl[actuator_ids[name]] = position
    data.qpos[qpos_ids["Gripper"]] = float(open_target)
    data.qpos[qpos_ids["Gripper_mimic"]] = -float(open_target)
    data.qvel[dof_ids["Gripper"]] = 0.0
    data.qvel[dof_ids["Gripper_mimic"]] = 0.0
    data.ctrl[actuator_ids["Gripper"]] = float(open_target)
    data.qpos[object_qpos : object_qpos + 7] = [
        float(value) for value in initial_center
    ] + [1.0, 0.0, 0.0, 0.0]
    data.qvel[object_dof : object_dof + 6] = 0.0
    mujoco.mj_forward(model, data)

    object_body_name = "placeholder_pick_object"
    jaw_names = {"link6", "link7"}
    pair_counts: dict[str, Counter[str]] = {
        "close": Counter(),
        "settle": Counter(),
    }
    bilateral_steps = {"close": 0, "settle": 0}
    object_z_min = float(data.qpos[object_qpos + 2])
    object_z_max = object_z_min

    def observe(stage: str) -> None:
        nonlocal object_z_min, object_z_max
        contacting_jaws: set[str] = set()
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            bodies = tuple(
                mujoco.mj_id2name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.geom_bodyid[geom_id]),
                )
                for geom_id in geom_ids
            )
            if object_body_name not in bodies:
                continue
            other_body = next(
                (body for body in bodies if body != object_body_name), "unknown"
            )
            geom_names = tuple(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                or f"unnamed_geom:{geom_id}"
                for geom_id in geom_ids
            )
            pair_counts[stage][
                f"{other_body}:" + "/".join(geom_names)
            ] += 1
            if other_body in jaw_names:
                contacting_jaws.add(other_body)
        if contacting_jaws == jaw_names:
            bilateral_steps[stage] += 1
        object_z = float(data.qpos[object_qpos + 2])
        object_z_min = min(object_z_min, object_z)
        object_z_max = max(object_z_max, object_z)

    initial_object_z = float(data.qpos[object_qpos + 2])
    for index in range(1, close_steps + 1):
        fraction = index / close_steps
        data.ctrl[actuator_ids["Gripper"]] = float(open_target) + (
            float(close_target) - float(open_target)
        ) * fraction
        mujoco.mj_step(model, data)
        observe("close")
    for _ in range(settle_steps):
        data.ctrl[actuator_ids["Gripper"]] = float(close_target)
        mujoco.mj_step(model, data)
        observe("settle")

    final_contacts: Counter[str] = Counter()
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom_ids = (int(contact.geom1), int(contact.geom2))
        bodies = tuple(
            mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                int(model.geom_bodyid[geom_id]),
            )
            for geom_id in geom_ids
        )
        if object_body_name in bodies:
            other = next(
                (body for body in bodies if body != object_body_name), "unknown"
            )
            geom_names = tuple(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                or f"unnamed_geom:{geom_id}"
                for geom_id in geom_ids
            )
            final_contacts[f"{other}:" + "/".join(geom_names)] += 1

    result = {
        "schema_version": 1,
        "kind": "mujoco_gripper_without_shelf_diagnostic",
        "source_audit_path": audit_path.relative_to(REPO_ROOT).as_posix(),
        "source_audit_sha256": source_audit_sha256,
        "source_audit_id": audit.get("audit_id"),
        "model_source_files": model_sources,
        "mujoco_version": str(mujoco.__version__),
        "diagnostic_conditions": {
            "disabled_shelf_collision_geoms": disabled_shelf_geoms,
            "gravity_m_s2": model.opt.gravity.tolist(),
            "object_pose_source": "initial placeholder object center from source audit; identity orientation from scene keyframe",
            "arm_pose_source": "measured MuJoCo close-settle qpos recorded in source audit",
            "arm_motion_during_diagnostic": False,
            "floor_collision_enabled": True,
        },
        "initial_object_center_z_m": initial_object_z,
        "final_object_center_z_m": float(data.qpos[object_qpos + 2]),
        "object_center_z_min_m": object_z_min,
        "object_center_z_max_m": object_z_max,
        "bilateral_jaw_contact_steps": bilateral_steps,
        "object_contact_pair_observation_counts": {
            stage: dict(sorted(counts.items()))
            for stage, counts in pair_counts.items()
        },
        "final_object_contacts": dict(sorted(final_contacts.items())),
        "simulation_only": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
