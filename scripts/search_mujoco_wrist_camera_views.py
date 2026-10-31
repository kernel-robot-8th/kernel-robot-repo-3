#!/usr/bin/env python3
"""Search synthetic link6 camera attachment candidates across fixed IK states."""

from __future__ import annotations

import hashlib
import json
import math
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation/mujoco_omx/robotis_omx/shelf_pick_scene.xml"
CANDIDATE_ROOT = REPO_ROOT / ".control-station/mujoco-ik-candidates"
OUTPUT_ROOT = REPO_ROOT / ".control-station/mujoco-wrist-view-candidates"
CAMERA_NAME = "placeholder_wrist_camera"
BODY_NAME = "link6"
TARGET_GEOM_NAME = "placeholder_pick_object_geom"
TARGET_CENTER_M = (0.05, 0.25, 0.27)
JOINT_NAMES = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")
STATE_FILES = (
    "staged-front-edge-z40.json",
    "staged-y145-z40.json",
    "staged-y145-z32.json",
    "staged-y18-z32.json",
    "staged-y21-z32.json",
)
POSITION_GRID_M = {
    "x": (0.0, 0.04, 0.08, 0.12),
    "y": (-0.08, -0.04, 0.0, 0.04, 0.08),
    "z": (0.02, 0.07, 0.12),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"IK candidate contains duplicate JSON key: {key}")
        result[key] = value
    return result


def _model_sources() -> list[dict[str, Any]]:
    model_directory = SCENE_PATH.parent
    paths = [SCENE_PATH, model_directory / "omx.xml"]
    paths.extend(path for path in (model_directory / "assets").rglob("*") if path.is_file())
    return [
        {
            "path": path.relative_to(REPO_ROOT).as_posix(),
            "path_base": "REPOSITORY_ROOT",
            "hash_algorithm": "sha256",
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
        for path in sorted(paths, key=lambda item: item.relative_to(REPO_ROOT).as_posix())
    ]


def _read_states(model_sources: list[dict[str, Any]], mujoco_version: str) -> list[dict[str, Any]]:
    states = []
    for filename in STATE_FILES:
        path = CANDIDATE_ROOT / filename
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 1_000_000:
            raise ValueError(f"IK candidate is missing, oversized, or not a regular file: {filename}")
        with path.open("rb") as source:
            raw = source.read(1_000_001)
        if len(raw) > 1_000_000:
            raise ValueError(f"IK candidate exceeds the 1 MB limit: {filename}")
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_json_object)
        if (
            not isinstance(value, dict)
            or value.get("schema_version") != 2
            or value.get("kind") != "mujoco_ik_candidate"
            or value.get("simulation_only") is not True
            or value.get("usable_for_robot_control") is not False
            or value.get("arm_command_sent") is not False
            or value.get("mujoco_version") != mujoco_version
            or value.get("model_source_files") != model_sources
            or value.get("pose_frame") != "link6"
            or value.get("body") != BODY_NAME
            or value.get("pose_converged") is not True
            or not isinstance(value.get("contacts"), dict)
            or value["contacts"].get("final_pose") != []
        ):
            raise ValueError(f"IK candidate failed simulation/source/contact provenance checks: {filename}")
        joints = value.get("joint_candidate_rad")
        if not isinstance(joints, dict) or set(joints) != set(JOINT_NAMES):
            raise ValueError(f"IK candidate must contain exactly the five arm joints: {filename}")
        for name, position in joints.items():
            if (
                isinstance(position, bool)
                or not isinstance(position, (int, float))
                or not math.isfinite(position)
            ):
                raise ValueError(f"IK candidate has an invalid joint position for {name}: {filename}")
        states.append(
            {"name": filename, "sha256": hashlib.sha256(raw).hexdigest(), "joints": joints}
        )
    return states


def _aimed_local_quaternion(
    mujoco: Any, body_quaternion: Any, camera_position_world: Any, np: Any
) -> Any:
    direction = camera_position_world - np.asarray(TARGET_CENTER_M, dtype=np.float64)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-8:
        raise ValueError("candidate camera origin overlaps the placeholder target center")
    z_axis = direction / norm  # MuJoCo cameras look along local -Z.
    up = np.asarray((0.0, 0.0, 1.0), dtype=np.float64)
    y_axis = up - z_axis * float(np.dot(up, z_axis))
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm < 1e-8:
        up = np.asarray((0.0, 1.0, 0.0), dtype=np.float64)
        y_axis = up - z_axis * float(np.dot(up, z_axis))
        y_norm = float(np.linalg.norm(y_axis))
    y_axis /= y_norm
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    world_rotation = np.column_stack((x_axis, y_axis, z_axis))
    world_quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(world_quaternion, world_rotation.ravel())
    inverse_body_quaternion = body_quaternion.copy()
    inverse_body_quaternion[1:] *= -1.0
    local_quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mulQuat(local_quaternion, inverse_body_quaternion, world_quaternion)
    return local_quaternion


def _state_view(
    renderer: Any,
    model: Any,
    data: Any,
    camera_id: int,
    geom_id: int,
    geom_type: int,
    qpos: np.ndarray,
) -> dict[str, Any]:
    import mujoco
    import numpy as np

    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=camera_id)
    renderer.enable_segmentation_rendering()
    segmentation = renderer.render().copy()
    renderer.disable_segmentation_rendering()
    rows, columns = np.nonzero(
        (segmentation[..., 0] == geom_id) & (segmentation[..., 1] == geom_type)
    )
    bbox = (
        [int(columns.min()), int(rows.min()), int(columns.max()), int(rows.max())]
        if rows.size
        else None
    )
    margin = (
        min(bbox[0], bbox[1], renderer.width - 1 - bbox[2], renderer.height - 1 - bbox[3])
        if bbox
        else -1
    )
    return {
        "visible_target_pixel_count": int(rows.size),
        "visible_target_bbox_xyxy": bbox,
        "minimum_frame_edge_margin_px": int(margin),
    }


def main() -> int:
    try:
        import mujoco
        import numpy as np
    except ImportError:
        print(
            "MuJoCo/NumPy is not installed; install requirements-mujoco.txt in the simulator environment.",
            file=sys.stderr,
        )
        return 2
    try:
        if CANDIDATE_ROOT.parent.is_symlink() or CANDIDATE_ROOT.is_symlink():
            raise ValueError("IK candidate directory cannot be a symbolic link")
        model_sources = _model_sources()
        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        states = _read_states(model_sources, str(mujoco.__version__))
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BODY_NAME)
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TARGET_GEOM_NAME)
        if min(body_id, camera_id, geom_id) < 0:
            raise ValueError("placeholder link6 camera or target geom is missing from the MuJoCo scene")
        geom_type = int(mujoco.mjtObj.mjOBJ_GEOM)
        data = mujoco.MjData(model)
        state_qpos = []
        for state in states:
            for name, value in state["joints"].items():
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                lower, upper = model.jnt_range[joint_id]
                if not lower <= value <= upper:
                    raise ValueError(f"IK candidate joint position is outside the MuJoCo range: {name}")
                data.qpos[model.jnt_qposadr[joint_id]] = value
            mujoco.mj_forward(model, data)
            if data.ncon:
                raise ValueError(f"IK candidate endpoint has MuJoCo contacts: {state['name']}")
            state_qpos.append(data.qpos.copy())

        results = []
        width, height = 320, 240
        with mujoco.Renderer(model, height=height, width=width) as renderer:
            for x in POSITION_GRID_M["x"]:
                for y in POSITION_GRID_M["y"]:
                    for z in POSITION_GRID_M["z"]:
                        local_position = np.asarray((x, y, z), dtype=np.float64)
                        data.qpos[:] = state_qpos[-1]
                        mujoco.mj_forward(model, data)
                        body_rotation = data.xmat[body_id].reshape(3, 3)
                        world_position = data.xpos[body_id] + body_rotation @ local_position
                        local_quaternion = _aimed_local_quaternion(
                            mujoco, data.xquat[body_id].copy(), world_position, np
                        )
                        model.cam_pos[camera_id] = local_position
                        model.cam_quat[camera_id] = local_quaternion
                        views = []
                        for state, qpos in zip(states, state_qpos, strict=True):
                            views.append(
                                {
                                    "ik_candidate": state["name"],
                                    "ik_candidate_sha256": state["sha256"],
                                    **_state_view(
                                        renderer,
                                        model,
                                        data,
                                        camera_id,
                                        geom_id,
                                        geom_type,
                                        qpos,
                                    ),
                                }
                            )
                        results.append(
                            {
                                "position_in_link6_m": [float(value) for value in local_position],
                                "quaternion_wxyz_in_link6": [float(value) for value in local_quaternion],
                                "camera_world_position_at_last_state_m": [
                                    float(value) for value in world_position
                                ],
                                "minimum_visible_target_pixels": min(
                                    item["visible_target_pixel_count"] for item in views
                                ),
                                "minimum_frame_edge_margin_px": min(
                                    item["minimum_frame_edge_margin_px"] for item in views
                                ),
                                "visible_in_all_sampled_states": all(
                                    item["visible_target_pixel_count"] > 0 for item in views
                                ),
                                "bbox_interior_in_all_sampled_states": all(
                                    item["minimum_frame_edge_margin_px"] >= 5 for item in views
                                ),
                                "state_views": views,
                            }
                        )
        results.sort(
            key=lambda item: (
                item["bbox_interior_in_all_sampled_states"],
                item["minimum_visible_target_pixels"],
                item["minimum_frame_edge_margin_px"],
            ),
            reverse=True,
        )
        if OUTPUT_ROOT.parent.is_symlink() or OUTPUT_ROOT.is_symlink():
            raise ValueError("camera view candidate output directory cannot be a symbolic link")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        output = OUTPUT_ROOT / (
            f"candidate-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}"
            f"-{uuid.uuid4().hex[:8]}.json"
        )
        record = {
            "schema_version": 1,
            "kind": "mujoco_wrist_camera_view_search",
            "created_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "simulator": "MuJoCo",
            "mujoco_version": str(mujoco.__version__),
            "scene": SCENE_PATH.relative_to(REPO_ROOT).as_posix(),
            "model_source_files": model_sources,
            "sampled_ik_candidates": states,
            "camera": {
                "name": CAMERA_NAME,
                "parent_body": BODY_NAME,
                "fovy_degrees": float(model.cam_fovy[camera_id]),
                "position_grid_m_in_link6": POSITION_GRID_M,
                "orientation_method": "look_at_placeholder_object_center_from_last_sampled_state",
                "target_geom": TARGET_GEOM_NAME,
                "target_center_m_placeholder": list(TARGET_CENTER_M),
                "image_size_px": {"width": width, "height": height},
            },
            "positions_sampled": len(results),
            "visible_in_all_sampled_states_count": sum(
                item["visible_in_all_sampled_states"] for item in results
            ),
            "bbox_interior_in_all_sampled_states_count": sum(
                item["bbox_interior_in_all_sampled_states"] for item in results
            ),
            "ranked_candidates": results,
            "simulation_only": True,
            "physical_camera_pose_verified": False,
            "usable_for_robot_control": False,
            "arm_command_sent": False,
            "review_required": True,
        }
        with output.open("x", encoding="utf-8") as destination:
            json.dump(record, destination, ensure_ascii=False, indent=2, allow_nan=False)
            destination.write("\n")
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"MuJoCo wrist-view search failed: {exc}", file=sys.stderr)
        return 2
    print(f"simulation-only wrist camera candidate record: {output.relative_to(REPO_ROOT)}")
    print(f"camera positions sampled: {len(results)}")
    print(f"target visible in all five states: {record['visible_in_all_sampled_states_count']}")
    print(f"target bbox interior in all five states: {record['bbox_interior_in_all_sampled_states_count']}")
    print("No ROS connection, actuator target, physical camera, or robot command was used.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
