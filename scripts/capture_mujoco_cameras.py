#!/usr/bin/env python3
"""Capture synthetic MuJoCo fixed/wrist RGB, metric-depth, and segmentation frames."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
import sys
import uuid
import zlib
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"
CAMERA_NAMES = (
    "placeholder_fixed_camera",
    "placeholder_wrist_camera",
    "placeholder_wrist_aimed_candidate",
)
JOINT_NAMES = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Gripper")
ARM_JOINT_NAMES = JOINT_NAMES[:-1]
MAX_IK_CANDIDATE_BYTES = 1_000_000


def _parse_target(value: str) -> tuple[str, float]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("target format is JOINT=RADIANS")
    name, raw_radians = value.split("=", 1)
    if name not in JOINT_NAMES:
        raise argparse.ArgumentTypeError(f"unsupported simulation joint: {name}")
    try:
        radians = float(raw_radians)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("target radians must be numeric") from exc
    if not math.isfinite(radians):
        raise argparse.ArgumentTypeError("target radians must be finite")
    return name, radians


def _png_chunk(name: bytes, data: bytes) -> bytes:
    checksum = zlib.crc32(name + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + name + data + struct.pack(">I", checksum)


def _write_rgb_png(path: Path, rgb: Any) -> None:
    import numpy as np

    pixels = np.asarray(rgb)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
        raise ValueError("MuJoCo RGB capture must be an HxWx3 uint8 array")
    height, width = pixels.shape[:2]
    scanlines = b"".join(b"\x00" + pixels[row].tobytes(order="C") for row in range(height))
    png = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(scanlines, level=6))
        + _png_chunk(b"IEND", b"")
    )
    with path.open("xb") as output:
        output.write(png)


def _new_capture_directory(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        candidate = root / f"capture-{stamp}-{uuid.uuid4().hex[:8]}"
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue


def _joint_state(mujoco: Any, model: Any, data: Any) -> dict[str, float]:
    state: dict[str, float] = {}
    for name in JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"simulation joint is missing: {name}")
        qpos_address = int(model.jnt_qposadr[joint_id])
        state[name] = float(data.qpos[qpos_address])
    return state


def _geom_segmentation_map(mujoco: Any, model: Any) -> list[dict[str, object]]:
    geom_object_type = int(mujoco.mjtObj.mjOBJ_GEOM)
    return [
        {
            "object_id": geom_id,
            "object_type": geom_object_type,
            "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id),
        }
        for geom_id in range(model.ngeom)
    ]


def _file_artifact(
    path: Path, base_directory: Path, path_base: str
) -> dict[str, object]:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return {
        "path": path.relative_to(base_directory).as_posix(),
        "path_base": path_base,
        "hash_algorithm": "sha256",
        "sha256": digest.hexdigest(),
        "size_bytes": path.stat().st_size,
    }


def _scene_source_files() -> list[Path]:
    model_directory = SCENE_PATH.parent
    sources = [SCENE_PATH, model_directory / "omx.xml"]
    assets_directory = model_directory / "assets"
    sources.extend(path for path in assets_directory.rglob("*") if path.is_file())
    return sorted(sources, key=lambda path: path.relative_to(REPO_ROOT).as_posix())


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    value: dict[str, object] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"IK candidate contains a duplicate JSON key: {key}")
        value[key] = item
    return value


def _read_ik_candidate(path: Path) -> tuple[dict[str, object], bytes, str]:
    candidate_root = REPO_ROOT / ".control-station" / "mujoco-ik-candidates"
    source_path = path if path.is_absolute() else REPO_ROOT / path
    if (
        candidate_root.parent.is_symlink()
        or candidate_root.is_symlink()
        or source_path.absolute().parent != candidate_root
    ):
        raise ValueError("IK candidate directory cannot be a symbolic link")
    try:
        resolved_candidate = source_path.resolve(strict=True)
        resolved_root = candidate_root.resolve(strict=False)
    except OSError as exc:
        raise ValueError("IK candidate path cannot be resolved") from exc
    if (
        source_path.is_symlink()
        or resolved_candidate.parent != resolved_root
        or resolved_candidate.suffix != ".json"
        or not resolved_candidate.is_file()
    ):
        raise ValueError("IK candidate must be a direct regular JSON file under .control-station/mujoco-ik-candidates")
    try:
        if resolved_candidate.stat().st_size > MAX_IK_CANDIDATE_BYTES:
            raise ValueError("IK candidate JSON exceeds the 1 MB limit")
        with resolved_candidate.open("rb") as candidate_file:
            raw = candidate_file.read(MAX_IK_CANDIDATE_BYTES + 1)
        if len(raw) > MAX_IK_CANDIDATE_BYTES:
            raise ValueError("IK candidate JSON exceeds the 1 MB limit")
        candidate = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
    except (OSError, UnicodeError, json.JSONDecodeError, RecursionError, ValueError) as exc:
        raise ValueError("IK candidate JSON is missing or invalid") from exc
    if not isinstance(candidate, dict) or (
        isinstance(candidate.get("schema_version"), bool)
        or not isinstance(candidate.get("schema_version"), int)
        or candidate.get("schema_version") != 2
        or candidate.get("kind") != "mujoco_ik_candidate"
        or candidate.get("simulation_only") is not True
        or candidate.get("usable_for_robot_control") is not False
        or candidate.get("arm_command_sent") is not False
        or candidate.get("model") != SCENE_PATH.relative_to(REPO_ROOT).as_posix()
        or candidate.get("pose_frame") != "link6"
        or candidate.get("body") != "link6"
        or candidate.get("pose_converged") is not True
    ):
        raise ValueError("IK candidate is not a converged simulation-only link6 candidate")
    contacts = candidate.get("contacts")
    if not isinstance(contacts, dict) or contacts.get("final_pose") != []:
        raise ValueError("IK candidate endpoint must be recorded without contact")
    if not isinstance(candidate.get("joint_candidate_rad"), dict):
        raise ValueError("IK candidate joint_candidate_rad is invalid")
    return candidate, raw, hashlib.sha256(raw).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=640, help="render width in pixels")
    parser.add_argument("--height", type=int, default=480, help="render height in pixels")
    parser.add_argument("--steps", type=int, default=500, help="simulation steps before capture")
    parser.add_argument(
        "--target",
        action="append",
        type=_parse_target,
        default=[],
        metavar="JOINT=RADIANS",
        help="optional MuJoCo-only actuator target; may be repeated",
    )
    parser.add_argument(
        "--ik-candidate",
        type=Path,
        help="render the endpoint of a converged, endpoint-contact-free link6 IK candidate JSON",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(".control-station/mujoco-camera-captures"),
        help="directory under which a unique capture directory is created",
    )
    args = parser.parse_args()
    if not 32 <= args.width <= 2048 or not 32 <= args.height <= 2048:
        parser.error("width and height must be between 32 and 2048 pixels")
    if not 0 <= args.steps <= 100_000:
        parser.error("steps must be between 0 and 100000")
    targets = dict(args.target)
    if len(targets) != len(args.target):
        parser.error("each simulation joint may be specified at most once")
    ik_candidate = None
    ik_candidate_bytes = None
    ik_candidate_sha256 = None
    if args.ik_candidate is not None:
        if targets:
            parser.error("--ik-candidate cannot be combined with --target")
        if args.steps != 0:
            parser.error("--ik-candidate requires --steps 0 to capture its static endpoint")
        try:
            ik_candidate, ik_candidate_bytes, ik_candidate_sha256 = _read_ik_candidate(
                args.ik_candidate
            )
        except (OSError, ValueError) as exc:
            parser.error(str(exc))

    try:
        import mujoco
        import numpy as np
    except ImportError:
        print(
            "MuJoCo and NumPy are optional. Install requirements-mujoco.txt in the simulator environment.",
            file=sys.stderr,
        )
        return 2

    renderer = None
    try:
        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        data = mujoco.MjData(model)
        home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if home_key >= 0:
            mujoco.mj_resetDataKeyframe(model, data, home_key)
        if ik_candidate is not None:
            if ik_candidate.get("mujoco_version") != mujoco.__version__:
                raise ValueError("IK candidate MuJoCo version differs from the capture runtime")
            current_sources = [
                _file_artifact(path, REPO_ROOT, "REPOSITORY_ROOT")
                for path in _scene_source_files()
            ]
            if ik_candidate.get("model_source_files") != current_sources:
                raise ValueError("IK candidate scene/model source hashes are stale")
            joint_values = ik_candidate["joint_candidate_rad"]
            assert isinstance(joint_values, dict)
            if set(joint_values) != set(ARM_JOINT_NAMES):
                raise ValueError("IK candidate must contain exactly the five OMX arm joints")
            for name in ARM_JOINT_NAMES:
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                    raise ValueError(f"simulation hinge joint is missing: {name}")
                raw_radians = joint_values[name]
                if isinstance(raw_radians, bool) or not isinstance(raw_radians, (int, float)):
                    raise ValueError(f"IK candidate has an invalid position for {name}")
                try:
                    radians = float(raw_radians)
                except (OverflowError, ValueError) as exc:
                    raise ValueError(f"IK candidate has an invalid position for {name}") from exc
                if not math.isfinite(radians):
                    raise ValueError(f"IK candidate has a non-finite position for {name}")
                if model.jnt_limited[joint_id] and not (
                    model.jnt_range[joint_id, 0] <= radians <= model.jnt_range[joint_id, 1]
                ):
                    raise ValueError(f"IK candidate position is outside the MuJoCo range for {name}")
                data.qpos[int(model.jnt_qposadr[joint_id])] = radians
        for name, radians in targets.items():
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if actuator_id < 0:
                raise ValueError(f"simulation actuator is missing: {name}")
            if model.actuator_ctrllimited[actuator_id] and not (
                model.actuator_ctrlrange[actuator_id, 0]
                <= radians
                <= model.actuator_ctrlrange[actuator_id, 1]
            ):
                raise ValueError(f"{name}={radians} is outside the MuJoCo actuator range")
            data.ctrl[actuator_id] = radians
        for _ in range(args.steps):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        if ik_candidate is not None and data.ncon != 0:
            raise ValueError(
                f"IK candidate endpoint has {data.ncon} MuJoCo contact(s); refusing candidate capture"
            )

        for camera_name in CAMERA_NAMES:
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if camera_id < 0:
                raise ValueError(f"simulation camera is missing: {camera_name}")

        capture_directory = _new_capture_directory(args.output_root)
        candidate_artifact = None
        if ik_candidate_bytes is not None:
            candidate_copy_path = capture_directory / "ik_candidate.json"
            with candidate_copy_path.open("xb") as candidate_copy:
                candidate_copy.write(ik_candidate_bytes)
            candidate_artifact = _file_artifact(
                candidate_copy_path, capture_directory, "CAPTURE_DIRECTORY"
            )
        renderer = mujoco.Renderer(model, height=args.height, width=args.width)
        camera_manifest: dict[str, dict[str, object]] = {}
        for camera_name in CAMERA_NAMES:
            renderer.update_scene(data, camera=camera_name)
            rgb = renderer.render().copy()
            _write_rgb_png(capture_directory / f"{camera_name}_rgb.png", rgb)

            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera_name)
            depth_m = renderer.render().copy()
            renderer.disable_depth_rendering()
            np.save(capture_directory / f"{camera_name}_depth_m.npy", depth_m, allow_pickle=False)

            renderer.enable_segmentation_rendering()
            renderer.update_scene(data, camera=camera_name)
            segmentation = renderer.render().copy()
            renderer.disable_segmentation_rendering()
            np.save(
                capture_directory / f"{camera_name}_segmentation_id_type.npy",
                segmentation,
                allow_pickle=False,
            )

            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            camera_body_id = int(model.cam_bodyid[camera_id])
            camera_body_name = (
                "world"
                if camera_body_id == 0
                else mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_BODY, camera_body_id
                )
            )
            if camera_body_name is None:
                raise ValueError(f"simulation camera parent body has no name: {camera_name}")
            finite_depth = depth_m[np.isfinite(depth_m)]
            rgb_path = capture_directory / f"{camera_name}_rgb.png"
            depth_path = capture_directory / f"{camera_name}_depth_m.npy"
            segmentation_path = capture_directory / f"{camera_name}_segmentation_id_type.npy"
            camera_manifest[camera_name] = {
                "camera_id": camera_id,
                "model_attachment": {
                    "parent_body": camera_body_name,
                    "position_in_parent_m": [
                        float(value) for value in model.cam_pos[camera_id]
                    ],
                    "quaternion_wxyz_in_parent": [
                        float(value) for value in model.cam_quat[camera_id]
                    ],
                },
                "fovy_degrees": float(model.cam_fovy[camera_id]),
                "simulation_world_pose": {
                    "position_m": [float(value) for value in data.cam_xpos[camera_id]],
                    "rotation_matrix_row_major": [
                        [float(value) for value in row]
                        for row in data.cam_xmat[camera_id].reshape(3, 3)
                    ],
                },
                "rgb": {
                    **_file_artifact(rgb_path, capture_directory, "CAPTURE_DIRECTORY"),
                    "shape_hwc": list(rgb.shape),
                    "dtype": str(rgb.dtype),
                    "channel_order": "RGB",
                },
                "depth": {
                    **_file_artifact(depth_path, capture_directory, "CAPTURE_DIRECTORY"),
                    "shape_hw": list(depth_m.shape),
                    "dtype": str(depth_m.dtype),
                    "unit": "m",
                    "depth_min": float(finite_depth.min()) if finite_depth.size else None,
                    "depth_max": float(finite_depth.max()) if finite_depth.size else None,
                },
                "segmentation": {
                    **_file_artifact(
                        segmentation_path, capture_directory, "CAPTURE_DIRECTORY"
                    ),
                    "shape": list(segmentation.shape),
                    "dtype": str(segmentation.dtype),
                    "channels": ["object_id", "object_type"],
                },
            }

        manifest = {
            "schema_version": 4 if candidate_artifact is not None else 3,
            "capture_scope": "MUJOCO_SYNTHETIC_CAMERA_DATA_ONLY",
            "created_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "simulator": "MuJoCo",
            "mujoco_version": mujoco.__version__,
            "scene": str(SCENE_PATH.relative_to(REPO_ROOT)),
            "mujoco_gl_backend": os.environ.get("MUJOCO_GL"),
            "source_files": [
                _file_artifact(path, REPO_ROOT, "REPOSITORY_ROOT")
                for path in _scene_source_files()
            ],
            "image_size_px": {"width": args.width, "height": args.height},
            "simulation_steps": args.steps,
            "simulation_time_seconds": float(data.time),
            "joint_state_simulation_radians": _joint_state(mujoco, model, data),
            "actuator_targets_simulation_radians": targets,
            **(
                {
                    "initial_state_source": {
                        "kind": "MUJOCO_IK_CANDIDATE_ENDPOINT",
                        "candidate_sha256": ik_candidate_sha256,
                        "candidate_artifact": candidate_artifact,
                    }
                }
                if candidate_artifact is not None
                else {}
            ),
            "cameras": camera_manifest,
            "segmentation_object_type": int(mujoco.mjtObj.mjOBJ_GEOM),
            "segmentation_geom_id_map": _geom_segmentation_map(mujoco, model),
            "physical_camera_pose_verified": False,
            "usable_for_robot_control": False,
            "arm_command_sent": False,
            "review_required": True,
        }
        with (capture_directory / "manifest.json").open("x", encoding="utf-8") as manifest_file:
            json.dump(manifest, manifest_file, ensure_ascii=False, indent=2, allow_nan=False)
            manifest_file.write("\n")
    except Exception as exc:
        print(f"MuJoCo camera capture failed: {exc}", file=sys.stderr)
        return 2
    finally:
        if renderer is not None:
            renderer.close()

    print(f"Synthetic camera capture: {capture_directory}")
    if ik_candidate is not None:
        print("hash-검증 IK 후보 종점의 고정·손목 카메라 simulation RGB/depth/segmentation 저장 완료; 실물 장치는 열지 않았습니다.")
    else:
        print("고정·기준 손목·대상 지향 후보의 시뮬레이션 RGB/depth/segmentation 저장 완료; 실물 장치는 열지 않았습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
