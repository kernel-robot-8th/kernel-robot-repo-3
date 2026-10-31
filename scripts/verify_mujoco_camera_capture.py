#!/usr/bin/env python3
"""Verify hashes and array metadata in a saved synthetic MuJoCo camera capture."""

from __future__ import annotations

import argparse
import hashlib
from io import BytesIO
import json
import math
import re
import struct
import sys
from pathlib import Path, PurePosixPath
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = REPO_ROOT / "simulation" / "mujoco_omx" / "robotis_omx" / "shelf_pick_scene.xml"
CAMERA_NAMES_V1 = (
    "placeholder_fixed_camera",
    "placeholder_wrist_camera",
)
CAMERA_NAMES_V2 = (
    *CAMERA_NAMES_V1,
)
CAMERA_NAMES_V3 = (
    *CAMERA_NAMES_V2,
    "placeholder_wrist_aimed_candidate",
)
CAMERA_NAMES_V4 = CAMERA_NAMES_V3
CAMERA_PARENT_BODIES = {
    "placeholder_fixed_camera": "world",
    "placeholder_wrist_camera": "link6",
    "placeholder_wrist_aimed_candidate": "link6",
}
MAX_CAPTURE_FILE_BYTES = 64 * 1024 * 1024
MAX_MANIFEST_BYTES = 1_000_000
MAX_IK_CANDIDATE_BYTES = 1_000_000
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
ARM_JOINT_NAMES = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5")


class CaptureVerificationError(ValueError):
    """Raised when a capture does not satisfy its saved manifest contract."""


class _DuplicateJSONKeyError(ValueError):
    """Raised when a manifest object repeats a member name."""


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise _DuplicateJSONKeyError(f"manifest contains a duplicate JSON key: {key}")
        value[key] = item
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _artifact_path(
    path_value: object,
    path_base: object,
    capture_directory: Path,
) -> Path:
    if not isinstance(path_value, str) or not path_value:
        raise CaptureVerificationError("artifact path is missing")
    pure_path = PurePosixPath(path_value)
    if pure_path.is_absolute() or ".." in pure_path.parts or "\\" in path_value:
        raise CaptureVerificationError("artifact path must be a safe relative path")
    if path_base == "CAPTURE_DIRECTORY":
        if len(pure_path.parts) != 1:
            raise CaptureVerificationError("capture outputs must be direct children of the capture directory")
        base_directory = capture_directory
    elif path_base == "REPOSITORY_ROOT":
        base_directory = REPO_ROOT
    else:
        raise CaptureVerificationError("artifact path_base is unsupported")
    unresolved = base_directory.joinpath(*pure_path.parts)
    component = base_directory
    for part in pure_path.parts:
        component = component / part
        if component.is_symlink():
            raise CaptureVerificationError("artifact paths cannot contain symbolic links")
    resolved = unresolved.resolve()
    if resolved == unresolved and (
        resolved.parent == base_directory or base_directory in resolved.parents
    ):
        return resolved
    raise CaptureVerificationError("artifact resolves outside its declared base directory")


def _verify_file_descriptor(
    descriptor: object,
    expected_path_base: str,
    capture_directory: Path,
) -> Path:
    if not isinstance(descriptor, dict):
        raise CaptureVerificationError("artifact descriptor must be an object")
    if (
        descriptor.get("path_base") != expected_path_base
        or descriptor.get("hash_algorithm") != "sha256"
        or not isinstance(descriptor.get("sha256"), str)
        or not SHA256_PATTERN.fullmatch(descriptor["sha256"])
        or isinstance(descriptor.get("size_bytes"), bool)
        or not isinstance(descriptor.get("size_bytes"), int)
        or descriptor["size_bytes"] < 0
    ):
        raise CaptureVerificationError("artifact hash metadata is invalid")
    path = _artifact_path(descriptor.get("path"), expected_path_base, capture_directory)
    if not path.is_file():
        raise CaptureVerificationError(f"artifact is missing: {descriptor.get('path')}")
    actual_size = path.stat().st_size
    if actual_size != descriptor["size_bytes"]:
        raise CaptureVerificationError(f"artifact size differs from manifest: {descriptor.get('path')}")
    if actual_size > MAX_CAPTURE_FILE_BYTES:
        raise CaptureVerificationError(f"artifact exceeds 64 MB verification limit: {descriptor.get('path')}")
    if _sha256(path) != descriptor["sha256"]:
        raise CaptureVerificationError(f"artifact SHA-256 differs from manifest: {descriptor.get('path')}")
    return path


def read_verified_artifact_bytes(
    capture_directory: Path, descriptor: object
) -> bytes:
    """Read and hash the exact capture artifact bytes a downstream consumer will use."""
    path = _verify_file_descriptor(
        descriptor, "CAPTURE_DIRECTORY", capture_directory
    )
    if not isinstance(descriptor, dict):
        raise CaptureVerificationError("artifact descriptor must be an object")
    try:
        with path.open("rb") as artifact_file:
            payload = artifact_file.read(MAX_CAPTURE_FILE_BYTES + 1)
    except OSError as exc:
        raise CaptureVerificationError("artifact cannot be read") from exc
    if (
        len(payload) != descriptor.get("size_bytes")
        or hashlib.sha256(payload).hexdigest() != descriptor.get("sha256")
    ):
        raise CaptureVerificationError("artifact changed after capture verification")
    return payload


def _verify_initial_state_source(
    manifest: dict[str, Any],
    capture_directory: Path,
    current_source_files: list[object],
) -> bool:
    if manifest.get("schema_version") < 4:
        if "initial_state_source" in manifest:
            raise CaptureVerificationError("initial_state_source requires capture schema v4")
        return False
    source = manifest.get("initial_state_source")
    if (
        not isinstance(source, dict)
        or set(source) != {"kind", "candidate_sha256", "candidate_artifact"}
        or source.get("kind") != "MUJOCO_IK_CANDIDATE_ENDPOINT"
        or not isinstance(source.get("candidate_sha256"), str)
        or not SHA256_PATTERN.fullmatch(source["candidate_sha256"])
        or not isinstance(source.get("candidate_artifact"), dict)
        or source["candidate_artifact"].get("path") != "ik_candidate.json"
        or source["candidate_artifact"].get("sha256") != source.get("candidate_sha256")
    ):
        raise CaptureVerificationError("IK candidate initial_state_source metadata is invalid")
    artifact = source["candidate_artifact"]
    artifact_path = _verify_file_descriptor(artifact, "CAPTURE_DIRECTORY", capture_directory)
    if artifact["size_bytes"] > MAX_IK_CANDIDATE_BYTES:
        raise CaptureVerificationError("IK candidate artifact exceeds the 1 MB verification limit")
    candidate_bytes = read_verified_artifact_bytes(capture_directory, artifact)
    try:
        candidate = json.loads(
            candidate_bytes.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
    except (
        UnicodeError,
        json.JSONDecodeError,
        _DuplicateJSONKeyError,
        RecursionError,
    ) as exc:
        raise CaptureVerificationError("captured IK candidate JSON is invalid") from exc
    if not isinstance(candidate, dict) or (
        isinstance(candidate.get("schema_version"), bool)
        or not isinstance(candidate.get("schema_version"), int)
        or candidate.get("schema_version") != 2
        or candidate.get("kind") != "mujoco_ik_candidate"
        or candidate.get("model") != SCENE_PATH.relative_to(REPO_ROOT).as_posix()
        or candidate.get("mujoco_version") != manifest.get("mujoco_version")
        or candidate.get("pose_frame") != "link6"
        or candidate.get("body") != "link6"
        or candidate.get("pose_converged") is not True
        or candidate.get("simulation_only") is not True
        or candidate.get("usable_for_robot_control") is not False
        or candidate.get("arm_command_sent") is not False
        or candidate.get("model_source_files") != current_source_files
    ):
        raise CaptureVerificationError("captured IK candidate is stale or outside simulation-only scope")
    contacts = candidate.get("contacts")
    joints = candidate.get("joint_candidate_rad")
    captured_joints = manifest.get("joint_state_simulation_radians")
    simulation_steps = manifest.get("simulation_steps")
    if (
        not isinstance(contacts, dict)
        or contacts.get("final_pose") != []
        or not isinstance(joints, dict)
        or set(joints) != set(ARM_JOINT_NAMES)
        or not isinstance(captured_joints, dict)
        or isinstance(simulation_steps, bool)
        or not isinstance(simulation_steps, int)
        or simulation_steps != 0
        or manifest.get("actuator_targets_simulation_radians") != {}
    ):
        raise CaptureVerificationError("captured IK candidate state does not match the static endpoint contract")
    for name in ARM_JOINT_NAMES:
        source_value = _finite_number(joints.get(name))
        captured_value = _finite_number(captured_joints.get(name))
        if (
            source_value is None
            or captured_value is None
            or not math.isclose(source_value, captured_value, rel_tol=0.0, abs_tol=1e-12)
        ):
            raise CaptureVerificationError(f"IK candidate joint state differs from capture: {name}")
    if artifact_path.name != "ik_candidate.json":
        raise CaptureVerificationError("captured IK candidate artifact path is invalid")
    return True


def _verify_png_bytes(payload: bytes, label: str, width: int, height: int) -> None:
    header = payload[:29]
    if (
        len(header) != 29
        or header[:8] != b"\x89PNG\r\n\x1a\n"
        or header[12:16] != b"IHDR"
    ):
        raise CaptureVerificationError(f"RGB artifact is not a PNG with an IHDR: {label}")
    actual_width, actual_height, bit_depth, color_type = struct.unpack(">IIBB", header[16:26])
    if (actual_width, actual_height, bit_depth, color_type) != (width, height, 8, 2):
        raise CaptureVerificationError(f"RGB PNG dimensions or pixel format differ: {label}")


def _finite_vector(value: object, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(_finite_number(item) is not None for item in value)
    )


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except (OverflowError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _valid_rotation_matrix(value: object) -> bool:
    if (
        not isinstance(value, list)
        or len(value) != 3
        or not all(_finite_vector(row, 3) for row in value)
    ):
        return False
    rows = [[float(component) for component in row] for row in value]
    for first in range(3):
        for second in range(3):
            dot = sum(rows[first][axis] * rows[second][axis] for axis in range(3))
            expected = 1.0 if first == second else 0.0
            if not math.isclose(dot, expected, rel_tol=0.0, abs_tol=1e-5):
                return False
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    return math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-5)


def load_verified_capture(
    capture_path: Path,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    """Return the verified manifest snapshot, summary, and exact manifest hash."""
    try:
        capture_directory = capture_path.resolve(strict=True)
    except OSError as exc:
        raise CaptureVerificationError("capture directory does not exist") from exc
    if not capture_directory.is_dir():
        raise CaptureVerificationError("capture path must be a directory")
    manifest_path = capture_directory / "manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise CaptureVerificationError("manifest.json must be a regular non-symlink file")
    try:
        if manifest_path.stat().st_size > MAX_MANIFEST_BYTES:
            raise CaptureVerificationError("manifest.json exceeds the 1 MB verification limit")
        with manifest_path.open("rb") as manifest_file:
            manifest_bytes = manifest_file.read(MAX_MANIFEST_BYTES + 1)
        if len(manifest_bytes) > MAX_MANIFEST_BYTES:
            raise CaptureVerificationError("manifest.json exceeds the 1 MB verification limit")
        manifest = json.loads(
            manifest_bytes.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
    except (
        OSError,
        UnicodeError,
        json.JSONDecodeError,
        _DuplicateJSONKeyError,
        RecursionError,
    ) as exc:
        raise CaptureVerificationError("manifest.json is missing or invalid") from exc
    if not isinstance(manifest, dict):
        raise CaptureVerificationError("manifest must be a JSON object")
    if (
        isinstance(manifest.get("schema_version"), bool)
        or not isinstance(manifest.get("schema_version"), int)
        or manifest.get("schema_version") not in {1, 2, 3, 4}
        or manifest.get("capture_scope") != "MUJOCO_SYNTHETIC_CAMERA_DATA_ONLY"
        or manifest.get("simulator") != "MuJoCo"
        or manifest.get("physical_camera_pose_verified") is not False
        or manifest.get("usable_for_robot_control") is not False
        or manifest.get("arm_command_sent") is not False
        or manifest.get("review_required") is not True
    ):
        raise CaptureVerificationError("manifest scope or review-only flags are invalid")

    image_size = manifest.get("image_size_px")
    if (
        not isinstance(image_size, dict)
        or set(image_size) != {"width", "height"}
        or any(
            isinstance(image_size.get(key), bool)
            or not isinstance(image_size.get(key), int)
            or image_size[key] < 32
            for key in ("width", "height")
        )
    ):
        raise CaptureVerificationError("manifest image_size_px is invalid")
    width, height = image_size["width"], image_size["height"]

    camera_records = manifest.get("cameras")
    camera_names_by_schema = {
        1: CAMERA_NAMES_V1,
        2: CAMERA_NAMES_V2,
        3: CAMERA_NAMES_V3,
        4: CAMERA_NAMES_V4,
    }
    camera_names = camera_names_by_schema[manifest["schema_version"]]
    if not isinstance(camera_records, dict) or set(camera_records) != set(camera_names):
        raise CaptureVerificationError("manifest camera set differs from the configured simulation views")
    verified_artifacts = 0
    import numpy as np

    for camera_name in camera_names:
        camera = camera_records[camera_name]
        if not isinstance(camera, dict):
            raise CaptureVerificationError(f"camera record is invalid: {camera_name}")
        pose = camera.get("simulation_world_pose")
        if (
            not isinstance(pose, dict)
            or not _finite_vector(pose.get("position_m"), 3)
            or not _valid_rotation_matrix(pose.get("rotation_matrix_row_major"))
        ):
            raise CaptureVerificationError(f"simulation camera pose is invalid: {camera_name}")
        fovy = camera.get("fovy_degrees")
        fovy_number = _finite_number(fovy)
        if (
            fovy_number is None
            or not 0 < fovy_number < 180
        ):
            raise CaptureVerificationError(f"camera FOV is invalid: {camera_name}")

        attachment = camera.get("model_attachment")
        if manifest["schema_version"] >= 2 and not isinstance(attachment, dict):
            raise CaptureVerificationError(
                f"model camera attachment provenance is missing: {camera_name}"
            )
        if attachment is not None:
            quaternion = (
                attachment.get("quaternion_wxyz_in_parent")
                if isinstance(attachment, dict)
                else None
            )
            if (
                not isinstance(attachment, dict)
                or attachment.get("parent_body") != CAMERA_PARENT_BODIES[camera_name]
                or not _finite_vector(attachment.get("position_in_parent_m"), 3)
                or not _finite_vector(quaternion, 4)
                or not math.isclose(
                    math.hypot(*(float(value) for value in quaternion)),
                    1.0,
                    rel_tol=0.0,
                    abs_tol=1e-5,
                )
            ):
                raise CaptureVerificationError(
                    f"model camera attachment provenance is invalid: {camera_name}"
                )

        rgb = camera.get("rgb")
        if (
            not isinstance(rgb, dict)
            or rgb.get("shape_hwc") != [height, width, 3]
            or rgb.get("dtype") != "uint8"
            or rgb.get("channel_order") != "RGB"
        ):
            raise CaptureVerificationError(f"RGB metadata is invalid: {camera_name}")
        rgb_bytes = read_verified_artifact_bytes(capture_directory, rgb)
        _verify_png_bytes(rgb_bytes, camera_name, width, height)
        verified_artifacts += 1

        depth = camera.get("depth")
        if (
            not isinstance(depth, dict)
            or depth.get("shape_hw") != [height, width]
            or depth.get("dtype") != "float32"
            or depth.get("unit") != "m"
        ):
            raise CaptureVerificationError(f"depth metadata is invalid: {camera_name}")
        depth_bytes = read_verified_artifact_bytes(capture_directory, depth)
        depth_array = np.load(BytesIO(depth_bytes), allow_pickle=False)
        if depth_array.shape != (height, width) or depth_array.dtype != np.dtype("float32"):
            raise CaptureVerificationError(f"depth array shape or dtype differs: {camera_name}")
        finite_depth = depth_array[np.isfinite(depth_array)]
        depth_min = depth.get("depth_min")
        depth_max = depth.get("depth_max")
        if finite_depth.size:
            depth_min_number = _finite_number(depth_min)
            depth_max_number = _finite_number(depth_max)
            if (
                depth_min_number is None
                or depth_max_number is None
                or not math.isclose(float(finite_depth.min()), depth_min_number, rel_tol=1e-6)
                or not math.isclose(float(finite_depth.max()), depth_max_number, rel_tol=1e-6)
            ):
                raise CaptureVerificationError(f"depth range differs from manifest: {camera_name}")
        elif depth_min is not None or depth_max is not None:
            raise CaptureVerificationError(f"depth range differs from manifest: {camera_name}")
        verified_artifacts += 1

        segmentation = camera.get("segmentation")
        segmentation_bytes = read_verified_artifact_bytes(
            capture_directory, segmentation
        )
        segmentation_array = np.load(BytesIO(segmentation_bytes), allow_pickle=False)
        if (
            not isinstance(segmentation, dict)
            or segmentation.get("shape") != list(segmentation_array.shape)
            or segmentation.get("dtype") != str(segmentation_array.dtype)
            or segmentation.get("channels") != ["object_id", "object_type"]
            or segmentation_array.shape != (height, width, 2)
            or segmentation_array.dtype != np.dtype("int32")
        ):
            raise CaptureVerificationError(f"segmentation array shape or dtype differs: {camera_name}")
        verified_artifacts += 1

    source_files = manifest.get("source_files")
    if not isinstance(source_files, list) or not source_files:
        raise CaptureVerificationError("manifest source_files list is missing")
    expected_source_paths = {
        path.relative_to(REPO_ROOT).as_posix()
        for path in (
            [SCENE_PATH, SCENE_PATH.parent / "omx.xml"]
            + list((SCENE_PATH.parent / "assets").rglob("*"))
        )
        if path.is_file()
    }
    recorded_source_paths = {
        source.get("path")
        for source in source_files
        if isinstance(source, dict)
    }
    if recorded_source_paths != expected_source_paths or len(source_files) != len(expected_source_paths):
        raise CaptureVerificationError("manifest source_files do not match the current MuJoCo scene inputs")
    verified_sources = 0
    for source in source_files:
        _verify_file_descriptor(source, "REPOSITORY_ROOT", capture_directory)
        verified_sources += 1
    state_source_verified = _verify_initial_state_source(
        manifest, capture_directory, source_files
    )
    if state_source_verified:
        verified_artifacts += 1
    summary = {
        "capture_directory": str(capture_directory),
        "camera_count": len(camera_names),
        "verified_capture_artifacts": verified_artifacts,
        "verified_scene_source_files": verified_sources,
        "ik_candidate_state_source_verified": state_source_verified,
        "simulation_only": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
    }
    return manifest, summary, hashlib.sha256(manifest_bytes).hexdigest()


def verify_capture(capture_path: Path) -> dict[str, Any]:
    """Verify a saved capture and return its compact summary."""
    _, summary, _ = load_verified_capture(capture_path)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_directory", type=Path, help="directory produced by capture_mujoco_cameras.py")
    args = parser.parse_args()
    try:
        summary = verify_capture(args.capture_directory)
    except (ImportError, OSError, EOFError, ValueError, TypeError, OverflowError) as exc:
        print(f"Capture verification failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
