#!/usr/bin/env python3
"""Back-project one segmented MuJoCo geom's visible surface pixels to 3D."""

from __future__ import annotations

import argparse
from io import BytesIO
import json
import math
import sys
from pathlib import Path
from typing import Any


SCRIPTS_DIRECTORY = Path(__file__).resolve().parent
CAMERA_NAMES = (
    "placeholder_fixed_camera",
    "placeholder_wrist_camera",
    "placeholder_wrist_aimed_candidate",
)
sys.path.insert(0, str(SCRIPTS_DIRECTORY))
from verify_mujoco_camera_capture import (  # noqa: E402
    CaptureVerificationError,
    load_verified_capture,
    read_verified_artifact_bytes,
)


def localize_geom(
    capture_directory: Path,
    camera_name: str,
    geom_name: str,
) -> dict[str, Any]:
    """Return a review-only mean of visible depth pixels in two sim frames."""
    capture_directory = capture_directory.resolve(strict=True)
    manifest, _, manifest_sha256 = load_verified_capture(capture_directory)
    camera = manifest["cameras"][camera_name]
    image_size = manifest["image_size_px"]
    width, height = image_size["width"], image_size["height"]
    geom_object_type = manifest.get("segmentation_object_type")
    geom_map = manifest.get("segmentation_geom_id_map")
    if (
        isinstance(geom_object_type, bool)
        or not isinstance(geom_object_type, int)
        or not isinstance(geom_map, list)
        or any(not isinstance(item, dict) for item in geom_map)
    ):
        raise ValueError("manifest segmentation geom map is invalid")
    geom_matches = [
        item
        for item in geom_map
        if item.get("name") == geom_name
        and item.get("object_type") == geom_object_type
    ]
    if len(geom_matches) != 1:
        raise ValueError(f"geom name must match exactly one segmented geom: {geom_name}")
    geom_id = geom_matches[0].get("object_id")
    if isinstance(geom_id, bool) or not isinstance(geom_id, int) or geom_id < 0:
        raise ValueError("manifest geom ID is invalid")

    import numpy as np

    depth_bytes = read_verified_artifact_bytes(capture_directory, camera["depth"])
    segmentation_bytes = read_verified_artifact_bytes(
        capture_directory, camera["segmentation"]
    )
    depth = np.load(BytesIO(depth_bytes), allow_pickle=False)
    segmentation = np.load(BytesIO(segmentation_bytes), allow_pickle=False)
    if depth.shape != (height, width) or depth.dtype != np.dtype("float32"):
        raise CaptureVerificationError("depth array shape or dtype changed after verification")
    if (
        segmentation.shape != (height, width, 2)
        or segmentation.dtype != np.dtype("int32")
    ):
        raise CaptureVerificationError(
            "segmentation array shape or dtype changed after verification"
        )
    selected = (segmentation[..., 0] == geom_id) & (
        segmentation[..., 1] == geom_object_type
    )
    valid = selected & np.isfinite(depth) & (depth > 0.0)
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        raise ValueError(
            f"geom has no finite positive-depth pixels in camera {camera_name}"
        )

    fovy_radians = math.radians(float(camera["fovy_degrees"]))
    focal_y = height / (2.0 * math.tan(fovy_radians / 2.0))
    focal_x = focal_y
    center_x = (width - 1.0) / 2.0
    center_y = (height - 1.0) / 2.0
    z_depth = np.asarray(depth[rows, columns], dtype=np.float64)
    points_camera = np.column_stack(
        (
            (columns - center_x) * z_depth / focal_x,
            -(rows - center_y) * z_depth / focal_y,
            -z_depth,
        )
    )

    pose = camera["simulation_world_pose"]
    camera_position = np.asarray(pose["position_m"], dtype=np.float64)
    camera_rotation = np.asarray(
        pose["rotation_matrix_row_major"], dtype=np.float64
    )
    points_world = points_camera @ camera_rotation.T + camera_position
    visible_surface_mean_camera = points_camera.mean(axis=0)
    visible_surface_mean_world = points_world.mean(axis=0)
    pixel_center = [float(columns.mean()), float(rows.mean())]

    return {
        "schema_version": 1,
        "estimate_scope": "VISIBLE_SURFACE_PIXEL_MEAN_ONLY",
        "capture_manifest_sha256": manifest_sha256,
        "camera": camera_name,
        "geom_name": geom_name,
        "geom_id": geom_id,
        "visible_pixel_count": int(rows.size),
        "visible_pixel_mean_xy": pixel_center,
        "visible_surface_pixel_mean": {
            "camera_frame": camera_name,
            "visible_surface_position_m": [
                float(value) for value in visible_surface_mean_camera
            ],
            "mujoco_world_position_m": [
                float(value) for value in visible_surface_mean_world
            ],
        },
        "projection": {
            "model": "centered_pinhole_from_vertical_fovy",
            "image_size_px": {"width": width, "height": height},
            "fovy_degrees": float(camera["fovy_degrees"]),
            "fx_px": focal_x,
            "fy_px": focal_y,
            "depth_semantics": "distance_from_camera_plane_m",
            "camera_forward_axis": "negative_z",
        },
        "not_estimated": [
            "object_center",
            "object_orientation",
            "object_dimensions",
            "robot_base_pose",
            "grasp_pose",
            "graspability",
        ],
        "simulation_only": True,
        "real_camera_pose_verified": False,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "review_required": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_directory", type=Path)
    parser.add_argument("--camera", required=True, choices=CAMERA_NAMES)
    parser.add_argument(
        "--geom", required=True, help="exact geom name from the capture manifest"
    )
    args = parser.parse_args()
    try:
        result = localize_geom(args.capture_directory, args.camera, args.geom)
    except ImportError:
        print(
            "NumPy is optional; install requirements-mujoco.txt in the simulator environment.",
            file=sys.stderr,
        )
        return 2
    except (
        CaptureVerificationError,
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
    ) as exc:
        print(f"MuJoCo geom localization failed: {exc}", file=sys.stderr)
        return 2
    json.dump(result, sys.stdout, ensure_ascii=False, indent=2, allow_nan=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
