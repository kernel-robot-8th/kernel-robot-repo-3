#!/usr/bin/env python3
"""Estimate the front camera pose relative to the shelves and project the slot areas into the image.

Inputs: the operator's clicked shelf keypoints (scripts/shelf_pose_editor.html), the same source frame,
the camera intrinsic draft, and config/shelf-slots.json. solvePnP (SQPnP, then Levenberg-Marquardt
refinement) gives the camera pose in the `shelf_front` frame. Each used slot's volume (deck footprint
up to its clear height) is projected to an image polygon so a detection's bottom-center can be
matched to a slot.

The output is a DRAFT_ONLY review artifact plus an overlay PNG. Residuals are reported, not judged:
there is no automatic pass threshold, and the result does not authorize robot motion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from calibration_artifacts import (
    MAX_CALIBRATION_IMAGE_BYTES,
    ensure_distinct_output_path,
    install_file_exclusive,
    read_regular_file_bounded,
    unique_json_object,
    validate_intrinsic_draft,
)


MIN_POINTS = 6
MAX_JSON_BYTES = 1_000_000
DEFAULT_OPEN_SLOT_HEIGHT_M = 0.10
COLORS = [(66, 135, 245), (48, 209, 88), (255, 159, 10), (191, 90, 242), (255, 69, 58), (100, 210, 255)]


def _load_json(path: Path, label: str) -> tuple[dict[str, Any], str]:
    raw = read_regular_file_bounded(path, MAX_JSON_BYTES, label)
    payload = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_json_object)
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return payload, hashlib.sha256(raw).hexdigest()


def _point(value: object, label: str) -> tuple[float, float]:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value)
        or not all(math.isfinite(float(item)) for item in value)
    ):
        raise ValueError(f"{label} must be two finite numbers")
    return float(value[0]), float(value[1])


def solve_pose(
    object_points: np.ndarray, image_points: np.ndarray, camera_matrix: np.ndarray, distortion: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion, flags=cv2.SOLVEPNP_SQPNP
    )
    if not ok:
        raise ValueError("solvePnP did not find a pose")
    rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, camera_matrix, distortion, rvec, tvec)
    return rvec, tvec


def slot_volume_corners(slot: dict[str, Any], open_height: float, top: float | None = None) -> np.ndarray:
    """Corners of the slot footprint box from the deck up to `top` (default: the whole slot volume)."""
    footprint = slot["footprint_m"]
    bottom = slot["deck_height_m"]
    if top is None:
        top = bottom + (slot["clear_height_m"] if slot["clear_height_m"] is not None else open_height)
    return np.array(
        [
            [x, y, z]
            for x in (footprint["x_min"], footprint["x_max"])
            for y in (footprint["y_min"], footprint["y_max"])
            for z in (bottom, top)
        ],
        dtype=np.float64,
    )


def build_artifact(
    points_doc: dict[str, Any],
    points_sha: str,
    image: np.ndarray,
    image_sha: str,
    intrinsics: dict[str, Any],
    intrinsics_sha: str,
    slot_map: dict[str, Any],
    slot_map_sha: str,
    open_height: float,
    anchor_margin: float = 0.02,
) -> tuple[dict[str, Any], np.ndarray]:
    if points_doc.get("schema_version") != 1:
        raise ValueError("points JSON must be schema_version 1")
    if points_doc.get("source_image_sha256") != image_sha:
        raise ValueError("points JSON was made on a different image (source_image_sha256 mismatch)")
    if points_doc.get("slot_map_sha256") != slot_map_sha:
        raise ValueError("points JSON was made with a different slot map (slot_map_sha256 mismatch)")
    if points_doc.get("camera_id") != intrinsics["camera_id"]:
        raise ValueError("points JSON camera_id does not match the intrinsic calibration camera_id")
    height, width = image.shape[:2]
    size = intrinsics["image_size_px"]
    if points_doc.get("source_image_size_px") != {"width": width, "height": height} or size != {
        "width": width,
        "height": height,
    }:
        raise ValueError("image size must match both the points JSON and the intrinsic calibration")

    keypoints = {item["keypoint_id"]: item for item in slot_map.get("keypoints", [])}
    clicked = points_doc.get("points")
    if not isinstance(clicked, list) or len(clicked) < MIN_POINTS:
        raise ValueError(f"at least {MIN_POINTS} clicked keypoints are required")
    ids = [item.get("keypoint_id") for item in clicked]
    if len(set(ids)) != len(ids) or any(keypoint_id not in keypoints for keypoint_id in ids):
        raise ValueError("clicked keypoints must be unique keypoint_id values from the slot map")
    object_points = np.array(
        [[keypoints[i]["point_m"][axis] for axis in ("x", "y", "z")] for i in ids], dtype=np.float64
    )
    image_points = np.array(
        [_point(item.get("image_px"), f"{item.get('keypoint_id')}.image_px") for item in clicked],
        dtype=np.float64,
    )
    if np.linalg.matrix_rank(object_points - object_points.mean(axis=0), tol=1e-3) < 2:
        raise ValueError("clicked keypoints are collinear; pick corners on different edges")

    camera_matrix = np.array(intrinsics["camera_matrix"], dtype=np.float64)
    distortion = np.array(intrinsics["distortion_coefficients"], dtype=np.float64).reshape(-1)
    rvec, tvec = solve_pose(object_points, image_points, camera_matrix, distortion)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    residuals = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    rotation, _ = cv2.Rodrigues(rvec)
    camera_in_shelf = (-rotation.T @ tvec).reshape(3)

    # Leave-one-out: re-solve without each point and predict it, to show points that disagree.
    leave_one_out: list[float | None] = []
    for index in range(len(ids)):
        keep = np.arange(len(ids)) != index
        if keep.sum() < MIN_POINTS - 1:
            leave_one_out.append(None)
            continue
        try:
            r, t = solve_pose(object_points[keep], image_points[keep], camera_matrix, distortion)
        except (ValueError, cv2.error):
            leave_one_out.append(None)
            continue
        predicted, _ = cv2.projectPoints(object_points[index : index + 1], r, t, camera_matrix, distortion)
        leave_one_out.append(round(float(np.linalg.norm(predicted.reshape(2) - image_points[index])), 2))

    overlay = image.copy()
    slots_out = []
    active = [slot for slot in slot_map["slots"] if slot["active"]]
    def projected_hull(corners: np.ndarray) -> np.ndarray:
        camera_side = (rotation @ corners.T + tvec).T
        if np.any(camera_side[:, 2] <= 0):
            raise ValueError(f"{slot['slot_id']} projects behind the camera; check the clicked points")
        pixels, _ = cv2.projectPoints(corners, rvec, tvec, camera_matrix, distortion)
        return cv2.convexHull(pixels.reshape(-1, 2).astype(np.float32)).reshape(-1, 2)

    for index, slot in enumerate(active):
        if not isinstance(slot.get("lip_top_height_m"), (int, float)):
            raise ValueError(f"{slot['slot_id']} has no lip_top_height_m; regenerate the slot map")
        corners = slot_volume_corners(slot, open_height)
        hull = projected_hull(corners)
        # Where a detection's bottom-center can appear: the deck up to just above its front lip.
        # Objects standing behind the lip show their bottom at or slightly above the lip top line.
        anchor_top = slot["lip_top_height_m"] + anchor_margin
        anchor = projected_hull(slot_volume_corners(slot, open_height, top=anchor_top))
        deck_px = projected_hull(slot_volume_corners(slot, open_height, top=slot["deck_height_m"]))
        color = COLORS[index % len(COLORS)]
        layer = overlay.copy()
        cv2.fillPoly(layer, [np.round(anchor).astype(np.int32)], color)
        overlay = cv2.addWeighted(layer, 0.35, overlay, 0.65, 0)
        cv2.polylines(overlay, [np.round(hull).astype(np.int32)], True, color, 1, cv2.LINE_AA)
        cv2.polylines(overlay, [np.round(anchor).astype(np.int32)], True, color, 2, cv2.LINE_AA)
        center = anchor.mean(axis=0)
        cv2.putText(overlay, slot["slot_id"], (int(center[0]) - 34, int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, slot["slot_id"], (int(center[0]) - 34, int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
        slots_out.append(
            {
                "slot_id": slot["slot_id"],
                "label": slot["label"],
                "volume_height_m": round(float(corners[:, 2].max() - corners[:, 2].min()), 4),
                "volume_height_source": "clear_height" if slot["clear_height_m"] is not None else "open_slot_height",
                "polygon_px": [[round(float(x), 1), round(float(y), 1)] for x, y in hull],
                "anchor_top_height_m": round(float(anchor_top), 4),
                "anchor_polygon_px": [[round(float(x), 1), round(float(y), 1)] for x, y in anchor],
                "deck_polygon_px": [[round(float(x), 1), round(float(y), 1)] for x, y in deck_px],
            }
        )
    for (x, y), (px, py) in zip(image_points, projected.reshape(-1, 2)):
        cv2.drawMarker(overlay, (int(round(x)), int(round(y))), (80, 255, 80), cv2.MARKER_CROSS, 10, 1, cv2.LINE_AA)
        cv2.circle(overlay, (int(round(px)), int(round(py))), 3, (60, 60, 255), 1, cv2.LINE_AA)

    artifact = {
        "schema_version": 1,
        "pose_id": str(uuid.uuid4()),
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "DRAFT_ONLY",
        "review_required": True,
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "camera_id": intrinsics["camera_id"],
        "pixel_transform_id": intrinsics["pixel_transform_id"],
        "image_size_px": {"width": width, "height": height},
        "frame_id": slot_map["frame"]["frame_id"],
        "sources": {
            "points_json_sha256": points_sha,
            "source_image_sha256": image_sha,
            "intrinsic_calibration_id": intrinsics["calibration_id"],
            "intrinsic_file_sha256": intrinsics_sha,
            "slot_map_sha256": slot_map_sha,
        },
        "solver": {"method": "SOLVEPNP_SQPNP + solvePnPRefineLM", "opencv_version": cv2.__version__},
        "pose": {
            "rvec": [round(float(v), 6) for v in rvec.reshape(3)],
            "tvec_m": [round(float(v), 5) for v in tvec.reshape(3)],
            "camera_position_in_shelf_frame_m": {
                axis: round(float(value), 4) for axis, value in zip(("x", "y", "z"), camera_in_shelf)
            },
        },
        "points": [
            {
                "keypoint_id": keypoint_id,
                "image_px": [round(float(v), 2) for v in image_points[index]],
                "reprojection_error_px": round(float(residuals[index]), 2),
                "leave_one_out_error_px": leave_one_out[index],
            }
            for index, keypoint_id in enumerate(ids)
        ],
        "reprojection_rms_px": round(float(np.sqrt(np.mean(residuals**2))), 3),
        "open_slot_height_m": open_height,
        "anchor_margin_m": anchor_margin,
        "slots": slots_out,
    }
    return artifact, overlay


def _write_exclusive(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", delete=False) as handle:
            handle.write(data)
            temporary = Path(handle.name)
        install_file_exclusive(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("points", type=Path, help="shelf-pose-points.json from shelf_pose_editor.html")
    parser.add_argument("image", type=Path, help="the same source frame the points were clicked on")
    parser.add_argument(
        "--intrinsics", type=Path, default=Path(".control-station/camera-calibrations/shelf_camera.json")
    )
    parser.add_argument("--slots", type=Path, default=Path("config/shelf-slots.json"))
    parser.add_argument(
        "--open-slot-height",
        type=float,
        default=DEFAULT_OPEN_SLOT_HEIGHT_M,
        help="height (m) drawn for slots with an open top; display/matching assumption, not a measurement",
    )
    parser.add_argument(
        "--anchor-margin",
        type=float,
        default=0.02,
        help="height (m) above each front lip still counted as the slot for a detection's bottom-center (assumption)",
    )
    parser.add_argument("--output", type=Path, help="artifact JSON path (default: managed calibration directory)")
    args = parser.parse_args()
    try:
        if not math.isfinite(args.anchor_margin) or not 0 <= args.anchor_margin <= 0.1:
            raise ValueError("--anchor-margin must be in [0, 0.1] m")
        if not math.isfinite(args.open_slot_height) or not 0 < args.open_slot_height <= 0.5:
            raise ValueError("--open-slot-height must be in (0, 0.5] m")
        points_doc, points_sha = _load_json(args.points, "points JSON")
        intrinsics, intrinsics_sha = _load_json(args.intrinsics, "intrinsic calibration")
        validate_intrinsic_draft(intrinsics)
        slot_map, slot_map_sha = _load_json(args.slots, "slot map")
        image_bytes = read_regular_file_bounded(args.image, MAX_CALIBRATION_IMAGE_BYTES, "source image")
        image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("source image could not be decoded")
        artifact, overlay = build_artifact(
            points_doc, points_sha, image, hashlib.sha256(image_bytes).hexdigest(),
            intrinsics, intrinsics_sha, slot_map, slot_map_sha, args.open_slot_height, args.anchor_margin,
        )
        output = args.output or Path(".control-station/camera-calibrations") / (
            f"{artifact['camera_id']}-shelf-pose-{artifact['pose_id'][:8]}.json"
        )
        ensure_distinct_output_path(output, [args.points, args.image, args.intrinsics, args.slots])
        ok, png = cv2.imencode(".png", overlay)
        if not ok:
            raise ValueError("overlay PNG could not be encoded")
        overlay_path = output.with_suffix(".overlay.png")
        artifact["overlay_png"] = overlay_path.name
        _write_exclusive(output, (json.dumps(artifact, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
        _write_exclusive(overlay_path, png.tobytes())
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    position = artifact["pose"]["camera_position_in_shelf_frame_m"]
    print(f"재투영 RMS {artifact['reprojection_rms_px']} px, 기준점 {len(artifact['points'])}개")
    for point in artifact["points"]:
        print(f"  {point['keypoint_id']:32s} 오차 {point['reprojection_error_px']:5.2f} px  "
              f"빼고 예측 {point['leave_one_out_error_px'] if point['leave_one_out_error_px'] is not None else '-'} px")
    print(
        f"카메라 위치 (선반 기준): 앞으로 {-position['x']*100:.1f} cm, 왼쪽 {position['y']*100:.1f} cm, "
        f"높이 {position['z']*100:.1f} cm"
    )
    print(f"저장: {output}\n오버레이: {overlay_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
