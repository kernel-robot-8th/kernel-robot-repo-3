#!/usr/bin/env python3
"""Generate the fixtures that test/test_python_parity.cpp compares the C++ port against.

Every expected value comes from running the Python modules (parking_check, parking_observation, parking_tracker)
on the same frames, so the fixtures pin the Python behaviour the C++ code must reproduce. Re-run after changing
them, with the OpenCV release the C++ library links (the system one on ROS 2 Jazzy):

    /usr/bin/python3 cpp/parking_vision/tools/make_parity_fixtures.py

--out writes elsewhere, e.g. fixtures from another OpenCV release; point the C++ test at them with
PARKING_VISION_FIXTURES=<dir>.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import tempfile
import time
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from control_station.parking_check import (  # noqa: E402
    PIXEL_TRANSFORM_ID,
    SCHEMA_VERSION,
    FrameObservation,
    MarkerPose,
    ParkingJudge,
    ParkingResult,
    detect_markers,
    estimate_marker_pose,
    load_spot,
    marker_object_points,
    observe_frame,
    project_to_plane,
    sha256_of,
)
from control_station.parking_observation import SPOT_DIR, ParkingObservationError, observe_parking  # noqa: E402
from control_station.parking_tracker import ParkingTracker  # noqa: E402

FIXTURE_DIR = Path(__file__).resolve().parents[1] / "test" / "fixtures"
SIDE_MM = 70.0
SCENE_CAMERA = np.array([[980.0, 0.0, 320.0], [0.0, 980.0, 240.0], [0.0, 0.0, 1.0]])
# Synthetic lens close to the ceiling camera's (strong barrel distortion, off-centre principal point).
DISTORTED_CAMERA = np.array([[986.0, 0.0, 182.0], [0.0, 985.0, 222.5], [0.0, 0.0, 1.0]])
DISTORTION = np.array([-0.38, 0.06, 0.006, 0.0025, 0.07])
SCENE = {21: (420, 140), 1: (80, 60), 3: (120, 360)}


def rotation_z(degrees: float) -> np.ndarray:
    a = math.radians(degrees)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def rotation_x(degrees: float) -> np.ndarray:
    a = math.radians(degrees)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def floor_pose(x_mm: float, y_mm: float, yaw_deg: float, tilt_deg: float) -> MarkerPose:
    floor = rotation_x(tilt_deg) @ np.diag([1.0, -1.0, -1.0])
    origin = rotation_x(tilt_deg) @ np.array([-260.0, -20.0, 1500.0])
    return MarkerPose(floor @ rotation_z(yaw_deg), origin + floor @ np.array([x_mm, y_mm, 0.0]))


def distorted_corners(pose: MarkerPose) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(pose.rotation)
    points, _ = cv2.projectPoints(marker_object_points(SIDE_MM), rvec, pose.translation_mm, DISTORTED_CAMERA, DISTORTION)
    return points.reshape(4, 2)


def render_scene(markers: dict[int, tuple[int, int]], size: int = 60) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    render = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    for marker_id, (x, y) in markers.items():
        canvas[y:y + size, x:x + size] = render(dictionary, marker_id, size)
    return canvas


def rotated_marker_scene(degrees: float) -> np.ndarray:
    """SCENE with Pinky's marker turned in the image, resampled like a real camera frame."""
    anchors = render_scene({i: xy for i, xy in SCENE.items() if i != 21})
    pinky = render_scene({21: SCENE[21]})
    x, y = SCENE[21]
    turn = cv2.getRotationMatrix2D((x + 29.5, y + 29.5), degrees, 1.0)
    pinky = cv2.warpAffine(pinky, turn, (640, 480), flags=cv2.INTER_LINEAR, borderValue=255)
    return np.minimum(anchors, pinky)


def camera_like(image: np.ndarray, seed: int) -> np.ndarray:
    """Blur and sensor noise, so subpixel corner refinement works on non-binary edges."""
    noise = np.random.default_rng(seed).normal(0.0, 4.0, image.shape)
    blurred = cv2.GaussianBlur(image, (5, 5), 1.1).astype(np.float64)
    return np.clip(blurred + noise, 0, 255).round().astype(np.uint8)


def points(array: np.ndarray) -> list[list[float]]:
    return np.asarray(array, dtype=np.float64).reshape(-1, 2).tolist()


def pose_dict(pose: MarkerPose) -> dict[str, object]:
    return {"rotation": pose.rotation.tolist(), "translation_mm": pose.translation_mm.tolist()}


def plane_dict(marker) -> dict[str, float]:  # type: ignore[no-untyped-def]
    return {"x_mm": marker.x_mm, "y_mm": marker.y_mm, "yaw_deg": marker.yaw_deg, "side_mm": marker.side_mm}


def offset_dict(offset) -> dict[str, float] | None:  # type: ignore[no-untyped-def]
    return None if offset is None else {"dx_mm": offset.dx_mm, "dy_mm": offset.dy_mm, "dyaw_deg": offset.dyaw_deg}


def observation_dict(observation: FrameObservation) -> dict[str, object]:
    return {
        "offset": offset_dict(observation.offset),
        "corners": None if observation.corners is None else points(observation.corners),
        "reason": observation.reason,
        "blocking_status": None if observation.blocking_status is None else observation.blocking_status.value,
        "skipped_status": observation.skipped_status.value,
        "view_shift_px": observation.view_shift_px,
        "view_shift_xy_px": None if observation.view_shift_xy_px is None else list(observation.view_shift_xy_px),
    }


def result_dict(result: ParkingResult) -> dict[str, object]:
    return {
        "status": result.status.value,
        "offset": offset_dict(result.offset),
        "reason": result.reason,
        "view_shift_px": result.view_shift_px,
        "deviation_level": None if result.deviation_level is None else result.deviation_level.value,
    }


def geometry_cases() -> tuple[list[dict], list[dict]]:
    rng = np.random.default_rng(20261102)
    reference = floor_pose(0.0, 0.0, 0.0, tilt_deg=26.0)
    plane = estimate_marker_pose(distorted_corners(reference), SIDE_MM, DISTORTED_CAMERA, DISTORTION)
    geometry, poses = [], []
    for index in range(8):
        x, y = rng.uniform(-120.0, 120.0, 2)
        yaw = rng.uniform(-180.0, 180.0)
        corners = distorted_corners(floor_pose(float(x), float(y), float(yaw), tilt_deg=26.0))
        on_plane = project_to_plane(corners, plane, DISTORTED_CAMERA, DISTORTION)
        geometry.append({"name": f"distorted_floor_{index}", "plane": pose_dict(plane), "corners_px": points(corners),
                         "expected": plane_dict(on_plane)})
        poses.append({"name": f"distorted_floor_{index}", "corners_px": points(corners),
                      "expected": pose_dict(estimate_marker_pose(corners, SIDE_MM, DISTORTED_CAMERA, DISTORTION))})
    return geometry, poses


def write_spot(reference: np.ndarray, spot_dir: Path) -> Path:
    """A schema 2 spot file registered from one frame, in the same layout scripts/pinky_parking.py writes."""
    spot_dir.mkdir(parents=True)
    intrinsics = spot_dir / "ceiling_camera.json"
    intrinsics.write_text(json.dumps({
        "camera_id": "ceiling_camera",
        "pixel_transform_id": PIXEL_TRANSFORM_ID,
        "calibration_id": "parity-fixture",
        "camera_matrix": SCENE_CAMERA.tolist(),
        "distortion_coefficients": [0.0] * 5,
        "image_size_px": {"width": 640, "height": 480},
    }, indent=2) + "\n", encoding="utf-8")
    found = detect_markers(reference, "DICT_4X4_50")
    corners = found[21][0]
    plane = estimate_marker_pose(corners, SIDE_MM, SCENE_CAMERA, np.zeros(5))
    path = spot_dir / "spot.json"
    path.write_text(json.dumps({
        "schema_version": SCHEMA_VERSION,
        "spot_id": "parity_dock",
        "camera_id": "ceiling_camera",
        "pixel_transform_id": PIXEL_TRANSFORM_ID,
        "intrinsic_calibration": {"path": intrinsics.name, "sha256": sha256_of(intrinsics),
                                  "calibration_id": "parity-fixture"},
        "aruco_dictionary": "DICT_4X4_50",
        "marker_id": 21,
        "marker_side_mm": SIDE_MM,
        "marker_forward_axis": "-x",
        "opencv_version": cv2.__version__,
        "reference_pose_in_camera": pose_dict(plane),
        "reference_corners_px": points(corners),
        "anchors": [{"marker_id": i, "center_px": [round(float(v), 2) for v in c[0].mean(axis=0)]}
                    for i, c in found.items() if i != 21],
        "view_check": {"mode": "FIXED_MARKERS", "max_anchor_shift_px": 15.0},
        "tolerance": {"position_mm": 20.0, "yaw_deg": 5.0},
    }, indent=2) + "\n", encoding="utf-8")
    return path


def bgr(image: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image


def control_station_root(spot_path: Path, directory: Path) -> Path:
    """A repository root holding the fixture spot where parking_observation and parking_tracker look for it."""
    shutil.copy(spot_path.parent / "ceiling_camera.json", directory / "ceiling_camera.json")
    (directory / SPOT_DIR).mkdir(parents=True)
    shutil.copy(spot_path, directory / SPOT_DIR / "parity_dock.json")
    return directory


def payload_fields(payload: dict) -> dict[str, object]:
    """The judged fields both Python payloads share; offsets are rounded there (0.1 mm, 0.01°)."""
    offset = payload.get("offset")
    overlay = payload.get("image_overlay") or {}
    return {
        "reason": payload["reason"],
        "deviation_level": payload.get("deviation_level"),
        "offset": None if offset is None else {k: offset[k] for k in ("dx_mm", "dy_mm", "dyaw_deg")},
        "view_shift_px": payload.get("view_shift_px"),
        # Drawn only from judged frames and cleared when the view changes (rounded to 0.1 px).
        "marker_corners_px": overlay.get("marker_corners_px"),
        "view_shift_xy_px": overlay.get("view_shift_xy_px"),
    }


# Frames captured before the request are prefixed with "before:".
OBSERVATION_CASES = {
    "arrived": ["before:moved"] * 3 + ["registered", "camera_like_registered"] * 10,
    "too_few_frames": ["registered"] * 3,
    "view_change_clears_earlier_frames": ["registered"] * 6 + ["wrapped_96"] + ["moved"] * 6,
    "skipped_frames_keep_earlier_ones": ["registered"] * 4 + ["missing_pinky"] * 3 + ["no_fixed_marker"] * 2
    + ["camera_like_moved"] * 4,
    "duplicate_marker_last": ["registered"] * 6 + ["duplicate"],
    "only_stale_frames": ["before:registered"] * 5,
    "resolution_mismatch": ["registered", "small"],
}


def observation_cases(spot_path: Path, frames: dict[str, np.ndarray]) -> list[dict]:
    cases = []
    with tempfile.TemporaryDirectory() as directory:
        root = control_station_root(spot_path, Path(directory))
        for name, script in OBSERVATION_CASES.items():
            served = []

            def frames_after(camera_id: str, after: int, script=script, served=served):  # type: ignore[no-untyped-def]
                if served:
                    return []
                future = time.monotonic() + 1000.0
                for sequence, entry in enumerate(script, start=1):
                    stale = entry.startswith("before:")
                    image = bgr(frames[entry.removeprefix("before:")])
                    height, width = image.shape[:2]
                    served.append((sequence, "sha", image.tobytes(), width, height, "", 0.0 if stale else future))
                return served

            try:
                payload = observe_parking("parity_dock", "head_camera", frames_after, root, collect_seconds=0.05)
                expected = {"status": payload["status"], **payload_fields(payload),
                            "frames_used": payload["frames_used"], "frames_examined": payload["frames_examined"],
                            "frame_sequence_range": payload["frame_sequence_range"]}
            except ParkingObservationError as error:
                expected = {"error_code": error.error_code}
            cases.append({"name": name, "frames": script, "expected": expected})
    return cases


# (feed result each tick, frame name, ticks): "frame" serves a new frame, "empty" none yet, "missing" no feed.
TRACKER_SCRIPT = [
    ("empty", None, 10), ("empty", None, 15), ("frame", "missing_pinky", 5),
    *[("frame", f"drive_{step}", 1) for step in range(5)],
    ("frame", "registered", 12), ("frame", "drift_2_1", 3), ("frame", "moved", 12), ("frame", "wrapped_96", 3),
    ("frame", "registered", 12), ("frame", "duplicate", 2), ("frame", "camera_like_registered", 12),
    ("frame", "missing_pinky", 12), ("frame", "no_fixed_marker", 3), ("missing", None, 2), ("frame", "registered", 3),
    ("frame", "lifted", 12),
]


def tracker_ticks(spot_path: Path, frames: dict[str, np.ndarray]) -> list[dict]:
    """Drives the Python tracker on a fake clock, one tick per 0.1 s, like tests/test_parking_tracker.py."""
    ticks = []
    with tempfile.TemporaryDirectory() as directory:
        root = control_station_root(spot_path, Path(directory))
        clock = {"now": 0.0, "sequence": 0, "feed": "empty", "image": None}

        def frames_after(camera_id: str, after: int):  # type: ignore[no-untyped-def]
            if clock["feed"] == "missing":
                return None
            if clock["feed"] == "empty":
                return []
            clock["sequence"] += 1
            image = clock["image"]
            height, width = image.shape[:2]
            return [(clock["sequence"], "sha", image.tobytes(), width, height, "", clock["now"])]

        tracker = ParkingTracker("parity_dock", "head_camera", frames_after, root, clock=lambda: clock["now"])
        for feed, name, count in TRACKER_SCRIPT:
            clock["feed"], clock["image"] = feed, None if name is None else bgr(frames[name])
            for _ in range(count):
                clock["now"] += 0.1
                tracker.tick()
                snapshot = tracker.snapshot()
                ticks.append({"now": clock["now"], "feed": feed, "image": None if name is None else f"frames/{name}.png",
                              "sequence": clock["sequence"], "expected": {"state": snapshot["state"],
                                                                          "frame_sequence": snapshot["frame_sequence"],
                                                                          **payload_fields(snapshot)}})
    return ticks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=FIXTURE_DIR)
    fixture_dir = parser.parse_args().out.resolve()
    if fixture_dir.exists():
        if any(fixture_dir.iterdir()) and not (fixture_dir / "parity.json").is_file():
            parser.error(f"{fixture_dir} is not empty and holds no parity.json; refusing to replace it")
        shutil.rmtree(fixture_dir)
    frame_dir = fixture_dir / "frames"
    frame_dir.mkdir(parents=True)

    scene = render_scene(SCENE)
    spot_path = write_spot(scene, fixture_dir / "spot")
    spot = load_spot(spot_path, spot_path.parent)
    moved = {**SCENE, 21: (440, 150)}
    frames = {
        "registered": scene,
        "moved": render_scene(moved),
        "rotated_7deg": rotated_marker_scene(7.0),
        "camera_like_registered": camera_like(scene, seed=1),
        "camera_like_moved": camera_like(render_scene(moved), seed=2),
        "camera_like_rotated": camera_like(rotated_marker_scene(-12.0), seed=3),
        "drift_2_1": np.roll(scene, (2, 1), axis=(0, 1)),
        "drift_10_-4": np.roll(scene, (10, -4), axis=(0, 1)),
        "wrapped_96": np.roll(scene, 96, axis=1),
        "no_fixed_marker": render_scene({21: SCENE[21]}),
        "missing_pinky": render_scene({i: xy for i, xy in SCENE.items() if i != 21}),
        "duplicate": render_scene(SCENE) & render_scene({21: (300, 300)}),
        "lifted": render_scene({i: xy for i, xy in SCENE.items() if i != 21}) & render_scene({21: SCENE[21]}, size=80),
        "small": cv2.resize(scene, (320, 240), interpolation=cv2.INTER_AREA),
        # Pinky driving in from 20 px below the spot, 4 px per frame (tests/test_parking_tracker.py).
        **{f"drive_{step}": render_scene({**SCENE, 21: (SCENE[21][0], SCENE[21][1] + 20 - 4 * step)}) for step in range(5)},
    }
    for name, image in frames.items():
        cv2.imwrite(str(frame_dir / f"{name}.png"), image)

    spot_variants = {"registered": spot, "view_check_off": replace(spot, anchors={}),
                     "max_view_shift_3": replace(spot, max_view_shift_px=3.0)}
    frame_cases = [
        {"name": f"{variant}/{frame}", "image": f"frames/{frame}.png", "spot": variant,
         "expected": observation_dict(observe_frame(frames[frame], spot_variants[variant]))}
        for variant, names in {
            "registered": [name for name in frames if name != "small" and not name.startswith("drive_")],
            "view_check_off": ["registered", "wrapped_96", "no_fixed_marker"],
            "max_view_shift_3": ["drift_2_1", "drift_10_-4"],
        }.items()
        for frame in names
    ]

    judgment = ParkingJudge(spot.tolerance, frames=3)
    sequence = ["registered", "camera_like_registered", "missing_pinky", "moved", "camera_like_moved", "moved",
                "no_fixed_marker", "wrapped_96", "missing_pinky", "missing_pinky", "missing_pinky", "registered"]
    judge_steps = [{"image": f"frames/{name}.png", "expected": result_dict(judgment.add(observe_frame(frames[name], spot)))}
                   for name in sequence]

    geometry, poses = geometry_cases()
    (fixture_dir / "parity.json").write_text(json.dumps({
        "generated_by": "cpp/parking_vision/tools/make_parity_fixtures.py",
        "python_opencv_version": cv2.__version__,
        "distorted_camera": {"camera_matrix": DISTORTED_CAMERA.tolist(), "distortion": DISTORTION.tolist()},
        "marker_side_mm": SIDE_MM,
        "geometry_cases": geometry,
        "pose_cases": poses,
        "spot": {"path": "spot/spot.json", "reference": plane_dict(spot.reference)},
        "frame_cases": frame_cases,
        "judge_sequence": {"frames": 3, "steps": judge_steps},
        "observation_cases": observation_cases(spot_path, frames),
        "tracker_ticks": tracker_ticks(spot_path, frames),
    }, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {len(frame_cases)} frame cases, {len(judge_steps)} judge steps, {len(geometry)} geometry cases, "
          f"{len(OBSERVATION_CASES)} observation cases, {sum(n for _, _, n in TRACKER_SCRIPT)} tracker ticks "
          f"with OpenCV {cv2.__version__} to {fixture_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
