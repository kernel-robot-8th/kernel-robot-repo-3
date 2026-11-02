from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from control_station.parking_check import (
    FrameObservation,
    Intrinsics,
    MarkerPose,
    ParkingJudge,
    ParkingSpot,
    ParkingSpotError,
    ParkingStatus,
    ParkingTolerance,
    PlanarOffset,
    anchor_shift_px,
    detect_markers,
    estimate_marker_pose,
    judge,
    load_spot,
    marker_object_points,
    median_offset,
    observe_frame,
    plane_offset,
    project_to_plane,
)

CAMERA = np.array([[980.0, 0.0, 320.0], [0.0, 980.0, 240.0], [0.0, 0.0, 1.0]])
NO_DISTORTION = np.zeros(5)
SIDE_MM = 70.0
TOLERANCE = ParkingTolerance(position_mm=20.0, yaw_deg=10.0)


def rotation_z(degrees: float) -> np.ndarray:
    a = math.radians(degrees)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def rotation_x(degrees: float) -> np.ndarray:
    a = math.radians(degrees)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def floor_pose(x_mm: float, y_mm: float, yaw_deg: float, tilt_deg: float = 0.0) -> MarkerPose:
    """A marker lying on the floor 1.5 m below the camera, moved by (x, y) and turned by yaw in the floor plane.

    With tilt 0 the camera looks straight down; a tilt turns the whole floor about the camera x axis,
    like the ceiling camera that sees the floor 26° off its optical axis.
    """
    # Camera looking down: marker +z points back to the camera (-z camera), marker +y maps to camera -y.
    floor = rotation_x(tilt_deg) @ np.diag([1.0, -1.0, -1.0])
    origin = rotation_x(tilt_deg) @ np.array([40.0, -30.0, 1500.0])
    return MarkerPose(floor @ rotation_z(yaw_deg), origin + floor @ np.array([x_mm, y_mm, 0.0]))


def project_corners(pose: MarkerPose) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(pose.rotation)
    points, _ = cv2.projectPoints(marker_object_points(SIDE_MM), rvec, pose.translation_mm, CAMERA, NO_DISTORTION)
    return points.reshape(4, 2)


def measure(plane: MarkerPose, reference: MarkerPose, current: MarkerPose) -> PlanarOffset:
    def on_plane(pose: MarkerPose):  # type: ignore[no-untyped-def]
        return project_to_plane(project_corners(pose), plane, CAMERA, NO_DISTORTION)

    return plane_offset(on_plane(reference), on_plane(current))


class PlaneProjectionTest(unittest.TestCase):
    def test_same_corners_are_zero(self) -> None:
        pose = floor_pose(0.0, 0.0, 15.0)
        offset = measure(pose, pose, pose)
        for value in (offset.dx_mm, offset.dy_mm, offset.dyaw_deg):
            self.assertAlmostEqual(value, 0.0, places=6)

    def test_shift_and_yaw_in_reference_axes(self) -> None:
        reference = floor_pose(0.0, 0.0, 0.0)
        offset = measure(reference, reference, floor_pose(15.0, -30.0, 8.0))
        self.assertAlmostEqual(offset.dx_mm, 15.0, places=4)
        self.assertAlmostEqual(offset.dy_mm, -30.0, places=4)
        self.assertAlmostEqual(offset.dyaw_deg, 8.0, places=4)

    def test_shift_is_expressed_in_reference_marker_axes(self) -> None:
        # Reference turned 90° CCW, so its right edge (+x) points along the floor +y.
        reference = floor_pose(0.0, 0.0, 90.0)
        offset = measure(reference, reference, floor_pose(0.0, 50.0, 90.0))
        self.assertAlmostEqual(offset.dx_mm, 50.0, places=4)
        self.assertAlmostEqual(offset.dy_mm, 0.0, places=4)

    def test_tilted_camera(self) -> None:
        reference = floor_pose(0.0, 0.0, 0.0, tilt_deg=26.0)
        offset = measure(reference, reference, floor_pose(-12.0, 90.0, -3.0, tilt_deg=26.0))
        self.assertAlmostEqual(offset.dx_mm, -12.0, places=4)
        self.assertAlmostEqual(offset.dy_mm, 90.0, places=4)
        self.assertAlmostEqual(offset.dyaw_deg, -3.0, places=4)

    def test_measured_side_on_plane(self) -> None:
        pose = floor_pose(20.0, 10.0, 30.0, tilt_deg=26.0)
        side = project_to_plane(project_corners(pose), pose, CAMERA, NO_DISTORTION).side_mm
        self.assertAlmostEqual(side, SIDE_MM, places=4)

    def test_plane_depth_error_scales_offset_only_slightly(self) -> None:
        # A registered plane 3% too far (the PnP depth error seen on the ceiling camera) scales offsets by 3%,
        # instead of adding a fixed tens-of-mm bias.
        reference = floor_pose(0.0, 0.0, 0.0, tilt_deg=26.0)
        biased = MarkerPose(reference.rotation, reference.translation_mm * 1.03)
        offset = measure(biased, reference, floor_pose(0.0, 90.0, 0.0, tilt_deg=26.0))
        self.assertAlmostEqual(offset.dx_mm, 0.0, delta=0.5)
        self.assertAlmostEqual(offset.dy_mm, 90.0 * 1.03, delta=0.5)

    def test_registered_pose_from_pixels(self) -> None:
        reference = floor_pose(0.0, 0.0, 0.0, tilt_deg=26.0)
        plane = estimate_marker_pose(project_corners(reference), SIDE_MM, CAMERA, NO_DISTORTION)
        offset = measure(plane, reference, floor_pose(15.0, -30.0, 8.0, tilt_deg=26.0))
        self.assertAlmostEqual(offset.dx_mm, 15.0, delta=0.5)
        self.assertAlmostEqual(offset.dy_mm, -30.0, delta=0.5)
        self.assertAlmostEqual(offset.dyaw_deg, 8.0, delta=0.2)


def render_scene(markers: dict[int, tuple[int, int]], size: int = 60) -> np.ndarray:
    """White 640x480 image with flat markers whose top-left corners sit at the given pixels."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    render = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    for marker_id, (x, y) in markers.items():
        canvas[y:y + size, x:x + size] = render(dictionary, marker_id, size)
    return canvas


def spot_from_image(image: np.ndarray, marker_id: int = 21, side_mm: float = 70.0) -> ParkingSpot:
    found = detect_markers(image, "DICT_4X4_50")
    corners = found[marker_id][0]
    plane = estimate_marker_pose(corners, side_mm, CAMERA, NO_DISTORTION)
    return ParkingSpot(
        spot_id="test",
        camera_id="ceiling_camera",
        dictionary="DICT_4X4_50",
        marker_id=marker_id,
        marker_side_mm=side_mm,
        plane=plane,
        reference=project_to_plane(corners, plane, CAMERA, NO_DISTORTION),
        reference_corners_px=corners,
        anchors={i: c[0].mean(axis=0) for i, c in found.items() if i != marker_id},
        tolerance=TOLERANCE,
        intrinsics=Intrinsics(CAMERA, NO_DISTORTION, 640, 480, None, Path("unused.json"), ""),
    )


SCENE = {21: (420, 140), 1: (80, 60), 3: (120, 360)}


class ObserveFrameTest(unittest.TestCase):
    def setUp(self) -> None:
        self.spot = spot_from_image(render_scene(SCENE))

    def test_registered_frame_is_zero_offset(self) -> None:
        observation = observe_frame(render_scene(SCENE), self.spot)
        self.assertIsNone(observation.blocking_status)
        assert observation.offset is not None
        self.assertAlmostEqual(observation.offset.distance_mm, 0.0, delta=0.5)
        self.assertAlmostEqual(observation.view_shift_px or 0.0, 0.0, delta=0.5)

    def test_moved_marker_is_measured(self) -> None:
        observation = observe_frame(render_scene({**SCENE, 21: (440, 140)}), self.spot)
        assert observation.offset is not None
        self.assertGreater(observation.offset.distance_mm, 10.0)

    def test_small_view_drift_is_subtracted(self) -> None:
        observation = observe_frame(np.roll(render_scene(SCENE), (2, 1), axis=(0, 1)), self.spot)
        self.assertIsNone(observation.blocking_status)
        assert observation.offset is not None
        self.assertAlmostEqual(observation.offset.distance_mm, 0.0, delta=0.5)
        self.assertAlmostEqual(observation.view_shift_px or 0.0, math.hypot(2, 1), delta=0.3)

    def test_wrapped_frame_is_not_judged(self) -> None:
        # The ceiling camera sometimes opens with the whole picture wrapped 96 px sideways.
        observation = observe_frame(np.roll(render_scene(SCENE), 96, axis=1), self.spot)
        self.assertEqual(observation.blocking_status, ParkingStatus.VIEW_UNVERIFIED)
        self.assertIsNone(observation.offset)
        self.assertAlmostEqual(observation.view_shift_px or 0.0, 96.0, delta=1.0)

    def test_no_fixed_marker_is_not_judged(self) -> None:
        observation = observe_frame(render_scene({21: SCENE[21]}), self.spot)
        self.assertEqual(observation.blocking_status, ParkingStatus.VIEW_UNVERIFIED)

    def test_one_fixed_marker_is_enough(self) -> None:
        observation = observe_frame(render_scene({21: SCENE[21], 1: SCENE[1]}), self.spot)
        self.assertIsNone(observation.blocking_status)
        self.assertIsNotNone(observation.offset)

    def test_duplicate_marker_is_ambiguous(self) -> None:
        observation = observe_frame(render_scene(SCENE) & render_scene({21: (300, 300)}), self.spot)
        self.assertEqual(observation.blocking_status, ParkingStatus.AMBIGUOUS)

    def test_marker_off_the_plane_is_rejected(self) -> None:
        # A bigger marker in the image reads as a bigger side on the registered plane (e.g. Pinky lifted).
        anchors_only = render_scene({i: xy for i, xy in SCENE.items() if i != 21})
        observation = observe_frame(anchors_only & render_scene({21: SCENE[21]}, size=80), self.spot)
        self.assertIsNone(observation.offset)
        self.assertIn("marker side", observation.reason)

    def test_detects_rendered_marker(self) -> None:
        found = detect_markers(render_scene({20: (170, 140)}, size=200), "DICT_4X4_50")
        self.assertEqual(list(found), [20])
        self.assertTrue(np.allclose(found[20][0].mean(axis=0), (269.5, 239.5), atol=1.0))


class AnchorShiftTest(unittest.TestCase):
    def test_none_when_no_anchor_visible(self) -> None:
        self.assertIsNone(anchor_shift_px({1: np.array([10.0, 10.0])}, {}))

    def test_duplicated_anchor_is_ignored(self) -> None:
        square = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
        self.assertIsNone(anchor_shift_px({1: np.array([1.0, 1.0])}, {1: [square, square + 50]}))

    def test_median_of_visible_anchors(self) -> None:
        square = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
        anchors = {1: np.array([1.0, 1.0]), 2: np.array([1.0, 1.0]), 3: np.array([1.0, 1.0])}
        found = {1: [square], 2: [square + [1.0, -2.0]], 3: [square + [40.0, -3.0]]}
        np.testing.assert_allclose(anchor_shift_px(anchors, found), [1.0, -2.0])


class JudgeTest(unittest.TestCase):
    def test_not_found(self) -> None:
        self.assertEqual(judge(None, TOLERANCE).status, ParkingStatus.NOT_FOUND)

    def test_arrived_inside_both_tolerances(self) -> None:
        result = judge(PlanarOffset(10.0, -12.0, 9.0), TOLERANCE)
        self.assertEqual(result.status, ParkingStatus.ARRIVED)

    def test_misaligned_position(self) -> None:
        result = judge(PlanarOffset(15.0, 15.0, 0.0), TOLERANCE)
        self.assertEqual(result.status, ParkingStatus.MISALIGNED)
        self.assertIn("position", result.reason)

    def test_misaligned_yaw(self) -> None:
        result = judge(PlanarOffset(0.0, 0.0, -11.0), TOLERANCE)
        self.assertEqual(result.status, ParkingStatus.MISALIGNED)
        self.assertIn("yaw", result.reason)


class ParkingJudgeTest(unittest.TestCase):
    def test_brief_miss_keeps_recent_frames(self) -> None:
        judgment = ParkingJudge(TOLERANCE, frames=3)
        judgment.add(FrameObservation(PlanarOffset(1.0, 0.0, 0.0), None, "", None, 0.0))
        result = judgment.add(FrameObservation(None, None, "marker not visible", None, 0.0))
        self.assertEqual(result.status, ParkingStatus.ARRIVED)

    def test_misses_for_all_frames_drop_old_values(self) -> None:
        judgment = ParkingJudge(TOLERANCE, frames=2)
        judgment.add(FrameObservation(PlanarOffset(1.0, 0.0, 0.0), None, "", None, 0.0))
        judgment.add(FrameObservation(None, None, "marker not visible", None, 0.0))
        result = judgment.add(FrameObservation(None, None, "marker not visible", None, 0.0))
        self.assertEqual(result.status, ParkingStatus.NOT_FOUND)

    def test_view_change_clears_earlier_frames(self) -> None:
        judgment = ParkingJudge(TOLERANCE, frames=3)
        judgment.add(FrameObservation(PlanarOffset(1.0, 0.0, 0.0), None, "", None, 0.0))
        result = judgment.add(FrameObservation(None, None, "moved", ParkingStatus.VIEW_UNVERIFIED, 96.0))
        self.assertEqual(result.status, ParkingStatus.VIEW_UNVERIFIED)
        self.assertEqual(judgment.recent, [])
        self.assertEqual(result.to_dict()["view_shift_px"], 96.0)

    def test_keeps_only_latest_frames(self) -> None:
        judgment = ParkingJudge(TOLERANCE, frames=2)
        for dx in (100.0, 1.0, 2.0):
            result = judgment.add(FrameObservation(PlanarOffset(dx, 0.0, 0.0), None, "", None, 0.0))
        self.assertEqual(result.status, ParkingStatus.ARRIVED)


class MedianOffsetTest(unittest.TestCase):
    def test_yaw_median_across_wraparound(self) -> None:
        offsets = [PlanarOffset(0, 0, yaw) for yaw in (179.0, -179.0, 178.0)]
        self.assertAlmostEqual(abs(median_offset(offsets).dyaw_deg), 179.0, places=6)


class LoadSpotTest(unittest.TestCase):
    def test_schema_1_needs_registering_again(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.json"
            path.write_text(json.dumps({"schema_version": 1}), encoding="utf-8")
            with self.assertRaisesRegex(ParkingSpotError, "다시 등록"):
                load_spot(path, Path(directory))


if __name__ == "__main__":
    unittest.main()
