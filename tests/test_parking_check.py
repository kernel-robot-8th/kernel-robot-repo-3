from __future__ import annotations

import math
import unittest

import cv2
import numpy as np

from control_station.parking_check import (
    MarkerPose,
    ParkingStatus,
    ParkingTolerance,
    PlanarOffset,
    detect_markers,
    estimate_marker_pose,
    judge,
    marker_object_points,
    median_offset,
    planar_offset,
)

CAMERA = np.array([[980.0, 0.0, 320.0], [0.0, 980.0, 240.0], [0.0, 0.0, 1.0]])
NO_DISTORTION = np.zeros(5)
SIDE_MM = 70.0
TOLERANCE = ParkingTolerance(position_mm=20.0, yaw_deg=10.0)


def floor_pose(x_mm: float, y_mm: float, yaw_deg: float, height_mm: float = 1500.0) -> MarkerPose:
    """A marker lying flat under a camera that looks straight down (marker face toward the camera)."""
    yaw = math.radians(yaw_deg)
    # Camera looking down: marker +z points back to the camera (-z camera), marker +y maps to camera -y.
    base = np.diag([1.0, -1.0, -1.0])
    spin = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    return MarkerPose(base @ spin, np.array([x_mm, -y_mm, height_mm]))


def project_corners(pose: MarkerPose) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(pose.rotation)
    points, _ = cv2.projectPoints(marker_object_points(SIDE_MM), rvec, pose.translation_mm, CAMERA, NO_DISTORTION)
    return points.reshape(4, 2)


class PlanarOffsetTest(unittest.TestCase):
    def test_same_pose_is_zero(self) -> None:
        pose = floor_pose(30.0, -20.0, 15.0)
        offset = planar_offset(pose, pose)
        for value in (offset.dx_mm, offset.dy_mm, offset.dz_mm, offset.dyaw_deg):
            self.assertAlmostEqual(value, 0.0, places=6)

    def test_shift_is_expressed_in_reference_marker_axes(self) -> None:
        # Reference turned 90° CCW, so its right edge (+x) points along the floor +y.
        reference = floor_pose(0.0, 0.0, 90.0)
        moved = floor_pose(0.0, 50.0, 90.0)
        offset = planar_offset(reference, moved)
        self.assertAlmostEqual(offset.dx_mm, 50.0, places=6)
        self.assertAlmostEqual(offset.dy_mm, 0.0, places=6)

    def test_forward_shift_without_rotation(self) -> None:
        offset = planar_offset(floor_pose(0.0, 0.0, 0.0), floor_pose(10.0, 25.0, 0.0))
        self.assertAlmostEqual(offset.dx_mm, 10.0, places=6)
        self.assertAlmostEqual(offset.dy_mm, 25.0, places=6)

    def test_yaw_is_counterclockwise_from_camera(self) -> None:
        offset = planar_offset(floor_pose(0.0, 0.0, 10.0), floor_pose(0.0, 0.0, 22.0))
        self.assertAlmostEqual(offset.dyaw_deg, 12.0, places=6)


class PoseFromPixelsTest(unittest.TestCase):
    def test_round_trip_through_projection(self) -> None:
        reference = estimate_marker_pose(project_corners(floor_pose(0, 0, 0)), SIDE_MM, CAMERA, NO_DISTORTION)
        current = estimate_marker_pose(project_corners(floor_pose(15, -30, 8)), SIDE_MM, CAMERA, NO_DISTORTION)
        offset = planar_offset(reference, current)
        self.assertAlmostEqual(offset.dx_mm, 15.0, delta=0.5)
        self.assertAlmostEqual(offset.dy_mm, -30.0, delta=0.5)
        self.assertAlmostEqual(offset.dyaw_deg, 8.0, delta=0.2)

    def test_detects_rendered_marker(self) -> None:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        render = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
        canvas = np.full((400, 400), 255, dtype=np.uint8)
        canvas[100:300, 100:300] = render(dictionary, 20, 200)
        found = detect_markers(canvas, "DICT_4X4_50")
        self.assertEqual(list(found), [20])
        self.assertTrue(np.allclose(found[20][0].mean(axis=0), (199.5, 199.5), atol=1.0))


class JudgeTest(unittest.TestCase):
    def test_not_found(self) -> None:
        self.assertEqual(judge(None, TOLERANCE).status, ParkingStatus.NOT_FOUND)

    def test_arrived_inside_both_tolerances(self) -> None:
        result = judge(PlanarOffset(10.0, -12.0, 0.0, 9.0), TOLERANCE)
        self.assertEqual(result.status, ParkingStatus.ARRIVED)

    def test_misaligned_position(self) -> None:
        result = judge(PlanarOffset(15.0, 15.0, 0.0, 0.0), TOLERANCE)
        self.assertEqual(result.status, ParkingStatus.MISALIGNED)
        self.assertIn("position", result.reason)

    def test_misaligned_yaw(self) -> None:
        result = judge(PlanarOffset(0.0, 0.0, 0.0, -11.0), TOLERANCE)
        self.assertEqual(result.status, ParkingStatus.MISALIGNED)
        self.assertIn("yaw", result.reason)


class MedianOffsetTest(unittest.TestCase):
    def test_yaw_median_across_wraparound(self) -> None:
        offsets = [PlanarOffset(0, 0, 0, yaw) for yaw in (179.0, -179.0, 178.0)]
        self.assertAlmostEqual(abs(median_offset(offsets).dyaw_deg), 179.0, places=6)


if __name__ == "__main__":
    unittest.main()
