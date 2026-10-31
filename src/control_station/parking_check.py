"""Ceiling-camera ArUco check of whether Pinky stopped on its registered parking spot."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np

# A pose that reprojects the four corners worse than this is treated as a failed estimate.
MAX_REPROJECTION_ERROR_PX = 3.0


class ParkingStatus(str, Enum):
    ARRIVED = "ARRIVED"
    MISALIGNED = "MISALIGNED"
    NOT_FOUND = "NOT_FOUND"
    AMBIGUOUS = "AMBIGUOUS"


@dataclass(frozen=True)
class MarkerPose:
    """Marker pose in the camera frame. Marker axes: +x to its right edge, +y to its top edge, +z out of its face."""

    rotation: np.ndarray  # 3x3
    translation_mm: np.ndarray  # (3,)


@dataclass(frozen=True)
class PlanarOffset:
    """Current marker relative to the reference marker, expressed in the reference marker axes."""

    dx_mm: float  # + toward the reference marker's right edge
    dy_mm: float  # + toward the reference marker's top edge
    dz_mm: float  # + out of the reference marker face (toward the camera)
    dyaw_deg: float  # + counterclockwise as seen from the camera

    @property
    def distance_mm(self) -> float:
        return math.hypot(self.dx_mm, self.dy_mm)


@dataclass(frozen=True)
class ParkingTolerance:
    position_mm: float
    yaw_deg: float


@dataclass(frozen=True)
class ParkingResult:
    status: ParkingStatus
    offset: PlanarOffset | None
    reason: str

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {"status": self.status.value, "reason": self.reason}
        if self.offset is not None:
            result.update(
                dx_mm=round(self.offset.dx_mm, 1),
                dy_mm=round(self.offset.dy_mm, 1),
                dz_mm=round(self.offset.dz_mm, 1),
                dyaw_deg=round(self.offset.dyaw_deg, 2),
                distance_mm=round(self.offset.distance_mm, 1),
            )
        return result


def detect_markers(image: np.ndarray, dictionary_name: str) -> dict[int, list[np.ndarray]]:
    """Return {marker id: [4x2 corner arrays]} in ArUco order (top-left, top-right, bottom-right, bottom-left)."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    # Integer corners on a ~40 px marker let the pose flip between two tilts (yaw jumped ~12° on the
    # ceiling camera); subpixel refinement kept it within 0.6°.
    if hasattr(cv2.aruco, "ArucoDetector"):
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers(gray)
    else:  # OpenCV < 4.7
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    found: dict[int, list[np.ndarray]] = {}
    if ids is not None:
        for marker_id, marker_corners in zip(ids.ravel(), corners):
            found.setdefault(int(marker_id), []).append(marker_corners.reshape(4, 2).astype(np.float64))
    return found


def marker_object_points(side_mm: float) -> np.ndarray:
    half = side_mm / 2.0
    return np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]])


def estimate_marker_pose(
    corners_px: np.ndarray, side_mm: float, camera_matrix: np.ndarray, distortion: np.ndarray
) -> MarkerPose:
    # IPPE returns two candidate poses for a square. Near a straight-down view the wrong one can win,
    # so keep only poses whose marker face points toward the camera and take the lowest reprojection error.
    # IPPE degenerates for an exactly fronto-parallel square at the principal point; ITERATIVE covers that case.
    object_points = marker_object_points(side_mm)
    image_points = np.asarray(corners_px, dtype=np.float64).reshape(4, 1, 2)
    candidates = []
    for flags in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
        count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
            object_points, image_points, camera_matrix, distortion, flags=flags
        )
        for index in range(count):
            rotation, _ = cv2.Rodrigues(rvecs[index])
            translation = tvecs[index].reshape(3)
            error = float(np.ravel(errors)[index])
            if translation[2] > 0 and float(rotation[:, 2] @ -translation) > 0 and error < MAX_REPROJECTION_ERROR_PX:
                candidates.append((error, rotation, translation))
        if candidates:
            break
    if not candidates:
        raise ValueError("marker pose estimation failed")
    _, rotation, translation = min(candidates, key=lambda item: item[0])
    return MarkerPose(rotation, translation)


def planar_offset(reference: MarkerPose, current: MarkerPose) -> PlanarOffset:
    relative_rotation = reference.rotation.T @ current.rotation
    relative_translation = reference.rotation.T @ (current.translation_mm - reference.translation_mm)
    dyaw = math.degrees(math.atan2(relative_rotation[1, 0], relative_rotation[0, 0]))
    return PlanarOffset(*(float(v) for v in relative_translation), dyaw)


def median_offset(offsets: list[PlanarOffset]) -> PlanarOffset:
    """Median over frames; yaw is unwrapped around the first sample so ±180° does not average to 0."""
    base = offsets[0].dyaw_deg
    yaws = [base + (o.dyaw_deg - base + 180.0) % 360.0 - 180.0 for o in offsets]
    return PlanarOffset(
        float(np.median([o.dx_mm for o in offsets])),
        float(np.median([o.dy_mm for o in offsets])),
        float(np.median([o.dz_mm for o in offsets])),
        float((np.median(yaws) + 180.0) % 360.0 - 180.0),
    )


def judge(offset: PlanarOffset | None, tolerance: ParkingTolerance, reason: str = "") -> ParkingResult:
    if offset is None:
        return ParkingResult(ParkingStatus.NOT_FOUND, None, reason or "marker not visible")
    outside = []
    if offset.distance_mm > tolerance.position_mm:
        outside.append(f"position {offset.distance_mm:.0f} mm > {tolerance.position_mm:g} mm")
    if abs(offset.dyaw_deg) > tolerance.yaw_deg:
        outside.append(f"yaw {offset.dyaw_deg:+.1f} deg > ±{tolerance.yaw_deg:g} deg")
    if outside:
        return ParkingResult(ParkingStatus.MISALIGNED, offset, "; ".join(outside))
    return ParkingResult(ParkingStatus.ARRIVED, offset, "within tolerance")
