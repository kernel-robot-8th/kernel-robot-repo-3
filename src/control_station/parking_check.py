"""Ceiling-camera ArUco check of whether Pinky stopped on its registered parking spot."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path

import cv2
import numpy as np

SCHEMA_VERSION = 2
PIXEL_TRANSFORM_ID = "opencv-v4l2-capture-no-app-geometry-v1"
# A pose that reprojects the four corners worse than this is treated as a failed estimate.
MAX_REPROJECTION_ERROR_PX = 3.0
# Default for a spot's view_check.max_anchor_shift_px. Fixed markers moving more than this means the frame
# is not the registered view: the ceiling camera sometimes opens with the whole image wrapped sideways by
# ~96 px for the entire session. Smaller shifts are the camera mount drifting and are subtracted; that drift
# reached 2-6 px within an hour on 2026-11-02, so the first 3 px limit blocked correctly parked Pinkys.
MAX_VIEW_SHIFT_PX = 15.0
# The drift is subtracted only when the fixed markers agree on it. Visible markers whose shifts differ from
# their median by more than this mean one of them was moved, not the camera (a tilted camera spread ~1.5 px).
ANCHOR_AGREE_PX = 3.0
# With a single visible fixed marker a moved marker and a drifted camera look the same, so only a shift up to
# this much is subtracted; beyond it the view is unverified. A spot whose operator confirmed the marker is
# fastened down (view_check.single_anchor_trusted) uses the full max_anchor_shift_px instead.
SINGLE_ANCHOR_MAX_SHIFT_PX = 3.0
# The marker side measured on the registered plane must stay this close to the printed side;
# otherwise the marker is not on that plane (lifted, tilted or a different print).
MAX_SIDE_ERROR_RATIO = 0.1
# Up to this multiple of the tolerance a misalignment counts as NEAR, beyond it as FAR.
NEAR_LIMIT_RATIO = 2.0
# Marker axis that points to Pinky's front -> (forward, left) unit vectors in marker (x, y), with marker +z up.
MARKER_FORWARD_AXES = {"+x": (1.0, 0.0), "-x": (-1.0, 0.0), "+y": (0.0, 1.0), "-y": (0.0, -1.0)}


class ParkingStatus(str, Enum):
    ARRIVED = "ARRIVED"
    MISALIGNED = "MISALIGNED"
    NOT_FOUND = "NOT_FOUND"
    AMBIGUOUS = "AMBIGUOUS"
    VIEW_UNVERIFIED = "VIEW_UNVERIFIED"


class DeviationLevel(str, Enum):
    """How far the measured spot is from the registered one, relative to the tolerance."""

    WITHIN = "WITHIN"  # both ratios <= 1
    NEAR = "NEAR"  # worst ratio <= NEAR_LIMIT_RATIO
    FAR = "FAR"


class ParkingSpotError(ValueError):
    pass


@dataclass(frozen=True)
class MarkerPose:
    """Marker pose in the camera frame. Marker axes: +x to its right edge, +y to its top edge, +z out of its face."""

    rotation: np.ndarray  # 3x3
    translation_mm: np.ndarray  # (3,)


@dataclass(frozen=True)
class PlaneMarker:
    """Marker corners back-projected onto the registered marker plane, in that plane's marker axes."""

    x_mm: float
    y_mm: float
    yaw_deg: float  # direction of the marker +x edge, counterclockwise as seen from the camera
    side_mm: float


@dataclass(frozen=True)
class PlanarOffset:
    """Current marker relative to the reference marker, expressed in the reference marker axes."""

    dx_mm: float  # + toward the reference marker's right edge
    dy_mm: float  # + toward the reference marker's top edge
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
    view_shift_px: float | None = None
    position_ratio: float | None = None  # distance / position tolerance
    yaw_ratio: float | None = None  # |dyaw| / yaw tolerance
    deviation_level: DeviationLevel | None = None

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {"status": self.status.value, "reason": self.reason}
        if self.offset is not None:
            result.update(
                dx_mm=round(self.offset.dx_mm, 1),
                dy_mm=round(self.offset.dy_mm, 1),
                dyaw_deg=round(self.offset.dyaw_deg, 2),
                distance_mm=round(self.offset.distance_mm, 1),
            )
        if self.deviation_level is not None:
            result.update(
                deviation_level=self.deviation_level.value,
                position_ratio=round(self.position_ratio or 0.0, 2),
                yaw_ratio=round(self.yaw_ratio or 0.0, 2),
            )
        if self.view_shift_px is not None:
            result["view_shift_px"] = round(self.view_shift_px, 1)
        return result


@dataclass(frozen=True)
class Intrinsics:
    camera_matrix: np.ndarray
    distortion: np.ndarray
    width: int
    height: int
    calibration_id: str | None
    path: Path
    sha256: str


@dataclass(frozen=True)
class ParkingSpot:
    spot_id: str
    camera_id: str
    dictionary: str
    marker_id: int
    marker_side_mm: float
    plane: MarkerPose  # reference marker pose; only its plane and axes are used for measuring
    reference: PlaneMarker
    reference_corners_px: np.ndarray  # 4x2, for drawing the registered spot
    # Fixed marker id -> registered center pixel. Empty only when registered with the view check off.
    anchors: dict[int, np.ndarray]
    tolerance: ParkingTolerance
    intrinsics: Intrinsics
    marker_forward_axis: str | None = None  # key of MARKER_FORWARD_AXES, or None when not recorded
    max_view_shift_px: float = MAX_VIEW_SHIFT_PX
    single_anchor_trusted: bool = False
    # Corner detection differs slightly between OpenCV releases (~1 mm here between 4.6 and 4.14).
    registration_opencv_version: str | None = None


@dataclass(frozen=True)
class FrameObservation:
    offset: PlanarOffset | None
    corners: np.ndarray | None
    reason: str
    blocking_status: ParkingStatus | None  # AMBIGUOUS or VIEW_UNVERIFIED invalidates earlier frames too
    view_shift_px: float | None
    # Status to report when this unusable frame leaves nothing to judge; a skipped frame keeps earlier ones.
    skipped_status: ParkingStatus = ParkingStatus.NOT_FOUND
    view_shift_xy_px: tuple[float, float] | None = None  # fixed-marker drift subtracted from this frame


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_intrinsics(path: Path, camera_id: str) -> Intrinsics:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("camera_id") != camera_id:
        raise ParkingSpotError(f"내부 보정 파일의 camera_id가 {data.get('camera_id')!r}입니다 ({camera_id!r} 필요): {path}")
    if data.get("pixel_transform_id") != PIXEL_TRANSFORM_ID:
        raise ParkingSpotError(f"내부 보정 파일의 pixel_transform_id가 다릅니다: {data.get('pixel_transform_id')!r}")
    return Intrinsics(
        camera_matrix=np.array(data["camera_matrix"], dtype=np.float64),
        distortion=np.array(data["distortion_coefficients"], dtype=np.float64),
        width=int(data["image_size_px"]["width"]),
        height=int(data["image_size_px"]["height"]),
        calibration_id=data.get("calibration_id"),
        path=path,
        sha256=sha256_of(path),
    )


def load_spot(path: Path, repository_root: Path) -> ParkingSpot:
    if not path.is_file():
        raise ParkingSpotError(f"등록된 주차 위치가 없습니다: {path} (먼저 register)")
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ParkingSpotError(
            f"지원하지 않는 주차 위치 schema {data.get('schema_version')} (필요 {SCHEMA_VERSION}). 다시 등록하세요: {path}"
        )
    intrinsics = load_intrinsics(repository_root / data["intrinsic_calibration"]["path"], data["camera_id"])
    if intrinsics.sha256 != data["intrinsic_calibration"]["sha256"]:
        raise ParkingSpotError("등록 이후 내부 보정 파일이 바뀌었습니다. 주차 위치를 다시 등록하세요.")
    plane = MarkerPose(
        np.array(data["reference_pose_in_camera"]["rotation"], dtype=np.float64),
        np.array(data["reference_pose_in_camera"]["translation_mm"], dtype=np.float64),
    )
    corners = np.array(data["reference_corners_px"], dtype=np.float64)
    if not data["anchors"] and data.get("view_check", {}).get("mode") != "OFF":
        raise ParkingSpotError(f"고정 마커가 없는데 view_check.mode가 OFF로 기록되지 않았습니다: {path}")
    tolerance = ParkingTolerance(float(data["tolerance"]["position_mm"]), float(data["tolerance"]["yaw_deg"]))
    if not (tolerance.position_mm > 0 and tolerance.yaw_deg > 0):
        raise ParkingSpotError(f"허용 오차는 0보다 커야 합니다: {data['tolerance']}")
    max_view_shift = float(data.get("view_check", {}).get("max_anchor_shift_px", MAX_VIEW_SHIFT_PX))
    single_anchor_trusted = data.get("view_check", {}).get("single_anchor_trusted", False)
    if not isinstance(single_anchor_trusted, bool):
        raise ParkingSpotError(f"view_check.single_anchor_trusted는 true/false여야 합니다: {single_anchor_trusted!r}")
    if not max_view_shift > 0:
        raise ParkingSpotError(f"view_check.max_anchor_shift_px는 0보다 커야 합니다: {max_view_shift}")
    dictionary = data["aruco_dictionary"]
    if not isinstance(dictionary, str) or not dictionary.startswith("DICT_") or not hasattr(cv2.aruco, dictionary):
        raise ParkingSpotError(f"지원하지 않는 ArUco 사전입니다: {dictionary!r}")
    forward_axis = data.get("marker_forward_axis")
    if forward_axis is not None and forward_axis not in MARKER_FORWARD_AXES:
        raise ParkingSpotError(f"marker_forward_axis는 {sorted(MARKER_FORWARD_AXES)} 중 하나여야 합니다: {forward_axis!r}")
    return ParkingSpot(
        spot_id=data["spot_id"],
        camera_id=data["camera_id"],
        dictionary=dictionary,
        marker_id=int(data["marker_id"]),
        marker_side_mm=float(data["marker_side_mm"]),
        plane=plane,
        reference=project_to_plane(corners, plane, intrinsics.camera_matrix, intrinsics.distortion),
        reference_corners_px=corners,
        anchors={int(a["marker_id"]): np.array(a["center_px"], dtype=np.float64) for a in data["anchors"]},
        tolerance=tolerance,
        intrinsics=intrinsics,
        marker_forward_axis=forward_axis,
        max_view_shift_px=max_view_shift,
        single_anchor_trusted=single_anchor_trusted,
        registration_opencv_version=data.get("opencv_version"),
    )


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


def project_to_plane(
    corners_px: np.ndarray, plane: MarkerPose, camera_matrix: np.ndarray, distortion: np.ndarray
) -> PlaneMarker:
    # Pinky's marker stays on the floor-parallel plane it was registered on, so intersect each corner ray
    # with that plane instead of trusting a per-frame PnP depth. On the ~40 px ceiling-camera marker the
    # PnP depth drifted ~45 mm and leaked ~20 mm into the in-plane offset; the plane keeps it under 0.1 mm.
    rays = cv2.undistortPoints(np.asarray(corners_px, dtype=np.float64).reshape(4, 1, 2), camera_matrix, distortion)
    rays = np.hstack([rays.reshape(4, 2), np.ones((4, 1))])
    normal = plane.rotation[:, 2]
    points = rays * (float(normal @ plane.translation_mm) / (rays @ normal))[:, None]
    local = (plane.rotation.T @ (points - plane.translation_mm).T).T[:, :2]
    x_edge = (local[1] - local[0]) + (local[2] - local[3])  # both edges running toward the marker's right
    sides = np.linalg.norm(local - np.roll(local, -1, axis=0), axis=1)
    center = local.mean(axis=0)
    return PlaneMarker(float(center[0]), float(center[1]), math.degrees(math.atan2(x_edge[1], x_edge[0])), float(sides.mean()))


def plane_offset(reference: PlaneMarker, current: PlaneMarker) -> PlanarOffset:
    dyaw = (current.yaw_deg - reference.yaw_deg + 180.0) % 360.0 - 180.0
    return PlanarOffset(current.x_mm - reference.x_mm, current.y_mm - reference.y_mm, dyaw)


def pinky_frame_offset(offset: PlanarOffset, forward_axis: str) -> tuple[float, float]:
    """(forward, left) in mm from Pinky's own view; dyaw already reads as a left turn when positive."""
    fx, fy = MARKER_FORWARD_AXES[forward_axis]
    return offset.dx_mm * fx + offset.dy_mm * fy, -offset.dx_mm * fy + offset.dy_mm * fx


def anchor_shifts(anchors: dict[int, np.ndarray], found: dict[int, list[np.ndarray]]) -> list[np.ndarray]:
    """(x, y) pixel shift of each registered fixed marker that is visible exactly once."""
    return [
        found[marker_id][0].mean(axis=0) - center
        for marker_id, center in anchors.items()
        if len(found.get(marker_id, [])) == 1
    ]


def anchor_shift_px(anchors: dict[int, np.ndarray], found: dict[int, list[np.ndarray]]) -> np.ndarray | None:
    """Median (x, y) pixel shift of the registered fixed markers visible exactly once, or None if none are."""
    shifts = anchor_shifts(anchors, found)
    return np.median(shifts, axis=0) if shifts else None


def observe_frame(image: np.ndarray, spot: ParkingSpot) -> FrameObservation:
    found = detect_markers(image, spot.dictionary)
    if not spot.anchors:  # view check off: a wrapped or moved view goes unnoticed and shows up as an offset
        return _observe_marker(found, spot, np.zeros(2), None)
    shifts = anchor_shifts(spot.anchors, found)
    if not shifts:  # a small fixed marker drops out for single frames; skip it like a missed Pinky marker
        return FrameObservation(
            None, None, "no fixed marker visible to verify the camera view", None, None, ParkingStatus.VIEW_UNVERIFIED
        )
    view_shift = np.median(shifts, axis=0)
    shift = float(np.linalg.norm(view_shift))
    vector = (float(view_shift[0]), float(view_shift[1]))

    def unverified(reason: str) -> FrameObservation:
        return FrameObservation(None, None, reason, ParkingStatus.VIEW_UNVERIFIED, shift, view_shift_xy_px=vector)

    if shift > spot.max_view_shift_px:
        return unverified(f"fixed markers moved {shift:.1f} px from registration")
    disagreement = max(float(np.linalg.norm(s - view_shift)) for s in shifts)
    if disagreement > ANCHOR_AGREE_PX:
        return unverified(f"fixed markers moved differently ({disagreement:.1f} px apart); one may have been moved")
    if len(shifts) == 1 and not spot.single_anchor_trusted and shift > SINGLE_ANCHOR_MAX_SHIFT_PX:
        return unverified(
            f"only one fixed marker visible and it moved {shift:.1f} px; cannot tell camera drift from a moved marker"
        )
    return replace(_observe_marker(found, spot, view_shift, shift), view_shift_xy_px=vector)


def _observe_marker(
    found: dict[int, list[np.ndarray]], spot: ParkingSpot, view_shift: np.ndarray, shift: float | None
) -> FrameObservation:
    markers = found.get(spot.marker_id, [])
    if not markers:
        return FrameObservation(None, None, "marker not visible", None, shift)
    if len(markers) > 1:  # 같은 ID가 둘이면 어느 쪽이 Pinky인지 모른다
        return FrameObservation(None, None, f"marker {spot.marker_id} seen {len(markers)} times", ParkingStatus.AMBIGUOUS, shift)
    # The fixed markers lie on the same floor, so their small shift is the view drift to undo.
    current = project_to_plane(
        markers[0] - view_shift, spot.plane, spot.intrinsics.camera_matrix, spot.intrinsics.distortion
    )
    finite = all(math.isfinite(v) for v in (current.x_mm, current.y_mm, current.yaw_deg, current.side_mm))
    # Written as "not within" so a NaN side from a degenerate projection is rejected instead of passing.
    if not finite or not abs(current.side_mm - spot.marker_side_mm) <= MAX_SIDE_ERROR_RATIO * spot.marker_side_mm:
        return FrameObservation(
            None, markers[0], f"marker side {current.side_mm:.1f} mm on the registered plane (printed {spot.marker_side_mm:g} mm)",
            None, shift,
        )
    return FrameObservation(plane_offset(spot.reference, current), markers[0], "", None, shift)


def median_offset(offsets: list[PlanarOffset]) -> PlanarOffset:
    """Median over frames; yaw is unwrapped around the first sample so ±180° does not average to 0."""
    base = offsets[0].dyaw_deg
    yaws = [base + (o.dyaw_deg - base + 180.0) % 360.0 - 180.0 for o in offsets]
    return PlanarOffset(
        float(np.median([o.dx_mm for o in offsets])),
        float(np.median([o.dy_mm for o in offsets])),
        float((np.median(yaws) + 180.0) % 360.0 - 180.0),
    )


def judge(offset: PlanarOffset | None, tolerance: ParkingTolerance, reason: str = "") -> ParkingResult:
    if offset is None:
        return ParkingResult(ParkingStatus.NOT_FOUND, None, reason or "marker not visible")
    if not all(math.isfinite(v) for v in (offset.dx_mm, offset.dy_mm, offset.dyaw_deg)):
        # NaN compares false against every limit and would otherwise read as ARRIVED.
        return ParkingResult(ParkingStatus.NOT_FOUND, None, "non-finite offset (marker projection failed)")
    position_ratio = offset.distance_mm / tolerance.position_mm
    yaw_ratio = abs(offset.dyaw_deg) / tolerance.yaw_deg
    worst = max(position_ratio, yaw_ratio)
    level = (
        DeviationLevel.WITHIN if worst <= 1.0
        else DeviationLevel.NEAR if worst <= NEAR_LIMIT_RATIO
        else DeviationLevel.FAR
    )
    outside = []
    if position_ratio > 1.0:
        outside.append(f"position {offset.distance_mm:.0f} mm > {tolerance.position_mm:g} mm")
    if yaw_ratio > 1.0:
        outside.append(f"yaw {offset.dyaw_deg:+.1f} deg > ±{tolerance.yaw_deg:g} deg")
    status, reason = (ParkingStatus.MISALIGNED, "; ".join(outside)) if outside else (ParkingStatus.ARRIVED, "within tolerance")
    return ParkingResult(status, offset, reason, None, position_ratio, yaw_ratio, level)


class ParkingJudge:
    """Median judgment over the most recent usable frames, shared by the CLI and later callers."""

    def __init__(self, tolerance: ParkingTolerance, frames: int) -> None:
        self.tolerance = tolerance
        self.frames = frames
        self.recent: list[PlanarOffset] = []
        self.misses = 0
        self.result = judge(None, tolerance, "no frame yet")

    def add(self, observation: FrameObservation) -> ParkingResult:
        if observation.blocking_status is not None:
            self.recent.clear()
            self.result = ParkingResult(observation.blocking_status, None, observation.reason, observation.view_shift_px)
            return self.result
        if observation.offset is None:
            self.misses += 1
            if self.misses >= self.frames:  # 연속으로 못 찾으면 이전 값을 버린다
                self.recent.clear()
        else:
            self.misses = 0
            self.recent = (self.recent + [observation.offset])[-self.frames:]
        if self.recent:
            self.result = judge(median_offset(self.recent), self.tolerance)
        else:
            self.result = ParkingResult(observation.skipped_status, None, observation.reason or "marker not visible")
        self.result = replace(self.result, view_shift_px=observation.view_shift_px)
        return self.result
