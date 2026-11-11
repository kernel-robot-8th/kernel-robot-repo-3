"""Front-camera shelf coordinates -> OMX arm coordinates (omx_link0), from touches measured on site.

The front camera puts a detected object on the shelf plane (`position_shelf_m`, shelf_front frame). The arm's
pose relative to the shelf is not measured directly. Instead an operator touched an object with the arm's fingertip
at two places while the camera saw it there, and the planar rotation + translation that maps the camera's numbers
onto the arm's fingertip positions was fitted (PC/arm specific file, CONTROL_STATION_CAMERA_TO_ARM). Because the
fit pairs the camera's own output with the touched positions, a constant camera bias (its depth estimate is weak)
is absorbed; anything else is not, so a target can be off by about the fit residual (about 1 cm on 2026-11-07).

`hover_target` turns an object position into a MOVE_TO_POSITION target above it: the fingertip some centimetres
above the touch height and slightly back toward the arm, the wrist pitched down. The MOVE_TO_POSITION position is
the jaw mount (the tool point of scripts/omx_kinematics.py), so the finger length is subtracted along the wrist.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Wrist pitch (degrees below horizontal) for each MOVE_TO_POSITION approach; the arm worker uses the same table.
APPROACH_PITCH_DEG = {"FRONT_HORIZONTAL": 0.0, "PITCH_DOWN_45": 45.0}


class CameraToArmError(ValueError):
    pass


@dataclass(frozen=True)
class CameraToArm:
    path: Path
    theta_rad: float
    translation_m: tuple[float, float]
    touch_z_m: float
    finger_m: float
    follower_board: str
    max_fit_error_m: float

    def to_arm(self, shelf_xy: tuple[float, float]) -> tuple[float, float]:
        c, s = math.cos(self.theta_rad), math.sin(self.theta_rad)
        x, y = shelf_xy
        return (c * x - s * y + self.translation_m[0], s * x + c * y + self.translation_m[1])

    def hover_target(
        self, position_shelf_m: dict[str, float], *, above_m: float = 0.06, back_m: float = 0.02,
        approach: str = "PITCH_DOWN_45",
    ) -> dict[str, Any]:
        """MOVE_TO_POSITION target (jaw mount, omx_link0) with the fingertip above the object."""
        ox, oy = self.to_arm((float(position_shelf_m["x"]), float(position_shelf_m["y"])))
        reach = math.hypot(ox, oy)
        if reach < 0.05:
            raise CameraToArmError("object is too close to the arm's base axis")
        ux, uy = ox / reach, oy / reach  # horizontal direction from the arm base to the object
        pitch = math.radians(APPROACH_PITCH_DEG[approach])
        fingertip = (ox - back_m * ux, oy - back_m * uy, self.touch_z_m + above_m)
        jaw = (
            fingertip[0] - self.finger_m * math.cos(pitch) * ux,
            fingertip[1] - self.finger_m * math.cos(pitch) * uy,
            fingertip[2] + self.finger_m * math.sin(pitch),
        )
        return {
            "frame_id": "omx_link0",
            "unit": "m",
            "position": {axis: round(value, 4) for axis, value in zip("xyz", jaw)},
            "approach": approach,
            "object_arm_m": {"x": round(ox, 4), "y": round(oy, 4), "z": round(self.touch_z_m, 4)},
            "fingertip_arm_m": {axis: round(value, 4) for axis, value in zip("xyz", fingertip)},
        }


    def object_center_target(self, position_shelf_m: dict[str, float], *, below_touch_m: float = 0.008) -> dict[str, Any]:
        """PICK_AT_POSITION target: the object's centre in arm coordinates. Its height is the fingertip height
        measured when touching the top of the carrot, minus about half the carrot's thickness."""
        ox, oy = self.to_arm((float(position_shelf_m["x"]), float(position_shelf_m["y"])))
        return {
            "frame_id": "omx_link0",
            "unit": "m",
            "position": {"x": round(ox, 4), "y": round(oy, 4), "z": round(self.touch_z_m - below_touch_m, 4)},
            "approach": "PITCH_DOWN_45",
        }


def load_camera_to_arm(path: str | Path | None = None) -> CameraToArm | None:
    """The calibration named by CONTROL_STATION_CAMERA_TO_ARM, or None when it is not set."""
    raw_path = path or os.environ.get("CONTROL_STATION_CAMERA_TO_ARM", "").strip()
    if not raw_path:
        return None
    file = Path(raw_path)
    try:
        document = json.loads(file.read_text(encoding="utf-8"))
        points = document["points"]
        calibration = CameraToArm(
            path=file,
            theta_rad=float(document["theta_rad"]),
            translation_m=(float(document["translation_m"][0]), float(document["translation_m"][1])),
            touch_z_m=float(document["carrot_touch_z_m"]),
            finger_m=float(document["fingertip_offset_m"]),
            follower_board=str(document["follower_board"]),
            max_fit_error_m=0.0,
        )
    except (OSError, KeyError, TypeError, ValueError, IndexError) as exc:
        raise CameraToArmError(f"카메라→팔 변환 파일을 읽지 못했습니다 ({file}): {exc}") from exc
    if len(points) < 2:
        raise CameraToArmError("카메라→팔 변환에는 측정점이 2개 이상 필요합니다")
    errors = [
        math.dist(calibration.to_arm(tuple(point["camera"][:2])), tuple(point["arm"][:2])) for point in points
    ]
    return CameraToArm(**{**calibration.__dict__, "max_fit_error_m": round(max(errors), 4)})
