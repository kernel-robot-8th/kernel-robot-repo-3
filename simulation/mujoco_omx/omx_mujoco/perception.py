"""A small RGB-D red-fruit detector for the synthetic strawberry scene."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .camera import CameraRig
from .simulation import OMXSimulation


@dataclass(frozen=True)
class StrawberryObservation:
    camera_name: str
    pixel_xy: tuple[float, float]
    red_pixel_count: int
    median_depth_m: float
    visible_surface_world_m: tuple[float, float, float]
    estimated_center_world_m: tuple[float, float, float]


def detect_strawberry(
    simulation: OMXSimulation,
    *,
    camera_name: str = "fixed_camera",
    width: int = 640,
    height: int = 480,
    estimated_radius_m: float = 0.025,
) -> StrawberryObservation | None:
    """Find a red blob in rendered RGB and unproject it with camera depth.

    The center correction uses the approximate proxy radius; this is a scene
    experiment, not a general detector or a calibrated physical camera model.
    """
    if not math.isfinite(estimated_radius_m) or not 0.0 <= estimated_radius_m <= 0.1:
        raise ValueError("estimated_radius_m must be finite and between 0 and 0.1 m")

    with CameraRig(simulation, width=width, height=height) as cameras:
        rgb = cameras.render_rgb(camera_name)
        depth = cameras.render_depth(camera_name)

    red = rgb[:, :, 0].astype(np.int16)
    green = rgb[:, :, 1].astype(np.int16)
    blue = rgb[:, :, 2].astype(np.int16)
    mask = (red > 75) & (red > green * 1.45) & (red > blue * 1.35)
    ys, xs = np.nonzero(mask)
    if xs.size < 12:
        return None

    valid_depth = depth[ys, xs]
    valid = np.isfinite(valid_depth) & (valid_depth > 0.0) & (valid_depth < 5.0)
    if int(np.count_nonzero(valid)) < 12:
        return None
    xs = xs[valid]
    ys = ys[valid]
    valid_depth = valid_depth[valid]
    u = float(np.median(xs))
    v = float(np.median(ys))
    z_camera = float(np.median(valid_depth))

    camera_id = simulation.mujoco.mj_name2id(
        simulation.model, simulation.mujoco.mjtObj.mjOBJ_CAMERA, camera_name
    )
    if camera_id < 0:
        raise ValueError(f"Simulation camera is missing: {camera_name}")
    fovy_rad = math.radians(float(simulation.model.cam_fovy[camera_id]))
    focal_px = height / (2.0 * math.tan(fovy_rad / 2.0))
    camera_point = np.asarray(
        [
            (u - width / 2.0) * z_camera / focal_px,
            (height / 2.0 - v) * z_camera / focal_px,
            -z_camera,
        ],
        dtype=np.float64,
    )
    rotation_world_camera = simulation.data.cam_xmat[camera_id].reshape(3, 3)
    camera_position_world = simulation.data.cam_xpos[camera_id]
    visible_surface = camera_position_world + rotation_world_camera @ camera_point
    camera_forward_world = -rotation_world_camera[:, 2]
    estimated_center = visible_surface + estimated_radius_m * camera_forward_world

    return StrawberryObservation(
        camera_name=camera_name,
        pixel_xy=(u, v),
        red_pixel_count=int(xs.size),
        median_depth_m=z_camera,
        visible_surface_world_m=tuple(float(value) for value in visible_surface),
        estimated_center_world_m=tuple(float(value) for value in estimated_center),
    )
