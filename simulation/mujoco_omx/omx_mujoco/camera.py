"""RGB rendering helpers using MuJoCo's Python renderer."""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np

from .simulation import OMXSimulation


CAMERA_NAMES = ("fixed_camera", "wrist_camera")


class CameraRig:
    def __init__(self, simulation: OMXSimulation, width: int = 640, height: int = 480):
        if not 32 <= width <= 2048 or not 32 <= height <= 2048:
            raise ValueError("camera width and height must be between 32 and 2048")
        self.simulation = simulation
        self.width = width
        self.height = height
        self._renderer = simulation.mujoco.Renderer(
            simulation.model, width=width, height=height
        )
        self._camera_ids = {
            name: int(
                simulation.mujoco.mj_name2id(
                    simulation.model, simulation.mujoco.mjtObj.mjOBJ_CAMERA, name
                )
            )
            for name in CAMERA_NAMES
        }
        missing = [name for name, camera_id in self._camera_ids.items() if camera_id < 0]
        if missing:
            self._renderer.close()
            raise ValueError(f"Required camera is missing: {', '.join(missing)}")

    def render_rgb(self, camera_name: str) -> np.ndarray:
        if camera_name not in self._camera_ids:
            raise ValueError(f"Unsupported camera: {camera_name}")
        self._renderer.update_scene(self.simulation.data, camera=camera_name)
        image = np.asarray(self._renderer.render())
        if image.dtype != np.uint8 or image.shape != (self.height, self.width, 3):
            raise RuntimeError("MuJoCo renderer returned an unexpected RGB image")
        return image.copy()

    def render_depth(self, camera_name: str) -> np.ndarray:
        """Return MuJoCo's metric camera-plane depth image in meters."""
        if camera_name not in self._camera_ids:
            raise ValueError(f"Unsupported camera: {camera_name}")
        self._renderer.enable_depth_rendering()
        try:
            self._renderer.update_scene(self.simulation.data, camera=camera_name)
            depth = np.asarray(self._renderer.render()).copy()
        finally:
            self._renderer.disable_depth_rendering()
        if depth.shape != (self.height, self.width) or depth.dtype != np.float32:
            raise RuntimeError("MuJoCo renderer returned unexpected depth data")
        return depth

    def close(self) -> None:
        self._renderer.close()

    def __enter__(self) -> "CameraRig":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def save_rgb_png(path: str | Path, rgb: np.ndarray) -> None:
    """Write an RGB uint8 PNG using the standard library only."""
    pixels = np.asarray(rgb)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
        raise ValueError("rgb must be an HxWx3 uint8 array")
    height, width = pixels.shape[:2]

    def chunk(name: bytes, payload: bytes) -> bytes:
        checksum = zlib.crc32(name + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + name + payload + struct.pack(">I", checksum)

    scanlines = b"".join(b"\x00" + pixels[row].tobytes() for row in range(height))
    data = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(scanlines, level=6))
        + chunk(b"IEND", b"")
    )
    Path(path).write_bytes(data)
