#!/usr/bin/env python3
"""Render fixed and wrist RGB images from the MuJoCo simulation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "simulation" / "mujoco_omx"))

from omx_mujoco import OMXSimulation  # noqa: E402
from omx_mujoco.camera import CAMERA_NAMES, CameraRig, save_rgb_png  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--scene", type=Path, help="optional MJCF scene path")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPOSITORY_ROOT / ".control-station" / "mujoco-python-camera",
    )
    parser.add_argument("--display", action="store_true", help="show images using optional OpenCV")
    args = parser.parse_args()
    try:
        simulation = OMXSimulation(args.scene) if args.scene else OMXSimulation()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        with CameraRig(simulation, args.width, args.height) as cameras:
            images = {name: cameras.render_rgb(name) for name in CAMERA_NAMES}
        for name, image in images.items():
            output_path = args.output_dir / f"{name}.png"
            save_rgb_png(output_path, image)
            print(f"{name}: RGB {image.shape}, saved {output_path}")
        if args.display:
            try:
                import cv2
            except ImportError as exc:
                raise RuntimeError("--display requires OpenCV; omit it to write RGB PNG files") from exc
            for name, image in images.items():
                cv2.imshow(name, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            print("Press any key in an image window to close the OpenCV views.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"Camera render failed: {exc}", file=sys.stderr)
        return 2
    print("Synthetic MuJoCo camera images only; no physical camera was opened.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
