"""Camera list, MJPEG streams and the on-demand camera target observation (observation only)."""

from __future__ import annotations

import re
import time
from typing import TYPE_CHECKING
from urllib.parse import parse_qs, urlsplit

from control_station.api.observations import target_observation
from control_station.command_parser import MAX_SHELF_LEVEL

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_cameras(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    zones = h.app.shelf_slot_zones
    h._json(200, {"cameras": [zones.annotate(camera.to_dict()) for camera in h.app.camera_manager.snapshots()]})


def get_camera_target_observation(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """GET /api/cameras/{camera_id}/target-observation?class_name=&color=&shelf_level="""
    camera_id = route.removeprefix("/api/cameras/").removesuffix("/target-observation")
    query = parse_qs(urlsplit(h.path).query, keep_blank_values=True)
    class_names = query.get("class_name", [])
    colors = query.get("color", [])
    shelf_levels = query.get("shelf_level", [])
    if (
        set(query) - {"class_name", "color", "shelf_level"}
        or len(class_names) != 1
        or len(colors) > 1
        or len(shelf_levels) > 1
    ):
        h._json(400, {"error": "class_name is required once; color and shelf_level may be provided once"})
        return
    target_class = class_names[0].strip()
    if not target_class or len(target_class) > 80:
        h._json(400, {"error": "class_name must contain 1 to 80 characters"})
        return
    requested_color = colors[0].strip() if colors else None
    if requested_color is not None and (not requested_color or len(requested_color) > 40):
        h._json(400, {"error": "color must contain 1 to 40 characters"})
        return
    requested_shelf_level: int | None = None
    if shelf_levels:
        raw_level = shelf_levels[0].strip()
        if (
            not re.fullmatch(r"[1-9][0-9]*", raw_level)
            or len(raw_level) > len(str(MAX_SHELF_LEVEL))
            or int(raw_level) > MAX_SHELF_LEVEL
        ):
            h._json(400, {"error": f"shelf_level must be an integer from 1 to {MAX_SHELF_LEVEL}"})
            return
        requested_shelf_level = int(raw_level)
    camera = next((item for item in h.app.camera_manager.snapshots() if item.id == camera_id), None)
    if camera is None:
        h._json(404, {"error": "Camera not found"})
        return
    h._json(
        200,
        {"observation": target_observation(camera, target_class, requested_color, requested_shelf_level)},
    )


def get_stream(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """GET /streams/{camera_id}.mjpg: the camera's latest JPEG frames as multipart MJPEG until the client leaves."""
    camera_id = route.removeprefix("/streams/").removesuffix(".mjpg")
    manager = h.app.camera_manager
    known_ids = {camera.id for camera in manager.snapshots()}
    if camera_id not in known_ids or not hasattr(manager, "sequence_for"):
        h._json(404, {"error": "Camera stream not found"})
        return

    frame = manager.frame_for(camera_id)
    for _ in range(50):
        if frame is not None:
            break
        time.sleep(0.1)
        frame = manager.frame_for(camera_id)
    if frame is None:
        h._json(503, {"error": "Camera is not producing frames"})
        return

    h.send_response(200)
    h.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
    h.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
    h.send_header("Connection", "close")
    h.end_headers()
    last_sequence = -1
    try:
        while True:
            sequence = manager.sequence_for(camera_id)
            if sequence == last_sequence:
                time.sleep(0.02)
                continue
            frame = manager.frame_for(camera_id)
            if frame is None:
                time.sleep(0.05)
                continue
            h.wfile.write(
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                + frame
                + b"\r\n"
            )
            h.wfile.flush()
            last_sequence = sequence
    except (BrokenPipeError, ConnectionResetError, OSError):
        return
