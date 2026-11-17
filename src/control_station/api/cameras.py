"""카메라 목록, 영상 스트림"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_cameras(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """카메라 목록"""
    zones = h.app.shelf_slot_zones
    h._json(200, {"cameras": [zones.annotate(camera.to_dict()) for camera in h.app.camera_manager.snapshots()]})


def get_stream(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """카메라 MJPEG 스트림"""
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
