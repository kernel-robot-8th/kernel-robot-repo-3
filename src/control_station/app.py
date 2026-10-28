"""HTTP API and small status page for the robot control station."""

from __future__ import annotations

import json
import os
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from control_station.arm_supervisor import ArmTeleopSupervisor
from control_station.camera_adapters import (
    OpenCVArmCameraStreamManager,
    UnconfiguredArmCameraStreamManager,
)
from control_station.robot_adapters import make_status_provider
from control_station.store import TaskStore


WEB_DIR = Path(__file__).with_name("web")


class ControlStationServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class ControlStationHandler(SimpleHTTPRequestHandler):
    server_version = "ControlStation/0.2"

    @property
    def app(self) -> ControlStationServer:
        return self.server  # type: ignore[return-value]

    def log_message(self, format: str, *args: object) -> None:
        super().log_message(format, *args)

    def _json(self, status: int, body: dict[str, Any]) -> None:
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        route = urlsplit(self.path).path
        if route.startswith("/streams/") and route.endswith(".mjpg"):
            camera_id = route.removeprefix("/streams/").removesuffix(".mjpg")
            self._stream_camera(camera_id)
            return
        if route == "/api/health":
            self._json(200, {"status": "ok", "mode": self.app.mode})
        elif route == "/api/robots":
            self._json(200, {"robots": self.app.robot_provider.snapshots()})
        elif route == "/api/cameras":
            self._json(
                200,
                {"cameras": [camera.to_dict() for camera in self.app.camera_manager.snapshots()]},
            )
        elif route == "/api/arm/status":
            self._json(200, {"arm": self.app.arm_supervisor.snapshot()})
        elif route == "/api/tasks":
            self._json(200, {"tasks": self.app.task_store.list_tasks()})
        elif route.startswith("/api/tasks/"):
            task_id = route.removeprefix("/api/tasks/")
            if not task_id or "/" in task_id:
                self._json(404, {"error": "Task not found"})
                return
            task = self.app.task_store.get_task(task_id)
            if task is None:
                self._json(404, {"error": "Task not found"})
            else:
                self._json(200, {"task": task})
        else:
            if route == "/":
                self.path = "/index.html"
            elif route.startswith("/api/"):
                self._json(404, {"error": "API route not found"})
                return
            super().do_GET()

    def _stream_camera(self, camera_id: str) -> None:
        manager = self.app.camera_manager
        known_ids = {camera.id for camera in manager.snapshots()}
        if camera_id not in known_ids or not hasattr(manager, "sequence_for"):
            self._json(404, {"error": "Camera stream not found"})
            return

        frame = manager.frame_for(camera_id)
        for _ in range(50):
            if frame is not None:
                break
            time.sleep(0.1)
            frame = manager.frame_for(camera_id)
        if frame is None:
            self._json(503, {"error": "Camera is not producing frames"})
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Connection", "close")
        self.end_headers()
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
                self.wfile.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                    + frame
                    + b"\r\n"
                )
                self.wfile.flush()
                last_sequence = sequence
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def do_POST(self) -> None:
        route = urlsplit(self.path).path
        if route in {"/api/arm/start", "/api/arm/stop"}:
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self._json(403, {"error": "Cross-origin arm-control request rejected"})
                return
            try:
                state = (
                    self.app.arm_supervisor.start()
                    if route == "/api/arm/start"
                    else self.app.arm_supervisor.stop()
                )
            except RuntimeError as exc:
                self._json(409, {"error": str(exc), "arm": self.app.arm_supervisor.snapshot()})
                return
            self._json(202, {"arm": state})
            return
        if route != "/api/tasks":
            self._json(404, {"error": "API route not found"})
            return
        if self.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
            self._json(415, {"error": "Content-Type must be application/json"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._json(400, {"error": "Invalid Content-Length"})
            return
        if not 0 < length <= 4096:
            self._json(400, {"error": "Request body must be between 1 and 4096 bytes"})
            return
        try:
            body = json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self._json(400, {"error": "Request body must be valid JSON"})
            return
        if not isinstance(body, dict):
            self._json(400, {"error": "Request body must be a JSON object"})
            return
        destination = body.get("destination")
        arm_job = body.get("arm_job")
        if not isinstance(destination, str) or not isinstance(arm_job, str):
            self._json(400, {"error": "destination and arm_job must be strings"})
            return
        destination, arm_job = destination.strip(), arm_job.strip()
        if not destination or len(destination) > 40:
            self._json(400, {"error": "destination must contain 1 to 40 characters"})
            return
        if not arm_job or len(arm_job) > 120:
            self._json(400, {"error": "arm_job must contain 1 to 120 characters"})
            return
        task = self.app.task_store.create_task(destination, arm_job)
        self._json(201, {"task": task})


def main() -> None:
    mode = os.environ.get("CONTROL_STATION_MODE", "mock").strip().lower()
    provider = make_status_provider(mode)
    task_store = TaskStore()
    host = "127.0.0.1"
    port = int(os.environ.get("CONTROL_STATION_PORT", "8000"))
    handler = partial(ControlStationHandler, directory=str(WEB_DIR))
    server = ControlStationServer((host, port), handler)
    server.mode = mode
    server.robot_provider = provider
    server.arm_supervisor = ArmTeleopSupervisor()
    head_yolo_enabled = bool(os.environ.get("CONTROL_STATION_HEAD_YOLO_WEIGHTS", "").strip())
    server.camera_manager = (
        OpenCVArmCameraStreamManager(joint_enabled=server.arm_supervisor.enabled)
        if server.arm_supervisor.enabled or head_yolo_enabled
        else UnconfiguredArmCameraStreamManager()
    )
    server.task_store = task_store
    print(f"Control station ({mode} mode): http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nControl station stopped.", flush=True)
    finally:
        server.server_close()
        server.camera_manager.close()
        server.arm_supervisor.close()
        task_store.close()
        provider.close()


if __name__ == "__main__":
    main()
