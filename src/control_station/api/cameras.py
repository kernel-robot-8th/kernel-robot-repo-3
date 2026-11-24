"""카메라 목록과 MJPEG 영상 스트림 API.

GET /api/cameras            카메라마다 상태·검출 결과(선반 칸 표시 포함)와 핑키 쟁반·선반 영역의 현재 개수
GET /streams/{camera}.mjpg  브라우저 <img>에 그대로 넣는 MJPEG 스트림

카메라를 실제로 여는 것은 cameras/adapters.py의 카메라 관리자(h.app.camera_manager)이고, 여기서는 그 최신
프레임과 스냅샷을 읽기만 한다.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_cameras(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """카메라 목록. 검출 결과마다 어느 선반 칸에 있는지를 붙여(annotate) 돌려준다."""
    zones = h.app.shelf_slot_zones
    payload: dict = {"cameras": [zones.annotate(camera.to_dict()) for camera in h.app.camera_manager.snapshots()]}
    load_view = getattr(h.app, "load_view", None)
    if load_view is not None:  # 탑재 목록: 최신 프레임에서 핑키 쟁반 위에 보이는 물체 (화면 표시 전용)
        payload["pinky_load"] = load_view.live()
    # 탑 캠 화면 중 선반 영역(head_shelf)과 가운데 영역(head_middle)의 현재 개수. 설정되지 않았으면 빠진다.
    for region in ("head_shelf", "head_middle"):
        view = getattr(h.app, f"{region}_view", None)
        if view is not None:
            payload[region] = view.live()
    h._json(200, payload)


def get_stream(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """카메라 MJPEG 스트림(multipart/x-mixed-replace). 연결이 끊길 때까지 이 요청 스레드가 계속 프레임을 보낸다.

    서버가 요청마다 스레드를 쓰므로(ThreadingHTTPServer) 스트림 하나가 다른 API 요청을 막지 않는다.
    """
    camera_id = route.removeprefix("/streams/").removesuffix(".mjpg")
    manager = h.app.camera_manager
    known_ids = {camera.id for camera in manager.snapshots()}
    # 카메라가 하나도 설정되지 않은 관리자(UnconfiguredArmCameraStreamManager)는 sequence_for가 없어 스트림을 줄 수 없다.
    if camera_id not in known_ids or not hasattr(manager, "sequence_for"):
        h._json(404, {"error": "Camera stream not found"})
        return

    # 첫 프레임을 최대 5초(0.1초 x 50번) 기다린다. 서버가 막 시작해 카메라가 아직 첫 프레임을 못 냈을 수 있다.
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
    # 프레임 번호(sequence)가 바뀌었을 때만 보낸다. 같은 프레임을 다시 보내지 않으려고 20 ms마다 번호만 확인한다.
    last_sequence = -1
    try:
        while True:
            sequence = manager.sequence_for(camera_id)
            if sequence == last_sequence:
                time.sleep(0.02)
                continue
            frame = manager.frame_for(camera_id)
            if frame is None:  # 번호는 바뀌었는데 프레임이 비어 있음: 연결을 끊지 않고 다음 프레임을 기다린다
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
        # 브라우저가 탭을 닫거나 <img>를 바꾸면 쓰기가 실패한다. 정상 종료로 보고 스레드를 끝낸다.
        return
