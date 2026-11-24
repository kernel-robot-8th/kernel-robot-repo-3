"""Read-only robot communication probes used by the dashboard server."""

from __future__ import annotations

import os
import threading
from typing import Protocol


class RobotStatusProvider(Protocol):
    def snapshots(self) -> list[dict[str, object]]: ...

    def close(self) -> None: ...


# The arm entry is a placeholder: the arms are driven by the OMX HTTP worker (`/api/arm/...`), whose state the
# manual-control view shows; this probe does not watch the arm.
_ARM_SNAPSHOT = {
    "id": "arm-01",
    "name": "팔관절로봇",
    "kind": "ARM",
    "communication": "미설정",
    "state": "unconfigured",
    "detail": "이 상태 관측은 팔을 보지 않음 · 팔 상태는 OMX 워커 표시를 확인",
}


class MockRobotStatusProvider:
    """Keep device status explicitly unconfigured in local development mode."""

    def snapshots(self) -> list[dict[str, object]]:
        return [
            {
                "id": "mobile-01",
                "name": "주행로봇",
                "kind": "MOBILE",
                "communication": "미연결",
                "state": "mock",
                "detail": "모의 모드 · Nav2에 접속하지 않음",
            },
            dict(_ARM_SNAPSHOT),
        ]

    def close(self) -> None:
        pass


class Ros2RobotStatusProvider:
    """Observe Nav2 action-server discovery without sending commands."""

    def __init__(self, domain_id: int) -> None:
        try:
            from rclpy.action import ActionClient
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 Jazzy 환경을 source한 뒤 다시 실행하세요."
            ) from exc
        try:
            from nav2_msgs.action import NavigateToPose
        except ImportError as exc:
            raise RuntimeError(
                "Nav2 상태 관측을 사용하려면 nav2_msgs 환경을 source하세요."
            ) from exc

        self._domain_id = domain_id
        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("control_station_status", context=self._context)
        self._client = ActionClient(self._node, NavigateToPose, "navigate_to_pose")
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, name="ros-status-probe", daemon=True
        )
        self._thread.start()

    def snapshots(self) -> list[dict[str, object]]:
        available = self._client.server_is_ready()
        mobile_snapshot = {
            "id": "mobile-01",
            "name": "주행로봇",
            "kind": "MOBILE",
            "communication": "액션 서버 발견" if available else "발견 대기",
            "state": "available" if available else "waiting",
            "detail": (
                f"ROS 2 domain {self._domain_id} · Nav2 NavigateToPose"
                if available
                else f"ROS 2 domain {self._domain_id} · Nav2 액션 서버를 기다리는 중"
            ),
        }
        return [mobile_snapshot, dict(_ARM_SNAPSHOT)]

    def close(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
        self._context.try_shutdown()


def make_status_provider(mode: str) -> RobotStatusProvider:
    if mode == "mock":
        return MockRobotStatusProvider()
    if mode == "ros2":
        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN")
        if raw_domain is None:
            raw_domain = os.environ.get("ROS_DOMAIN_ID")
        if raw_domain is None:
            raw_domain = "0"
        try:
            domain_id = int(raw_domain)
        except ValueError as exc:
            raise RuntimeError("ROS domain ID 설정은 정수여야 합니다.") from exc
        if not 0 <= domain_id <= 232:
            raise RuntimeError("ROS 2 domain ID는 0부터 232 사이여야 합니다.")
        return Ros2RobotStatusProvider(domain_id)
    raise RuntimeError("CONTROL_STATION_MODE는 mock 또는 ros2여야 합니다.")
