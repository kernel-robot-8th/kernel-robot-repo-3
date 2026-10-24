"""Read-only robot communication probes used by the dashboard server."""

from __future__ import annotations

import os
import threading
from typing import Protocol


class RobotStatusProvider(Protocol):
    def snapshots(self) -> list[dict[str, object]]: ...

    def close(self) -> None: ...


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
            {
                "id": "arm-01",
                "name": "팔관절로봇",
                "kind": "ARM",
                "communication": "미설정",
                "state": "unconfigured",
                "detail": "제어 인터페이스 확인 전",
            },
        ]

    def close(self) -> None:
        pass


class Ros2MobileStatusProvider:
    """Probe Nav2 action-server discovery without sending a navigation goal."""

    def __init__(self, domain_id: int) -> None:
        try:
            import rclpy
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 Jazzy와 nav2_msgs 환경을 source한 뒤 다시 실행하세요."
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
        return [
            {
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
            },
            {
                "id": "arm-01",
                "name": "팔관절로봇",
                "kind": "ARM",
                "communication": "미설정",
                "state": "unconfigured",
                "detail": "제어 인터페이스 확인 전 · 접속하지 않음",
            },
        ]

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
            raw_domain = os.environ.get("ROS_DOMAIN_ID", "0")
        try:
            domain_id = int(raw_domain)
        except ValueError as exc:
            raise RuntimeError("CONTROL_STATION_MOBILE_DOMAIN은 정수여야 합니다.") from exc
        if domain_id < 0:
            raise RuntimeError("ROS 2 domain ID는 0 이상이어야 합니다.")
        return Ros2MobileStatusProvider(domain_id)
    raise RuntimeError("CONTROL_STATION_MODE는 mock 또는 ros2여야 합니다.")
