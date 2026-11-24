"""관제 서버(대시보드)가 쓰는 읽기 전용 로봇 통신 상태 관측.

현황 화면의 로봇 통신 카드(주행로봇, 팔관절로봇)를 만든다. app.py가 CONTROL_STATION_MODE에 따라
`make_status_provider()`로 하나를 고른다.
- mock: 아무 데도 접속하지 않고, 장치 상태를 '미설정/미연결'로 분명히 보여 준다(로컬 개발용).
- ros2: Nav2 NavigateToPose 액션 서버가 ROS 그래프에 보이는지만 관측한다. 명령은 하나도 보내지 않는다.
팔 항목은 자리만 채운다. 팔 상태는 OMX HTTP 워커가 갖고 있고, 수동 조작 화면이 그것을 보여 준다.
"""

from __future__ import annotations

import os
import threading
from typing import Protocol


class RobotStatusProvider(Protocol):
    """로봇 통신 상태를 주는 쪽의 인터페이스. snapshots()는 카드 목록, close()는 서버 종료 때 자원 정리."""

    def snapshots(self) -> list[dict[str, object]]: ...

    def close(self) -> None: ...


# 팔 항목은 자리만 채운다: 팔은 OMX HTTP 워커(`/api/arm/...`)가 움직이고, 그 상태는 수동 조작 화면이 보여 준다.
# 이 관측은 팔을 보지 않는다.
_ARM_SNAPSHOT = {
    "id": "arm-01",
    "name": "팔관절로봇",
    "kind": "ARM",
    "communication": "미설정",
    "state": "unconfigured",
    "detail": "이 상태 관측은 팔을 보지 않음 · 팔 상태는 OMX 워커 표시를 확인",
}


class MockRobotStatusProvider:
    """로컬 개발 모드에서 장치 상태를 분명히 '설정 안 됨'으로 둔다."""

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
    """명령을 보내지 않고 Nav2 액션 서버가 발견되는지만 관측한다.

    자체 rclpy Context를 만들어 같은 프로세스의 다른 rclpy 사용(주차 판정 노드, ROS 카메라 토픽)과 init/shutdown이
    섞이지 않게 한다. executor는 데몬 스레드("ros-status-probe")에서 돌아 서버 종료를 막지 않는다.
    """

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
        """주행로봇 카드(액션 서버가 보이면 available, 아니면 waiting)와 팔 자리 카드."""
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
        """executor를 멈추고(최대 2초) 스레드가 끝나기를 기다린 뒤 노드와 Context를 정리한다."""
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
        self._context.try_shutdown()


def make_status_provider(mode: str) -> RobotStatusProvider:
    """CONTROL_STATION_MODE 값(mock 또는 ros2)에 맞는 관측기를 만든다. 다른 값이면 RuntimeError."""
    if mode == "mock":
        return MockRobotStatusProvider()
    if mode == "ros2":
        # ROS 도메인: CONTROL_STATION_MOBILE_DOMAIN → ROS_DOMAIN_ID → 0 순서로 정한다.
        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN")
        if raw_domain is None:
            raw_domain = os.environ.get("ROS_DOMAIN_ID")
        if raw_domain is None:
            raw_domain = "0"
        try:
            domain_id = int(raw_domain)
        except ValueError as exc:
            raise RuntimeError("ROS domain ID 설정은 정수여야 합니다.") from exc
        # DDS 포트 계산 때문에 ROS 2가 받는 도메인 ID는 0~232다.
        if not 0 <= domain_id <= 232:
            raise RuntimeError("ROS 2 domain ID는 0부터 232 사이여야 합니다.")
        return Ros2RobotStatusProvider(domain_id)
    raise RuntimeError("CONTROL_STATION_MODE는 mock 또는 ros2여야 합니다.")
