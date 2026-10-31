"""Read-only robot communication probes used by the dashboard server."""

from __future__ import annotations

import math
import os
import re
import threading
import time
from datetime import datetime, timezone
from typing import Any, Protocol


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
                "trajectory_action_server": {
                    "name": None,
                    "state": "unconfigured",
                    "discovered": None,
                },
            },
        ]

    def close(self) -> None:
        pass


class Ros2RobotStatusProvider:
    """Observe optional Nav2, arm joint-state, and trajectory discovery without commands."""

    def __init__(
        self,
        domain_id: int,
        arm_joint_states_topic: str | None,
        arm_joint_states_max_age: float,
        arm_trajectory_action_name: str | None = None,
        *,
        mobile_discovery_enabled: bool = True,
    ) -> None:
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 Jazzy 환경을 source한 뒤 다시 실행하세요."
            ) from exc
        if mobile_discovery_enabled:
            try:
                from nav2_msgs.action import NavigateToPose
            except ImportError as exc:
                raise RuntimeError(
                    "Nav2 상태 관측을 사용하려면 nav2_msgs 환경을 source하세요."
                ) from exc
        else:
            NavigateToPose = None
        if arm_joint_states_topic is not None:
            try:
                from sensor_msgs.msg import JointState
            except ImportError as exc:
                raise RuntimeError(
                    "팔 관절 관측을 사용하려면 sensor_msgs 환경을 source해야 합니다."
                ) from exc
        else:
            JointState = None
        if arm_trajectory_action_name is not None:
            try:
                from control_msgs.action import FollowJointTrajectory
            except ImportError as exc:
                raise RuntimeError(
                    "trajectory action discovery를 사용하려면 control_msgs 환경을 source해야 합니다."
                ) from exc
        else:
            FollowJointTrajectory = None

        self._domain_id = domain_id
        self._arm_joint_states_topic = arm_joint_states_topic
        self._arm_joint_states_max_age = arm_joint_states_max_age
        self._arm_trajectory_action_name = arm_trajectory_action_name
        self._arm_joint_state_lock = threading.Lock()
        self._arm_joint_state: tuple[
            float,
            str,
            tuple[str, ...],
            tuple[float, ...],
            int | None,
            int | None,
            str,
            str | None,
        ] | None = None
        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("control_station_status", context=self._context)
        self._client = (
            ActionClient(self._node, NavigateToPose, "navigate_to_pose")
            if mobile_discovery_enabled
            else None
        )
        self._arm_trajectory_client = (
            ActionClient(
                self._node,
                FollowJointTrajectory,
                arm_trajectory_action_name,
            )
            if arm_trajectory_action_name is not None
            else None
        )
        self._joint_state_subscription = (
            self._node.create_subscription(
                JointState,
                arm_joint_states_topic,
                self._on_arm_joint_state,
                10,
            )
            if arm_joint_states_topic
            else None
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, name="ros-status-probe", daemon=True
        )
        self._thread.start()

    def _on_arm_joint_state(self, message: Any) -> None:
        """Keep a read-only snapshot of the last ROS joint-state sample."""
        names = tuple(str(name) for name in message.name)
        positions = tuple(float(position) for position in message.position)
        velocity_count = len(getattr(message, "velocity", ()))
        effort_count = len(getattr(message, "effort", ()))
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        source_sec = getattr(stamp, "sec", None)
        source_nanosec = getattr(stamp, "nanosec", None)
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
        sample_error = None
        if not names:
            sample_error = "joint name 목록이 비어 있음"
        elif len(names) != len(positions):
            sample_error = "joint name과 position 배열 길이가 다름"
        elif velocity_count not in (0, len(names)):
            sample_error = "velocity 배열 길이는 0 또는 joint name 개수와 같아야 함"
        elif effort_count not in (0, len(names)):
            sample_error = "effort 배열 길이는 0 또는 joint name 개수와 같아야 함"
        elif any(not name.strip() for name in names):
            sample_error = "빈 joint name이 포함됨"
        elif len(set(names)) != len(names):
            sample_error = "중복 joint name이 포함됨"
        elif not all(math.isfinite(position) for position in positions):
            sample_error = "유한하지 않은 position 값이 포함됨"
        with self._arm_joint_state_lock:
            received_at = datetime.now(timezone.utc).isoformat(timespec="microseconds")
            self._arm_joint_state = (
                time.monotonic(),
                received_at,
                names,
                positions,
                int(source_sec) if source_sec is not None else None,
                int(source_nanosec) if source_nanosec is not None else None,
                frame_id,
                sample_error,
            )

    def snapshots(self) -> list[dict[str, object]]:
        available = self._client.server_is_ready() if self._client is not None else False
        arm_name = (
            "MuJoCo OMX 시뮬레이션"
            if self._client is None
            else "팔관절로봇"
        )
        with self._arm_joint_state_lock:
            arm_sample = self._arm_joint_state
        if self._arm_joint_states_topic is None:
            arm_snapshot = {
                "id": "arm-01",
                "name": arm_name,
                "kind": "ARM",
                "communication": "관절 토픽 미설정",
                "state": "unconfigured",
                "detail": (
                    "장치별 관절 상태 토픽의 출처 확인 전 · 구독하지 않음 · "
                    "준비/안전/명령 가능 상태는 확인하지 않음"
                ),
            }
        elif arm_sample is None:
            arm_snapshot = {
                "id": "arm-01",
                "name": arm_name,
                "kind": "ARM",
                "communication": "관절 상태 수신 대기",
                "state": "waiting",
                "detail": (
                    f"ROS 2 domain {self._domain_id} · {self._arm_joint_states_topic} · "
                    "읽기 전용 관측 · 준비/안전/명령 가능 상태는 확인하지 않음"
                ),
            }
        else:
            (
                received_monotonic,
                received_at,
                names,
                positions,
                source_sec,
                source_nanosec,
                frame_id,
                sample_error,
            ) = arm_sample
            age = max(0.0, time.monotonic() - received_monotonic)
            is_fresh = age <= self._arm_joint_states_max_age
            state = "invalid" if sample_error else "observed" if is_fresh else "stale"
            joint_summary = ", ".join(
                f"{name}={position:.3f}" for name, position in zip(names, positions)
            ) or "관절 값 없음"
            source_stamp_summary = (
                f"ROS stamp {source_sec}.{source_nanosec:09d} (원본, clock 미검증)"
                if source_sec is not None and source_nanosec is not None
                else "ROS stamp 없음"
            )
            frame_summary = f"frame_id={frame_id}" if frame_id else "frame_id 없음"
            arm_snapshot = {
                "id": "arm-01",
                "name": arm_name,
                "kind": "ARM",
                "communication": (
                    "관절 데이터 오류"
                    if sample_error
                    else "관절 상태 관측"
                    if is_fresh
                    else "관절 상태 오래됨"
                ),
                "state": state,
                "detail": (
                    f"ROS 2 {self._arm_joint_states_topic} · {age:.1f}초 전 수신 · "
                    f"{joint_summary} · {source_stamp_summary} · {frame_summary} · "
                    "position 단위 미확정 · "
                    f"{sample_error + ' · ' if sample_error else ''}"
                    "publisher 출처는 검증되지 않음 · "
                    "읽기 전용 관측이며 준비/안전/명령 가능 상태는 확인하지 않음"
                ),
                "joint_names": list(names),
                "positions": list(positions) if sample_error is None else None,
                "position_unit_status": "UNSPECIFIED",
                "sample_valid": sample_error is None,
                "sample_error": sample_error,
                "received_at": received_at,
                "source_stamp": (
                    {"sec": source_sec, "nanosec": source_nanosec}
                    if source_sec is not None and source_nanosec is not None
                    else None
                ),
                "frame_id": frame_id,
                "age_seconds": round(age, 2),
            }
        if self._arm_trajectory_client is None:
            arm_snapshot["trajectory_action_server"] = {
                "name": None,
                "state": "unconfigured",
                "discovered": None,
            }
        else:
            trajectory_action_discovered = self._arm_trajectory_client.server_is_ready()
            arm_snapshot["trajectory_action_server"] = {
                "name": self._arm_trajectory_action_name,
                "state": "discovered" if trajectory_action_discovered else "not_discovered",
                "discovered": trajectory_action_discovered,
            }
            arm_snapshot["detail"] += (
                f" · FollowJointTrajectory {self._arm_trajectory_action_name} "
                f"{'발견' if trajectory_action_discovered else '미발견'} "
                "(goal 미전송; 준비/안전 상태를 증명하지 않음)"
            )
        if self._client is None:
            mobile_snapshot = {
                "id": "mobile-01",
                "name": "주행로봇",
                "kind": "MOBILE",
                "communication": "시뮬레이션 모드 · 미사용",
                "state": "unconfigured",
                "detail": "MuJoCo OMX 관측 모드 · Pinky/Nav2 action을 조회하지 않음",
            }
        else:
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
        return [mobile_snapshot, arm_snapshot]

    def close(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
        self._context.try_shutdown()


def make_status_provider(mode: str) -> RobotStatusProvider:
    if mode == "mock":
        return MockRobotStatusProvider()
    if mode in {"ros2", "mujoco"}:
        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN")
        if raw_domain is None:
            raw_domain = os.environ.get("ROS_DOMAIN_ID")
        if mode == "mujoco" and raw_domain is None:
            raise RuntimeError(
                "MuJoCo 관측 모드에는 실물 장치와 분리된 ROS domain을 명시해야 합니다."
            )
        if raw_domain is None:
            raw_domain = "0"
        try:
            domain_id = int(raw_domain)
        except ValueError as exc:
            raise RuntimeError("ROS domain ID 설정은 정수여야 합니다.") from exc
        minimum_domain_id = 1 if mode == "mujoco" else 0
        if not minimum_domain_id <= domain_id <= 232:
            if mode == "mujoco":
                raise RuntimeError("MuJoCo 관측 모드의 ROS domain ID는 1부터 232 사이여야 합니다.")
            raise RuntimeError("ROS 2 domain ID는 0부터 232 사이여야 합니다.")
        raw_topic = (
            "/mujoco_omx/joint_state_broadcaster/joint_states"
            if mode == "mujoco"
            else os.environ.get("CONTROL_STATION_ARM_JOINT_STATES_TOPIC")
        )
        topic = raw_topic.strip() if raw_topic is not None else None
        if topic == "":
            topic = None
        if topic is not None and (not topic.startswith("/") or topic.startswith("~")):
            raise RuntimeError("CONTROL_STATION_ARM_JOINT_STATES_TOPIC은 절대 토픽 이름이어야 합니다.")
        raw_action_name = (
            "/mujoco_omx/joint_trajectory_controller/follow_joint_trajectory"
            if mode == "mujoco"
            else os.environ.get("CONTROL_STATION_ARM_TRAJECTORY_ACTION_NAME")
        )
        action_name = raw_action_name.strip() if raw_action_name is not None else None
        if action_name == "":
            action_name = None
        if action_name is not None and (
            re.fullmatch(r"/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*", action_name) is None
        ):
            raise RuntimeError(
                "CONTROL_STATION_ARM_TRAJECTORY_ACTION_NAME은 유효한 절대 action 이름이어야 합니다."
            )
        raw_max_age = os.environ.get("CONTROL_STATION_ARM_JOINT_STATES_MAX_AGE", "2.0")
        try:
            max_age = float(raw_max_age)
        except ValueError as exc:
            raise RuntimeError("CONTROL_STATION_ARM_JOINT_STATES_MAX_AGE는 양수여야 합니다.") from exc
        if not math.isfinite(max_age) or max_age <= 0:
            raise RuntimeError("CONTROL_STATION_ARM_JOINT_STATES_MAX_AGE는 양수여야 합니다.")
        return Ros2RobotStatusProvider(
            domain_id,
            topic,
            max_age,
            action_name,
            mobile_discovery_enabled=mode != "mujoco",
        )
    raise RuntimeError("CONTROL_STATION_MODE는 mock, ros2 또는 mujoco여야 합니다.")
