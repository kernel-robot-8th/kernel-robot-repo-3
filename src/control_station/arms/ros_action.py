"""팔 집기 액션 서버(/arm_01/pick_object, /arm_02/pick_object; PickObject.action)의 ROS 2 클라이언트.

CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2일 때 arms/pick.py의 ArmActionPick이 쓴다(예전 모듈 이름 arm_pick).
pinky/parking_ros.py처럼 팔 쪽 ROS domain에 자기만의 ROS context를 만들고, 실행기(executor) 하나를 데몬 스레드에서
돌린다. 그래서 HTTP 서버 스레드가 ROS 때문에 막히는 일이 없다. 이 모듈은 목표·피드백·결과를 나르기만 하고, 그것이
주문과 팔 명령 기록에 어떤 뜻인지는 arms/pick.py가 정한다.

rclpy와 omx_interfaces는 RosPickTransport를 만들 때만 가져온다. ROS 2가 없는 PC에서도 관제의 나머지 기능은 돈다.
"""

from __future__ import annotations

import threading
from datetime import datetime
from typing import Any, Callable

# 목표를 보낸 뒤 서버가 받았는지(accepted) 답할 때까지 기다리는 최대 시간(초).
GOAL_RESPONSE_TIMEOUT_S = 5.0


def action_name(arm_id: str) -> str:
    """arm-01 -> /arm_01/pick_object (scripts/omx_pick_action_server.py가 같은 이름으로 서비스한다)."""
    return "/" + arm_id.replace("-", "_") + "/pick_object"


class PickActionUnavailable(RuntimeError):
    """팔의 액션 서버가 답하지 않음: 팔에는 아무것도 보내지 않았다."""


class _RosGoal:
    """보낸 목표 하나: 받아들여졌는지, 취소, 도착한 결과.

    결과 콜백은 ROS 실행기 스레드에서 불리고, result()는 주문 흐름 스레드에서 부른다. 둘은 threading.Event로 맞춘다."""

    def __init__(self, handle: Any) -> None:
        self.accepted = bool(handle.accepted)
        self._handle = handle
        self._done = threading.Event()
        self._result: dict[str, Any] | None = None
        if self.accepted:
            handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future: Any) -> None:
        """결과 메시지를 dict로 바꿔 두고 기다리는 쪽을 깨운다 (ROS 실행기 스레드)."""
        response = future.result()
        result = response.result
        self._result = {
            "outcome": result.outcome, "reason_code": result.reason_code, "message": result.message,
            "pose_reached": bool(result.pose_reached), "simulated": bool(result.simulated),
            "run_s": float(result.run_s), "goal_status": int(response.status),
        }
        self._done.set()

    def cancel(self) -> None:
        """취소를 요청한다(비동기). 실제로 멈췄는지는 결과의 outcome으로 알 수 있다."""
        self._handle.cancel_goal_async()

    def result(self, timeout_s: float) -> dict[str, Any] | None:
        """최대 timeout_s초 기다려 결과를 돌려준다. 아직 없으면 None."""
        return self._result if self._done.wait(timeout_s) else None


class RosPickTransport:
    """팔 집기 액션 서버들에 send_goal(arm_id, target, on_feedback)을 제공한다. 팔마다 ActionClient 하나."""

    def __init__(self, domain_id: int, arm_ids: list[str]) -> None:
        try:
            from omx_interfaces.action import PickObject
            from rclpy.action import ActionClient
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError(
                "CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2에는 ROS 2 Jazzy와 cpp/install/setup.bash를 source해야 합니다"
                " (omx_interfaces)."
            ) from exc
        self._goal_type = PickObject.Goal
        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("control_station_arm_pick", context=self._context)
        self._clients = {arm_id: ActionClient(self._node, PickObject, action_name(arm_id)) for arm_id in arm_ids}
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, name="arm-pick-ros", daemon=True)
        self._thread.start()

    def send_goal(self, arm_id: str, target: dict[str, Any], on_feedback: Callable[[dict[str, Any]], None],
                  server_wait_s: float = 2.0) -> _RosGoal:
        """목표 하나를 보내고 서버의 수락 여부를 받을 때까지 기다린다.

        서버를 server_wait_s초 안에 찾지 못하거나 GOAL_RESPONSE_TIMEOUT_S초 안에 답이 없으면 PickActionUnavailable이다.
        on_feedback은 ROS 실행기 스레드에서 불린다."""
        client = self._clients.get(arm_id)
        if client is None:
            raise PickActionUnavailable(f"{arm_id}의 집기 액션 클라이언트가 없습니다")
        if not client.wait_for_server(timeout_sec=server_wait_s):
            raise PickActionUnavailable(f"집기 액션 서버에 연결할 수 없습니다: {action_name(arm_id)}")
        goal = self._goal_type()
        for field in ("command_id", "arm_id", "object_class", "policy_id", "policy_model", "policy_registry_version"):
            setattr(goal, field, str(target[field]))
        # valid_until(ISO 문자열)을 ROS Time(sec, nanosec)으로 바꾼다.
        nanoseconds = int(datetime.fromisoformat(target["valid_until"]).timestamp() * 1_000_000_000)
        goal.valid_until.sec, goal.valid_until.nanosec = divmod(nanoseconds, 1_000_000_000)

        def feedback(message: Any) -> None:
            item = message.feedback
            on_feedback({"phase": item.phase, "elapsed_s": float(item.elapsed_s), "message": item.message,
                         "simulated": bool(item.simulated)})

        answered = threading.Event()
        future = client.send_goal_async(goal, feedback_callback=feedback)
        future.add_done_callback(lambda _: answered.set())
        if not answered.wait(GOAL_RESPONSE_TIMEOUT_S):
            raise PickActionUnavailable(f"집기 액션 서버가 목표에 답하지 않았습니다: {action_name(arm_id)}")
        return _RosGoal(future.result())

    def close(self) -> None:
        """실행기·노드·context를 정리하고 스레드를 최대 2초 기다린다."""
        self._executor.shutdown()
        self._node.destroy_node()
        self._context.try_shutdown()
        self._thread.join(timeout=2)
