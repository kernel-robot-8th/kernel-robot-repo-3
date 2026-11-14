"""ROS 2 client of the arm pick action servers (/arm_01/pick_object, /arm_02/pick_object; PickObject.action).

Used by arm_pick.ArmActionPick when CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2. Like parking_ros.py, it has its own
ROS context on the arms' domain and spins one executor in a daemon thread, so the HTTP server never blocks on ROS.
This module only carries goals, feedback and results; what they mean for the order and the arm command record is
decided in arm_pick.py.
"""

from __future__ import annotations

import threading
from datetime import datetime
from typing import Any, Callable

GOAL_RESPONSE_TIMEOUT_S = 5.0


def action_name(arm_id: str) -> str:
    """arm-01 -> /arm_01/pick_object (scripts/omx_pick_action_server.py serves the same name)."""
    return "/" + arm_id.replace("-", "_") + "/pick_object"


class PickActionUnavailable(RuntimeError):
    """No answer from the arm's action server: nothing was sent to the arm."""


class _RosGoal:
    """One sent goal: accepted or not, a cancel, and the result once it arrives."""

    def __init__(self, handle: Any) -> None:
        self.accepted = bool(handle.accepted)
        self._handle = handle
        self._done = threading.Event()
        self._result: dict[str, Any] | None = None
        if self.accepted:
            handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future: Any) -> None:
        response = future.result()
        result = response.result
        self._result = {
            "outcome": result.outcome, "reason_code": result.reason_code, "message": result.message,
            "pose_reached": bool(result.pose_reached), "simulated": bool(result.simulated),
            "run_s": float(result.run_s), "goal_status": int(response.status),
        }
        self._done.set()

    def cancel(self) -> None:
        self._handle.cancel_goal_async()

    def result(self, timeout_s: float) -> dict[str, Any] | None:
        return self._result if self._done.wait(timeout_s) else None


class RosPickTransport:
    """send_goal(arm_id, target, on_feedback) for the arms' pick action servers."""

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
        client = self._clients.get(arm_id)
        if client is None:
            raise PickActionUnavailable(f"{arm_id}의 집기 액션 클라이언트가 없습니다")
        if not client.wait_for_server(timeout_sec=server_wait_s):
            raise PickActionUnavailable(f"집기 액션 서버에 연결할 수 없습니다: {action_name(arm_id)}")
        goal = self._goal_type()
        for field in ("command_id", "arm_id", "object_class", "policy_id", "policy_model", "policy_registry_version"):
            setattr(goal, field, str(target[field]))
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
        self._executor.shutdown()
        self._node.destroy_node()
        self._context.try_shutdown()
        self._thread.join(timeout=2)
