"""Order picks on the real arms: PICK_WITH_POLICY through the arm command queue.

The order flow asks for one unit at a time with `pick(arm_id, object_class, shelf_id, pinky_id, cancel_event,
progress)`, the same call the mock world answers. Here that queues PICK_WITH_POLICY for the planned arm, using the
policy recorded on that arm, and waits for that arm's worker. The ACT policy picks, places on the Pinky and goes back
to rest in one run, so the worker can only say the arm is home again: it ends the command UNKNOWN with
PLACE_NOT_VERIFIED. Whether the object really moved is the flow's VERIFY_TRANSFER step, judged from the cameras.
When that check passes, `settle` closes the command with the camera evidence, so the arm takes its next pick.

Turned on with CONTROL_STATION_ORDER_REAL_ARM=1. One worker per arm must be running
(scripts/omx_lerobot_worker.py --robot-id arm-01 / arm-02).
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

from control_station.arm_commands import IN_FLIGHT, ArmCommandConflict, ArmCommandError, ArmCommandStore
from control_station.order_flow import OrderFlowError

HOME_REACHED_UNVERIFIED = "ARM_HOME_PLACEMENT_NOT_VERIFIED"
# Waiting for the worker to take the command, ACK it and move home again, on top of the policy's own run time.
QUEUE_AND_HOME_S = 180.0
_WAITING = {"QUEUED", *(state.value for state in IN_FLIGHT)}


class ArmQueuePick:
    """The order flow's `pick` for real arms (see the module docstring)."""

    def __init__(self, commands: ArmCommandStore, *, poll_s: float = 0.5, valid_for_s: int = 60,
                 extra_wait_s: float = QUEUE_AND_HOME_S, clock: Callable[[], float] = time.monotonic) -> None:
        self.commands = commands
        self.poll_s = poll_s
        self.valid_for_s = valid_for_s
        self.extra_wait_s = extra_wait_s
        self.clock = clock

    def __call__(self, arm_id: str, object_class: str, shelf_id: str, pinky_id: str,
                 cancel_event: threading.Event, progress: Callable[[str], None]) -> dict[str, Any] | None:
        policies = self.commands.act_policies
        policy = policies.resolve(object_class, arm_id) if policies is not None else None
        if policy is None:
            raise OrderFlowError("POLICY_NOT_FOR_ARM",
                                 f"{arm_id}에서 녹화한 {object_class} 정책이 없어 팔을 움직이지 않았습니다", "arm")
        try:
            command = self.commands.create({
                "robot_id": arm_id, "operation": "PICK_WITH_POLICY", "valid_for_s": self.valid_for_s,
                "target": {"object_class": policy["object_class"]}, "place": {"slot_id": "CART_1"},
                "source": {"kind": "OPERATOR_INPUT", "note": f"주문 흐름 · {shelf_id} → 핑키 {pinky_id}"},
            })
        except (ArmCommandError, ArmCommandConflict) as exc:
            raise OrderFlowError(getattr(exc, "error_code", "ARM_COMMAND_INVALID"),
                                 f"{arm_id}: 팔 명령을 만들지 못했습니다 — {exc}", "arm") from exc
        command_id = command["command_id"]
        progress(f"{arm_id}: 팔 명령 {command_id} 대기열에 넣음 ({policy['policy_id']})")
        deadline = self.clock() + float(policy.get("max_run_s") or 60) + self.extra_wait_s
        last_state, cancel_sent = None, False
        while True:
            current = self.commands.get(command_id)
            state = current["state"]
            if state != last_state:
                last_state = state
                progress(f"{arm_id}: 팔 명령 {state}")
            if state not in _WAITING:
                break
            if cancel_event.is_set() and not cancel_sent:
                try:
                    self.commands.cancel(command_id)  # the worker stops the arm where it is
                except ArmCommandConflict:
                    pass
                cancel_sent = True
            if self.clock() > deadline:
                raise OrderFlowError("ARM_RESULT_TIMEOUT",
                                     f"팔 명령 {command_id}의 결과가 오지 않았습니다. 팔을 현장에서 확인하세요.", "arm")
            time.sleep(self.poll_s)
        if cancel_sent:
            return None
        result = current.get("result") or {}
        base = {"source": "arm_worker", "arm_id": arm_id, "arm_command_id": command_id,
                "policy_id": policy["policy_id"], "simulated": bool(result.get("simulated"))}
        if state == "SUCCEEDED":
            return {**base, "outcome": "ARM_REPORTED_SUCCESS"}
        if state == "UNKNOWN" and current.get("reason_code") == "PLACE_NOT_VERIFIED" and result.get("pose_reached") is True:
            return {**base, "outcome": HOME_REACHED_UNVERIFIED}
        raise OrderFlowError(current.get("reason_code") or state,
                             f"팔 명령 {command_id}이(가) {state}({current.get('reason_code')})로 끝났습니다. "
                             f"{result.get('message') or ''}".strip(), "arm")

    def settle(self, pick: dict[str, Any], evidence: dict[str, Any]) -> None:
        """Close a home-but-unverified command once the cameras saw the object move. Simulated counts never do:
        the command then stays UNKNOWN for a person to check, and the arm takes no new command before that."""
        if pick.get("outcome") != HOME_REACHED_UNVERIFIED or evidence.get("simulated"):
            return
        try:
            self.commands.clear_unknown(pick["arm_command_id"],
                                        f"주문 {evidence.get('task_id')}: 카메라로 옮김 확인({evidence.get('source')})")
        except (ArmCommandConflict, ArmCommandError) as exc:
            raise OrderFlowError("ARM_COMMAND_NOT_CLOSED",
                                 f"옮김은 확인했지만 팔 명령 {pick['arm_command_id']}을(를) 닫지 못했습니다: {exc}", "arm") from exc
