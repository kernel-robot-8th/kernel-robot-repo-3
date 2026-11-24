"""Order picks on the real arms: PICK_WITH_POLICY for the planned arm, sent over HTTP or as a ROS 2 action.

The order flow asks for one unit at a time with `pick(arm_id, object_class, shelf_id, pinky_id, cancel_event,
progress)`, the same call the mock world answers. Both transports below first make the same arm command record
(the policy recorded on that arm), so the screen's command history, cancel and the UNKNOWN rule work the same:

- ArmQueuePick (CONTROL_STATION_ORDER_ARM_TRANSPORT=http, the default): the arm's HTTP worker
  (scripts/omx_lerobot_worker.py --robot-id arm-0N) takes the command from the queue and reports to it.
- ArmActionPick (=ros2): the control station takes the command itself and sends it as a goal to the arm's action
  server (scripts/omx_pick_action_server.py, /arm_0N/pick_object). Its feedback and result are written to the same
  record as worker events: STARTING_POLICY = ACCEPTED, RUNNING_POLICY = PROGRESS, the result = the final event.

The ACT policy picks, places on the Pinky and goes back to rest in one run, so either way the arm can only say it is
home again: the command ends UNKNOWN with PLACE_NOT_VERIFIED. Whether the object really moved is the flow's
VERIFY_TRANSFER step, judged from the cameras. When that check passes, `settle` closes the command with the camera
evidence, so the arm takes its next pick.

Turned on with CONTROL_STATION_ORDER_REAL_ARM=1.
"""

from __future__ import annotations

import itertools
import queue
import threading
import time
from collections.abc import Callable
from datetime import datetime, timedelta, timezone
from typing import Any, Protocol

from control_station.arm_commands import IN_FLIGHT, ArmCommandConflict, ArmCommandError, ArmCommandStore
from control_station.order_flow import OrderFlowError

HOME_REACHED_UNVERIFIED = "ARM_HOME_PLACEMENT_NOT_VERIFIED"
# Waiting for the worker to take the command, ACK it and move home again, on top of the policy's own run time.
QUEUE_AND_HOME_S = 180.0
_WAITING = {"QUEUED", *(state.value for state in IN_FLIGHT)}
# Action feedback phases from which the arm may move: the record must be ACCEPTED by then.
_MOVING_PHASES = {"STARTING_POLICY", "RUNNING_POLICY", "RETURNING_HOME"}
_PHASE_WORDS = {"CHECKING": "확인 중", "STARTING_POLICY": "정책 시작", "RUNNING_POLICY": "정책 실행 중",
                "RETURNING_HOME": "home으로 복귀 중"}


class _ArmPick:
    """What both transports share: the policy for the arm, the command record, its final state, `settle`."""

    source = "arm_worker"

    def __init__(self, commands: ArmCommandStore, *, poll_s: float = 0.5, valid_for_s: int = 60,
                 extra_wait_s: float = QUEUE_AND_HOME_S, clock: Callable[[], float] = time.monotonic) -> None:
        self.commands = commands
        self.poll_s = poll_s
        self.valid_for_s = valid_for_s
        self.extra_wait_s = extra_wait_s
        self.clock = clock

    def _policy(self, arm_id: str, object_class: str) -> dict[str, Any]:
        policies = self.commands.act_policies
        policy = policies.resolve(object_class, arm_id) if policies is not None else None
        if policy is None:
            raise OrderFlowError("POLICY_NOT_FOR_ARM",
                                 f"{arm_id}에서 녹화한 {object_class} 정책이 없어 팔을 움직이지 않았습니다", "arm")
        return policy

    def _create(self, arm_id: str, policy: dict[str, Any], shelf_id: str, pinky_id: str) -> dict[str, Any]:
        try:
            return self.commands.create({
                "robot_id": arm_id, "operation": "PICK_WITH_POLICY", "valid_for_s": self.valid_for_s,
                "target": {"object_class": policy["object_class"]}, "place": {"slot_id": "CART_1"},
                "source": {"kind": "OPERATOR_INPUT", "note": f"주문 흐름 · {shelf_id} → 핑키 {pinky_id}"},
            })
        except (ArmCommandError, ArmCommandConflict) as exc:
            raise OrderFlowError(getattr(exc, "error_code", "ARM_COMMAND_INVALID"),
                                 f"{arm_id}: 팔 명령을 만들지 못했습니다 — {exc}", "arm") from exc

    def _deadline(self, policy: dict[str, Any]) -> float:
        return self.clock() + float(policy.get("max_run_s") or 60) + self.extra_wait_s

    def _wait_store(self, arm_id: str, policy: dict[str, Any], command_id: str, cancel_event: threading.Event,
                    progress: Callable[[str], None]) -> dict[str, Any] | None:
        """Follow the record until a worker finishes it; a cancel of the order cancels the command."""
        deadline = self._deadline(policy)
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
        return None if cancel_sent else self._conclude(arm_id, policy, command_id, current)

    def _conclude(self, arm_id: str, policy: dict[str, Any], command_id: str, current: dict[str, Any]) -> dict[str, Any]:
        """The record's final state as the pick result, or the order stops with the arm's reason."""
        state = current["state"]
        result = current.get("result") or {}
        base = {"source": self.source, "arm_id": arm_id, "arm_command_id": command_id,
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


class ArmQueuePick(_ArmPick):
    """The order flow's `pick` through the HTTP command queue (see the module docstring)."""

    def __call__(self, arm_id: str, object_class: str, shelf_id: str, pinky_id: str,
                 cancel_event: threading.Event, progress: Callable[[str], None]) -> dict[str, Any] | None:
        policy = self._policy(arm_id, object_class)
        command_id = self._create(arm_id, policy, shelf_id, pinky_id)["command_id"]
        progress(f"{arm_id}: 팔 명령 {command_id} 대기열에 넣음 ({policy['policy_id']})")
        return self._wait_store(arm_id, policy, command_id, cancel_event, progress)


# ---- ROS 2 action transport ---------------------------------------------------------------------------------

class PickGoal(Protocol):
    accepted: bool

    def cancel(self) -> None: ...

    def result(self, timeout_s: float) -> dict[str, Any] | None: ...


class PickTransport(Protocol):
    def send_goal(self, arm_id: str, target: dict[str, Any],
                  on_feedback: Callable[[dict[str, Any]], None]) -> PickGoal: ...


class _Record:
    """Writes the action's progress into the arm command record as this transport's worker events."""

    def __init__(self, commands: ArmCommandStore, command_id: str, arm_id: str, worker_id: str) -> None:
        self.commands, self.command_id, self.arm_id, self.worker_id = commands, command_id, arm_id, worker_id
        self.numbers = itertools.count(1)
        self.simulated = False
        self.refusal: str | None = None

    def event(self, event: str, **fields: Any) -> bool:
        """True when the record took the event. A refusal (canceled or expired meanwhile, already finished) is kept
        in `refusal` and the record stays as the store left it."""
        body = {"event_id": f"{self.command_id}.{next(self.numbers)}", "robot_id": self.arm_id,
                "worker_id": self.worker_id, "event": event, "simulated": self.simulated,
                **{key: value for key, value in fields.items() if value not in (None, "")}}
        try:
            self.commands.apply_worker_event(self.command_id, body)
        except (ArmCommandConflict, ArmCommandError) as exc:
            self.refusal = f"{event}: {exc}"
            return False
        return True


class ArmActionPick(_ArmPick):
    """The order flow's `pick` as a ROS 2 action to the arm's server (see the module docstring)."""

    source = "arm_action"

    def __init__(self, commands: ArmCommandStore, transport: PickTransport, **options: Any) -> None:
        super().__init__(commands, **options)
        self.transport = transport

    def _start_deadline(self, command: dict[str, Any]) -> str:
        """The goal's valid_until: the command's, but never past the record's ACK window (CONTROL_STATION_ARM_ACK_
        TIMEOUT_S, less a second for the feedback's way back). The server does not start the policy after it, so the
        arm does not move on a record the store has already expired."""
        ack_end = datetime.now(timezone.utc) + timedelta(seconds=max(1.0, self.commands.ack_timeout_s - 1.0))
        return min(datetime.fromisoformat(command["valid_until"]), ack_end).isoformat()

    def __call__(self, arm_id: str, object_class: str, shelf_id: str, pinky_id: str,
                 cancel_event: threading.Event, progress: Callable[[str], None]) -> dict[str, Any] | None:
        policy = self._policy(arm_id, object_class)
        command_id = self._create(arm_id, policy, shelf_id, pinky_id)["command_id"]
        worker_id = f"ros2-action.{arm_id}"
        claimed = self.commands.claim_next(arm_id, worker_id, 0)
        if claimed is None or claimed["command_id"] != command_id:
            # An HTTP worker of this arm took it first: it runs the command, so only its record is followed.
            progress(f"{arm_id}: 팔 명령 {command_id}을(를) HTTP 워커가 가져감 — 그 결과를 기다림")
            return self._wait_store(arm_id, policy, command_id, cancel_event, progress)
        record = _Record(self.commands, command_id, arm_id, worker_id)
        target = {"command_id": command_id, "arm_id": arm_id, "object_class": policy["object_class"],
                  "policy_id": policy["policy_id"], "policy_model": policy["model"],
                  "policy_registry_version": self.commands.act_policies.version,
                  "valid_until": self._start_deadline(claimed)}
        feedback: queue.Queue[dict[str, Any]] = queue.Queue()
        progress(f"{arm_id}: 팔 명령 {command_id} → ROS 2 액션 ({policy['policy_id']})")
        try:
            goal = self.transport.send_goal(arm_id, target, feedback.put)
        except Exception as exc:  # no server, no answer: nothing was sent to the arm
            record.event("REJECTED", reason_code="ACTION_SERVER_UNAVAILABLE", message=str(exc)[:300])
            raise OrderFlowError("ACTION_SERVER_UNAVAILABLE", f"{arm_id}: {exc}", "arm") from exc
        if not goal.accepted:
            record.event("REJECTED", reason_code="ACTION_GOAL_REJECTED", message="액션 서버가 목표를 받지 않았습니다(다른 목표 실행 중)")
            raise OrderFlowError("ACTION_GOAL_REJECTED", f"{arm_id}: 집기 액션 서버가 목표를 받지 않았습니다", "arm")

        state = {"accepted": False, "running": False, "phase": None}

        def take_feedback() -> None:
            while True:
                try:
                    item = feedback.get_nowait()
                except queue.Empty:
                    return
                record.simulated = item.get("simulated", record.simulated)
                if item["phase"] != state["phase"]:
                    state["phase"] = item["phase"]
                    progress(f"{arm_id}: {_PHASE_WORDS.get(item['phase'], item['phase'])}"
                             f"{' (모의)' if record.simulated else ''}")
                if item["phase"] in _MOVING_PHASES and not state["accepted"]:
                    state["accepted"] = True
                    if not record.event("ACCEPTED"):
                        goal.cancel()  # the record was canceled or expired before the arm started: stop it
                if item["phase"] == "RUNNING_POLICY" and not state["running"]:
                    state["running"] = True
                    record.event("PROGRESS", phase="APPROACH_TARGET")

        deadline = self._deadline(policy)
        cancel_sent = order_canceled = False
        while True:
            take_feedback()
            result = goal.result(self.poll_s)
            if result is not None:
                break
            if not cancel_sent and (cancel_event.is_set() or self.commands.get(command_id)["cancel_requested"]):
                if cancel_event.is_set():
                    order_canceled = True
                    try:
                        self.commands.cancel(command_id)  # the record must ask for the cancel the arm will report
                    except ArmCommandConflict:
                        pass
                goal.cancel()  # the server stops the policy where the arm is
                cancel_sent = True
            if self.clock() > deadline:
                goal.cancel()
                record.event("RESULT_UNKNOWN", reason_code="ACTION_RESULT_TIMEOUT",
                             message="집기 액션의 결과가 오지 않아 취소를 보냈습니다. 팔을 현장에서 확인하세요.")
                raise OrderFlowError("ARM_RESULT_TIMEOUT",
                                     f"팔 명령 {command_id}의 결과가 오지 않았습니다. 팔을 현장에서 확인하세요.", "arm")
        take_feedback()  # the result can overtake the last feedback; ACCEPTED must be written before the end
        record.simulated = result["simulated"]
        self._record_result(record, result, state["accepted"])
        if order_canceled and result["outcome"] == "CANCELED":
            return None  # the order itself was canceled; a cancel from the screen stops the order below
        return self._conclude(arm_id, policy, command_id, self.commands.get(command_id))

    @staticmethod
    def _record_result(record: _Record, result: dict[str, Any], accepted: bool) -> None:
        outcome, reason, message = result["outcome"], result["reason_code"] or None, result["message"] or None
        if outcome == "REJECTED" and not accepted:
            record.event("REJECTED", reason_code=reason or "ACTION_REJECTED", message=message)
            return
        if not accepted and not record.event("ACCEPTED"):
            return  # the store has already closed the record (canceled or expired); it keeps that state
        if outcome == "HOME_REACHED_PLACE_NOT_VERIFIED" or (outcome == "SUCCEEDED" and result["pose_reached"]):
            record.event("RESULT_UNKNOWN", reason_code="PLACE_NOT_VERIFIED", pose_reached=True, message=message)
        elif outcome == "CANCELED":
            if not record.event("CANCELED", phase="APPROACH_TARGET", message=message):
                record.event("FAILED", reason_code=reason or "CANCELED_BY_ARM", message=message)
        elif outcome == "FAILED" or outcome == "REJECTED":
            record.event("FAILED", reason_code=reason or "ACTION_FAILED", message=message)
        else:  # RESULT_UNKNOWN or anything new: unknown is never turned into success or failure
            record.event("RESULT_UNKNOWN", reason_code=reason or "ACTION_RESULT_UNKNOWN", message=message)
