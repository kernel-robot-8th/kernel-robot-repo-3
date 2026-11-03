"""HTTP-delivered arm command queue for an external arm worker.

The control station never drives the arm here. It freezes one target per command, an external
worker (the LeRobot OMX laptop, or the same machine) claims it by long-poll over HTTP, and reports
lifecycle events back. The worker may move the arm only after its ACCEPTED event is answered with
HTTP 200; any other answer means "do not execute". This handshake is why a command whose ACCEPTED
never arrived can be closed without guessing that the arm moved.

Implemented directly because the arm stack (LeRobot) is not a ROS 2 node, so a ROS 2 action is not
available on that side yet. The fields map one-to-one onto an action goal/feedback/result if the
worker later moves to ROS 2.
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
import uuid
from datetime import datetime, timedelta, timezone
from typing import Any, Mapping

from sqlalchemy import select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import Session, sessionmaker

from control_station.database import ArmCommandEventRecord, ArmCommandRecord
from control_station.models import StringEnum


ARM_ROBOT_IDS = frozenset({"arm-01"})
OPERATIONS = frozenset({"PICK_AND_PLACE"})
APPROACHES = frozenset({"FRONT_HORIZONTAL"})
SOURCE_KINDS = frozenset({"OPERATOR_INPUT"})
# Phase names follow the control station's pick-and-place step types so progress can map onto steps.
PHASES = frozenset(
    {
        "APPROACH_TARGET",
        "REFINE_TARGET",
        "GRASP_TARGET",
        "VERIFY_GRASP",
        "LIFT_TARGET",
        "MOVE_TO_DESTINATION",
        "PLACE_TARGET",
        "VERIFY_PLACEMENT",
    }
)
# Input sanity bound that catches millimetres typed as metres; it is not a workspace check.
MAX_ABS_POSITION_M = 1.0
MAX_WAIT_S = 30.0

_FRAME_ID = re.compile(r"^[A-Za-z0-9_./-]{1,80}$")
_SLOT_ID = re.compile(r"^[A-Za-z0-9_.-]{1,40}$")
_TOKEN = re.compile(r"^[A-Za-z0-9_.:-]{1,80}$")
_REASON_CODE = re.compile(r"^[A-Z0-9_]{1,60}$")


class ArmCommandState(StringEnum):
    QUEUED = "QUEUED"
    DISPATCHED = "DISPATCHED"
    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    REJECTED = "REJECTED"
    CANCELED = "CANCELED"
    EXPIRED = "EXPIRED"
    UNKNOWN = "UNKNOWN"


IN_FLIGHT = frozenset({ArmCommandState.DISPATCHED, ArmCommandState.ACCEPTED, ArmCommandState.RUNNING})

# Worker event -> {current state: next state}. Anything else is rejected without changing state.
WORKER_TRANSITIONS: dict[str, dict[ArmCommandState, ArmCommandState]] = {
    "ACCEPTED": {ArmCommandState.DISPATCHED: ArmCommandState.ACCEPTED},
    "REJECTED": {ArmCommandState.DISPATCHED: ArmCommandState.REJECTED},
    "PROGRESS": {
        ArmCommandState.ACCEPTED: ArmCommandState.RUNNING,
        ArmCommandState.RUNNING: ArmCommandState.RUNNING,
    },
    "SUCCEEDED": {
        ArmCommandState.ACCEPTED: ArmCommandState.SUCCEEDED,
        ArmCommandState.RUNNING: ArmCommandState.SUCCEEDED,
    },
    "FAILED": {
        ArmCommandState.ACCEPTED: ArmCommandState.FAILED,
        ArmCommandState.RUNNING: ArmCommandState.FAILED,
    },
    "RESULT_UNKNOWN": {
        ArmCommandState.ACCEPTED: ArmCommandState.UNKNOWN,
        ArmCommandState.RUNNING: ArmCommandState.UNKNOWN,
    },
    "CANCELED": {
        ArmCommandState.ACCEPTED: ArmCommandState.CANCELED,
        ArmCommandState.RUNNING: ArmCommandState.CANCELED,
    },
}
REASON_REQUIRED = frozenset({"REJECTED", "FAILED", "RESULT_UNKNOWN"})
WORKER_EVENT_FIELDS = frozenset(
    {
        "event_id",
        "robot_id",
        "worker_id",
        "event",
        "simulated",
        "occurred_at",
        "phase",
        "reason_code",
        "message",
        "grasp_confirmed",
        "place_confirmed",
        "details",
    }
)


class ArmCommandError(ValueError):
    """The request does not match the arm command contract (HTTP 400)."""


class ArmCommandNotFound(LookupError):
    pass


class ArmCommandConflict(RuntimeError):
    """The request is valid but not allowed in the current state (HTTP 409)."""

    def __init__(self, error_code: str, message: str, command: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.command = command


def _now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(moment: datetime) -> str:
    return moment.isoformat(timespec="microseconds")


def _text(value: object, field: str, maximum: int, *, required: bool = True) -> str | None:
    if value is None and not required:
        return None
    if (
        not isinstance(value, str)
        or not value.strip()
        or len(value.strip()) > maximum
        or any(ord(character) < 32 or ord(character) == 127 for character in value)
    ):
        raise ArmCommandError(f"{field} must contain 1 to {maximum} characters without control characters")
    return value.strip()


def _matching(value: object, field: str, pattern: re.Pattern[str]) -> str:
    if not isinstance(value, str) or not pattern.fullmatch(value):
        raise ArmCommandError(f"{field} has an invalid format")
    return value


def _xyz(value: object, field: str, *, minimum: float, maximum: float) -> dict[str, float]:
    if not isinstance(value, Mapping) or set(value) != {"x", "y", "z"}:
        raise ArmCommandError(f"{field} must be an object with exactly x, y, and z")
    result: dict[str, float] = {}
    for axis in ("x", "y", "z"):
        number = value[axis]
        if isinstance(number, bool) or not isinstance(number, (int, float)):
            raise ArmCommandError(f"{field}.{axis} must be a number")
        number = float(number)
        if not math.isfinite(number) or not minimum <= number <= maximum:
            raise ArmCommandError(f"{field}.{axis} must be a finite number from {minimum} to {maximum}")
        result[axis] = number
    return result


def validate_command_request(payload: Mapping[str, object]) -> tuple[str, str, int, dict[str, Any]]:
    """Return (robot_id, operation, valid_for_s, frozen request) or raise ArmCommandError."""
    allowed = {"robot_id", "operation", "valid_for_s", "target", "place", "source"}
    if set(payload) - allowed:
        raise ArmCommandError("Unsupported fields: " + ", ".join(sorted(set(payload) - allowed)))
    robot_id = payload.get("robot_id")
    if robot_id not in ARM_ROBOT_IDS:
        raise ArmCommandError("robot_id must be one of: " + ", ".join(sorted(ARM_ROBOT_IDS)))
    operation = payload.get("operation")
    if operation not in OPERATIONS:
        raise ArmCommandError("operation must be one of: " + ", ".join(sorted(OPERATIONS)))
    valid_for_s = payload.get("valid_for_s")
    if isinstance(valid_for_s, bool) or not isinstance(valid_for_s, int) or not 5 <= valid_for_s <= 600:
        raise ArmCommandError("valid_for_s must be an integer from 5 to 600")

    target = payload.get("target")
    target_fields = {"object_class", "frame_id", "unit", "position", "uncertainty_m", "approach"}
    if not isinstance(target, Mapping) or set(target) - target_fields:
        raise ArmCommandError("target must be an object with " + ", ".join(sorted(target_fields)))
    if target.get("unit") != "m":
        raise ArmCommandError("target.unit must be 'm'")
    frozen_target: dict[str, Any] = {
        "object_class": _text(target.get("object_class"), "target.object_class", 80),
        "frame_id": _matching(target.get("frame_id"), "target.frame_id", _FRAME_ID),
        "unit": "m",
        "position": _xyz(
            target.get("position"), "target.position", minimum=-MAX_ABS_POSITION_M, maximum=MAX_ABS_POSITION_M
        ),
        "uncertainty_m": (
            _xyz(target["uncertainty_m"], "target.uncertainty_m", minimum=0.0, maximum=MAX_ABS_POSITION_M)
            if target.get("uncertainty_m") is not None
            else None
        ),
        "approach": target.get("approach", "FRONT_HORIZONTAL"),
    }
    if frozen_target["approach"] not in APPROACHES:
        raise ArmCommandError("target.approach must be one of: " + ", ".join(sorted(APPROACHES)))

    place = payload.get("place")
    if not isinstance(place, Mapping) or set(place) != {"slot_id"}:
        raise ArmCommandError("place must be an object with exactly slot_id")
    source = payload.get("source")
    if not isinstance(source, Mapping) or set(source) - {"kind", "note"}:
        raise ArmCommandError("source must be an object with kind and optional note")
    if source.get("kind") not in SOURCE_KINDS:
        raise ArmCommandError("source.kind must be one of: " + ", ".join(sorted(SOURCE_KINDS)))
    request = {
        "target": frozen_target,
        "place": {"slot_id": _matching(place.get("slot_id"), "place.slot_id", _SLOT_ID)},
        "source": {
            "kind": source["kind"],
            "note": _text(source.get("note"), "source.note", 200, required=False),
        },
    }
    return str(robot_id), str(operation), valid_for_s, request


class ArmCommandStore:
    """Persist arm commands and serialize their state changes inside this server process."""

    def __init__(self, sessions: sessionmaker, *, ack_timeout_s: float | None = None) -> None:
        self._sessions = sessions
        self.ack_timeout_s = (
            ack_timeout_s
            if ack_timeout_s is not None
            else float(os.environ.get("CONTROL_STATION_ARM_ACK_TIMEOUT_S", "10"))
        )
        if not math.isfinite(self.ack_timeout_s) or self.ack_timeout_s <= 0:
            raise ValueError("CONTROL_STATION_ARM_ACK_TIMEOUT_S must be a positive number")
        self._lock = threading.RLock()
        self._new_command = threading.Condition(self._lock)
        self._workers: dict[str, dict[str, Any]] = {}

    # ---- presentation ----

    @staticmethod
    def _present(command: ArmCommandRecord, *, with_events: bool = False) -> dict[str, Any]:
        state = ArmCommandState(command.state)
        presented: dict[str, Any] = {
            "schema_version": 1,
            "command_id": command.id,
            "robot_id": command.robot_id,
            "operation": command.operation,
            "state": state.value,
            "blocks_new_commands": ArmCommandStore._blocks(command),
            "created_at": command.created_at,
            "valid_until": command.valid_until,
            "dispatched_at": command.dispatched_at,
            "worker_id": command.worker_id,
            "accepted_at": command.accepted_at,
            "cancel_requested": command.cancel_requested_at is not None,
            "cancel_requested_at": command.cancel_requested_at,
            "finished_at": command.finished_at,
            "last_event_at": command.last_event_at,
            "operator_cleared_at": command.operator_cleared_at,
            "reason_code": command.reason_code,
            "result": command.result,
            **command.request,
        }
        if with_events:
            presented["events"] = [
                {
                    "id": event.id,
                    "worker_event_id": event.worker_event_id,
                    "event_type": event.event_type,
                    "previous_state": event.previous_state,
                    "new_state": event.new_state,
                    "occurred_at": event.occurred_at,
                    "received_at": event.received_at,
                    "payload": event.payload,
                }
                for event in command.events
            ]
        return presented

    @staticmethod
    def _blocks(command: ArmCommandRecord) -> bool:
        state = ArmCommandState(command.state)
        return (
            state == ArmCommandState.QUEUED
            or state in IN_FLIGHT
            or (state == ArmCommandState.UNKNOWN and command.operator_cleared_at is None)
        )

    # ---- internal state changes ----

    @staticmethod
    def _record(
        session: Session,
        command: ArmCommandRecord,
        event_type: str,
        new_state: ArmCommandState | None,
        payload: dict[str, Any],
        now: datetime,
        *,
        worker_event_id: str | None = None,
        occurred_at: str | None = None,
    ) -> None:
        previous = command.state
        if new_state is not None:
            command.state = new_state.value
            if new_state not in IN_FLIGHT and new_state != ArmCommandState.QUEUED:
                command.finished_at = command.finished_at or _iso(now)
        session.add(
            ArmCommandEventRecord(
                id=f"AEV-{uuid.uuid4().hex[:12].upper()}",
                command_id=command.id,
                worker_event_id=worker_event_id,
                event_type=event_type,
                previous_state=previous,
                new_state=command.state,
                occurred_at=occurred_at,
                received_at=_iso(now),
                payload=payload,
            )
        )

    def _settle(self, session: Session, command: ArmCommandRecord, now: datetime) -> None:
        """Apply deadlines lazily: an unclaimed command expires, an unacknowledged claim becomes UNKNOWN."""
        state = ArmCommandState(command.state)
        if state == ArmCommandState.QUEUED and now >= datetime.fromisoformat(command.valid_until):
            command.reason_code = "EXPIRED_BEFORE_DISPATCH"
            self._record(
                session, command, "EXPIRED", ArmCommandState.EXPIRED, {"source": "control_station"}, now
            )
        elif (
            state == ArmCommandState.DISPATCHED
            and command.dispatched_at is not None
            and now >= datetime.fromisoformat(command.dispatched_at) + timedelta(seconds=self.ack_timeout_s)
        ):
            command.reason_code = "ACK_TIMEOUT"
            self._record(
                session,
                command,
                "ACK_TIMEOUT",
                ArmCommandState.UNKNOWN,
                {
                    "source": "control_station",
                    "ack_timeout_s": self.ack_timeout_s,
                    "message": "워커가 명령을 가져간 뒤 ACCEPTED를 보내지 않았습니다. 팔 상태를 직접 확인하세요.",
                },
                now,
            )

    def _robot_commands(self, session: Session, robot_id: str, now: datetime) -> list[ArmCommandRecord]:
        commands = list(
            session.scalars(
                select(ArmCommandRecord)
                .where(ArmCommandRecord.robot_id == robot_id)
                .where(
                    ArmCommandRecord.state.in_(
                        [state.value for state in (ArmCommandState.QUEUED, *IN_FLIGHT, ArmCommandState.UNKNOWN)]
                    )
                )
                .order_by(ArmCommandRecord.created_at)
            )
        )
        for command in commands:
            self._settle(session, command, now)
        return commands

    def _load(self, session: Session, command_id: str, now: datetime) -> ArmCommandRecord:
        command = session.get(ArmCommandRecord, command_id)
        if command is None:
            raise ArmCommandNotFound(command_id)
        self._settle(session, command, now)
        return command

    # ---- operator side ----

    def create(self, payload: Mapping[str, object]) -> dict[str, Any]:
        robot_id, operation, valid_for_s, request = validate_command_request(payload)
        now = _now()
        with self._lock:
            with self._sessions.begin() as session:
                blocking = [command for command in self._robot_commands(session, robot_id, now) if self._blocks(command)]
                if blocking:
                    raise ArmCommandConflict(
                        "ARM_COMMAND_ACTIVE",
                        "이 팔에 끝나지 않았거나 확인되지 않은 명령이 있어 새 명령을 만들 수 없습니다.",
                        self._present(blocking[0]),
                    )
                command = ArmCommandRecord(
                    id=f"CMD-{uuid.uuid4().hex[:12].upper()}",
                    robot_id=robot_id,
                    operation=operation,
                    state=ArmCommandState.QUEUED.value,
                    request=request,
                    created_at=_iso(now),
                    valid_until=_iso(now + timedelta(seconds=valid_for_s)),
                )
                session.add(command)
                self._record(
                    session, command, "CREATED", None, {"source": "operator", "valid_for_s": valid_for_s}, now
                )
                presented = self._present(command)
            self._new_command.notify_all()
        return presented

    def cancel(self, command_id: str) -> dict[str, Any]:
        now = _now()
        with self._lock, self._sessions.begin() as session:
            command = self._load(session, command_id, now)
            state = ArmCommandState(command.state)
            if state == ArmCommandState.QUEUED:
                command.reason_code = "CANCELED_BEFORE_DISPATCH"
                self._record(session, command, "CANCELED", ArmCommandState.CANCELED, {"source": "operator"}, now)
            elif state in IN_FLIGHT:
                if command.cancel_requested_at is None:
                    command.cancel_requested_at = _iso(now)
                    self._record(session, command, "CANCEL_REQUESTED", None, {"source": "operator"}, now)
            else:
                raise ArmCommandConflict(
                    "ARM_COMMAND_FINISHED", f"{state.value} 상태의 명령은 취소할 수 없습니다.", self._present(command)
                )
            return self._present(command)

    def clear_unknown(self, command_id: str, note: object) -> dict[str, Any]:
        """Record that an operator checked the arm after UNKNOWN; the outcome itself stays unknown."""
        note_text = _text(note, "note", 300)
        now = _now()
        with self._lock, self._sessions.begin() as session:
            command = self._load(session, command_id, now)
            if command.state != ArmCommandState.UNKNOWN.value or command.operator_cleared_at is not None:
                raise ArmCommandConflict(
                    "ARM_COMMAND_NOT_UNKNOWN",
                    "확인 처리는 아직 확인되지 않은 UNKNOWN 명령에만 할 수 있습니다.",
                    self._present(command),
                )
            command.operator_cleared_at = _iso(now)
            self._record(session, command, "OPERATOR_CLEARED", None, {"source": "operator", "note": note_text}, now)
            return self._present(command)

    def get(self, command_id: str) -> dict[str, Any]:
        now = _now()
        with self._lock, self._sessions.begin() as session:
            return self._present(self._load(session, command_id, now), with_events=True)

    def list_recent(self, limit: int = 20) -> dict[str, Any]:
        now = _now()
        with self._lock, self._sessions.begin() as session:
            for robot_id in ARM_ROBOT_IDS:
                self._robot_commands(session, robot_id, now)
            commands = session.scalars(
                select(ArmCommandRecord).order_by(ArmCommandRecord.created_at.desc()).limit(limit)
            )
            return {
                "commands": [self._present(command) for command in commands],
                "workers": self.workers(),
                "ack_timeout_s": self.ack_timeout_s,
            }

    def workers(self) -> list[dict[str, Any]]:
        with self._lock:
            return [{"robot_id": robot_id, **dict(state)} for robot_id, state in sorted(self._workers.items())]

    # ---- worker side ----

    def claim_next(self, robot_id: str, worker_id: object, wait_s: float) -> dict[str, Any] | None:
        """Long-poll for the oldest queued command of one arm; None when nothing arrives in time."""
        if robot_id not in ARM_ROBOT_IDS:
            raise ArmCommandNotFound(robot_id)
        worker = _matching(worker_id, "worker_id", _TOKEN)
        if not math.isfinite(wait_s) or not 0 <= wait_s <= MAX_WAIT_S:
            raise ArmCommandError(f"wait_s must be from 0 to {MAX_WAIT_S:g}")
        deadline = time.monotonic() + wait_s
        with self._lock:
            heartbeat = self._workers.setdefault(robot_id, {"waiting_polls": 0})
            heartbeat.update(worker_id=worker, last_poll_at=_iso(_now()))
            heartbeat["waiting_polls"] += 1
            try:
                while True:
                    claimed = self._try_claim(robot_id, worker)
                    remaining = deadline - time.monotonic()
                    if claimed is not None or remaining <= 0:
                        return claimed
                    self._new_command.wait(min(remaining, 1.0))
            finally:
                heartbeat["waiting_polls"] -= 1
                heartbeat["last_poll_at"] = _iso(_now())

    def _try_claim(self, robot_id: str, worker_id: str) -> dict[str, Any] | None:
        now = _now()
        with self._sessions.begin() as session:
            commands = self._robot_commands(session, robot_id, now)
            if any(ArmCommandState(command.state) in IN_FLIGHT for command in commands):
                return None
            queued = next((c for c in commands if c.state == ArmCommandState.QUEUED.value), None)
            if queued is None:
                return None
            queued.dispatched_at = _iso(now)
            queued.worker_id = worker_id
            self._record(
                session,
                queued,
                "DISPATCHED",
                ArmCommandState.DISPATCHED,
                {"source": "control_station", "worker_id": worker_id, "ack_timeout_s": self.ack_timeout_s},
                now,
            )
            return self._present(queued)

    def apply_worker_event(self, command_id: str, body: Mapping[str, object]) -> tuple[dict[str, Any], bool]:
        """Apply one worker event; returns (command, applied). A repeated event_id is a no-op."""
        if set(body) - WORKER_EVENT_FIELDS:
            raise ArmCommandError("Unsupported fields: " + ", ".join(sorted(set(body) - WORKER_EVENT_FIELDS)))
        event_id = _matching(body.get("event_id"), "event_id", _TOKEN)
        worker_id = _matching(body.get("worker_id"), "worker_id", _TOKEN)
        event = body.get("event")
        if event not in WORKER_TRANSITIONS:
            raise ArmCommandError("event must be one of: " + ", ".join(sorted(WORKER_TRANSITIONS)))
        if not isinstance(body.get("simulated"), bool):
            raise ArmCommandError("simulated must be true or false")
        occurred_at = body.get("occurred_at")
        if occurred_at is not None:
            try:
                if not isinstance(occurred_at, str) or len(occurred_at) > 40:
                    raise ValueError
                datetime.fromisoformat(occurred_at)
            except ValueError as exc:
                raise ArmCommandError("occurred_at must be an ISO 8601 timestamp") from exc
        phase = body.get("phase")
        if event == "PROGRESS" and phase not in PHASES:
            raise ArmCommandError("PROGRESS needs phase, one of: " + ", ".join(sorted(PHASES)))
        if phase is not None and phase not in PHASES:
            raise ArmCommandError("phase must be one of: " + ", ".join(sorted(PHASES)))
        reason_code = body.get("reason_code")
        if event in REASON_REQUIRED or reason_code is not None:
            reason_code = _matching(reason_code, "reason_code", _REASON_CODE)
        _text(body.get("message"), "message", 300, required=False)
        for field in ("grasp_confirmed", "place_confirmed"):
            if body.get(field) is not None and not isinstance(body.get(field), bool):
                raise ArmCommandError(f"{field} must be true or false")
        if event == "SUCCEEDED" and not (body.get("grasp_confirmed") is True and body.get("place_confirmed") is True):
            raise ArmCommandError(
                "SUCCEEDED needs grasp_confirmed and place_confirmed true; use RESULT_UNKNOWN when they cannot be confirmed"
            )
        if body.get("details") is not None and not isinstance(body.get("details"), Mapping):
            raise ArmCommandError("details must be an object")

        now = _now()
        payload = {"source": "worker", **dict(body)}
        with self._lock:
            try:
                with self._sessions.begin() as session:
                    command = self._load(session, command_id, now)
                    if body.get("robot_id") != command.robot_id:
                        raise ArmCommandConflict("ROBOT_MISMATCH", "robot_id does not match the command", self._present(command))
                    previous = session.scalar(
                        select(ArmCommandEventRecord).where(
                            ArmCommandEventRecord.command_id == command_id,
                            ArmCommandEventRecord.worker_event_id == event_id,
                        )
                    )
                    if previous is not None:
                        if previous.event_type != event:
                            raise ArmCommandConflict(
                                "EVENT_ID_REUSED", "event_id was already used for a different event", self._present(command)
                            )
                        return self._present(command), False
                    if worker_id != command.worker_id:
                        raise ArmCommandConflict(
                            "WORKER_MISMATCH", "이 명령을 가져간 워커가 아닙니다.", self._present(command)
                        )
                    current = ArmCommandState(command.state)
                    next_state = WORKER_TRANSITIONS[event].get(current)
                    if next_state is None:
                        raise ArmCommandConflict(
                            "INVALID_TRANSITION",
                            f"{current.value} 상태에서 {event} 이벤트를 받을 수 없습니다.",
                            self._present(command),
                        )
                    if event == "ACCEPTED" and (
                        command.cancel_requested_at is not None
                        or now >= datetime.fromisoformat(command.valid_until)
                    ):
                        # The worker is told not to execute, so the command closes without motion.
                        canceled = command.cancel_requested_at is not None
                        command.reason_code = "CANCELED_BEFORE_ACCEPT" if canceled else "EXPIRED_BEFORE_ACCEPT"
                        self._record(
                            session,
                            command,
                            "ACCEPT_REFUSED",
                            ArmCommandState.CANCELED if canceled else ArmCommandState.EXPIRED,
                            payload,
                            now,
                            worker_event_id=event_id,
                            occurred_at=occurred_at,
                        )
                        presented = self._present(command)
                        error = ArmCommandConflict(
                            command.reason_code, "이 명령은 실행하면 안 됩니다 (취소 또는 유효 시간 초과).", presented
                        )
                    else:
                        if event == "CANCELED" and command.cancel_requested_at is None:
                            raise ArmCommandConflict(
                                "CANCEL_NOT_REQUESTED",
                                "취소 요청이 없는 명령입니다. 스스로 멈췄다면 FAILED를 보내세요.",
                                self._present(command),
                            )
                        if event == "ACCEPTED":
                            command.accepted_at = _iso(now)
                        if event in REASON_REQUIRED:
                            command.reason_code = reason_code
                        if event in {"SUCCEEDED", "FAILED", "RESULT_UNKNOWN", "REJECTED", "CANCELED"}:
                            command.result = payload
                        command.last_event_at = _iso(now)
                        self._record(
                            session,
                            command,
                            event,
                            next_state,
                            payload,
                            now,
                            worker_event_id=event_id,
                            occurred_at=occurred_at,
                        )
                        return self._present(command), True
            except IntegrityError as exc:
                raise ArmCommandConflict("EVENT_ID_REUSED", "event_id was recorded concurrently") from exc
        raise error
