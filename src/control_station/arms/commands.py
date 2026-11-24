"""외부 팔 워커에게 HTTP로 전달하는 팔 명령 큐 (계약: docs/implementation/arm-command-api.md).

관제는 여기서 팔을 직접 움직이지 않는다. 명령마다 대상을 하나 고정해 기록하면, 외부 워커(LeRobot OMX 노트북,
또는 같은 PC)가 HTTP long-poll로 가져가고(claim) 진행 이벤트를 보고한다. 워커는 자기 ACCEPTED 이벤트에 HTTP 200을
받은 뒤에만 팔을 움직일 수 있고, 그 밖의 답은 "실행하지 말라"는 뜻이다. 이 약속 덕분에 ACCEPTED가 오지 않은 명령은
팔이 움직였는지 추측하지 않고 닫을 수 있다.

직접 구현한 이유: 팔 쪽 스택(LeRobot)이 ROS 2 노드가 아니라서 그쪽에서는 아직 ROS 2 action을 쓸 수 없었다. 나중에
워커가 ROS 2로 옮겨 가면 필드가 action의 goal/feedback/result에 하나씩 대응한다(주문 집기는 arms/pick.py의
ArmActionPick이 이미 그렇게 한다).

핵심 규칙
- 상태: QUEUED -> DISPATCHED(워커가 가져감) -> ACCEPTED -> RUNNING -> SUCCEEDED/FAILED/UNKNOWN/CANCELED,
  그 밖에 REJECTED, EXPIRED. 워커 이벤트로 바꿀 수 있는 전이는 WORKER_TRANSITIONS 표에 있는 것뿐이다.
- 팔 하나에 끝나지 않은 명령은 1건만 둔다(ARM_COMMAND_ACTIVE). 정책 집기가 진행 중이면 어느 팔에도 새 명령을 받지
  않는다(POLICY_RUN_ACTIVE). 두 팔은 번갈아 움직인다.
- UNKNOWN은 자동으로 재전송하거나 성공·실패로 바꾸지 않는다. 운영자가 현장 확인(clear_unknown)을 남기기 전까지
  그 팔의 새 명령을 막는다.
- 마감 시각(valid_until, ACK 제한 시간)은 타이머가 아니라 명령을 읽을 때마다 _settle에서 적용한다(lazy).
- 모든 상태 변경은 self._lock 안의 DB 트랜잭션 하나로 한다. 서버 프로세스 하나 안에서만 직렬화된다.
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

from control_station.arms.policies import ActPolicyRegistry
from control_station.cameras.shelf_slots import ShelfSlotMap
from control_station.storage.database import ArmCommandEventRecord, ArmCommandRecord
from control_station.storage.models import StringEnum

# arm-01은 왼쪽 팔(왼쪽 선반), arm-02는 오른쪽 팔(오른쪽 선반)이다. 선반과의 대응은 config/arms.json에 있다.
ARM_ROBOT_IDS = frozenset({"arm-01", "arm-02"})
# MOVE_TO_SLOT: 집지 않고, 그 칸에 대해 기록해 둔 자세로 팔을 옮긴다. 동작을 먼저 확인하는 데 쓴다.
# RETURN_HOME: 워커가 기록한 "home" 자세로 돌려보낸다. 물체·칸·놓을 곳을 지정하지 않는다.
# PICK_WITH_POLICY: 지정한 물체용으로 학습한 LeRobot ACT 정책을 실행한다(config/omx-act-policies.json). 정책은
#   자기 카메라 영상에서 물체를 찾는다. 카메라가 본 칸이 있으면 그것은 근거일 뿐 입력이 아니다.
# JOG_JOINT: 운영자 전용, 고정 크기(워커 쪽 20틱)의 상대 조그. 목표 틱 값이나 좌표는 받지 않는다.
# 좌표(XYZ) 명령은 2026-11-11에 없앴다.
OPERATIONS = frozenset({"MOVE_TO_SLOT", "RETURN_HOME", "PICK_WITH_POLICY", "JOG_JOINT"})
# SUCCEEDED가 "집고 놓았다"가 아니라 "측정한 관절값이 계획한 자세에 도달했다"를 뜻하는 동작들.
POSE_OPERATIONS = frozenset({"MOVE_TO_SLOT", "RETURN_HOME", "JOG_JOINT"})
JOG_JOINTS = frozenset({"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"})
JOG_DIRECTIONS = frozenset({"POSITIVE", "NEGATIVE"})
# 클라이언트가 보낼 수 있는 출처는 OPERATOR_INPUT뿐이다. CAMERA 출처는 서버가 직접 붙인다(create의 camera_source).
SOURCE_KINDS = frozenset({"OPERATOR_INPUT"})
# 진행 단계(phase) 이름은 관제의 집기 단계 이름을 따른다. 그래야 진행 보고를 단계에 대응시킬 수 있다.
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
        "RETURNING_HOME",
    }
)
# 워커 long-poll 한 번이 기다릴 수 있는 최대 시간(초).
MAX_WAIT_S = 30.0

# 칸 ID, 워커·이벤트 ID, 이유 코드의 허용 형식.
_SLOT_ID = re.compile(r"^[A-Za-z0-9_.-]{1,40}$")
_TOKEN = re.compile(r"^[A-Za-z0-9_.:-]{1,80}$")
_REASON_CODE = re.compile(r"^[A-Z0-9_]{1,60}$")


class ArmCommandState(StringEnum):
    """팔 명령 상태 (값은 DB와 API에 그대로 쓰는 문자열)."""
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


# 워커가 가져간 뒤 아직 끝나지 않은 상태. 이 상태인 명령이 있으면 그 팔은 다음 명령을 가져갈 수 없다.
IN_FLIGHT = frozenset({ArmCommandState.DISPATCHED, ArmCommandState.ACCEPTED, ArmCommandState.RUNNING})

# 워커 이벤트 -> {현재 상태: 다음 상태}. 표에 없는 조합은 상태를 바꾸지 않고 거부한다(409 INVALID_TRANSITION).
# REJECTED는 DISPATCHED(아직 ACCEPTED 전)에서만 받는다. 팔이 움직이기 전에만 거부할 수 있다는 뜻이다.
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
# reason_code가 반드시 있어야 하는 이벤트.
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
        "pose_reached",
        "place_confirmed",
        "details",
    }
)


class ArmCommandError(ValueError):
    """요청이 팔 명령 계약에 맞지 않는다 (HTTP 400)."""


class ArmCommandNotFound(LookupError):
    """없는 명령 ID 또는 모르는 팔 (HTTP 404)."""
    pass


class ArmCommandConflict(RuntimeError):
    """요청 형식은 맞지만 지금 상태에서는 허용되지 않는다 (HTTP 409). command는 응답에 함께 보낼 현재 명령."""

    def __init__(self, error_code: str, message: str, command: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.command = command


def _now() -> datetime:
    """지금 시각 (UTC, 시간대 포함)."""
    return datetime.now(timezone.utc)


def _iso(moment: datetime) -> str:
    """DB에 저장하는 시각 문자열. 마이크로초까지 남겨 created_at 정렬이 같은 초 안에서도 순서를 지킨다."""
    return moment.isoformat(timespec="microseconds")


def _text(value: object, field: str, maximum: int, *, required: bool = True) -> str | None:
    """1–maximum자, 제어 문자 없는 글자 필드를 검증하고 앞뒤 공백을 뺀다. required=False면 None도 받는다."""
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
    """정규식에 통째로 맞는 문자열인지 확인한다."""
    if not isinstance(value, str) or not pattern.fullmatch(value):
        raise ArmCommandError(f"{field} has an invalid format")
    return value


def _slot_target(target: Mapping[str, object], slot_map: ShelfSlotMap | None) -> dict[str, Any]:
    """칸 대상은 선반 칸 하나를 가리킨다. 워커는 그 칸에 대해 자기가 기록한 자세를 재생한다.

    칸 이름과 칸 정의 파일 버전을 함께 고정해 둔다. 워커는 버전이 자기 것과 다르면 SLOT_MAP_MISMATCH로 거부한다."""
    if slot_map is None:
        raise ArmCommandError("칸 정의 파일(config/shelf-slots.json)이 없어 pick_slot_id를 쓸 수 없습니다")
    slot = slot_map.active_slots.get(target.get("pick_slot_id"))  # type: ignore[arg-type]
    if slot is None:
        raise ArmCommandError("pick_slot_id must be one of: " + ", ".join(sorted(slot_map.active_slots)))
    return {
        "pick_slot_id": slot["slot_id"],
        "slot_label": slot["label"],
        "slot_map_version": slot_map.version,
    }


def _policy_target(
    target: Mapping[str, object], slot_map: ShelfSlotMap | None, policies: ActPolicyRegistry | None, robot_id: str
) -> dict[str, Any]:
    """정책은 물체 이름으로 고른다. 선택 항목인 칸은 카메라가 물체를 본 곳(근거일 뿐)이다.

    고른 정책의 ID·모델·정책 파일 버전을 명령에 고정한다. 워커는 버전이 다르면 POLICY_REGISTRY_MISMATCH로 거부한다."""
    if policies is None:
        raise ArmCommandError("정책 정의 파일(config/omx-act-policies.json)이 없어 PICK_WITH_POLICY를 쓸 수 없습니다")
    if set(target) - {"object_class", "pick_slot_id"}:
        raise ArmCommandError("PICK_WITH_POLICY target takes only object_class and an optional pick_slot_id")
    name = _text(target.get("object_class"), "target.object_class", 80)
    policy = policies.resolve(name, robot_id)  # type: ignore[arg-type]
    if policy is None:
        known = ", ".join(
            f"{item['label']}({item['object_class']})"
            for item in policies.policies.values()
            if item["arm_id"] in (None, robot_id)
        )
        raise ArmCommandError(f"{robot_id}에는 {name}에 학습된 정책이 없습니다. 가능한 물체: {known}")
    slot = (
        _slot_target(target, slot_map)
        if target.get("pick_slot_id") is not None
        else dict.fromkeys(("pick_slot_id", "slot_label", "slot_map_version"))
    )
    return {
        "object_class": policy["object_class"],
        **slot,
        "policy": {
            "policy_id": policy["policy_id"],
            "label": policy["label"],
            "model": policy["model"],
            "registry_version": policies.version,
        },
    }


def validate_command_request(
    payload: Mapping[str, object],
    slot_map: ShelfSlotMap | None = None,
    policies: ActPolicyRegistry | None = None,
) -> tuple[str, str, int, dict[str, Any]]:
    """(robot_id, operation, valid_for_s, 고정된 요청)을 돌려주거나 ArmCommandError를 낸다.

    MOVE_TO_SLOT은 선반 칸 하나(`pick_slot_id`), RETURN_HOME은 아무것도 지정하지 않고, PICK_WITH_POLICY는 학습 정책이
    있는 물체 하나, JOG_JOINT는 관절 하나와 방향 하나를 지정한다(틱 값은 절대 받지 않는다).
    valid_for_s는 5–600초이고, JOG_JOINT만 5–30초다(조그는 바로 실행될 때만 의미가 있다).
    """
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
    source = _operator_source(payload.get("source"))
    if operation == "JOG_JOINT":
        if not 5 <= valid_for_s <= 30:
            raise ArmCommandError("JOG_JOINT valid_for_s must be an integer from 5 to 30")
        target = payload.get("target")
        if not isinstance(target, Mapping) or set(target) != {"joint", "direction"}:
            raise ArmCommandError("JOG_JOINT target must contain exactly joint and direction")
        joint, direction = target.get("joint"), target.get("direction")
        if not isinstance(joint, str) or joint not in JOG_JOINTS:
            raise ArmCommandError("target.joint must name one of the five arm joints")
        if not isinstance(direction, str) or direction not in JOG_DIRECTIONS:
            raise ArmCommandError("target.direction must be POSITIVE or NEGATIVE")
        if payload.get("place") is not None:
            raise ArmCommandError("JOG_JOINT takes no place")
        return str(robot_id), str(operation), valid_for_s, {
            "target": {"joint": joint, "direction": direction},
            "place": None,
            "source": source,
        }
    if operation == "RETURN_HOME":
        if payload.get("target") not in (None, {}) or payload.get("place") is not None:
            raise ArmCommandError("RETURN_HOME takes no target or place; the worker uses its recorded home pose")
        home_target = dict.fromkeys(("object_class", "pick_slot_id", "slot_label", "slot_map_version"))
        return str(robot_id), str(operation), valid_for_s, {"target": home_target, "place": None, "source": source}

    target = payload.get("target")
    if operation == "PICK_WITH_POLICY":
        if not isinstance(target, Mapping):
            raise ArmCommandError("PICK_WITH_POLICY needs target.object_class")
        place = payload.get("place")
        if not isinstance(place, Mapping) or set(place) != {"slot_id"}:
            raise ArmCommandError("place must be an object with exactly slot_id")
        request = {
            "target": _policy_target(target, slot_map, policies, str(robot_id)),
            "place": {"slot_id": _matching(place.get("slot_id"), "place.slot_id", _SLOT_ID)},
            "source": source,
        }
        return str(robot_id), str(operation), valid_for_s, request
    target_fields = {"object_class", "pick_slot_id"}
    if not isinstance(target, Mapping) or set(target) - target_fields:
        raise ArmCommandError("target must be an object with " + ", ".join(sorted(target_fields)))
    if target.get("pick_slot_id") is None:
        raise ArmCommandError("MOVE_TO_SLOT needs target.pick_slot_id")
    frozen_target: dict[str, Any] = {
        "object_class": _text(target.get("object_class"), "target.object_class", 80, required=False),
        **_slot_target(target, slot_map),
    }
    place = payload.get("place")
    if not isinstance(place, Mapping) or set(place) != {"slot_id"}:
        raise ArmCommandError("place must be an object with exactly slot_id")
    frozen_place = {"slot_id": _matching(place.get("slot_id"), "place.slot_id", _SLOT_ID)}
    request = {"target": frozen_target, "place": frozen_place, "source": source}
    return str(robot_id), str(operation), valid_for_s, request


def _operator_source(source: object) -> dict[str, Any]:
    """출처는 {kind: OPERATOR_INPUT, note?}만 받는다(note는 200자까지)."""
    if not isinstance(source, Mapping) or set(source) - {"kind", "note"}:
        raise ArmCommandError("source must be an object with kind and optional note")
    if source.get("kind") not in SOURCE_KINDS:
        raise ArmCommandError("source.kind must be one of: " + ", ".join(sorted(SOURCE_KINDS)))
    return {"kind": source["kind"], "note": _text(source.get("note"), "source.note", 200, required=False)}


def _require_success_evidence(operation: str, body: Mapping[str, object]) -> None:
    """SUCCEEDED에는 그 동작에 필요한 근거가 있어야 한다. 근거가 없으면 워커는 RESULT_UNKNOWN을 보내야 한다.

    자세 동작은 pose_reached, 집기는 grasp_confirmed와 place_confirmed가 모두 true여야 한다(SR-14)."""
    if operation in POSE_OPERATIONS:
        if body.get("pose_reached") is not True:
            raise ArmCommandError(f"{operation} SUCCEEDED needs pose_reached true (measured joint positions at the goal)")
    elif not (body.get("grasp_confirmed") is True and body.get("place_confirmed") is True):
        raise ArmCommandError(
            "SUCCEEDED needs grasp_confirmed and place_confirmed true; use RESULT_UNKNOWN when they cannot be confirmed"
        )


class ArmCommandStore:
    """팔 명령을 DB에 저장하고, 이 서버 프로세스 안에서 상태 변경을 한 번에 하나씩 처리한다.

    _lock(RLock)과 그 위의 Condition(_new_command): 새 명령이 생기면 long-poll로 기다리는 워커들을 깨운다.
    _workers: 팔마다 마지막 long-poll 시각과 지금 기다리는 요청 수 (화면의 "워커 대기 중" 표시, DB에 저장하지 않음)."""

    def __init__(
        self,
        sessions: sessionmaker,
        *,
        ack_timeout_s: float | None = None,
        slot_map: ShelfSlotMap | None = None,
        slot_map_error: str | None = None,
        act_policies: ActPolicyRegistry | None = None,
    ) -> None:
        self._sessions = sessions
        self.slot_map = slot_map
        self.slot_map_error = slot_map_error
        self.act_policies = act_policies
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

    # ---- 응답 모양 ----

    @staticmethod
    def _present(command: ArmCommandRecord, *, with_events: bool = False) -> dict[str, Any]:
        """DB 기록을 API의 command 객체로 바꾼다. 고정된 요청(target, place, source)은 맨 위 수준에 펼친다."""
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
        """이 명령이 같은 팔의 새 명령을 막는지: 대기·실행 중이거나, 운영자가 아직 확인하지 않은 UNKNOWN."""
        state = ArmCommandState(command.state)
        return (
            state == ArmCommandState.QUEUED
            or state in IN_FLIGHT
            or (state == ArmCommandState.UNKNOWN and command.operator_cleared_at is None)
        )

    # ---- 내부 상태 변경 (모두 호출하는 쪽의 잠금·트랜잭션 안에서 불린다) ----

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
        """상태를 바꾸고(new_state가 None이면 그대로) 이벤트 이력 한 줄을 남긴다. 끝 상태가 되면 finished_at을 적는다."""
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
        """마감 시각을 읽는 시점에 적용한다(lazy): 가져가지 않은 명령은 EXPIRED, 가져갔는데 ACK가 없으면 UNKNOWN.

        ACK 없는 명령을 UNKNOWN으로 두는 이유: 워커 약속대로라면 팔은 움직이지 않았겠지만, 워커 버그 가능성 때문에
        사람이 확인하게 한다."""
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
        """그 팔의 열린 명령(대기·실행 중·UNKNOWN)을 오래된 순으로, 마감을 적용한 뒤 돌려준다."""
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
        """명령 하나를 읽고 마감을 적용한다. 없으면 ArmCommandNotFound."""
        command = session.get(ArmCommandRecord, command_id)
        if command is None:
            raise ArmCommandNotFound(command_id)
        self._settle(session, command, now)
        return command

    # ---- 운영자(관제) 쪽 ----

    def create(
        self, payload: Mapping[str, object], camera_source: Mapping[str, object] | None = None
    ) -> dict[str, Any]:
        """명령을 큐에 넣는다. `camera_source`는 서버가 카메라 스냅숏에서 직접 얻은 근거다. 클라이언트는
        `source.kind: OPERATOR_INPUT`만 보낼 수 있으므로, 카메라 출처를 HTTP로 직접 꾸며 보낼 수 없다.

        거부 순서: 정책 집기가 진행 중(POLICY_RUN_ACTIVE) -> 정책 집기인데 다른 팔 명령이 진행 중
        (OTHER_ARM_COMMAND_ACTIVE) -> 이 팔에 끝나지 않았거나 확인 안 된 명령(ARM_COMMAND_ACTIVE)."""
        robot_id, operation, valid_for_s, request = validate_command_request(payload, self.slot_map, self.act_policies)
        if camera_source is not None:
            # 칸 대상은 카메라가 본 칸과 같아야 한다.
            if request["target"]["pick_slot_id"] is not None and camera_source.get("slot_id") != request["target"]["pick_slot_id"]:
                raise ArmCommandError("camera evidence is for a different slot than the command target")
            request["source"] = {"kind": "CAMERA", **camera_source}
        now = _now()
        with self._lock:
            with self._sessions.begin() as session:
                open_commands = list(session.scalars(
                    select(ArmCommandRecord)
                    .where(
                        ArmCommandRecord.state.in_(
                            [state.value for state in (ArmCommandState.QUEUED, *IN_FLIGHT, ArmCommandState.UNKNOWN)]
                        )
                    )
                    .order_by(ArmCommandRecord.created_at)
                ))
                # 마감을 먼저 적용해야 이미 만료된 명령이 새 명령을 막지 않는다.
                for active in open_commands:
                    self._settle(session, active, now)
                blocking_all = [active for active in open_commands if self._blocks(active)]
                active_policy = next(
                    (active for active in blocking_all if active.operation == "PICK_WITH_POLICY"), None
                )
                if active_policy is not None:
                    raise ArmCommandConflict(
                        "POLICY_RUN_ACTIVE",
                        "정책 집기 실행이 끝나거나 현장 확인될 때까지 다른 팔 명령을 등록할 수 없습니다.",
                        self._present(active_policy),
                    )
                if operation == "PICK_WITH_POLICY":
                    other_arm_command = next(
                        (active for active in blocking_all if active.robot_id != robot_id), None
                    )
                    if other_arm_command is not None:
                        raise ArmCommandConflict(
                            "OTHER_ARM_COMMAND_ACTIVE",
                            "다른 팔 명령이 끝나거나 현장 확인될 때까지 정책 집기를 등록할 수 없습니다.",
                            self._present(other_arm_command),
                        )
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
                    session,
                    command,
                    "CREATED",
                    None,
                    {"source": "camera" if camera_source is not None else "operator", "valid_for_s": valid_for_s},
                    now,
                )
                presented = self._present(command)
            # 커밋 뒤에 기다리는 워커들을 깨운다(커밋 전에 깨우면 워커가 아직 새 명령을 못 본다).
            self._new_command.notify_all()
        return presented

    def cancel(self, command_id: str) -> dict[str, Any]:
        """취소. QUEUED면 바로 CANCELED, 실행 중이면 취소 요청만 표시한다(워커가 단계 사이에 보고 멈춘 뒤 CANCELED를
        보낸다). 이미 끝난 명령은 409 ARM_COMMAND_FINISHED."""
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

    def cancel_active(self) -> list[dict[str, Any]]:
        """대기 중이거나 실행 중인 명령을 모두 취소한다(전체 정지 요청, stop_all.py). 대기 중인 것은 바로 끝나고,
        실행 중인 것에는 워커가 움직이는 동안 약 1초마다 확인하는 취소 플래그가 켜진다.
        요청 뒤의 명령들을 돌려준다. 그사이 끝난 명령은 빠진다."""
        active = [state.value for state in (ArmCommandState.QUEUED, *IN_FLIGHT)]
        with self._lock, self._sessions.begin() as session:
            command_ids = list(session.scalars(
                select(ArmCommandRecord.id).where(ArmCommandRecord.state.in_(active)).order_by(ArmCommandRecord.created_at)
            ))
        canceled: list[dict[str, Any]] = []
        for command_id in command_ids:
            try:
                canceled.append(self.cancel(command_id))
            except (ArmCommandConflict, ArmCommandNotFound):
                continue  # 두 번 읽는 사이에 끝났거나 만료됨: 취소할 것이 없다
        return canceled

    def clear_unknown(self, command_id: str, note: object) -> dict[str, Any]:
        """UNKNOWN 뒤에 운영자가 팔을 확인했다는 기록을 남긴다. 결과 자체는 계속 UNKNOWN이다(성공·실패로 바꾸지 않음).
        이 기록이 생기면 그 팔은 다시 새 명령을 받을 수 있다."""
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
        """명령 하나와 전체 이벤트 이력."""
        now = _now()
        with self._lock, self._sessions.begin() as session:
            return self._present(self._load(session, command_id, now), with_events=True)

    def list_recent(self, limit: int = 20) -> dict[str, Any]:
        """최근 명령(기본 20건, 최신 순), 워커 접속 상태, ACK 제한 시간. 화면이 1초마다 부르므로 여기서 마감도 적용된다."""
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
        """팔마다 워커 접속 상태: worker_id, last_poll_at, waiting_polls(지금 기다리는 long-poll 수)."""
        with self._lock:
            return [{"robot_id": robot_id, **dict(state)} for robot_id, state in sorted(self._workers.items())]

    # ---- 워커 쪽 ----

    def claim_next(self, robot_id: str, worker_id: object, wait_s: float) -> dict[str, Any] | None:
        """팔 하나의 가장 오래된 대기 명령을 long-poll로 받는다. 시간 안에 없으면 None(API는 204).

        Condition.wait는 잠금을 잠시 놓고 기다리므로 다른 요청을 막지 않는다. 깨우는 신호를 놓쳐도 늦어도 1초마다
        다시 확인한다(min(remaining, 1.0)). 기다리는 동안 이 요청이 워커의 접속 신호(heartbeat)가 된다."""
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
        """그 팔에 실행 중 명령이 없으면 가장 오래된 QUEUED 명령을 DISPATCHED로 바꿔 넘긴다. self._lock 안에서 부른다."""
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
        """워커 이벤트 하나를 적용하고 (command, applied)를 돌려준다. 같은 event_id의 재전송은 아무것도 바꾸지 않는다
        (applied False). 같은 event_id로 다른 이벤트를 보내면 409 EVENT_ID_REUSED.

        본문 형식 검사는 잠금 밖에서 먼저 하고, 상태 확인과 변경은 잠금 안의 트랜잭션 하나에서 한다."""
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
        for field in ("grasp_confirmed", "place_confirmed", "pose_reached"):
            if body.get(field) is not None and not isinstance(body.get(field), bool):
                raise ArmCommandError(f"{field} must be true or false")
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
                    if event == "SUCCEEDED":
                        _require_success_evidence(command.operation, body)
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
                        # 워커에게 실행하지 말라고 답하므로, 명령은 팔이 움직이지 않은 채로 닫힌다(CANCELED/EXPIRED).
                        # 여기서 바로 raise하면 트랜잭션이 롤백되어 이 기록이 사라진다. 그래서 오류를 만들어만 두고,
                        # 트랜잭션이 커밋된 뒤 함수 끝에서 raise한다.
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
                # 같은 event_id가 두 요청에서 동시에 기록되려 한 경우(DB 고유 제약 위반).
                raise ArmCommandConflict("EVENT_ID_REUSED", "event_id was recorded concurrently") from exc
        raise error
