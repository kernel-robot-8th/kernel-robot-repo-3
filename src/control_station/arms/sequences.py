"""운영자가 시작하는, 등록된 ACT 정책 여러 개의 연속 실행 (DB에 저장되어 서버를 다시 켜도 남는다).

예: "당근 -> 사과 -> 휴지"를 차례로 집기. 단계마다 PICK_WITH_POLICY 팔 명령을 하나씩 만들고, 워커 이벤트로 그
명령이 끝나면(api/arm.py의 post_command -> on_worker_event) 다음 단계 명령을 만든다.

- 정의: config/omx-act-sequences.json의 시퀀스(start), 또는 운영자가 고른 물체 순서(start_custom, 최대 4개).
- 다음 단계로 넘어가는 조건: 팔이 home으로 돌아온 것이 측정됐을 때(pose_reached)만이다. SUCCEEDED이거나
  UNKNOWN(PLACE_NOT_VERIFIED)이면서 pose_reached가 true여야 한다. 그 밖에는 NEEDS_INTERVENTION으로 멈춘다.
- 실행 상태: QUEUED -> DISPATCHED -> RUNNING -> COMPLETED. 취소하면 대기 중 명령은 바로 CANCELED, 실행 중이면
  CANCELING을 거쳐 NEEDS_INTERVENTION(사람이 팔 확인)이 된다.
- 한 번에 하나의 시퀀스만 진행한다.

HTTP 응답을 잃어도 진행이 멈추지 않도록, 조회할 때(_reconcile)와 같은 이벤트를 다시 받을 때 명령 상태로 다시
맞춘다.
"""

from __future__ import annotations

import json
import threading
import uuid
from pathlib import Path
from typing import Any

from sqlalchemy import select
from sqlalchemy.orm import sessionmaker

from control_station import REPO_ROOT, utc_now_iso
from control_station.arms.commands import ArmCommandStore
from control_station.storage.database import ArmSequenceRecord

DEFAULT_PATH = REPO_ROOT / "config" / "omx-act-sequences.json"
# 운영자가 고르는 연속 실행(start_custom)의 최대 단계 수.
MAX_SEQUENCE_STEPS = 4


class ArmSequenceError(ValueError):
    """정의 파일이 잘못됐거나, 지금 상태에서 시작·취소할 수 없는 요청 (API는 409)."""
    pass


class ArmSequenceSupervisor:
    """연속 실행의 시작·진행·취소를 맡는다. 진행 상태는 ArmSequenceRecord(DB)에 있고, 팔 명령은 ArmCommandStore로 만든다.

    self._lock은 시작과 단계 진행이 겹쳐 같은 시퀀스에 명령이 두 번 만들어지는 일을 막는다."""

    def __init__(self, sessions: sessionmaker, commands: ArmCommandStore, path: Path = DEFAULT_PATH) -> None:
        self.sessions = sessions
        self.commands = commands
        self.path = path
        self._lock = threading.RLock()

    def definitions(self) -> list[dict[str, Any]]:
        """정의 파일을 읽어(부를 때마다 새로) 단계마다 정책을 확인한 목록을 돌려준다. 잘못되면 ArmSequenceError.

        단계의 robot_id가 없으면 시퀀스의 robot_id를 쓴다. 그 팔에서 쓸 수 있는 정책이 없는 물체는 거부한다."""
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            if raw.get("schema_version") != 1 or not isinstance(raw.get("sequences"), list):
                raise ValueError("schema_version/sequences 형식이 올바르지 않습니다")
            policies = self.commands.act_policies
            if policies is None:
                raise ValueError("ACT 정책 설정을 읽을 수 없습니다")
            output = []
            for seq in raw["sequences"]:
                if not isinstance(seq, dict) or not isinstance(seq.get("steps"), list) or not seq["steps"]:
                    raise ValueError("각 sequence에는 하나 이상의 steps가 필요합니다")
                steps = []
                for step in seq["steps"]:
                    if not isinstance(step, dict):
                        raise ValueError("각 step은 object_class를 가진 객체여야 합니다")
                    arm_id = step.get("robot_id", seq.get("robot_id"))
                    if arm_id not in {"arm-01", "arm-02"}:
                        raise ValueError("sequence 또는 각 step의 robot_id는 arm-01 또는 arm-02여야 합니다")
                    policy = policies.resolve(step.get("object_class", ""), arm_id)
                    if policy is None:
                        raise ValueError(f"{arm_id}에 등록된 정책이 없습니다: {step.get('object_class')}")
                    steps.append({"object_class": policy["object_class"], "label": policy["label"],
                                  "policy_id": policy["policy_id"], "robot_id": arm_id})
                output.append({"sequence_id": seq["sequence_id"], "label": seq["label"],
                               "robot_id": steps[0]["robot_id"], "steps": steps})
            return output
        except (OSError, json.JSONDecodeError, AttributeError, TypeError, KeyError, ValueError) as exc:
            raise ArmSequenceError(f"시퀀스 설정을 읽지 못했습니다: {exc}") from exc

    def _present(self, row: ArmSequenceRecord) -> dict[str, Any]:
        """DB 행을 API 응답 모양으로 바꾼다."""
        return {"sequence_run_id": row.id, "sequence_id": row.sequence_key, "robot_id": row.robot_id,
                "state": row.state, "current_step": row.current_step, "steps": row.steps,
                "results": row.results, "current_command_id": row.current_command_id,
                "created_at": row.created_at, "updated_at": row.updated_at}

    def get(self, run_id: str) -> dict[str, Any]:
        """실행 하나. 먼저 명령 상태와 맞춘 뒤 읽는다. 없으면 KeyError."""
        self._reconcile(run_id)
        with self.sessions() as session:
            row = session.get(ArmSequenceRecord, run_id)
            if row is None:
                raise KeyError(run_id)
            return self._present(row)

    def latest(self) -> dict[str, Any] | None:
        """가장 최근에 만든 실행, 없으면 None."""
        with self.sessions() as session:
            row = session.scalar(select(ArmSequenceRecord).order_by(ArmSequenceRecord.created_at.desc()).limit(1))
            run_id = row.id if row else None
        if run_id:
            self._reconcile(run_id)
            return self.get(run_id)
        return None

    def _reconcile(self, run_id: str) -> None:
        """조회할 때 현재 명령 상태로 실행 상태를 다시 맞춘다. HTTP 응답을 잃어 다음 단계로 못 넘어간 경우도 여기서
        복구되고, 끝났지만 받아들일 수 없는 명령 상태면 실행을 멈춘다(on_worker_event와 같은 규칙)."""
        with self.sessions() as session:
            row = session.get(ArmSequenceRecord, run_id)
            command_id = row.current_command_id if row and row.state in {"QUEUED", "DISPATCHED", "RUNNING", "CANCELING"} else None
        if command_id:
            command = self.commands.get(command_id)
            if command["state"] in {"SUCCEEDED", "UNKNOWN", "FAILED", "REJECTED", "CANCELED", "EXPIRED"}:
                self.on_worker_event(command, False)
            elif command["state"] in {"DISPATCHED", "ACCEPTED", "RUNNING"}:
                with self.sessions.begin() as session:
                    row = session.scalar(select(ArmSequenceRecord).where(
                        ArmSequenceRecord.current_command_id == command_id,
                        ArmSequenceRecord.state.in_(["QUEUED", "DISPATCHED", "RUNNING"])))
                    if row:
                        row.state = "RUNNING" if command["state"] in {"ACCEPTED", "RUNNING"} else "DISPATCHED"
                        row.updated_at = utc_now_iso("microseconds")
            elif command["state"] == "QUEUED":
                with self.sessions.begin() as session:
                    row = session.scalar(select(ArmSequenceRecord).where(
                        ArmSequenceRecord.current_command_id == command_id,
                        ArmSequenceRecord.state.in_(["DISPATCHED", "RUNNING"])))
                    if row:
                        row.state = "QUEUED"
                        row.updated_at = utc_now_iso("microseconds")

    def start(self, sequence_id: str) -> dict[str, Any]:
        """정의 파일의 시퀀스를 시작한다(첫 단계 명령을 바로 만든다)."""
        with self._lock:
            definition = next((item for item in self.definitions() if item["sequence_id"] == sequence_id), None)
            if definition is None:
                raise ArmSequenceError(f"등록되지 않은 시퀀스입니다: {sequence_id}")
            return self._start_definition(definition)

    def start_custom(self, object_classes: object) -> dict[str, Any]:
        """운영자가 고른 순서대로, 서로 다른 등록 정책들을 시작한다.

        이름은 팔 지정 없이 찾고(resolve), 각 단계의 팔은 그 정책에 묶인 팔이다. 팔에 묶이지 않은 정책은 받지 않는다."""
        if not isinstance(object_classes, list) or not 1 <= len(object_classes) <= MAX_SEQUENCE_STEPS:
            raise ArmSequenceError(f"object_classes는 1~{MAX_SEQUENCE_STEPS}개여야 합니다")
        policies = self.commands.act_policies
        if policies is None:
            raise ArmSequenceError("ACT 정책 설정을 읽을 수 없습니다")
        steps: list[dict[str, Any]] = []
        seen: set[str] = set()
        for object_class in object_classes:
            if not isinstance(object_class, str) or not object_class.strip():
                raise ArmSequenceError("object_classes 항목은 물체 이름이어야 합니다")
            policy = policies.resolve(object_class.strip())
            if policy is None:
                raise ArmSequenceError(f"등록된 정책이 없습니다: {object_class}")
            if policy["policy_id"] in seen:
                raise ArmSequenceError(f"같은 물체를 두 번 선택했습니다: {policy['label']}")
            if policy["arm_id"] not in {"arm-01", "arm-02"}:
                raise ArmSequenceError(f"{policy['label']} 정책에 담당 팔이 지정되지 않았습니다")
            seen.add(policy["policy_id"])
            steps.append({"object_class": policy["object_class"], "label": policy["label"],
                          "policy_id": policy["policy_id"], "robot_id": policy["arm_id"]})
        labels = " → ".join(step["label"] for step in steps)
        definition = {"sequence_id": f"custom_{uuid.uuid4().hex[:12]}", "label": labels,
                      "robot_id": steps[0]["robot_id"], "steps": steps}
        with self._lock:
            return self._start_definition(definition)

    def _start_definition(self, definition: dict[str, Any]) -> dict[str, Any]:
        """진행 중인 시퀀스가 없으면 첫 단계 명령을 만들고 실행 기록을 저장한다. self._lock을 잡은 채로 부른다.

        명령을 먼저 만든다. 팔 명령 저장소가 거부하면(다른 명령 진행 중 등) 실행 기록이 남지 않는다.
        valid_for_s 600초는 저장소가 허용하는 최대값이다."""
        with self.sessions() as session:
            active = session.scalar(select(ArmSequenceRecord).where(
                ArmSequenceRecord.state.in_(["QUEUED", "DISPATCHED", "RUNNING", "CANCELING"])))
            if active:
                raise ArmSequenceError(f"이미 진행 중인 시퀀스가 있습니다: {active.id}")
        run_id = f"SEQ-{uuid.uuid4().hex[:12].upper()}"
        first = definition["steps"][0]
        command = self.commands.create({"robot_id": first["robot_id"], "operation": "PICK_WITH_POLICY",
            "valid_for_s": 600, "target": {"object_class": first["object_class"]},
            "place": {"slot_id": "CART_1"}, "source": {"kind": "OPERATOR_INPUT", "note": f"연속 실행 {definition['label']} 1단계"}})
        now = utc_now_iso("microseconds")
        with self.sessions.begin() as session:
            row = ArmSequenceRecord(id=run_id, sequence_key=definition["sequence_id"], robot_id=first["robot_id"],
                state="QUEUED", current_step=0, steps=definition["steps"], results=[],
                current_command_id=command["command_id"], created_at=now, updated_at=now)
            session.add(row)
        return self._present(row)

    def on_worker_event(self, command: dict[str, Any], applied: bool) -> None:
        """팔 명령의 워커 이벤트 뒤에 불린다(같은 이벤트의 재전송, _reconcile에서도). 이 명령이 현재 단계인 실행만 바꾼다.

        ACCEPTED면 RUNNING으로 표시한다. 끝 상태면 결과를 적고, 조건을 만족하면 다음 단계 명령을 만들거나 완료한다.
        applied는 지금 쓰지 않는다: 재전송이어도 같은 처리를 해야 진행을 복구할 수 있기 때문이다."""
        if command.get("state") == "ACCEPTED":
            with self._lock, self.sessions.begin() as session:
                row = session.scalar(select(ArmSequenceRecord).where(
                    ArmSequenceRecord.current_command_id == command["command_id"],
                    ArmSequenceRecord.state.in_(["QUEUED", "DISPATCHED"])))
                if row:
                    row.state = "RUNNING"
                    row.updated_at = utc_now_iso("microseconds")
            return
        # 명령 이벤트가 커밋된 뒤 HTTP 요청이 실패했어도, 같은 마지막 이벤트를 다시 받으면 시퀀스 진행을 복구할 수 있다.
        if command.get("state") not in {"SUCCEEDED", "UNKNOWN", "FAILED", "REJECTED", "CANCELED", "EXPIRED"}:
            return
        with self._lock:
            with self.sessions.begin() as session:
                row = session.scalar(select(ArmSequenceRecord).where(
                    ArmSequenceRecord.current_command_id == command["command_id"],
                    ArmSequenceRecord.state.in_(["QUEUED", "DISPATCHED", "RUNNING", "CANCELING"])))
                if row is None:
                    return
                # 정책 연속 실행은 home 복귀가 측정으로 확인됐을 때만 다음 단계로 간다.
                # 운영자가 명령을 취소했으면 사람이 팔을 확인해야 하므로 NEEDS_INTERVENTION이다.
                state, result = command["state"], command.get("result") or {}
                if state == "CANCELED":
                    row.state = "NEEDS_INTERVENTION"
                    row.current_command_id = None
                    row.updated_at = utc_now_iso("microseconds")
                    return
                # 시퀀스 취소 중에 명령이 (취소가 아닌 상태로) 끝남: 결과만 적고 멈춘다.
                if row.state == "CANCELING":
                    row.state = "NEEDS_INTERVENTION"
                    row.results = [*row.results, {"step": row.current_step,
                                                  "object_class": row.steps[row.current_step]["object_class"],
                                                  "command_id": command["command_id"], "state": state,
                                                  "reason_code": command.get("reason_code"),
                                                  "home_confirmed": result.get("pose_reached") is True,
                                                  "placement_confirmed": result.get("place_confirmed") is True}]
                    row.current_command_id = None
                    row.updated_at = utc_now_iso("microseconds")
                    return
                allowed_unknown = state == "UNKNOWN" and command.get("reason_code") == "PLACE_NOT_VERIFIED"
                if (state != "SUCCEEDED" and not allowed_unknown) or result.get("pose_reached") is not True:
                    row.state = "NEEDS_INTERVENTION"
                    row.results = [*row.results, {"step": row.current_step, "object_class": row.steps[row.current_step]["object_class"],
                                                  "command_id": command["command_id"], "state": state,
                                                  "reason_code": command.get("reason_code"), "home_confirmed": False}]
                    row.updated_at = utc_now_iso("microseconds")
                    return
                current_step = row.current_step
                row.results = [*row.results, {"step": current_step, "object_class": row.steps[current_step]["object_class"],
                                              "command_id": command["command_id"], "state": state,
                                              "reason_code": command.get("reason_code"), "home_confirmed": True,
                                              "placement_confirmed": result.get("place_confirmed") is True}]
                sequence_label = row.sequence_key
                previous_command_id = command["command_id"]
                finished_sequence = current_step + 1 >= len(row.steps)
                if finished_sequence:
                    row.state = "COMPLETED"
                    row.current_command_id = None
                    row.updated_at = utc_now_iso("microseconds")
                else:
                    next_step = row.steps[current_step + 1]
                    robot_id = next_step["robot_id"]
                    run_id = row.id
            # 아래는 DB 트랜잭션 밖이다(팔 명령 저장소가 자기 트랜잭션과 잠금을 쓴다).
            # home 복귀가 측정됐으면, 확인되지 않은 명령(UNKNOWN)을 풀어 줘서 다음 명령을 막지 않게 한다.
            # 이것은 팔 자세만 확인한 것이고, PLACE_NOT_VERIFIED를 성공으로 바꾸지 않는다.
            if command["state"] == "UNKNOWN" and not command.get("operator_cleared_at"):
                decision = "시퀀스 완료" if finished_sequence else "다음 정책 단계 진행"
                self.commands.clear_unknown(
                    previous_command_id,
                    f"시퀀스 {sequence_label}: home 복귀 확인, {decision}. 적재 성공은 미확인.",
                )
            if finished_sequence:
                return
            next_command = self.commands.create({"robot_id": robot_id, "operation": "PICK_WITH_POLICY", "valid_for_s": 600,
                "target": {"object_class": next_step["object_class"]}, "place": {"slot_id": "CART_1"},
                "source": {"kind": "OPERATOR_INPUT", "note": f"시퀀스 {sequence_label} {current_step + 2}단계"}})
            # 명령을 만드는 사이에 취소됐거나 다른 경로가 이미 진행시켰으면 이 결과는 버린다(그 명령은 큐에 남는다).
            with self.sessions.begin() as session:
                row = session.get(ArmSequenceRecord, run_id)
                if row and row.state == "RUNNING" and row.current_command_id == previous_command_id:
                    row.current_step += 1
                    row.current_command_id = next_command["command_id"]
                    row.updated_at = utc_now_iso("microseconds")

    def cancel(self, run_id: str) -> dict[str, Any]:
        """진행 중인 실행을 취소한다. 현재 명령이 바로 취소되면(QUEUED였음) CANCELED, 실행 중이라 워커의 취소 보고를
        기다려야 하면 CANCELING이다. 없으면 KeyError, 진행 중이 아니면 ArmSequenceError."""
        with self._lock:
            with self.sessions() as session:
                row = session.get(ArmSequenceRecord, run_id)
                if row is None:
                    raise KeyError(run_id)
                if row.state not in {"QUEUED", "DISPATCHED", "RUNNING"}:
                    raise ArmSequenceError("진행 중인 시퀀스가 아닙니다")
                command_id = row.current_command_id
            canceled_command = self.commands.cancel(command_id) if command_id else None
            with self.sessions.begin() as session:
                row = session.get(ArmSequenceRecord, run_id)
                row.state = "CANCELING" if command_id and canceled_command["state"] != "CANCELED" else "CANCELED"
                row.updated_at = utc_now_iso("microseconds")
                return self._present(row)
