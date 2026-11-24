"""Persistent operator initiated sequences of registered ACT policies."""

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
MAX_SEQUENCE_STEPS = 4


class ArmSequenceError(ValueError):
    pass


class ArmSequenceSupervisor:
    def __init__(self, sessions: sessionmaker, commands: ArmCommandStore, path: Path = DEFAULT_PATH) -> None:
        self.sessions = sessions
        self.commands = commands
        self.path = path
        self._lock = threading.RLock()

    def definitions(self) -> list[dict[str, Any]]:
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
        return {"sequence_run_id": row.id, "sequence_id": row.sequence_key, "robot_id": row.robot_id,
                "state": row.state, "current_step": row.current_step, "steps": row.steps,
                "results": row.results, "current_command_id": row.current_command_id,
                "created_at": row.created_at, "updated_at": row.updated_at}

    def get(self, run_id: str) -> dict[str, Any]:
        self._reconcile(run_id)
        with self.sessions() as session:
            row = session.get(ArmSequenceRecord, run_id)
            if row is None:
                raise KeyError(run_id)
            return self._present(row)

    def latest(self) -> dict[str, Any] | None:
        with self.sessions() as session:
            row = session.scalar(select(ArmSequenceRecord).order_by(ArmSequenceRecord.created_at.desc()).limit(1))
            run_id = row.id if row else None
        if run_id:
            self._reconcile(run_id)
            return self.get(run_id)
        return None

    def _reconcile(self, run_id: str) -> None:
        """A refresh can recover advancement after a lost HTTP response; unresolved command states stop the run."""
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
        with self._lock:
            definition = next((item for item in self.definitions() if item["sequence_id"] == sequence_id), None)
            if definition is None:
                raise ArmSequenceError(f"등록되지 않은 시퀀스입니다: {sequence_id}")
            return self._start_definition(definition)

    def start_custom(self, object_classes: object) -> dict[str, Any]:
        """Start a user-ordered selection of distinct, registered object policies."""
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
        if command.get("state") == "ACCEPTED":
            with self._lock, self.sessions.begin() as session:
                row = session.scalar(select(ArmSequenceRecord).where(
                    ArmSequenceRecord.current_command_id == command["command_id"],
                    ArmSequenceRecord.state.in_(["QUEUED", "DISPATCHED"])))
                if row:
                    row.state = "RUNNING"
                    row.updated_at = utc_now_iso("microseconds")
            return
        # A retried final event can recover sequence advancement if the HTTP request failed
        # after the command event was committed.
        if command.get("state") not in {"SUCCEEDED", "UNKNOWN", "FAILED", "REJECTED", "CANCELED", "EXPIRED"}:
            return
        with self._lock:
            with self.sessions.begin() as session:
                row = session.scalar(select(ArmSequenceRecord).where(
                    ArmSequenceRecord.current_command_id == command["command_id"],
                    ArmSequenceRecord.state.in_(["QUEUED", "DISPATCHED", "RUNNING", "CANCELING"])))
                if row is None:
                    return
                # A policy sequence only continues after an explicit measured home return.
                state, result = command["state"], command.get("result") or {}
                if state == "CANCELED":
                    row.state = "NEEDS_INTERVENTION"
                    row.current_command_id = None
                    row.updated_at = utc_now_iso("microseconds")
                    return
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
            # Once home return is measured, release an unverified command so it cannot block a later run.
            # This acknowledges only the arm pose; it does not turn PLACE_NOT_VERIFIED into success.
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
            with self.sessions.begin() as session:
                row = session.get(ArmSequenceRecord, run_id)
                if row and row.state == "RUNNING" and row.current_command_id == previous_command_id:
                    row.current_step += 1
                    row.current_command_id = next_command["command_id"]
                    row.updated_at = utc_now_iso("microseconds")

    def cancel(self, run_id: str) -> dict[str, Any]:
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
