"""작업 실행: 저장된 작업·단계 상태 기계 위에서 단계를 차례로 돌린다.

주문(order_items가 있는 작업)은 자기 단계를 orders/flow.py의 OrderFlow로 돌린다. 이 파일이 직접 단계를 돌리는 것은
이전 형식 작업(이동 + 팔 7단계)이다.

이전 형식 작업의 단계는 모두 모의(`source: mock`)다. 예외는 CONTROL_STATION_TASK_REAL_ARM=1로 켠 EXECUTE_ARM_TASK
하나다. 작업의 팔 일(예: "당근 집기")이 가리키는 정책으로 PICK_WITH_POLICY 팔 명령을 하나 넣고 팔 워커의 결과를
기다린다. 명령은 정면 카메라가 물체를 정확히 한 칸에서만 볼 때만 만든다(SR-13). 그 관측이 명령의 CAMERA 근거로
함께 저장되고, 관측이 없으면 팔 명령을 만들기 전에 단계가 실패한다.

워커는 물체를 잡아서 놓았는지 볼 수 없다. 그래서 가장 좋은 결과가 "원위치로 돌아옴, 놓은 것은 확인 안 됨"이다.
VERIFY_ARM_RESULT는 판단을 VERIFY_LOAD에 넘긴다고 기록하고, 모의가 아닌 적재 관측이 확인해 주지 않으면 팔 명령은
운영자가 볼 수 있게 UNKNOWN으로 남는다.

적재 확인 (VERIFY_LOAD, 아래 첫 부분)
  팔 단계 뒤에, 작업이 요구한 것이 모두 핑키 위에 올라갔는지 본다(docs/spec/data-flow.md의 B 구역 "목표 수량 적재
  성공 확인"). 관측기가 핑키 위에서 본 개수를 알려 주면 judge_load가 기대 개수와 비교한다. 팔이 놓았다고 보고한
  것은 별개의 결과이고, 핑키 위에서 본 것으로 치지 않는다. 관측기는 아직 MockLoadObserver뿐이다. 카메라를 열지
  않고 설정한 개수를 알려 주므로 관측 결과에 simulated가 붙는다.

스레드와 잠금
  start()가 작업마다 작업 스레드 하나(_run)를 띄우고 단계를 차례로 돌린다. 시작·취소·실패 마무리는 `_lock`(RLock)으로
  줄 세워, 취소 요청과 실패 처리가 겹쳐도 먼저 잠금을 잡은 쪽의 끝 상태 하나만 기록된다. 저장소의 CAS가 그 밖의
  경쟁을 막는다. 작업이 끝나면 주문 흐름에 release를 알려 적재 자리를 비우거나(멈춘 주문은) 쥔 채로 둔다.
"""

from __future__ import annotations

import os
import threading
import time
import uuid
from typing import Any, Callable

from control_station.arms.commands import IN_FLIGHT, ArmCommandConflict, ArmCommandError, ArmCommandStore
from control_station.cameras.shelf_slots import SlotSelectionError
from control_station.orders.flow import ORDER_STEP_TYPES, OrderFlow, OrderFlowError, StepContext
from control_station.storage.models import StepStatus, TaskStatus
from control_station.storage.store import (
    DEFAULT_TASK_STEPS,
    ROBOT_ID_BY_KIND,
    InvalidStateTransition,
    StateTransitionConflict,
    TaskNotFoundError,
    TaskStore,
)

# ──── 적재 확인 (VERIFY_LOAD) ─────────────────────────────────────────────────────────────────────────────────

# 흐름을 검증하는 동안 작업 하나에 물체 하나(2026-11-03 결정). 주문은 여러 물체(예: 3개)를 담을 수 있다. 그런
# 주문은 놓을 때마다 확인(앞뒤 개수가 정확히 하나 차이)하고, 핑키가 떠나기 전에 이 총량 확인을 하는 것으로 계획했다
# (작업이 수량을 갖게 되면). 지금 그 일은 주문 흐름(flow.py의 VERIFY_TRANSFER, VERIFY_LOAD_TOTAL)이 한다.
EXPECTED_LOAD_COUNT = 1
LOAD_OBSERVATION_SCHEMA_VERSION = 1

# {"source", "observed_count": int | None, "reason", ...}를 돌려준다. 관측을 시작조차 못 하면 error_code 속성이
# 있는 예외를 올린다.
LoadObserver = Callable[[], dict[str, Any]]

# judge_load 결과 → 작업 오류 코드.
LOAD_ERROR_CODES = {
    "INCOMPLETE": "LOAD_INCOMPLETE",
    "EXCESS": "LOAD_EXCESS",
    "NOT_VERIFIED": "LOAD_NOT_VERIFIED",
}


def judge_load(observation: dict[str, Any], expected_count: int) -> str:
    """관측 개수가 기대 개수와 같을 때만 LOADED. 적으면 INCOMPLETE, 많으면 EXCESS, 개수가 없거나 정수가 아니면
    NOT_VERIFIED(bool은 int로 치지 않는다)."""
    count = observation.get("observed_count")
    if not isinstance(count, int) or isinstance(count, bool) or count < 0:
        return "NOT_VERIFIED"
    if count == expected_count:
        return "LOADED"
    return "INCOMPLETE" if count < expected_count else "EXCESS"


class MockLoadObserver:
    """아무것도 보지 않고 정해 둔 개수를 보고한다. 모의 모드에서 VERIFY_LOAD 흐름을 시험하려는 것이다."""

    def __init__(self, observed_count: int) -> None:
        self.observed_count = observed_count

    def __call__(self) -> dict[str, Any]:
        return {
            "observation_schema_version": LOAD_OBSERVATION_SCHEMA_VERSION,
            "source": "mock",
            "simulated": True,
            "observed_count": self.observed_count,
            "reason": "모의 관측: 카메라를 열지 않고 CONTROL_STATION_MOCK_LOAD_OBSERVED 값을 보고함",
            "usable_for_robot_control": False,
        }


def mock_load_observer_from_env() -> MockLoadObserver | None:
    """CONTROL_STATION_MOCK_LOAD_OBSERVED가 있으면 그 개수를 보고하는 모의 관측기, 없으면 None. 음수·숫자 아님은 ValueError."""
    raw = os.environ.get("CONTROL_STATION_MOCK_LOAD_OBSERVED", "").strip()
    if not raw:
        return None
    try:
        count = int(raw)
    except ValueError:
        count = -1
    if count < 0:
        raise ValueError("CONTROL_STATION_MOCK_LOAD_OBSERVED must be a non-negative integer")
    return MockLoadObserver(count)


# ──── 작업 실행 ───────────────────────────────────────────────────────────────────────────────────────────────

class MockScenarioFailure(RuntimeError):
    """모의 전용으로 일부러 설정한 단계 실패(CONTROL_STATION_MOCK_FAIL_STEP)."""


# 주차 관측 하나를 돌려준다(interface-contracts의 "Pinky 주차 정렬 관측"). 관측을 시작하지 못하면 error_code 속성이
# 있는 예외를 올린다.
ArrivalCheck = Callable[[], dict[str, Any]]
# 물체 클래스 하나에 대한 정면 카메라 근거: cameras/shelf_slots.py의 select_slot_for_class가 남긴 선택 기록.
# 카메라가 칸 하나를 집어내지 못하면 SlotSelectionError(error_code, details)를 올린다.
TargetEvidence = Callable[[str], dict[str, Any]]


class ArrivalNotConfirmed(RuntimeError):
    """이동 뒤 주차 관측에서 핑키가 등록 자리에 있지 않았다."""

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


class LoadNotConfirmed(RuntimeError):
    """팔 단계 뒤 적재 관측에서 기대한 물건이 카트(핑키) 위에 보이지 않았다."""

    def __init__(self, error_code: str, message: str, source: str) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.source = source


class ArmTaskNotConfirmed(RuntimeError):
    """EXECUTE_ARM_TASK의 팔 명령을 보낼 수 없었거나, 팔이 원위치로 돌아와 끝나지 않았다."""

    def __init__(self, error_code: str, message: str, source: str = "arm_worker") -> None:
        super().__init__(message)
        self.error_code = error_code
        # 어느 쪽이 확인하지 못했는지: 정면 카메라(명령을 만들지 않음) 또는 팔 워커 / 관제.
        self.source = source


def policy_for_arm_job(registry: Any, arm_job: str) -> dict[str, Any] | None:
    """팔 일 글자에 물체 이름이 들어 있는 학습 정책("당근 집기" → carrot 정책). 정확히 일치하는 것을 먼저 본다."""
    if registry is None:
        return None
    exact = registry.resolve(arm_job)
    if exact is not None:
        return exact
    text = arm_job.casefold()
    for policy in registry.policies.values():
        if any(name.casefold() in text for name in {policy["object_class"], policy["label"], *policy["aliases"]}):
            return policy
    return None


def policy_for_task(registry: Any, task: dict[str, Any]) -> dict[str, Any] | None:
    """주문한 물체의 정책: pick_request.category("carrot" 또는 "당근")를 먼저, 그다음 팔 일 글자로 찾는다."""
    if registry is None:
        return None
    category = (task.get("pick_request") or {}).get("category")
    if isinstance(category, str) and category.strip():
        policy = registry.resolve(category.strip())
        if policy is not None:
            return policy
    return policy_for_arm_job(registry, str(task.get("arm_job") or ""))


class TaskExecutionConflict(RuntimeError):
    """작업을 시작·취소할 수 없다. error_code는 API가 돌려줄 코드, task는 그때의 작업(있으면)."""

    def __init__(
        self,
        message: str,
        task: dict[str, Any] | None = None,
        *,
        error_code: str = "TASK_EXECUTION_CONFLICT",
    ) -> None:
        super().__init__(message)
        self.task = task
        self.error_code = error_code


class MockTaskOrchestrator:
    """저장된 작업 흐름을 장치나 ROS 연결을 직접 열지 않고 실행한다.

    이전 형식 작업은 단계마다 모의 결과를 남기고, 주문은 단계를 OrderFlow에 맡긴다(실제 장치는 OrderFlow 안의
    OrderDevices가 다룬다). mode가 "mock"이 아니면 시작·취소를 거절한다."""

    def __init__(
        self,
        task_store: TaskStore,
        mode: str = "mock",
        arrival_check: ArrivalCheck | None = None,
        load_check: LoadObserver | None = None,
        arm_commands: ArmCommandStore | None = None,
        target_evidence: TargetEvidence | None = None,
        order_flow: OrderFlow | None = None,
    ) -> None:
        self.task_store = task_store
        # 주문(order_items가 있는 작업)의 단계를 돌린다. None이면 주문 단계에서 ORDER_FLOW_NOT_CONFIGURED로 멈춘다.
        self.order_flow = order_flow
        self.mode = mode
        # 있으면 EXECUTE_ARM_TASK가 모의 성공 대신 실제 PICK_WITH_POLICY 팔 명령을 보낸다.
        self.arm_commands = arm_commands
        # 그 실제 팔 명령에 꼭 필요하다: 정면 카메라 근거가 없으면 명령도 없다(SR-13).
        self.target_evidence = target_evidence
        self._arm_results: dict[str, dict[str, Any]] = {}  # 작업 id → EXECUTE_ARM_TASK 결과(VERIFY_LOAD가 닫는다)
        # 있으면 VERIFY_ARRIVAL은 주차 관측이 ARRIVED일 때만 성공한다.
        self.arrival_check = arrival_check
        # 있으면 VERIFY_LOAD는 카트 위 관측 개수가 기대 개수와 같을 때만 성공한다.
        self.load_check = load_check
        self.step_delay = self._read_step_delay()
        self.fail_step = self._read_failure_step() if mode == "mock" else None
        self._lock = threading.RLock()
        self._cancel_events: dict[str, threading.Event] = {}  # 작업 id → 그 작업 스레드의 취소 신호
        self._threads: dict[str, threading.Thread] = {}  # 이 프로세스가 돌리는 작업 스레드

    @staticmethod
    def _read_step_delay() -> float:
        """모의 단계 하나의 시간(초): CONTROL_STATION_MOCK_STEP_DELAY, 0.05~2.0으로 자른다. 잘못된 값이면 0.25."""
        raw = os.environ.get("CONTROL_STATION_MOCK_STEP_DELAY", "0.25")
        try:
            return min(2.0, max(0.05, float(raw)))
        except ValueError:
            return 0.25

    @staticmethod
    def _read_failure_step() -> str | None:
        """CONTROL_STATION_MOCK_FAIL_STEP: 일부러 실패시킬 단계 종류. 없는 단계 이름이면 시작부터 ValueError."""
        raw = os.environ.get("CONTROL_STATION_MOCK_FAIL_STEP", "").strip()
        if not raw:
            return None
        valid_steps = {step_type for step_type, _ in DEFAULT_TASK_STEPS} | ORDER_STEP_TYPES
        if raw not in valid_steps:
            raise ValueError(
                "CONTROL_STATION_MOCK_FAIL_STEP must match a configured task step type"
            )
        return raw

    def start(self, task_id: str) -> tuple[dict[str, Any], bool]:
        """작업을 선점하고 작업 스레드를 띄운다: (작업, 새로 시작했는지).

        이미 이 프로세스에서 돌고 있으면 그대로 돌려준다(멱등). 저장소에는 진행 중인데 이 프로세스에 스레드가 없으면
        (예: 서버가 다시 켜짐) 주인이 불분명하니 TASK_WORKER_OWNERSHIP_UNCERTAIN으로 운영자에게 넘긴다."""
        if self.mode != "mock":
            raise TaskExecutionConflict(
                "작업 실행은 모의 모드에서만 제공됩니다. 장치 명령은 보내지 않았습니다.",
                error_code="EXECUTION_MODE_UNSUPPORTED",
            )
        with self._lock:
            try:
                task, claimed = self.task_store.claim_task(task_id)
            except TaskNotFoundError:
                raise
            except (InvalidStateTransition, StateTransitionConflict) as exc:
                raise TaskExecutionConflict(
                    str(exc), error_code="TASK_STATE_CONFLICT"
                ) from exc
            if not claimed:
                if task["status"] in {TaskStatus.VALIDATING.value, TaskStatus.RUNNING.value}:
                    # 시작을 반복해도 같은 결과이고 두 번째 작업 스레드는 절대 만들지 않는다.
                    if task_id in self._threads:
                        return task, False
                    raise TaskExecutionConflict(
                        "Task is active in storage but has no in-process mock worker; operator review is required",
                        task,
                        error_code="TASK_WORKER_OWNERSHIP_UNCERTAIN",
                    )
                raise TaskExecutionConflict(
                    f"Task {task_id} is already active or requires operator review",
                    task,
                    error_code="TASK_NOT_STARTABLE",
                )
            cancel_event = threading.Event()
            worker = threading.Thread(
                target=self._run,
                args=(task_id, cancel_event),
                name=f"mock-task-{task_id}",
                daemon=True,
            )
            self._cancel_events[task_id] = cancel_event
            self._threads[task_id] = worker
            try:
                worker.start()
            except RuntimeError as exc:
                # 스레드를 못 띄웠다: 첫 단계를 실패로, 나머지를 실행 안 함으로 닫고 작업을 FAILED로 끝낸다.
                self._cancel_events.pop(task_id, None)
                self._threads.pop(task_id, None)
                if task["steps"]:
                    self.task_store.transition_step(
                        task_id,
                        task["steps"][0]["id"],
                        StepStatus.FAILED,
                        expected_status=StepStatus.PENDING,
                        error_code="MOCK_WORKER_START_FAILED",
                        error_message=str(exc),
                        event_payload={
                            "source": "mock",
                            "event": "MOCK_WORKER_START_FAILED",
                            "task_id": task_id,
                            "step_id": task["steps"][0]["id"],
                        },
                    )
                self._mark_pending_steps_not_executed(task_id, reason="FAILURE")
                failed = self.task_store.transition_task(
                    task_id,
                    TaskStatus.FAILED,
                    expected_status=TaskStatus.VALIDATING,
                    payload={
                        "source": "mock",
                        "event": "MOCK_WORKER_START_FAILED",
                        "error": str(exc),
                    },
                )
                raise TaskExecutionConflict(
                    "Mock worker could not start",
                    failed,
                    error_code="MOCK_WORKER_START_FAILED",
                ) from exc
            return self.task_store.get_task(task_id) or task, True

    def cancel(self, task_id: str) -> tuple[dict[str, Any], bool]:
        """작업을 취소한다: (작업, 이번에 바꿨는지). 시작 전이면 바로 CANCELED, 진행 중이면 CANCELING으로 바꾸고
        작업 스레드에 취소 신호를 보낸다(스레드가 지금 단계를 멈추고 CANCELED로 마무리한다)."""
        if self.mode != "mock":
            raise TaskExecutionConflict(
                "작업 취소는 모의 모드에서만 제공됩니다. 장치 명령은 보내지 않았습니다.",
                error_code="EXECUTION_MODE_UNSUPPORTED",
            )
        with self._lock:
            task = self.task_store.get_task(task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            status = TaskStatus(task["status"])
            if status == TaskStatus.PENDING:
                try:
                    updated = self.task_store.cancel_pending_task(task_id)
                    return updated, True
                except StateTransitionConflict:
                    latest = self.task_store.get_task(task_id)
                    raise TaskExecutionConflict(
                        "Task state changed; reload before retrying",
                        latest,
                        error_code="TASK_STATE_CONFLICT",
                    )
            if status == TaskStatus.CANCELED:
                return task, False
            if status == TaskStatus.CANCELING:
                if task_id in self._threads:
                    return task, False
                raise TaskExecutionConflict(
                    "Task is canceling in storage but has no in-process mock worker; operator review is required",
                    task,
                    error_code="TASK_WORKER_OWNERSHIP_UNCERTAIN",
                )
            if status not in {TaskStatus.VALIDATING, TaskStatus.RUNNING}:
                raise TaskExecutionConflict(
                    f"Task cannot be canceled from {status.value}",
                    task,
                    error_code="TASK_NOT_CANCELLABLE",
                )
            event = self._cancel_events.get(task_id)
            if event is None:
                raise TaskExecutionConflict(
                    "No in-process mock worker owns this active task; operator review is required",
                    task,
                    error_code="TASK_WORKER_OWNERSHIP_UNCERTAIN",
                )
            try:
                updated = self.task_store.transition_task(
                    task_id,
                    TaskStatus.CANCELING,
                    expected_status=status,
                    payload={"source": "mock", "event": "CANCEL_REQUESTED"},
                )
            except StateTransitionConflict as exc:
                latest = self.task_store.get_task(task_id)
                if latest and latest["status"] in {
                    TaskStatus.CANCELING.value,
                    TaskStatus.CANCELED.value,
                }:
                    return latest, False
                raise TaskExecutionConflict(
                    "Task state changed; reload before retrying",
                    latest,
                    error_code="TASK_STATE_CONFLICT",
                ) from exc
            event.set()
            return updated, True

    def local_worker_state(self, task_id: str) -> str:
        """이 프로세스가 그 작업의 스레드를 갖고 있는지 알려 준다. 저장된 작업 상태를 추정하거나 바꾸지 않는다."""
        with self._lock:
            task = self.task_store.get_task(task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task["status"] not in {
                TaskStatus.VALIDATING.value,
                TaskStatus.RUNNING.value,
                TaskStatus.CANCELING.value,
            }:
                return "NOT_ACTIVE"
            if self.mode != "mock":
                return "MODE_NOT_MANAGED"
            worker = self._threads.get(task_id)
            return (
                "LOCAL_WORKER_ACTIVE"
                if worker is not None and worker.is_alive()
                else "NO_LOCAL_WORKER"
            )

    def close(self) -> None:
        """서버 종료: 진행 중인 작업을 CANCELING으로 바꾸고 취소 신호를 보낸 뒤, 스레드마다 최대 3초 기다린다."""
        with self._lock:
            workers = list(self._threads.values())
            for task_id, event in self._cancel_events.items():
                task = self.task_store.get_task(task_id)
                if task and task["status"] in {
                    TaskStatus.VALIDATING.value,
                    TaskStatus.RUNNING.value,
                }:
                    try:
                        self.task_store.transition_task(
                            task_id,
                            TaskStatus.CANCELING,
                            expected_status=TaskStatus(task["status"]),
                            payload={"source": "mock", "event": "SERVER_SHUTDOWN"},
                        )
                    except StateTransitionConflict:
                        pass
                event.set()
        for worker in workers:
            worker.join(timeout=3.0)

    def _run(self, task_id: str, cancel_event: threading.Event) -> None:
        """작업 스레드 본체: 첫 단계(VALIDATING) → RUNNING → 나머지 단계 → SUCCEEDED. 예외는 _finish_error가,
        취소는 _finish_cancel이 마무리한다. 끝나면 주문 흐름에 release를 알린다(멈춘 주문은 핑키를 쥔 채로)."""
        active_step_id: str | None = None
        try:
            task = self.task_store.get_task(task_id)
            if task is None:
                return
            steps = task["steps"]
            if not steps:
                raise RuntimeError("Task has no persisted steps")

            # 준비 확인(첫 단계)은 모의임을 밝히고 모의 근거로 표시한다.
            active_step_id = steps[0]["id"]
            if not self._run_step(task_id, steps[0], cancel_event):
                self._finish_cancel(task_id, active_step_id)
                return
            current = self.task_store.get_task(task_id)
            if current is None or current["status"] == TaskStatus.CANCELING.value:
                self._finish_cancel(task_id, active_step_id)
                return
            self.task_store.transition_task(
                task_id,
                TaskStatus.RUNNING,
                expected_status=TaskStatus.VALIDATING,
                payload={"source": "mock", "event": "MOCK_VALIDATION_PASSED"},
            )

            for step in steps[1:]:
                active_step_id = step["id"]
                if cancel_event.is_set() or not self._run_step(task_id, step, cancel_event):
                    self._finish_cancel(task_id, active_step_id)
                    return

            current = self.task_store.get_task(task_id)
            if current is None or current["status"] == TaskStatus.CANCELING.value:
                self._finish_cancel(task_id, active_step_id)
                return
            self.task_store.transition_task(
                task_id,
                TaskStatus.SUCCEEDED,
                expected_status=TaskStatus.RUNNING,
                payload={"source": "mock", "event": "MOCK_TASK_COMPLETED"},
            )
        except Exception as exc:
            self._finish_error(task_id, active_step_id, exc)
        finally:
            with self._lock:
                self._threads.pop(task_id, None)
                self._cancel_events.pop(task_id, None)
                self._arm_results.pop(task_id, None)
            if self.order_flow is not None:
                final = self.task_store.get_task(task_id) or {}
                stopped = final.get("status") == TaskStatus.FAILED.value and final.get("pinky_id")
                self.order_flow.release(task_id, stopped_pinky=final.get("pinky_id") if stopped else None)

    def _run_step(
        self,
        task_id: str,
        step: dict[str, Any],
        cancel_event: threading.Event,
    ) -> bool:
        """단계 하나를 돌린다. 성공하면 True, 취소되면 False, 실패는 예외로 올린다.

        단계를 COMMAND_SENT → ACCEPTED → RUNNING으로 바꾼 뒤(잠금 안에서, 취소와 겹치지 않게) 단계 종류에 따라
        주문 흐름 / 실제 팔 명령 / 모의 진행 중 하나로 실행하고 SUCCEEDED로 닫는다. 첫 단계는 작업이 VALIDATING,
        나머지는 RUNNING이어야 한다."""
        step_id = step["id"]
        expected_task_status = (
            TaskStatus.VALIDATING
            if step.get("sequence") == 1
            else TaskStatus.RUNNING
        )
        command_id = f"CMD-{uuid.uuid4().hex[:12].upper()}"
        real_arm = self.arm_commands is not None and step.get("step_type") in {"EXECUTE_ARM_TASK", "VERIFY_ARM_RESULT"}
        base = {
            "source": "arm_worker" if real_arm else "mock",
            "robot_id": self._mock_robot_id(step),
            "task_id": task_id,
            "step_id": step_id,
            "command_id": command_id,
        }
        with self._lock:
            if cancel_event.is_set():
                return False
            task = self.task_store.get_task(task_id)
            if task is None or task["status"] == TaskStatus.CANCELING.value:
                return False
            if task["status"] != expected_task_status.value:
                raise StateTransitionConflict(
                    "Mock step cannot start unless the task is in its expected active state"
                )
            self.task_store.transition_step(
                task_id,
                step_id,
                StepStatus.COMMAND_SENT,
                expected_status=StepStatus.PENDING,
                command_id=command_id,
                event_payload={**base, "event": "COMMAND_SENT"},
            )
            self.task_store.transition_step(
                task_id,
                step_id,
                StepStatus.ACCEPTED,
                expected_status=StepStatus.COMMAND_SENT,
                event_payload={**base, "event": "COMMAND_ACCEPTED"},
            )
            self.task_store.transition_step(
                task_id,
                step_id,
                StepStatus.RUNNING,
                expected_status=StepStatus.ACCEPTED,
                event_payload={**base, "event": "EXECUTION_STARTED"},
            )
        if task.get("order_items") and step.get("step_type") in ORDER_STEP_TYPES:
            return self._run_order_step(task_id, step_id, command_id, base, expected_task_status, cancel_event)
        if real_arm and step.get("step_type") == "EXECUTE_ARM_TASK":
            arm_result = self._execute_arm_policy(task_id, step_id, command_id, base, cancel_event)
            if arm_result is None:
                return False
            self._arm_results[task_id] = arm_result
            self.task_store.transition_step(
                task_id, step_id, StepStatus.SUCCEEDED, expected_status=StepStatus.RUNNING,
                result_payload=arm_result, event_payload={**base, "event": "ARM_COMMAND_FINISHED"},
            )
            return True
        if real_arm and step.get("step_type") == "VERIFY_ARM_RESULT":
            arm_result = self._arm_results.get(task_id) or {}
            self.task_store.transition_step(
                task_id, step_id, StepStatus.SUCCEEDED, expected_status=StepStatus.RUNNING,
                result_payload={
                    "source": "control_station",
                    "outcome": "ARM_HOME_PLACEMENT_LEFT_TO_VERIFY_LOAD",
                    "arm_command_id": arm_result.get("arm_command_id"),
                    "message": "팔은 home으로 돌아왔다고 보고했지만 집기·적재는 확인하지 않았습니다. 적재 확인 단계가 판단합니다.",
                },
                event_payload={**base, "event": "ARM_RESULT_RECORDED"},
            )
            return True
        progress_delay = self.step_delay / 2
        if cancel_event.wait(progress_delay):
            return False
        self.task_store.record_step_event(
            task_id,
            step_id,
            command_id,
            "PROGRESS",
            {
                **base,
                "event": "PROGRESS",
                "progress": 50,
                "message": "모의 단계 진행 중",
            },
            expected_task_status=expected_task_status,
        )
        if cancel_event.wait(self.step_delay - progress_delay):
            return False
        if step.get("step_type") == self.fail_step:
            raise MockScenarioFailure(
                f"Configured mock scenario failure at step {self.fail_step}"
            )
        result_payload: dict[str, Any] = {"source": "mock", "outcome": "SIMULATED_SUCCESS"}
        if step.get("step_type") == "VERIFY_ARRIVAL" and self.arrival_check is not None:
            result_payload = self._verify_arrival(task_id, step_id, command_id, base)
        if step.get("step_type") == "VERIFY_LOAD" and self.load_check is not None:
            result_payload = self._verify_load(task_id, step_id, command_id, base)
            self._close_arm_command_after_load(task_id, result_payload)
        self.task_store.transition_step(
            task_id,
            step_id,
            StepStatus.SUCCEEDED,
            expected_status=StepStatus.RUNNING,
            result_payload=result_payload,
            event_payload={**base, "event": "MOCK_STEP_COMPLETED"},
        )
        return True

    def _run_order_step(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        base: dict[str, Any],
        expected_task_status: TaskStatus,
        cancel_event: threading.Event,
    ) -> bool:
        """주문 단계를 OrderFlow.run_step으로 돌리고 결과를 단계 결과로 저장한다. 취소면 False."""
        if self.order_flow is None:
            raise OrderFlowError("ORDER_FLOW_NOT_CONFIGURED", "주문 흐름이 설정되지 않아 주문을 진행할 수 없습니다")
        # 다시 읽는다: PLAN_ARM_SEQUENCE가 작업 시작 뒤에 뒤 단계의 팔을 채우기 때문이다.
        task = self.task_store.get_task(task_id) or {}
        step = next(item for item in task["steps"] if item["id"] == step_id)
        source = {"VERIFY_TRANSFER": "cameras", "VERIFY_LOAD_TOTAL": "cameras", "CHECK_SHELF_STOCK": "shelf_camera",
                  "WAIT_PINKY_ARRIVAL": "pinky_fleet", "CONFIRM_DOCKING": "pinky_fleet",
                  "NOTIFY_PINKY_RECEIVED": "pinky_fleet", "PICK_ITEM": "arm"}.get(step["step_type"], "control_station")

        def record(event_type: str, payload: dict[str, Any]) -> None:
            try:
                self.task_store.record_step_event(
                    task_id, step_id, command_id, event_type,
                    {**base, "source": source, "event": event_type, **payload},
                    expected_task_status=expected_task_status,
                )
            except StateTransitionConflict:
                pass  # 그사이 취소됐다. 결과는 취소 경로가 기록한다

        def set_plan(target_step_id: str, robot_id: str | None, detail: dict[str, Any]) -> None:
            self.task_store.set_step_plan(task_id, target_step_id, robot_id, detail)

        result = self.order_flow.run_step(StepContext(task, step, cancel_event, record, set_plan))
        if result is None:
            return False
        self.task_store.transition_step(
            task_id, step_id, StepStatus.SUCCEEDED, expected_status=StepStatus.RUNNING,
            result_payload=result, event_payload={**base, "source": source, "event": "ORDER_STEP_COMPLETED"},
        )
        return True

    def _execute_arm_policy(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        base: dict[str, Any],
        cancel_event: threading.Event,
    ) -> dict[str, Any] | None:
        """주문한 물체의 PICK_WITH_POLICY를 큐에 넣고 워커를 기다린다. None이면 취소됐다.

        잘 끝난 경우도 "원위치로 돌아옴, 놓은 것은 미확인"(UNKNOWN/PLACE_NOT_VERIFIED + pose_reached)이 최선이다.
        그 밖의 끝은 모두 ArmTaskNotConfirmed다. 0.5초마다 명령 상태를 본다."""
        assert self.arm_commands is not None
        task = self.task_store.get_task(task_id) or {}
        policy = policy_for_task(self.arm_commands.act_policies, task)
        if policy is None:
            ordered = (task.get("pick_request") or {}).get("category") or task.get("arm_job")
            raise ArmTaskNotConfirmed(
                "ARM_JOB_POLICY_NOT_FOUND", f"주문 물체 '{ordered}'에 맞는 학습 정책이 없습니다",
                source="control_station",
            )
        # SR-13: 팔에 집으라고 하기 전에 정면 카메라가 그 물체를 정확히 한 칸에서 봐야 한다.
        if self.target_evidence is None:
            raise ArmTaskNotConfirmed(
                "TARGET_EVIDENCE_NOT_CONFIGURED", "정면 카메라 확인 경로가 없어 팔 명령을 만들지 않았습니다",
                source="control_station",
            )
        try:
            evidence = self.target_evidence(policy["object_class"])
        except SlotSelectionError as exc:
            raise ArmTaskNotConfirmed(
                exc.error_code, f"{exc} — 팔 명령을 만들지 않았습니다", source="front_camera"
            ) from exc
        self.task_store.record_step_event(
            task_id, step_id, command_id, "TARGET_CONFIRMED",
            {**base, "event": "TARGET_CONFIRMED", "camera_id": evidence.get("camera_id"),
             "object_class": evidence.get("class_name"), "slot_id": evidence.get("slot_id"),
             "confidence": evidence.get("confidence"), "captured_at": evidence.get("captured_at"),
             "message": f"정면 카메라: {evidence.get('class_name')} → {evidence.get('slot_label')}"},
            expected_task_status=TaskStatus.RUNNING,
        )
        try:
            arm_command = self.arm_commands.create(
                {
                    "robot_id": base.get("robot_id") or "arm-01",
                    "operation": "PICK_WITH_POLICY",
                    "valid_for_s": 60,
                    "target": {"object_class": policy["object_class"], "pick_slot_id": evidence.get("slot_id")},
                    "place": {"slot_id": "CART_1"},
                    "source": {"kind": "OPERATOR_INPUT"},  # camera_source로 넘긴 카메라 근거로 바뀐다
                },
                camera_source=evidence,
            )
        except (ArmCommandError, ArmCommandConflict) as exc:
            raise ArmTaskNotConfirmed(
                getattr(exc, "error_code", "ARM_COMMAND_INVALID"), str(exc), source="control_station"
            ) from exc
        arm_id = arm_command["command_id"]
        self.task_store.record_step_event(
            task_id, step_id, command_id, "ARM_COMMAND_CREATED",
            {**base, "event": "ARM_COMMAND_CREATED", "arm_command_id": arm_id, "policy_id": policy["policy_id"],
             "slot_id": evidence.get("slot_id")},
            expected_task_status=TaskStatus.RUNNING,
        )
        last_state, cancel_sent = None, False
        while True:
            current = self.arm_commands.get(arm_id)
            state = current["state"]
            if state != last_state and not cancel_event.is_set():
                last_state = state
                try:
                    self.task_store.record_step_event(
                        task_id, step_id, command_id, "PROGRESS",
                        {**base, "event": "PROGRESS", "arm_command_id": arm_id, "arm_state": state,
                         "message": f"팔 명령 {arm_id}: {state}"},
                        expected_task_status=TaskStatus.RUNNING,
                    )
                except StateTransitionConflict:
                    pass
            # 큐에 있거나 워커가 실행 중이 아니면(끝 상태) 기다림을 끝낸다.
            if state != "QUEUED" and state not in {s.value for s in IN_FLIGHT}:
                break
            if cancel_event.is_set() and not cancel_sent:
                try:
                    self.arm_commands.cancel(arm_id)  # 워커가 팔을 그 자리에서 멈춘다
                except ArmCommandConflict:
                    pass
                cancel_sent = True
            time.sleep(0.5)
        if cancel_sent:
            return None
        result = current.get("result") or {}
        if state == "UNKNOWN" and current.get("reason_code") == "PLACE_NOT_VERIFIED" and result.get("pose_reached") is True:
            return {
                "source": "arm_worker",
                "outcome": "ARM_HOME_PLACEMENT_NOT_VERIFIED",
                "arm_command_id": arm_id,
                "policy_id": policy["policy_id"],
                "simulated": result.get("simulated"),
            }
        raise ArmTaskNotConfirmed(
            current.get("reason_code") or state,
            f"팔 명령 {arm_id}이(가) {state}({current.get('reason_code')})로 끝났습니다. {result.get('message') or ''}".strip(),
        )

    def _close_arm_command_after_load(self, task_id: str, load_result: dict[str, Any]) -> None:
        """실제(모의가 아닌) 적재 확인은 팔 워커에게 없던 근거다. 그것을 팔 명령에 기록해 다음 명령을 허락한다.
        모의 적재 개수라면 명령을 UNKNOWN으로 두어 운영자가 보게 한다."""
        arm_result = self._arm_results.pop(task_id, None)
        if self.arm_commands is None or not arm_result or load_result.get("source") == "mock":
            return
        try:
            self.arm_commands.clear_unknown(
                arm_result["arm_command_id"],
                f"작업 {task_id} 적재 확인({load_result.get('source')}): 관측 {load_result.get('observed_count')}개",
            )
        except (ArmCommandConflict, ArmCommandError):
            pass

    def _verify_arrival(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        base: dict[str, Any],
    ) -> dict[str, Any]:
        # 이동은 이미 성공했으니, 판정은 지금부터 찍은 프레임만으로 한다.
        assert self.arrival_check is not None
        try:
            observation = self.arrival_check()
        except Exception as exc:
            raise ArrivalNotConfirmed(
                getattr(exc, "error_code", "PARKING_CHECK_ERROR"),
                f"주차 판정을 시작하지 못했습니다: {exc}",
            ) from exc
        self.task_store.record_step_event(
            task_id,
            step_id,
            command_id,
            "PARKING_OBSERVED",
            {
                **base,
                "source": "parking_check",
                "event": "PARKING_OBSERVED",
                "observation": observation,
                "task_state_changed": False,
                "arm_command_sent": False,
            },
            expected_task_status=TaskStatus.RUNNING,
        )
        status = observation.get("status")
        if status != "ARRIVED":
            # 자동으로 다시 몰지 않는다: 운영자가 어긋남을 보고 판단한다(pinky-arrival-gate 계획).
            raise ArrivalNotConfirmed(
                "PARKING_MISALIGNED" if status == "MISALIGNED" else "PARKING_NOT_VERIFIED",
                f"주차 판정 {status}: {observation.get('reason')}",
            )
        return {
            "source": "parking_check",
            "outcome": "PARKING_ARRIVED",
            "spot_id": observation.get("spot_id"),
        }

    def _verify_load(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        base: dict[str, Any],
    ) -> dict[str, Any]:
        """적재 관측을 받아 기대 개수(EXPECTED_LOAD_COUNT)와 비교한다. LOADED가 아니면 LoadNotConfirmed."""
        assert self.load_check is not None
        try:
            observation = self.load_check()
        except Exception as exc:
            raise LoadNotConfirmed(
                getattr(exc, "error_code", "LOAD_CHECK_ERROR"),
                f"적재 확인을 시작하지 못했습니다: {exc}",
                "load_check",
            ) from exc
        source = str(observation.get("source") or "load_check")
        status = judge_load(observation, EXPECTED_LOAD_COUNT)
        self.task_store.record_step_event(
            task_id,
            step_id,
            command_id,
            "LOAD_OBSERVED",
            {
                **base,
                "source": source,
                "event": "LOAD_OBSERVED",
                "observation": observation,
                "expected_count": EXPECTED_LOAD_COUNT,
                "status": status,
                "task_state_changed": False,
            },
            expected_task_status=TaskStatus.RUNNING,
        )
        if status != "LOADED":
            # 확인되지 않은 짐을 싣고 핑키가 떠나지 않는다. 다음 일은 운영자가 정한다.
            raise LoadNotConfirmed(
                LOAD_ERROR_CODES[status],
                f"적재 확인 {status}: 기대 {EXPECTED_LOAD_COUNT}개, 관측 {observation.get('observed_count')}개"
                f" ({observation.get('reason')})",
                source,
            )
        return {
            "source": source,
            "outcome": "LOAD_CONFIRMED",
            "expected_count": EXPECTED_LOAD_COUNT,
            "observed_count": observation.get("observed_count"),
        }

    def _finish_cancel(self, task_id: str, active_step_id: str | None) -> None:
        """취소 마무리: 진행 중이던 단계를 CANCELED로, 남은 단계를 실행 안 함으로 닫고, CANCELING이면 CANCELED로."""
        task = self.task_store.get_task(task_id)
        if task is None:
            return
        if active_step_id is not None:
            step = next((item for item in task["steps"] if item["id"] == active_step_id), None)
            if step and step["status"] in {
                StepStatus.PENDING.value,
                StepStatus.COMMAND_SENT.value,
                StepStatus.ACCEPTED.value,
                StepStatus.RUNNING.value,
            }:
                self.task_store.transition_step(
                    task_id,
                    active_step_id,
                    StepStatus.CANCELED,
                    expected_status=StepStatus(step["status"]),
                    result_payload={"source": "mock", "outcome": "CANCEL_CONFIRMED"},
                    event_payload={
                        "source": "mock",
                        "robot_id": self._mock_robot_id(step),
                        "event": "CANCEL_CONFIRMED",
                        "task_id": task_id,
                        "step_id": active_step_id,
                        "command_id": step.get("command_id"),
                    },
                )
        self._mark_pending_steps_not_executed(task_id, reason="CANCEL")
        task = self.task_store.get_task(task_id)
        if task and task["status"] == TaskStatus.CANCELING.value:
            self.task_store.transition_task(
                task_id,
                TaskStatus.CANCELED,
                expected_status=TaskStatus.CANCELING,
                payload={"source": "mock", "event": "MOCK_STOP_CONFIRMED"},
            )

    def _mark_pending_steps_not_executed(self, task_id: str, *, reason: str) -> None:
        """아직 대기 중인 단계를 모두 CANCELED(NOT_EXECUTED_AFTER_CANCEL 또는 _FAILURE)로 닫는다."""
        if reason not in {"CANCEL", "FAILURE"}:
            raise ValueError(f"Unsupported pending-step closure reason: {reason}")
        outcome = f"NOT_EXECUTED_AFTER_{reason}"
        task = self.task_store.get_task(task_id)
        if task is None:
            return
        for step in task["steps"]:
            if step["status"] == StepStatus.PENDING.value:
                self.task_store.transition_step(
                    task_id,
                    step["id"],
                    StepStatus.CANCELED,
                    expected_status=StepStatus.PENDING,
                    result_payload={"source": "mock", "outcome": outcome},
                    event_payload={
                        "source": "mock",
                        "robot_id": self._mock_robot_id(step),
                        "event": outcome,
                        "task_id": task_id,
                        "step_id": step["id"],
                        "command_id": None,
                    },
                )

    def _finish_error(
        self,
        task_id: str,
        active_step_id: str | None,
        error: Exception,
    ) -> None:
        # 실패 마무리를 동시에 온 취소 요청과 줄 세운다.
        # 잠금을 먼저 잡은 쪽이 기록될 끝 경로를 정한다.
        with self._lock:
            self._finish_error_locked(task_id, active_step_id, error)

    def _finish_error_locked(
        self,
        task_id: str,
        active_step_id: str | None,
        error: Exception,
    ) -> None:
        """예외 종류로 출처·오류 코드·이벤트 이름을 정하고, 진행 중 단계를 FAILED로, 남은 단계를 실행 안 함으로 닫고,
        작업을 FAILED로 끝낸다. 이미 취소 중이면 취소로 마무리한다. 자동 재시도는 없다."""
        task = self.task_store.get_task(task_id)
        if task is None:
            return
        source = "mock"
        if isinstance(error, ArrivalNotConfirmed):
            source, error_code, event_name = "parking_check", error.error_code, "ARRIVAL_NOT_CONFIRMED"
        elif isinstance(error, LoadNotConfirmed):
            source, error_code, event_name = error.source, error.error_code, "LOAD_NOT_CONFIRMED"
        elif isinstance(error, ArmTaskNotConfirmed):
            source, error_code, event_name = error.source, error.error_code, "ARM_TASK_NOT_CONFIRMED"
        elif isinstance(error, OrderFlowError):
            source, error_code, event_name = error.source, error.error_code, "ORDER_STOPPED"
        elif isinstance(error, MockScenarioFailure):
            error_code = event_name = "MOCK_SCENARIO_FAILURE"
        else:
            error_code, event_name = "MOCK_EXECUTION_ERROR", "MOCK_EXECUTION_FAILED"
        if task["status"] == TaskStatus.CANCELING.value:
            self._finish_cancel(task_id, active_step_id)
            return
        if active_step_id is not None:
            step = next((item for item in task["steps"] if item["id"] == active_step_id), None)
            if step and step["status"] in {
                StepStatus.PENDING.value,
                StepStatus.COMMAND_SENT.value,
                StepStatus.ACCEPTED.value,
                StepStatus.RUNNING.value,
            }:
                self.task_store.transition_step(
                    task_id,
                    active_step_id,
                    StepStatus.FAILED,
                    expected_status=StepStatus(step["status"]),
                    error_code=error_code,
                    error_message=str(error),
                    event_payload={
                        "source": source,
                        "robot_id": self._mock_robot_id(step),
                        "event": event_name,
                        "task_id": task_id,
                        "step_id": active_step_id,
                        "command_id": step.get("command_id"),
                        "error_code": error_code,
                        "error": str(error),
                    },
                )
        self._mark_pending_steps_not_executed(task_id, reason="FAILURE")
        task = self.task_store.get_task(task_id)
        if task and task["status"] in {
            TaskStatus.VALIDATING.value,
            TaskStatus.RUNNING.value,
        }:
            self.task_store.transition_task(
                task_id,
                TaskStatus.FAILED,
                expected_status=TaskStatus(task["status"]),
                payload={
                    "source": source,
                    "error_code": error_code,
                    "error": str(error),
                    "event": event_name,
                },
            )

    @staticmethod
    def _mock_robot_id(step: dict[str, Any]) -> str | None:
        """이전 형식의 로봇 종류("MOBILE"/"ARM")는 로봇 id로 바꾸고, 설정된 로봇 id는 그대로 둔다."""
        robot_kind = step.get("robot_id")
        return ROBOT_ID_BY_KIND.get(robot_kind, robot_kind) if isinstance(robot_kind, str) else None
