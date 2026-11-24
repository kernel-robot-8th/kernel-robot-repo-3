"""실제 팔로 하는 주문 집기: 계획된 팔에 PICK_WITH_POLICY를 HTTP 또는 ROS 2 액션으로 보낸다.

주문 흐름(orders/flow.py의 PICK_ITEM 단계)은 물체 한 개씩 `pick(arm_id, object_class, shelf_id, pinky_id,
cancel_event, progress)`로 요청한다. 모의 세계(mock_world.MockWorld.pick)가 답하는 것과 같은 호출이다. 아래 두
전송 방식은 모두 먼저 같은 팔 명령 기록(그 팔에서 녹화한 정책)을 만든다. 그래서 화면의 명령 이력, 취소, UNKNOWN
규칙이 똑같이 동작한다.

- ArmQueuePick (CONTROL_STATION_ORDER_ARM_TRANSPORT=http, 기본값): 그 팔의 HTTP 워커
  (scripts/omx_lerobot_worker.py --robot-id arm-0N)가 큐에서 명령을 가져가 결과를 알린다.
- ArmActionPick (=ros2): 관제가 명령을 직접 가져가(claim) 그 팔의 액션 서버(scripts/omx_pick_action_server.py,
  /arm_0N/pick_object)에 목표(goal)로 보낸다. 피드백과 결과는 같은 기록에 워커 이벤트로 적는다:
  STARTING_POLICY = ACCEPTED, RUNNING_POLICY = PROGRESS, 결과 = 마지막 이벤트.

ACT 정책은 한 번 실행에 집기, 핑키 위에 놓기, 쉬는 자세로 돌아가기를 모두 한다. 그래서 어느 방식이든 팔이 말할 수
있는 것은 "home에 돌아왔다"뿐이고, 명령은 UNKNOWN(PLACE_NOT_VERIFIED)으로 끝난다. 물체가 정말 옮겨졌는지는 주문
흐름의 VERIFY_TRANSFER 단계가 카메라로 판정한다. 그 확인을 통과하면 `settle`이 카메라 근거로 명령을 닫아서, 팔이
다음 집기 명령을 받을 수 있게 된다(UNKNOWN이 남아 있으면 그 팔에 새 명령을 만들 수 없다).

CONTROL_STATION_ORDER_REAL_ARM=1일 때 켜진다(app.py에서 조립).
"""

from __future__ import annotations

import itertools
import queue
import threading
import time
from collections.abc import Callable
from datetime import datetime, timedelta, timezone
from typing import Any, Protocol

from control_station.arms.commands import IN_FLIGHT, ArmCommandConflict, ArmCommandError, ArmCommandStore
from control_station.orders.flow import OrderFlowError

# 집기 결과(outcome): 팔은 home에 돌아왔지만 적재는 확인 못 함. 모의 세계의 pick도 같은 값을 돌려준다.
HOME_REACHED_UNVERIFIED = "ARM_HOME_PLACEMENT_NOT_VERIFIED"
# 정책 자체 실행 시간(max_run_s)에 더해, 워커가 명령을 가져가고 ACK하고 home으로 돌아오는 데 주는 여유 시간(초).
QUEUE_AND_HOME_S = 180.0
# 아직 끝나지 않은 명령 상태: 이 상태인 동안 기록을 계속 따라간다.
_WAITING = {"QUEUED", *(state.value for state in IN_FLIGHT)}
# 이 액션 피드백 단계부터 팔이 움직일 수 있다. 그 전에 기록이 ACCEPTED가 되어 있어야 한다.
_MOVING_PHASES = {"STARTING_POLICY", "RUNNING_POLICY", "RETURNING_HOME"}
_PHASE_WORDS = {"CHECKING": "확인 중", "STARTING_POLICY": "정책 시작", "RUNNING_POLICY": "정책 실행 중",
                "RETURNING_HOME": "home으로 복귀 중"}


class _ArmPick:
    """두 전송 방식이 함께 쓰는 부분: 팔에 맞는 정책 고르기, 명령 기록 만들기, 최종 상태 해석, `settle`.

    poll_s: 기록을 다시 읽는 간격(초). valid_for_s: 명령 유효 시간(초, 워커가 이 안에 가져가야 함).
    clock: 시험에서 바꿔 끼울 수 있는 단조 시계."""

    source = "arm_worker"

    def __init__(self, commands: ArmCommandStore, *, poll_s: float = 0.5, valid_for_s: int = 60,
                 extra_wait_s: float = QUEUE_AND_HOME_S, clock: Callable[[], float] = time.monotonic) -> None:
        self.commands = commands
        self.poll_s = poll_s
        self.valid_for_s = valid_for_s
        self.extra_wait_s = extra_wait_s
        self.clock = clock

    def _policy(self, arm_id: str, object_class: str) -> dict[str, Any]:
        """그 팔에서 쓸 수 있는 정책. 없으면 팔을 움직이지 않고 주문을 멈춘다(POLICY_NOT_FOR_ARM)."""
        policies = self.commands.act_policies
        policy = policies.resolve(object_class, arm_id) if policies is not None else None
        if policy is None:
            raise OrderFlowError("POLICY_NOT_FOR_ARM",
                                 f"{arm_id}에서 녹화한 {object_class} 정책이 없어 팔을 움직이지 않았습니다", "arm")
        return policy

    def _create(self, arm_id: str, policy: dict[str, Any], shelf_id: str, pinky_id: str) -> dict[str, Any]:
        """PICK_WITH_POLICY 명령 기록을 만든다. 저장소가 거부하면(다른 명령 진행 중 등) 그 오류 코드로 주문을 멈춘다."""
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
        """결과를 기다릴 마감 시각(clock 기준): 정책의 max_run_s(없으면 60초) + 여유 시간."""
        return self.clock() + float(policy.get("max_run_s") or 60) + self.extra_wait_s

    def _wait_store(self, arm_id: str, policy: dict[str, Any], command_id: str, cancel_event: threading.Event,
                    progress: Callable[[str], None]) -> dict[str, Any] | None:
        """워커가 끝낼 때까지 기록을 따라간다. 주문이 취소되면 명령도 취소한다.

        주문 취소로 끝났으면 None, 아니면 _conclude의 결과다. 마감까지 끝나지 않으면 ARM_RESULT_TIMEOUT으로 멈춘다
        (팔이 어떤 상태인지 모르므로 사람이 현장에서 확인해야 한다)."""
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
                    self.commands.cancel(command_id)  # 워커가 팔을 그 자리에서 멈춘다
                except ArmCommandConflict:
                    pass
                cancel_sent = True
            if self.clock() > deadline:
                raise OrderFlowError("ARM_RESULT_TIMEOUT",
                                     f"팔 명령 {command_id}의 결과가 오지 않았습니다. 팔을 현장에서 확인하세요.", "arm")
            time.sleep(self.poll_s)
        return None if cancel_sent else self._conclude(arm_id, policy, command_id, current)

    def _conclude(self, arm_id: str, policy: dict[str, Any], command_id: str, current: dict[str, Any]) -> dict[str, Any]:
        """기록의 최종 상태를 집기 결과로 바꾼다. 받아들일 수 없는 결과면 팔이 알린 이유로 주문을 멈춘다.

        받아들이는 경우는 둘이다: SUCCEEDED, 또는 home 도착을 측정한(pose_reached) UNKNOWN(PLACE_NOT_VERIFIED)."""
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
        """home에는 왔지만 적재가 확인되지 않은 명령을, 카메라가 물체가 옮겨진 것을 본 뒤에 닫는다(clear_unknown).
        모의 개수(simulated)로는 절대 닫지 않는다. 그때 명령은 사람이 확인할 때까지 UNKNOWN으로 남고, 그 전까지
        그 팔은 새 명령을 받지 않는다."""
        if pick.get("outcome") != HOME_REACHED_UNVERIFIED or evidence.get("simulated"):
            return
        try:
            self.commands.clear_unknown(pick["arm_command_id"],
                                        f"주문 {evidence.get('task_id')}: 카메라로 옮김 확인({evidence.get('source')})")
        except (ArmCommandConflict, ArmCommandError) as exc:
            raise OrderFlowError("ARM_COMMAND_NOT_CLOSED",
                                 f"옮김은 확인했지만 팔 명령 {pick['arm_command_id']}을(를) 닫지 못했습니다: {exc}", "arm") from exc


class ArmQueuePick(_ArmPick):
    """HTTP 명령 큐를 거치는 주문 흐름의 `pick` (모듈 설명 참고)."""

    def __call__(self, arm_id: str, object_class: str, shelf_id: str, pinky_id: str,
                 cancel_event: threading.Event, progress: Callable[[str], None]) -> dict[str, Any] | None:
        policy = self._policy(arm_id, object_class)
        command_id = self._create(arm_id, policy, shelf_id, pinky_id)["command_id"]
        progress(f"{arm_id}: 팔 명령 {command_id} 대기열에 넣음 ({policy['policy_id']})")
        return self._wait_store(arm_id, policy, command_id, cancel_event, progress)


# ---- ROS 2 액션 전송 ------------------------------------------------------------------------------------------

class PickGoal(Protocol):
    """보낸 목표 하나: 서버가 받았는지, 취소, 결과 기다리기(시간 안에 없으면 None). ros_action._RosGoal이 구현한다."""
    accepted: bool

    def cancel(self) -> None: ...

    def result(self, timeout_s: float) -> dict[str, Any] | None: ...


class PickTransport(Protocol):
    """목표를 보내는 쪽. 실제로는 ros_action.RosPickTransport이고, 시험에서는 가짜로 바꿔 끼운다."""
    def send_goal(self, arm_id: str, target: dict[str, Any],
                  on_feedback: Callable[[dict[str, Any]], None]) -> PickGoal: ...


class _Record:
    """액션의 진행을 이 전송 방식의 워커 이벤트로 팔 명령 기록에 적는다.

    event_id는 "{command_id}.{번호}"로 1부터 차례로 붙인다. simulated는 액션 서버가 알려 준 값을 따른다."""

    def __init__(self, commands: ArmCommandStore, command_id: str, arm_id: str, worker_id: str) -> None:
        self.commands, self.command_id, self.arm_id, self.worker_id = commands, command_id, arm_id, worker_id
        self.numbers = itertools.count(1)
        self.simulated = False
        self.refusal: str | None = None

    def event(self, event: str, **fields: Any) -> bool:
        """기록이 이벤트를 받았으면 True. 거부되면(그사이 취소·만료됐거나 이미 끝남) 이유를 `refusal`에 남기고,
        기록은 저장소가 둔 상태 그대로다. 값이 None이거나 빈 문자열인 필드는 보내지 않는다."""
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
    """팔의 액션 서버로 보내는 ROS 2 액션 방식의 주문 흐름 `pick` (모듈 설명 참고)."""

    source = "arm_action"

    def __init__(self, commands: ArmCommandStore, transport: PickTransport, **options: Any) -> None:
        super().__init__(commands, **options)
        self.transport = transport

    def _start_deadline(self, command: dict[str, Any]) -> str:
        """목표의 valid_until: 명령의 유효 시각과 같지만, 기록의 ACK 제한 시간(CONTROL_STATION_ARM_ACK_TIMEOUT_S에서
        피드백이 돌아오는 데 쓸 1초를 뺀 값)을 넘지 않는다. 서버는 이 시각이 지나면 정책을 시작하지 않으므로, 저장소가
        이미 UNKNOWN(ACK_TIMEOUT)으로 닫은 기록에 대해 팔이 움직이는 일이 없다."""
        ack_end = datetime.now(timezone.utc) + timedelta(seconds=max(1.0, self.commands.ack_timeout_s - 1.0))
        return min(datetime.fromisoformat(command["valid_until"]), ack_end).isoformat()

    def __call__(self, arm_id: str, object_class: str, shelf_id: str, pinky_id: str,
                 cancel_event: threading.Event, progress: Callable[[str], None]) -> dict[str, Any] | None:
        policy = self._policy(arm_id, object_class)
        command_id = self._create(arm_id, policy, shelf_id, pinky_id)["command_id"]
        worker_id = f"ros2-action.{arm_id}"
        claimed = self.commands.claim_next(arm_id, worker_id, 0)
        if claimed is None or claimed["command_id"] != command_id:
            # 이 팔의 HTTP 워커가 먼저 가져갔다. 그 워커가 실행하므로 여기서는 기록만 따라간다.
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
        except Exception as exc:  # 서버가 없거나 답이 없음: 팔에는 아무것도 가지 않았다
            record.event("REJECTED", reason_code="ACTION_SERVER_UNAVAILABLE", message=str(exc)[:300])
            raise OrderFlowError("ACTION_SERVER_UNAVAILABLE", f"{arm_id}: {exc}", "arm") from exc
        if not goal.accepted:
            record.event("REJECTED", reason_code="ACTION_GOAL_REJECTED", message="액션 서버가 목표를 받지 않았습니다(다른 목표 실행 중)")
            raise OrderFlowError("ACTION_GOAL_REJECTED", f"{arm_id}: 집기 액션 서버가 목표를 받지 않았습니다", "arm")

        # 피드백 처리 상태: ACCEPTED·PROGRESS를 한 번씩만 적고, 단계가 바뀔 때만 진행 문구를 낸다.
        state = {"accepted": False, "running": False, "phase": None}

        def take_feedback() -> None:
            """쌓인 피드백을 모두 처리한다. 피드백 콜백은 ROS 스레드에서 오므로 큐로 받아 이 스레드에서 처리한다."""
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
                        goal.cancel()  # 팔이 움직이기 전에 기록이 취소·만료됨: 액션을 멈춘다
                if item["phase"] == "RUNNING_POLICY" and not state["running"]:
                    state["running"] = True
                    record.event("PROGRESS", phase="APPROACH_TARGET")

        deadline = self._deadline(policy)
        # 취소는 두 군데서 온다: 주문 취소(cancel_event), 화면에서의 명령 취소(기록의 cancel_requested).
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
                        self.commands.cancel(command_id)  # 팔이 보고할 CANCELED를 받으려면 기록에도 취소 요청이 있어야 한다
                    except ArmCommandConflict:
                        pass
                goal.cancel()  # 서버가 팔이 있는 자리에서 정책을 멈춘다
                cancel_sent = True
            if self.clock() > deadline:
                goal.cancel()
                record.event("RESULT_UNKNOWN", reason_code="ACTION_RESULT_TIMEOUT",
                             message="집기 액션의 결과가 오지 않아 취소를 보냈습니다. 팔을 현장에서 확인하세요.")
                raise OrderFlowError("ARM_RESULT_TIMEOUT",
                                     f"팔 명령 {command_id}의 결과가 오지 않았습니다. 팔을 현장에서 확인하세요.", "arm")
        take_feedback()  # 결과가 마지막 피드백보다 먼저 올 수 있다. 끝 이벤트 전에 ACCEPTED를 먼저 적어야 한다
        record.simulated = result["simulated"]
        self._record_result(record, result, state["accepted"])
        if order_canceled and result["outcome"] == "CANCELED":
            return None  # 주문 자체가 취소됨. 화면에서 한 취소는 아래 _conclude에서 주문을 멈춘다
        return self._conclude(arm_id, policy, command_id, self.commands.get(command_id))

    @staticmethod
    def _record_result(record: _Record, result: dict[str, Any], accepted: bool) -> None:
        """액션 결과를 기록의 마지막 워커 이벤트로 바꾼다.

        home 도착이 측정된 성공(SUCCEEDED + pose_reached)도 RESULT_UNKNOWN(PLACE_NOT_VERIFIED)으로 적는다. 팔은 적재를
        확인할 수 없고, 적재 확인은 카메라(VERIFY_TRANSFER)가 하기 때문이다."""
        outcome, reason, message = result["outcome"], result["reason_code"] or None, result["message"] or None
        if outcome == "REJECTED" and not accepted:
            record.event("REJECTED", reason_code=reason or "ACTION_REJECTED", message=message)
            return
        if not accepted and not record.event("ACCEPTED"):
            return  # 저장소가 이미 기록을 닫았다(취소 또는 만료). 그 상태를 그대로 둔다
        if outcome == "HOME_REACHED_PLACE_NOT_VERIFIED" or (outcome == "SUCCEEDED" and result["pose_reached"]):
            record.event("RESULT_UNKNOWN", reason_code="PLACE_NOT_VERIFIED", pose_reached=True, message=message)
        elif outcome == "CANCELED":
            # 기록에 취소 요청이 없으면 CANCELED를 받지 않는다(팔이 스스로 멈춘 것). 그때는 FAILED로 적는다.
            if not record.event("CANCELED", phase="APPROACH_TARGET", message=message):
                record.event("FAILED", reason_code=reason or "CANCELED_BY_ARM", message=message)
        elif outcome == "FAILED" or outcome == "REJECTED":
            record.event("FAILED", reason_code=reason or "ACTION_FAILED", message=message)
        else:  # RESULT_UNKNOWN 또는 새로 생긴 값: 모르는 결과는 성공이나 실패로 바꾸지 않는다
            record.event("RESULT_UNKNOWN", reason_code=reason or "ACTION_RESULT_UNKNOWN", message=message)
