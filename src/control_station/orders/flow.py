"""주문 흐름: 선반 재고 → 팔 계획 → 핑키 기다리기 → 물체마다 집고 확인 → 총량 → 신호.

단계(주문마다 store.order_step_definitions가 만든다):
  VALIDATE_ORDER       물체에 정책이 있고 팔이 설정돼 있는지. 핑키를 출발시킨다(dispatch가 있으면; 모의 세계도 여기서)
  CHECK_SHELF_STOCK    선반 카메라가 물체마다 주문 수량 이상을 보는지(아니면 SHELF_STOCK_SHORT)
  PLAN_ARM_SEQUENCE    단위마다 그 물체가 보인 선반의 팔로 배정(왼쪽 선반 → 왼쪽 팔)하고 팔별로 묶는다.
                       결과를 PICK_ITEM/VERIFY_TRANSFER 단계에 적는다
  WAIT_PINKY_ARRIVAL   적재 자리가 이 주문 차지이고, 핑키 관제가 핑키가 정류장에 섰다고(docking_manual) 알리고,
                       천장 카메라가 ARRIVED로 판정할 때 통과. 어긋난 핑키는 PINKY_POSITION_ABNORMAL을 띄운다.
                       카메라가 redock_after_checks번 연속 MISALIGNED라고 하면 핑키를 다시 들여보낸다(redock: 물러나
                       방향을 맞추고 다시 들어옴). max_redocks번까지 하고, 그 뒤에는 단계가 운영자를 기다린다.
                       이 단계를 통과하기 전에는 팔이 절대 움직이지 않는다.
  CONFIRM_DOCKING      핑키 관제에 dock_complete. 핑키 관제는 그 뒤 핑키를 세워 둔다(docked_manual)
  PICK_ITEM            계획된 팔이 학습된 정책으로 단위 하나를 집어 핑키 위에 놓는다
  VERIFY_TRANSFER      선반 −1, 핑키 +1인지. 선반에 그대로 → PICK_FAILED_STILL_ON_SHELF,
                       둘 다 없음 → ITEM_LOST, 그 밖 → TRANSFER_NOT_VERIFIED. 모두 주문을 멈춘다.
  VERIFY_LOAD_TOTAL    핑키가 주문한 물체를 정확히 싣고 있는지
  NOTIFY_PINKY_RECEIVED  핑키 관제에 received. 요청이 받아들여졌다는 것은 핑키 관제가 신호를 받았다는 뜻이지,
                       핑키가 떠났다는 뜻이 아니다
  WAIT_AT_RECEIVING_ZONE / CONFIRM_PICKUP / WAIT_PINKY_PARKED  핑키가 구역에 닿고, 손님이 확인하고,
                       핑키가 주차장으로 돌아간다
쟁반 하나보다 큰 주문은 운송분으로 나눠 차례로 돈다(store.order_step_definitions): WAIT_PINKY_ARRIVAL부터
WAIT_PINKY_PARKED까지의 단계가 운송분마다 되풀이되고, 뒤 운송분은 앞 핑키가 주차장에 돌아온 뒤 START_SHIPMENT가
연다(한 번에 핑키 한 대만 달린다). START_SHIPMENT는 그 적재분의 핑키를 고르고 출발시킨다.
실패하면 재시도도, 핑키에 신호도 없이 주문을 멈춘다. 운영자가 경보를 보고 판단한다. 실패 뒤의 운송분은 시작하지 않는다.
관측기와 집는 장치는 바깥에서 끼워 넣는다(OrderDevices: 모의 세계 또는 카메라·핑키 관제·팔 큐).

스레드: 단계는 주문마다 작업 실행기(task_runner)의 작업 스레드에서 돈다. 여러 주문이 동시에 돌 수 있어서, 적재 자리
주인과 대기 표(_arrivals, _pickups, _order_pinky)는 `_guard` 잠금으로 지킨다. 잠금은 보거나 바꿀 때만 잡고,
기다리는 동안에는 쥐지 않는다.
"""

from __future__ import annotations

import threading
import time
from collections import Counter
from dataclasses import dataclass
from typing import Any, Callable

from control_station.orders.assignment import ArmPlanError, PickStats, plan_arms
from control_station.pinky.docking import COMPLETE, OPERATOR, REDOCK, DockHandshake, _result_text  # noqa: F401
from control_station.pinky.fleet_client import STATION_PHASES, PinkyFleetError
from control_station.storage.store import PINKY_NAMES, shipment_of

ORDER_STEP_TYPES = frozenset({
    "VALIDATE_ORDER", "CHECK_SHELF_STOCK", "PLAN_ARM_SEQUENCE", "WAIT_PINKY_ARRIVAL", "CONFIRM_DOCKING",
    "PICK_ITEM", "VERIFY_TRANSFER", "VERIFY_LOAD_TOTAL", "NOTIFY_PINKY_RECEIVED",
    "WAIT_AT_RECEIVING_ZONE", "CONFIRM_PICKUP", "WAIT_PINKY_PARKED", "START_SHIPMENT",
})
# 적재 카메라가 보지만 짐이 아닌 것(YOLO는 핑키 자신을 car라고 부른다).
NOT_CARGO = frozenset({"car"})


class OrderFlowError(RuntimeError):
    """주문이 여기서 멈춘다. error_code는 운영자 경보에 보이는 코드, source는 누가 확인하지 못했는지다."""

    def __init__(self, error_code: str, message: str, source: str = "control_station") -> None:
        super().__init__(message)
        self.error_code = error_code
        self.source = source


@dataclass
class StepContext:
    """단계가 실행되는 동안 작업에 할 수 있는 일. 작업 실행기가 만들어 넘긴다."""

    task: dict[str, Any]  # 단계 시작 직전에 다시 읽은 작업(단계·이벤트 포함)
    step: dict[str, Any]
    cancel_event: threading.Event  # 취소되면 켜진다. 기다리는 곳마다 이것으로 기다린다
    record: Callable[[str, dict[str, Any]], None]  # 이 단계에 (event_type, payload) 이벤트 기록
    set_plan: Callable[[str, str | None, dict[str, Any]], None]  # 대기 단계에 (step_id, robot_id, detail) 계획 적기



@dataclass
class OrderDevices:
    """흐름이 쓰는 관측기와 구동기. 모의 세계가 전부를 줄 수도 있고, 설정에 따라 실제 장치로 바뀐다(app.py)."""

    shelf_stock: Callable[[], dict[str, Any]]  # 선반별 물체 개수 {"shelves": {...}, "source", "simulated"}
    pinky_load: Callable[[str], dict[str, Any]]  # 핑키 위 물체 개수 {"counts": {...}, ...}
    fleet_status: Callable[[str], dict[str, Any]]  # 핑키 관제가 본 핑키 상태(phase, mission_token, ...)
    fleet_confirm: Callable[[str, str, str | None], dict[str, Any]]  # (핑키, action, 토큰): dock_complete 등
    parking: Callable[[str], dict[str, Any]]  # 천장 카메라의 주차 판정(ARRIVED/MISALIGNED/...)
    # (팔, 물체, 선반, 핑키, cancel_event, 진행 알림) → 결과, 취소면 None.
    pick: Callable[..., dict[str, Any] | None]
    # 핑키 경로 출발(핑키 번호, 수령 구역까지의 웨이포인트 이름). None이면 출발은 핑키 관제에 맡긴다.
    dispatch: Callable[[str, list[str] | None], Any] | None = None
    # 수령 구역에서 수령이 확인됨: 핑키에게 집으로 가라고 알린다(핑키 번호, 임무 토큰).
    mission_complete: Callable[[str, str | None], dict[str, Any]] | None = None
    # `pick`이 실제 팔을 움직이면 True(arm_pick.ArmQueuePick). 그러면 옮김마다 카메라로 세야 한다.
    pick_is_real: bool = False
    # VERIFY_TRANSFER가 물체가 옮겨진 것을 본 뒤: (집기 결과, 근거)로 팔의 미확인(UNKNOWN) 명령을 닫는다.
    pick_settle: Callable[[dict[str, Any], dict[str, Any]], None] | None = None


def _label(policies: Any, object_class: str) -> str:
    """물체 클래스 → 정책 파일의 사람용 이름. 없으면 클래스 그대로."""
    if policies is not None:
        found = policies.matches(object_class)
        if found:
            return found[0]["label"]
    return object_class


class OrderFlow:
    """주문 단계 하나하나를 실행한다(run_step). 작업 실행기가 주문의 단계마다 부른다.

    시간 값은 모두 초다: dwell_s(빠른 단계도 화면에 보이게 머무는 시간), poll_s(핑키·카메라를 다시 보는 간격),
    arrival_timeout_s(핑키 도착·구역 도착·복귀·다음 운송분 핑키를 기다리는 한도), signal_timeout_s(dock_complete 뒤
    docked_manual이 되기까지), pickup_timeout_s(손님 수령 확인 한도, 기본 30분)."""

    def __init__(
        self,
        arms: Any,
        policies: Any,
        devices: OrderDevices,
        *,
        dwell_s: float = 0.25,
        poll_s: float = 0.5,
        arrival_timeout_s: float = 600.0,
        # dock_complete → docked_manual 사이에 docking_anchor(5번 지도 위치 재설정)를 거치는데, 핑키가 여기에
        # 최대 18초를 쓴다. 그래서 30초를 준다.
        signal_timeout_s: float = 30.0,
        max_redocks: int = 5,
        redock_after_checks: int = 3,
        # 카메라가 이 횟수만큼 연속으로 ARRIVED라고 해야 한다. redock을 보낸 뒤에는 핑키가 자리를 떠날 시간
        # redock_wait_s를 주고 그 전에는 같은 화면을 다시 판정하지 않는다. camera_blind_s 동안 판정하지 못하는
        # 카메라는 도착 확인을 운영자에게 넘긴다(pinky/docking.py).
        arrived_checks: int = 2,
        redock_wait_s: float = 15.0,
        camera_blind_s: float = 20.0,
        pick_stats: Callable[[], PickStats] | None = None,
        route_for: Callable[[str | None], list[str] | None] | None = None,
        pickup_timeout_s: float = 1800.0,
        choose_shipment_pinky: Callable[[str, int], dict[str, Any]] | None = None,
    ) -> None:
        self.arms = arms
        # (주문 id, 운송분 번호) → {"pinky_id": ... 또는 None, "summary": ...}: 뒤 운송분의 핑키. 주문 창구가
        # 골라 주문에 적는다(OrderDesk.assign_next_shipment). None이면 운송분을 돌릴 수 없다.
        self.choose_shipment_pinky = choose_shipment_pinky
        # 주차장에서 주문의 수령 구역까지의 웨이포인트 이름(config/receiving-zones.json).
        self.route_for = route_for or (lambda zone_id: None)
        self.pickup_timeout_s = pickup_timeout_s
        # 핑키가 수령 구역에서 기다리는 주문: 주문 id → 손님이 수령을 확인하면 켜지는 이벤트.
        self._pickups: dict[str, threading.Event] = {}
        # 핑키가 등록된 마커 없이(또는 카메라가 판정하지 못한 채) 적재 자리에 선 주문: 천장 카메라가 판정할 수
        # 없으니 운영자가 도착을 확인한다(confirm_arrival). 주문 id → 확인하면 켜지는 이벤트.
        self._arrivals: dict[str, threading.Event] = {}
        # 팔 선택에 쓰는 (정책, 팔)별 지난 집기 결과. 없으면 모든 팔이 같은 성공률에서 시작한다.
        self.pick_stats = pick_stats or dict
        self.policies = policies
        self.devices = devices
        self.dwell_s = dwell_s
        self.poll_s = poll_s
        self.arrival_timeout_s = arrival_timeout_s
        self.max_redocks = max_redocks
        self.redock_after_checks = redock_after_checks
        self.arrived_checks, self.redock_wait_s, self.camera_blind_s = arrived_checks, redock_wait_s, camera_blind_s
        self.signal_timeout_s = signal_timeout_s
        # 두 팔 사이에는 핑키 한 대만 설 수 있으니, 한 번에 한 주문만 팔과 적재 자리를 쓴다(그 주문 id).
        self.station_owner: str | None = None
        # 멈춘 주문은 운영자가 경보를 확인하거나 그 핑키가 정류장을 떠날 때까지 자리를 쥔다(핑키가 아직 서 있으므로):
        # 주문 id → 핑키 번호.
        self.held_by_stopped: dict[str, str] = {}
        # 주문 id → 핑키 관제가 단계마다 새로 주는 임무 토큰. 신호에 같이 보내 늦은 신호·엉뚱한 핑키를 막는다.
        self._tokens: dict[str, str | None] = {}
        # 주문의 흐름이 적재 자리에서 직접 상대하는 핑키(주문 id → 핑키 번호). 도착 대기부터 주문이 끝날 때까지
        # (NOTIFY_PINKY_RECEIVED의 release에서도 지운다). 테스트 화면의 도킹 감독은 이 핑키들을 건드리지 않는다.
        self._order_pinky: dict[str, str] = {}
        self._before: dict[str, dict[str, int]] = {}  # 주문 id → 집기 직전 {"shelf": 선반 개수, "pinky": 핑키 위 개수}
        self._picked: dict[str, dict[str, Any]] = {}  # 주문 id → 마지막 집기 결과. VERIFY_TRANSFER가 닫는다
        self._guard = threading.Lock()

    # ---- 작업 실행기가 부르는 곳 ----
    def run_step(self, ctx: StepContext) -> dict[str, Any] | None:
        """단계의 결과 payload, 취소면 None. 주문을 멈춰야 하면 OrderFlowError를 올린다.

        단계 종류 이름으로 메서드를 찾는다(VALIDATE_ORDER → _validate_order)."""
        if ctx.cancel_event.wait(self.dwell_s):  # 빠른 단계도 화면에 잠깐 보이게 머문다
            return None
        handler = getattr(self, "_" + ctx.step["step_type"].lower())
        return handler(ctx)

    def release(self, task_id: str, *, stopped_pinky: str | None = None) -> None:
        """주문이 끝났다(또는 팔 일이 끝났다). 끝났거나 취소된 주문은 적재 자리를 비운다. 멈춘(실패한) 주문은 그
        핑키(stopped_pinky)가 아직 서 있을 수 있으니 자리를 쥔 채 held_by_stopped에 남긴다."""
        with self._guard:
            if self.station_owner == task_id:
                if stopped_pinky is not None:
                    self.held_by_stopped[task_id] = stopped_pinky
                else:
                    self.station_owner = None
            self._tokens.pop(task_id, None)
            self._order_pinky.pop(task_id, None)
            self._before.pop(task_id, None)
            self._picked.pop(task_id, None)

    def free_spot(self, task_id: str) -> bool:
        """운영자가 멈춘 주문을 확인했다(또는 그 핑키가 떠났다): 그 핑키는 더 이상 적재 자리를 막지 않는다.
        자리를 실제로 비웠으면 True."""
        with self._guard:
            self.held_by_stopped.pop(task_id, None)
            if self.station_owner == task_id:
                self.station_owner = None
                return True
            return False

    # ---- 도우미 ----
    @staticmethod
    def _pinky(ctx: StepContext) -> str:
        """이 단계를 실행할 때 주문이 쥔 핑키 번호."""
        return str(ctx.task["pinky_id"])

    @staticmethod
    def _name(pinky_id: str) -> str:
        """핑키 번호 → 화면 이름("11" → "P1")."""
        return PINKY_NAMES.get(pinky_id, pinky_id)

    @staticmethod
    def _observe(what: str, observe: Callable[..., dict[str, Any]], *args: Any) -> dict[str, Any]:
        """카메라(또는 모의)에서 개수를 받는다. 못 받으면 관측기의 error_code(없으면 COUNT_FAILED)로 주문을 멈춘다."""
        try:
            return observe(*args)
        except OrderFlowError:
            raise
        except Exception as exc:
            raise OrderFlowError(getattr(exc, "error_code", "COUNT_FAILED"), f"{what}을(를) 세지 못했습니다: {exc}",
                                 "cameras") from exc

    def _ordered(self, ctx: StepContext) -> Counter:
        """주문 전체의 물체별 수량."""
        return Counter({item["object_class"]: int(item["quantity"]) for item in ctx.task["order_items"]})

    @staticmethod
    def _shipment_load(ctx: StepContext) -> Counter:
        """이 단계 운송분의 핑키가 실을 것: 그 운송분의 PICK_ITEM 단위들(나누지 않은 주문은 주문 전체)."""
        number = shipment_of(ctx.step)
        return Counter(step["detail"]["object_class"] for step in ctx.task["steps"]
                       if step["step_type"] == "PICK_ITEM" and shipment_of(step) == number)

    def _confirm(self, ctx: StepContext, action: str) -> dict[str, Any]:
        """핑키 관제에 확인 신호(dock_complete, received)를 지금 임무 토큰과 함께 보낸다. 거절·불통이면 주문을 멈춘다."""
        pinky = self._pinky(ctx)
        token = self._tokens.get(ctx.task["id"])
        try:
            answer = self.devices.fleet_confirm(pinky, action, token)
        except PinkyFleetError as exc:
            raise OrderFlowError(exc.error_code, f"핑키 {self._name(pinky)} {action} 신호 실패: {exc}", "pinky_fleet") from exc
        except OSError as exc:
            raise OrderFlowError("PINKY_FLEET_UNREACHABLE", f"핑키 관제에 연결하지 못했습니다: {exc}", "pinky_fleet") from exc
        ctx.record("PINKY_SIGNAL_SENT", {"pinky_id": pinky, "action": action, "answer": answer,
                                         "message": f"핑키 관제에 {action} 전달"})
        return answer

    # ---- 단계 ----
    def _validate_order(self, ctx: StepContext) -> dict[str, Any]:
        """팔 설정·정책 파일이 있고 주문한 물체마다 정책이 있는지 본 뒤, 핑키를 출발시킨다."""
        if self.arms is None:
            raise OrderFlowError("ARMS_NOT_CONFIGURED", "팔 설정(config/arms.json)이 없어 주문을 진행할 수 없습니다")
        if self.policies is None:
            raise OrderFlowError("ACT_POLICIES_NOT_CONFIGURED", "정책 정의 파일이 없어 주문을 진행할 수 없습니다")
        missing = [name for name in self._ordered(ctx) if not self.policies.matches(name)]
        if missing:
            raise OrderFlowError("ORDER_POLICY_NOT_FOUND", f"학습된 정책이 없는 물체: {', '.join(missing)}")
        if self.devices.dispatch is not None:
            self._dispatch(ctx)
        return {"source": "control_station", "outcome": "ORDER_VALID", "order_items": ctx.task["order_items"]}

    def _dispatch(self, ctx: StepContext, pinky: str | None = None) -> None:
        """핑키를 경로로 출발시킨다: 주차장 → 5번 정류장 → 주문의 수령 구역. pinky를 주면 그 핑키(뒤 운송분)."""
        pinky, zone = pinky or self._pinky(ctx), ctx.task.get("receiving_zone")
        labels = self.route_for(zone)
        try:
            answer = self.devices.dispatch(pinky, labels) or {}
        except PinkyFleetError as exc:
            raise OrderFlowError(exc.error_code, f"핑키 {self._name(pinky)} 출발 실패: {exc}", "pinky_fleet") from exc
        except OSError as exc:
            raise OrderFlowError("PINKY_FLEET_UNREACHABLE", f"핑키 관제에 연결하지 못했습니다: {exc}", "pinky_fleet") from exc
        simulated = bool(answer.get("simulated"))
        where = f"5번을 거쳐 {zone}까지" if labels else "5번으로"
        ctx.record("PINKY_DISPATCHED", {"pinky_id": pinky, "route": labels, "simulated": simulated,
                                        "message": ("모의: " if simulated else "") + f"핑키 {self._name(pinky)} 출발, {where}"})

    def _check_shelf_stock(self, ctx: StepContext) -> dict[str, Any]:
        """선반 합계가 주문 수량 이상인지(선반 구분 없이). 어느 선반의 어느 팔이 집을지는 다음 단계가 본다."""
        observation = self._observe("선반 재고", self.devices.shelf_stock)
        totals: Counter = Counter()
        for counts in observation["shelves"].values():
            totals.update(counts)
        ordered = self._ordered(ctx)
        short = {name: (count, totals.get(name, 0)) for name, count in ordered.items() if totals.get(name, 0) < count}
        ctx.record("SHELF_STOCK_OBSERVED", {"observation": observation, "ordered": dict(ordered), "short": short})
        if short:
            text = ", ".join(f"{_label(self.policies, name)} 주문 {want} / 선반 {have}" for name, (want, have) in short.items())
            raise OrderFlowError("SHELF_STOCK_SHORT", f"선반 재고 부족: {text}", observation.get("source", "shelf_camera"))
        return {"source": observation.get("source"), "outcome": "STOCK_CONFIRMED", "shelves": observation["shelves"],
                "simulated": observation.get("simulated", False)}

    def _plan_arm_sequence(self, ctx: StepContext) -> dict[str, Any]:
        """배정 규칙(control_station.orders.assignment.plan_arms)으로 단위마다 팔을 정한다.

        계획은 실행 순서대로 돌아오고, PICK_ITEM/VERIFY_TRANSFER 쌍에 단계 순서대로 적는다. 그래서 단계가 계획대로
        정확히 돈다. 선반 개수는 앞 단계(CHECK_SHELF_STOCK)가 본 값을 다시 쓰고, 없을 때만 새로 읽는다."""
        checked = next((step.get("result") for step in ctx.task["steps"] if step["step_type"] == "CHECK_SHELF_STOCK"), None)
        shelves = (checked or {}).get("shelves") or self._observe("선반 재고", self.devices.shelf_stock)["shelves"]
        steps = ctx.task["steps"]
        pairs = [(step, steps[index + 1]) for index, step in enumerate(steps) if step["step_type"] == "PICK_ITEM"]
        # 운송분마다 따로 계획한다(그 단위는 그 핑키에 실린다). 차례대로, 앞 운송분이 선반에 남긴 재고로.
        stock = {shelf: Counter(counts) for shelf, counts in shelves.items()}
        units: list[dict[str, Any]] = []
        summaries = []
        for number in sorted({shipment_of(pick) for pick, _ in pairs}):
            mine = [(pick, verify) for pick, verify in pairs if shipment_of(pick) == number]
            tag = {key: mine[0][0]["detail"][key] for key in ("shipment", "shipments") if key in mine[0][0]["detail"]}
            try:
                plan = plan_arms(
                    [pick["detail"] for pick, _ in mine], stock, self.arms.arms, self.policies.resolve,
                    lambda object_class: _label(self.policies, object_class), self.pick_stats(),
                )
            except ArmPlanError as exc:
                raise OrderFlowError(exc.error_code, str(exc)) from exc
            for pick in plan.picks:
                stock[pick.shelf_id][pick.object_class] -= 1
            planned = [{**pick.as_dict(), **tag} for pick in plan.picks]
            for (pick_step, verify_step), unit in zip(mine, planned, strict=True):
                ctx.set_plan(pick_step["id"], unit["arm_id"], unit)
                ctx.set_plan(verify_step["id"], None, unit)
            units += planned
            summaries.append(f"운송 {number}: {plan.summary}" if tag else plan.summary)
        summary = " / ".join(summaries)
        sequence = [f"{unit['arm_label']} · {unit['label']} #{unit['unit']}" for unit in units]
        ctx.record("PLAN_CREATED", {"sequence": units, "summary": summary, "message": " → ".join(sequence)})
        return {"source": "control_station", "outcome": "PLAN_READY", "sequence": units, "summary": summary}

    def _start_shipment(self, ctx: StepContext) -> dict[str, Any] | None:
        """다음 운송분을 연다: 앞 핑키는 주차됐다(그 단계들이 이 단계 앞에 있다). 그러니 이 적재분의 핑키를 지금의
        핑키 상태로 골라 출발시킨다. 빈 핑키가 없으면 기다리고, arrival_timeout_s를 넘으면 SHIPMENT_PINKY_TIMEOUT."""
        task_id, number = ctx.task["id"], shipment_of(ctx.step)
        count = int((ctx.step.get("detail") or {}).get("shipments") or number)
        if self.choose_shipment_pinky is None:
            raise OrderFlowError("SHIPMENTS_NOT_CONFIGURED", "다음 운송분에 핑키를 정할 수 없습니다(주문 창구 연결 없음)")
        deadline = time.monotonic() + self.arrival_timeout_s
        waiting_reason = None
        while True:
            if ctx.cancel_event.is_set():
                return None
            choice = self.choose_shipment_pinky(task_id, number)
            pinky = choice.get("pinky_id")
            if pinky is not None:
                break
            if choice.get("summary") != waiting_reason:  # 기다리는 이유가 바뀔 때만 기록한다
                waiting_reason = choice.get("summary")
                ctx.record("SHIPMENT_WAITING_PINKY", {"shipment": number,
                                                      "message": f"운송 {number}/{count}: 빈 핑키 대기 · {waiting_reason}"})
            if time.monotonic() > deadline:
                raise OrderFlowError("SHIPMENT_PINKY_TIMEOUT",
                                     f"운송 {number}/{count}에 보낼 핑키가 {int(self.arrival_timeout_s)}초 동안 나오지 않았습니다",
                                     "pinky_fleet")
            ctx.cancel_event.wait(self.poll_s)
        ctx.record("SHIPMENT_STARTED", {"shipment": number, "shipments": count, "pinky_id": pinky,
                                        "message": f"운송 {number}/{count} 시작 · 핑키 {self._name(pinky)} · {choice.get('summary')}"})
        if self.devices.dispatch is not None:
            self._dispatch(ctx, pinky)
        return {"source": "control_station", "outcome": "SHIPMENT_STARTED", "shipment": number, "shipments": count,
                "pinky_id": pinky}

    def holds_pinky(self, pinky_id: str) -> bool:
        """진행 중인 주문의 흐름이 이 핑키의 도착과 신호를 직접 다루는 동안 True(테스트 화면은 손대지 않는다)."""
        with self._guard:
            return pinky_id in self._order_pinky.values()

    def _wait_pinky_arrival(self, ctx: StepContext) -> dict[str, Any] | None:
        """핑키가 적재 자리에 제대로 설 때까지 기다린다. 팔이 움직이기 전의 마지막 관문이다.

        한 바퀴마다: 핑키 관제 상태를 보고 → 핑키가 정류장(docking_manual)에 섰으면 적재 자리 주인을 정하고 →
        주인이면 천장 카메라 판정을 DockHandshake에 넣어 다음 할 일(통과·redock·운영자에게 묻기·기다리기)을 받는다.
        통과하면 PINKY_ARRIVED와 그때의 임무 토큰을 남긴다. arrival_timeout_s 안에 못 서면 PINKY_ARRIVAL_TIMEOUT."""
        pinky, task_id = self._pinky(ctx), ctx.task["id"]
        with self._guard:
            self._order_pinky[task_id] = pinky
        deadline = time.monotonic() + self.arrival_timeout_s
        last_phase = last_parking = waiting_for = fleet_error = None
        handshake = DockHandshake(arrived_checks=self.arrived_checks, misaligned_checks=self.redock_after_checks,
                                  max_redocks=self.max_redocks, blind_s=self.camera_blind_s,
                                  redock_wait_s=self.redock_wait_s)
        name = self._name(pinky)
        while True:
            if ctx.cancel_event.is_set():
                return None
            if time.monotonic() > deadline:
                raise OrderFlowError("PINKY_ARRIVAL_TIMEOUT",
                                     f"핑키 {self._name(pinky)}가 {int(self.arrival_timeout_s)}초 안에 적재 위치에 정차하지 않았습니다", "pinky_fleet")
            try:
                status = self.devices.fleet_status(pinky)
                fleet_error = None
            except (PinkyFleetError, OSError) as exc:
                if fleet_error is None:
                    ctx.record("PINKY_FLEET_UNREACHABLE", {"pinky_id": pinky, "error": str(exc), "alert": True,
                                                           "message": f"핑키 관제 연결 오류: {exc}"})
                fleet_error = str(exc)
                ctx.cancel_event.wait(self.poll_s)
                continue
            if status["phase"] != last_phase:
                last_phase = status["phase"]
                ctx.record("PINKY_STATUS", {"pinky_id": pinky, "phase": status["phase"], "message": status.get("message"),
                                            "location": status.get("location"), "simulated": status.get("simulated", False)})
            if status["phase"] != "docking_manual":
                handshake.step(status["phase"], None)  # redock 진행 중: 핑키가 자리를 떠났다
            if status["phase"] == "docking_manual":
                # 정류장에 먼저 선 핑키의 주문이 적재 자리를 가진다. 잠금은 주인을 보고 정할 때만 잡고, 기다리는
                # 동안에는 절대 쥐지 않는다.
                with self._guard:
                    owner = self.station_owner
                    held_pinky = self.held_by_stopped.get(owner) if owner else None
                if held_pinky is not None:
                    # 멈춘 주문의 핑키가 스스로 정류장을 떠났다: 자리가 다시 빈다.
                    try:
                        gone = self.devices.fleet_status(held_pinky)["phase"] not in STATION_PHASES
                    except (PinkyFleetError, OSError):
                        gone = False
                    if gone:
                        self.free_spot(owner)
                        owner = None
                with self._guard:
                    if self.station_owner is None:
                        self.station_owner = task_id
                    owner = self.station_owner
                if owner != task_id:
                    if waiting_for != owner:
                        waiting_for = owner
                        message = (
                            f"적재 자리 대기 · 멈춘 주문 {owner}의 핑키 {self._name(held_pinky)}가 아직 서 있습니다. 현장 확인 뒤 경보를 [확인]하세요"
                            if held_pinky else f"적재 자리 대기 · {owner} 적재 중"
                        )
                        ctx.record("STATION_BUSY", {"pinky_id": pinky, "busy_with": owner, "message": message,
                                                    "alert": bool(held_pinky)})
                    ctx.cancel_event.wait(self.poll_s)
                    continue
                try:
                    observation = self.devices.parking(pinky)
                except Exception as exc:  # 노드 꺼짐, 화면 움직임, 프레임 없음: 계속 기다리며 이유를 보인다
                    observation = {"status": getattr(exc, "error_code", "PARKING_CHECK_FAILED"), "reason": str(exc)}
                # 핑키에게 무엇을 돌려보낼지는 pinky/docking.py가 정한다(ARRIVED 두 번 → 다음으로; 계속 MISALIGNED →
                # 카메라가 잰 어긋남과 함께 redock, 그 뒤 핑키가 떠났다 돌아올 때까지 기다림; 등록 없음, 카메라가 못 봄,
                # redock을 다 씀 → 운영자가 보고 확인).
                decision = handshake.step("docking_manual", observation)
                offset = observation.get("offset") or {}
                distance = offset.get("distance_mm")
                if decision.result is not None:
                    ctx.record("PINKY_REDOCK_RESULT", {"pinky_id": pinky, **decision.result, "observation": observation,
                                                       "message": f"핑키 {name} {_result_text(decision.result)}"})
                if decision.not_applied:
                    ctx.record("PINKY_REDOCK_NOT_APPLIED", {
                        "pinky_id": pinky, "alert": True,
                        "message": f"핑키 {name}에 다시 맞추기를 보냈지만 {int(self.redock_wait_s)}초 동안 움직이지 않았어요"})
                if decision.action == COMPLETE:
                    with self._guard:
                        self._arrivals.pop(task_id, None)
                    self._tokens[task_id] = status.get("mission_token")
                    ctx.record("PARKING_OBSERVED", {"observation": observation, "pinky_id": pinky,
                                                    "redocks": handshake.redocks,
                                                    "message": f"핑키 {name} 정상 위치 · 다음 단계로"})
                    return {"source": "parking_check", "outcome": "PINKY_ARRIVED", "pinky_id": pinky,
                            "parking": observation, "redocks": handshake.redocks,
                            "simulated": observation.get("simulated", False)}
                if decision.action == OPERATOR:
                    # 카메라로는 정할 수 없다(2026-11-17 기준 등록된 마커가 있는 핑키는 P2뿐). 그래서 운영자가 보고
                    # 확인한다. 그 전에는 여전히 팔이 움직이지 않는다.
                    with self._guard:
                        waiting = self._arrivals.setdefault(task_id, threading.Event())
                    if last_parking != decision.reason:  # 같은 이유로는 한 번만 묻는다
                        last_parking = decision.reason
                        why = ("마커가 없어 카메라로 확인할 수 없어요" if decision.reason == "NOT_REGISTERED"
                               else decision.message.split(" · ")[0])
                        ctx.record("ARRIVAL_NEEDS_OPERATOR", {
                            "pinky_id": pinky, "observation": observation, "reason": decision.reason,
                            "message": f"핑키 {name}가 적재 자리에 섰어요 · {why}"
                                       " · 제자리에 섰는지 보고 [도착 확인]을 눌러 주세요"})
                    if waiting.is_set():
                        with self._guard:
                            self._arrivals.pop(task_id, None)
                        self._tokens[task_id] = status.get("mission_token")
                        confirmed = {**observation, "status": "OPERATOR_CONFIRMED", "camera_status": observation.get("status")}
                        ctx.record("PARKING_OBSERVED", {"observation": confirmed, "pinky_id": pinky,
                                                        "redocks": handshake.redocks,
                                                        "message": f"핑키 {name} 도착 · 운영자 확인 · 다음 단계로"})
                        return {"source": "operator", "outcome": "PINKY_ARRIVED", "pinky_id": pinky,
                                "parking": confirmed, "redocks": handshake.redocks, "simulated": False}
                    ctx.cancel_event.wait(self.poll_s)
                    continue
                with self._guard:
                    # 카메라가 다시 판정한다: 더 이상 운영자에게 묻지 않는다. 다만 이 판정 직전에 누른 [도착 확인]은
                    # 그대로 인정한다(NOT_FOUND 한 프레임이 누른 것을 삼키면 안 된다).
                    pressed = self._arrivals.pop(task_id, None)
                if pressed is not None and pressed.is_set():
                    self._tokens[task_id] = status.get("mission_token")
                    confirmed = {**observation, "status": "OPERATOR_CONFIRMED", "camera_status": observation.get("status")}
                    ctx.record("PARKING_OBSERVED", {"observation": confirmed, "pinky_id": pinky,
                                                    "redocks": handshake.redocks,
                                                    "message": f"핑키 {name} 도착 · 운영자 확인 · 다음 단계로"})
                    return {"source": "operator", "outcome": "PINKY_ARRIVED", "pinky_id": pinky,
                            "parking": confirmed, "redocks": handshake.redocks, "simulated": False}
                if decision.action == REDOCK:
                    # 카메라가 잰 값을 같이 보내, 핑키가 같은 실수를 되풀이하지 않고 그만큼 고쳐 들어오게 한다
                    # (2026-11-17). 핑키 기준 어긋남이 없으면 그냥 다시 들어온다.
                    guide = decision.guide
                    try:
                        answer = (self.devices.fleet_confirm(pinky, "redock", status.get("mission_token"), guide)
                                  if guide else self.devices.fleet_confirm(pinky, "redock", status.get("mission_token")))
                        guided = bool(guide and answer.get("guided"))
                        handshake.redock_sent(guided, None if distance is None else float(distance))
                        how = (f"카메라가 본 만큼 고쳐서(앞 {guide['forward_mm']:+.0f} mm, 왼쪽 {guide['left_mm']:+.0f} mm, "
                               f"{guide['dyaw_deg']:+.1f}°) " if guided else "")
                        ctx.record("PINKY_REDOCK", {
                            "pinky_id": pinky, "attempt": decision.attempt, "observation": observation, "answer": answer,
                            "guide": guide, "guided": guided,
                            "message": (f"핑키 {name} 카메라에 안 보임 · 다시 들어오라고 함 {decision.attempt}/{self.max_redocks}"
                                        if decision.reason == "NOT_SEEN" else
                                        f"핑키 {name} 위치 어긋남 · {how}다시 맞춰 들어가기 {decision.attempt}/{self.max_redocks}")})
                    except (PinkyFleetError, OSError) as exc:
                        handshake.redocks -= 1  # 보내지 못했다: redock 횟수를 쓰지 않은 것으로 되돌린다
                        ctx.record("PINKY_REDOCK_FAILED", {"pinky_id": pinky, "attempt": decision.attempt, "error": str(exc),
                                                           "alert": True, "message": f"다시 맞추기 요청 실패: {exc}"})
                    last_parking = None
                    ctx.cancel_event.wait(self.poll_s)
                    continue
                if decision.reason in {"MISALIGNED", "CAMERA_BLIND"}:
                    # 실제 어긋남은 판정마다 몇 mm씩 흔들린다: 판정마다가 아니라 상태와 1 cm 구간마다 경보 하나.
                    key = (observation.get("status"), None if distance is None else round(float(distance) / 10))
                    if key != last_parking:
                        last_parking = key
                        detail = (
                            f"오차 {float(distance):.0f} mm / {float(offset.get('dyaw_deg') or 0):.1f}°"
                            if distance is not None else str(observation.get("reason") or "")
                        )
                        ctx.record("PINKY_POSITION_ABNORMAL", {
                            "pinky_id": pinky, "observation": observation, "alert": True,
                            "message": f"핑키 {name} 위치 이상 · {observation.get('status')} · {detail}"
                                       " · 정상 위치에 오면 자동으로 다음 단계로 넘어갑니다",
                        })
            ctx.cancel_event.wait(self.poll_s)

    def _confirm_docking(self, ctx: StepContext) -> dict[str, Any] | None:
        """dock_complete를 보내고 핑키가 docked_manual(정지 유지)이 될 때까지 signal_timeout_s 기다린다.
        안 바뀌면 PINKY_SIGNAL_NOT_APPLIED. 바뀐 단계의 새 임무 토큰을 기억한다."""
        answer = self._confirm(ctx, "dock_complete")
        pinky = self._pinky(ctx)
        deadline = time.monotonic() + self.signal_timeout_s
        last_phase, last_message = None, None
        while time.monotonic() < deadline:
            if ctx.cancel_event.is_set():
                return None
            try:
                status = self.devices.fleet_status(pinky)
            except (PinkyFleetError, OSError):
                ctx.cancel_event.wait(self.poll_s)  # 한 번 놓친 조회일 뿐. 아래 마감은 그대로 적용된다
                continue
            if status["phase"] != last_phase:  # 예: 핑키가 5번에서 위치를 다시 잡는 동안 docking_anchor
                last_phase, last_message = status["phase"], status.get("message")
                ctx.record("PINKY_STATUS", {"pinky_id": pinky, "phase": status["phase"], "message": status.get("message"),
                                            "location": status.get("location"), "simulated": status.get("simulated", False)})
            if status["phase"] == "docked_manual":
                self._tokens[ctx.task["id"]] = status.get("mission_token")
                return {"source": "pinky_fleet", "outcome": "PINKY_HOLDING", "answer": answer}
            ctx.cancel_event.wait(self.poll_s)
        reason = f" (핑키 관제: {last_message})" if last_message else ""
        raise OrderFlowError("PINKY_SIGNAL_NOT_APPLIED",
                             f"핑키 {self._name(pinky)}가 도킹완료 뒤 정지 유지 단계로 바뀌지 않았습니다{reason}", "pinky_fleet")

    def _counts(self, ctx: StepContext, object_class: str) -> tuple[int, int, dict[str, Any], dict[str, Any]]:
        """(모든 선반의 그 물체 수, 핑키 위 그 물체 수, 선반 관측, 핑키 관측)."""
        shelf = self._observe("선반 재고", self.devices.shelf_stock)
        load = self._observe("핑키 위 물건", self.devices.pinky_load, self._pinky(ctx))
        on_shelves = sum(counts.get(object_class, 0) for counts in shelf["shelves"].values())
        return on_shelves, load["counts"].get(object_class, 0), shelf, load

    def _pick_item(self, ctx: StepContext) -> dict[str, Any] | None:
        """집기 전 개수를 기억하고, 계획된 팔에 단위 하나를 집게 한다. 옮겨졌는지는 VERIFY_TRANSFER가 판단한다."""
        unit = ctx.step["detail"] or {}
        if "arm_id" not in unit:
            raise OrderFlowError("PLAN_MISSING", "이 물체에 배정된 팔이 없습니다")
        on_shelves, on_pinky, shelf, load = self._counts(ctx, unit["object_class"])
        if self.devices.pick_is_real and (shelf.get("simulated") or load.get("simulated")):
            # 실제 팔은 실제 물체를 옮기는데 모의 개수는 그것을 볼 수 없다. 옮김을 확인할 수 없고, 팔의 다음 집기가
            # 확인 안 된 집기 뒤에 이어지게 된다. 팔이 움직이기 전에 멈춘다.
            raise OrderFlowError("TRANSFER_EVIDENCE_REQUIRED",
                                 "실제 팔로 집으려면 선반과 핑키 위 개수를 카메라로 세야 해요(지금은 모의 개수). "
                                 "CONTROL_STATION_ORDER_COUNTS=camera로 켜 주세요.", "cameras")
        self._before[ctx.task["id"]] = {"shelf": on_shelves, "pinky": on_pinky}
        ctx.record("ARM_PICK_STARTED", {"arm_id": unit["arm_id"], "unit": unit,
                                        "message": f"{unit['arm_label']}: {unit['label']} #{unit['unit']} 집기 시작"})
        result = self.devices.pick(
            unit["arm_id"], unit["object_class"], unit["shelf_id"], self._pinky(ctx), ctx.cancel_event,
            lambda message: ctx.record("ARM_PROGRESS", {"arm_id": unit["arm_id"], "message": message}),
        )
        if result is None:
            return None
        self._picked[ctx.task["id"]] = result
        return {**result, "unit": unit}

    def _verify_transfer(self, ctx: StepContext) -> dict[str, Any]:
        """집기 전과 비교해 (선반 변화, 핑키 변화)가 (−1, +1)인지 본다. 맞으면 팔의 미확인 명령을 닫는다(pick_settle)."""
        unit = ctx.step["detail"] or {}
        before = self._before.pop(ctx.task["id"], None)
        if before is None:
            raise OrderFlowError("TRANSFER_NOT_VERIFIED", "집기 전 개수가 없어 옮김을 판단할 수 없습니다")
        on_shelves, on_pinky, shelf, load = self._counts(ctx, unit["object_class"])
        shelf_delta, pinky_delta = on_shelves - before["shelf"], on_pinky - before["pinky"]
        verdict = {(-1, 1): "TRANSFERRED", (0, 0): "PICK_FAILED_STILL_ON_SHELF", (-1, 0): "ITEM_LOST"}.get(
            (shelf_delta, pinky_delta), "TRANSFER_NOT_VERIFIED"
        )
        name = f"{unit.get('label', unit.get('object_class'))} #{unit.get('unit')}"
        ctx.record("TRANSFER_OBSERVED", {
            "unit": unit, "verdict": verdict, "shelf_delta": shelf_delta, "pinky_delta": pinky_delta,
            "shelf": shelf, "pinky_load": load, "alert": verdict != "TRANSFERRED",
            "message": f"{name}: 선반 {shelf_delta:+d}, 핑키 {pinky_delta:+d}",
        })
        if verdict == "PICK_FAILED_STILL_ON_SHELF":
            raise OrderFlowError(verdict, f"{name}: 선반에 그대로 있습니다(집기 실패). 팔과 선반을 확인하세요.", "cameras")
        if verdict == "ITEM_LOST":
            raise OrderFlowError(verdict, f"{name}: 선반에도 핑키에도 없습니다(분실). 팔 주변 바닥과 핑키를 확인하세요.", "cameras")
        if verdict != "TRANSFERRED":
            raise OrderFlowError(verdict, f"{name}: 선반 {shelf_delta:+d}, 핑키 {pinky_delta:+d}로 옮김을 판단할 수 없습니다.", "cameras")
        picked = self._picked.pop(ctx.task["id"], None)
        if picked is not None and self.devices.pick_settle is not None:
            self.devices.pick_settle(picked, {"task_id": ctx.task["id"], "source": load.get("source"),
                                              "simulated": bool(load.get("simulated") or shelf.get("simulated"))})
        return {"source": load.get("source"), "outcome": "TRANSFERRED", "unit": unit, "simulated": load.get("simulated", False)}

    def _verify_load_total(self, ctx: StepContext) -> dict[str, Any]:
        """핑키 위 물체(짐이 아닌 것 제외)가 이 운송분의 적재 목록과 정확히 같은지. 부족·초과·섞임을 가려 멈춘다."""
        load = self._observe("핑키 위 물건", self.devices.pinky_load, self._pinky(ctx))
        seen = Counter({name: count for name, count in load["counts"].items() if name not in NOT_CARGO and count > 0})
        ordered = self._shipment_load(ctx)
        ok = seen == ordered
        ctx.record("LOAD_TOTAL_OBSERVED", {"pinky_load": load, "ordered": dict(ordered), "loaded": ok, "alert": not ok,
                                           "message": "핑키 위 " + (", ".join(f"{_label(self.policies, n)} {c}" for n, c in seen.items()) or "없음")})
        if not ok:
            extra = {name: count for name, count in (seen - ordered).items()}
            missing = {name: count for name, count in (ordered - seen).items()}
            code = "LOAD_EXTRA_ITEM" if extra and not missing else "LOAD_INCOMPLETE" if missing and not extra else "LOAD_MISMATCH"
            raise OrderFlowError(code, f"핑키 위 물건이 주문과 다릅니다 · 부족 {missing or '없음'} · 초과 {extra or '없음'}", "cameras")
        return {"source": load.get("source"), "outcome": "LOAD_CONFIRMED", "loaded": dict(seen),
                "simulated": load.get("simulated", False)}

    def confirm_arrival(self, task_id: str) -> bool:
        """운영자가 주문의 핑키(마커 없음 또는 카메라 판정 불가)가 적재 자리에 제대로 선 것을 봤다. 주문이 그것을
        기다리고 있지 않으면 False."""
        with self._guard:
            waiting = self._arrivals.get(task_id)
        if waiting is None:
            return False
        waiting.set()
        return True

    def confirm_pickup(self, task_id: str) -> bool:
        """손님(또는 운영자)이 수령 구역에서 수령을 확인했다. 주문의 핑키가 아직 거기서 기다리지 않으면 False."""
        with self._guard:
            waiting = self._pickups.get(task_id)
        if waiting is None:
            return False
        waiting.set()
        return True

    def _wait_phase(self, ctx: StepContext, wanted: str, timeout_s: float, timeout_code: str,
                    timeout_message: str) -> dict[str, Any] | None:
        """핑키의 임무 단계가 `wanted`가 될 때까지 본다. 단계가 바뀔 때마다 기록한다. 취소면 None, 시간 초과면
        timeout_code로 주문을 멈춘다."""
        pinky = self._pinky(ctx)
        deadline = time.monotonic() + timeout_s
        last_phase = None
        while time.monotonic() < deadline:
            if ctx.cancel_event.is_set():
                return None
            try:
                status = self.devices.fleet_status(pinky)
            except (PinkyFleetError, OSError):
                ctx.cancel_event.wait(self.poll_s)  # 한 번 놓친 조회일 뿐. 마감은 그대로 적용된다
                continue
            if status["phase"] != last_phase:
                last_phase = status["phase"]
                ctx.record("PINKY_STATUS", {"pinky_id": pinky, "phase": status["phase"], "message": status.get("message"),
                                            "location": status.get("location"), "simulated": status.get("simulated", False)})
            if status["phase"] == wanted:
                return status
            ctx.cancel_event.wait(self.poll_s)
        raise OrderFlowError(timeout_code, timeout_message, "pinky_fleet")

    def _wait_at_receiving_zone(self, ctx: StepContext) -> dict[str, Any] | None:
        """핑키가 수령 구역(경로 끝)에 닿아 course_complete가 될 때까지 기다린다. 그때의 임무 토큰을 기억한다."""
        status = self._wait_phase(ctx, "course_complete", self.arrival_timeout_s, "PINKY_ZONE_TIMEOUT",
                                  f"핑키 {self._name(self._pinky(ctx))}가 수령 구역에 도착하지 않았습니다")
        if status is None:
            return None
        self._tokens[ctx.task["id"]] = status.get("mission_token")
        return {"source": "pinky_fleet", "outcome": "AT_RECEIVING_ZONE", "zone": ctx.task.get("receiving_zone"),
                "simulated": status.get("simulated", False)}

    def _confirm_pickup(self, ctx: StepContext) -> dict[str, Any] | None:
        """손님의 수령 확인을 기다렸다가, 핑키가 집으로 가도록 mission-complete를 보낸다. pickup_timeout_s 한도."""
        task_id, pinky = ctx.task["id"], self._pinky(ctx)
        waiting = threading.Event()
        with self._guard:
            self._pickups[task_id] = waiting
        ctx.record("PICKUP_WAITING", {"pinky_id": pinky, "zone": ctx.task.get("receiving_zone"),
                                      "message": f"{ctx.task.get('destination')} 도착 · 손님 수령 확인을 기다립니다"})
        try:
            deadline = time.monotonic() + self.pickup_timeout_s
            while not waiting.wait(self.poll_s):
                if ctx.cancel_event.is_set():
                    return None
                if time.monotonic() > deadline:
                    raise OrderFlowError("PICKUP_TIMEOUT", "손님 수령 확인이 오지 않았습니다. 수령 구역을 확인하세요")
        finally:
            with self._guard:
                self._pickups.pop(task_id, None)
        if self.devices.mission_complete is None:
            raise OrderFlowError("PINKY_RETURN_NOT_CONFIGURED", "핑키에 주행완료를 보낼 수 없습니다", "pinky_fleet")
        token = self._tokens.get(task_id)
        try:
            answer = self.devices.mission_complete(pinky, token)
        except PinkyFleetError as exc:
            raise OrderFlowError(exc.error_code, f"핑키 {self._name(pinky)} 주행완료 신호 실패: {exc}", "pinky_fleet") from exc
        except OSError as exc:
            raise OrderFlowError("PINKY_FLEET_UNREACHABLE", f"핑키 관제에 연결하지 못했습니다: {exc}", "pinky_fleet") from exc
        ctx.record("PINKY_SIGNAL_SENT", {"pinky_id": pinky, "action": "mission-complete", "answer": answer,
                                         "message": "수령 확인 · 핑키 관제에 주행완료 전달"})
        return {"source": "control_station", "outcome": "PICKUP_CONFIRMED", "answer": answer}

    def _wait_pinky_parked(self, ctx: StepContext) -> dict[str, Any] | None:
        """핑키가 주차장으로 돌아와 parked가 될 때까지 기다린다."""
        status = self._wait_phase(ctx, "parked", self.arrival_timeout_s, "PINKY_RETURN_TIMEOUT",
                                  f"핑키 {self._name(self._pinky(ctx))}가 주차장으로 돌아오지 않았습니다")
        if status is None:
            return None
        return {"source": "pinky_fleet", "outcome": "PINKY_PARKED", "simulated": status.get("simulated", False)}

    def _notify_pinky_received(self, ctx: StepContext) -> dict[str, Any]:
        """핑키 관제에 received를 보내고 적재 자리를 비운다. 핑키가 실제로 출발했는지는 다음 단계가 상태로 본다."""
        answer = self._confirm(ctx, "received")
        self.release(ctx.task["id"])  # 팔은 이 핑키와 일을 마쳤다. 다음 주문이 자리를 써도 된다
        return {"source": "pinky_fleet", "outcome": "RECEIVED_SIGNAL_ACCEPTED", "answer": answer,
                "message": "핑키 관제가 수령 완료 신호를 받았습니다. 출발은 핑키 상태로 확인합니다."}
