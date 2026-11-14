"""The order flow: shelf stock -> arm plan -> wait for the Pinky -> pick and check each object -> total -> signal.

Steps (made per order by store.order_step_definitions):
  VALIDATE_ORDER       objects have policies, arms are configured; the mock world sends the Pinky off
  CHECK_SHELF_STOCK    shelf cameras see at least the ordered number of each object (SHELF_STOCK_SHORT otherwise)
  PLAN_ARM_SEQUENCE    each unit goes to the arm on the shelf where it was seen (left shelf -> left arm), grouped
                       by arm; written into the PICK_ITEM/VERIFY_TRANSFER steps
  WAIT_PINKY_ARRIVAL   the loading spot is free for this order, the Pinky fleet reports the Pinky stopped at the
                       station (docking_manual) and the ceiling camera judges it ARRIVED. A misaligned Pinky raises
                       PINKY_POSITION_ABNORMAL and the step keeps waiting; the arms never move before this passes.
  CONFIRM_DOCKING      dock_complete to the Pinky fleet, which then holds the Pinky (docked_manual)
  PICK_ITEM            the planned arm picks one unit with its trained policy and puts it on the Pinky
  VERIFY_TRANSFER      shelf count -1 and Pinky count +1; still on the shelf -> PICK_FAILED_STILL_ON_SHELF;
                       on neither -> ITEM_LOST; anything else -> TRANSFER_NOT_VERIFIED. All stop the order.
  VERIFY_LOAD_TOTAL    the Pinky carries exactly the ordered objects
  NOTIFY_PINKY_RECEIVED  received to the Pinky fleet; an accepted request means the dashboard took the signal,
                       not that the Pinky left
  WAIT_AT_RECEIVING_ZONE / CONFIRM_PICKUP / WAIT_PINKY_PARKED  the Pinky reaches the zone, the customer confirms,
                       the Pinky drives home
An order larger than one tray runs as shipments, one after another (store.order_step_definitions): the steps from
WAIT_PINKY_ARRIVAL to WAIT_PINKY_PARKED repeat per shipment, and START_SHIPMENT opens each later one once the
previous Pinky is parked again (one Pinky drives at a time), choosing that load's Pinky and sending it off.
Failures stop the order with no retry and no signal to the Pinky; the operator sees the alert and decides. The
shipments after a failure never start.
The devices behind the observers and the picker are plugged in from outside (the mock world for now).
"""

from __future__ import annotations

import threading
import time
from collections import Counter
from dataclasses import dataclass
from typing import Any, Callable

from control_station.assignment import ArmPlanError, PickStats, plan_arms
from control_station.pinky_fleet import STATION_PHASES, PinkyFleetError
from control_station.store import PINKY_NAMES, shipment_of

ORDER_STEP_TYPES = frozenset({
    "VALIDATE_ORDER", "CHECK_SHELF_STOCK", "PLAN_ARM_SEQUENCE", "WAIT_PINKY_ARRIVAL", "CONFIRM_DOCKING",
    "PICK_ITEM", "VERIFY_TRANSFER", "VERIFY_LOAD_TOTAL", "NOTIFY_PINKY_RECEIVED",
    "WAIT_AT_RECEIVING_ZONE", "CONFIRM_PICKUP", "WAIT_PINKY_PARKED", "START_SHIPMENT",
})
# Objects the load camera sees that are not cargo (YOLO calls Pinky itself a car).
NOT_CARGO = frozenset({"car"})


class OrderFlowError(RuntimeError):
    """The order stops here; error_code is what the operator alert shows."""

    def __init__(self, error_code: str, message: str, source: str = "control_station") -> None:
        super().__init__(message)
        self.error_code = error_code
        self.source = source


@dataclass
class StepContext:
    """What a step may do to the task while it runs, provided by the orchestrator."""

    task: dict[str, Any]
    step: dict[str, Any]
    cancel_event: threading.Event
    record: Callable[[str, dict[str, Any]], None]  # (event_type, payload) on this step
    set_plan: Callable[[str, str | None, dict[str, Any]], None]  # (step_id, robot_id, detail) for a pending step


@dataclass
class OrderDevices:
    """Observers and actuators the flow uses; the mock world provides all of them for now."""

    shelf_stock: Callable[[], dict[str, Any]]
    pinky_load: Callable[[str], dict[str, Any]]
    fleet_status: Callable[[str], dict[str, Any]]
    fleet_confirm: Callable[[str, str, str | None], dict[str, Any]]
    parking: Callable[[str], dict[str, Any]]
    pick: Callable[..., dict[str, Any] | None]
    # Start the Pinky's route (pinky id, waypoint labels up to the receiving zone); None leaves it to the fleet.
    dispatch: Callable[[str, list[str] | None], Any] | None = None
    # Pickup confirmed at the receiving zone: tell the Pinky to drive home (pinky id, mission token).
    mission_complete: Callable[[str, str | None], dict[str, Any]] | None = None
    # True when `pick` moves a real arm (arm_pick.ArmQueuePick). Each transfer must then be counted by cameras.
    pick_is_real: bool = False
    # After VERIFY_TRANSFER saw the object move: (pick result, evidence) closes the arm's unverified command.
    pick_settle: Callable[[dict[str, Any], dict[str, Any]], None] | None = None


def _label(policies: Any, object_class: str) -> str:
    if policies is not None:
        found = policies.matches(object_class)
        if found:
            return found[0]["label"]
    return object_class


class OrderFlow:
    def __init__(
        self,
        arms: Any,
        policies: Any,
        devices: OrderDevices,
        *,
        dwell_s: float = 0.25,
        poll_s: float = 0.5,
        arrival_timeout_s: float = 600.0,
        # dock_complete -> docked_manual passes through docking_anchor, which the Pinky allows up to 18 s.
        signal_timeout_s: float = 30.0,
        pick_stats: Callable[[], PickStats] | None = None,
        route_for: Callable[[str | None], list[str] | None] | None = None,
        pickup_timeout_s: float = 1800.0,
        choose_shipment_pinky: Callable[[str, int], dict[str, Any]] | None = None,
    ) -> None:
        self.arms = arms
        # (order id, shipment number) -> {"pinky_id": ... or None, "summary": ...}: the Pinky for a later shipment,
        # chosen and written on the order by the order desk (OrderDesk.assign_next_shipment). None: no shipments.
        self.choose_shipment_pinky = choose_shipment_pinky
        # Waypoint labels from parking to an order's receiving zone (config/receiving-zones.json).
        self.route_for = route_for or (lambda zone_id: None)
        self.pickup_timeout_s = pickup_timeout_s
        # Orders whose Pinky waits at the receiving zone: order id -> set when the customer confirms pickup.
        self._pickups: dict[str, threading.Event] = {}
        # Past pick results per (policy, arm) for the arm choice; none -> every arm starts at the same rate.
        self.pick_stats = pick_stats or dict
        self.policies = policies
        self.devices = devices
        self.dwell_s = dwell_s
        self.poll_s = poll_s
        self.arrival_timeout_s = arrival_timeout_s
        self.signal_timeout_s = signal_timeout_s
        # One Pinky can stand between the two arms, so one order at a time uses the arms and the spot.
        self.station_owner: str | None = None
        # A stopped order keeps the spot (its Pinky is still standing there) until the operator acknowledges the
        # alert or that Pinky leaves the station: order id -> Pinky id.
        self.held_by_stopped: dict[str, str] = {}
        self._tokens: dict[str, str | None] = {}
        self._before: dict[str, dict[str, int]] = {}
        self._picked: dict[str, dict[str, Any]] = {}  # task id -> the last pick's result, settled by VERIFY_TRANSFER
        self._guard = threading.Lock()

    # ---- orchestrator hooks ----
    def run_step(self, ctx: StepContext) -> dict[str, Any] | None:
        """Result payload of the step, None when canceled; raises OrderFlowError to stop the order."""
        if ctx.cancel_event.wait(self.dwell_s):  # keeps quick steps visible on the screen
            return None
        handler = getattr(self, "_" + ctx.step["step_type"].lower())
        return handler(ctx)

    def release(self, task_id: str, *, stopped_pinky: str | None = None) -> None:
        """The order ended. A finished or canceled order frees the loading spot; a stopped (failed) order keeps it
        while its Pinky may still stand there (stopped_pinky)."""
        with self._guard:
            if self.station_owner == task_id:
                if stopped_pinky is not None:
                    self.held_by_stopped[task_id] = stopped_pinky
                else:
                    self.station_owner = None
            self._tokens.pop(task_id, None)
            self._before.pop(task_id, None)
            self._picked.pop(task_id, None)

    def free_spot(self, task_id: str) -> bool:
        """The operator acknowledged a stopped order: its Pinky no longer blocks the loading spot."""
        with self._guard:
            self.held_by_stopped.pop(task_id, None)
            if self.station_owner == task_id:
                self.station_owner = None
                return True
            return False

    # ---- helpers ----
    @staticmethod
    def _pinky(ctx: StepContext) -> str:
        return str(ctx.task["pinky_id"])

    @staticmethod
    def _name(pinky_id: str) -> str:
        return PINKY_NAMES.get(pinky_id, pinky_id)

    @staticmethod
    def _observe(what: str, observe: Callable[..., dict[str, Any]], *args: Any) -> dict[str, Any]:
        """A count from the cameras (or the mock); no count stops the order with the observer's error code."""
        try:
            return observe(*args)
        except OrderFlowError:
            raise
        except Exception as exc:
            raise OrderFlowError(getattr(exc, "error_code", "COUNT_FAILED"), f"{what}을(를) 세지 못했습니다: {exc}",
                                 "cameras") from exc

    def _ordered(self, ctx: StepContext) -> Counter:
        return Counter({item["object_class"]: int(item["quantity"]) for item in ctx.task["order_items"]})

    @staticmethod
    def _shipment_load(ctx: StepContext) -> Counter:
        """What the Pinky of this step's shipment carries: its PICK_ITEM units (the whole order when not split)."""
        number = shipment_of(ctx.step)
        return Counter(step["detail"]["object_class"] for step in ctx.task["steps"]
                       if step["step_type"] == "PICK_ITEM" and shipment_of(step) == number)

    def _confirm(self, ctx: StepContext, action: str) -> dict[str, Any]:
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

    # ---- steps ----
    def _validate_order(self, ctx: StepContext) -> dict[str, Any]:
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
        """Send the Pinky off on its route: parking -> station 5 -> the order's receiving zone."""
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
        """Give every unit an arm with the assignment rules (control_station.assignment.plan_arms).

        The plan comes back in execution order and is written onto the PICK_ITEM/VERIFY_TRANSFER pairs in their
        step order, so the steps run exactly as planned."""
        checked = next((step.get("result") for step in ctx.task["steps"] if step["step_type"] == "CHECK_SHELF_STOCK"), None)
        shelves = (checked or {}).get("shelves") or self._observe("선반 재고", self.devices.shelf_stock)["shelves"]
        steps = ctx.task["steps"]
        pairs = [(step, steps[index + 1]) for index, step in enumerate(steps) if step["step_type"] == "PICK_ITEM"]
        # Each shipment is planned on its own (its units stay on its Pinky), in order, each with the stock the
        # earlier ones leave on the shelves.
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
        """Open the next shipment: the previous Pinky is parked (its steps came before this one), so choose the
        Pinky for this load now, with the fleet state of this moment, and send it off. No free Pinky: wait."""
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
            if choice.get("summary") != waiting_reason:
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

    def _wait_pinky_arrival(self, ctx: StepContext) -> dict[str, Any] | None:
        pinky, task_id = self._pinky(ctx), ctx.task["id"]
        deadline = time.monotonic() + self.arrival_timeout_s
        last_phase = last_parking = waiting_for = fleet_error = None
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
            if status["phase"] == "docking_manual":
                # The Pinky that stopped at the station first takes the loading spot; the lock is held only to
                # look at and set the owner, never while waiting.
                with self._guard:
                    owner = self.station_owner
                    held_pinky = self.held_by_stopped.get(owner) if owner else None
                if held_pinky is not None:
                    # A stopped order's Pinky left the station by itself: the spot is free again.
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
                except Exception as exc:  # node down, view moved, no frame: keep waiting, show why
                    observation = {"status": getattr(exc, "error_code", "PARKING_CHECK_FAILED"), "reason": str(exc)}
                if observation.get("status") == "ARRIVED":
                    self._tokens[task_id] = status.get("mission_token")
                    ctx.record("PARKING_OBSERVED", {"observation": observation, "pinky_id": pinky,
                                                    "message": f"핑키 {self._name(pinky)} 정상 위치 · 다음 단계로"})
                    return {"source": "parking_check", "outcome": "PINKY_ARRIVED", "pinky_id": pinky,
                            "parking": observation, "simulated": observation.get("simulated", False)}
                offset = observation.get("offset") or {}
                distance = offset.get("distance_mm")
                # Real offsets move by a few mm every check: one alert per status and 1 cm band, not per check.
                key = (observation.get("status"), None if distance is None else round(float(distance) / 10))
                if key != last_parking:
                    last_parking = key
                    detail = (
                        f"오차 {float(distance):.0f} mm / {float(offset.get('dyaw_deg') or 0):.1f}°"
                        if distance is not None else str(observation.get("reason") or "")
                    )
                    ctx.record("PINKY_POSITION_ABNORMAL", {
                        "pinky_id": pinky, "observation": observation, "alert": True,
                        "message": f"핑키 {self._name(pinky)} 위치 이상 · {observation.get('status')} · {detail}"
                                   " · 정상 위치에 오면 자동으로 다음 단계로 넘어갑니다",
                    })
            ctx.cancel_event.wait(self.poll_s)

    def _confirm_docking(self, ctx: StepContext) -> dict[str, Any] | None:
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
                ctx.cancel_event.wait(self.poll_s)  # a missed poll; the deadline below still applies
                continue
            if status["phase"] != last_phase:  # e.g. docking_anchor while the Pinky re-anchors at station 5
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
        shelf = self._observe("선반 재고", self.devices.shelf_stock)
        load = self._observe("핑키 위 물건", self.devices.pinky_load, self._pinky(ctx))
        on_shelves = sum(counts.get(object_class, 0) for counts in shelf["shelves"].values())
        return on_shelves, load["counts"].get(object_class, 0), shelf, load

    def _pick_item(self, ctx: StepContext) -> dict[str, Any] | None:
        unit = ctx.step["detail"] or {}
        if "arm_id" not in unit:
            raise OrderFlowError("PLAN_MISSING", "이 물체에 배정된 팔이 없습니다")
        on_shelves, on_pinky, shelf, load = self._counts(ctx, unit["object_class"])
        if self.devices.pick_is_real and (shelf.get("simulated") or load.get("simulated")):
            # A real arm moves a real object; mock counts cannot see it, so the transfer could not be checked and
            # the arm's next pick would follow an unverified one. Stop before the arm moves.
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

    def confirm_pickup(self, task_id: str) -> bool:
        """The customer (or the operator) confirmed pickup at the receiving zone; False if the order's Pinky is
        not waiting there yet."""
        with self._guard:
            waiting = self._pickups.get(task_id)
        if waiting is None:
            return False
        waiting.set()
        return True

    def _wait_phase(self, ctx: StepContext, wanted: str, timeout_s: float, timeout_code: str,
                    timeout_message: str) -> dict[str, Any] | None:
        """Poll the Pinky's mission phase until `wanted`; records each phase change. None when canceled."""
        pinky = self._pinky(ctx)
        deadline = time.monotonic() + timeout_s
        last_phase = None
        while time.monotonic() < deadline:
            if ctx.cancel_event.is_set():
                return None
            try:
                status = self.devices.fleet_status(pinky)
            except (PinkyFleetError, OSError):
                ctx.cancel_event.wait(self.poll_s)  # a missed poll; the deadline still applies
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
        status = self._wait_phase(ctx, "course_complete", self.arrival_timeout_s, "PINKY_ZONE_TIMEOUT",
                                  f"핑키 {self._name(self._pinky(ctx))}가 수령 구역에 도착하지 않았습니다")
        if status is None:
            return None
        self._tokens[ctx.task["id"]] = status.get("mission_token")
        return {"source": "pinky_fleet", "outcome": "AT_RECEIVING_ZONE", "zone": ctx.task.get("receiving_zone"),
                "simulated": status.get("simulated", False)}

    def _confirm_pickup(self, ctx: StepContext) -> dict[str, Any] | None:
        """Wait for the customer's pickup confirmation, then send mission-complete so the Pinky goes home."""
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
        status = self._wait_phase(ctx, "parked", self.arrival_timeout_s, "PINKY_RETURN_TIMEOUT",
                                  f"핑키 {self._name(self._pinky(ctx))}가 주차장으로 돌아오지 않았습니다")
        if status is None:
            return None
        return {"source": "pinky_fleet", "outcome": "PINKY_PARKED", "simulated": status.get("simulated", False)}

    def _notify_pinky_received(self, ctx: StepContext) -> dict[str, Any]:
        answer = self._confirm(ctx, "received")
        self.release(ctx.task["id"])  # the arms are done with this Pinky; the next order may use the spot
        return {"source": "pinky_fleet", "outcome": "RECEIVED_SIGNAL_ACCEPTED", "answer": answer,
                "message": "핑키 관제가 수령 완료 신호를 받았습니다. 출발은 핑키 상태로 확인합니다."}
