"""The order desk: orders from the order window wait here until an operator starts them.

Receiving an order stores it without a Pinky and, unless an operator named one, without a receiving zone. The
preview shows which Pinky, which arms and which receiving zone would serve it right now, and why; starting runs the
same choices again with the state of that moment, records them on the order and hands the order to the task
orchestrator. The arm plan is made again inside the order flow, after the
shelf check, so the preview is advice and never a reservation. Nothing here moves a device.
"""

from __future__ import annotations

import hashlib
import json
import threading
from collections import Counter
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from typing import Any

from control_station.api.common import validate_order_items
from control_station.assignment import (
    ArmPlanError, PickStats, PinkyCandidate, ShipmentError, ZoneAssignment, ZoneCandidate, assign_fleet, choose_pinky,
    choose_zone, plan_arms, schedule_loading, split_into_shipments,
)
from control_station.fleet_planning import FleetPlanning, load_fleet_planning
from control_station.order_flow import OrderDevices
from control_station.pinky_fleet import PinkyFleetError
from control_station.receiving_zones import ReceivingZone
from control_station.store import (
    PINKY_IDS, PINKY_NAMES, InvalidStateTransition, PinkyUnavailable, TaskStore, ZoneUnavailable, shipment_of,
)


class OrderDeskError(RuntimeError):
    """A request the desk refuses; `status` is the HTTP status the API answers with."""

    def __init__(self, error_code: str, message: str, status: int = 409, **details: object) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.status = status
        self.details = details


# The customer's stages of one order, and which order-flow steps belong to each.
_CUSTOMER_STAGES = (("received", "접수"), ("preparing", "준비"), ("pinky_coming", "핑키 출발"),
                    ("loading", "싣는 중"), ("delivering", "가는 중"), ("arrived", "도착"), ("done", "완료"))
_STAGE_INDEX = {key: index for index, (key, _) in enumerate(_CUSTOMER_STAGES)}
_STEP_STAGE = {
    "VALIDATE_ORDER": "preparing", "CHECK_SHELF_STOCK": "preparing", "PLAN_ARM_SEQUENCE": "preparing",
    "WAIT_PINKY_ARRIVAL": "pinky_coming", "CONFIRM_DOCKING": "loading", "PICK_ITEM": "loading",
    "VERIFY_TRANSFER": "loading", "VERIFY_LOAD_TOTAL": "loading", "NOTIFY_PINKY_RECEIVED": "delivering",
    "WAIT_AT_RECEIVING_ZONE": "delivering", "CONFIRM_PICKUP": "arrived", "WAIT_PINKY_PARKED": "done",
    "FINALIZE_TASK": "done",
}
_CUSTOMER_TEXT = {
    "received": ("주문을 받았어요", "곧 준비를 시작해요"),
    "preparing": ("주문을 준비하고 있어요", "선반 재고를 확인하고 로봇을 배정해요"),
    "pinky_coming": ("핑키가 물건을 실으러 가고 있어요", "적재 자리에 도착하면 로봇 팔이 실어요"),
    "loading": ("로봇 팔이 물건을 싣고 있어요", "{moved}/{total}개 실었어요"),
    "delivering": ("핑키가 {zone}로 가고 있어요", "도착하면 알려 드릴게요"),
    "arrived": ("{zone}에 도착했어요", "물건을 받은 뒤 아래 버튼을 눌러 주세요"),
    "done": ("주문이 끝났어요", "이용해 주셔서 감사해요"),
    "canceled": ("주문이 취소됐어요", "다시 주문할 수 있어요"),
    "problem": ("주문 처리에 문제가 생겼어요", "직원이 확인하고 있어요"),
}
# What the customer sees in place of the zone until the order starts and the server picks one.
ZONE_PENDING_NOTE = "받을 곳은 로봇이 출발할 때 알려 드려요"
# An order's destination until its zone is assigned (shown on the operator screen).
ZONE_PENDING_LABEL = "수령 구역 (시작 때 배정)"


def _request_hash(order_items: list[dict[str, Any]], zone_id: object) -> str:
    """Fingerprint of what an order asks for, to tell a resent request from a different one with the same key."""
    body = json.dumps({"items": order_items, "receiving_zone": zone_id}, sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(body.encode("utf-8")).hexdigest()


def _customer_stage(status: str, step_type: str | None) -> str:
    if status == "PENDING":
        return "received"
    if status == "CANCELED":
        return "canceled"
    if status == "FAILED":
        return "problem"
    if status == "SUCCEEDED":
        return "done"
    return _STEP_STAGE.get(step_type or "", "preparing")


# What the order window says under a product that cannot be ordered as usual.
_PRODUCT_NOTICE = {"unknown": "재고 확인 중이라 주문을 닫았습니다", "closed": "판매 중지", "sold_out": "품절"}


def _topic(word: str) -> str:
    """The word with its Korean topic particle: 은 after a final consonant, 는 otherwise (사과는, 당근은)."""
    last = word[-1:] if word else ""
    if "가" <= last <= "힣":
        return word + ("은" if (ord(last) - ord("가")) % 28 else "는")
    return word + "은(는)"


def _start_of_today() -> str:
    """Local midnight as a UTC ISO time, comparable with the store's timestamps."""
    midnight = datetime.now().astimezone().replace(hour=0, minute=0, second=0, microsecond=0)
    return midnight.astimezone(timezone.utc).isoformat(timespec="microseconds")


@dataclass
class OrderDesk:
    store: TaskStore
    orchestrator: Any  # .start(task_id) -> (task, changed)
    devices: OrderDevices
    arms: Any  # control_station.arms registry (.arms: list of arm dicts in config order)
    policies: Any  # control_station.act_policies registry
    zones: Mapping[str, ReceivingZone]
    pick_stats: Callable[[], PickStats]
    planning: FleetPlanning = field(default_factory=load_fleet_planning)
    today: Callable[[], str] = _start_of_today
    # The shelf inventory (inventory.InventoryMonitor.snapshot). With it the window sells what is available
    # (shelves minus open orders) and refuses more; without it the desk reads the shelves directly.
    inventory: Callable[[], dict[str, Any]] | None = None
    # OrderFlow.confirm_pickup: True when the order's Pinky was waiting at its receiving zone.
    pickup: Callable[[str], bool] | None = None
    _intake: threading.Lock = field(default_factory=threading.Lock, repr=False)

    # ---- the order window ----------------------------------------------------------------------------------

    def catalog(self) -> dict[str, Any]:
        """What can be ordered (objects with a trained policy), how many the shelves hold, and where to collect."""
        if self.policies is None:
            return {"products": [], "stock_error": "정책 정의 파일이 없어 주문을 받을 수 없습니다", "zones": []}
        zones = [zone.as_dict() for zone in self.zones.values()]
        if self.inventory is not None:
            view = self.inventory()
            products = [{"object_class": row["object_class"], "label": row["label"], "available": row["available"],
                         "status": row["status"], "notice": _PRODUCT_NOTICE.get(row["status"])}
                        for row in view["products"]]
            notice = None
            if view["state"] == "unknown" and any(row["status"] == "unknown" for row in view["products"]):
                notice = f"재고를 확인하지 못해 해당 물건의 주문을 잠시 닫았습니다 ({view['reason']})"
            return {"products": products, "stock_error": notice, "notice": notice, "zones": zones}
        stock, stock_error = self._stock()
        on_shelves: Counter = Counter()
        for counts in stock.values():
            on_shelves.update(counts)
        products: dict[str, dict[str, Any]] = {}
        for policy in self.policies.policies.values():
            products.setdefault(policy["object_class"], {
                "object_class": policy["object_class"],
                "label": policy["label"],
                "available": on_shelves[policy["object_class"]] if stock_error is None else None,
            })
        return {
            "products": list(products.values()),
            "stock_error": stock_error,
            "zones": [zone.as_dict() for zone in self.zones.values()],
        }

    def receive(self, items: object, zone_id: object = None) -> dict[str, Any]:
        """Store a new order in the intake (status PENDING, no Pinky yet).

        The customer does not pick a receiving zone: `start` assigns one (choose_zone). A zone given here is an
        operator's choice and is kept as it is."""
        return self.receive_once(items, zone_id)[0]

    def receive_once(self, items: object, zone_id: object = None, idempotency_key: str | None = None,
                     ) -> tuple[dict[str, Any], bool]:
        """`receive` that a resent request cannot double: (order, made now).

        With an Idempotency-Key the first request makes the order and every later one with the same key and body
        gets that same order back (made now = False), even when stock has since run out because of it. The same key
        with a different body is refused: it would hide a second order behind the first."""
        try:
            order_items = validate_order_items(items, self.policies)
        except ValueError as exc:
            raise OrderDeskError("ORDER_INVALID", str(exc), status=400) from exc
        request_hash = _request_hash(order_items, zone_id)
        # Check and reserve as one step, so two windows cannot both sell the last unit.
        with self._intake:
            if idempotency_key is not None:
                known = self.store.order_request(idempotency_key)
                if known is not None:
                    stored_hash, order_id = known
                    if stored_hash != request_hash:
                        raise OrderDeskError("IDEMPOTENCY_KEY_REUSED",
                                             "같은 주문 키로 다른 주문이 왔어요. 새 주문은 새 키로 보내 주세요", status=422)
                    order = self.store.get_task(order_id)
                    if order is not None:
                        return order, False
            if not self.zones:
                raise OrderDeskError("RECEIVING_ZONE_NOT_CONFIGURED", "수령 구역이 설정되지 않아 주문을 받을 수 없어요")
            zone = None if zone_id is None else self._zone(zone_id)
            summary = " · ".join(f"{self._label(item['object_class'])} {item['quantity']}" for item in order_items)
            self._check_available(order_items)
            shipments = self._shipments({"order_items": order_items})
            order = self.store.create_task(
                zone.label if zone else ZONE_PENDING_LABEL, f"{summary} 적재", order_items=order_items,
                receiving_zone=zone.zone_id if zone else None, defer_pinky=True, zone_steps=True,
                idempotency=None if idempotency_key is None else (idempotency_key, request_hash),
                shipments=[list(shipment.units) for shipment in shipments] if len(shipments) > 1 else None,
            )
            return order, True

    def _shipments(self, order: dict[str, Any]) -> tuple[Any, ...]:
        """The order's Pinky loads (tray capacity and sizes from config/fleet-planning.json), heaviest first."""
        try:
            return split_into_shipments(self._units(order), self.planning.pinky_capacity, self.planning.size_of,
                                        self._label)
        except ShipmentError as exc:
            raise OrderDeskError(exc.error_code, str(exc), status=400) from exc

    def _check_available(self, order_items: list[dict[str, Any]]) -> None:
        """Refuse what cannot be sold now: an object closed (stock unknown, or by the operator), or more than the
        shelves cover after open orders. Every refused object is named in one message."""
        if self.inventory is None:
            return
        rows = {row["object_class"]: row for row in self.inventory()["products"]}
        closed, short = [], []
        for item in order_items:
            row = rows.get(item["object_class"]) or {"status": "unknown", "available": None}
            subject = _topic(self._label(item["object_class"]))
            if row["status"] == "unknown":
                closed.append(f"{subject} 재고를 확인하지 못해 주문을 닫았습니다")
            elif row["status"] == "closed":
                closed.append(f"{subject} 지금 판매하지 않습니다")
            elif item["quantity"] > (row["available"] or 0):
                short.append(f"{subject} 품절입니다" if not row["available"] else f"{subject} {row['available']}개 남았습니다")
        if closed or short:
            raise OrderDeskError("ITEM_CLOSED" if closed else "OUT_OF_STOCK", " · ".join(closed + short))

    # ---- the operator's intake ---------------------------------------------------------------------------

    def pending(self) -> list[dict[str, Any]]:
        """Orders waiting for an operator, oldest first."""
        return self.store.pending_orders()

    def preview(self, order_id: str) -> dict[str, Any]:
        """Who would serve this order if it started now, and why. Changes nothing.

        `pinky` and `arms` describe a one-Pinky order; `fleet` splits a larger order into shipments (tray capacity
        from config/fleet-planning.json) that run one after another and, per shipment, plans the arms and times the
        loading. The first shipment's Pinky is the one `start` would choose; a later one is chosen again when that
        shipment starts, so the preview shows the best guess with the earlier trips counted."""
        order = self._pending_order(order_id)
        units = self._units(order)
        candidates = self.pinky_candidates()
        pinky = choose_pinky(candidates)
        zone = self.zone_plan(order).as_dict()
        try:
            shipments = split_into_shipments(units, self.planning.pinky_capacity, self.planning.size_of, self._label)
        except ShipmentError as exc:
            error = {"code": exc.error_code, "message": str(exc)}
            return {"order_id": order_id, "pinky": pinky.as_dict(), "zone": zone, "arms": {"error": error},
                    "fleet": {"error": error}}
        fleet = assign_fleet(shipments, candidates)
        stock, stock_error = self._stock()
        guesses = self._later_pinkys(candidates, len(shipments))
        arm_ids = [arm["arm_id"] for arm in self.arms.arms]
        stats = self.pick_stats()
        trips = []
        for index, trip in enumerate(fleet.trips):  # later trips plan with what the earlier ones leave on the shelves
            entry = trip.as_dict()
            if len(fleet.trips) > 1:  # shipments run one after another: the Pinky of each is chosen as it starts
                guess = guesses[index]
                entry.update(pinky_id=guess.pinky_id, pinky_name=PINKY_NAMES.get(guess.pinky_id or ""),
                             reason=guess.summary if index == 0 else f"앞 운송이 끝나면 다시 고름 · 지금 기준 {guess.summary}")
            for unit in entry["units"]:  # what the tray drawing needs: the name and how many slots it takes
                unit["label"] = self._label(unit["object_class"])
                unit["size"] = self.planning.size_of(unit["object_class"])
            if stock_error is not None:
                entry["arms"] = {"error": {"code": "SHELF_STOCK_UNAVAILABLE", "message": stock_error}}
            else:
                try:
                    plan = plan_arms(trip.shipment.units, stock, self.arms.arms, self.policies.resolve,
                                     self._label, stats)
                except ArmPlanError as exc:
                    entry["arms"] = {"error": {"code": exc.error_code, "message": str(exc)}}
                else:
                    for pick in plan.picks:
                        stock[pick.shelf_id][pick.object_class] -= 1
                    entry["arms"] = plan.as_dict()
                    entry["schedule"] = schedule_loading(plan.picks, self.planning.timing, arm_ids,
                                                         overlap=self.planning.arms_overlap).as_dict()
            trips.append(entry)
        sequential = len(trips) > 1
        summary = (f"운송 {len(trips)}번으로 나눠 차례로 · " + ", ".join(
            f"{index}. {trip['pinky_name'] or '핑키 대기'} {trip['load']}칸" for index, trip in enumerate(trips, start=1))
                   if sequential else fleet.summary)
        arms = trips[0]["arms"] if not sequential else {
            "error": {"code": "MULTI_SHIPMENT", "message": f"{summary} · 운송별 팔 계획은 fleet에 있습니다"}}
        return {"order_id": order_id, "pinky": pinky.as_dict(), "zone": zone, "arms": arms,
                "fleet": {"summary": summary, "complete": all(trip["pinky_id"] for trip in trips),
                          "capacity": self.planning.pinky_capacity, "trips": trips, "sequential": sequential}}

    def start(self, order_id: str) -> tuple[dict[str, Any], bool]:
        """Assign the receiving zone and the Pinky now (where the order has none) and start the order flow.

        Both are checked before either is written, so a refusal leaves the order as it was."""
        order = self._pending_order(order_id, allow_started=True)
        if order["status"] == "PENDING":
            self._refuse_unsplit(order)
            zone = self.zone_plan(order)
            if zone.zone_id is None:
                raise OrderDeskError("ZONE_BUSY", zone.summary, zone=zone.as_dict())
            if order["pinky_id"] is None:
                assignment = choose_pinky(self.pinky_candidates())
                if assignment.pinky_id is None:
                    raise OrderDeskError("PINKY_UNAVAILABLE", assignment.summary, assignment=assignment.as_dict())
                try:
                    self.store.assign_pinky(order_id, assignment.pinky_id, assignment.as_dict())
                except PinkyUnavailable as exc:  # taken by another order between the choice and the write
                    raise OrderDeskError("PINKY_UNAVAILABLE", str(exc)) from exc
            if order["receiving_zone"] is None:
                try:
                    self.store.assign_zone(order_id, zone.zone_id, self.zones[zone.zone_id].label, zone.as_dict())
                except ZoneUnavailable as exc:  # another order started there between the choice and the write
                    raise OrderDeskError("ZONE_BUSY", f"수령 구역이 모두 사용 중이에요: {exc}") from exc
        return self.orchestrator.start(order_id)

    @staticmethod
    def _later_pinkys(candidates: list[PinkyCandidate], count: int) -> list[Any]:
        """The Pinky choice for each shipment if they ran now one after another: the Pinky of each earlier
        shipment counts one more trip today and has just finished."""
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        pool = list(candidates)
        choices = []
        for _ in range(count):
            choice = choose_pinky(pool)
            choices.append(choice)
            pool = [replace(c, trips_today=c.trips_today + 1, last_finished_at=now) if c.pinky_id == choice.pinky_id
                    else c for c in pool]
        return choices

    def _refuse_unsplit(self, order: dict[str, Any]) -> None:
        """An order received before shipments existed (or by the older /api/tasks path) has one load's steps; if it
        no longer fits one tray it cannot run as it is."""
        needed = len(self._shipments(order))
        have = max((shipment_of(step) for step in order["steps"]), default=1)
        if needed > have:
            raise OrderDeskError(
                "ORDER_NEEDS_SHIPMENTS",
                f"이 주문은 핑키 쟁반 {self.planning.pinky_capacity}칸에 다 실리지 않아 운송 {needed}번이 필요한데, "
                "나누지 않은 채로 접수돼 있습니다. 거절하고 다시 주문해 주세요",
            )

    def assign_next_shipment(self, order_id: str, shipment: int) -> dict[str, Any]:
        """The Pinky for a later shipment of a running order, chosen now and written on the order.

        The order's own Pinky of the previous shipment is parked again by now and may go again; the ranking (battery,
        trips today, last finished) usually sends another one. No free Pinky: {"pinky_id": None} and the flow waits."""
        candidates = [replace(candidate, holding_order=None) if candidate.holding_order == order_id else candidate
                      for candidate in self.pinky_candidates()]
        assignment = choose_pinky(candidates)
        if assignment.pinky_id is None:
            return {"pinky_id": None, "summary": assignment.summary, "assignment": assignment.as_dict()}
        try:
            self.store.assign_shipment_pinky(order_id, shipment, assignment.pinky_id,
                                             {**assignment.as_dict(), "shipment": shipment})
        except (PinkyUnavailable, InvalidStateTransition) as exc:
            return {"pinky_id": None, "summary": str(exc), "assignment": assignment.as_dict()}
        return {"pinky_id": assignment.pinky_id, "summary": assignment.summary, "assignment": assignment.as_dict()}

    def confirm_pickup(self, order_id: str) -> dict[str, Any]:
        """The customer took the order at the receiving zone; the flow then sends the Pinky home."""
        order = self.store.get_task(order_id)
        if order is None or order.get("order_items") is None:
            raise OrderDeskError("ORDER_NOT_FOUND", f"주문 {order_id}이 없습니다", status=404)
        if self.pickup is None or not self.pickup(order_id):
            raise OrderDeskError("PICKUP_NOT_READY", "핑키가 아직 수령 구역에서 기다리고 있지 않습니다")
        return {"order_id": order_id, "accepted": True, "message": "수령 확인을 받았습니다. 핑키가 주차장으로 돌아갑니다"}

    def track(self, order_id: str) -> dict[str, Any]:
        """The customer's view of one order: where it is, in plain words, and whether pickup can be confirmed."""
        order = self.store.get_task(order_id)
        if order is None or order.get("order_items") is None:
            raise OrderDeskError("ORDER_NOT_FOUND", f"주문 {order_id}이 없습니다", status=404)
        steps = order["steps"]
        active = next((step for step in steps if step["id"] == order.get("active_step_id")), None)
        stage = _customer_stage(order["status"], active["step_type"] if active else None)
        count = max((int((step.get("detail") or {}).get("shipments") or 1) for step in steps), default=1)
        current = shipment_of(active) if active else (count if order["status"] == "SUCCEEDED" else 1)
        between = False
        if count > 1 and active is not None and order["status"] in {"RUNNING", "VALIDATING"}:
            if active["step_type"] == "WAIT_PINKY_PARKED" and current < count:
                current, between = current + 1, True  # this load was picked up; the next one is on its way
            elif active["step_type"] == "START_SHIPMENT":
                between = True
            if between:
                stage = "pinky_coming"
        zone_id = order.get("receiving_zone")
        known = self.zones.get(zone_id or "")
        zone_label = (known.label if known else order.get("destination")) if zone_id else None
        zone = zone_label or "수령 구역"
        mine = [step for step in steps if shipment_of(step) == current]
        moved = sum(1 for step in mine if step["step_type"] == "VERIFY_TRANSFER" and step["status"] == "SUCCEEDED")
        total = sum(1 for step in mine if step["step_type"] == "VERIFY_TRANSFER")
        title, detail = _CUSTOMER_TEXT[stage]
        title, detail = title.format(zone=zone), detail.format(zone=zone, moved=moved, total=total)
        if between:
            title, detail = "남은 물건을 가지러 가요", f"{current - 1}/{count}번째 운송을 받았어요 · 다음 운송을 준비해요"
        load = Counter(step["detail"]["object_class"] for step in mine if step["step_type"] == "PICK_ITEM")
        return {
            "order_id": order_id, "status": order["status"], "stage": stage,
            "stages": [{"key": key, "label": label, "state": "done" if index < _STAGE_INDEX.get(stage, -1)
                        else "current" if key == stage else "todo"}
                       for index, (key, label) in enumerate(_CUSTOMER_STAGES)],
            "title": title, "detail": detail,
            "items": [{"label": self._label(item["object_class"]), "quantity": item["quantity"]} for item in order["order_items"]],
            "zone": zone_label, "zone_note": None if zone_label else ZONE_PENDING_NOTE,
            "pinky": PINKY_NAMES.get(order.get("pinky_id") or ""),
            "loaded": moved, "total": total, "can_confirm_pickup": stage == "arrived",
            # A larger order comes in several loads, one after another; each one is picked up on its own.
            "shipment": current, "shipments": count,
            "shipment_items": ([{"label": self._label(name), "quantity": quantity} for name, quantity in load.items()]
                               if count > 1 else None),
            "created_at": order.get("created_at"), "finished_at": order.get("finished_at"),
        }

    def reject(self, order_id: str) -> dict[str, Any]:
        """Turn down an order that has not started; no device was involved yet."""
        self._pending_order(order_id)
        return self.store.cancel_pending_task(order_id)

    # ---- what the zone and Pinky choices are made from ---------------------------------------------------

    def zone_candidates(self) -> list[ZoneCandidate]:
        holding = self.store.busy_zones()
        usage = self.store.zone_usage(self.today())
        return [ZoneCandidate(zone.zone_id, zone.label, len(zone.route_labels), holding.get(zone.zone_id),
                              usage.get(zone.zone_id, 0)) for zone in self.zones.values()]

    def zone_plan(self, order: dict[str, Any]) -> ZoneAssignment:
        """The zone this order would go to now: its own (an operator's choice, only checked) or choose_zone's.
        A waiting order holds no zone itself, so it never stands in its own way."""
        return choose_zone(self.zone_candidates(), requested=order.get("receiving_zone"))

    def pinky_candidates(self) -> list[PinkyCandidate]:
        holding = self.store.busy_pinkies()
        usage = self.store.pinky_usage(self.today())
        candidates = []
        for pinky_id in PINKY_IDS:
            used = usage.get(pinky_id, {})
            common = {
                "pinky_id": pinky_id, "name": PINKY_NAMES[pinky_id], "holding_order": holding.get(pinky_id),
                "trips_today": int(used.get("trips", 0)), "last_finished_at": used.get("last_finished_at"),
            }
            try:
                status = self.devices.fleet_status(pinky_id)
            except (PinkyFleetError, OSError):
                candidates.append(PinkyCandidate(**common, reachable=False))
                continue
            candidates.append(PinkyCandidate(
                **common, phase=status.get("phase"), ready_for_motion=status.get("ready_for_motion"),
                readiness_label=status.get("readiness_label"), battery_percent=status.get("battery_percent"),
            ))
        return candidates

    # ---- helpers -------------------------------------------------------------------------------------------

    def _stock(self) -> tuple[dict[str, Counter], str | None]:
        try:
            shelves = self.devices.shelf_stock()["shelves"]
        except Exception as exc:  # camera not ready, no frame, node down: the window says why
            return {}, f"선반 재고를 읽지 못했습니다: {exc}"
        return {shelf: Counter(counts) for shelf, counts in shelves.items()}, None

    @staticmethod
    def _units(order: dict[str, Any]) -> list[dict[str, Any]]:
        return [{"object_class": item["object_class"], "unit": index, "of": item["quantity"]}
                for item in order["order_items"] for index in range(1, int(item["quantity"]) + 1)]

    def _zone(self, zone_id: object) -> ReceivingZone:
        """A zone named with the order (an operator's choice)."""
        if not isinstance(zone_id, str) or zone_id not in self.zones:
            known = ", ".join(self.zones) or "없음"
            raise OrderDeskError("RECEIVING_ZONE_UNKNOWN", f"알 수 없는 수령 구역이에요 (가능: {known})", status=400)
        return self.zones[zone_id]

    def _label(self, object_class: str) -> str:
        found = self.policies.matches(object_class)
        return found[0]["label"] if found else object_class

    def _pending_order(self, order_id: str, *, allow_started: bool = False) -> dict[str, Any]:
        order = self.store.get_task(order_id)
        if order is None or order.get("order_items") is None:
            raise OrderDeskError("ORDER_NOT_FOUND", f"주문 {order_id}이 없습니다", status=404)
        if order["status"] != "PENDING" and not allow_started:
            raise OrderDeskError("ORDER_NOT_PENDING", f"주문 {order_id}은 이미 {order['status']} 상태입니다")
        return order
