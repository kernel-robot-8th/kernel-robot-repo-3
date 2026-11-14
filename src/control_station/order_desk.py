"""The order desk: orders from the order window wait here until an operator starts them.

Receiving an order stores it without a Pinky. The preview shows which Pinky and which arms would serve it right
now, and why; starting runs the same Pinky choice again with the fleet state of that moment, records it on the
order and hands the order to the task orchestrator. The arm plan is made again inside the order flow, after the
shelf check, so the preview is advice and never a reservation. Nothing here moves a device.
"""

from __future__ import annotations

import threading
from collections import Counter
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from control_station.api.common import validate_order_items
from control_station.assignment import (
    ArmPlanError, PickStats, PinkyCandidate, ShipmentError, assign_fleet, choose_pinky, plan_arms, schedule_loading,
    split_into_shipments,
)
from control_station.fleet_planning import FleetPlanning, load_fleet_planning
from control_station.order_flow import OrderDevices
from control_station.pinky_fleet import PinkyFleetError
from control_station.receiving_zones import ReceivingZone
from control_station.store import PINKY_IDS, PINKY_NAMES, PinkyUnavailable, TaskStore


class OrderDeskError(RuntimeError):
    """A request the desk refuses; `status` is the HTTP status the API answers with."""

    def __init__(self, error_code: str, message: str, status: int = 409, **details: object) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.status = status
        self.details = details


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

    def receive(self, items: object, zone_id: object) -> dict[str, Any]:
        """Store a new order in the intake (status PENDING, no Pinky yet)."""
        try:
            order_items = validate_order_items(items, self.policies)
        except ValueError as exc:
            raise OrderDeskError("ORDER_INVALID", str(exc), status=400) from exc
        zone = self._zone(zone_id)
        summary = " · ".join(f"{self._label(item['object_class'])} {item['quantity']}" for item in order_items)
        # Check and reserve as one step, so two windows cannot both sell the last unit.
        with self._intake:
            self._check_available(order_items)
            return self.store.create_task(
                zone.label, f"{summary} 적재", order_items=order_items, receiving_zone=zone.zone_id, defer_pinky=True,
            )

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

        `pinky` and `arms` describe a one-Pinky order; `fleet` splits a larger order over Pinkys (tray capacity
        from config/fleet-planning.json) and, per trip, plans the arms and times the loading with both arms
        overlapping at their shelves and taking turns over the Pinky."""
        order = self._pending_order(order_id)
        units = self._units(order)
        candidates = self.pinky_candidates()
        pinky = choose_pinky(candidates)
        try:
            shipments = split_into_shipments(units, self.planning.pinky_capacity, self.planning.size_of, self._label)
        except ShipmentError as exc:
            error = {"code": exc.error_code, "message": str(exc)}
            return {"order_id": order_id, "pinky": pinky.as_dict(), "arms": {"error": error}, "fleet": {"error": error}}
        fleet = assign_fleet(shipments, candidates)
        stock, stock_error = self._stock()
        arm_ids = [arm["arm_id"] for arm in self.arms.arms]
        stats = self.pick_stats()
        trips = []
        for trip in fleet.trips:  # later trips plan with what the earlier ones leave on the shelves
            entry = trip.as_dict()
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
        arms = trips[0]["arms"] if len(trips) == 1 else {
            "error": {"code": "MULTI_PINKY", "message": f"{fleet.summary} · 배송별 팔 계획은 fleet에 있습니다"}}
        return {"order_id": order_id, "pinky": pinky.as_dict(), "arms": arms,
                "fleet": {"summary": fleet.summary, "complete": fleet.complete, "trips": trips}}

    def start(self, order_id: str) -> tuple[dict[str, Any], bool]:
        """Assign the Pinky now (if the order has none) and start the order flow."""
        order = self._pending_order(order_id, allow_started=True)
        if order["status"] == "PENDING" and order["pinky_id"] is None:
            self._refuse_multi_pinky(order)
            assignment = choose_pinky(self.pinky_candidates())
            if assignment.pinky_id is None:
                raise OrderDeskError("PINKY_UNAVAILABLE", assignment.summary, assignment=assignment.as_dict())
            try:
                self.store.assign_pinky(order_id, assignment.pinky_id, assignment.as_dict())
            except PinkyUnavailable as exc:  # taken by another order between the choice and the write
                raise OrderDeskError("PINKY_UNAVAILABLE", str(exc)) from exc
        return self.orchestrator.start(order_id)

    def _refuse_multi_pinky(self, order: dict[str, Any]) -> None:
        """An order larger than one tray is planned (preview) but not run yet: one order runs with one Pinky."""
        try:
            shipments = split_into_shipments(self._units(order), self.planning.pinky_capacity,
                                             self.planning.size_of, self._label)
        except ShipmentError as exc:
            raise OrderDeskError(exc.error_code, str(exc)) from exc
        if len(shipments) > 1:
            raise OrderDeskError(
                "MULTI_PINKY_NOT_SUPPORTED",
                f"이 주문은 핑키 {len(shipments)}대가 필요합니다(쟁반 용량 {self.planning.pinky_capacity}). "
                "여러 핑키로 나눠 실행하는 단계는 아직 없어서 주문을 나눠 넣어야 합니다",
            )

    def reject(self, order_id: str) -> dict[str, Any]:
        """Turn down an order that has not started; no device was involved yet."""
        self._pending_order(order_id)
        return self.store.cancel_pending_task(order_id)

    # ---- what the Pinky choice is made from ----------------------------------------------------------------

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
        if zone_id is None and len(self.zones) == 1:
            return next(iter(self.zones.values()))
        if not isinstance(zone_id, str) or zone_id not in self.zones:
            known = ", ".join(self.zones) or "없음"
            raise OrderDeskError("RECEIVING_ZONE_UNKNOWN", f"수령 구역을 골라 주세요 (가능: {known})", status=400)
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
