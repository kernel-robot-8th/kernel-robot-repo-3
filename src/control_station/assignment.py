"""Who serves an order: one Pinky for the trip, and an OMX arm for every unit it carries.

Both choices are ordered rule lists rather than weighted scores, so every decision reads as one sentence on the
operator screen ("P1 · 배터리 80%대 · 오늘 1회"). Nothing here talks to a device or the database: callers pass in
what they observed, which keeps the rules the same in mock and real mode and easy to test.
"""

from __future__ import annotations

import math
from collections import Counter, deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
from functools import partial
from typing import Any

MIN_BATTERY_PERCENT = 30.0


# ---------------------------------------------------------------------------------------------------------------
# Pinky
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PinkyCandidate:
    """What is known about one Pinky at the moment an order is assigned."""

    pinky_id: str
    name: str
    holding_order: str | None = None  # an unfinished order that keeps this Pinky
    reachable: bool = True  # the fleet answered for it
    phase: str | None = None  # mission phase reported by the fleet ("parked", "driving", ...)
    ready_for_motion: bool | None = None  # fleet readiness; None when the source does not report it
    readiness_label: str | None = None
    battery_percent: float | None = None
    trips_today: int = 0
    last_finished_at: str | None = None  # ISO time its last order ended; None if it has not run


@dataclass(frozen=True)
class PinkyVerdict:
    pinky_id: str
    name: str
    eligible: bool
    reason: str
    rank: int | None = None  # 1 = chosen; None when excluded
    # Every exclusion rule as {rule, ok, detail}, so the screen can show the whole matrix, not only the first miss.
    checks: tuple[dict[str, Any], ...] = ()
    # The values the ranking compared, in order: battery band (%), trips today, last finished.
    keys: dict[str, Any] | None = None


@dataclass(frozen=True)
class PinkyAssignment:
    pinky_id: str | None
    summary: str
    verdicts: tuple[PinkyVerdict, ...]

    def as_dict(self) -> dict[str, Any]:
        return {"pinky_id": self.pinky_id, "summary": self.summary, "verdicts": [asdict(v) for v in self.verdicts]}


def _exclusion(candidate: PinkyCandidate, min_battery: float) -> str | None:
    """Why this Pinky cannot take a new order now, or None when it can."""
    if candidate.holding_order:
        return f"주문 {candidate.holding_order} 진행 중"
    if not candidate.reachable:
        return "핑키 관제 연결 안 됨"
    if candidate.phase != "parked":
        return "주차장 대기가 아님"
    if candidate.ready_for_motion is False:
        return candidate.readiness_label or "운행 준비 안 됨"
    if candidate.battery_percent is not None and candidate.battery_percent < min_battery:
        return f"배터리 {candidate.battery_percent:.0f}% · 충전 필요"
    return None


def _checks(candidate: PinkyCandidate, min_battery: float) -> tuple[dict[str, Any], ...]:
    """Each exclusion rule, passed or not, in the order `_exclusion` applies them."""
    battery = candidate.battery_percent
    return (
        {"rule": "진행 중 주문 없음", "ok": not candidate.holding_order,
         "detail": f"주문 {candidate.holding_order}" if candidate.holding_order else None},
        {"rule": "핑키 관제 연결", "ok": candidate.reachable, "detail": None if candidate.reachable else "응답 없음"},
        {"rule": "주차장 대기", "ok": candidate.phase == "parked", "detail": candidate.phase},
        {"rule": "운행 준비", "ok": candidate.ready_for_motion is not False, "detail": candidate.readiness_label},
        {"rule": f"배터리 {min_battery:.0f}% 이상", "ok": battery is None or battery >= min_battery,
         "detail": None if battery is None else f"{battery:.0f}%"},
    )


def _keys(candidate: PinkyCandidate) -> dict[str, Any]:
    band = _battery_band(candidate)
    return {"battery_band": None if band < 0 else band * 10, "trips_today": candidate.trips_today,
            "last_finished_at": candidate.last_finished_at}


def _battery_band(candidate: PinkyCandidate) -> int:
    # 10 % bands so that a 1-2 % difference does not flip the choice; an unknown battery ranks below any known one.
    return -1 if candidate.battery_percent is None else int(candidate.battery_percent // 10)


def _preference(candidate: PinkyCandidate) -> tuple[int, int, str, str]:
    """Sort key, best first: higher battery band, fewer trips today, rested longer, lower id."""
    return (-_battery_band(candidate), candidate.trips_today, candidate.last_finished_at or "", candidate.pinky_id)


def _describe(candidate: PinkyCandidate) -> str:
    band = _battery_band(candidate)
    battery = "배터리 정보 없음" if band < 0 else f"배터리 {band * 10}%대"
    return f"{battery} · 오늘 {candidate.trips_today}회"


def choose_pinky(
    candidates: Sequence[PinkyCandidate], *, min_battery: float = MIN_BATTERY_PERCENT
) -> PinkyAssignment:
    """Pick the Pinky for one order: drop the ones that cannot go, then rank the rest by `_preference`."""
    excluded = {c.pinky_id: reason for c in candidates if (reason := _exclusion(c, min_battery)) is not None}
    ranked = sorted((c for c in candidates if c.pinky_id not in excluded), key=_preference)
    rank_of = {c.pinky_id: index for index, c in enumerate(ranked, start=1)}
    verdicts = tuple(
        PinkyVerdict(c.pinky_id, c.name, False, excluded[c.pinky_id], None, _checks(c, min_battery), _keys(c))
        if c.pinky_id in excluded
        else PinkyVerdict(c.pinky_id, c.name, True, _describe(c), rank_of[c.pinky_id], _checks(c, min_battery), _keys(c))
        for c in candidates
    )
    if not ranked:
        return PinkyAssignment(None, "배정 가능한 핑키 없음", verdicts)
    chosen = ranked[0]
    return PinkyAssignment(chosen.pinky_id, f"{chosen.name} · {_describe(chosen)}", verdicts)


# ---------------------------------------------------------------------------------------------------------------
# OMX arms
# ---------------------------------------------------------------------------------------------------------------

# (successes, attempts) of one policy on one arm, from finished transfer checks.
PickStats = Mapping[tuple[str, str], tuple[int, int]]
PolicyResolver = Callable[[str, str], Mapping[str, Any] | None]


class ArmPlanError(ValueError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


@dataclass(frozen=True)
class ArmPick:
    """One unit of the order, the arm that picks it, and why that arm."""

    object_class: str
    label: str
    unit: int
    of: int
    arm_id: str
    arm_label: str
    shelf_id: str
    policy_id: str
    reason: str
    # Which rule decided: only_option, success_rate, balance, stock or config (see plan_arms).
    rule: str = ""
    # Every arm as it stood when this unit was placed: {arm_id, arm_label, possible, why, success_rate,
    # assigned, remaining, chosen}.
    options: tuple[dict[str, Any], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ArmPlan:
    picks: tuple[ArmPick, ...]  # in execution order
    summary: str

    def as_dict(self) -> dict[str, Any]:
        return {"summary": self.summary, "picks": [pick.as_dict() for pick in self.picks]}


def success_rate(stats: PickStats, policy_id: str, arm_id: str) -> float:
    """Laplace-smoothed pick success, (s + 1) / (n + 2): 0.5 with no history, and a single result moves it little."""
    successes, attempts = stats.get((policy_id, arm_id), (0, 0))
    return (successes + 1) / (attempts + 2)


def plan_arms(
    units: Sequence[Mapping[str, Any]],
    stock: Mapping[str, Mapping[str, int]],
    arms: Sequence[Mapping[str, Any]],
    resolve_policy: PolicyResolver,
    label_for: Callable[[str], str],
    stats: PickStats | None = None,
) -> ArmPlan:
    """Give every unit an arm, then order the picks so the arms take turns at the single loading spot.

    An arm picks only from its own shelf and only objects it has a trained policy for, and no shelf gives more
    units than its camera counted. The unit with the fewest workable arms is placed first (most-constrained
    first), so an object found on one shelf only is not starved by one that both arms could pick. Between two
    workable arms the better success rate wins, then the arm with fewer units so far (two busy arms finish sooner
    when their picks overlap), then the shelf with more of that object left, then config order.
    """
    stats = stats or {}
    remaining = {shelf: Counter(counts) for shelf, counts in stock.items()}
    order = {arm["arm_id"]: index for index, arm in enumerate(arms)}
    assigned: Counter = Counter()  # units given to each arm so far

    def workable(object_class: str) -> list[tuple[Mapping[str, Any], Mapping[str, Any]]]:
        options = []
        for arm in arms:
            policy = resolve_policy(object_class, arm["arm_id"])
            if policy is not None and remaining.get(arm["shelf_id"], Counter())[object_class] > 0:
                options.append((arm, policy))
        return options

    pending = list(enumerate(units))
    placed: list[tuple[int, ArmPick]] = []
    while pending:
        position, unit = min(pending, key=lambda entry: (len(workable(entry[1]["object_class"])), entry[0]))
        pending.remove((position, unit))
        object_class = unit["object_class"]
        options = workable(object_class)
        if not options:
            raise _no_arm_error(object_class, label_for(object_class), arms, remaining)

        ranked = sorted(options, key=partial(_arm_preference, object_class=object_class, stats=stats,
                                             assigned=assigned, remaining=remaining, order=order))
        arm, policy = ranked[0]
        rule, reason = _arm_reason(ranked, object_class, stats, remaining, assigned)
        options = _arm_options(arms, object_class, resolve_policy, stats, remaining, assigned, arm["arm_id"])
        remaining[arm["shelf_id"]][object_class] -= 1
        assigned[arm["arm_id"]] += 1
        placed.append((position, ArmPick(
            object_class=object_class, label=label_for(object_class), unit=int(unit["unit"]), of=int(unit["of"]),
            arm_id=arm["arm_id"], arm_label=arm["label"], shelf_id=arm["shelf_id"],
            policy_id=policy["policy_id"], reason=reason, rule=rule, options=options,
        )))

    # One arm finishes before the other starts (they share the loading spot): the busier arm goes first, and each
    # arm keeps the order's own unit order.
    load = Counter(pick.arm_id for _, pick in placed)
    placed.sort(key=lambda entry: (-load[entry[1].arm_id], order[entry[1].arm_id], entry[0]))
    picks = tuple(pick for _, pick in placed)
    turns = [f"{pick.arm_label} {load[pick.arm_id]}개" for index, pick in enumerate(picks)
             if index == 0 or picks[index - 1].arm_id != pick.arm_id]
    return ArmPlan(picks, " → ".join(turns))


def _arm_preference(
    option: tuple[Mapping[str, Any], Mapping[str, Any]],
    *,
    object_class: str,
    stats: PickStats,
    assigned: Counter,
    remaining: Mapping[str, Counter],
    order: Mapping[str, int],
) -> tuple[float, int, int, int]:
    """Sort key for the arms that can take a unit, best first (see plan_arms)."""
    arm, policy = option
    return (
        -success_rate(stats, policy["policy_id"], arm["arm_id"]),
        assigned[arm["arm_id"]],
        -remaining[arm["shelf_id"]][object_class],
        order[arm["arm_id"]],
    )


def _arm_reason(
    ranked: Sequence[tuple[Mapping[str, Any], Mapping[str, Any]]],
    object_class: str,
    stats: PickStats,
    remaining: Mapping[str, Counter],
    assigned: Counter,
) -> tuple[str, str]:
    """(rule, words): the first rule that separated the chosen arm from the runner-up."""
    (arm, policy), *others = ranked
    if not others:
        return "only_option", f"{arm['label']} 쪽에서만 집을 수 있음"
    other_arm, other_policy = others[0]
    rate = success_rate(stats, policy["policy_id"], arm["arm_id"])
    other_rate = success_rate(stats, other_policy["policy_id"], other_arm["arm_id"])
    if rate != other_rate:
        return "success_rate", f"성공률 {rate:.2f} vs {other_rate:.2f}"
    if assigned[arm["arm_id"]] != assigned[other_arm["arm_id"]]:
        return "balance", f"맡은 개수 {assigned[arm['arm_id']]} vs {assigned[other_arm['arm_id']]}"
    left, right = remaining[arm["shelf_id"]][object_class], remaining[other_arm["shelf_id"]][object_class]
    if left != right:
        return "stock", f"남은 재고 {left} vs {right}"
    return "config", "조건이 같아 설정 순서대로"


def _arm_options(
    arms: Sequence[Mapping[str, Any]],
    object_class: str,
    resolve_policy: PolicyResolver,
    stats: PickStats,
    remaining: Mapping[str, Counter],
    assigned: Counter,
    chosen_arm_id: str,
) -> tuple[dict[str, Any], ...]:
    """Every arm as it stood for this unit: why it could not take it, or the values the rules compared."""
    options = []
    for arm in arms:
        policy = resolve_policy(object_class, arm["arm_id"])
        left = remaining.get(arm["shelf_id"], Counter())[object_class]
        why = "정책 없음" if policy is None else ("선반에 없음" if left <= 0 else None)
        options.append({
            "arm_id": arm["arm_id"], "arm_label": arm["label"], "possible": why is None, "why": why,
            "success_rate": None if policy is None else round(success_rate(stats, policy["policy_id"], arm["arm_id"]), 3),
            "assigned": assigned[arm["arm_id"]], "remaining": left, "chosen": arm["arm_id"] == chosen_arm_id,
        })
    return tuple(options)


def _no_arm_error(
    object_class: str, label: str, arms: Sequence[Mapping[str, Any]], remaining: Mapping[str, Counter]
) -> ArmPlanError:
    seen_on = [arm["label"] for arm in arms if remaining.get(arm["shelf_id"], Counter())[object_class] > 0]
    if not seen_on:
        return ArmPlanError("SHELF_STOCK_SHORT", f"{label}: 선반에 남은 수량이 주문보다 적습니다")
    return ArmPlanError("POLICY_NOT_FOR_ARM", f"{label}: 물체가 있는 쪽 팔({', '.join(seen_on)})에 학습된 정책이 없습니다")


# ---------------------------------------------------------------------------------------------------------------
# Several Pinkys for one order
# ---------------------------------------------------------------------------------------------------------------


class ShipmentError(ValueError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


@dataclass(frozen=True)
class Shipment:
    """The units one Pinky carries for an order, and how much of its tray they take."""

    units: tuple[Mapping[str, Any], ...]
    load: int


def split_into_shipments(
    units: Sequence[Mapping[str, Any]],
    capacity: int,
    size_of: Callable[[str], int],
    label_for: Callable[[str], str],
) -> tuple[Shipment, ...]:
    """Split an order over as few Pinkys as the tray capacity allows, with the load spread evenly between them.

    Start from the fewest Pinkys that could hold the total, place the largest units first, each on the least loaded
    tray it still fits on; add a Pinky only when a unit fits nowhere. Heaviest shipment first."""
    sizes = [size_of(unit["object_class"]) for unit in units]
    for unit, size in zip(units, sizes, strict=True):
        if size > capacity:
            raise ShipmentError("ITEM_TOO_BIG_FOR_PINKY",
                                f"{label_for(unit['object_class'])}: 핑키 한 대에 실을 수 없습니다 (크기 {size} > {capacity})")
    count = max(1, math.ceil(sum(sizes) / capacity))
    while True:
        trays: list[list[int]] = [[] for _ in range(count)]
        loads = [0] * count
        for index in sorted(range(len(units)), key=lambda i: (-sizes[i], i)):
            fitting = [tray for tray in range(count) if loads[tray] + sizes[index] <= capacity]
            if not fitting:
                break
            tray = min(fitting, key=lambda t: (loads[t], t))
            trays[tray].append(index)
            loads[tray] += sizes[index]
        else:
            shipments = [Shipment(tuple(units[i] for i in sorted(tray)), load) for tray, load in zip(trays, loads, strict=True)]
            return tuple(sorted(shipments, key=lambda shipment: -shipment.load))
        count += 1


@dataclass(frozen=True)
class Trip:
    """One shipment and the Pinky that carries it; no Pinky means it waits for one to come free."""

    shipment: Shipment
    pinky_id: str | None
    pinky_name: str | None
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {"units": [dict(unit) for unit in self.shipment.units], "load": self.shipment.load,
                "pinky_id": self.pinky_id, "pinky_name": self.pinky_name, "reason": self.reason}


@dataclass(frozen=True)
class FleetPlan:
    trips: tuple[Trip, ...]
    pinky: PinkyAssignment  # every Pinky's verdict and rank
    summary: str

    @property
    def complete(self) -> bool:
        return all(trip.pinky_id is not None for trip in self.trips)

    def as_dict(self) -> dict[str, Any]:
        return {"summary": self.summary, "complete": self.complete,
                "trips": [trip.as_dict() for trip in self.trips], "pinky": self.pinky.as_dict()}


def assign_fleet(
    shipments: Sequence[Shipment], candidates: Sequence[PinkyCandidate], *, min_battery: float = MIN_BATTERY_PERCENT
) -> FleetPlan:
    """Give the heaviest shipment to the best ranked Pinky, the next to the next; the rest wait for a free one."""
    ranking = choose_pinky(candidates, min_battery=min_battery)
    ranked = sorted((v for v in ranking.verdicts if v.eligible), key=lambda v: v.rank or 0)
    trips = tuple(
        Trip(shipment, ranked[index].pinky_id, ranked[index].name, ranked[index].reason)
        if index < len(ranked) else Trip(shipment, None, None, "빈 핑키를 기다림")
        for index, shipment in enumerate(shipments)
    )
    carried = [f"{trip.pinky_name} {trip.shipment.load}" for trip in trips if trip.pinky_id is not None]
    if len(carried) == len(trips):
        summary = f"핑키 {len(trips)}대 · " + ", ".join(carried)
    else:
        summary = f"핑키 {len(trips)}대 필요 · 지금 {len(carried)}대 가능"
    return FleetPlan(trips, ranking, summary)


# ---------------------------------------------------------------------------------------------------------------
# Two arms, one loading spot
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PickTiming:
    """Seconds one pick takes, split where the arm enters the shared space above the Pinky."""

    shelf_s: float  # at its own shelf: grasp and lift to the waiting pose, outside the loading space
    place_s: float  # waiting pose -> over the Pinky -> release -> back out: inside the loading space


@dataclass(frozen=True)
class TimedPick:
    pick: ArmPick
    shelf_start_s: float
    place_start_s: float
    place_end_s: float
    waited_s: float  # held at the waiting pose because the other arm was over the Pinky

    def as_dict(self) -> dict[str, Any]:
        return {**self.pick.as_dict(), "shelf_start_s": self.shelf_start_s, "place_start_s": self.place_start_s,
                "place_end_s": self.place_end_s, "waited_s": self.waited_s}


@dataclass(frozen=True)
class LoadingSchedule:
    picks: tuple[TimedPick, ...]  # in the order they reach the Pinky
    total_s: float
    one_arm_at_a_time_s: float
    overlap: bool = True

    def as_dict(self) -> dict[str, Any]:
        return {"total_s": self.total_s, "one_arm_at_a_time_s": self.one_arm_at_a_time_s, "overlap": self.overlap,
                "picks": [pick.as_dict() for pick in self.picks]}


def schedule_loading(
    picks: Sequence[ArmPick], timing: Callable[[ArmPick], PickTiming], arm_ids: Sequence[str], *, overlap: bool = True
) -> LoadingSchedule:
    """When each pick happens.

    Without `overlap`, one whole pick after another in plan order: what the arms do while each ACT policy runs
    grasp -> place -> rest in one go. With it, the loading space above the Pinky is the only shared resource and
    becomes a lock: an arm grasps at its own shelf while the other places, then waits at its waiting pose until the
    space is free. The arm that is ready first places first (ties: the arm with more picks left, then config
    order), which makes the arms alternate whenever both have work. Each arm keeps the plan's order for its picks."""
    sequential = sum(timing(pick).shelf_s + timing(pick).place_s for pick in picks)
    if not overlap:
        timeline, clock = [], 0.0
        for pick in picks:
            step = timing(pick)
            timeline.append(TimedPick(pick, clock, clock + step.shelf_s, clock + step.shelf_s + step.place_s, 0.0))
            clock += step.shelf_s + step.place_s
        return LoadingSchedule(tuple(timeline), clock, sequential, overlap=False)
    queues = {arm: deque(pick for pick in picks if pick.arm_id == arm) for arm in arm_ids}
    free_at = dict.fromkeys(arm_ids, 0.0)
    space_free_at = 0.0
    timeline: list[TimedPick] = []
    while any(queues.values()):
        ready_at, _, _, arm = min(
            (free_at[arm] + timing(queue[0]).shelf_s, -len(queue), position, arm)
            for position, (arm, queue) in enumerate(queues.items()) if queue
        )
        pick = queues[arm].popleft()
        place_start = max(ready_at, space_free_at)
        place_end = place_start + timing(pick).place_s
        timeline.append(TimedPick(pick, free_at[arm], place_start, place_end, place_start - ready_at))
        free_at[arm] = space_free_at = place_end
    return LoadingSchedule(tuple(timeline), max((p.place_end_s for p in timeline), default=0.0), sequential, overlap=True)
