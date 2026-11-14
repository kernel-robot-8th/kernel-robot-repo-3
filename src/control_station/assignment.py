"""Who serves an order: one Pinky for the trip, and an OMX arm for every unit it carries.

Both choices are ordered rule lists rather than weighted scores, so every decision reads as one sentence on the
operator screen ("P1 · 배터리 80%대 · 오늘 1회"). Nothing here talks to a device or the database: callers pass in
what they observed, which keeps the rules the same in mock and real mode and easy to test.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
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
        PinkyVerdict(c.pinky_id, c.name, False, excluded[c.pinky_id])
        if c.pinky_id in excluded
        else PinkyVerdict(c.pinky_id, c.name, True, _describe(c), rank_of[c.pinky_id])
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
    workable arms the better success rate wins, then the shelf with more of that object left, then config order.
    """
    stats = stats or {}
    remaining = {shelf: Counter(counts) for shelf, counts in stock.items()}
    order = {arm["arm_id"]: index for index, arm in enumerate(arms)}

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

        def preference(option: tuple[Mapping[str, Any], Mapping[str, Any]]) -> tuple[float, int, int]:
            arm, policy = option
            return (
                -success_rate(stats, policy["policy_id"], arm["arm_id"]),
                -remaining[arm["shelf_id"]][object_class],
                order[arm["arm_id"]],
            )

        ranked = sorted(options, key=preference)
        arm, policy = ranked[0]
        reason = _arm_reason(ranked, object_class, stats, remaining)
        remaining[arm["shelf_id"]][object_class] -= 1
        placed.append((position, ArmPick(
            object_class=object_class, label=label_for(object_class), unit=int(unit["unit"]), of=int(unit["of"]),
            arm_id=arm["arm_id"], arm_label=arm["label"], shelf_id=arm["shelf_id"],
            policy_id=policy["policy_id"], reason=reason,
        )))

    # One arm finishes before the other starts (they share the loading spot): the busier arm goes first, and each
    # arm keeps the order's own unit order.
    load = Counter(pick.arm_id for _, pick in placed)
    placed.sort(key=lambda entry: (-load[entry[1].arm_id], order[entry[1].arm_id], entry[0]))
    picks = tuple(pick for _, pick in placed)
    turns = [f"{pick.arm_label} {load[pick.arm_id]}개" for index, pick in enumerate(picks)
             if index == 0 or picks[index - 1].arm_id != pick.arm_id]
    return ArmPlan(picks, " → ".join(turns))


def _arm_reason(
    ranked: Sequence[tuple[Mapping[str, Any], Mapping[str, Any]]],
    object_class: str,
    stats: PickStats,
    remaining: Mapping[str, Counter],
) -> str:
    """The first rule that separated the chosen arm from the runner-up, in words."""
    (arm, policy), *others = ranked
    if not others:
        return f"{arm['label']} 쪽에서만 집을 수 있음"
    other_arm, other_policy = others[0]
    rate = success_rate(stats, policy["policy_id"], arm["arm_id"])
    other_rate = success_rate(stats, other_policy["policy_id"], other_arm["arm_id"])
    if rate != other_rate:
        return f"성공률 {rate:.2f} vs {other_rate:.2f}"
    left, right = remaining[arm["shelf_id"]][object_class], remaining[other_arm["shelf_id"]][object_class]
    if left != right:
        return f"남은 재고 {left} vs {right}"
    return "조건이 같아 설정 순서대로"


def _no_arm_error(
    object_class: str, label: str, arms: Sequence[Mapping[str, Any]], remaining: Mapping[str, Counter]
) -> ArmPlanError:
    seen_on = [arm["label"] for arm in arms if remaining.get(arm["shelf_id"], Counter())[object_class] > 0]
    if not seen_on:
        return ArmPlanError("SHELF_STOCK_SHORT", f"{label}: 선반에 남은 수량이 주문보다 적습니다")
    return ArmPlanError("POLICY_NOT_FOR_ARM", f"{label}: 물체가 있는 쪽 팔({', '.join(seen_on)})에 학습된 정책이 없습니다")
