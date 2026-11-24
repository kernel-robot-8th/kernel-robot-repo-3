"""누가 주문을 맡는가: 운행을 맡을 핑키 한 대, 수령 구역, 그리고 실을 물체 한 개마다 OMX 팔 한 대.

선택은 모두 가중치 점수가 아니라 차례가 있는 규칙 목록이다. 그래서 모든 결정이 운영자 화면에서 한 문장으로
읽힌다("P1 · 배터리 80%대 · 오늘 1회"). 여기서는 장치나 DB와 말하지 않는다: 부르는 쪽(desk.py, flow.py)이 본 것을
넘겨주므로, 규칙이 모의·실제 모드에서 같고 시험하기 쉽다.

구성: 핑키 고르기(choose_pinky) / 수령 구역 고르기(choose_zone) / 팔 배정(plan_arms) /
큰 주문을 운송분으로 나누기(split_into_shipments, assign_fleet) / 두 팔의 적재 시간표(schedule_loading).
"""

from __future__ import annotations

import math
from collections import Counter, deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
from functools import partial
from typing import Any

MIN_BATTERY_PERCENT = 30.0  # 배터리가 이보다 낮은 핑키(%)는 새 주문에서 뺀다


# ---------------------------------------------------------------------------------------------------------------
# 핑키
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PinkyCandidate:
    """주문을 배정하는 순간 핑키 하나에 대해 알고 있는 것."""

    pinky_id: str
    name: str
    holding_order: str | None = None  # 이 핑키를 쥐고 있는 끝나지 않은 주문
    reachable: bool = True  # 핑키 관제가 이 핑키에 대해 답했다
    phase: str | None = None  # 핑키 관제가 보고한 임무 단계("parked", "driving", ...)
    ready_for_motion: bool | None = None  # 핑키 관제의 운행 준비 여부. 출처가 알려 주지 않으면 None
    readiness_label: str | None = None  # 준비 안 된 이유(사람용 글자)
    battery_percent: float | None = None
    trips_today: int = 0
    last_finished_at: str | None = None  # 마지막 주문이 끝난 ISO 시각. 한 번도 안 달렸으면 None


@dataclass(frozen=True)
class PinkyVerdict:
    """핑키 하나의 판정: 갈 수 있는지, 이유, 순위, 규칙별 통과 여부, 순위 비교값."""

    pinky_id: str
    name: str
    eligible: bool
    reason: str  # 빠졌으면 첫 번째로 걸린 이유, 아니면 비교값 요약
    rank: int | None = None  # 1 = 뽑힘. 빠졌으면 None
    # 모든 제외 규칙을 {rule, ok, detail}로. 첫 번째로 걸린 것만이 아니라 표 전체를 화면에 보이려고.
    checks: tuple[dict[str, Any], ...] = ()
    # 순위에서 비교한 값, 차례대로: 배터리 구간(%), 오늘 운행 횟수, 마지막으로 끝난 시각.
    keys: dict[str, Any] | None = None


@dataclass(frozen=True)
class PinkyAssignment:
    """핑키 고르기 결과: 뽑힌 핑키(없으면 None), 한 줄 요약, 핑키마다의 판정."""

    pinky_id: str | None
    summary: str
    verdicts: tuple[PinkyVerdict, ...]

    def as_dict(self) -> dict[str, Any]:
        return {"pinky_id": self.pinky_id, "summary": self.summary, "verdicts": [asdict(v) for v in self.verdicts]}


def _exclusion(candidate: PinkyCandidate, min_battery: float) -> str | None:
    """이 핑키가 지금 새 주문을 맡을 수 없는 이유, 맡을 수 있으면 None. 배터리를 모르면 배터리로는 빼지 않는다."""
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
    """제외 규칙마다 통과했는지, `_exclusion`이 적용하는 순서대로."""
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
    """순위에서 비교한 값(화면용). 배터리 구간은 % 단위로 보인다(80 = 80%대, 모르면 None)."""
    band = _battery_band(candidate)
    return {"battery_band": None if band < 0 else band * 10, "trips_today": candidate.trips_today,
            "last_finished_at": candidate.last_finished_at}


def _battery_band(candidate: PinkyCandidate) -> int:
    # 10 % 구간으로 본다: 1~2 % 차이로 선택이 뒤집히지 않게. 배터리를 모르면(-1) 아는 어떤 핑키보다 뒤다.
    return -1 if candidate.battery_percent is None else int(candidate.battery_percent // 10)


def _preference(candidate: PinkyCandidate) -> tuple[int, int, str, str]:
    """정렬 키, 좋은 것부터: 배터리 구간이 높은 순 → 오늘 운행이 적은 순 → 오래 쉰 순(마지막 끝난 시각이 이른 순,
    한 번도 안 달렸으면 맨 앞) → 번호 순."""
    return (-_battery_band(candidate), candidate.trips_today, candidate.last_finished_at or "", candidate.pinky_id)


def _describe(candidate: PinkyCandidate) -> str:
    """화면에 보일 한 줄(예: "배터리 80%대 · 오늘 1회")."""
    band = _battery_band(candidate)
    battery = "배터리 정보 없음" if band < 0 else f"배터리 {band * 10}%대"
    return f"{battery} · 오늘 {candidate.trips_today}회"


def choose_pinky(
    candidates: Sequence[PinkyCandidate], *, min_battery: float = MIN_BATTERY_PERCENT
) -> PinkyAssignment:
    """주문 하나의 핑키를 고른다: 갈 수 없는 핑키를 빼고, 남은 핑키를 `_preference`로 순위를 매겨 1등을 고른다.

    남은 핑키가 없으면 pinky_id=None("배정 가능한 핑키 없음")이다. 판정은 빠진 핑키까지 모두 담는다."""
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
# 수령 구역
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ZoneCandidate:
    """주문을 시작하는 순간의 수령 구역 하나(config/receiving-zones.json 순서로 넘긴다)."""

    zone_id: str
    label: str
    route_length: int  # 거기까지 가며 지나는 웨이포인트 수. 적을수록 짧은 길이다
    holding_order: str | None = None  # 시작했고 아직 거기서 수령되지 않은 주문이 구역을 쥔다
    uses_today: int = 0  # 오늘 시작해 이 구역으로 간 주문 수


@dataclass(frozen=True)
class ZoneVerdict:
    """구역 하나의 판정(PinkyVerdict와 같은 모양)."""

    zone_id: str
    label: str
    eligible: bool
    reason: str
    rank: int | None = None  # 1 = 뽑힘. 빠졌으면 None
    checks: tuple[dict[str, Any], ...] = ()
    keys: dict[str, Any] | None = None  # 순위에서 비교한 값: 오늘 사용 횟수, 경로 길이, 설정 순서


@dataclass(frozen=True)
class ZoneAssignment:
    """구역 고르기 결과. zone_id가 None이면 쓸 구역이 없고 summary가 이유다."""

    zone_id: str | None
    summary: str
    verdicts: tuple[ZoneVerdict, ...]
    operator_chosen: bool = False  # 구역이 주문과 함께 왔고(작업자 지정) 비었는지만 확인했다

    def as_dict(self) -> dict[str, Any]:
        return {"zone_id": self.zone_id, "summary": self.summary, "operator_chosen": self.operator_chosen,
                "verdicts": [asdict(v) for v in self.verdicts]}


def _zone_reason(candidate: ZoneCandidate) -> str:
    """화면에 보일 한 줄(예: "오늘 1회 · 경로 2곳")."""
    return f"오늘 {candidate.uses_today}회 · 경로 {candidate.route_length}곳"


def _zone_verdicts(candidates: Sequence[ZoneCandidate]) -> tuple[tuple[ZoneVerdict, ...], list[ZoneCandidate]]:
    """(구역마다의 판정, 빈 구역을 좋은 순서로). 순서: 오늘 덜 쓴 구역 → 짧은 경로 → 설정 순서."""
    order = {c.zone_id: index for index, c in enumerate(candidates)}
    free = sorted((c for c in candidates if not c.holding_order),
                  key=lambda c: (c.uses_today, c.route_length, order[c.zone_id]))
    rank_of = {c.zone_id: index for index, c in enumerate(free, start=1)}
    verdicts = tuple(
        ZoneVerdict(
            c.zone_id, c.label, not c.holding_order,
            f"주문 {c.holding_order}이 사용 중" if c.holding_order else _zone_reason(c),
            rank_of.get(c.zone_id),
            ({"rule": "비어 있음(시작~수령 확인 중인 주문 없음)", "ok": not c.holding_order,
              "detail": f"주문 {c.holding_order}" if c.holding_order else None},),
            {"uses_today": c.uses_today, "route_length": c.route_length, "config_order": order[c.zone_id] + 1},
        )
        for c in candidates
    )
    return verdicts, free


def choose_zone(candidates: Sequence[ZoneCandidate], *, requested: str | None = None) -> ZoneAssignment:
    """핑키가 이 주문을 가져갈 곳: 시작한 다른 주문이 아직 쓰지 않는 구역 중에서, 오늘 덜 쓴 구역 → 짧은 경로 →
    설정 순서로 고른다. 두 주문이 한 구역을 나눠 쓰는 일은 없으므로, 손님이 거기서 남의 핑키를 만나지 않는다.

    `requested`는 주문과 함께 온 구역(작업자 지정)이다: 다른 구역으로 바꾸지 않고 비었는지만 확인한다. 자기가
    고른 자리를 기다리는 손님처럼. 비어 있지 않거나 설정에 없으면 zone_id=None이다."""
    verdicts, free = _zone_verdicts(candidates)
    if requested is not None:
        wanted = next((c for c in candidates if c.zone_id == requested), None)
        if wanted is None:
            return ZoneAssignment(None, f"설정에 없는 수령 구역이에요 ({requested})", verdicts, operator_chosen=True)
        if wanted.holding_order:
            return ZoneAssignment(None, f"작업자가 고른 구역({wanted.label})은 주문 {wanted.holding_order}이 쓰고 있어요",
                                  verdicts, operator_chosen=True)
        return ZoneAssignment(wanted.zone_id, f"{wanted.label} · 작업자 지정", verdicts, operator_chosen=True)
    if not free:
        busy = ", ".join(f"{c.label}: 주문 {c.holding_order}" for c in candidates) or "설정된 구역 없음"
        return ZoneAssignment(None, f"수령 구역이 모두 사용 중이에요 ({busy})", verdicts)
    chosen = free[0]
    return ZoneAssignment(chosen.zone_id, f"{chosen.label} · {_zone_reason(chosen)}", verdicts)


# ---------------------------------------------------------------------------------------------------------------
# OMX 팔
# ---------------------------------------------------------------------------------------------------------------

# (policy_id, arm_id) → 끝난 옮김 확인에서 센 (성공 수, 시도 수). TaskStore.pick_success_stats가 만든다.
PickStats = Mapping[tuple[str, str], tuple[int, int]]
# (물체, 팔 id) → 그 팔에서 그 물체를 집도록 학습된 정책, 없으면 None.
PolicyResolver = Callable[[str, str], Mapping[str, Any] | None]


class ArmPlanError(ValueError):
    """팔 계획을 세울 수 없다. error_code는 SHELF_STOCK_SHORT 또는 POLICY_NOT_FOR_ARM이다."""

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


@dataclass(frozen=True)
class ArmPick:
    """주문의 물체 한 개(단위), 그것을 집을 팔, 그 팔인 이유."""

    object_class: str
    label: str
    unit: int  # 같은 물체 중 몇 번째인지(1부터)
    of: int  # 같은 물체의 주문 수량
    arm_id: str
    arm_label: str
    shelf_id: str
    policy_id: str
    reason: str
    # 어느 규칙이 정했는지: only_option, success_rate, balance, stock, config 중 하나(plan_arms 참고).
    rule: str = ""
    # 이 단위를 배정할 때 모든 팔의 형편: {arm_id, arm_label, possible, why, success_rate,
    # assigned, remaining, chosen}.
    options: tuple[dict[str, Any], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ArmPlan:
    """팔 계획: 실행 순서대로의 집기와 한 줄 요약(예: "왼쪽 팔 2개 → 오른쪽 팔 1개")."""

    picks: tuple[ArmPick, ...]  # 실행 순서대로
    summary: str

    def as_dict(self) -> dict[str, Any]:
        return {"summary": self.summary, "picks": [pick.as_dict() for pick in self.picks]}


def success_rate(stats: PickStats, policy_id: str, arm_id: str) -> float:
    """라플라스 보정한 집기 성공률 (s + 1) / (n + 2): 기록이 없으면 0.5이고, 결과 하나로는 조금만 움직인다."""
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
    """단위마다 팔을 정하고, 하나뿐인 적재 자리에서 팔이 차례를 지키도록 집기 순서를 정한다.

    팔은 자기 선반에서만, 학습된 정책이 있는 물체만 집고, 어떤 선반도 카메라가 센 것보다 많이 내주지 않는다.
    집을 수 있는 팔이 가장 적은 단위부터 정한다(가장 제약이 큰 것 먼저). 그래서 한 선반에만 있는 물체가 두 팔이 다
    집을 수 있는 물체에 밀려 굶지 않는다. 두 팔이 다 되면 성공률이 높은 팔 → 지금까지 맡은 개수가 적은 팔(집기가
    겹칠 수 있다면 두 팔이 함께 바쁠 때 더 빨리 끝난다) → 그 물체가 더 많이 남은 선반 → 설정 순서로 고른다.

    units는 {object_class, unit, of} dict들, stock은 선반 → 물체 → 개수, arms는 {arm_id, label, shelf_id} 설정이다.
    어떤 단위도 맡을 팔이 없으면 ArmPlanError.
    """
    stats = stats or {}
    remaining = {shelf: Counter(counts) for shelf, counts in stock.items()}
    order = {arm["arm_id"]: index for index, arm in enumerate(arms)}
    assigned: Counter = Counter()  # 지금까지 팔마다 맡긴 단위 수

    def workable(object_class: str) -> list[tuple[Mapping[str, Any], Mapping[str, Any]]]:
        """이 물체를 지금 집을 수 있는 (팔, 정책): 정책이 있고 그 팔의 선반에 아직 남은 팔."""
        options = []
        for arm in arms:
            policy = resolve_policy(object_class, arm["arm_id"])
            if policy is not None and remaining.get(arm["shelf_id"], Counter())[object_class] > 0:
                options.append((arm, policy))
        return options

    pending = list(enumerate(units))
    placed: list[tuple[int, ArmPick]] = []
    while pending:
        # 집을 수 있는 팔이 가장 적은 단위(같으면 주문 순서)를 먼저 정한다. 남은 수량이 줄 때마다 다시 센다.
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
        # 화면용 기록은 이 단위를 빼기 전의 형편으로 남긴다.
        options = _arm_options(arms, object_class, resolve_policy, stats, remaining, assigned, arm["arm_id"])
        remaining[arm["shelf_id"]][object_class] -= 1
        assigned[arm["arm_id"]] += 1
        placed.append((position, ArmPick(
            object_class=object_class, label=label_for(object_class), unit=int(unit["unit"]), of=int(unit["of"]),
            arm_id=arm["arm_id"], arm_label=arm["label"], shelf_id=arm["shelf_id"],
            policy_id=policy["policy_id"], reason=reason, rule=rule, options=options,
        )))

    # 한 팔이 끝나야 다른 팔이 시작한다(적재 자리를 같이 쓰므로): 맡은 개수가 많은 팔이 먼저 하고, 팔마다 주문의
    # 단위 순서를 지킨다.
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
    """단위를 맡을 수 있는 팔의 정렬 키, 좋은 것부터(plan_arms 참고): 성공률 높은 순, 맡은 개수 적은 순,
    남은 수량 많은 순, 설정 순서."""
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
    """(규칙, 설명): 뽑힌 팔과 2등을 처음으로 가른 규칙. 팔이 하나뿐이면 only_option."""
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
    """이 단위에 대해 팔마다의 형편: 맡을 수 없었던 이유, 또는 규칙이 비교한 값."""
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
    """팔을 못 찾은 이유: 어느 팔의 선반에도 남지 않았으면 SHELF_STOCK_SHORT, 남았는데 그 팔에 정책이 없으면
    POLICY_NOT_FOR_ARM."""
    seen_on = [arm["label"] for arm in arms if remaining.get(arm["shelf_id"], Counter())[object_class] > 0]
    if not seen_on:
        return ArmPlanError("SHELF_STOCK_SHORT", f"{label}: 선반에 남은 수량이 주문보다 적습니다")
    return ArmPlanError("POLICY_NOT_FOR_ARM", f"{label}: 물체가 있는 쪽 팔({', '.join(seen_on)})에 학습된 정책이 없습니다")


# ---------------------------------------------------------------------------------------------------------------
# 한 주문에 핑키 여러 대
# ---------------------------------------------------------------------------------------------------------------


class ShipmentError(ValueError):
    """주문을 운송분으로 나눌 수 없다(ITEM_TOO_BIG_FOR_PINKY: 핑키 한 대에 실리지 않는 물체)."""

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


@dataclass(frozen=True)
class Shipment:
    """핑키 한 대가 주문에서 나르는 단위들과 그것이 차지하는 쟁반 칸 수(load)."""

    units: tuple[Mapping[str, Any], ...]
    load: int


def split_into_shipments(
    units: Sequence[Mapping[str, Any]],
    capacity: int,
    size_of: Callable[[str], int],
    label_for: Callable[[str], str],
) -> tuple[Shipment, ...]:
    """쟁반 칸 수가 허락하는 가장 적은 핑키에 주문을 나누고, 짐은 고르게 펼친다.

    전체 크기를 담을 수 있는 최소 대수부터 시작해, 큰 단위부터 아직 들어가는 쟁반 중 가장 덜 찬 쟁반에 놓는다.
    어디에도 안 들어가는 단위가 생길 때만 핑키를 한 대 늘려 다시 한다. 결과는 무거운 운송분부터다.
    단위마다 크기가 capacity 이하임을 먼저 확인하므로, 핑키 수가 단위 수에 이르면 반드시 끝난다."""
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
                break  # 이 대수로는 안 된다: 아래 else를 건너뛰고 한 대 늘린다
            tray = min(fitting, key=lambda t: (loads[t], t))
            trays[tray].append(index)
            loads[tray] += sizes[index]
        else:  # 모든 단위가 들어갔다
            shipments = [Shipment(tuple(units[i] for i in sorted(tray)), load) for tray, load in zip(trays, loads, strict=True)]
            return tuple(sorted(shipments, key=lambda shipment: -shipment.load))
        count += 1


@dataclass(frozen=True)
class Trip:
    """운송분 하나와 그것을 나를 핑키. 핑키가 없으면 빈 핑키를 기다린다."""

    shipment: Shipment
    pinky_id: str | None
    pinky_name: str | None
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {"units": [dict(unit) for unit in self.shipment.units], "load": self.shipment.load,
                "pinky_id": self.pinky_id, "pinky_name": self.pinky_name, "reason": self.reason}


@dataclass(frozen=True)
class FleetPlan:
    """운송분마다의 핑키 계획(미리보기용)."""

    trips: tuple[Trip, ...]
    pinky: PinkyAssignment  # 모든 핑키의 판정과 순위
    summary: str

    @property
    def complete(self) -> bool:
        """모든 운송분에 지금 핑키가 있는지."""
        return all(trip.pinky_id is not None for trip in self.trips)

    def as_dict(self) -> dict[str, Any]:
        return {"summary": self.summary, "complete": self.complete,
                "trips": [trip.as_dict() for trip in self.trips], "pinky": self.pinky.as_dict()}


def assign_fleet(
    shipments: Sequence[Shipment], candidates: Sequence[PinkyCandidate], *, min_battery: float = MIN_BATTERY_PERCENT
) -> FleetPlan:
    """가장 무거운 운송분을 1등 핑키에게, 다음 것을 2등에게 준다. 남는 운송분은 빈 핑키를 기다린다."""
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
# 두 팔, 적재 자리 하나
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PickTiming:
    """집기 하나가 걸리는 시간(초). 팔이 핑키 위 공용 공간에 들어가는 순간에서 둘로 나눈다."""

    shelf_s: float  # 자기 선반에서: 잡아서 대기 자세까지 들어 올림. 적재 공간 밖
    place_s: float  # 대기 자세 → 핑키 위 → 놓기 → 빠져나옴. 적재 공간 안


@dataclass(frozen=True)
class TimedPick:
    """시간표 위의 집기 하나. 시각은 적재 시작부터의 초."""

    pick: ArmPick
    shelf_start_s: float
    place_start_s: float
    place_end_s: float
    waited_s: float  # 다른 팔이 핑키 위에 있어 대기 자세로 기다린 시간

    def as_dict(self) -> dict[str, Any]:
        return {**self.pick.as_dict(), "shelf_start_s": self.shelf_start_s, "place_start_s": self.place_start_s,
                "place_end_s": self.place_end_s, "waited_s": self.waited_s}


@dataclass(frozen=True)
class LoadingSchedule:
    """적재 시간표: 집기들, 전체 시간, 한 팔씩 통째로 했을 때의 시간(비교용), 겹치기 여부."""

    picks: tuple[TimedPick, ...]  # 핑키에 닿는 순서대로
    total_s: float
    one_arm_at_a_time_s: float
    overlap: bool = True

    def as_dict(self) -> dict[str, Any]:
        return {"total_s": self.total_s, "one_arm_at_a_time_s": self.one_arm_at_a_time_s, "overlap": self.overlap,
                "picks": [pick.as_dict() for pick in self.picks]}


def schedule_loading(
    picks: Sequence[ArmPick], timing: Callable[[ArmPick], PickTiming], arm_ids: Sequence[str], *, overlap: bool = True
) -> LoadingSchedule:
    """집기마다 언제 일어나는지 계산한다(미리보기용 추정, 장치는 움직이지 않는다).

    `overlap`이 없으면 계획 순서대로 집기 하나를 통째로 하나씩: ACT 정책이 잡기 → 놓기 → 쉬기를 한 번에 돌리는
    지금 팔이 하는 그대로다. 있으면 핑키 위 적재 공간만이 공용 자원이고 잠금이 된다: 한 팔이 놓는 동안 다른 팔은
    자기 선반에서 잡고, 공간이 빌 때까지 대기 자세로 기다린다. 먼저 준비된 팔이 먼저 놓는다(같으면 남은 집기가
    많은 팔, 그다음 설정 순서). 그래서 두 팔 모두 일이 있으면 번갈아 놓는다. 팔마다 계획의 집기 순서는 지킨다."""
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
        # (선반 쪽을 마치고 준비되는 시각, 남은 집기가 많은 팔 먼저, 설정 순서)가 가장 작은 팔이 다음에 놓는다.
        ready_at, _, _, arm = min(
            (free_at[arm] + timing(queue[0]).shelf_s, -len(queue), position, arm)
            for position, (arm, queue) in enumerate(queues.items()) if queue
        )
        pick = queues[arm].popleft()
        place_start = max(ready_at, space_free_at)
        place_end = place_start + timing(pick).place_s
        timeline.append(TimedPick(pick, free_at[arm], place_start, place_end, place_start - ready_at))
        # 놓고 빠져나와야 그 팔도 다음 집기를 시작하고 적재 공간도 빈다.
        free_at[arm] = space_free_at = place_end
    return LoadingSchedule(tuple(timeline), max((p.place_end_s for p in timeline), default=0.0), sequential, overlap=True)
