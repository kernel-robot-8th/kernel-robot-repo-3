"""Pinky and OMX assignment rules (pure functions; no devices, no database)."""

from __future__ import annotations

import unittest

from control_station.assignment import ArmPlanError, PinkyCandidate, choose_pinky, plan_arms, success_rate

ARMS = [
    {"arm_id": "arm-01", "label": "왼쪽 팔", "shelf_id": "shelf_a"},
    {"arm_id": "arm-02", "label": "오른쪽 팔", "shelf_id": "shelf_b"},
]
LABELS = {"apple": "사과", "carrot": "당근", "tissue": "물티슈"}


def resolver(missing: set[tuple[str, str]] = frozenset()):
    """Every object has a policy on every arm, except the (object, arm) pairs in `missing`."""
    def resolve(object_class: str, arm_id: str):
        if (object_class, arm_id) in missing:
            return None
        return {"policy_id": f"{object_class}_act"}
    return resolve


def units(*pairs: tuple[str, int]) -> list[dict]:
    return [{"object_class": name, "unit": index, "of": count}
            for name, count in pairs for index in range(1, count + 1)]


def parked(pinky_id: str, **overrides) -> PinkyCandidate:
    values = {"name": f"P{int(pinky_id) - 10}", "phase": "parked", "ready_for_motion": True, "battery_percent": 80.0}
    return PinkyCandidate(pinky_id, **{**values, **overrides})


class ChoosePinkyTest(unittest.TestCase):
    def test_each_exclusion_is_explained(self) -> None:
        result = choose_pinky([
            parked("10", holding_order="TASK-1"),
            parked("11", phase="driving"),
            parked("12", ready_for_motion=False, readiness_label="센서·위치 수신 대기"),
        ])
        self.assertIsNone(result.pinky_id)
        self.assertEqual(result.summary, "배정 가능한 핑키 없음")
        self.assertEqual([v.reason for v in result.verdicts],
                         ["주문 TASK-1 진행 중", "주차장 대기가 아님", "센서·위치 수신 대기"])

    def test_low_battery_and_unreachable_are_excluded(self) -> None:
        result = choose_pinky([parked("10", battery_percent=22), parked("11", reachable=False), parked("12")])
        self.assertEqual(result.pinky_id, "12")
        self.assertEqual(result.verdicts[0].reason, "배터리 22% · 충전 필요")
        self.assertEqual(result.verdicts[1].reason, "핑키 관제 연결 안 됨")

    def test_battery_band_beats_trip_count_but_small_differences_do_not(self) -> None:
        self.assertEqual(choose_pinky([parked("10", battery_percent=72), parked("11", battery_percent=91,
                                                                               trips_today=3)]).pinky_id, "11")
        # 81 % and 88 % share the 80 % band, so the Pinky that ran less today goes.
        self.assertEqual(choose_pinky([parked("10", battery_percent=88, trips_today=2),
                                       parked("11", battery_percent=81, trips_today=1)]).pinky_id, "11")

    def test_rest_time_then_id_break_ties(self) -> None:
        rested = choose_pinky([parked("10", last_finished_at="2026-11-14T10:30:00"),
                               parked("11", last_finished_at="2026-11-14T09:00:00")])
        self.assertEqual(rested.pinky_id, "11")
        self.assertEqual(choose_pinky([parked("11"), parked("10")]).pinky_id, "10")

    def test_ranks_and_summary(self) -> None:
        result = choose_pinky([parked("10", battery_percent=55), parked("11", battery_percent=85, trips_today=1)])
        self.assertEqual(result.summary, "P1 · 배터리 80%대 · 오늘 1회")
        self.assertEqual([(v.pinky_id, v.rank) for v in result.verdicts], [("10", 2), ("11", 1)])


class PlanArmsTest(unittest.TestCase):
    def plan(self, order, stock, *, missing=frozenset(), stats=None):
        return plan_arms(units(*order), stock, ARMS, resolver(missing), LABELS.get, stats)

    def test_object_on_one_shelf_only_is_placed_first(self) -> None:
        # Apples are on both shelves; the only carrot is on the left, so only the left arm can take it.
        plan = self.plan([("apple", 1), ("carrot", 1)], {"shelf_a": {"apple": 2, "carrot": 1}, "shelf_b": {"apple": 1}})
        by_object = {pick.object_class: pick for pick in plan.picks}
        self.assertEqual(by_object["carrot"].arm_id, "arm-01")
        self.assertEqual(by_object["carrot"].reason, "왼쪽 팔 쪽에서만 집을 수 있음")

    def test_scarce_shelf_is_not_starved(self) -> None:
        # Two apples: one on each shelf. The left arm alone cannot take both, so each arm gets one.
        plan = self.plan([("apple", 2)], {"shelf_a": {"apple": 1}, "shelf_b": {"apple": 1}})
        self.assertEqual(sorted(pick.arm_id for pick in plan.picks), ["arm-01", "arm-02"])

    def test_success_rate_then_remaining_stock_then_config_order(self) -> None:
        stock = {"shelf_a": {"apple": 3}, "shelf_b": {"apple": 3}}
        better_right = self.plan([("apple", 1)], stock, stats={("apple_act", "arm-02"): (8, 10)})
        self.assertEqual(better_right.picks[0].arm_id, "arm-02")
        self.assertEqual(better_right.picks[0].reason, "성공률 0.75 vs 0.50")
        more_left = self.plan([("apple", 1)], {"shelf_a": {"apple": 3}, "shelf_b": {"apple": 1}})
        self.assertEqual((more_left.picks[0].arm_id, more_left.picks[0].reason), ("arm-01", "남은 재고 3 vs 1"))
        tie = self.plan([("apple", 1)], stock)
        self.assertEqual((tie.picks[0].arm_id, tie.picks[0].reason), ("arm-01", "조건이 같아 설정 순서대로"))

    def test_arms_take_turns_busier_arm_first(self) -> None:
        plan = self.plan([("apple", 1), ("tissue", 2)],
                         {"shelf_a": {"apple": 1}, "shelf_b": {"tissue": 2}})
        self.assertEqual([pick.arm_id for pick in plan.picks], ["arm-02", "arm-02", "arm-01"])
        self.assertEqual(plan.summary, "오른쪽 팔 2개 → 왼쪽 팔 1개")

    def test_missing_policy_and_missing_stock_have_their_own_codes(self) -> None:
        with self.assertRaises(ArmPlanError) as no_policy:
            self.plan([("tissue", 1)], {"shelf_b": {"tissue": 1}}, missing={("tissue", "arm-02")})
        self.assertEqual(no_policy.exception.error_code, "POLICY_NOT_FOR_ARM")
        self.assertIn("오른쪽 팔", str(no_policy.exception))
        with self.assertRaises(ArmPlanError) as no_stock:
            self.plan([("carrot", 2)], {"shelf_a": {"carrot": 1}})
        self.assertEqual(no_stock.exception.error_code, "SHELF_STOCK_SHORT")

    def test_success_rate_is_smoothed(self) -> None:
        self.assertEqual(success_rate({}, "p", "arm-01"), 0.5)
        self.assertAlmostEqual(success_rate({("p", "arm-01"): (1, 1)}, "p", "arm-01"), 2 / 3)


if __name__ == "__main__":
    unittest.main()
