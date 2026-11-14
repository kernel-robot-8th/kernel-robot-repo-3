"""Pinky and OMX assignment rules (pure functions; no devices, no database)."""

from __future__ import annotations

import unittest

from control_station.assignment import (
    ArmPick, ArmPlanError, PickTiming, PinkyCandidate, ShipmentError, assign_fleet, choose_pinky, plan_arms,
    schedule_loading, split_into_shipments, success_rate,
)

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

    def test_every_check_and_ranking_key_is_reported(self) -> None:
        result = choose_pinky([parked("10", battery_percent=22), parked("11", trips_today=2)])
        low = result.verdicts[0]
        self.assertEqual([(c["rule"], c["ok"]) for c in low.checks],
                         [("진행 중 주문 없음", True), ("핑키 관제 연결", True), ("주차장 대기", True),
                          ("운행 준비", True), ("배터리 30% 이상", False)])
        self.assertEqual(result.verdicts[1].keys, {"battery_band": 80, "trips_today": 2, "last_finished_at": None})

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

    def test_each_pick_carries_the_deciding_rule_and_both_arms(self) -> None:
        plan = self.plan([("apple", 1)], {"shelf_a": {"apple": 3}, "shelf_b": {"apple": 1}}, missing={("apple", "arm-02")})
        pick = plan.picks[0]
        self.assertEqual(pick.rule, "only_option")
        self.assertEqual([(o["arm_id"], o["possible"], o["why"], o["chosen"]) for o in pick.options],
                         [("arm-01", True, None, True), ("arm-02", False, "정책 없음", False)])
        self.assertEqual(pick.options[0]["remaining"], 3)  # stock as it stood before this unit was taken

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


def sizes(**overrides: int):
    return lambda object_class: overrides.get(object_class, 1)


class SplitIntoShipmentsTest(unittest.TestCase):
    def test_fewest_pinkys_with_even_loads(self) -> None:
        shipments = split_into_shipments(units(("apple", 4), ("carrot", 3)), 3, sizes(), LABELS.get)
        self.assertEqual([shipment.load for shipment in shipments], [3, 2, 2])  # not 3, 3, 1
        self.assertEqual(sum(len(shipment.units) for shipment in shipments), 7)

    def test_big_objects_take_two_slots_of_four(self) -> None:
        # The configured tray: 4 slots, tissue 2. Two tissues and two apples need two Pinkys.
        shipments = split_into_shipments(units(("tissue", 2), ("apple", 2)), 4, sizes(tissue=2), LABELS.get)
        self.assertEqual([shipment.load for shipment in shipments], [3, 3])

    def test_sizes_count_against_the_tray(self) -> None:
        shipments = split_into_shipments(units(("tissue", 2), ("apple", 2)), 3, sizes(tissue=2), LABELS.get)
        self.assertEqual([sorted(u["object_class"] for u in s.units) for s in shipments],
                         [["apple", "tissue"], ["apple", "tissue"]])

    def test_a_pinky_is_added_when_a_unit_fits_nowhere(self) -> None:
        # Total 8 fits two trays of 4 on paper, but 3 + 3 + 2 cannot: a third Pinky is needed.
        shipments = split_into_shipments(units(("tissue", 2), ("apple", 1)), 4, sizes(tissue=3, apple=2), LABELS.get)
        self.assertEqual([shipment.load for shipment in shipments], [3, 3, 2])

    def test_a_unit_bigger_than_the_tray_is_refused(self) -> None:
        with self.assertRaises(ShipmentError) as error:
            split_into_shipments(units(("tissue", 1)), 2, sizes(tissue=3), LABELS.get)
        self.assertEqual(error.exception.error_code, "ITEM_TOO_BIG_FOR_PINKY")


class AssignFleetTest(unittest.TestCase):
    def test_heaviest_shipment_goes_to_the_best_pinky(self) -> None:
        shipments = split_into_shipments(units(("apple", 5)), 3, sizes(), LABELS.get)
        plan = assign_fleet(shipments, [parked("10", battery_percent=55), parked("11", battery_percent=95)])
        self.assertEqual([(trip.pinky_id, trip.shipment.load) for trip in plan.trips], [("11", 3), ("10", 2)])
        self.assertEqual(plan.summary, "핑키 2대 · P1 3, P0 2")
        self.assertTrue(plan.complete)

    def test_shipments_beyond_the_free_pinkys_wait(self) -> None:
        shipments = split_into_shipments(units(("apple", 7)), 3, sizes(), LABELS.get)
        plan = assign_fleet(shipments, [parked("10"), parked("11"), parked("12", holding_order="TASK-9")])
        self.assertEqual([trip.pinky_id for trip in plan.trips], ["10", "11", None])
        self.assertEqual(plan.trips[-1].reason, "빈 핑키를 기다림")
        self.assertEqual(plan.summary, "핑키 3대 필요 · 지금 2대 가능")
        self.assertFalse(plan.complete)


def arm_pick(arm_id: str, object_class: str, unit: int) -> ArmPick:
    return ArmPick(object_class, LABELS[object_class], unit, 2, arm_id, arm_id, "shelf", f"{object_class}_act", "")


class ScheduleLoadingTest(unittest.TestCase):
    TIMING = staticmethod(lambda pick: PickTiming(shelf_s=8, place_s=5))

    def test_arms_overlap_and_take_turns_over_the_pinky(self) -> None:
        picks = [arm_pick("arm-01", "apple", 1), arm_pick("arm-01", "apple", 2), arm_pick("arm-02", "carrot", 1)]
        schedule = schedule_loading(picks, self.TIMING, ["arm-01", "arm-02"])
        self.assertEqual([(p.pick.arm_id, p.place_start_s, p.place_end_s) for p in schedule.picks],
                         [("arm-01", 8, 13), ("arm-02", 13, 18), ("arm-01", 21, 26)])
        self.assertEqual(schedule.picks[1].waited_s, 5)  # the right arm held the carrot while the left one placed
        self.assertEqual((schedule.total_s, schedule.one_arm_at_a_time_s), (26, 39))

    def test_without_overlap_whole_picks_follow_each_other(self) -> None:
        picks = [arm_pick("arm-02", "apple", 1), arm_pick("arm-01", "carrot", 1)]
        schedule = schedule_loading(picks, self.TIMING, ["arm-01", "arm-02"], overlap=False)
        self.assertEqual([(p.pick.arm_id, p.shelf_start_s, p.place_end_s) for p in schedule.picks],
                         [("arm-02", 0, 13), ("arm-01", 13, 26)])
        self.assertEqual((schedule.total_s, schedule.overlap), (26, False))

    def test_one_arm_gains_nothing(self) -> None:
        picks = [arm_pick("arm-01", "apple", 1), arm_pick("arm-01", "apple", 2)]
        schedule = schedule_loading(picks, self.TIMING, ["arm-01", "arm-02"])
        self.assertEqual((schedule.total_s, schedule.one_arm_at_a_time_s), (26, 26))


if __name__ == "__main__":
    unittest.main()
