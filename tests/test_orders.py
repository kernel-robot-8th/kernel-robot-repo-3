"""Orders with several objects and quantities, Pinky assignment, arm-per-shelf config and arm-specific policies."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from control_station.act_policies import ActPolicyError, load_act_policies
from control_station.api.common import validate_order_items
from control_station.arm_commands import ArmCommandError, ArmCommandStore
from control_station.arms import ArmConfigError, load_arms
from control_station.command_parser import parse_order_text
from control_station.models import TaskStatus
from control_station.store import PinkyUnavailable, TaskStore
from test_api_routes import ApiTestCase

POLICY = {
    "policy_type": "act", "task": "Pick up Doll", "cameras": {"front": "OVERHEAD", "wrist": "GRIPPER"}, "max_run_s": 30,
}


def write_policies(directory: Path, policies: list[dict[str, object]]) -> Path:
    path = directory / "policies.json"
    path.write_text(json.dumps({"schema_version": 1, "policies": policies}), encoding="utf-8")
    return path


class ArmConfigTests(unittest.TestCase):
    def test_committed_config_puts_the_left_arm_on_the_left_shelf(self) -> None:
        arms = load_arms()
        self.assertEqual(arms.arm_ids, frozenset({"arm-01", "arm-02"}))
        self.assertEqual(arms.for_shelf("shelf_a")["arm_id"], "arm-01")
        self.assertEqual(arms.for_shelf("shelf_b")["arm_id"], "arm-02")

    def test_two_arms_on_one_shelf_are_refused(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "arms.json"
            arm = {"label": "팔", "shelf_id": "shelf_a", "stock_camera_id": "shelf_camera"}
            path.write_text(json.dumps({"schema_version": 1, "arms": [{"arm_id": "arm-01", **arm}, {"arm_id": "arm-02", **arm}]}))
            with self.assertRaisesRegex(ArmConfigError, "shelf_id"):
                load_arms(path)


class ArmPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def test_the_same_object_may_have_one_policy_per_arm(self) -> None:
        path = write_policies(self.directory, [
            {**POLICY, "policy_id": "carrot_left", "object_class": "carrot", "label": "당근", "aliases": [],
             "model": "a/left", "arm_id": "arm-01"},
            {**POLICY, "policy_id": "carrot_right", "object_class": "carrot", "label": "당근", "aliases": [],
             "model": "a/right", "arm_id": "arm-02"},
            {**POLICY, "policy_id": "apple_any", "object_class": "apple", "label": "사과", "aliases": [], "model": "a/apple"},
        ])
        registry = load_act_policies(path)
        self.assertEqual(registry.resolve("당근", "arm-02")["policy_id"], "carrot_right")
        self.assertEqual(registry.resolve("carrot", "arm-01")["policy_id"], "carrot_left")
        self.assertEqual(registry.resolve("사과", "arm-02")["policy_id"], "apple_any")  # no arm_id: any arm
        self.assertIsNone(registry.resolve("당근", "arm-03"))

    def test_a_name_twice_on_the_same_arm_is_refused(self) -> None:
        for arms in (("arm-01", "arm-01"), (None, "arm-02")):
            policies = [
                {**POLICY, "policy_id": f"p{index}", "object_class": "carrot", "label": "당근", "aliases": [],
                 "model": f"a/m{index}", **({"arm_id": arm} if arm else {})}
                for index, arm in enumerate(arms)
            ]
            with self.subTest(arms=arms), self.assertRaisesRegex(ActPolicyError, "same arm"):
                load_act_policies(write_policies(self.directory, policies))

    def test_policy_command_on_an_arm_without_that_policy_is_refused(self) -> None:
        path = write_policies(self.directory, [
            {**POLICY, "policy_id": "carrot_left", "object_class": "carrot", "label": "당근", "aliases": [],
             "model": "a/left", "arm_id": "arm-01"},
        ])
        with tempfile.TemporaryDirectory() as db:
            store = TaskStore(str(Path(db) / "t.sqlite3"))
            self.addCleanup(store.close)
            arm = ArmCommandStore(store.session_factory, act_policies=load_act_policies(path))
            request = {"operation": "PICK_WITH_POLICY", "valid_for_s": 60, "target": {"object_class": "carrot"},
                       "place": {"slot_id": "CART_1"}, "source": {"kind": "OPERATOR_INPUT"}}
            with self.assertRaisesRegex(ArmCommandError, "arm-02에는"):
                arm.create({"robot_id": "arm-02", **request})
            with self.assertRaisesRegex(ArmCommandError, "robot_id"):
                arm.create({"robot_id": "arm-03", **request})
            self.assertEqual(arm.create({"robot_id": "arm-01", **request})["target"]["policy"]["policy_id"], "carrot_left")


class OrderItemTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policies = load_act_policies()  # committed: carrot, tissue, apple

    def test_names_become_object_classes_and_repeats_are_added(self) -> None:
        items = validate_order_items(
            [{"object_class": "당근", "quantity": 1}, {"object_class": "apple", "quantity": 2},
             {"object_class": "carrot", "quantity": 2}],
            self.policies,
        )
        self.assertEqual(items, [{"object_class": "carrot", "quantity": 3}, {"object_class": "apple", "quantity": 2}])

    def test_invalid_orders_are_refused(self) -> None:
        for value, message in (
            ([], "1 to 5"),
            ([{"object_class": "banana", "quantity": 1}], "학습된 정책이 없습니다"),
            ([{"object_class": "당근", "quantity": 0}], "quantity"),
            ([{"object_class": "당근", "quantity": True}], "quantity"),
            ([{"object_class": "당근", "quantity": 3}, {"object_class": "carrot", "quantity": 3}], "5개까지"),
            ([{"object_class": "당근"}], "exactly"),
        ):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, message):
                validate_order_items(value, self.policies)
        with self.assertRaisesRegex(ValueError, "정책 정의 파일"):
            validate_order_items([{"object_class": "당근", "quantity": 1}], None)

    def test_order_sentences(self) -> None:
        cases = {
            "당근 1개랑 사과 두 개 핑키에 실어줘": [("carrot", 1), ("apple", 2)],
            "물티슈 하나, 당근": [("tissue", 1), ("carrot", 1)],
            "사과를 세 개 주세요": [("apple", 3)],
            "carrot x2 and apple": [("carrot", 2), ("apple", 1)],
            "티슈 2": [("tissue", 2)],  # 티슈 alone, not double-counted inside 물티슈
            "물티슈 2개": [("tissue", 2)],
        }
        for text, expected in cases.items():
            with self.subTest(text=text):
                draft = parse_order_text(text, self.policies)
                self.assertEqual([(item["object_class"], item["quantity"]) for item in draft["order_items"]], expected)
                self.assertFalse(draft["task_created"])

    def test_negated_and_unknown_objects_are_left_out_with_a_warning(self) -> None:
        draft = parse_order_text("사과 말고 당근 3개", self.policies)
        self.assertEqual(draft["order_items"], [{"object_class": "carrot", "quantity": 3}])
        self.assertIn("사과", draft["warnings"][0])
        self.assertEqual(parse_order_text("바나나 2개", self.policies)["order_items"], [])


class PinkyAssignmentTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.store = TaskStore(str(Path(directory.name) / "t.sqlite3"))
        self.addCleanup(self.store.close)
        self.items = [{"object_class": "carrot", "quantity": 1}]

    def test_free_pinkies_are_given_in_order_and_a_finished_order_frees_its_pinky(self) -> None:
        tasks = [self.store.create_task("핑키 적재 위치", "주문", order_items=self.items) for _ in range(3)]
        self.assertEqual([task["pinky_id"] for task in tasks], ["10", "11", "12"])
        with self.assertRaises(PinkyUnavailable):
            self.store.create_task("핑키 적재 위치", "주문", order_items=self.items)
        self.store.cancel_pending_task(tasks[1]["id"])
        self.assertEqual(self.store.create_task("핑키 적재 위치", "주문", order_items=self.items)["pinky_id"], "11")

    def test_a_stopped_order_keeps_its_pinky_until_acknowledged(self) -> None:
        task = self.store.create_task("A", "B", order_items=self.items)
        self.store.claim_task(task["id"])
        self.store.transition_task(task["id"], TaskStatus.FAILED, expected_status=TaskStatus.VALIDATING,
                                   payload={"event": "TEST_FAILED"})
        self.assertEqual(self.store.create_task("A", "B", order_items=self.items)["pinky_id"], "11")
        self.store.acknowledge_alert(task["id"], "확인")
        self.assertNotIn("10", self.store.busy_pinkies())

    def test_a_requested_pinky_must_be_free(self) -> None:
        self.assertEqual(self.store.create_task("A", "B", order_items=self.items, pinky_id="12")["pinky_id"], "12")
        with self.assertRaisesRegex(PinkyUnavailable, "12"):
            self.store.create_task("A", "B", order_items=self.items, pinky_id="12")
        self.assertEqual(self.store.busy_pinkies()["12"][:5], "TASK-")

    def test_each_shipment_counts_as_a_trip_of_its_own_pinky(self) -> None:
        task = self.store.create_task("A", "B", order_items=self.items, defer_pinky=True)
        self.store.assign_pinky(task["id"], "10", {})
        self.store.claim_task(task["id"])
        self.store.transition_task(task["id"], TaskStatus.RUNNING, expected_status=TaskStatus.VALIDATING,
                                   payload={"event": "TEST_RUNNING"})
        self.store.assign_shipment_pinky(task["id"], 2, "11", {})
        usage = self.store.pinky_usage("2000-01-01T00:00:00+00:00")
        self.assertEqual((usage["10"]["trips"], usage["11"]["trips"], usage["12"]["trips"]), (1, 1, 0))
        self.assertIsNotNone(usage["10"]["last_finished_at"])  # P0's trip ended when shipment 2 took P1

    def test_tasks_without_items_take_no_pinky(self) -> None:
        task = self.store.create_task("작업대 A", "상자 이동")
        self.assertIsNone(task["pinky_id"])
        self.assertIsNone(task["order_items"])
        self.assertEqual(task["status"], TaskStatus.PENDING.value)


class OrderRouteTests(ApiTestCase):
    def setUp(self) -> None:
        super().setUp()
        self.server.arm_commands.act_policies = load_act_policies()

    def test_order_is_stored_with_a_pinky_and_a_summary(self) -> None:
        status, body = self.post("/api/tasks", {"order_items": [{"object_class": "당근", "quantity": 1},
                                                                {"object_class": "사과", "quantity": 2}]})
        self.assertEqual(status, 201, body)
        task = body["task"]  # type: ignore[index]
        self.assertEqual(task["order_items"], [{"object_class": "carrot", "quantity": 1}, {"object_class": "apple", "quantity": 2}])
        self.assertEqual((task["pinky_id"], task["arm_job"], task["destination"]), ("10", "당근 1 · 사과 2 적재", "핑키 적재 위치"))

    def test_order_errors(self) -> None:
        carrot = [{"object_class": "carrot", "quantity": 1}]
        for body, status in (
            ({"order_items": carrot, "pick_request": {"category": "carrot"}}, 400),
            ({"order_items": carrot, "pinky_id": "13"}, 400),
            ({"destination": "A", "arm_job": "B", "pinky_id": "10"}, 400),
            ({"order_items": [{"object_class": "banana", "quantity": 1}]}, 400),
        ):
            with self.subTest(body=body):
                self.assertEqual(self.post("/api/tasks", body)[0], status)
        for _ in range(3):
            self.assertEqual(self.post("/api/tasks", {"order_items": carrot})[0], 201)
        status, body = self.post("/api/tasks", {"order_items": carrot})
        self.assertEqual((status, body["error_code"]), (409, "PINKY_UNAVAILABLE"))  # type: ignore[index]

    def test_parse_command_returns_an_order_draft(self) -> None:
        status, body = self.post("/api/tasks/parse-command", {"text": "당근 2개랑 물티슈 하나"})
        self.assertEqual(status, 200)
        self.assertEqual(body["order"]["order_items"],  # type: ignore[index]
                         [{"object_class": "carrot", "quantity": 2}, {"object_class": "tissue", "quantity": 1}])


if __name__ == "__main__":
    unittest.main()
