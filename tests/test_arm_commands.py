from __future__ import annotations

import json
import tempfile
import threading
import time
import unittest
from functools import partial
from pathlib import Path
from unittest import mock
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from control_station import arm_commands
from control_station.act_policies import load_act_policies
from control_station.api import arm as arm_api
from control_station.app import ControlStationHandler, ControlStationServer, WEB_DIR
from control_station.arm_commands import ArmCommandConflict, ArmCommandError, ArmCommandStore
from control_station.shelf_slot_map import ShelfSlotMapError, load_shelf_slot_map
from control_station.store import TaskStore


def command_request(**overrides: object) -> dict[str, object]:
    request: dict[str, object] = {
        "robot_id": "arm-01",
        "operation": "PICK_AND_PLACE",
        "valid_for_s": 60,
        "target": {
            "object_class": "strawberry",
            "frame_id": "omx_link0",
            "unit": "m",
            "position": {"x": 0.21, "y": -0.05, "z": 0.12},
            "uncertainty_m": {"x": 0.01, "y": 0.03, "z": 0.01},
        },
        "place": {"slot_id": "CART_1"},
        "source": {"kind": "OPERATOR_INPUT"},
    }
    request.update(overrides)
    return request


class ArmCommandStoreTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.task_store = TaskStore(str(Path(directory.name) / "tasks.sqlite3"))
        self.addCleanup(self.task_store.close)
        self.store = ArmCommandStore(self.task_store.session_factory, ack_timeout_s=10)

    def event(self, command: dict, event: str, event_id: str | None = None, **fields: object):
        body = {
            "event_id": event_id or f"{event}-1",
            "robot_id": "arm-01",
            "worker_id": "w1",
            "event": event,
            "simulated": True,
            **fields,
        }
        return self.store.apply_worker_event(command["command_id"], body)

    def claimed(self) -> dict:
        self.store.create(command_request())
        command = self.store.claim_next("arm-01", "w1", 0)
        self.assertIsNotNone(command)
        return command

    def test_full_success_path_records_each_state(self) -> None:
        command = self.claimed()
        self.assertEqual(command["state"], "DISPATCHED")
        self.assertEqual(command["target"]["frame_id"], "omx_link0")
        self.assertEqual(self.event(command, "ACCEPTED")[0]["state"], "ACCEPTED")
        self.assertEqual(self.event(command, "PROGRESS", phase="GRASP_TARGET")[0]["state"], "RUNNING")
        final, applied = self.event(command, "SUCCEEDED", grasp_confirmed=True, place_confirmed=True)
        self.assertTrue(applied)
        self.assertEqual(final["state"], "SUCCEEDED")
        self.assertTrue(final["result"]["simulated"])
        events = [item["event_type"] for item in self.store.get(command["command_id"])["events"]]
        self.assertEqual(events, ["CREATED", "DISPATCHED", "ACCEPTED", "PROGRESS", "SUCCEEDED"])

    def test_success_requires_grasp_and_place_confirmation(self) -> None:
        command = self.claimed()
        self.event(command, "ACCEPTED")
        with self.assertRaises(ArmCommandError):
            self.event(command, "SUCCEEDED", grasp_confirmed=True)
        unknown, _ = self.event(command, "RESULT_UNKNOWN", reason_code="NO_GRASP_SENSOR")
        self.assertEqual(unknown["state"], "UNKNOWN")
        self.assertTrue(unknown["blocks_new_commands"])

    def test_only_one_unfinished_command_per_arm(self) -> None:
        self.store.create(command_request())
        with self.assertRaises(ArmCommandConflict) as raised:
            self.store.create(command_request())
        self.assertEqual(raised.exception.error_code, "ARM_COMMAND_ACTIVE")

    def test_invalid_requests_are_rejected(self) -> None:
        for request in (
            command_request(robot_id="arm-02"),
            command_request(valid_for_s=1),
            command_request(target={**command_request()["target"], "unit": "mm"}),
            command_request(
                target={**command_request()["target"], "position": {"x": 210, "y": 0, "z": 0}}
            ),
            command_request(source={"kind": "CAMERA"}),
            {**command_request(), "extra": 1},
        ):
            with self.subTest(request=request), self.assertRaises(ArmCommandError):
                self.store.create(request)

    def test_repeated_event_id_is_idempotent(self) -> None:
        command = self.claimed()
        _, applied = self.event(command, "ACCEPTED", event_id="e1")
        again, applied_again = self.event(command, "ACCEPTED", event_id="e1")
        self.assertTrue(applied)
        self.assertFalse(applied_again)
        self.assertEqual(again["state"], "ACCEPTED")
        with self.assertRaises(ArmCommandConflict) as raised:
            self.event(command, "FAILED", event_id="e1", reason_code="X")
        self.assertEqual(raised.exception.error_code, "EVENT_ID_REUSED")

    def test_reject_needs_reason_and_other_worker_is_refused(self) -> None:
        command = self.claimed()
        with self.assertRaises(ArmCommandError):
            self.event(command, "REJECTED")
        with self.assertRaises(ArmCommandConflict) as raised:
            self.store.apply_worker_event(
                command["command_id"],
                {"event_id": "x", "robot_id": "arm-01", "worker_id": "w2", "event": "ACCEPTED", "simulated": False},
            )
        self.assertEqual(raised.exception.error_code, "WORKER_MISMATCH")
        rejected, _ = self.event(command, "REJECTED", reason_code="FRAME_UNSUPPORTED")
        self.assertEqual(rejected["state"], "REJECTED")
        self.assertFalse(rejected["blocks_new_commands"])

    def test_missing_ack_becomes_unknown_and_late_accept_is_refused(self) -> None:
        store = ArmCommandStore(self.task_store.session_factory, ack_timeout_s=0.05)
        store.create(command_request())
        command = store.claim_next("arm-01", "w1", 0)
        time.sleep(0.1)
        current = store.get(command["command_id"])
        self.assertEqual((current["state"], current["reason_code"]), ("UNKNOWN", "ACK_TIMEOUT"))
        with self.assertRaises(ArmCommandConflict) as raised:
            store.apply_worker_event(
                command["command_id"],
                {"event_id": "late", "robot_id": "arm-01", "worker_id": "w1", "event": "ACCEPTED", "simulated": True},
            )
        self.assertEqual(raised.exception.error_code, "INVALID_TRANSITION")
        with self.assertRaises(ArmCommandConflict):
            store.create(command_request())
        cleared = store.clear_unknown(command["command_id"], "팔이 움직이지 않은 것을 현장에서 확인")
        self.assertEqual(cleared["state"], "UNKNOWN")
        self.assertFalse(cleared["blocks_new_commands"])
        store.create(command_request())

    def test_expired_queue_entry_is_never_dispatched(self) -> None:
        command = self.store.create(command_request(valid_for_s=5))
        later = arm_commands._now() + arm_commands.timedelta(seconds=6)
        with mock.patch.object(arm_commands, "_now", return_value=later):
            self.assertIsNone(self.store.claim_next("arm-01", "w1", 0))
            self.assertEqual(self.store.get(command["command_id"])["state"], "EXPIRED")

    def test_cancel_before_and_after_accept(self) -> None:
        queued = self.store.create(command_request())
        self.assertEqual(self.store.cancel(queued["command_id"])["state"], "CANCELED")

        command = self.claimed()
        self.store.cancel(command["command_id"])
        with self.assertRaises(ArmCommandConflict) as raised:
            self.event(command, "ACCEPTED")
        self.assertEqual(raised.exception.error_code, "CANCELED_BEFORE_ACCEPT")
        self.assertEqual(self.store.get(command["command_id"])["state"], "CANCELED")

        command = self.claimed()
        self.event(command, "ACCEPTED")
        with self.assertRaises(ArmCommandConflict):
            self.event(command, "CANCELED", phase="GRASP_TARGET")
        requested = self.store.cancel(command["command_id"])
        self.assertTrue(requested["cancel_requested"])
        self.assertEqual(self.event(command, "CANCELED")[0]["state"], "CANCELED")

    def test_long_poll_wakes_when_a_command_is_created(self) -> None:
        results: list[dict | None] = []
        poller = threading.Thread(target=lambda: results.append(self.store.claim_next("arm-01", "w1", 5)))
        poller.start()
        time.sleep(0.2)
        self.assertEqual(self.store.workers()[0]["waiting_polls"], 1)
        started = time.monotonic()
        self.store.create(command_request())
        poller.join(5)
        self.assertLess(time.monotonic() - started, 2)
        self.assertEqual(results[0]["state"], "DISPATCHED")


def slot_request(**target: object) -> dict[str, object]:
    return command_request(target={"object_class": "strawberry", **target})


class ShelfSlotCommandTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.task_store = TaskStore(str(self.directory / "tasks.sqlite3"))
        self.addCleanup(self.task_store.close)
        self.slot_map = load_shelf_slot_map()  # the committed config/shelf-slots.json
        self.store = ArmCommandStore(self.task_store.session_factory, slot_map=self.slot_map)

    def test_committed_map_has_the_six_agreed_slots(self) -> None:
        self.assertEqual(
            sorted(self.slot_map.active_slots),
            ["slot_a_1_1", "slot_a_1_2", "slot_a_2_1", "slot_a_2_2", "slot_b_2_1", "slot_b_2_2"],
        )
        self.assertEqual(self.slot_map.active_slots["slot_a_2_1"]["label"], "왼쪽 선반 아래 단 좌")

    def test_slot_command_freezes_slot_and_map_version_without_coordinates(self) -> None:
        command = self.store.create(slot_request(pick_slot_id="slot_b_2_2"))
        target = command["target"]
        self.assertEqual(target["pick_slot_id"], "slot_b_2_2")
        self.assertEqual(target["slot_label"], "오른쪽 선반 아래 단 우")
        self.assertEqual(target["slot_map_version"], self.slot_map.version)
        self.assertIsNone(target["position"])
        self.assertIsNone(target["frame_id"])

    def test_unknown_inactive_or_mixed_slot_targets_are_rejected(self) -> None:
        for target in (
            {"pick_slot_id": "slot_z_9_9"},
            {"pick_slot_id": "slot_b_1_1"},  # right shelf upper tier exists in the model but is not used
            {"pick_slot_id": "slot_a_2_1", "frame_id": "omx_link0", "unit": "m", "position": {"x": 0, "y": 0, "z": 0}},
        ):
            with self.subTest(target=target), self.assertRaises(ArmCommandError):
                self.store.create(slot_request(**target))

    def test_move_to_slot_needs_a_slot_and_succeeds_only_on_a_reached_pose(self) -> None:
        with self.assertRaisesRegex(ArmCommandError, "pick_slot_id"):
            self.store.create(command_request(operation="MOVE_TO_SLOT"))
        command = self.store.create(
            command_request(operation="MOVE_TO_SLOT", target={"pick_slot_id": "slot_a_2_1"})
        )
        self.assertIsNone(command["target"]["object_class"])
        claimed = self.store.claim_next("arm-01", "w1", 0)
        base = {"robot_id": "arm-01", "worker_id": "w1", "simulated": False}
        self.store.apply_worker_event(claimed["command_id"], {**base, "event_id": "a", "event": "ACCEPTED"})
        with self.assertRaisesRegex(ArmCommandError, "pose_reached"):
            self.store.apply_worker_event(
                claimed["command_id"],
                {**base, "event_id": "s", "event": "SUCCEEDED", "grasp_confirmed": True, "place_confirmed": True},
            )
        done, _ = self.store.apply_worker_event(
            claimed["command_id"], {**base, "event_id": "s2", "event": "SUCCEEDED", "pose_reached": True}
        )
        self.assertEqual(done["state"], "SUCCEEDED")

    def test_return_home_names_no_target_and_succeeds_only_on_a_reached_pose(self) -> None:
        home = {"robot_id": "arm-01", "operation": "RETURN_HOME", "valid_for_s": 60, "source": {"kind": "OPERATOR_INPUT"}}
        for extra in ({"target": {"pick_slot_id": "slot_a_2_1"}}, {"place": {"slot_id": "CART_1"}}):
            with self.subTest(extra=extra), self.assertRaisesRegex(ArmCommandError, "RETURN_HOME"):
                self.store.create({**home, **extra})
        command = self.store.create(home)
        self.assertIsNone(command["place"])
        self.assertTrue(all(value is None for value in command["target"].values()))
        claimed = self.store.claim_next("arm-01", "w1", 0)
        base = {"robot_id": "arm-01", "worker_id": "w1", "simulated": False}
        self.store.apply_worker_event(claimed["command_id"], {**base, "event_id": "a", "event": "ACCEPTED"})
        with self.assertRaisesRegex(ArmCommandError, "pose_reached"):
            self.store.apply_worker_event(claimed["command_id"], {**base, "event_id": "s", "event": "SUCCEEDED"})
        done, _ = self.store.apply_worker_event(
            claimed["command_id"], {**base, "event_id": "s2", "event": "SUCCEEDED", "pose_reached": True}
        )
        self.assertEqual(done["state"], "SUCCEEDED")

    def test_move_to_position_carries_only_a_position_and_succeeds_on_a_reached_pose(self) -> None:
        position = {"frame_id": "omx_link0", "unit": "m", "position": {"x": 0.2, "y": 0.0, "z": 0.15}}
        move = {"robot_id": "arm-01", "operation": "MOVE_TO_POSITION", "valid_for_s": 60, "source": {"kind": "OPERATOR_INPUT"}}
        for bad in (
            {"target": {"pick_slot_id": "slot_a_2_1"}},
            {"target": position, "place": {"slot_id": "CART_1"}},
            {"target": {**position, "unit": "mm"}},
        ):
            with self.subTest(bad=bad), self.assertRaises(ArmCommandError):
                self.store.create({**move, **bad})
        command = self.store.create({**move, "target": position})
        self.assertIsNone(command["place"])
        self.assertIsNone(command["target"]["pick_slot_id"])
        self.assertEqual(command["target"]["position"], {"x": 0.2, "y": 0.0, "z": 0.15})
        claimed = self.store.claim_next("arm-01", "w1", 0)
        base = {"robot_id": "arm-01", "worker_id": "w1", "simulated": False}
        self.store.apply_worker_event(claimed["command_id"], {**base, "event_id": "a", "event": "ACCEPTED"})
        with self.assertRaisesRegex(ArmCommandError, "pose_reached"):
            self.store.apply_worker_event(claimed["command_id"], {**base, "event_id": "s", "event": "SUCCEEDED"})

    def test_camera_evidence_is_stored_only_when_it_matches_the_target_slot(self) -> None:
        evidence = {"camera_id": "shelf_camera", "slot_id": "slot_b_2_1", "class_name": "ball", "confidence": 0.62}
        request = command_request(operation="MOVE_TO_SLOT", target={"object_class": "ball", "pick_slot_id": "slot_a_2_1"})
        with self.assertRaisesRegex(ArmCommandError, "different slot"):
            self.store.create(request, camera_source=evidence)
        request["target"]["pick_slot_id"] = "slot_b_2_1"
        command = self.store.create(request, camera_source=evidence)
        self.assertEqual(command["source"]["kind"], "CAMERA")
        self.assertEqual(command["source"]["confidence"], 0.62)
        with self.assertRaises(ArmCommandError):  # clients cannot post camera provenance themselves
            self.store.create(command_request(source={"kind": "CAMERA"}))

    def test_slot_commands_need_a_slot_map(self) -> None:
        store = ArmCommandStore(self.task_store.session_factory)
        with self.assertRaisesRegex(ArmCommandError, "칸 정의"):
            store.create(slot_request(pick_slot_id="slot_a_2_1"))
        self.assertEqual(store.create(command_request())["target"]["pick_slot_id"], None)

    def test_slot_map_loader_rejects_duplicate_or_inactive_only_maps(self) -> None:
        document = json.loads(self.slot_map.path.read_text(encoding="utf-8"))
        duplicate = {**document, "slots": [document["slots"][0], document["slots"][0]]}
        inactive = {**document, "slots": [{**slot, "active": False} for slot in document["slots"]]}
        for name, broken in (("duplicate", duplicate), ("inactive", inactive)):
            path = self.directory / f"{name}.json"
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.subTest(name=name), self.assertRaises(ShelfSlotMapError):
                load_shelf_slot_map(path)


class PolicyCommandTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.task_store = TaskStore(str(Path(directory.name) / "tasks.sqlite3"))
        self.addCleanup(self.task_store.close)
        self.policies = load_act_policies()  # the committed config/omx-act-policies.json
        self.store = ArmCommandStore(
            self.task_store.session_factory, slot_map=load_shelf_slot_map(), act_policies=self.policies
        )

    def request(self, **target: object) -> dict[str, object]:
        return {
            "robot_id": "arm-01",
            "operation": "PICK_WITH_POLICY",
            "valid_for_s": 60,
            "target": target,
            "place": {"slot_id": "CART_1"},
            "source": {"kind": "OPERATOR_INPUT"},
        }

    def test_korean_name_chooses_the_policy_and_freezes_its_model_and_registry_version(self) -> None:
        command = self.store.create(self.request(object_class="당근", pick_slot_id="slot_a_2_1"))
        target = command["target"]
        self.assertEqual(target["object_class"], "carrot")
        self.assertEqual(target["pick_slot_id"], "slot_a_2_1")
        self.assertIsNone(target["position"])
        self.assertEqual(
            target["policy"],
            {"policy_id": "carrot_act_v2", "label": "당근", "model": "JJinJJin2/omx_act_policy_v2",
             "registry_version": self.policies.version},
        )

    def test_unknown_object_coordinates_or_missing_registry_are_rejected(self) -> None:
        with self.assertRaisesRegex(ArmCommandError, "학습된 정책이 없습니다"):
            self.store.create(self.request(object_class="banana"))
        with self.assertRaisesRegex(ArmCommandError, "only object_class"):
            self.store.create(self.request(object_class="carrot", frame_id="omx_link0"))
        without = ArmCommandStore(self.task_store.session_factory)
        with self.assertRaisesRegex(ArmCommandError, "omx-act-policies"):
            without.create(self.request(object_class="carrot"))

    def test_home_after_the_policy_is_not_success_without_grasp_and_place(self) -> None:
        command = self.store.create(self.request(object_class="물티슈"))
        self.assertEqual(command["target"]["policy"]["policy_id"], "tissue_act_v1")
        self.assertIsNone(command["target"]["pick_slot_id"])
        claimed = self.store.claim_next("arm-01", "w1", 0)
        base = {"robot_id": "arm-01", "worker_id": "w1", "simulated": False}
        self.store.apply_worker_event(claimed["command_id"], {**base, "event_id": "a", "event": "ACCEPTED"})
        with self.assertRaisesRegex(ArmCommandError, "grasp_confirmed"):
            self.store.apply_worker_event(
                claimed["command_id"], {**base, "event_id": "s", "event": "SUCCEEDED", "pose_reached": True}
            )
        done, _ = self.store.apply_worker_event(
            claimed["command_id"],
            {**base, "event_id": "u", "event": "RESULT_UNKNOWN", "reason_code": "PLACE_NOT_VERIFIED", "pose_reached": True},
        )
        self.assertEqual((done["state"], done["reason_code"]), ("UNKNOWN", "PLACE_NOT_VERIFIED"))


class PolicyCameraHandoverTests(PolicyCommandTests):
    """The control station closes the policy's cameras only while a PICK_WITH_POLICY command is with the worker."""

    def app(self) -> object:
        manager = mock.Mock()
        manager.lend_devices.return_value = ["head_camera"]
        return mock.Mock(
            arm_commands=self.store,
            camera_manager=manager,
            policy_camera_devices=["/dev/overhead"],
            policy_camera_lock=threading.Lock(),
            policy_cameras_lent=False,
        )

    def test_cameras_close_on_dispatch_and_reopen_when_the_command_ends(self) -> None:
        app = self.app()
        self.store.create(self.request(object_class="당근"))
        arm_api.sync_policy_cameras(app)  # queued only: the worker has not taken it
        app.camera_manager.lend_devices.assert_not_called()
        claimed = self.store.claim_next("arm-01", "w1", 0)
        arm_api.sync_policy_cameras(app)
        arm_api.sync_policy_cameras(app)  # repeated polls do not lend twice
        app.camera_manager.lend_devices.assert_called_once()
        self.assertEqual(app.camera_manager.lend_devices.call_args.args[0], ["/dev/overhead"])
        base = {"robot_id": "arm-01", "worker_id": "w1", "simulated": False}
        self.store.apply_worker_event(claimed["command_id"], {**base, "event_id": "r", "event": "REJECTED", "reason_code": "X"})
        arm_api.sync_policy_cameras(app)
        app.camera_manager.take_back_devices.assert_called_once()
        self.assertFalse(app.policy_cameras_lent)

    def test_other_commands_never_close_cameras(self) -> None:
        app = self.app()
        self.store.create(command_request(operation="MOVE_TO_SLOT", target={"pick_slot_id": "slot_a_2_1"}))
        self.store.claim_next("arm-01", "w1", 0)
        arm_api.sync_policy_cameras(app)
        app.camera_manager.lend_devices.assert_not_called()


class ArmCommandHttpTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        task_store = TaskStore(str(Path(directory.name) / "tasks.sqlite3"))
        self.addCleanup(task_store.close)
        server = ControlStationServer(
            ("127.0.0.1", 0), partial(ControlStationHandler, directory=str(WEB_DIR))
        )
        server.arm_commands = ArmCommandStore(task_store.session_factory, slot_map=load_shelf_slot_map())
        self.server = server
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.base = f"http://127.0.0.1:{server.server_address[1]}"

    def call(self, method: str, path: str, body: object = None, headers: dict | None = None) -> tuple[int, dict]:
        data = json.dumps(body).encode() if body is not None else None
        request = Request(
            self.base + path,
            data=data,
            method=method,
            headers={**({"Content-Type": "application/json"} if data else {}), **(headers or {})},
        )
        try:
            with urlopen(request, timeout=10) as response:
                raw = response.read()
                return response.status, json.loads(raw) if raw else {}
        except HTTPError as error:
            return error.code, json.loads(error.read() or b"{}")

    def test_worker_round_trip_over_http(self) -> None:
        self.assertEqual(self.call("GET", "/api/arm/arm-01/commands/next?worker_id=w1&wait_s=0")[0], 204)
        status, created = self.call("POST", "/api/arm/commands", command_request())
        self.assertEqual(status, 201)
        command_id = created["command"]["command_id"]
        status, claimed = self.call("GET", "/api/arm/arm-01/commands/next?worker_id=w1&wait_s=0")
        self.assertEqual((status, claimed["command"]["command_id"]), (200, command_id))
        event = {"event_id": "a1", "robot_id": "arm-01", "worker_id": "w1", "event": "ACCEPTED", "simulated": True}
        status, accepted = self.call("POST", f"/api/arm/commands/{command_id}/events", event)
        self.assertEqual((status, accepted["applied"]), (200, True))
        status, conflict = self.call("POST", "/api/arm/commands", command_request())
        self.assertEqual((status, conflict["error_code"]), (409, "ARM_COMMAND_ACTIVE"))
        status, listing = self.call("GET", "/api/arm/commands")
        self.assertEqual(listing["commands"][0]["state"], "ACCEPTED")
        self.assertEqual(listing["workers"][0]["worker_id"], "w1")

    def test_shelf_slots_are_served_and_usable_over_http(self) -> None:
        status, body = self.call("GET", "/api/arm/shelf-slots")
        self.assertEqual(status, 200)
        active = [slot["slot_id"] for slot in body["slot_map"]["slots"] if slot["active"]]
        self.assertEqual(len(active), 6)
        status, created = self.call("POST", "/api/arm/commands", slot_request(pick_slot_id="slot_a_1_1"))
        self.assertEqual((status, created["command"]["target"]["slot_map_version"]), (201, body["slot_map_version"]))

    def test_policy_list_and_camera_policy_command_without_a_policy(self) -> None:
        status, body = self.call("GET", "/api/arm/policies")
        self.assertEqual((status, body["error_code"]), (404, "ACT_POLICIES_NOT_CONFIGURED"))
        self.server.arm_commands.act_policies = load_act_policies()
        status, body = self.call("GET", "/api/arm/policies")
        self.assertEqual(status, 200)
        self.assertEqual([policy["label"] for policy in body["policies"]], ["당근", "물티슈"])
        status, body = self.call(
            "POST", "/api/arm/commands/from-camera", {"object_class": "banana", "operation": "PICK_WITH_POLICY"}
        )
        self.assertEqual((status, body["error_code"]), (409, "POLICY_NOT_FOUND"))
        status, body = self.call("POST", "/api/arm/commands/from-camera", {"object_class": "carrot", "operation": "GRAB"})
        self.assertEqual((status, body["error_code"]), (400, "ARM_COMMAND_INVALID"))

    def test_cross_origin_browser_request_is_rejected(self) -> None:
        status, body = self.call(
            "POST", "/api/arm/commands", command_request(), headers={"Origin": "http://evil.example"}
        )
        self.assertEqual((status, body["error_code"]), (403, "CROSS_ORIGIN_REQUEST_REJECTED"))


if __name__ == "__main__":
    unittest.main()
