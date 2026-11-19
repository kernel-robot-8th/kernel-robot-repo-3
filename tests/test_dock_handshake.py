"""The camera <-> Pinky exchange at the loading spot (control_station/dock_handshake.py), without devices."""
from __future__ import annotations

import unittest

from control_station.dock_handshake import COMPLETE, OPERATOR, REDOCK, WAIT, DockHandshake, DockSupervisor

OFF = {"status": "MISALIGNED", "offset": {"distance_mm": 54.0, "forward_mm": 30.0, "left_mm": -45.0, "dyaw_deg": 6.0}}
OK = {"status": "ARRIVED", "offset": {"distance_mm": 6.0, "forward_mm": 4.0, "left_mm": -4.0, "dyaw_deg": 0.5}}
HIDDEN = {"status": "NOT_IN_VIEW", "reason": "marker not visible"}


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


class DockHandshakeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = Clock()
        self.handshake = DockHandshake(clock=self.clock)

    def steps(self, phase: str, observation: dict | None, count: int):
        return [self.handshake.step(phase, observation) for _ in range(count)][-1]

    def test_in_place_has_to_be_seen_twice_in_a_row(self) -> None:
        first = self.handshake.step("docking_manual", OK)
        self.assertEqual((first.action, first.reason), (WAIT, "SETTLING"))
        self.handshake.step("docking_manual", HIDDEN)  # one lucky frame does not count
        self.assertEqual(self.handshake.step("docking_manual", OK).action, WAIT)
        self.assertEqual(self.handshake.step("docking_manual", OK).action, COMPLETE)

    def test_steadily_off_is_redocked_with_the_cameras_offset_and_not_judged_again_until_it_is_back(self) -> None:
        self.assertEqual(self.steps("docking_manual", OFF, 2).action, WAIT)
        asked = self.handshake.step("docking_manual", OFF)
        self.assertEqual((asked.action, asked.attempt), (REDOCK, 1))
        self.assertEqual(asked.guide, {"forward_mm": 30.0, "left_mm": -45.0, "dyaw_deg": 6.0})
        self.handshake.redock_sent(True, 54.0)
        # The Pinky has not moved yet: the same picture must not ask for a second redock.
        self.assertEqual(self.steps("docking_manual", OFF, 5).reason, "REDOCK_PENDING")
        self.handshake.step("redocking", None)
        back = self.handshake.step("docking_manual", OK)
        self.assertEqual(back.result, {"attempt": 1, "guided": True, "before_mm": 54.0, "after_mm": 6.0, "status": "ARRIVED"})
        done = self.handshake.step("docking_manual", OK)
        self.assertEqual((done.action, done.result), (COMPLETE, None))  # the result is reported once

    def test_a_redock_the_pinky_never_acted_on_is_reported_and_judging_goes_on(self) -> None:
        self.steps("docking_manual", OFF, 3)
        self.handshake.redock_sent(False, 54.0)
        self.clock.now += 16.0
        after = self.handshake.step("docking_manual", OFF)
        self.assertTrue(after.not_applied)
        self.assertEqual((after.action, after.reason), (WAIT, "MISALIGNED"))

    def test_after_the_last_redock_a_person_decides(self) -> None:
        for attempt in (1, 2, 3, 4, 5):
            asked = self.steps("docking_manual", OFF, 3)
            self.assertEqual((asked.action, asked.attempt), (REDOCK, attempt))
            self.handshake.redock_sent(True, 54.0)
            self.handshake.step("redocking", None)
        used = self.steps("docking_manual", OFF, 3)
        self.assertEqual((used.action, used.reason), (OPERATOR, "REDOCKS_USED"))
        self.assertEqual(self.handshake.step("docking_manual", OFF).action, OPERATOR)  # keeps asking, sends nothing
        self.assertEqual(self.steps("docking_manual", OK, 2).action, COMPLETE)  # straightened by hand: the camera takes over

    def test_no_registration_or_a_blind_camera_goes_to_a_person(self) -> None:
        self.assertEqual(self.handshake.step("docking_manual", {"status": "NOT_REGISTERED"}).action, OPERATOR)
        moved = {"status": "VIEW_UNVERIFIED", "reason": "fixed markers moved"}  # not something a redock could mend
        blind = self.handshake.step("docking_manual", moved)
        self.assertEqual((blind.action, blind.reason), (WAIT, "CAMERA_BLIND"))
        self.clock.now += 19.0
        self.assertEqual(self.handshake.step("docking_manual", None).action, WAIT)  # a failed check is blind too
        self.clock.now += 2.0
        self.assertEqual(self.handshake.step("docking_manual", moved).action, OPERATOR)
        self.assertEqual(self.steps("docking_manual", OK, 2).action, COMPLETE)

    def test_a_pinky_the_working_camera_does_not_see_is_called_in_again_instead_of_waiting_for_a_person(self) -> None:
        gone = {"status": "NOT_FOUND", "reason": "marker 21 not in view"}
        self.assertEqual(self.handshake.step("docking_manual", gone).action, WAIT)
        self.clock.now += 21.0
        called = self.handshake.step("docking_manual", gone)
        self.assertEqual((called.action, called.reason, called.guide, called.attempt), (REDOCK, "NOT_SEEN", None, 1))
        self.handshake.redock_sent(False, None)
        self.handshake.step("redocking", None)
        self.assertEqual(self.handshake.step("docking_manual", gone).action, WAIT)  # the clock starts again
        self.assertEqual(self.steps("docking_manual", OK, 2).action, COMPLETE)
        # A view that moved is not the Pinky's fault: calling it in again changes nothing, a person looks.
        moved = {"status": "VIEW_UNVERIFIED", "reason": "fixed markers moved 20.4 px from registration"}
        other = DockHandshake(clock=self.clock)
        other.step("docking_manual", moved)
        self.clock.now += 21.0
        self.assertEqual(other.step("docking_manual", moved).action, OPERATOR)

    def test_an_offset_without_pinky_axes_redocks_without_a_guide(self) -> None:
        bare = {"status": "MISALIGNED", "offset": {"distance_mm": 50.0, "dyaw_deg": 2.0}}
        self.assertIsNone(self.steps("docking_manual", bare, 3).guide)


class DockSupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.phase, self.seen, self.sent, self.in_order = "docking_manual", OFF, [], False

        def signal(pinky_id: str, action: str, offset: dict | None) -> dict:
            self.sent.append((pinky_id, action, offset))
            if action == "redock":
                self.phase = "redocking"
            return {"accepted": True, "guided": offset is not None}

        self.supervisor = DockSupervisor(lambda pinky_id: {"phase": self.phase, "mission_token": "t"},
                                         lambda pinky_id: self.seen, signal, in_order=lambda pinky_id: self.in_order)

    def test_auto_redocks_with_the_offset_then_completes_once_and_keeps_the_log(self) -> None:
        for _ in range(3):
            answer = self.supervisor.step("12", True)
        self.assertEqual((answer["sent"], self.sent), ("redock", [("12", "redock", {"forward_mm": 30.0, "left_mm": -45.0, "dyaw_deg": 6.0})]))
        self.supervisor.step("12", True)  # redocking: nothing is judged
        self.phase, self.seen = "docking_manual", OK
        self.supervisor.step("12", True)
        answer = self.supervisor.step("12", True)
        self.assertEqual((answer["sent"], self.sent[-1][1]), ("dock_complete", "dock_complete"))
        self.assertEqual([entry["event"] for entry in answer["log"]], ["REDOCK", "REDOCK_RESULT", "COMPLETE"])
        self.assertIn("54 mm → 6 mm", answer["log"][1]["message"])
        self.assertIsNone(self.supervisor.step("12", True)["sent"])  # still docking_manual for a moment: not sent twice
        self.assertEqual(len(self.sent), 2)

    def test_without_auto_it_only_judges_and_keeps_saying_what_to_press(self) -> None:
        answers = [self.supervisor.step("12", False) for _ in range(5)]
        self.assertEqual([answer["decision"]["action"] for answer in answers], [WAIT, WAIT, REDOCK, REDOCK, REDOCK])
        self.assertEqual(self.sent, [])

    def test_a_pinky_an_order_handles_or_one_that_left_is_left_alone(self) -> None:
        self.in_order = True
        self.assertEqual(self.supervisor.step("12", True)["in_order"], True)
        self.in_order, self.phase = False, "driving"
        self.assertEqual(self.supervisor.step("12", True), {"pinky_id": "12", "active": False, "phase": "driving"})
        self.assertEqual(self.sent, [])


if __name__ == "__main__":
    unittest.main()


class DockRepeatTests(unittest.TestCase):
    """The repeat test of the entry: confirmed entries are collected, the Pinky goes out and in again in between."""

    def test_each_confirmed_entry_is_recorded_and_the_pinky_is_sent_in_again_until_enough_are_collected(self) -> None:
        sent, recorded, state = [], [], {"phase": "docking_manual", "seen": OK}
        def signal(pinky_id, action, offset):
            sent.append((action, offset))
            return {"guided": bool(offset)}
        supervisor = DockSupervisor(lambda pinky_id: {"phase": state["phase"], "mission_token": "t"},
                                    lambda pinky_id: state["seen"], signal, record=recorded.append)
        def entry():
            answer = None
            for _ in range(8):
                answer = supervisor.step("12", True, 2)
                if answer["sent"]:
                    break
            return answer
        first = entry()
        self.assertEqual((first["sent"], first["repeat"]["successes"], first["repeat"]["target"]), ("redock", 1, 2))
        self.assertEqual(sent, [("redock", None)])  # out and in again, no correction: it stood right
        state["phase"] = "redocking"; supervisor.step("12", True, 2)
        state.update(phase="docking_manual", seen=OFF)  # this time it came in off the spot
        missed = entry()
        self.assertEqual(missed["sent"], "redock")
        self.assertEqual(sent[-1][1], {"forward_mm": 30.0, "left_mm": -45.0, "dyaw_deg": 6.0})  # corrected as always
        state["phase"] = "redocking"; supervisor.step("12", True, 2)
        state.update(phase="docking_manual", seen=OK)
        last = entry()
        self.assertEqual((last["sent"], last["repeat"]["successes"], last["repeat"]["trials"]), ("dock_complete", 2, 3))
        self.assertEqual([trial["outcome"] for trial in recorded], ["SUCCESS", "MISALIGNED", "SUCCESS"])
        self.assertEqual(recorded[0]["left_mm"], -4.0)
