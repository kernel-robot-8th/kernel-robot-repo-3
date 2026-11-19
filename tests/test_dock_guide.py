"""The relay of the ceiling camera's live offset to the Pinky driving in (control_station/dock_guide.py)."""
from __future__ import annotations

import unittest

from control_station.dock_guide import DockGuideRelay, guide_from


def live(state: str = "MOVING", frame: int = 1, **offset: float) -> dict:
    return {"state": state, "frame_sequence": frame,
            "offset": {"forward_mm": -120.0, "left_mm": 18.0, "dyaw_deg": 2.0, **offset} if state != "NOT_IN_VIEW" else None}


class DockGuideRelayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now, self.live, self.phase, self.sent, self.accept = 0.0, live(), "redocking", [], True

        def send(pinky_id: str, guide: dict) -> bool:
            self.sent.append((pinky_id, guide))
            return self.accept

        self.relay = DockGuideRelay(lambda: self.live, "12", lambda pinky_id: self.phase, send, clock=lambda: self.now)

    def test_each_new_frame_of_a_tracked_marker_goes_out_once(self) -> None:
        self.assertEqual(self.relay.tick(), "SENT")
        self.assertEqual(self.sent, [("12", {"forward_mm": -120.0, "left_mm": 18.0, "dyaw_deg": 2.0})])
        self.assertEqual(self.relay.tick(), "SAME_FRAME")  # a stalled camera must not look fresh
        self.live = live("OFF_POSITION", frame=2, forward_mm=-3.0)
        self.assertEqual(self.relay.tick(), "SENT")
        self.assertEqual(len(self.sent), 2)

    def test_nothing_goes_out_for_a_hidden_marker_a_parked_pinky_or_an_offset_without_pinky_axes(self) -> None:
        self.live = live("NOT_IN_VIEW")
        self.assertEqual(self.relay.tick(), "NOT_TRACKED")
        self.live, self.phase = live(frame=3), "parked"
        self.assertEqual(self.relay.tick(), "PARKED")
        self.assertIsNone(guide_from({"state": "MOVING", "offset": {"dx_mm": 1.0, "dy_mm": 2.0, "dyaw_deg": 0.1}}))
        self.assertIsNone(guide_from({"state": "VIEW_UNVERIFIED", "offset": {"forward_mm": 1.0, "left_mm": 2.0, "dyaw_deg": 0.1}}))
        self.assertEqual(self.sent, [])

    def test_a_refusal_or_a_silent_dashboard_quiets_it_for_a_while(self) -> None:
        self.accept = False
        self.assertEqual(self.relay.tick(), "REFUSED")
        self.live = live(frame=2)
        self.assertEqual(self.relay.tick(), "QUIET")
        self.now, self.accept = 3.5, True
        self.assertEqual(self.relay.tick(), "SENT")
        self.relay.phase = lambda pinky_id: (_ for _ in ()).throw(OSError("dashboard away"))
        self.live = live(frame=3)
        self.assertEqual(self.relay.tick(), "REFUSED")
        self.assertEqual((self.relay.sent, self.relay.failed), (1, 2))


if __name__ == "__main__":
    unittest.main()
