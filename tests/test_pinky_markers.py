"""Each Pinky's marker and its own registration of the loading spot."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from control_station.pinky_markers import PinkyMarkerError, arrival_for, load_pinky_markers


class PinkyMarkersTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.spots = self.root / "spots"
        self.spots.mkdir()
        (self.spots / "dock_p1.json").write_text(json.dumps({"spot_id": "dock_p1", "marker_id": 21}))
        (self.spots / "dock_p2.json").write_text(json.dumps({"spot_id": "dock_p2", "marker_id": 99}))

    def write(self, pinkies: dict) -> Path:
        path = self.root / "markers.json"
        path.write_text(json.dumps({"schema_version": 1, "pinkies": pinkies}))
        return path

    def test_registered_missing_and_mismatched(self) -> None:
        markers = load_pinky_markers(self.write({
            "10": {"marker_id": 20, "spot_id": "dock_p0"}, "11": {"marker_id": 21, "spot_id": "dock_p1"},
            "12": {"marker_id": 22, "spot_id": "dock_p2"}}), self.spots)
        self.assertEqual([markers[p].registered for p in ("10", "11", "12")], [False, True, False])
        self.assertEqual(markers["10"].problem, "정차 위치가 등록되지 않았습니다 (dock_p0)")
        self.assertEqual(markers["12"].problem, "dock_p2는 마커 99번으로 등록돼 있습니다")

    def test_one_marker_cannot_belong_to_two_pinkys(self) -> None:
        with self.assertRaises(PinkyMarkerError):
            load_pinky_markers(self.write({"10": {"marker_id": 21, "spot_id": "a"}, "11": {"marker_id": 21, "spot_id": "b"}}),
                               self.spots)

    def test_arrival_uses_the_assigned_pinkys_own_spot(self) -> None:
        markers = load_pinky_markers(self.write({
            "10": {"marker_id": 20, "spot_id": "dock_p0"}, "11": {"marker_id": 21, "spot_id": "dock_p1"}}), self.spots)
        asked = []

        def check(spot_id: str) -> dict:
            asked.append(spot_id)
            return {"status": "ARRIVED", "spot_id": spot_id}

        self.assertEqual(arrival_for(markers, check, "11")["status"], "ARRIVED")
        self.assertEqual(asked, ["dock_p1"])
        unregistered = arrival_for(markers, check, "10")
        self.assertEqual((unregistered["status"], unregistered["reason"]),
                         ("NOT_REGISTERED", "핑키 P0(마커 20번): 정차 위치가 등록되지 않았습니다 (dock_p0)"))
        self.assertEqual(asked, ["dock_p1"])  # the node is not asked about a spot that does not exist


if __name__ == "__main__":
    unittest.main()
