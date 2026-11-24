"""Planning figures for splitting orders over Pinkys and timing the two arms (config/fleet-planning.json).

These are estimates, not measurements: the Pinky tray capacity in size units, each object's size (1 unless
listed), and how long one pick spends at the shelf and over the Pinky. They shape the preview and the split; they
never move a device or decide that a motion has finished.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from control_station.assignment import ArmPick, PickTiming
from control_station.json_files import load_json_file

DEFAULT_PLANNING_PATH = Path(__file__).resolve().parents[2] / "config" / "fleet-planning.json"
MAX_PLANNING_BYTES = 16 * 1024


class FleetPlanningError(ValueError):
    """The planning file is missing or does not match schema version 1."""


@dataclass(frozen=True)
class FleetPlanning:
    pinky_capacity: int
    shelf_s: float
    place_s: float
    unit_sizes: Mapping[str, int] = field(default_factory=dict)
    # Overlap the two arms only when a pick can be split where the arm enters the space above the Pinky. The ACT
    # policies run grasp -> place -> rest in one go, so the arms take turns whole picks at a time.
    arms_overlap: bool = False

    def size_of(self, object_class: str) -> int:
        return self.unit_sizes.get(object_class, 1)

    def timing(self, pick: ArmPick) -> PickTiming:
        return PickTiming(shelf_s=self.shelf_s, place_s=self.place_s)


def _positive_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value <= 600:
        raise FleetPlanningError(f"{name} must be a number of seconds above 0")
    return float(value)


def load_fleet_planning(path: str | Path | None = None) -> FleetPlanning:
    planning_path = Path(path or os.environ.get("CONTROL_STATION_FLEET_PLANNING") or DEFAULT_PLANNING_PATH)
    _, document = load_json_file(planning_path, MAX_PLANNING_BYTES, "핑키·팔 계획값 파일", FleetPlanningError)
    expected = {"schema_version", "notes", "pinky_capacity", "unit_sizes", "arms_overlap", "pick_timing_s"}
    if not isinstance(document, dict) or set(document) - expected or document.get("schema_version") != 1:
        raise FleetPlanningError("핑키·팔 계획값 파일은 schema_version 1과 정해진 항목만 가져야 합니다")
    capacity = document.get("pinky_capacity")
    if isinstance(capacity, bool) or not isinstance(capacity, int) or not 1 <= capacity <= 50:
        raise FleetPlanningError("pinky_capacity must be an integer from 1 to 50")
    sizes = document.get("unit_sizes", {})
    if not isinstance(sizes, dict) or not all(
        isinstance(name, str) and isinstance(size, int) and not isinstance(size, bool) and 1 <= size <= capacity
        for name, size in sizes.items()
    ):
        raise FleetPlanningError("unit_sizes must map object classes to integers from 1 to pinky_capacity")
    timing = document.get("pick_timing_s")
    if not isinstance(timing, dict) or set(timing) != {"shelf", "place"}:
        raise FleetPlanningError("pick_timing_s must have exactly shelf and place")
    overlap = document.get("arms_overlap", False)
    if not isinstance(overlap, bool):
        raise FleetPlanningError("arms_overlap must be true or false")
    return FleetPlanning(capacity, _positive_number(timing["shelf"], "pick_timing_s.shelf"),
                         _positive_number(timing["place"], "pick_timing_s.place"), dict(sizes), overlap)
