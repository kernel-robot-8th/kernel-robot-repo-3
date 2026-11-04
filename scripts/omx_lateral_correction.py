"""Shoulder-pan correction toward an object's left-right position on a shelf slot (no hardware imports)."""

from __future__ import annotations

import json
import math
from pathlib import Path


TICKS_PER_TURN = 4096


def load_base(path: Path) -> dict:
    """OMX shoulder-pan pivot in the shelf_front frame, measured on site, and the pan direction sign."""
    base = json.loads(path.read_text(encoding="utf-8"))
    pivot = base.get("pan_pivot_shelf_m") or {}
    if not all(isinstance(pivot.get(axis), (int, float)) for axis in ("x", "y")):
        raise SystemExit(f"{path}: pan_pivot_shelf_m.x and .y must be measured numbers")
    if base.get("pan_sign") not in (1, -1):
        raise SystemExit(f"{path}: pan_sign must be 1 or -1 (checked on the real arm)")
    return base


def pan_correction(base: dict, marker: dict, lateral_offset_m: float) -> tuple[int, float]:
    """Shoulder-pan change that turns the arm from the slot centre line to an object `lateral_offset_m` to its side.

    The taught slot pose is assumed to aim at the slot marker. The object is taken at the marker's depth,
    because the front camera does not measure depth well. Returns (ticks, degrees); positive degrees turn
    toward +y (the viewer's left), and `pan_sign` maps that onto the motor's tick direction.
    """
    pivot = base["pan_pivot_shelf_m"]
    reach = marker["x"] - pivot["x"]
    aim = math.atan2(marker["y"] - pivot["y"], reach)
    wanted = math.atan2(marker["y"] + lateral_offset_m - pivot["y"], reach)
    delta = wanted - aim
    return round(base["pan_sign"] * delta * TICKS_PER_TURN / (2 * math.pi)), math.degrees(delta)
