"""Receiving zones: where a customer collects an order (config/receiving-zones.json).

A zone is the last waypoint of the Pinky's route for that order. The Pinky fleet already stops at the end of a
route (`course_complete`) and waits for `mission-complete` before returning to its own parking spot, so a zone needs
no Pinky code: only a waypoint on the Pinky map and its place in the route.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path

from control_station.json_files import load_json_file

DEFAULT_ZONES_PATH = Path(__file__).resolve().parents[2] / "config" / "receiving-zones.json"
MAX_ZONES_BYTES = 16 * 1024
_ZONE_ID = re.compile(r"^[a-z0-9_]{1,32}$")
_WAYPOINT = re.compile(r"^[0-9A-Za-z가-힣_-]{1,20}$")


class ReceivingZoneError(ValueError):
    """The zone file is missing or does not match schema version 1."""


@dataclass(frozen=True)
class ReceivingZone:
    zone_id: str
    label: str
    route_labels: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {"zone_id": self.zone_id, "label": self.label, "route_labels": list(self.route_labels)}


def _zone(raw: object) -> ReceivingZone:
    if not isinstance(raw, dict) or set(raw) != {"zone_id", "label", "route_labels"}:
        raise ReceivingZoneError("each zone must have exactly zone_id, label and route_labels")
    zone_id, label, route = raw["zone_id"], raw["label"], raw["route_labels"]
    if not isinstance(zone_id, str) or not _ZONE_ID.fullmatch(zone_id):
        raise ReceivingZoneError("zone_id must be 1 to 32 lowercase letters, digits or _")
    if not isinstance(label, str) or not 1 <= len(label.strip()) <= 40:
        raise ReceivingZoneError(f"{zone_id}.label must contain 1 to 40 characters")
    if not isinstance(route, list) or len(route) < 2 or not all(isinstance(w, str) and _WAYPOINT.fullmatch(w) for w in route):
        raise ReceivingZoneError(f"{zone_id}.route_labels must list at least two waypoint names")
    return ReceivingZone(zone_id, label.strip(), tuple(route))


def load_receiving_zones(path: str | Path | None = None) -> dict[str, ReceivingZone]:
    """Zone id -> zone, in file order."""
    zones_path = Path(path or os.environ.get("CONTROL_STATION_RECEIVING_ZONES") or DEFAULT_ZONES_PATH)
    _, document = load_json_file(zones_path, MAX_ZONES_BYTES, "수령 구역 파일", ReceivingZoneError)
    if not isinstance(document, dict) or document.get("schema_version") != 1 or isinstance(document.get("schema_version"), bool):
        raise ReceivingZoneError("수령 구역 파일의 schema_version은 1이어야 합니다")
    raw_zones = document.get("zones")
    if not isinstance(raw_zones, list) or not raw_zones:
        raise ReceivingZoneError("zones must be a non-empty list")
    zones: dict[str, ReceivingZone] = {}
    for raw in raw_zones:
        zone = _zone(raw)
        if zone.zone_id in zones:
            raise ReceivingZoneError(f"duplicate zone_id: {zone.zone_id}")
        zones[zone.zone_id] = zone
    return zones
