"""Which ArUco marker each Pinky carries, and the parking-spot registration made with it (config/pinky-markers.json).

The ceiling camera judges arrival by the marker on top of the Pinky. Markers sit a little differently on each Pinky,
so each Pinky has its own registration of the same loading spot (.control-station/parking-spots/<spot_id>.json,
made with that Pinky standing at the right place). The order flow asks the parking node about the assigned Pinky's
own spot, so another Pinky standing there is never taken for it.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from control_station.json_files import load_json_file
from control_station.store import PINKY_IDS, PINKY_NAMES

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MARKERS_PATH = ROOT / "config" / "pinky-markers.json"
SPOTS_DIR = ROOT / ".control-station" / "parking-spots"


class PinkyMarkerError(ValueError):
    """The marker file is missing or does not match schema version 1."""


@dataclass(frozen=True)
class PinkyMarker:
    pinky_id: str
    marker_id: int
    spot_id: str
    registered: bool  # the spot file exists and was made with this marker
    problem: str | None  # why the spot cannot be used yet

    def as_dict(self) -> dict[str, Any]:
        return {"marker_id": self.marker_id, "spot_id": self.spot_id, "registered": self.registered,
                "problem": self.problem}


def _registration(spot_id: str, marker_id: int, spots_dir: Path) -> str | None:
    path = spots_dir / f"{spot_id}.json"
    try:
        spot = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return f"정차 위치가 등록되지 않았습니다 ({spot_id})"
    except (OSError, ValueError) as exc:
        return f"정차 위치 파일을 읽지 못했습니다 ({spot_id}): {exc}"
    if spot.get("marker_id") != marker_id:
        return f"{spot_id}는 마커 {spot.get('marker_id')}번으로 등록돼 있습니다"
    return None


def load_pinky_markers(path: str | Path | None = None, spots_dir: Path = SPOTS_DIR) -> dict[str, PinkyMarker]:
    markers_path = Path(path or os.environ.get("CONTROL_STATION_PINKY_MARKERS") or DEFAULT_MARKERS_PATH)
    _, document = load_json_file(markers_path, 16 * 1024, "핑키 마커 파일", PinkyMarkerError)
    if not isinstance(document, dict) or document.get("schema_version") != 1 or not isinstance(document.get("pinkies"), dict):
        raise PinkyMarkerError("핑키 마커 파일은 schema_version 1과 pinkies를 가져야 합니다")
    markers: dict[str, PinkyMarker] = {}
    used: dict[int, str] = {}
    for pinky_id, entry in document["pinkies"].items():
        if pinky_id not in PINKY_IDS or not isinstance(entry, dict):
            raise PinkyMarkerError(f"알 수 없는 핑키 {pinky_id}")
        marker_id, spot_id = entry.get("marker_id"), entry.get("spot_id")
        if isinstance(marker_id, bool) or not isinstance(marker_id, int) or not 0 <= marker_id < 50:
            raise PinkyMarkerError(f"{pinky_id}.marker_id는 DICT_4X4_50의 0~49여야 합니다")
        if not isinstance(spot_id, str) or not spot_id:
            raise PinkyMarkerError(f"{pinky_id}.spot_id가 필요합니다")
        if marker_id in used:
            raise PinkyMarkerError(f"마커 {marker_id}번을 핑키 {used[marker_id]}와 {pinky_id}가 같이 쓸 수 없습니다")
        used[marker_id] = pinky_id
        problem = _registration(spot_id, marker_id, spots_dir)
        markers[pinky_id] = PinkyMarker(pinky_id, marker_id, spot_id, problem is None, problem)
    return markers


def arrival_for(markers: dict[str, PinkyMarker], check: Any, pinky_id: str) -> dict[str, Any]:
    """Judge the assigned Pinky at its own registration; an unregistered one is a position problem to fix, which
    the order flow shows as an alert while it keeps waiting."""
    marker = markers.get(pinky_id)
    name = PINKY_NAMES.get(pinky_id, pinky_id)
    if marker is None:
        return {"status": "NOT_REGISTERED", "spot_id": None, "pinky_id": pinky_id,
                "reason": f"핑키 {name}의 주차 마커가 정해지지 않았습니다 (config/pinky-markers.json)"}
    if not marker.registered:
        return {"status": "NOT_REGISTERED", "spot_id": marker.spot_id, "pinky_id": pinky_id,
                "reason": f"핑키 {name}(마커 {marker.marker_id}번): {marker.problem}"}
    return {**check(marker.spot_id), "pinky_id": pinky_id}
