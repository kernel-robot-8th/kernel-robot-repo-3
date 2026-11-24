"""수령 구역: 손님이 주문을 받는 곳(config/receiving-zones.json).

구역은 그 주문의 핑키 경로에서 마지막 웨이포인트다. 핑키 관제는 이미 경로 끝에서 멈추고(`course_complete`)
`mission-complete`를 받은 뒤에야 자기 주차 자리로 돌아가므로, 구역을 위해 핑키 쪽 코드는 필요 없다. 핑키 지도의
웨이포인트 하나와 경로 안의 순서만 있으면 된다. OrderDesk가 구역 목록을, OrderFlow가 경로(route_labels)를 쓴다.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path

from control_station import REPO_ROOT
from control_station.storage.json_files import load_json_file

DEFAULT_ZONES_PATH = REPO_ROOT / "config" / "receiving-zones.json"
MAX_ZONES_BYTES = 16 * 1024  # 구역 파일 크기 상한(16 KiB)
_ZONE_ID = re.compile(r"^[a-z0-9_]{1,32}$")  # 소문자·숫자·_ 1~32자
# 웨이포인트 이름: 1~20자, 양 끝은 공백이 아니다. 이름 안의 공백은 허용한다: 핑키 관제가 "수령 1" 같은 이름으로
# 기록한다(2026-11-17).
_WAYPOINT = re.compile(r"^[0-9A-Za-z가-힣_-](?:[0-9A-Za-z가-힣_ -]{0,18}[0-9A-Za-z가-힣_-])?$")


class ReceivingZoneError(ValueError):
    """구역 파일이 없거나 schema_version 1 형식에 맞지 않는다."""


@dataclass(frozen=True)
class ReceivingZone:
    """수령 구역 하나."""

    zone_id: str
    label: str  # 화면과 손님 앱에 보일 이름(예: "수령 1구역")
    # 핑키 관제에 보낼 웨이포인트 이름 순서. 지금은 [적재 자리 "5", 수령 구역 웨이포인트].
    route_labels: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {"zone_id": self.zone_id, "label": self.label, "route_labels": list(self.route_labels)}


def _zone(raw: object) -> ReceivingZone:
    """파일의 구역 항목 하나를 검사해 ReceivingZone으로 만든다. 경로는 적어도 두 곳(적재 자리, 수령 구역)이다."""
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
    """구역 id → 구역, 파일 순서대로(이 순서가 구역 고르기의 마지막 기준이다).

    경로: 인자 → CONTROL_STATION_RECEIVING_ZONES → config/receiving-zones.json. schema_version이 bool이면
    True == 1이라 통과해 버리므로 따로 거른다."""
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
