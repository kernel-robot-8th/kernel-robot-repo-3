"""핑키마다 단 ArUco 마커와, 그 마커로 만든 적재 자리 등록을 다룬다(config/pinky-markers.json).

천장 카메라는 핑키 위에 붙은 마커로 도착을 판정한다. 마커가 붙은 위치가 핑키마다 조금씩 다르므로, 같은 적재 자리라도
핑키마다 자기 등록을 따로 가진다(.control-station/parking-spots/<spot_id>.json, 그 핑키를 올바른 자리에 세워 두고
만든 것). 주문 흐름은 주차 노드에 "배정된 핑키 자신의 자리"를 묻는다. 그래서 그 자리에 서 있는 다른 핑키를 배정된
핑키로 착각하는 일이 없다.

마커 파일 형식(schema_version 1):
    {"schema_version": 1, "pinkies": {"<pinky_id>": {"marker_id": <0~49>, "spot_id": "<자리 id>"}}}
마커 번호는 DICT_4X4_50 사전의 0~49이고, 두 핑키가 같은 번호를 쓸 수 없다.

위치: app.py가 시작할 때 `load_pinky_markers()`로 한 번 읽어 다음 곳에 넘긴다.
- 주문 흐름의 도착 판정: `arrival_for`를 주차 판정(RosParkingSource.check)과 묶어 쓴다.
- 핑키 화면 카드의 `marker`(pinky/gateway.py), 적재 자리 진입 안내(pinky/docking.py DockGuideRelay, 등록된 핑키마다 하나).
- 탑재 카드의 "핑키 도착" 줄(cameras/counts.py `pinky_arrivals`).
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from control_station import REPO_ROOT
from control_station.storage.json_files import load_json_file
from control_station.storage.store import PINKY_IDS, PINKY_NAMES

DEFAULT_MARKERS_PATH = REPO_ROOT / "config" / "pinky-markers.json"
# 주차 자리 등록 파일이 모이는 곳. 천장 캠 주차 노드(cpp/parking_vision_ros)도 같은 파일을 읽는다.
SPOTS_DIR = REPO_ROOT / ".control-station" / "parking-spots"


class PinkyMarkerError(ValueError):
    """마커 파일이 없거나 schema version 1 형식에 맞지 않는다."""


@dataclass(frozen=True)
class PinkyMarker:
    """핑키 한 대가 단 마커와, 그 핑키의 적재 자리 등록 상태."""

    pinky_id: str
    marker_id: int
    spot_id: str
    registered: bool  # 자리 파일이 있고, 이 마커로 만든 것이다
    problem: str | None  # 그 자리를 아직 쓸 수 없는 이유(쓸 수 있으면 None)

    def as_dict(self) -> dict[str, Any]:
        """화면 카드용(pinky_id는 카드에 이미 있으므로 뺀다)."""
        return {"marker_id": self.marker_id, "spot_id": self.spot_id, "registered": self.registered,
                "problem": self.problem}


def _registration(spot_id: str, marker_id: int, spots_dir: Path) -> str | None:
    """자리 파일이 이 마커로 등록돼 있는지 본다. 문제가 없으면 None, 있으면 화면에 보일 사유를 돌려준다."""
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
    """마커 파일을 읽고 검증해 {pinky_id: PinkyMarker}를 돌려준다.

    파일 경로는 인자 → CONTROL_STATION_PINKY_MARKERS → config/pinky-markers.json 순서로 정한다. 파일 형식이 틀리면
    (모르는 핑키, 범위 밖 마커 번호, 같은 번호 중복 등) PinkyMarkerError를 올린다. 반면 자리 등록이 없거나 다른
    마커로 돼 있는 것은 오류가 아니다. registered=False와 problem으로 남겨, 서버는 뜨고 그 핑키의 도착만 사람이
    확인하게 한다.
    """
    markers_path = Path(path or os.environ.get("CONTROL_STATION_PINKY_MARKERS") or DEFAULT_MARKERS_PATH)
    # 마커 파일은 작다(16 KiB 상한). 잘못 가리킨 큰 파일을 통째로 읽지 않는다.
    _, document = load_json_file(markers_path, 16 * 1024, "핑키 마커 파일", PinkyMarkerError)
    if not isinstance(document, dict) or document.get("schema_version") != 1 or not isinstance(document.get("pinkies"), dict):
        raise PinkyMarkerError("핑키 마커 파일은 schema_version 1과 pinkies를 가져야 합니다")
    markers: dict[str, PinkyMarker] = {}
    used: dict[int, str] = {}  # 마커 번호 -> 그 번호를 먼저 쓴 핑키(중복 검사용)
    for pinky_id, entry in document["pinkies"].items():
        if pinky_id not in PINKY_IDS or not isinstance(entry, dict):
            raise PinkyMarkerError(f"알 수 없는 핑키 {pinky_id}")
        marker_id, spot_id = entry.get("marker_id"), entry.get("spot_id")
        # bool은 int의 하위형이라 먼저 거른다(true가 1번 마커로 통과하지 않게).
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
    """배정된 핑키를 그 핑키 자신의 등록 자리에서 판정한다. 등록이 없는 핑키는 고쳐야 할 위치 문제로 돌려주고,
    주문 흐름은 이를 경보로 보여 주며 계속 기다린다.

    `check`는 spot_id를 받아 주차 판정 결과 dict를 주는 함수다(RosParkingSource.check 등). 결과에 pinky_id를
    덧붙여 돌려준다. 등록이 없으면 노드에 묻지 않고 status NOT_REGISTERED를 돌려준다.
    """
    marker = markers.get(pinky_id)
    name = PINKY_NAMES.get(pinky_id, pinky_id)
    if marker is None:
        return {"status": "NOT_REGISTERED", "spot_id": None, "pinky_id": pinky_id,
                "reason": f"핑키 {name}의 주차 마커가 정해지지 않았습니다 (config/pinky-markers.json)"}
    if not marker.registered:
        return {"status": "NOT_REGISTERED", "spot_id": marker.spot_id, "pinky_id": pinky_id,
                "reason": f"핑키 {name}(마커 {marker.marker_id}번): {marker.problem}"}
    return {**check(marker.spot_id), "pinky_id": pinky_id}
