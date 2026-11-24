"""주문을 핑키 여러 대로 나누고 두 팔의 적재 시간을 계산할 때 쓰는 계획값(config/fleet-planning.json).

측정값이 아니라 추정값이다: 핑키 쟁반이 담는 칸 수, 물체마다 차지하는 칸(목록에 없으면 1), 집기 한 번이 선반에서와
핑키 위에서 쓰는 시간(초). 이 값은 미리보기와 운송분 나누기에만 쓰이고, 장치를 움직이거나 동작이 끝났다고 판단하는
데는 절대 쓰이지 않는다. OrderDesk(desk.py)가 읽는다.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from control_station import REPO_ROOT
from control_station.orders.assignment import ArmPick, PickTiming
from control_station.storage.json_files import load_json_file

DEFAULT_PLANNING_PATH = REPO_ROOT / "config" / "fleet-planning.json"
MAX_PLANNING_BYTES = 16 * 1024  # 계획값 파일 크기 상한(16 KiB)


class FleetPlanningError(ValueError):
    """계획값 파일이 없거나 schema_version 1 형식에 맞지 않는다."""


@dataclass(frozen=True)
class FleetPlanning:
    """검사를 마친 계획값."""

    pinky_capacity: int  # 핑키 쟁반 칸 수
    shelf_s: float  # 집기 한 번 중 자기 선반에서 쓰는 시간(초): 잡고 들어 대기 자세까지
    place_s: float  # 핑키 위 적재 공간에서 쓰는 시간(초): 놓고 빠져나오기까지
    unit_sizes: Mapping[str, int] = field(default_factory=dict)  # 물체 → 차지하는 칸(없으면 1)
    # 집기를 팔이 핑키 위 공간에 들어가는 순간에서 나눌 수 있을 때만 두 팔을 겹친다. 지금 ACT 정책은 잡기 →
    # 놓기 → 쉬는 자세를 한 번에 돌리므로, 두 팔은 집기 하나를 통째로 번갈아 한다.
    arms_overlap: bool = False

    def size_of(self, object_class: str) -> int:
        """물체 하나가 쟁반에서 차지하는 칸 수(목록에 없으면 1)."""
        return self.unit_sizes.get(object_class, 1)

    def timing(self, pick: ArmPick) -> PickTiming:
        """집기 하나의 시간. 지금은 물체·팔과 상관없이 같은 값이다(pick은 schedule_loading 형식에 맞춰 받기만 한다)."""
        return PickTiming(shelf_s=self.shelf_s, place_s=self.place_s)


def _positive_number(value: object, name: str) -> float:
    """0초 초과 600초 이하의 숫자만 받는다. bool은 int의 하위 클래스라 true가 1로 통과하지 않게 따로 막는다."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value <= 600:
        raise FleetPlanningError(f"{name} must be a number of seconds above 0")
    return float(value)


def load_fleet_planning(path: str | Path | None = None) -> FleetPlanning:
    """계획값 파일을 읽고 검사한다. 경로: 인자 → CONTROL_STATION_FLEET_PLANNING → config/fleet-planning.json.

    정해진 항목 밖의 키가 있거나 값이 범위를 벗어나면 FleetPlanningError다(조용히 기본값으로 바꾸지 않는다)."""
    planning_path = Path(path or os.environ.get("CONTROL_STATION_FLEET_PLANNING") or DEFAULT_PLANNING_PATH)
    _, document = load_json_file(planning_path, MAX_PLANNING_BYTES, "핑키·팔 계획값 파일", FleetPlanningError)
    expected = {"schema_version", "notes", "pinky_capacity", "unit_sizes", "arms_overlap", "pick_timing_s"}
    if not isinstance(document, dict) or set(document) - expected or document.get("schema_version") != 1:
        raise FleetPlanningError("핑키·팔 계획값 파일은 schema_version 1과 정해진 항목만 가져야 합니다")
    capacity = document.get("pinky_capacity")
    if isinstance(capacity, bool) or not isinstance(capacity, int) or not 1 <= capacity <= 50:
        raise FleetPlanningError("pinky_capacity must be an integer from 1 to 50")
    # 물체 크기는 쟁반보다 클 수 없다: 그런 물체는 어떤 핑키에도 실을 수 없다.
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
