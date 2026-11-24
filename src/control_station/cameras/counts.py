"""주문 흐름을 위해 카메라의 YOLO 검출로 물체 개수를 센다(선반 재고, 핑키 위에 놓인 것).

선반 재고: 팔마다 재고 카메라(config/arms.json의 stock_camera_id)와 선반 칸 영역을 쓴다. 검출은 박스의 아래쪽
가운데가 그 선반의 칸 정확히 하나에 들어갈 때(IN_SLOT)만 그 선반 재고로 센다. 그래서 팔이 들고 있는 것, 핑키 위의
것, 책상 위의 것은 선반 재고가 아니다.
핑키 위 물건: 적재 카메라(주차 판정에도 쓰는 위 카메라)에서 중심이 핑키 쟁반 영역 안에 있는 검출을 센다. 그 영역은
CONTROL_STATION_PINKY_TRAY_REGION=x1,y1,x2,y2(픽셀)이고, 없으면 주차 자리에 등록된 핑키 마커 둘레의 상자다(핑키는
무엇을 싣기 전에 그 자리에 선다).

프레임 하나는 물체를 놓치거나 둘로 셀 수 있으므로, 개수를 셀 때마다 새 프레임 여러 장(기본 5장)을 보고 종류별
중앙값을 쓴다. 새 프레임을 보내지 않는 카메라나 준비되지 않은 검출기는 0이 아니라 아예 개수를 주지 않고
(CountUnavailable), 주문 흐름은 거기서 멈춘다.

화면 표시 전용(주문 흐름의 조건이 아님):
- `PinkyLoadCounter.live()`: 탑재 카드의 "탑재 목록"과 "핑키 도착" 줄. 기다리지 않고 최근 프레임 몇 장만 본다.
- `head_shelf_view_from_env`, `head_middle_view_from_env`: head_camera 화면의 선반 영역과 가운데 영역 개수.
- `camera_inventory`: api/orders.py가 응답의 `camera_stock`에 싣는 카메라 재고.

위치: app.py가 만든다. CONTROL_STATION_ORDER_COUNTS=camera일 때 ShelfStockCounter가 주문 흐름의 shelf_stock,
load_counter_from_env가 pinky_load가 된다. 카메라 스냅샷은 cameras/adapters.py의 snapshots()/load_snapshots()에서
오고, 칸 판정은 cameras/shelf_slots.py의 ShelfSlotZones.annotate가 붙인다.
"""

from __future__ import annotations

import json
import os
import statistics
import time
from collections import Counter
from pathlib import Path
from typing import Any, Callable

from control_station import utc_now_iso


class CountUnavailable(RuntimeError):
    """개수를 셀 수 없다. error_code(COUNT_*)는 주문 흐름이 멈춘 사유로 그대로 쓴다."""

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


def sample_frames(
    snapshots: Callable[[], list[Any]], camera_id: str, frames: int, max_wait_s: float, poll_s: float = 0.05
) -> list[dict[str, Any]]:
    """카메라 하나에서 서로 다른 새 프레임 `frames`장(검출 포함)을 모은다. 가장 새 것이 마지막.

    snapshots()를 poll_s(초)마다 다시 읽고, 같은 frame_sequence는 한 번만 센다. max_wait_s(초) 안에 다 모으지
    못하면 COUNT_CAMERA_NOT_FRESH, 검출기가 준비되지 않았으면 기다리지 않고 바로 COUNT_DETECTOR_NOT_READY.
    """
    seen: dict[int, dict[str, Any]] = {}
    deadline = time.monotonic() + max_wait_s
    last: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        camera = next((item for item in snapshots() if item.id == camera_id), None)
        if camera is None:
            raise CountUnavailable("COUNT_CAMERA_NOT_FOUND", f"카메라 {camera_id}가 설정되지 않았습니다")
        last = camera.to_dict()
        if last.get("detector_state") != "ready":
            raise CountUnavailable("COUNT_DETECTOR_NOT_READY", f"{camera_id} 물체 검출기가 준비되지 않았습니다 ({last.get('detector_state')})")
        if last.get("stream_state") == "STREAMING" and last.get("frame_freshness") == "FRESH":
            seen.setdefault(int(last.get("frame_sequence") or 0), last)
            if len(seen) >= frames:
                return [seen[key] for key in sorted(seen)][-frames:]
        time.sleep(poll_s)
    raise CountUnavailable(
        "COUNT_CAMERA_NOT_FRESH",
        f"{camera_id}에서 새 프레임 {frames}장을 {max_wait_s:.0f}초 안에 받지 못했습니다 "
        f"({(last or {}).get('stream_state')}, {(last or {}).get('frame_freshness')})",
    )


def median_counts(per_frame: list[Counter]) -> dict[str, int]:
    """프레임별 Counter에서 종류별 중앙값(반올림 정수)을 낸다. 어떤 프레임에 없는 종류는 그 프레임에서 0으로 치므로,
    한두 프레임에만 잠깐 보인 것은 걸러진다. 0개인 종류는 뺀다."""
    names = set().union(*per_frame) if per_frame else set()
    counts = {name: int(round(statistics.median(frame.get(name, 0) for frame in per_frame))) for name in names}
    return {name: count for name, count in sorted(counts.items()) if count > 0}


class ShelfStockCounter:
    """주문 흐름의 `shelf_stock()`: 팔마다 재고 카메라로 센 {shelf_id: {class: count}}.

    같은 카메라를 쓰는 선반(팔)들은 프레임을 한 번만 모아 선반별로 나눠 센다.
    """

    def __init__(self, snapshots: Callable[[], list[Any]], slot_zones: Any, slot_map: Any, arms: Any,
                 frames: int = 5, max_wait_s: float = 4.0) -> None:
        self.snapshots, self.slot_zones, self.slot_map, self.arms = snapshots, slot_zones, slot_map, arms
        self.frames, self.max_wait_s = frames, max_wait_s

    def __call__(self) -> dict[str, Any]:
        if self.arms is None or self.slot_map is None:
            raise CountUnavailable("COUNT_NOT_CONFIGURED", "팔 설정 또는 칸 정의가 없어 선반 재고를 셀 수 없습니다")
        shelves: dict[str, dict[str, int]] = {}
        frames_used: dict[str, list[int]] = {}
        cameras: dict[str, list[str]] = {}  # 재고 카메라 id -> 그 카메라가 보는 선반들
        for arm in self.arms.arms:
            cameras.setdefault(arm["stock_camera_id"], []).append(arm["shelf_id"])
        for camera_id, shelf_ids in cameras.items():
            per_shelf: dict[str, list[Counter]] = {shelf: [] for shelf in shelf_ids}
            frames = sample_frames(self.snapshots, camera_id, self.frames, self.max_wait_s)
            for frame in frames:
                annotated = self.slot_zones.annotate(frame)
                zones = annotated.get("slot_zones") or {}
                if zones.get("status") != "READY":
                    raise CountUnavailable("COUNT_SLOT_ZONES_NOT_READY",
                                           f"{camera_id} 칸 영역을 쓸 수 없습니다 ({zones.get('status', 'NOT_CONFIGURED')})")
                counts = {shelf: Counter() for shelf in shelf_ids}
                # IN_SLOT인 검출만, 그리고 지금 칸 정의에서 쓰는(active) 칸의 선반에만 센다.
                for detection in annotated.get("detections") or []:
                    assignment = detection.get("slot_assignment") or {}
                    slot = self.slot_map.active_slots.get(assignment.get("slot_id")) if assignment.get("status") == "IN_SLOT" else None
                    if slot is not None and slot.get("shelf_id") in counts:
                        counts[slot["shelf_id"]][str(detection.get("class_name"))] += 1
                for shelf, counter in counts.items():
                    per_shelf[shelf].append(counter)
            for shelf, frame_counts in per_shelf.items():
                shelves[shelf] = median_counts(frame_counts)
            frames_used[camera_id] = [int(frame.get("frame_sequence") or 0) for frame in frames]
        return {"source": "shelf_camera", "simulated": False, "captured_at": utc_now_iso("milliseconds"), "shelves": shelves,
                "frames": frames_used, "method": f"median of {self.frames} frames, IN_SLOT detections"}


def tray_region_from_spot(spot_path: Path, margin: float) -> list[float] | None:
    """주차 자리에 등록된 핑키 마커 둘레의 상자 [x1, y1, x2, y2](px). 마커 중심에서 상하좌우로 `margin` × 마커 한 변.
    자리 파일을 읽지 못하면 None."""
    try:
        corners = json.loads(spot_path.read_text(encoding="utf-8"))["reference_corners_px"]
        xs, ys = [point[0] for point in corners], [point[1] for point in corners]
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return None
    side = max(max(xs) - min(xs), max(ys) - min(ys))
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    return [round(cx - margin * side, 1), round(cy - margin * side, 1), round(cx + margin * side, 1), round(cy + margin * side, 1)]


def parse_region(text: str) -> list[float] | None:
    """'x1,y1,x2,y2' 문자열을 [x1, y1, x2, y2](px)로. 숫자 넷이 아니거나 x1<x2, y1<y2가 아니면 None."""
    try:
        values = [float(part) for part in text.split(",")]
    except ValueError:
        return None
    return values if len(values) == 4 and values[0] < values[2] and values[1] < values[3] else None


# camera_inventory가 세는 물체: (YOLO 클래스 이름, 화면 이름).
INVENTORY_CAMERA_PRODUCTS = (
    ("apple", "사과"), ("watermelon", "수박"), ("carrot", "당근"), ("kiwi", "키위"),
)


def camera_inventory(snapshots: Callable[[], list[Any]], shelf_region: list[float] | None,
                     front_region: list[float] | None) -> list[dict[str, Any]]:
    """head_camera의 선반 영역과 front_camera 영역에서 종류별로 센 카메라 재고(표시용).

    최신 프레임 한 장만 보고(중앙값 아님) 신뢰도 0.5 이상, 박스 중심이 영역 안인 검출만 센다. 카메라가
    ready/STREAMING/FRESH가 아니면 그 시점은 빼고, 남은 시점이 없으면 count는 None(모름)이다.
    """
    frames = {camera.id: camera.to_dict() for camera in snapshots()}
    views = []
    for camera_id, region in (("head_camera", shelf_region), ("front_camera", front_region)):
        frame = frames.get(camera_id)
        if not region or not frame or (frame["detector_state"], frame["stream_state"], frame["frame_freshness"]) != (
            "ready", "STREAMING", "FRESH"
        ):
            continue
        x1, y1, x2, y2 = region
        counts: Counter = Counter()
        for item in frame["detections"]:
            box = item.get("bbox_xyxy_px")
            if not box or len(box) != 4 or float(item.get("confidence") or 0) < 0.5:
                continue
            x, y = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            if x1 <= x <= x2 and y1 <= y <= y2:
                counts[item["class_name"]] += 1
        views.append(counts)
    # 두 시점이 같은 물체를 가리킨다는 공통 ID가 없으므로, 어느 쪽에서든 본 큰 개수를 쓴다(더하면 두 번 셈).
    return [{"object_class": name, "label": label,
             "count": max((counts[name] for counts in views), default=None)}
            for name, label in INVENTORY_CAMERA_PRODUCTS]


class PinkyLoadCounter:
    """주문 흐름의 `pinky_load(pinky_id)`: 적재 자리에서 핑키 쟁반 위의 {class: count}.

    같은 객체가 화면 표시용 live()도 맡는다. 기본으로 세지 않는 클래스는 pinky와 car다: 2026-11-07에 best11이 위에서
    본 핑키 본체를 car로 잘못 검출했다(docs/implementation/runbook.md). live()의 보관 상태(_held, _arrived)는
    잠금이 없으므로, 한 화면 조회 스레드에서 부른다고 가정한다.
    """

    def __init__(self, snapshots: Callable[[], list[Any]], camera_id: str, region: list[float] | None,
                 ignore: frozenset[str] = frozenset({"car", "pinky"}), frames: int = 5, max_wait_s: float = 4.0,
                 arrivals: list[dict[str, Any]] | None = None, arrival_tolerance_px: float = 30.0) -> None:
        self.snapshots, self.camera_id, self.region, self.ignore = snapshots, camera_id, region, ignore
        self.frames, self.max_wait_s = frames, max_wait_s
        # 카드의 "핑키 도착": 핑키마다 마커와, 적재 자리에서 서는 위치(등록 값, px).
        self.arrivals, self.arrival_tolerance_px = arrivals or [], arrival_tolerance_px
        self._arrived: dict[str, Any] | None = None  # 짐이 마커를 가리는 동안 마지막 ARRIVED를 유지한다
        # 화면의 적재 목록은 새 프레임을 최대 5장, 길어야 1초 본다.
        self.live_frames, self.live_wait_s = 5, 1.0
        try:  # 물체를 마지막으로 본 뒤 화면에 얼마나(초) 남겨 둘지
            self.live_hold_s = float(os.environ.get("CONTROL_STATION_LOAD_HOLD_S", "5"))
        except ValueError:
            self.live_hold_s = 5.0
        self._held: dict[str, dict[str, Any]] = {}  # 종류 -> 화면에 붙잡아 둔 {count, confidence, seen}
        self.live_min_confidence = 0.5  # 화면 목록에 올릴 최소 신뢰도

    def region_or_none(self) -> str:
        """시작 로그용 쟁반 영역 설명."""
        return "영역 " + ",".join(f"{value:.0f}" for value in self.region) if self.region else "영역 없음"

    def _on_tray(self, detection: dict[str, Any]) -> bool:
        """박스 중심이 쟁반 영역 안인가(영역이 없으면 모두 안으로 친다)."""
        if self.region is None:
            return True
        box = detection.get("bbox_xyxy_px")
        if not isinstance(box, list) or len(box) != 4:
            return False
        x, y = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        x1, y1, x2, y2 = self.region
        return x1 <= x <= x2 and y1 <= y <= y2

    def __call__(self, pinky_id: str) -> dict[str, Any]:
        """새 프레임 `frames`장의 쟁반 영역 검출로 센 종류별 중앙값. 영역을 모르면 COUNT_TRAY_REGION_UNKNOWN."""
        if self.region is None:
            raise CountUnavailable("COUNT_TRAY_REGION_UNKNOWN",
                                   "핑키 쟁반 영역을 모릅니다 (CONTROL_STATION_PINKY_TRAY_REGION 또는 주차 위치 등록 필요)")
        frames = sample_frames(self.snapshots, self.camera_id, self.frames, self.max_wait_s)
        per_frame = [
            Counter(str(item.get("class_name")) for item in frame.get("detections") or []
                    if self._on_tray(item) and str(item.get("class_name")) not in self.ignore)
            for frame in frames
        ]
        return {"source": self.camera_id, "simulated": False, "captured_at": utc_now_iso("milliseconds"), "pinky_id": pinky_id,
                "counts": median_counts(per_frame), "region_px": self.region,
                "frames": [int(frame.get("frame_sequence") or 0) for frame in frames],
                "ignored_classes": sorted(self.ignore), "method": f"median of {self.frames} frames inside the tray region"}


    def live(self) -> dict[str, Any]:
        """운영자 화면의 "탑재 목록": 기다리지 않고 최신 프레임에서 본 쟁반 위 개수.

        표시 전용이다. 프레임 하나는 물체를 놓치거나 둘로 셀 수 있으므로 주문 흐름은 계속 __call__(새 프레임들의
        중앙값)을 쓴다. 영역, 카메라, 검출기, 새 프레임이 없으면 0이 아니라 그 상태(state)로 알린다.
        """
        base: dict[str, Any] = {"camera_id": self.camera_id, "region_px": self.region, "counts": {}, "items": [],
                                "ignored_classes": sorted(self.ignore), "usable_for_robot_control": False}
        if self.region is None:
            return {**base, "state": "NO_TRAY_REGION",
                    "message": "핑키 쟁반 영역을 모릅니다 (CONTROL_STATION_PINKY_TRAY_REGION 또는 주차 위치 등록 필요)"}
        camera = next((item for item in self.snapshots() if item.id == self.camera_id), None)
        if camera is None:
            return {**base, "state": "NO_CAMERA", "message": f"카메라 {self.camera_id}가 설정되지 않았습니다"}
        frame = camera.to_dict()
        base["frame_sequence"] = int(frame.get("frame_sequence") or 0)
        if frame.get("detector_state") != "ready":
            return {**base, "state": "DETECTOR_NOT_READY",
                    "message": f"물체 검출기가 준비되지 않았습니다 ({frame.get('detector_state') or '없음'})"}
        if frame.get("stream_state") != "STREAMING" or frame.get("frame_freshness") != "FRESH":
            return {**base, "state": "NOT_FRESH",
                    "message": f"새 프레임이 없습니다 ({frame.get('stream_state')}, {frame.get('frame_freshness')})"}
        # 점수가 낮은 물체는 프레임마다 기준값 근처에서 깜빡이므로 최근 새 프레임 몇 장을 본다: 종류는 그중
        # 하나라도 보면 센다(한 프레임에서 가장 많이 본 수, Counter의 |). 마커도 하나라도 보면 쓴다.
        try:
            frames = sample_frames(self.snapshots, self.camera_id, self.live_frames, self.live_wait_s)
        except CountUnavailable:
            frames = [frame]  # 1초 안에 다 못 모으면 지금 프레임 하나로
        counts: Counter = Counter()
        best: dict[str, dict[str, Any]] = {}
        markers: dict[int, dict[str, Any]] = {}
        newest_counts: Counter = Counter()
        for sample in frames:
            seen = [item for item in sample.get("detections") or []
                    if self._on_tray(item) and str(item.get("class_name")) not in self.ignore
                    and float(item.get("confidence") or 0) >= self.live_min_confidence]
            newest_counts = Counter(str(item.get("class_name")) for item in seen)
            counts |= newest_counts
            for item in seen:
                name = str(item.get("class_name"))
                if name not in best or float(item.get("confidence") or 0) > float(best[name].get("confidence") or 0):
                    best[name] = item
            for marker in sample.get("markers") or []:
                if isinstance(marker, dict) and "id" in marker:
                    markers[int(marker["id"])] = marker  # 그 마커를 본 가장 새 프레임의 값을 쓴다
        base["frame_sequence"] = int(frames[-1].get("frame_sequence") or 0)
        pinky = self._arrival(list(markers.values()))
        # 종류마다 마지막으로 본 뒤 live_hold_s 동안 화면에 남겨, 몇 초 기준값 아래로 떨어진 물체가 깜빡 사라지지
        # 않게 한다. 핑키가 자리를 떠나는 것이 보이면 목록을 바로 비운다.
        now = time.monotonic()
        if pinky.get("state") == "NOT_AT_SPOT":
            self._held.clear()
        for name, count in counts.items():
            self._held[name] = {"count": count, "confidence": best[name].get("confidence"), "seen": now}
        for name in [name for name, held in self._held.items() if now - held["seen"] > self.live_hold_s]:
            del self._held[name]
        return {**base, "state": "READY", "captured_at": utc_now_iso("milliseconds"),
                "counts": {name: held["count"] for name, held in sorted(self._held.items())},
                "items": [{"class_name": name, "confidence": held["confidence"], "seen_now": name in newest_counts,
                           "seen_s_ago": round(now - held["seen"], 1)} for name, held in sorted(self._held.items())],
                "frames": [int(sample.get("frame_sequence") or 0) for sample in frames],
                "hold_s": self.live_hold_s, "pinky": pinky}

    def _arrival(self, markers: list[dict[str, Any]]) -> dict[str, Any]:
        """핑키의 마커가 등록된 적재 자리 위치에 있는가? 표시 전용이고 픽셀로 잰다(주차 판정은 C++ 노드가 한다).

        마커 중심이 등록 중심에서 arrival_tolerance_px 안이면 ARRIVED, 더 멀리 보이면 NOT_AT_SPOT(가장 가까운 핑키).
        짐이 마커를 가리는 동안은 마지막 ARRIVED를 그대로 둔다(marker_visible False).
        """
        if not self.arrivals:
            return {"state": "NOT_CONFIGURED", "message": "핑키 마커·적재 자리 등록이 없습니다"}
        seen = {int(marker["id"]): marker for marker in markers if isinstance(marker, dict) and "id" in marker}
        nearest: dict[str, Any] | None = None
        for entry in self.arrivals:
            marker = seen.get(entry["marker_id"])
            if marker is None:
                continue
            (x, y), (rx, ry) = marker["center_px"], entry["spot_center_px"]
            offset = round(((x - rx) ** 2 + (y - ry) ** 2) ** 0.5, 1)
            result = {**{key: entry[key] for key in ("pinky_id", "name", "marker_id", "spot_id")},
                      "offset_px": offset, "marker_visible": True, "last_seen_at": utc_now_iso("milliseconds")}
            if offset <= self.arrival_tolerance_px:
                self._arrived = {**result, "state": "ARRIVED"}
                return self._arrived
            if nearest is None or offset < nearest["offset_px"]:
                nearest = {**result, "state": "NOT_AT_SPOT"}
        if nearest is not None:
            if self._arrived and self._arrived["pinky_id"] == nearest["pinky_id"]:
                self._arrived = None  # 같은 핑키가 자리를 떠나는 것이 보였다
            if self._arrived is None:
                return nearest
        if self._arrived is not None:
            return {**self._arrived, "marker_visible": False}
        return {"state": "NOT_SEEN", "message": "핑키 마커가 보이지 않습니다"}


def head_region_view(snapshots: Callable[[], list[Any]], setting: str, default: str) -> PinkyLoadCounter:
    """head_camera 화면의 한 영역(환경 변수 `setting`, 없으면 `default` px)을 세는 표시용 뷰. 최신 프레임 1장만 본다."""
    region = parse_region(os.environ.get(setting, default))
    view = PinkyLoadCounter(snapshots, "head_camera", region, ignore=frozenset({"pinky"}))
    view.live_frames = 1
    return view


def head_shelf_view_from_env(snapshots: Callable[[], list[Any]]) -> PinkyLoadCounter:
    """head_camera 화면의 선반 영역(CONTROL_STATION_HEAD_SHELF_REGION, 기본 640x480 화면의 x 20~155 px)."""
    return head_region_view(snapshots, "CONTROL_STATION_HEAD_SHELF_REGION", "20,0,155,480")


def head_middle_view_from_env(snapshots: Callable[[], list[Any]]) -> PinkyLoadCounter:
    """head_camera 화면의 가운데 영역(CONTROL_STATION_HEAD_MIDDLE_REGION, 기본 x 156~365 px)."""
    return head_region_view(snapshots, "CONTROL_STATION_HEAD_MIDDLE_REGION", "156,0,365,480")


def load_counter_from_env(snapshots: Callable[[], list[Any]], repository_root: Path, spot_id: str | None) -> PinkyLoadCounter:
    """환경 변수로 핑키 탑재 세기를 만든다.

    카메라: CONTROL_STATION_LOAD_CAMERA(기본 head_camera). 쟁반 영역: CONTROL_STATION_PINKY_TRAY_REGION, 없으면
    spot_id 자리 파일의 마커 둘레(CONTROL_STATION_PINKY_TRAY_MARGIN 변, 기본 1.6). 세지 않을 클래스:
    pinky + CONTROL_STATION_LOAD_IGNORE_CLASSES(기본 car). 도착 허용 거리: CONTROL_STATION_ARRIVAL_TOLERANCE_PX(기본 30).
    숫자가 틀린 값은 기본값으로 둔다.
    """
    camera_id = os.environ.get("CONTROL_STATION_LOAD_CAMERA", "head_camera").strip() or "head_camera"
    region = parse_region(os.environ.get("CONTROL_STATION_PINKY_TRAY_REGION", ""))
    if region is None and spot_id:
        try:
            margin = float(os.environ.get("CONTROL_STATION_PINKY_TRAY_MARGIN", "1.6"))
        except ValueError:
            margin = 1.6
        region = tray_region_from_spot(repository_root / ".control-station" / "parking-spots" / f"{spot_id}.json", margin)
    ignore = frozenset({"pinky"}) | frozenset(
        name.strip() for name in os.environ.get("CONTROL_STATION_LOAD_IGNORE_CLASSES", "car").split(",") if name.strip()
    )
    try:
        tolerance = float(os.environ.get("CONTROL_STATION_ARRIVAL_TOLERANCE_PX", "30"))
    except ValueError:
        tolerance = 30.0
    return PinkyLoadCounter(snapshots, camera_id, region, ignore, arrivals=pinky_arrivals(repository_root),
                            arrival_tolerance_px=tolerance)


def pinky_arrivals(repository_root: Path) -> list[dict[str, Any]]:
    """적재 자리 등록이 있는 핑키마다: 마커 id와 그 자리에 등록된 마커 중심(px, 네 꼭짓점의 평균).
    마커 파일이나 자리 파일을 읽지 못하면 그 핑키(또는 전부)를 뺀다. 핑키 모듈 import는 이 함수를 부를 때만 한다."""
    from control_station.pinky.markers import PinkyMarkerError, load_pinky_markers
    from control_station.storage.store import PINKY_NAMES

    try:
        markers = load_pinky_markers(spots_dir=repository_root / ".control-station" / "parking-spots")
    except (PinkyMarkerError, OSError):
        return []
    arrivals = []
    for pinky_id, marker in markers.items():
        spot_path = repository_root / ".control-station" / "parking-spots" / f"{marker.spot_id}.json"
        try:
            corners = json.loads(spot_path.read_text(encoding="utf-8"))["reference_corners_px"]
            center = [sum(point[0] for point in corners) / 4, sum(point[1] for point in corners) / 4]
        except (OSError, ValueError, KeyError, TypeError, IndexError):
            continue
        arrivals.append({"pinky_id": pinky_id, "name": PINKY_NAMES.get(pinky_id, pinky_id), "marker_id": marker.marker_id,
                         "spot_id": marker.spot_id, "spot_center_px": [round(center[0], 1), round(center[1], 1)]})
    return arrivals
