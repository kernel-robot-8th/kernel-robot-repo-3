"""Counting objects with the cameras' YOLO detections for the order flow (shelf stock, what lies on the Pinky).

Shelf stock: each arm's stock camera (config/arms.json) with the shelf slot zones; a detection counts for a shelf
only when its bottom-center falls in exactly one slot of that shelf (IN_SLOT), so objects held by the arm, on the
Pinky or on the table are not shelf stock.
Pinky load: the load camera (the top camera that also feeds the parking judgment) counts detections whose center is
inside the Pinky tray region. That region is CONTROL_STATION_PINKY_TRAY_REGION=x1,y1,x2,y2 in pixels, or else a box
around the Pinky marker as registered for the parking spot (the Pinky stops there before anything is loaded).

One frame can miss or double an object, so each count looks at several fresh frames and takes the median per
class. A camera that is not streaming fresh frames or a detector that is not ready gives no count at all
(CountUnavailable) rather than a zero, and the order flow stops on it.
"""

from __future__ import annotations

import json
import os
import statistics
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


class CountUnavailable(RuntimeError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def sample_frames(
    snapshots: Callable[[], list[Any]], camera_id: str, frames: int, max_wait_s: float, poll_s: float = 0.05
) -> list[dict[str, Any]]:
    """`frames` distinct fresh frames (with their detections) of one camera, newest last."""
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
    names = set().union(*per_frame) if per_frame else set()
    counts = {name: int(round(statistics.median(frame.get(name, 0) for frame in per_frame))) for name in names}
    return {name: count for name, count in sorted(counts.items()) if count > 0}


class ShelfStockCounter:
    """Order flow `shelf_stock()`: {shelf_id: {class: count}} from each arm's stock camera."""

    def __init__(self, snapshots: Callable[[], list[Any]], slot_zones: Any, slot_map: Any, arms: Any,
                 frames: int = 5, max_wait_s: float = 4.0) -> None:
        self.snapshots, self.slot_zones, self.slot_map, self.arms = snapshots, slot_zones, slot_map, arms
        self.frames, self.max_wait_s = frames, max_wait_s

    def __call__(self) -> dict[str, Any]:
        if self.arms is None or self.slot_map is None:
            raise CountUnavailable("COUNT_NOT_CONFIGURED", "팔 설정 또는 칸 정의가 없어 선반 재고를 셀 수 없습니다")
        shelves: dict[str, dict[str, int]] = {}
        frames_used: dict[str, list[int]] = {}
        cameras: dict[str, list[str]] = {}
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
        return {"source": "shelf_camera", "simulated": False, "captured_at": _now(), "shelves": shelves,
                "frames": frames_used, "method": f"median of {self.frames} frames, IN_SLOT detections"}


def tray_region_from_spot(spot_path: Path, margin: float) -> list[float] | None:
    """A box around the Pinky marker registered for the parking spot, `margin` marker sides from its center."""
    try:
        corners = json.loads(spot_path.read_text(encoding="utf-8"))["reference_corners_px"]
        xs, ys = [point[0] for point in corners], [point[1] for point in corners]
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return None
    side = max(max(xs) - min(xs), max(ys) - min(ys))
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    return [round(cx - margin * side, 1), round(cy - margin * side, 1), round(cx + margin * side, 1), round(cy + margin * side, 1)]


def parse_region(text: str) -> list[float] | None:
    try:
        values = [float(part) for part in text.split(",")]
    except ValueError:
        return None
    return values if len(values) == 4 and values[0] < values[2] and values[1] < values[3] else None


class PinkyLoadCounter:
    """Order flow `pinky_load(pinky_id)`: {class: count} on the Pinky tray at the loading spot."""

    def __init__(self, snapshots: Callable[[], list[Any]], camera_id: str, region: list[float] | None,
                 ignore: frozenset[str] = frozenset({"car"}), frames: int = 5, max_wait_s: float = 4.0) -> None:
        self.snapshots, self.camera_id, self.region, self.ignore = snapshots, camera_id, region, ignore
        self.frames, self.max_wait_s = frames, max_wait_s

    def region_or_none(self) -> str:
        return "영역 " + ",".join(f"{value:.0f}" for value in self.region) if self.region else "영역 없음"

    def _on_tray(self, detection: dict[str, Any]) -> bool:
        if self.region is None:
            return True
        box = detection.get("bbox_xyxy_px")
        if not isinstance(box, list) or len(box) != 4:
            return False
        x, y = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        x1, y1, x2, y2 = self.region
        return x1 <= x <= x2 and y1 <= y <= y2

    def __call__(self, pinky_id: str) -> dict[str, Any]:
        if self.region is None:
            raise CountUnavailable("COUNT_TRAY_REGION_UNKNOWN",
                                   "핑키 쟁반 영역을 모릅니다 (CONTROL_STATION_PINKY_TRAY_REGION 또는 주차 위치 등록 필요)")
        frames = sample_frames(self.snapshots, self.camera_id, self.frames, self.max_wait_s)
        per_frame = [
            Counter(str(item.get("class_name")) for item in frame.get("detections") or []
                    if self._on_tray(item) and str(item.get("class_name")) not in self.ignore)
            for frame in frames
        ]
        return {"source": self.camera_id, "simulated": False, "captured_at": _now(), "pinky_id": pinky_id,
                "counts": median_counts(per_frame), "region_px": self.region,
                "frames": [int(frame.get("frame_sequence") or 0) for frame in frames],
                "ignored_classes": sorted(self.ignore), "method": f"median of {self.frames} frames inside the tray region"}


def load_counter_from_env(snapshots: Callable[[], list[Any]], repository_root: Path, spot_id: str | None) -> PinkyLoadCounter:
    camera_id = os.environ.get("CONTROL_STATION_LOAD_CAMERA", "head_camera").strip() or "head_camera"
    region = parse_region(os.environ.get("CONTROL_STATION_PINKY_TRAY_REGION", ""))
    if region is None and spot_id:
        try:
            margin = float(os.environ.get("CONTROL_STATION_PINKY_TRAY_MARGIN", "1.6"))
        except ValueError:
            margin = 1.6
        region = tray_region_from_spot(repository_root / ".control-station" / "parking-spots" / f"{spot_id}.json", margin)
    ignore = frozenset(
        name.strip() for name in os.environ.get("CONTROL_STATION_LOAD_IGNORE_CLASSES", "car").split(",") if name.strip()
    )
    return PinkyLoadCounter(snapshots, camera_id, region, ignore)
