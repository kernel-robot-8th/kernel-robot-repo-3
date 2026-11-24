"""선반 칸: 칸 정의 파일(config/shelf-slots.json)과 정면 카메라 화면 위의 칸 영역.

1) 칸 정의 (`load_shelf_slot_map`, `ShelfSlotMap`)
   팔 명령이 가리킬 수 있는 선반 칸 목록이다. scripts/derive_shelf_slots.py가 팀의 3D 선반 모델에서 만든다.
   파일의 SHA-256이 칸 정의 버전이 되고, 칸 명령마다 이 버전을 싣는다. 그래서 팔 워커는 자기가 배운 자세와
   다른 칸 정의로 만든 명령을 거절할 수 있다.

2) 칸 영역 (`ShelfSlotZones`, `select_slot_for_class`)
   선반 위치 보정 결과(scripts/calibrate_shelf_pose.py)를 운영자가 `<CONTROL_STATION_SHELF_POSE_DIR>/<camera_id>.json`으로
   복사하면 켜진다. 보정 결과가 지금 카메라(ID, 해상도, 영상 변환)와 불러온 칸 정의 버전에 맞을 때만 쓴다.

   검출 하나의 기준점은 박스의 아래쪽 가운데, 곧 물체가 선반 바닥에 닿는 곳이다. 칸 전체가 아니라 칸의 기준
   영역(바닥부터 앞 턱 바로 위까지)과 비교하므로, 위 칸에 있는 물체가 바로 아래 칸으로 잡히지 않는다.

   `ShelfSlotZones.annotate`는 카메라 상태에 칸 영역과 검출별 칸을 덧붙이기만 한다(화면이 그리고, 재고 세기가 센다).
   이것을 실제 선택으로 바꾸는 곳은 `select_slot_for_class` 하나다. 물체가 들어 있는 칸 하나를 고르거나 거절하고,
   그 기록이 팔 명령의 카메라 근거가 된다(POST /api/arm/commands/from-camera, 작업 흐름의 팔 단계).
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from control_station import REPO_ROOT
from control_station.storage.json_files import load_json_file, unique_keys


# ──── 1) 칸 정의 (config/shelf-slots.json) ───────────────────────────────────────────────────────────────────

DEFAULT_SLOT_MAP_PATH = REPO_ROOT / "config" / "shelf-slots.json"
MAX_SLOT_MAP_BYTES = 256 * 1024
_SLOT_ID = re.compile(r"^slot_[a-z]_\d+_\d+$")


class ShelfSlotMapError(ValueError):
    """The slot map file is missing or does not match schema version 1."""


@dataclass(frozen=True)
class ShelfSlotMap:
    path: Path
    version: str
    document: dict[str, Any] = field(repr=False)
    active_slots: dict[str, dict[str, Any]] = field(repr=False)


def load_shelf_slot_map(path: str | Path | None = None) -> ShelfSlotMap:
    slot_path = Path(path or os.environ.get("CONTROL_STATION_SHELF_SLOTS") or DEFAULT_SLOT_MAP_PATH)
    raw, document = load_json_file(slot_path, MAX_SLOT_MAP_BYTES, "칸 정의 파일", ShelfSlotMapError)
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ShelfSlotMapError("칸 정의 파일은 schema_version 1이어야 합니다")
    frame = document.get("frame")
    if not isinstance(frame, dict) or frame.get("unit") != "m" or not isinstance(frame.get("frame_id"), str):
        raise ShelfSlotMapError("칸 정의 파일의 frame에는 frame_id와 unit 'm'이 필요합니다")
    slots = document.get("slots")
    if not isinstance(slots, list) or not slots:
        raise ShelfSlotMapError("칸 정의 파일에 slots가 없습니다")
    seen: set[str] = set()
    active: dict[str, dict[str, Any]] = {}
    for slot in slots:
        slot_id = slot.get("slot_id") if isinstance(slot, dict) else None
        if not isinstance(slot_id, str) or not _SLOT_ID.fullmatch(slot_id) or slot_id in seen:
            raise ShelfSlotMapError(f"잘못되었거나 중복된 slot_id: {slot_id!r}")
        if not isinstance(slot.get("label"), str) or not isinstance(slot.get("active"), bool):
            raise ShelfSlotMapError(f"{slot_id}에 label과 active가 필요합니다")
        seen.add(slot_id)
        if slot["active"]:
            active[slot_id] = slot
    if not active:
        raise ShelfSlotMapError("사용하는(active) 칸이 하나도 없습니다")
    return ShelfSlotMap(slot_path, hashlib.sha256(raw).hexdigest(), document, active)


# ──── 2) 정면 카메라 화면 위의 칸 영역 ────────────────────────────────────────────────────────────────────────────────

DEFAULT_POSE_DIR = REPO_ROOT / ".control-station" / "shelf-pose-calibrations"
MAX_POSE_BYTES = 1_000_000
_CAMERA_ID = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")


def _point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> int:
    """Return 1 inside, 0 on a boundary, and -1 outside."""
    x, y = point
    inside = False
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        x1, y1 = start
        x2, y2 = end
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        scale = max(1.0, abs(x), abs(y), abs(x1), abs(y1), abs(x2), abs(y2))
        if (
            abs(cross) <= 1e-9 * scale
            and min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9
            and min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9
        ):
            return 0
        if (y1 > y) != (y2 > y):
            crossing_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < crossing_x:
                inside = not inside
    return 1 if inside else -1


def _polygon(value: object) -> list[tuple[float, float]] | None:
    if not isinstance(value, list) or len(value) < 3:
        return None
    points = []
    for point in value:
        if (
            not isinstance(point, list)
            or len(point) != 2
            or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in point)
        ):
            return None
        points.append((float(point[0]), float(point[1])))
    return points


class SlotSelectionError(ValueError):
    """The camera view does not single out one slot for the requested object; no command is made."""

    def __init__(self, error_code: str, message: str, details: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.details = details or {}


def select_slot_for_class(camera: dict[str, Any], object_class: str) -> dict[str, Any]:
    """Pick the one slot holding `object_class` in an annotated camera snapshot, or refuse.

    Refuses when the frame is not fresh, the detector or slot zones are not ready, the object is not
    seen in any slot, it sits on a slot boundary, or it is seen in more than one slot. The returned
    record is the camera evidence stored with the command.
    """
    if camera.get("stream_state") != "STREAMING" or camera.get("frame_freshness") != "FRESH":
        raise SlotSelectionError("CAMERA_NOT_FRESH", "정면 카메라 영상이 최신이 아닙니다",
                                 {"stream_state": camera.get("stream_state"), "frame_freshness": camera.get("frame_freshness")})
    if camera.get("detector_state") != "ready":
        raise SlotSelectionError("DETECTOR_NOT_READY", "정면 카메라 물체 검출기가 준비되지 않았습니다",
                                 {"detector_state": camera.get("detector_state")})
    zones = camera.get("slot_zones") or {"status": "NOT_CONFIGURED"}
    if zones.get("status") != "READY":
        raise SlotSelectionError("SLOT_ZONES_NOT_READY", f"칸 영역을 쓸 수 없습니다 ({zones.get('status')})",
                                 {"slot_zones_status": zones.get("status"), "detail": zones.get("detail")})
    wanted = object_class.strip().casefold()
    matches = [item for item in camera.get("detections") or [] if str(item.get("class_name", "")).casefold() == wanted]
    in_slot = [item for item in matches if (item.get("slot_assignment") or {}).get("status") == "IN_SLOT"]
    unsure = [item for item in matches if (item.get("slot_assignment") or {}).get("status") == "AMBIGUOUS"]
    seen = sorted({str(item.get("class_name")) for item in camera.get("detections") or []})
    if unsure:
        raise SlotSelectionError("TARGET_AMBIGUOUS", f"{object_class}이(가) 칸 경계에 걸려 칸을 정할 수 없습니다",
                                 {"candidates": [item["slot_assignment"]["candidates"] for item in unsure]})
    if not in_slot:
        raise SlotSelectionError("TARGET_NOT_FOUND", f"정면 카메라에서 칸 안의 {object_class}을(를) 찾지 못했습니다",
                                 {"detected_classes": seen, "outside_slots": len(matches)})
    if len(in_slot) > 1:
        raise SlotSelectionError("TARGET_MULTIPLE", f"{object_class}이(가) 여러 칸에 있어 하나를 고를 수 없습니다",
                                 {"slots": [item["slot_assignment"]["slot_id"] for item in in_slot]})
    detection = in_slot[0]
    assignment = detection["slot_assignment"]
    return {
        "camera_id": camera.get("id"),
        "captured_at": detection.get("captured_at") or camera.get("last_frame_at"),
        "frame_sequence": detection.get("frame_sequence") or camera.get("frame_sequence"),
        "frame_sha256": detection.get("frame_sha256") or camera.get("frame_sha256"),
        "pixel_transform_id": camera.get("pixel_transform_id"),
        "class_name": detection.get("class_name"),
        "confidence": detection.get("confidence"),
        "bbox_xyxy_px": detection.get("bbox_xyxy_px"),
        "anchor_px": assignment["anchor_px"],
        "slot_id": assignment["slot_id"],
        "slot_label": assignment["label"],
        "shelf_pose_id": zones.get("pose_id"),
        "shelf_pose_sha256": zones.get("artifact_sha256"),
        "detector_weights_sha256": camera.get("detector_weights_sha256"),
    }


class ShelfSlotZones:
    def __init__(self, slot_map: ShelfSlotMap | None, directory: str | Path | None = None) -> None:
        self.slot_map = slot_map
        self.directory = Path(
            directory or os.environ.get("CONTROL_STATION_SHELF_POSE_DIR") or DEFAULT_POSE_DIR
        )
        self._lock = threading.Lock()
        self._cache: dict[str, tuple[tuple[int, int], dict[str, Any]]] = {}

    def _load(self, camera_id: str) -> dict[str, Any]:
        """Read and validate the activated pose artifact; cached until the file changes."""
        if not _CAMERA_ID.fullmatch(camera_id):
            return {"status": "INVALID", "detail": "camera_id is invalid"}
        path = self.directory / f"{camera_id}.json"
        if self.directory.is_symlink() or path.is_symlink():
            return {"status": "INVALID", "detail": "pose directory and artifact must not be symbolic links"}
        try:
            stat = path.stat()
        except FileNotFoundError:
            return {"status": "NOT_CONFIGURED", "detail": f"{path} 없음"}
        except OSError as exc:
            return {"status": "INVALID", "detail": str(exc)}
        key = (stat.st_mtime_ns, stat.st_size)
        with self._lock:
            cached = self._cache.get(camera_id)
            if cached and cached[0] == key:
                return cached[1]
        result = self._validate(camera_id, path)
        with self._lock:
            self._cache[camera_id] = (key, result)
        return result

    def _validate(self, camera_id: str, path: Path) -> dict[str, Any]:
        try:
            if not path.is_file():
                raise ValueError("artifact is not a regular file")
            raw = path.read_bytes()
            if len(raw) > MAX_POSE_BYTES:
                raise ValueError("artifact is larger than 1 MB")
            artifact = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_keys)
            if not isinstance(artifact, dict) or artifact.get("schema_version") != 1:
                raise ValueError("unsupported shelf pose schema_version")
            if artifact.get("status") != "DRAFT_ONLY" or artifact.get("usable_for_robot_control") is not False:
                raise ValueError("shelf pose must be a DRAFT_ONLY artifact unusable for robot control")
            if artifact.get("camera_id") != camera_id:
                raise ValueError("artifact camera_id does not match its file name")
            if self.slot_map is None:
                return {"status": "SLOT_MAP_UNAVAILABLE", "detail": "칸 정의 파일을 불러오지 못했습니다"}
            if artifact.get("sources", {}).get("slot_map_sha256") != self.slot_map.version:
                return {
                    "status": "SLOT_MAP_MISMATCH",
                    "detail": "보정 결과가 지금 칸 정의와 다른 버전으로 만들어졌습니다. 다시 보정하세요.",
                }
            slots = []
            for slot in artifact.get("slots", []):
                # The anchor zone (deck up to just above the front lip) decides; the volume is shown only.
                anchor = _polygon(slot.get("anchor_polygon_px"))
                volume = _polygon(slot.get("polygon_px"))
                if slot.get("slot_id") not in self.slot_map.active_slots or anchor is None or volume is None:
                    raise ValueError(
                        f"invalid slot zone {slot.get('slot_id')!r}; re-run calibrate_shelf_pose.py for anchor zones"
                    )
                definition = self.slot_map.active_slots[slot["slot_id"]]
                slots.append(
                    {
                        "slot_id": slot["slot_id"],
                        "label": definition["label"],
                        "polygon_px": [list(point) for point in anchor],
                        "volume_polygon_px": [list(point) for point in volume],
                    }
                )
            if not slots:
                raise ValueError("artifact has no slot zones")
        except (OSError, ValueError, UnicodeDecodeError, AttributeError, TypeError) as exc:
            return {"status": "INVALID", "detail": str(exc)}
        return {
            "status": "READY",
            "pose_id": artifact.get("pose_id"),
            "artifact_sha256": hashlib.sha256(raw).hexdigest(),
            "image_size_px": artifact.get("image_size_px"),
            "pixel_transform_id": artifact.get("pixel_transform_id"),
            "reprojection_rms_px": artifact.get("reprojection_rms_px"),
            "slots": slots,
        }

    def annotate(self, camera: dict[str, Any]) -> dict[str, Any]:
        """Add `slot_zones` to a camera snapshot and `slot_assignment` to each detection, if configured."""
        zones = self._load(str(camera.get("id")))
        if zones["status"] == "NOT_CONFIGURED":
            return camera
        zones = {**zones, "used_for_selection": False}
        if zones["status"] == "READY":
            size = zones["image_size_px"] or {}
            if (size.get("width"), size.get("height")) != (camera.get("frame_width"), camera.get("frame_height")):
                zones = {"status": "IMAGE_MISMATCH", "detail": "카메라 해상도가 보정 때와 다릅니다", "used_for_selection": False}
            elif zones["pixel_transform_id"] != camera.get("pixel_transform_id"):
                zones = {"status": "IMAGE_MISMATCH", "detail": "pixel_transform_id가 보정 때와 다릅니다", "used_for_selection": False}
        camera = {**camera, "slot_zones": zones}
        if zones["status"] == "READY" and camera.get("detections"):
            camera["detections"] = [
                {**detection, "slot_assignment": self.assign(detection, zones["slots"])}
                for detection in camera["detections"]
            ]
        return camera

    @staticmethod
    def assign(detection: dict[str, Any], slots: list[dict[str, Any]]) -> dict[str, Any]:
        box = detection.get("bbox_xyxy_px")
        if not isinstance(box, list) or len(box) != 4:
            return {"status": "NO_BBOX", "used_for_selection": False}
        anchor = ((float(box[0]) + float(box[2])) / 2, float(box[3]))
        inside, edge = [], []
        for slot in slots:
            position = _point_in_polygon(anchor, [tuple(point) for point in slot["polygon_px"]])
            (inside if position > 0 else edge if position == 0 else []).append(slot)
        result: dict[str, Any] = {
            "anchor_px": [round(anchor[0], 1), round(anchor[1], 1)],
            "candidates": [slot["slot_id"] for slot in inside + edge],
            "used_for_selection": False,
        }
        if len(inside) == 1 and not edge:
            slot = inside[0]
            result.update(status="IN_SLOT", slot_id=slot["slot_id"], label=slot["label"])
        elif inside or edge:
            result["status"] = "AMBIGUOUS"
        else:
            result["status"] = "OUTSIDE"
        return result
