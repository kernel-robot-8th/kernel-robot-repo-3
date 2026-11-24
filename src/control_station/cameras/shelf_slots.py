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

   좌표는 모두 카메라 원본 영상의 픽셀(px)이다. 칸 영역 파일은 (수정 시각, 크기)가 바뀔 때만 다시 읽어 검증한다.
   여러 HTTP 스레드가 annotate를 부르므로 그 캐시는 잠금 아래에서 다룬다.

위치: app.py가 시작할 때 칸 정의를 읽고 ShelfSlotZones를 만든다(server.shelf_slot_zones). 카메라 API(api/cameras.py),
재고 세기(cameras/counts.py ShelfStockCounter), 팔 명령(api/arm.py, arms/commands.py)이 쓴다.
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
# 칸 id 형식: slot_<선반 글자>_<줄>_<칸>, 예: slot_a_1_2.
_SLOT_ID = re.compile(r"^slot_[a-z]_\d+_\d+$")


class ShelfSlotMapError(ValueError):
    """칸 정의 파일이 없거나 schema version 1 형식에 맞지 않는다."""


@dataclass(frozen=True)
class ShelfSlotMap:
    """불러온 칸 정의. version은 파일 바이트의 SHA-256이고, active_slots는 쓰는 칸만 모은 {slot_id: 칸}이다."""

    path: Path
    version: str
    document: dict[str, Any] = field(repr=False)
    active_slots: dict[str, dict[str, Any]] = field(repr=False)


def load_shelf_slot_map(path: str | Path | None = None) -> ShelfSlotMap:
    """칸 정의 파일을 읽고 검증한다. 경로: 인자 → CONTROL_STATION_SHELF_SLOTS → config/shelf-slots.json.

    frame의 단위는 m여야 하고, slot_id는 형식에 맞고 겹치지 않아야 하며, 쓰는(active) 칸이 하나 이상 있어야 한다.
    어긋나면 ShelfSlotMapError.
    """
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
# camera_id가 파일 이름이 되므로 경로를 벗어나는 글자(/ 등)를 막는다.
_CAMERA_ID = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")


def _point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> int:
    """안이면 1, 경계 위면 0, 밖이면 -1을 돌려준다.

    변마다 외적이 0에 가깝고(좌표 크기에 비례한 1e-9 허용) 점이 그 변의 범위 안이면 경계로 본다. 그 밖에는 오른쪽으로
    쏜 반직선이 변을 몇 번 지나는지(ray casting)로 안팎을 정한다. 경계 위 검출은 칸을 정할 수 없으므로 따로 구분한다.
    """
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
    """[[x, y], ...] 꼴(점 3개 이상, 숫자만)이면 (x, y) 튜플 목록, 아니면 None."""
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
    """카메라 화면으로는 요청한 물체의 칸을 하나로 정할 수 없다. 명령은 만들지 않는다. details는 API 응답에 싣는다."""

    def __init__(self, error_code: str, message: str, details: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.details = details or {}


def select_slot_for_class(camera: dict[str, Any], object_class: str) -> dict[str, Any]:
    """annotate된 카메라 스냅샷에서 `object_class`가 든 칸 하나를 고르거나 거절한다.

    프레임이 최신이 아님, 검출기나 칸 영역이 준비되지 않음, 어느 칸에도 그 물체가 없음, 칸 경계에 걸림, 여러 칸에
    보임 가운데 하나면 거절한다. 돌려주는 기록이 명령과 함께 저장되는 카메라 근거다(프레임 번호·해시, 가중치 해시,
    보정 결과 해시까지 실어 어떤 화면으로 정했는지 나중에 따질 수 있다). 클래스 이름은 대소문자를 가리지 않는다.
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
    # 경계에 걸린 것이 하나라도 있으면, 다른 하나가 칸 안에 있더라도 거절한다(어느 쪽을 집을지 확실하지 않음).
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
    """카메라별 칸 영역. `<directory>/<camera_id>.json`(선반 위치 보정 결과)을 읽어 카메라 스냅샷에 칸을 덧붙인다."""

    def __init__(self, slot_map: ShelfSlotMap | None, directory: str | Path | None = None) -> None:
        self.slot_map = slot_map  # None이면 칸 정의를 못 읽은 것: 모든 카메라가 SLOT_MAP_UNAVAILABLE
        self.directory = Path(
            directory or os.environ.get("CONTROL_STATION_SHELF_POSE_DIR") or DEFAULT_POSE_DIR
        )
        self._lock = threading.Lock()
        # camera_id -> ((st_mtime_ns, st_size), 검증 결과). 파일이 바뀌지 않으면 다시 검증하지 않는다.
        self._cache: dict[str, tuple[tuple[int, int], dict[str, Any]]] = {}

    def _load(self, camera_id: str) -> dict[str, Any]:
        """운영자가 켠(복사해 둔) 보정 결과를 읽고 검증한다. 파일이 바뀔 때까지 캐시한다.

        검증은 잠금 밖에서 한다(파일 읽기로 다른 스레드를 막지 않게). 두 스레드가 같은 파일을 함께 검증할 수는 있지만
        결과가 같으므로 문제없다.
        """
        if not _CAMERA_ID.fullmatch(camera_id):
            return {"status": "INVALID", "detail": "camera_id is invalid"}
        path = self.directory / f"{camera_id}.json"
        # 심볼릭 링크를 거부해, 보정 디렉터리 밖의 파일이 칸 영역으로 쓰이지 않게 한다.
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
        """보정 결과 파일 하나를 검사해 칸 영역(status READY)이나 쓸 수 없는 이유(INVALID, SLOT_MAP_*)를 돌려준다.

        보정 결과는 DRAFT_ONLY이고 usable_for_robot_control이 False여야 한다: 칸 영역은 카메라 근거일 뿐 로봇 좌표가
        아니다. 보정 때 쓴 칸 정의 버전(slot_map_sha256)이 지금 칸 정의와 같아야 한다.
        """
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
                # 판정은 기준 영역(바닥부터 앞 턱 바로 위까지)으로 하고, 칸 전체 부피(volume)는 화면에 보이기만 한다.
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
        """설정돼 있으면 카메라 스냅샷에 `slot_zones`를, 검출마다 `slot_assignment`를 덧붙인다(원본은 바꾸지 않고 새 dict).

        used_for_selection은 늘 False다: 여기서는 표시와 세기만 하고, 실제 선택은 select_slot_for_class가 한다.
        """
        zones = self._load(str(camera.get("id")))
        if zones["status"] == "NOT_CONFIGURED":
            return camera
        zones = {**zones, "used_for_selection": False}
        # 보정 때와 해상도나 픽셀 변환이 다르면 다각형 좌표가 맞지 않으므로 쓰지 않는다.
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
        """검출 하나의 칸: 기준점이 칸 하나의 안에만 있으면 IN_SLOT, 여러 칸이나 경계에 걸리면 AMBIGUOUS, 어디에도 없으면
        OUTSIDE, 박스가 없으면 NO_BBOX."""
        box = detection.get("bbox_xyxy_px")
        if not isinstance(box, list) or len(box) != 4:
            return {"status": "NO_BBOX", "used_for_selection": False}
        # 기준점 = 박스 아래쪽 가운데(x 가운데, y 아래 끝), 곧 물체가 선반 바닥에 닿는 곳.
        anchor =((float(box[0]) + float(box[2])) / 2, float(box[3]))
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
