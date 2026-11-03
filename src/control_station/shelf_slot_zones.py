"""Display-only shelf slot zones on the front camera image and the slot each detection falls in.

The zones come from a shelf pose artifact (scripts/calibrate_shelf_pose.py) that an operator activates
by copying it to `<CONTROL_STATION_SHELF_POSE_DIR>/<camera_id>.json`. A zone is used only when the
artifact matches the live camera (ID, resolution, pixel transform) and the loaded slot map version.
A detection's anchor is its bbox bottom-center, where an object meets the deck; it is matched against
each slot's anchor zone (deck up to just above the front lip), not the whole slot volume, so an
object on the upper deck cannot land in the lower slot below it. Results are for
display and review: they do not select targets or create arm commands.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import threading
from pathlib import Path
from typing import Any

from control_station.shelf_regions import _point_in_polygon
from control_station.shelf_slot_map import ShelfSlotMap


DEFAULT_POSE_DIR = Path(__file__).resolve().parents[2] / ".control-station" / "shelf-pose-calibrations"
MAX_POSE_BYTES = 1_000_000
_CAMERA_ID = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON key: {key}")
        value[key] = item
    return value


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
            artifact = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object)
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
                slots.append(
                    {
                        "slot_id": slot["slot_id"],
                        "label": self.slot_map.active_slots[slot["slot_id"]]["label"],
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
            "camera_position_in_shelf_frame_m": artifact.get("pose", {}).get("camera_position_in_shelf_frame_m"),
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
            result.update(status="IN_SLOT", slot_id=inside[0]["slot_id"], label=inside[0]["label"])
        elif inside or edge:
            result["status"] = "AMBIGUOUS"
        else:
            result["status"] = "OUTSIDE"
        return result
