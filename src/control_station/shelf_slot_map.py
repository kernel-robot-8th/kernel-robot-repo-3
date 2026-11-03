"""Load the shelf slot map (config/shelf-slots.json) that names the slots an arm command may target.

The file is produced by scripts/derive_shelf_slots.py from the team's 3D shelf models. Its SHA-256 is
the slot map version that every slot command carries, so the arm worker can refuse a command made
against a different slot map than the one its taught poses belong to.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DEFAULT_SLOT_MAP_PATH = Path(__file__).resolve().parents[2] / "config" / "shelf-slots.json"
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


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ShelfSlotMapError(f"duplicate JSON key: {key}")
        value[key] = item
    return value


def load_shelf_slot_map(path: str | Path | None = None) -> ShelfSlotMap:
    slot_path = Path(path or os.environ.get("CONTROL_STATION_SHELF_SLOTS") or DEFAULT_SLOT_MAP_PATH)
    try:
        if slot_path.is_symlink() or not slot_path.is_file():
            raise ShelfSlotMapError(f"칸 정의 파일이 없습니다: {slot_path}")
        raw = slot_path.read_bytes()
    except OSError as exc:
        raise ShelfSlotMapError(f"칸 정의 파일을 읽지 못했습니다: {exc}") from exc
    if len(raw) > MAX_SLOT_MAP_BYTES:
        raise ShelfSlotMapError(f"칸 정의 파일이 {MAX_SLOT_MAP_BYTES} bytes보다 큽니다")
    try:
        document = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ShelfSlotMapError(f"칸 정의 파일이 올바른 JSON이 아닙니다: {exc}") from exc
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
