"""Persist unexecutable Cartesian target drafts for operator review."""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping


_FRAME_ID = re.compile(r"^[A-Za-z0-9_./-]{1,80}$")


class CoordinateDraftError(ValueError):
    """Raised when a coordinate draft does not match the input contract."""


class CoordinateTargetDraftStore:
    """Save coordinate requests as drafts; this class has no robot connection."""

    def __init__(self, directory: str | Path = ".control-station/coordinate-target-drafts") -> None:
        self.directory = Path(directory)
        self._lock = threading.Lock()

    @staticmethod
    def _validate(payload: Mapping[str, object]) -> tuple[str, str, dict[str, float]]:
        if set(payload) != {"frame_id", "unit", "position"}:
            raise CoordinateDraftError(
                "request must contain exactly frame_id, unit, and position"
            )
        frame_id = payload.get("frame_id")
        unit = payload.get("unit")
        if not isinstance(frame_id, str) or not _FRAME_ID.fullmatch(frame_id):
            raise CoordinateDraftError("frame_id must be 1 to 80 letters, numbers, or _./- characters")
        if not isinstance(unit, str) or unit not in {"m", "mm"}:
            raise CoordinateDraftError("unit must be 'm' or 'mm'")

        raw_position = payload.get("position")
        if not isinstance(raw_position, Mapping):
            raise CoordinateDraftError("position must be an object with x, y, and z")
        if set(raw_position) != {"x", "y", "z"}:
            raise CoordinateDraftError("position must contain exactly x, y, and z")

        position: dict[str, float] = {}
        for axis in ("x", "y", "z"):
            value = raw_position[axis]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise CoordinateDraftError(f"position.{axis} must be a number")
            try:
                number = float(value)
            except (OverflowError, ValueError) as exc:
                raise CoordinateDraftError(f"position.{axis} must be finite") from exc
            if not math.isfinite(number):
                raise CoordinateDraftError(f"position.{axis} must be finite")
            position[axis] = number
        return frame_id, unit, position

    def create(self, payload: Mapping[str, object]) -> dict[str, object]:
        frame_id, unit, position = self._validate(payload)
        draft_id = str(uuid.uuid4())
        draft = {
            "schema_version": 1,
            "id": draft_id,
            "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "status": "DRAFT_ONLY",
            "arm_command_sent": False,
            "frame_id": frame_id,
            "unit": unit,
            "position": position,
        }

        self.directory.mkdir(parents=True, exist_ok=True)
        target = self.directory / f"{draft_id}.json"
        temporary_path: Path | None = None
        with self._lock:
            try:
                with tempfile.NamedTemporaryFile(
                    mode="w",
                    encoding="utf-8",
                    dir=self.directory,
                    prefix=f".{draft_id}.",
                    suffix=".tmp",
                    delete=False,
                ) as temporary_file:
                    json.dump(draft, temporary_file, ensure_ascii=False, indent=2)
                    temporary_file.write("\n")
                    temporary_path = Path(temporary_file.name)
                os.replace(temporary_path, target)
            finally:
                if temporary_path is not None and temporary_path.exists():
                    temporary_path.unlink()

        draft["file"] = str(target)
        return draft
