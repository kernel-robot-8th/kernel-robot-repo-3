"""Atomic per-robot IR calibration storage for the local dashboard."""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any

from .line_following import default_profile


class IRProfiles:
    def __init__(self, path: Path, robots: list[str]) -> None:
        self.path = path
        self.profiles = {rid: default_profile() for rid in robots}
        if path.exists():
            saved = json.loads(path.read_text(encoding='utf-8'))
            if not isinstance(saved, dict):
                raise ValueError('IR calibration file must be an object')
            for rid in robots:
                if isinstance(saved.get(rid), dict):
                    self.profiles[rid] = {**default_profile(), **saved[rid]}

    def get(self, robot: str) -> dict[str, Any]:
        return json.loads(json.dumps(self.profiles[robot], allow_nan=False))

    def save(self, robot: str, profile: dict[str, Any]) -> None:
        candidate = {**self.profiles, robot: profile}
        payload = json.dumps(candidate, ensure_ascii=False, allow_nan=False, indent=2)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=self.path.parent, delete=False) as stream:
            stream.write(payload)
            temporary = stream.name
        os.replace(temporary, self.path)
        self.profiles[robot] = profile
