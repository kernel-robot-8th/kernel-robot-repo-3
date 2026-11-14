"""JSON reading shared by the config loaders and the request reader: unique object keys, bounded regular files."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class DuplicateJsonKey(ValueError):
    """A JSON object names the same member twice; json.loads would silently keep the last one."""

    def __init__(self, key: str) -> None:
        super().__init__(f"duplicate JSON key: {key}")
        self.key = key


def unique_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """`object_pairs_hook` for json.loads that refuses a repeated member name."""
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise DuplicateJsonKey(key)
        value[key] = item
    return value


def load_json_file(path: Path, max_bytes: int, label: str, error: type[ValueError]) -> tuple[bytes, Any]:
    """Read a regular non-symlink file of at most `max_bytes` and parse it as JSON with unique keys.

    Returns the raw bytes (callers hash them as the file version) and the parsed document. Every failure raises
    `error` with a message that names the file by `label` (e.g. "칸 정의 파일")."""
    try:
        if path.is_symlink() or not path.is_file():
            raise error(f"{label}이 없습니다: {path}")
        raw = path.read_bytes()
    except OSError as exc:
        raise error(f"{label}을 읽지 못했습니다: {exc}") from exc
    if len(raw) > max_bytes:
        raise error(f"{label}이 {max_bytes} bytes보다 큽니다")
    try:
        document = json.loads(raw.decode("utf-8"), object_pairs_hook=unique_keys)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise error(f"{label}이 올바른 JSON이 아닙니다: {exc}") from exc
    except DuplicateJsonKey as exc:
        raise error(str(exc)) from exc
    return raw, document
