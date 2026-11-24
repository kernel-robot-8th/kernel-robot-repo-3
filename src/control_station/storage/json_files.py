"""설정 파일 읽기와 요청 본문 읽기가 함께 쓰는 JSON 읽기: 객체 키 중복 거부, 크기 상한이 있는 일반 파일만 읽기.

설정 파일(팔·정책·수령 구역·핑키·팔 계획값·선반 칸·마커)을 읽는 곳은 load_json_file을, HTTP 요청 본문을 읽는
api/common.py는 unique_keys를 쓴다. 표준 json.loads는 같은 키가 두 번 나오면 조용히 뒤의 값을 쓰므로, 설정이나
요청에 실수로 들어간 중복 키가 다른 값을 덮어도 알아채지 못한다. 여기서는 그런 문서를 아예 거부한다.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class DuplicateJsonKey(ValueError):
    """JSON 객체에 같은 키가 두 번 나온다. json.loads라면 조용히 마지막 값만 남겼을 것이다."""

    def __init__(self, key: str) -> None:
        super().__init__(f"duplicate JSON key: {key}")
        self.key = key


def unique_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """json.loads의 `object_pairs_hook`: 같은 키가 다시 나오면 DuplicateJsonKey로 거부한다."""
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise DuplicateJsonKey(key)
        value[key] = item
    return value


def load_json_file(path: Path, max_bytes: int, label: str, error: type[ValueError]) -> tuple[bytes, Any]:
    """심볼릭 링크가 아닌 일반 파일을 `max_bytes` 이하로 읽어, 키가 겹치지 않는 JSON으로 해석한다.

    (원본 바이트, 해석한 문서)를 돌려준다. 원본 바이트는 호출한 쪽이 해시해 파일 버전으로 쓴다. 실패는 모두
    `error` 예외로 올리고, 메시지에서 파일을 `label`(예: "칸 정의 파일")로 부른다."""
    try:
        # 링크를 따라가 엉뚱한 파일을 읽지 않도록 심볼릭 링크는 없는 파일로 취급한다.
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
