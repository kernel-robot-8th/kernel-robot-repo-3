"""로봇별 IR 주행 보정값(프로파일)을 원자적으로 저장하는 로컬 관제용 저장소다.

실행 위치: PC 관제(app.py가 import)
사용처: app.py의 IR_PROFILES (data/ir_profiles.json). 주행준비 때 해당 로봇의 프로파일이
runtime_bundle로 로봇의 nav2/ir_profile.json에 실리고, 로봇의 ultrasonic_safety.py가 읽어
line_following.py의 IR 조향 보조에 쓴다.

핵심: 저장된 값이 없거나 일부 키만 있으면 line_following.default_profile()로 빈 칸을 채운다.
저장은 임시 파일에 쓴 뒤 os.replace로 바꿔 끼우므로, 도중에 죽어도 파일이 반쯤 쓰인 채 남지 않는다.
"""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any

from .line_following import default_profile


class IRProfiles:
    """로봇 ID → IR 프로파일(dict) 저장소. 메모리에 들고 있다가 save 때 파일 전체를 다시 쓴다."""
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
        """프로파일의 깊은 복사본을 돌려준다. JSON 왕복이라 NaN·inf가 섞여 있으면 여기서 ValueError가 난다."""
        return json.loads(json.dumps(self.profiles[robot], allow_nan=False))

    def save(self, robot: str, profile: dict[str, Any]) -> None:
        """파일 쓰기가 성공한 뒤에만 메모리 값을 바꾼다. 실패하면 이전 값이 그대로 남는다."""
        candidate = {**self.profiles, robot: profile}
        payload = json.dumps(candidate, ensure_ascii=False, allow_nan=False, indent=2)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=self.path.parent, delete=False) as stream:
            stream.write(payload)
            temporary = stream.name
        os.replace(temporary, self.path)
        self.profiles[robot] = profile
