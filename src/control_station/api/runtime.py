"""실행 구성요소(핑키 관제·천장 캠 주차·정책 서버·팔 워커) 상태와 켜기"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from control_station import runtime_services

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_runtime(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """구성요소마다 켜짐·꺼짐과 화면에서 켤 수 있는지"""
    workers = h.app.arm_commands.workers()
    h._json(200, {"services": runtime_services.status(os.environ, workers)})


def post_start(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """POST /api/runtime/{id}/start: 꺼진 핑키 관제나 천장 캠 주차 launch를 켬 (202 = 켜는 중, 409 = 이미 켜짐)"""
    try:
        result = runtime_services.start(parts[2])
    except runtime_services.RuntimeStartError as error:
        h._json_error(error.status, error.code, error.message)
        return
    h._json(202, result)
