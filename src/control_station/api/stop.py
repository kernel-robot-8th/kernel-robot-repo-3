"""전체 정지 API"""

from __future__ import annotations

from typing import TYPE_CHECKING

from control_station.stop_all import stop_everything

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def post_stop_all(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """전체 정지 요청: 팔 명령·시퀀스·정책 실행을 취소하고 핑키마다 정지를 보낸 뒤 결과를 모두 돌려줌.
    202 = 보낸 요청이 모두 접수됨, 207 = 일부가 거절됐거나 보내지 못함(모의 모드 포함). 접수는 멈춤 확인이 아님."""
    result = stop_everything(h.app)
    h._json(202 if result["ok"] else 207, result)
