"""관제 서버 패키지.

주문을 받아 로봇팔과 핑키(이동 로봇)에게 일을 나눠 주고, 단계마다 카메라 관측으로 결과를 확인한다.

    app.py        서버 조립과 요청 분배. 실행: python -m control_station.app
    api/          HTTP 경로별 처리 (경로 표는 api/routes.py)
    orders/       주문 접수·배정·단계 실행·진행 화면
    arms/         로봇팔 설정·명령 큐·집기·학습 정책
    cameras/      카메라 feed·YOLO 검출·개수 세기·선반 칸
    pinky/        핑키 관제 연결·적재 자리 도착 판정·주차 판정 노드 연결
    storage/      SQLite 저장소·상태 어휘·설정 JSON 읽기
    mock_world.py 로봇 없이 돌리는 모의 세계
    stop_all.py   팔과 핑키에 한 번에 정지 요청
    web/          관제 화면과 손님 주문 창 (정적 파일)

이 파일 자체는 패키지 전체가 함께 쓰는 작은 도구 세 가지만 둔다: 저장소 루트 경로(REPO_ROOT),
UTC 시각 문자열(utc_now_iso), 켜기/끄기 환경 변수 읽기(env_flag).
"""

import os
from datetime import datetime, timezone
from pathlib import Path

# 저장소 루트. config/, data/, .control-station/ 같은 파일 경로는 모두 여기서 시작한다.
# 이 파일은 <루트>/src/control_station/__init__.py이므로 parents[2]가 루트다(실행 위치와 무관하게 같은 경로).
REPO_ROOT = Path(__file__).resolve().parents[2]


def utc_now_iso(timespec: str = "seconds") -> str:
    """지금 시각을 UTC ISO 8601 문자열로. timespec은 "seconds", "milliseconds", "microseconds" 중 하나.

    작업·단계·이벤트 기록은 같은 초 안의 순서까지 남기려고 "microseconds"를 쓴다.
    """
    return datetime.now(timezone.utc).isoformat(timespec=timespec)


def env_flag(name: str) -> bool:
    """환경 변수가 정확히 "1"일 때만 켜짐으로 본다. 비어 있거나 다른 값("true", "yes" 포함)이면 꺼짐이다.

    실제 팔을 움직이는 기능처럼 일부러 켜야 하는 설정(CONTROL_STATION_ORDER_REAL_ARM 등)에 쓴다.
    오타나 애매한 값으로 켜지는 일이 없도록 "1"만 받는다.
    """
    return os.environ.get(name, "").strip() == "1"
