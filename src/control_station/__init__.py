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
"""

from pathlib import Path

# 저장소 루트. config/, data/, .control-station/ 같은 파일 경로는 모두 여기서 시작한다.
REPO_ROOT = Path(__file__).resolve().parents[2]
