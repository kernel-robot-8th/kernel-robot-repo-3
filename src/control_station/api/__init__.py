"""관제 HTTP 경로 처리 (영역별 모듈).

app.py의 ControlStationHandler가 요청을 받으면 routes.py의 경로 표에서 맞는 항목을 찾아 아래 모듈의 처리 함수를
부른다. 처리 함수는 모두 (handler, route, parts, body) 네 인자를 받고, 실제 일은 서버 객체(handler.app)에 붙은
구성 요소(주문 데스크, 팔 명령 저장소, 핑키 게이트웨이 등)에 맡기는 얇은 HTTP 층이다.

    routes.py   GET/POST 경로 표
    common.py   다른 출처 판별, JSON 본문 읽기, 입력 검증 공통 함수
    arm.py      OMX 팔 명령 큐, 선반 칸, 학습 정책 실행·연속 실행
    cameras.py  카메라 목록, MJPEG 영상
    flow.py     주문 진행 화면, 경보 확인, 모의 세계 조작
    orders.py   주문 창·주문 접수·재고·자동 출발
    pinky.py    핑키 화면 (핑키 관제 대시보드로 전달)
    status.py   서버·로봇 상태, 주차 판정, 전체 정지
    tasks.py    작업 목록·생성·시작·취소, 명령 문장 해석
"""
