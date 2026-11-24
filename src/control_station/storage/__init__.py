"""저장: 작업·단계·이벤트를 담는 SQLite 저장소(SQLAlchemy), 상태 어휘, 설정 JSON 읽기.

- database.py: 테이블 정의(ORM 모델)와 엔진·세션 만들기
- store.py: TaskStore. 작업·주문 생성, 상태 전이, 이벤트 기록, 핑키·수령 구역 배정
- models.py: API와 어댑터가 함께 쓰는 상태 이름(TaskStatus, StepStatus, 카메라 상태)
- json_files.py: 설정 파일과 요청 본문을 읽는 안전한 JSON 읽기
스키마 변경은 저장소 최상위 migrations/(Alembic)에 마이그레이션으로 남긴다.
"""
