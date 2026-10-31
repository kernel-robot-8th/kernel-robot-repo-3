# 로봇 관제시스템 MVP

주행로봇과 로봇팔을 위한 Python 관제 서버입니다. 저장소만 복제한 환경에서는 모의 상태 화면과 API를 실행할 수 있습니다. 실제 팔 제어에는 별도의 LeRobot 및 OMX 지원 환경이 필요합니다.

현재 관제 기능을 보존하면서 에피소드 없는 선반 Pick & Place로 확장 중입니다. 제한된 한국어 문형과 일부 영어 물체/색상어를 사람이 검토할 작업 폼 초안으로 해석하고, 구조화된 대상 요청과 카메라 후보 관측 이력을 저장할 수 있습니다. mock 작업 흐름, fresh task observation을 고정 선반 평면 2D 초안으로 투영하는 CLI, MuJoCo 전용 관절 목표 및 position-only IK의 이산 관절 보간 접촉 검사도 있습니다. 카메라 기반 3D 위치 추정, 실제 orientation·연속 경로 충돌을 고려한 grasp/motion 계획, 접촉 파지와 실제 자동 제어는 아직 구현되지 않았습니다. 상세 진행은 [구현 현황](docs/implementation/current-state.md)과 [개발 로드맵](docs/development-plan.md)을 참고하세요.

저장된 카메라 관측 이미지의 선반 층 ROI polygon 입력은 `scripts/shelf_region_editor.html`을 사용하고, provenance 검증은 [선반 영상 영역 절차](docs/implementation/shelf-image-regions.md)를 따릅니다.

## 빠른 실행

처음 실행할 때 저장소 의존성을 설치합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
./run-control-station.sh
```

브라우저에서 <http://127.0.0.1:8000>을 엽니다. 상세 절차는 [실행 방법](docs/implementation/runbook.md)을 참고하세요.

## 문서

- [개발 원칙과 에이전트 지침](AGENTS.md)
- [문서 목차](docs/README.md)
- [시스템 명세](docs/spec/system-spec.md) — 완성 목표와 필수 제어 규칙
- [목표 데이터 흐름](docs/spec/data-flow.md) — 목표 시스템 구성과 이벤트 흐름
- [현재 구현 현황](docs/implementation/current-state.md) — 코드가 현재 제공하는 API와 한계
- [ROS 2 인터페이스 조사](docs/implementation/ros2-inventory.md) — 정적 workspace 설정과 읽기 전용 runtime graph 관측
- [MuJoCo 설정 절차](docs/implementation/simulation-setup.md) — ROS 의존성, simulation-only launch 및 관제 read-only 상태 확인
- [MuJoCo OMX 시뮬레이터 빠른 시작](simulation/mujoco_omx/README.md) — OMX 모델 GUI, 관절/그리퍼 제어, RGB 카메라 캡처
- [MuJoCo OMX 기술 기록](docs/implementation/mujoco-omx.md) — 모델 출처·IK/관측 실험과 검증 한계
- [에피소드 없는 Pick & Place 에픽](docs/epics/epic-07-episode-free-shelf-picking.md) — 단계별 범위와 갭
- [개발 로드맵](docs/development-plan.md) — 에픽별 목적과 진행 순서
- [원본 제어 흐름 요구사항](robot-control-flow.md)
