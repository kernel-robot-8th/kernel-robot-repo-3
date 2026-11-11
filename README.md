# 로봇 관제시스템 MVP

주행로봇과 로봇팔을 위한 Python 관제 서버입니다. 저장소만 복제한 환경에서는 모의 상태 화면과 API를 실행할 수 있습니다. 실제 팔 제어에는 별도의 LeRobot 및 OMX 지원 환경이 필요합니다.


저장된 카메라 관측 이미지의 선반 층 ROI polygon 입력은 `scripts/shelf_region_editor.html`을 사용하고, provenance 검증은 [선반 영상 영역 절차](docs/implementation/shelf-image-regions.md)를 따릅니다.

## 시작 전 환경 확인

새 PC에서 처음 작업하거나 새로 합류했다면, 먼저 필요한 것이 준비됐는지 확인합니다. 옵션 없이 실행하면 확인만 하고 아무것도 바꾸지 않습니다.

```bash
scripts/check_setup.sh                               # 확인만 (✓ 준비됨 · ✗ 필수 항목 없음 · △ 선택 항목 없음)
scripts/check_setup.sh --install                     # 빠진 apt 패키지(sudo 비밀번호 필요)와 .venv 의존성 설치
scripts/check_setup.sh --vision                      # YOLO(ultralytics, PyTorch)와 팀 모델 준비. 용량이 큼
scripts/check_setup.sh --install --vision --dry-run  # 실행할 명령만 보기
```

| 확인 영역 | 확인하는 것 | 필요한 때 |
|---|---|---|
| 관제 서버 (필수) | Python 3.10 이상, `.venv`, SQLAlchemy·Alembic, OpenCV 4.x | 항상 |
| 카메라 | `v4l2-ctl`, 연결된 카메라의 by-path 목록, 장치 권한 | 실물 카메라를 쓸 때 |
| Pinky 주차 판정 | 천장 카메라 내부 보정 파일, 등록된 주차 위치, 창을 띄우는 OpenCV | 주차 판정을 쓸 때 |
| YOLO / GPU | GPU, ultralytics, PyTorch CUDA, 팀 YOLO 모델 | 카메라에 YOLO를 띄울 때 |
| ROS 2 Jazzy | ROS 2, colcon, 카메라 드라이버(`usb_cam`, `v4l2_camera`), `image_transport`·`cv_bridge`·`vision_msgs` 등 | ROS 모드, C++ 노드 |
| C++ 빌드 | cmake, OpenCV·nlohmann-json·OpenSSL·GTest 개발 패키지 | `cpp/parking_vision` 빌드 |

필수 항목이 빠지면 종료 코드 1로 끝나고, 빠진 항목마다 설치 명령을 보여 줍니다.

- `--vision`을 쓰면 ultralytics가 `opencv-python` 5.x를 함께 설치해 `cv2`를 덮어씁니다. 이 저장소는 `opencv-python-headless` 4.x를 쓰므로 스크립트가 설치 뒤 4.x로 되돌립니다.
- 팀 YOLO 모델은 `feat/yolo-object-detection`의 2회차 모델을 Git에서 꺼내 `.control-station/models/`에 둡니다. 이 폴더는 Git에서 제외됩니다.
- ROS 2 Jazzy 자체 설치, 카메라 내부 보정, 주차 위치 등록은 스크립트가 하지 않습니다. 각각 [ROS 2 설치 안내](https://docs.ros.org/en/jazzy/Installation.html), [카메라 내부 보정](docs/implementation/camera-intrinsic-calibration.md), [실행 방법](docs/implementation/runbook.md#천장-카메라-pinky-주차-판정)을 따릅니다.

## 빠른 실행

처음 실행할 때 저장소 의존성을 설치합니다. `python3 -m venv`에서 pip이 없다는 오류가 나면 `scripts/check_setup.sh --install`을 쓰세요. uv가 있으면 uv로 가상환경을 만듭니다.

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
- [에피소드 없는 Pick & Place 에픽](docs/epics/epic-07-episode-free-shelf-picking.md) — 단계별 범위와 갭
- [개발 로드맵](docs/development-plan.md) — 에픽별 목적과 진행 순서
- [원본 제어 흐름 요구사항](robot-control-flow.md)
