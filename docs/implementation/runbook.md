# 실행 방법

## 대시보드와 API만 실행

Python 3.10 이상 환경에서 저장소 의존성을 설치한 뒤 실행합니다. 기본 모드에는 별도 LeRobot 폴더가 필요하지 않습니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
./run-control-station.sh
```

브라우저에서 <http://127.0.0.1:8000>을 엽니다. 기본 모드에서는 로봇 제어를 시작하지 않습니다. 서버를 종료하려면 실행 중인 터미널에서 `Ctrl+C`를 누릅니다.

## 모의 작업 흐름 실행

작업 이력의 자연어 입력에서 `명령 해석`을 누르면 `POST /api/tasks/parse-command`가 제한된 한국어 문형의 초안을 반환하고 구조화 작업 폼에 채웁니다. 지원 예시는 `2번째 선반의 빨간 상자를 집어서 왼쪽 정리함에 넣어줘`, `두 번째 선반의 빨간 상자를 집어서 왼쪽 정리함에 넣어줘`, `파란 병을 찾아 오른쪽 정리함에 넣어줘`, `상자를 오른쪽 정리함으로 옮겨줘`입니다. `상자를 집어서 가방 안에 넣어줘` 형태에서는 `가방`을 목적지 후보로 추출합니다. 한 작업은 물체 한 개를 대상으로 합니다. `빨간 상자 두 개`, `두 상자`, `상자들`과 같은 복수·수량 표현은 자동으로 한 개 요청으로 축약하지 않고 대상 누락/경고를 표시합니다. detector class가 알려져 있으면 문장 안에 쓴 label을 직접 대조하고, 상자/병 한국어 별칭도 해당 class와 연결합니다. Head Camera 또는 MuJoCo fixed camera가 ready YOLO-World인 경우에는 `2번째 선반에 있는 작은 텀블러를 집어서 왼쪽 정리함에 넣어줘`처럼 단일 한국어 대상구를 one-shot prompt 검토 초안으로 채울 수 있습니다. 이 대상구는 문장 해석 결과일 뿐 detector 검출이나 대상을 특정했다는 증거가 아니며, 작업 폼에서 사람이 확인·수정해야 합니다. YOLO-World가 ready가 아니거나 closed-set 모델이면 vocabulary 밖 category는 기존처럼 폼에서 직접 입력해야 합니다. 일부 영어 class/color label은 인식하지만 일반 영어 문장은 지원하지 않습니다. 아라비아 숫자 또는 일부 한국어 서수의 선반 층과 집기 동사 뒤 목적지도 추출합니다. 여러 대상·수량, 대조/부정 표현, destination 선택은 자동 선택하지 않고 누락/경고로 표시합니다. 색상 조건은 지원하는 색상 별칭을 폼에서 참고할 수 있지만 자유 입력도 허용합니다. 현재 색상 비교는 6개 색상 및 지원 한국어 별칭의 미보정 HSV hint에 한정되며 다른 색상은 작업에 보존되더라도 관측 비교에서 지원되지 않는 상태로 표시됩니다. 색상과 층은 각각 색상 hint와 활성 ROI 초안이 있으면 검토용 비교만 수행하며 후보 선택에는 쓰지 않는다고 안내합니다. 이 호출은 task를 만들지 않으므로 값을 확인·수정한 뒤 `작업 등록`을 별도로 눌러야 합니다. 일반 자연어 이해나 작업 실행 기능으로 간주하지 않습니다.

API 확인:

```bash
curl -sS -X POST http://127.0.0.1:8000/api/tasks/parse-command \
  -H 'Content-Type: application/json' \
  -d '{"text":"2번째 선반의 빨간 상자를 집어서 왼쪽 정리함에 넣어줘"}'
```

응답의 `draft.parsed`, `missing_fields`, `ambiguous_fields`, `warnings`를 검토합니다. 초안 응답에는 `task_created: false`, `arm_command_sent: false`가 포함되며 어떤 작업도 저장하지 않습니다.

기본 `mock` 모드에서 작업 이력 화면에 작업을 등록한 뒤 **모의 작업 시작**을 선택합니다. UI는 단계별 상태와 `robot_id`, task/step/command에 연결된 이벤트 및 모의 진행률을 주기적으로 다시 읽습니다. 진행률은 mock 지연 구간을 나타낼 뿐 실제 장치 feedback이 아닙니다. 시작·취소는 mock worker만 사용하며 ROS 2, 카메라, LeRobot 또는 모터에 접속하지 않습니다.

API로도 확인할 수 있습니다.

`POST /api/tasks`는 `destination`, `arm_job`, 선택적 `pick_request`만 받습니다. 그 밖의 최상위 필드는 오타나 미지원 입력으로 보고 HTTP 400을 반환합니다. Pick & Place 대상 조건은 `pick_request: {"category":"box","color":"red","shelf_level":2}`처럼 별도 객체로 보냅니다.

```bash
# 기존 이동·팔 작업 등록 예시
curl -sS -X POST http://127.0.0.1:8000/api/tasks \
  -H 'Content-Type: application/json' \
  -d '{"destination":"작업대 A","arm_job":"상자 이동"}'
# 대상 조건이 있는 Pick & Place 작업 등록 예시
curl -sS -X POST http://127.0.0.1:8000/api/tasks \
  -H 'Content-Type: application/json' \
  -d '{"destination":"작업대 A","arm_job":"상자 이동","pick_request":{"category":"box","color":"red","shelf_level":2}}'
# 둘 중 하나의 응답에서 task.id를 복사해 사용
curl -sS -X POST http://127.0.0.1:8000/api/tasks/TASK-ID/start
curl -sS http://127.0.0.1:8000/api/tasks/TASK-ID
```

`TASK-ID`는 생성 응답의 실제 작업 ID로 바꿉니다. 진행 중인 작업은 `/api/tasks/{task_id}/cancel`로 취소할 수 있습니다. `CONTROL_STATION_MOCK_STEP_DELAY`는 모의 단계별 표시 대기 시간(초)이며 기본은 `0.25`, 허용 범위는 `0.05–2.0`입니다. 시작 접수나 mock 성공은 실물 동작을 의미하지 않습니다.

모의 실패 종결 경로를 화면에서 확인해야 할 때만 서버 시작 환경에 `CONTROL_STATION_MOCK_FAIL_STEP`을 설정합니다. 예를 들어 `CONTROL_STATION_MOCK_FAIL_STEP=GRASP_TARGET ./run-control-station.sh`는 Pick & Place 작업의 해당 step이 `COMMAND_SENT → ACCEPTED → RUNNING`을 거친 뒤 `MOCK_SCENARIO_FAILURE`로 실패하고, 남은 step은 미실행으로 닫힙니다. 지원 값은 저장소의 표준 task step type입니다. 이 설정은 `mock` mode에서만 적용되며 실제 adapter·카메라·로봇 동작을 시험하거나 예측하지 않습니다. 기본값은 비활성입니다.

작업 상세에 “활성 상태지만 이 서버 프로세스에 실행 worker가 없습니다”가 표시되면 `/start`를 반복해 재실행하지 마세요. 해당 값은 현재 프로세스의 worker 소유만 보여 주며 다른 프로세스·장치 동작을 판단하지 않습니다. `.control-station/logs`의 해당 세션 로그와 저장된 task/step/event를 확인하고 운영 절차에 따라 처리합니다. 현재 UI에는 이런 활성 작업을 자동 복구·재생하는 기능이 없습니다.

`pick_request`가 있는 대기 작업에서 관측 버튼을 누르면, 서버가 카메라별 최신 검출 상태를 `/api/tasks/{task_id}/target-observation`으로 기록합니다. 예를 들어 `{"camera_id":"head_camera","save_frame":false}` body는 그 작업의 category를 해당 카메라의 현재 프레임과 대조해 `TARGET_OBSERVED` 이벤트로 남깁니다. 색상 조건은 후보별 미보정 HSV hint 비교를, 선반 층 조건은 명시적으로 활성화된 `{camera_id}.json` ROI draft가 source와 일치할 때 detection `center_px`의 층 영역 비교를 이벤트에 기록합니다. 둘 다 검토용이며 후보를 필터링하거나 자동 선택하지 않습니다. task는 계속 `PENDING`이고 이벤트에는 픽셀 후보 및 `arm_command_sent: false`가 표시됩니다. 화면에서 **원본 RGB 프레임 PNG도 저장**을 선택하거나 API에서 `save_frame: true`를 보내면, 해당 frame sequence와 SHA-256이 최근 캡처 ring에 남아 있는 경우 원본 BGR 픽셀을 lossless PNG로 `.control-station/observations/{task_id}/`에 저장하고 artifact 경로·파일 hash를 이벤트에 기록합니다. 이미지는 기본적으로 저장하지 않습니다. 저장 요청이 늦어 원본 frame이 메모리 ring에서 빠졌거나 provenance가 맞지 않으면 이벤트를 기록하지 않고 오류를 반환합니다. 이 디렉터리는 Git에서 제외됩니다. 이벤트 이력의 **프레임 보기** 링크가 저장된 PNG를 SHA-256 검증 후 제공합니다. 이미지 파일은 실측값이 아니며 3D 위치나 파지 가능 여부는 계산하지 않습니다. 이 기록은 아래처럼 상세 조회로 확인할 수 있습니다.
작업 이력에서 원본 PNG를 저장한 fresh `TARGET_OBSERVED` 이벤트의 후보마다 **선택 기록** 버튼을 눌러 운영자 선택을 남길 수 있습니다. 파일 경로와 SHA-256이 일치하지 않으면 API가 선택 기록을 거부합니다. API 형식은 아래와 같습니다.

task 관측·선택 POST 오류 응답은 `error_code`, `message`, 호환용 `error`를 반환하며 화면에도 오류 코드가 표시됩니다. 카메라별 GET 관측과 이 문서의 다른 API는 아직 동일한 공통 오류 형식으로 보장되지 않습니다.

```bash
curl -sS -X POST http://127.0.0.1:8000/api/tasks/TASK-ID/target-selection \
  -H 'Content-Type: application/json' \
  -d '{"observation_event_id":"EVT-ID","candidate_index":0}'
```

후보 index는 0부터 시작합니다. 서버는 task가 `PENDING`이고 source observation/candidate provenance가 맞는 경우 `TARGET_SELECTED` event를 append하고 응답의 `selection_event_id`로 event ID를 돌려줍니다. 동일한 관측 event와 후보 index의 재요청은 기존 event ID와 `already_recorded: true`를 돌려주며 이 경우 PNG 파일을 다시 읽지 않습니다. 화면도 새 선택 기록인지 기존 기록 재사용인지와 source observation ID를 표시합니다. 이 ID는 저장된 스냅샷의 검토용 좌표 초안 CLI에서 `--selection-event-id`로 참조할 수 있습니다. 선택은 사람의 review 기록만 남기며 후보 품질을 확인하거나 task/step 상태, 자동 대상 선택, 위치 추정, 팔 명령에 연결되지 않습니다.

카메라 화면에서 별도로 확인하려면 클래스와 선택 색상 조건을 넣습니다. 이 GET 요청은 이력을 저장하지 않습니다.

```bash
curl -sS 'http://127.0.0.1:8000/api/cameras/head_camera/target-observation?class_name=box&color=red&shelf_level=2'
```

`shelf_level_evaluation.status`가 `CALIBRATION_NOT_CONFIGURED`이면 활성 영역 파일이 없다는 뜻입니다. 영역 파일이 있어도 camera/frame 출처가 다르면 층 일치 비교를 만들지 않습니다. 활성 영역 파일 구성은 [선반 영역 초안 절차](shelf-image-regions.md)를 참고하세요.

```bash
curl -sS -X POST http://127.0.0.1:8000/api/tasks/TASK-ID/target-observation \
  -H 'Content-Type: application/json' \
  -d '{"camera_id":"head_camera","save_frame":true}'
curl -sS http://127.0.0.1:8000/api/tasks/TASK-ID
```

층별 영상 영역을 측정할 때는 저장된 event ID와 PNG를 확인한 뒤 [선반 영역 초안 절차](shelf-image-regions.md)의 입력 schema로 polygon을 기록하고 `scripts/calibrate_shelf_regions.py`를 실행합니다. editor의 `regions.json` schema v2는 원본 PNG file hash/해상도를 함께 저장하며 CLI는 task event 이미지와 원본 BGR frame hash를 대조합니다. CLI 산출물은 `DRAFT_ONLY`이며 자동 활성화되지 않습니다. 관측 UI/API에서 검토용 층 비교를 보려면 문서 절차에 따라 사람이 artifact를 검토한 뒤 활성 디렉터리에 `{camera_id}.json` 이름으로 명시 배치해야 합니다. 이 배치는 물리 측정의 승인이나 로봇 사용 허가가 아니며 결과는 후보 선택과 팔 제어에 쓰이지 않습니다.

런처로 시작할 때마다 `.control-station/logs/session-<시각>-<프로세스ID>.log`에 관제 서버 출력과 HTTP 요청 로그를 저장하고, 터미널에도 함께 표시합니다. `--arm` 실행의 LeRobot 출력은 같은 세션 ID를 가진 `arm-<시각>-<프로세스ID>.log`에 따로 저장합니다. 이 폴더는 Git에서 제외됩니다.

## 팔 텔레오퍼레이션 제어 켜기

저장소 루트에서 실행합니다. 런처는 현재 셸의 `PATH`를 먼저 확인하고, 없으면 기본 위치 `~/Desktop/lerobot/venv/bin/lerobot-teleoperate`를 자동으로 찾습니다. 관제용 `.venv`가 있으면 그 Python도 자동으로 사용합니다.

```bash
./run-control-station.sh --arm
```

LeRobot이 기본 위치가 아닌 곳에 설치된 경우에만 실행 파일 경로를 지정하면 됩니다.

```bash
CONTROL_STATION_ARM_PROGRAM=/경로/lerobot-teleoperate ./run-control-station.sh --arm
```

그 다음 화면의 **텔레오퍼레이션 시작** 버튼을 누르면 팔 프로세스를 시작합니다. `--arm`은 시작 API를 활성화할 뿐 서버 실행만으로 팔을 움직이지 않습니다. 종료 버튼은 프로세스에 `SIGINT`를 보냅니다.

카메라 캡처와 JPEG 인코딩은 관제 서버가 담당하므로 `opencv-python-headless`도 관제 가상환경에 설치되어야 합니다. `requirements.txt` 설치 후 `--arm`으로 실행하면 됩니다. LeRobot 가상환경에만 OpenCV가 설치된 경우 관제 서버에서는 사용할 수 없습니다.

카메라 worker 시작, 장치 열기/형식 설정 또는 프레임 read/변환/인코딩 처리에서 예외가 나면 해당 카메라 카드가 연결 끊김/오류 상태와 예외 사유를 표시합니다. worker가 실행 중 프레임 처리에 실패하면 약 2초 뒤 재연결을 시도하지만, worker 자체가 시작하지 못한 경우 자동 재시도는 하지 않으므로 원인을 정리한 뒤 관제 프로세스를 다시 시작해야 합니다. YOLO 추론 오류는 detector 오류로 표시하지만 원본 영상 캡처는 계속합니다. 재연결 시도나 서버 생존은 카메라의 현재 준비 상태를 증명하지 않으므로 카드의 연결·스트림·freshness 상태를 따로 확인하세요.

서버가 정상 종료되고 camera worker도 실제로 종료되면 연결 상태는 `DISCONNECTED`, stream 상태는 `STOPPED`로 표시됩니다. 종료 대기 제한 안에 worker가 멈추지 않으면 마지막 관측 상태를 그대로 두어 종료를 완료한 것처럼 표시하지 않습니다.

기본 장치 설정은 follower `/dev/omx_follower`, leader `/dev/omx_leader`입니다. LeRobot은 카메라를 열거나 별도 Rerun 화면을 띄우지 않습니다. `--arm` 모드에서 관제 서버가 Joint Camera `/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0`와 Head Camera `/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0`에서 MJPEG 영상을 읽습니다. Joint camera의 기본 ID 경로는 이전 `/dev/video4` 설정이 가리키던 Innomaker 장치를 따른 것이며 물리 장착 위치의 최종 확인은 아닙니다. 각 경로는 환경 변수 `CONTROL_STATION_JOINT_CAMERA`, `CONTROL_STATION_HEAD_CAMERA`로 바꿀 수 있습니다. 환경 변수 `CONTROL_STATION_FOLLOWER_PORT`, `CONTROL_STATION_LEADER_PORT`로 모터 포트를 바꿀 수 있습니다. 런처에서 시작한 팔 프로세스 로그는 `.control-station/logs/arm-<세션ID>.log`에 저장하며 `CONTROL_STATION_ARM_LOG`로 경로를 지정할 수 있습니다.

장치 번호는 연결 순서에 따라 바뀔 수 있습니다. 2026-10-31 read-only 열거에서는 HCAM0가 `/dev/video4`, Innomaker U20CAM-720P가 `/dev/video6`이었습니다. 기존 구성에서 `/dev/video4`를 Joint Camera로 선택했던 기록에 따라 이제 그 슬롯의 기본 경로를 Innomaker by-id로 고정했고 Head Camera는 HCAM0 by-id를 사용합니다. 단일 frame만으로 물리 장착 위치를 검증한 것은 아니므로 `--arm`을 실행하기 전에 `v4l2-ctl --list-devices`, `/dev/v4l/by-id`, 관제 화면을 통해 사람이 역할을 확인하세요. 환경 변수로 같은 실물 capture node를 두 슬롯에 지정하면 관제는 양쪽 stream을 열지 않고 충돌 오류를 표시합니다.

실제 팔 제어는 운영체제의 장치 권한, LeRobot 의존성 및 사용자별 OMX 지원 코드가 있어야 합니다. 이 구성은 현재 관제 저장소에 포함되지 않습니다. 저장소만 복제한 환경에서는 첫 번째 절차로 대시보드와 API를 실행할 수 있습니다.

## OMX 읽기 전용 상태 조회

LeRobot이 설치된 Python 환경에서 follower의 현재 관절 raw 값, torque enable, hardware error, operating mode, 설정된 register 범위와 전압 raw 값을 읽을 수 있습니다.

```bash
/경로/lerobot/venv/bin/python scripts/probe_omx_readonly.py
```

이 도구의 연결 handshake는 예상 모터 ID/모델 ping을 수행하고, 나머지는 `sync_read`만 사용합니다. 포트를 닫을 때 `disable_torque=False`를 지정합니다. 목표 위치·토크·모드·보정값은 쓰지 않습니다. 관제 텔레오퍼레이션 또는 다른 프로그램이 포트를 사용 중이면 동시에 실행하지 마세요.

출력은 한 시점의 관측값입니다. `readiness_state`는 항상 `UNKNOWN`으로 남으며, torque 비활성화·hardware error 0·USB 응답은 비상정지, 작업공간 clear, 동작 가능 상태 또는 집기 성공을 증명하지 않습니다. register `Min_Position_Limit`/`Max_Position_Limit`은 OMX-F의 확장 위치 모드에서 안전한 선반 작업 한계로 간주하지 않습니다. 읽기 전용 probe를 실물 준비 상태 API 대신 사용하지 않습니다.

## 카메라별 YOLO 검출 표시

Physical Head Camera와 Joint Camera에 각각 별도의 로컬 YOLO 모델과 YOLO-World vocabulary를 opt-in 설정할 수 있습니다. Head model은 `--arm` 없이도 Head Camera 피드와 함께 열립니다. Joint Camera와 그 detector는 `--arm` 모드에서만 열립니다. `mujoco` mode에서는 같은 Head YOLO 설정을 simulation fixed-camera detector에 적용할 수 있고, 모델은 background loader가 준비합니다. 각 feed에 별도 model instance를 로드하므로 동시에 켜면 추가 메모리와 추론 자원이 필요합니다.

```bash
source .venv/bin/activate
python -m pip install -r requirements-vision.txt   # 처음 한 번. PyTorch가 함께 설치되어 용량이 큽니다.
CONTROL_STATION_HEAD_YOLO_WEIGHTS=/경로/best.pt ./run-control-station.sh
```

팔 카메라의 접근 직전 관측에도 detector를 켜려면 별도 가중치 경로를 지정합니다.

```bash
CONTROL_STATION_HEAD_YOLO_WEIGHTS=/경로/shelf-search.pt \
CONTROL_STATION_JOINT_YOLO_WEIGHTS=/경로/wrist-refine.pt \
./run-control-station.sh --arm
```

| 환경 변수 | 기본값 | 내용 |
|---|---|---|
| `CONTROL_STATION_HEAD_YOLO_WEIGHTS` | 없음 (꺼짐) | YOLO 가중치 `.pt` 경로 |
| `CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS` | 없음 | 쉼표 구분 1–100개 text class prompt; 설정하면 같은 Ultralytics 의존성의 `YOLOWorld.set_classes()`를 쓰는 YOLO-World open-vocabulary 검출 모드를 선택 |
| `CONTROL_STATION_HEAD_YOLO_CONF` | `0.25` | 모델이 반환할 최소 confidence threshold. 유한한 `0`–`1` 값이어야 하며, 범위를 벗어나거나 숫자가 아니면 detector 오류로 표시되고 프레임 스트림은 유지됩니다. 검출 confidence는 보정된 확률이 아닙니다. |
| `CONTROL_STATION_HEAD_YOLO_DEVICE` | 자동 | `0`(첫 번째 GPU) 또는 `cpu` |
| `CONTROL_STATION_JOINT_YOLO_WEIGHTS` | 없음 (꺼짐) | `--arm` 모드 Joint Camera 전용 YOLO 가중치 `.pt` 경로 |
| `CONTROL_STATION_JOINT_YOLO_WORLD_PROMPTS` | 없음 | Joint Camera YOLO-World의 고정 prompt 목록; weights도 명시해야 모델이 로드됩니다 |
| `CONTROL_STATION_JOINT_YOLO_CONF` | `0.25` | Joint Camera detector confidence threshold |
| `CONTROL_STATION_JOINT_YOLO_DEVICE` | 자동 | Joint Camera detector device (`0` 또는 `cpu`) |

MuJoCo 고정 카메라에서 task 관측을 수행하려면 simulator와 `mujoco` mode 관제 서버를 실행하면서 Head detector 변수도 함께 전달합니다. 예를 들어 `CONTROL_STATION_MODE=mujoco CONTROL_STATION_MOBILE_DOMAIN=15 CONTROL_STATION_HEAD_YOLO_WEIGHTS=/경로/shelf-search.pt ./run-control-station.sh`처럼 실행합니다. detector가 활성화되어도 task 관측 API가 요청될 때만 해당 raw frame에 추론하며 결과는 review event로 남습니다. 모델 weights가 없으면 detector는 꺼진 상태로 관측되고, 설치된 모델의 정확도나 좌표·파지 성능을 보장하지 않습니다.

YOLO-World는 별도 데이터셋 episode 수집이나 fine-tuning 없이 지정한 text vocabulary에 대한 open-vocabulary 후보를 생성합니다. `requirements-vision.txt`의 Ultralytics 의존성을 재사용합니다. 가중치 파일은 이 저장소에 포함되지 않으며 존재하는 파일 경로를 명시해야 합니다. 상시 카메라 검출의 prompt list는 관제 프로세스 시작 시 고정됩니다. 대기 task에서 **카메라에서 대상 관측**을 명시적으로 요청하면, YOLO-World에서만 설정 vocabulary에 없는 task category를 일회성 prompt로 적용해 정확히 캡처한 raw frame을 추론하고 기존 vocabulary를 복원합니다. 일반 `GET` 관측은 모델 prompt를 변경하지 않으며, closed-set YOLO에는 요청별 prompt 기능이 없습니다. 프롬프트 class 후보는 관측/대상 명령 초안에 제공되지만 정확도 보정, 자동 후보 선택 또는 로봇 제어에 쓰이지 않습니다. 검증된 저장 가중치가 준비되면 다음처럼 opt-in으로 켤 수 있습니다.

```bash
CONTROL_STATION_HEAD_YOLO_WEIGHTS=/경로/yolov8s-worldv2.pt \
CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS='box,bottle,medicine bottle,can' \
./run-control-station.sh
```

Ultralytics는 YOLO-World prompt class를 `set_classes()`로 지정하는 API를 제공하며, 이 모드도 bbox object detection만 제공하고 segmentation은 생성하지 않습니다. [Ultralytics YOLO-World 문서](https://docs.ultralytics.com/models/yolo-world)

각 camera 카드에는 physical feed의 박스가 그려진 영상과 detector의 종류·class·confidence·보조 색상 힌트 및 provenance가 나타납니다. `/api/cameras` 응답은 해당 camera의 전체 weights SHA-256과 YOLO-World prompt 목록을 반환하고, 같은 provenance를 task `TARGET_OBSERVED` event에도 저장합니다. 자연어 명령 초안은 제한 문법과 Head 또는 MuJoCo `FIXED` 선반 탐색 camera detector class를 대조하므로, 지원되지 않는 category는 작업 폼의 대상 종류를 사람이 입력해야 합니다. 그 저장된 `pick_request.category`로 task 관측을 요청하면 MuJoCo fixed feed도 해당 raw frame을 검출하며, YOLO-World camera의 고정 vocabulary에 없는 category는 one-shot prompt로 사용합니다. closed-set camera는 여전히 해당 detector class label과 일치해야 합니다. 각 camera의 관측은 설정된 weights/model provenance로 별도 기록됩니다. Joint Camera 검출은 접근 직전 후보 관측에만 표시되며 자동 선택, 재인식 성공 판단 또는 팔 동작으로 연결되지 않습니다. device가 `null`이면 backend 자동 선택 요청이며 실제 선택 장치를 기록한 값은 아닙니다. SHA-256은 모델 파일 경로를 노출하지 않고 사용한 파일 내용을 식별합니다. 색상 힌트는 detection bbox 안쪽 crop의 미보정 HSV pixel fraction이며 확률이 아니고 자동 필터/선택에 쓰지 않습니다. 상시 camera worker의 가중치 로드 또는 프레임 추론 오류는 해당 서버 프로세스에서 자동 재시도하지 않고 detector 상태를 오류로 유지합니다. task별 one-shot YOLO-World prompt 추론 실패는 해당 관측 이벤트에 기록하며, 기본 vocabulary 복원까지 실패한 경우에는 detector 상태도 오류로 전환합니다. 설정이나 가중치를 수정한 뒤 오류 상태 detector를 다시 시작하려면 관제 프로세스를 재시작하세요. 이 동안 camera raw frame은 계속 표시됩니다.

검출 결과는 표시 전용입니다. 로봇 명령이나 작업 상태 판단에 쓰지 않습니다. 카메라는 한 프로그램만 열 수 있으므로 다른 카메라 프로그램을 먼저 종료합니다.

## ROS 2 Nav2 검색 모드

Nav2 action server discovery만 확인할 때 사용합니다. 이동 목표는 보내지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source /경로/ros2_워크스페이스/install/setup.bash
CONTROL_STATION_MODE=ros2 CONTROL_STATION_MOBILE_DOMAIN=15 ./run-control-station.sh
```

domain을 지정하지 않으면 `ROS_DOMAIN_ID`, 없으면 `0`을 사용합니다. 관제 설정은 ROS 2 domain ID `0`–`232`만 허용합니다.

`/api/robots`는 ROS 2 모드에서 Nav2 action-server 발견을 표시합니다. `CONTROL_STATION_ARM_TRAJECTORY_ACTION_NAME`을 설정하면 해당 `control_msgs/action/FollowJointTrajectory` endpoint의 server 발견 여부도 확인합니다. 이는 discovery-only이며 goal을 보내지 않고 controller readiness나 안전을 증명하지 않습니다. OMX 관절 상태를 함께 보려면 `CONTROL_STATION_ARM_JOINT_STATES_TOPIC`에 ROS graph에서 출처를 확인한 절대 토픽 이름을 지정합니다. 설정 전에는 팔 관절 topic을 구독하지 않습니다. 저장소 inventory에서 `/joint_states`가 주행 시뮬레이터 모델에도 사용되는 것을 확인했으므로, 토픽 이름만으로 OMX source를 판정하지 마세요. 관제는 지정 topic의 `sensor_msgs/msg/JointState`를 읽기만 하며 joint 이름·원본 position 숫자·원본 ROS stamp/frame ID·관제 수신 시각과 freshness를 표시합니다. 메시지에는 joint type이 없어 position 단위를 미확정으로 표시합니다. 배열 길이 불일치, 빈/중복 이름, 비유한 position은 `invalid`로 표시하고 position 배열을 응답에서 제외합니다. 기본 freshness 한계는 2초이고 `CONTROL_STATION_ARM_JOINT_STATES_MAX_AGE`로 조절할 수 있습니다. 원본 ROS stamp는 clock 기준을 확인하지 않으므로 카메라 시각과 비교하거나 TF 변환에 사용할 수 없습니다.

MuJoCo 상태를 관제에서 읽으려면 시뮬레이터와 관제 서버를 동일한, 실물 로봇 및 다른 simulator와 분리한 `ROS_DOMAIN_ID`로 실행합니다. 예를 들어 domain `15`를 운영자가 비워 둔 것을 확인한 경우:

```bash
# 터미널 A: MuJoCo 시뮬레이션만 실행
source /opt/ros/jazzy/setup.bash
cd simulation/ros2_ws
ROS_DOMAIN_ID=15 ./run_mujoco_sim.sh

# 터미널 B: 같은 domain에서 상태만 관측
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
CONTROL_STATION_MODE=mujoco \
CONTROL_STATION_MOBILE_DOMAIN=15 \
./run-control-station.sh
```

ROS mode를 처음 실행하기 전 `.venv/bin/python -m pip install -r requirements-ros.txt`로 선택적 PyYAML 의존성을 설치합니다. `rclpy`가 이 Python 모듈을 필요로 하며, system `dist-packages` 전체를 venv 앞에 두면 OpenCV/Numpy 버전이 섞일 수 있어 사용하지 않습니다. ROS setup과 workspace overlay source는 계속 필요합니다. ROS 연동이 필요 없는 mock 모드는 이 선택 의존성을 요구하지 않습니다.

domain 번호는 예시이며, 실제 graph와 실물 제어 환경에 쓰이지 않는지 먼저 확인합니다. `mujoco` mode는 domain 1–232를 요구하며 simulator package에 고정된 joint-state topic과 trajectory action만 read-only 관측합니다. 이 mode는 Pinky/Nav2 discovery, LeRobot 팔 텔레오퍼레이션, OpenCV 실물 camera open을 하지 않습니다. `--arm`은 이 mode에서 거부됩니다. 이 검사는 실제 domain 격리를 증명하지 않습니다. MuJoCo launch는 노드/controller 이름을 `/mujoco_omx` namespace에 두지만 `ROS_DOMAIN_ID`를 직접 설정하지 않습니다. 관제는 action-server discovery와 `JointState` subscription만 사용하며 action goal, service request, joint command를 보내지 않습니다. action server 발견 또는 토픽 수신은 controller/팔 준비, 안전, 명령 수락 가능 또는 Pick & Place 성공을 뜻하지 않습니다. system prefix에는 MuJoCo ROS plugin이 없지만, 2026-10-31에 필요한 apt package를 `/tmp` 임시 overlay로 추출한 격리 domain `231`에서 launch와 fixed RGB feed 및 관제 API를 runtime 확인했습니다. 이 확인은 system prefix 설치나 장시간 안정성을 뜻하지 않습니다.

2026-10-31에 격리 domain `231`과 임시 ROS binary overlay에서 이 절차로 MuJoCo camera manager 및 관제 서버를 실행해 `/api/health`, `/api/cameras`, fixed-camera task 관측 경로를 확인했습니다. `requirements-ros.txt`의 PyYAML을 프로젝트 `.venv`에 설치한 상태에서 frame sequence `2`, source frame `placeholder_fixed_camera_optical_frame`, clock 정렬 미확인 source stamp 상태가 기록됐습니다. detector weights가 없어 결과는 `DETECTOR_NOT_CONFIGURED`였고, task는 `PENDING`으로 남았으며 `arm_command_sent`는 `false`였습니다. 실제 물체 검출·3D 위치 추정·팔 명령은 수행하지 않았습니다. 이 확인은 해당 임시 overlay와 호스트의 ROS Python 경로에 한정되며, 기본 설치 절차나 장시간 안정성을 증명하지 않습니다.

graph 목록만 확인할 때는 작업 서버를 실행하지 않고 다음 read-only discovery 명령을 사용합니다. node/topic daemon 응답이 오래되었거나 XML-RPC 오류가 나면 `--no-daemon`을 붙여 현재 graph를 직접 조회합니다.

```bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
ros2 action list -t
ros2 service list -t
```

이 명령은 인터페이스 이름과 타입을 확인할 뿐 action goal이나 service request를 보내지 않습니다. 확인한 domain, source한 workspace overlay, 시각을 함께 기록하며 `/rosout`·`/parameter_events`만 관측됐다고 다른 domain이나 overlay에 장치가 없다고 결론내리지 않습니다.
