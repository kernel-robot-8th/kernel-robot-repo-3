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

### Front Camera 화면

Front Camera는 학습 정책 client의 `front` 입력과 같은 카메라입니다. `CONTROL_STATION_FRONT_CAMERA`를 지정하면 관제 화면에 Front Camera 카드가 추가되고, 지정하지 않으면 `CONTROL_STATION_POLICY_FRONT_CAMERA`를 같은 장치로 사용합니다. 둘 다 없으면 카드에 경로 미설정을 표시하고 장치를 열지 않습니다. `CONTROL_STATION_FRONT_YOLO_WEIGHTS`를 지정하면 Head Camera와 같은 방식으로 YOLO 검출을 표시하고, 지정하지 않으면 영상만 표시합니다(아래 "카메라별 YOLO 검출 표시" 참고).

Head Camera와 Front Camera가 같은 기종(HCAM0)이면 `/dev/v4l/by-id` 경로가 둘 중 하나만 가리키고, 다시 꽂을 때마다 가리키는 장치가 바뀔 수 있습니다. 2026-11-10 이 노트북에서는 USB 포트 1이 Head, 포트 2가 Front였으므로 포트 기준 경로로 지정합니다. 포트를 바꿔 꽂으면 경로도 바뀌므로 화면으로 역할을 다시 확인하세요.

```bash
CONTROL_STATION_HEAD_CAMERA=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0 \
CONTROL_STATION_FRONT_CAMERA=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0 \
./run-control-station.sh
```

카메라는 기본으로 640×480 MJPEG를 요청합니다. 센서가 4:3 모드에서 시야를 잘라내는 경우 `CONTROL_STATION_HEAD_CAMERA_RESOLUTION=1280x720`처럼 카메라별(`HEAD`, `JOINT`, `FRONT`)로 다른 모드를 요청할 수 있습니다. 해상도를 바꾸면 검출의 픽셀 좌표 기준도 그 해상도로 바뀌고, 처리량이 늘어 프레임률이 낮아질 수 있습니다. 실제 시야가 넓어지는지는 카메라마다 화면으로 확인해야 합니다.

V4L2 장치는 한 프로세스만 영상을 받을 수 있어, 관제가 Front Camera를 열고 있으면 정책 client가 같은 장치를 열지 못합니다. 그래서 정책 시작 요청을 받으면 관제가 Front Camera capture를 먼저 닫고, 닫힌 것을 확인한 뒤 정책 client를 시작합니다. 정책 프로세스가 살아 있는 동안 카드는 `연결 끊김`/`중지`와 사용 중 안내를 표시하고, 프로세스가 끝나면 자동으로 다시 엽니다. 5초 안에 닫힘을 확인하지 못하면 정책을 시작하지 않고 409 오류를 반환합니다. 이 해제는 Front Camera만 대상으로 하며, 정책의 `wrist` 입력과 Joint Camera가 같은 장치인 경우는 처리하지 않습니다.

실제 팔 제어는 운영체제의 장치 권한, LeRobot 의존성 및 사용자별 OMX 지원 코드가 있어야 합니다. 이 구성은 현재 관제 저장소에 포함되지 않습니다. 저장소만 복제한 환경에서는 첫 번째 절차로 대시보드와 API를 실행할 수 있습니다.

## 학습 정책 한 번 실행

LeRobot async inference 정책 서버를 별도 터미널에서 먼저 실행합니다. 제어 서버의 정책 실행 버튼은 async client만 시작하며 정책 서버를 자동으로 띄우지 않습니다.

```bash
cd ~/Desktop/lerobot
source venv/bin/activate
python -m lerobot.async_inference.policy_server --host=127.0.0.1 --port=8080 \
  --fps=30 --inference_latency=0.033 --obs_queue_timeout=1
```

LeRobot client에 `--max_action_steps` 옵션이 포함되도록 `~/Desktop/lerobot/src/lerobot` checkout을 사용합니다. 관제 서버는 `opencv`와 `omx_follower` config를 client보다 먼저 import합니다. 정책 프로필은 저장소의 `config/policy_profiles.json`에서 읽습니다. 현재 **사과 집기**는 모델 `neverlish/omx_act_record-test-20261104-1`과 학습 당시 task 문자열 `Pick up Doll`을 사용합니다. 표시 이름과 학습 조건 문자열은 서로 다른 용도입니다. 서버를 다시 시작하지 않고 프로필 항목을 추가하려면 `CONTROL_STATION_POLICY_PROFILES`로 지정한 JSON 파일을 편집한 뒤 서버를 재시작하세요. 각 항목은 고유한 `id`, 화면 표시용 `name`, LeRobot `policy_type`, `model`, 학습 task 문자열 `task`를 가집니다. 프로필 선택 UI와 API는 이 목록을 읽어 실행하므로 코드 변경 없이 추가 정책을 등록할 수 있습니다. 관제 서버를 시작할 때 정책 실행을 활성화하고, 한 번의 시연에 맞는 총 액션 상한을 지정합니다.

```bash
cd /경로/kernel-robot-repo-3
CONTROL_STATION_POLICY_ENABLED=1 \
CONTROL_STATION_POLICY_MAX_ACTION_STEPS=300 \
./run-control-station.sh
```

`300`은 설정 예시입니다. 실제 실행 전에 시연 한 회의 길이에 맞게 양의 정수로 바꾸세요. 이는 LeRobot이 로봇에 보낸 총 action 수이며 `actions_per_chunk=100`과는 별개입니다. 기본 프로필은 `neverlish/omx_act_record-test-20261104-1`, task `Pick up Doll`, follower `/dev/omx_follower`, 카메라 `/dev/video7` 및 `/dev/video9`, ACT/CUDA, 30 FPS, chunk 100, threshold 0.7입니다. 설정은 `CONTROL_STATION_POLICY_*` 환경변수로 바꿀 수 있습니다. Python은 기본 `~/Desktop/lerobot/venv/bin/python`을 사용하며 `CONTROL_STATION_LEROBOT_DIR`와 `CONTROL_STATION_LEROBOT_PYTHON`으로 checkout/환경 경로를 지정할 수 있습니다.

관제의 **학습 정책 한 번 실행** 버튼 또는 `POST /api/arm/policy/start?profile_id=apple_pick`가 클라이언트를 시작합니다. 응답과 상태 조회의 `run_id`로 실행을 식별할 수 있습니다. client는 설정된 총 액션 수를 보내면 종료하고 관제 상태를 `LIMIT_REACHED`로 표시합니다. 이는 제한에 도달했다는 뜻이며 물체를 잡거나 놓았다는 결과가 아닙니다. **중단 요청** 또는 `POST /api/arm/policy/stop`은 실행을 멈추도록 요청하고 결과를 `INTERRUPTED`/미확정으로 표시합니다. 텔레오퍼레이션과 정책 client는 동시에 시작할 수 없습니다. 실행 로그는 `.control-station/logs/policy-run-<run_id>.log`에 남으므로 해당 로그와 카메라/실물 상태를 확인하세요. 관제 UI의 프로세스 상태만으로 팔 준비나 Pick & Place 성공을 판단하지 않습니다.

## OMX 읽기 전용 상태 조회

LeRobot이 설치된 Python 환경에서 follower의 현재 관절 raw 값, torque enable, hardware error, operating mode, 설정된 register 범위와 전압 raw 값을 읽을 수 있습니다.

```bash
/경로/lerobot/venv/bin/python scripts/probe_omx_readonly.py
```

이 도구의 연결 handshake는 예상 모터 ID/모델 ping을 수행하고, 나머지는 `sync_read`만 사용합니다. 포트를 닫을 때 `disable_torque=False`를 지정합니다. 목표 위치·토크·모드·보정값은 쓰지 않습니다. 관제 텔레오퍼레이션 또는 다른 프로그램이 포트를 사용 중이면 동시에 실행하지 마세요.

출력은 한 시점의 관측값입니다. `readiness_state`는 항상 `UNKNOWN`으로 남으며, torque 비활성화·hardware error 0·USB 응답은 비상정지, 작업공간 clear, 동작 가능 상태 또는 집기 성공을 증명하지 않습니다. register `Min_Position_Limit`/`Max_Position_Limit`은 OMX-F의 확장 위치 모드에서 안전한 선반 작업 한계로 간주하지 않습니다. 읽기 전용 probe를 실물 준비 상태 API 대신 사용하지 않습니다.

## 카메라별 YOLO 검출 표시


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
| `CONTROL_STATION_FRONT_YOLO_WEIGHTS` | 없음 (꺼짐) | Front Camera 전용 YOLO 가중치 `.pt` 경로. 학습 정책 실행 중에는 카메라와 함께 검출도 멈춥니다. |
| `CONTROL_STATION_FRONT_YOLO_WORLD_PROMPTS` | 없음 | Front Camera YOLO-World의 고정 prompt 목록; weights도 명시해야 모델이 로드됩니다 |
| `CONTROL_STATION_FRONT_YOLO_CONF` | `0.25` | Front Camera detector confidence threshold |
| `CONTROL_STATION_FRONT_YOLO_DEVICE` | 자동 | Front Camera detector device (`0` 또는 `cpu`) |


YOLO-World는 별도 데이터셋 episode 수집이나 fine-tuning 없이 지정한 text vocabulary에 대한 open-vocabulary 후보를 생성합니다. `requirements-vision.txt`의 Ultralytics 의존성을 재사용합니다. 가중치 파일은 이 저장소에 포함되지 않으며 존재하는 파일 경로를 명시해야 합니다. 상시 카메라 검출의 prompt list는 관제 프로세스 시작 시 고정됩니다. 대기 task에서 **카메라에서 대상 관측**을 명시적으로 요청하면, YOLO-World에서만 설정 vocabulary에 없는 task category를 일회성 prompt로 적용해 정확히 캡처한 raw frame을 추론하고 기존 vocabulary를 복원합니다. 일반 `GET` 관측은 모델 prompt를 변경하지 않으며, closed-set YOLO에는 요청별 prompt 기능이 없습니다. 프롬프트 class 후보는 관측/대상 명령 초안에 제공되지만 정확도 보정, 자동 후보 선택 또는 로봇 제어에 쓰이지 않습니다. 검증된 저장 가중치가 준비되면 다음처럼 opt-in으로 켤 수 있습니다.

```bash
CONTROL_STATION_HEAD_YOLO_WEIGHTS=/경로/yolov8s-worldv2.pt \
CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS='box,bottle,medicine bottle,can' \
./run-control-station.sh
```

Ultralytics는 YOLO-World prompt class를 `set_classes()`로 지정하는 API를 제공하며, 이 모드도 bbox object detection만 제공하고 segmentation은 생성하지 않습니다. [Ultralytics YOLO-World 문서](https://docs.ultralytics.com/models/yolo-world)


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


```bash
source /opt/ros/jazzy/setup.bash
cd simulation/ros2_ws

# 터미널 B: 같은 domain에서 상태만 관측
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
CONTROL_STATION_MOBILE_DOMAIN=15 \
./run-control-station.sh
```

ROS mode를 처음 실행하기 전 `.venv/bin/python -m pip install -r requirements-ros.txt`로 선택적 PyYAML 의존성을 설치합니다. `rclpy`가 이 Python 모듈을 필요로 하며, system `dist-packages` 전체를 venv 앞에 두면 OpenCV/Numpy 버전이 섞일 수 있어 사용하지 않습니다. ROS setup과 workspace overlay source는 계속 필요합니다. ROS 연동이 필요 없는 mock 모드는 이 선택 의존성을 요구하지 않습니다.



graph 목록만 확인할 때는 작업 서버를 실행하지 않고 다음 read-only discovery 명령을 사용합니다. node/topic daemon 응답이 오래되었거나 XML-RPC 오류가 나면 `--no-daemon`을 붙여 현재 graph를 직접 조회합니다.

```bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
ros2 action list -t
ros2 service list -t
```

이 명령은 인터페이스 이름과 타입을 확인할 뿐 action goal이나 service request를 보내지 않습니다. 확인한 domain, source한 workspace overlay, 시각을 함께 기록하며 `/rosout`·`/parameter_events`만 관측됐다고 다른 domain이나 overlay에 장치가 없다고 결론내리지 않습니다.
