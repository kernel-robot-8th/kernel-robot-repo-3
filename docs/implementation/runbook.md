# 실행 방법

## 대시보드와 API만 실행

처음이라면 `scripts/check_setup.sh`로 필요한 것이 준비됐는지 먼저 확인합니다(사용법은 [README](../../README.md#시작-전-환경-확인)). Python 3.10 이상 환경에서 저장소 의존성을 설치한 뒤 실행합니다. 기본 모드에는 별도 LeRobot 폴더가 필요하지 않습니다.

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

이동·팔 작업의 `VERIFY_LOAD`(적재 확인) 흐름은 `CONTROL_STATION_MOCK_LOAD_OBSERVED`로 확인합니다. 값은 수레 위에서 "보였다고 칠" 물건 개수(0 이상 정수)입니다. 기대 수량은 작업당 1개로 고정돼 있습니다. 예를 들어 `CONTROL_STATION_MOCK_LOAD_OBSERVED=0 ./run-control-station.sh`로 띄운 뒤 작업을 실행하면 `LOAD_OBSERVED`(덜 실림)가 남고 `LOAD_INCOMPLETE`로 실패하며 `FINALIZE_TASK`는 실행하지 않습니다. `1`이면 `LOAD_CONFIRMED`, `2` 이상이면 `LOAD_EXCESS`입니다. 이 관측은 카메라를 열지 않고 설정값을 그대로 보고하는 모의 관측(`simulated: true`)이며, 실제 적재 여부를 확인한 것이 아닙니다. 설정하지 않으면 `VERIFY_LOAD`는 다른 단계처럼 모의 성공입니다. `mock` mode에서만 적용됩니다.

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

## 선반 카메라 표시

선반과 팔을 앞에서 보는 선반 카메라(정면 카메라)를 관제 화면에 함께 띄우려면 `CONTROL_STATION_SHELF_CAMERA`에 장치 경로를 지정합니다. feed ID는 내부 보정 파일 `shelf_camera.json`과 같은 `shelf_camera`입니다. 2026-11-02 기준 이 카메라는 USB 포트 10의 HCAM0입니다. 천장 카메라와 모델이 같아 by-id로는 구분되지 않으므로 by-path를 씁니다.

```bash
CONTROL_STATION_HEAD_CAMERA=/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0 \
CONTROL_STATION_SHELF_CAMERA=/dev/v4l/by-path/pci-0000:80:14.0-usb-0:10:1.0-video-index0 \
CONTROL_STATION_PARKING_CAMERA=head_camera CONTROL_STATION_PARKING_SPOT=pinky_dock_1 \
./run-control-station.sh
```

선반 카메라는 `shelf_camera`(`SHELF`) 카드로 영상과 연결 상태를 보여 줍니다. `CONTROL_STATION_SHELF_YOLO_WEIGHTS`(또는 `CONTROL_STATION_SHELF_YOLO_WORLD_PROMPTS`)를 주면 Head Camera와 같은 방식으로 YOLO 검출을 표시합니다. `_CONF`, `_DEVICE` 설정도 `CONTROL_STATION_SHELF_YOLO_` 접두어로 같습니다. 작업 관측 단계에는 연결하지 않습니다.

2026-11-07 기준 두 카메라 모두 팀 기준 모델을 씁니다. `feat/yolo-object-detection` 브랜치의 `2413f4c` "10클래스 YOLO26n 모델 best11.pt"(`yolo/models/best11.pt`, SHA-256 `c6850be0…`)입니다. 모델 파일은 Git에서 제외되는 `.control-station/models/best11_2413f4c.pt`에 두고, `CONTROL_STATION_HEAD_YOLO_WEIGHTS`와 `CONTROL_STATION_SHELF_YOLO_WEIGHTS`에 같은 경로를 지정합니다. `scripts/check_setup.sh --vision`이 이 파일을 준비합니다. 이전 기준 모델은 `e252f4e`의 3클래스 모델(`strawberry_car_ball_r2_e252f4e.pt`)이었습니다.

클래스는 strawberry, car, ball, pumpkin, apple, carrot, fruits(열매 송이), potato, watermelon, tissue(물티슈) 10종입니다. 2026-11-07에 천장 카메라 프레임 한 장으로 확인했을 때, best11은 위에서 본 Pinky 본체를 `car`(0.93)로 잘못 검출했고, 선반에 놓인 과일·장난감 10여 개는 하나도 검출하지 못했습니다(이전 모델은 2~3개를 낮은 신뢰도로 검출했고 일부는 틀림). 위에서 내려다본 사진으로 학습했는지는 확인하지 않았습니다. 천장 카메라 검출은 지금 표시용으로만 보고, 적재 개수 판단에 쓰지 않습니다. 선반 카메라는 이때 사람이 시야를 가려서 비교하지 못했습니다. 관제 가상환경에 `requirements-vision.txt`를 설치하면 ultralytics가 `opencv-python`을 함께 설치해 `cv2`가 OpenCV 5.0으로 바뀝니다. 이 저장소는 `opencv-python-headless<5`를 쓰므로, 설치 뒤 `opencv-python`을 지우고 `opencv-python-headless`를 다시 설치했습니다. 2026-11-02에 이전 모델로 이 PC(RTX 5080, PyTorch 2.15 CUDA 13.2)에서 두 카메라 모두 YOLO를 켠 채 29.7 fps를 유지했습니다. best11의 프레임당 추론 시간은 같은 GPU에서 약 7 ms로 이전 모델(7~10 ms)과 비슷했고, 관제에 올린 상태의 fps는 아직 재지 않았습니다. 그때 Head는 strawberry·ball을 검출했고, 선반 카메라는 이 시점에 검출이 없었습니다(모델이 선반 정면 시점으로 학습됐는지 미확인). 다른 카메라와 같은 장치로 지정하면 선반 카메라만 열지 않고 오류를 표시합니다. 2026-11-02에 천장·선반 카메라를 함께 열어 두 카드 모두 640×480, 29.7 fps로 수신되는 것을 확인했습니다. Joint Camera(손목 카메라)는 별도 장치이며 `--arm` 모드에서만 엽니다.

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

## C++ 주차 판정 노드로 관제 실행

관제의 주차 판정과 Head Camera 영상을 C++ ROS 2 노드 쪽으로 옮겨 실행하는 방법입니다([전환 계획](../plans/pinky-parking-ros2-node.md)). 천장 카메라는 드라이버 컨테이너가 열고, 관제는 영상과 판정을 ROS로 받습니다. 판정 규칙과 API 응답 형식은 아래 Python 판정과 같습니다.

```bash
cd cpp && source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && cd ..
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 로봇들의 ROS graph와 분리
# 1) 카메라 드라이버 + C++ 판정 노드 (관제가 Head Camera를 열고 있으면 먼저 끕니다)
source cpp/install/setup.bash
ros2 launch parking_vision_ros ceiling_parking.launch.py repository_root:=$PWD spot_id:=pinky_dock_1
# 2) 다른 터미널에서 관제 (같은 ROS_DOMAIN_ID, ROS_AUTOMATIC_DISCOVERY_RANGE)
CONTROL_STATION_PARKING_SPOT=pinky_dock_1 \
CONTROL_STATION_HEAD_CAMERA=ros:/ceiling_camera/image_raw/compressed \
CONTROL_STATION_SHELF_CAMERA=/dev/v4l/by-path/pci-0000:80:14.0-usb-0:10:1.0-video-index0 ./run-control-station.sh
```

- 천장 카메라 경로는 `video_device:=...`, 없으면 환경 변수 `CONTROL_STATION_CEILING_CAMERA`, 없으면 기본값(포트 4)을 씁니다. HCAM0 두 대는 by-id와 일련번호가 같아 USB 포트(by-path)로만 구분되므로, 포트 구성이 다른 PC는 Git에서 제외되는 `.control-station/control-station.env`에 `CONTROL_STATION_CEILING_CAMERA`를 두고 두 터미널에서 `set -a; source .control-station/control-station.env; set +a`를 먼저 실행합니다. 등록 CLI `pinky_parking`도 같은 변수를 씁니다.
- `run-control-station.sh`는 위 설정이 있으면 ROS와 `cpp/install/setup.bash`를 source합니다. `/api/health`의 `parking.source`가 `ros`이면 판정 노드에 연결된 구성입니다.
- Head Camera는 `/compressed` 토픽으로 받습니다. 같은 PC라도 raw 640×480 영상을 프로세스 사이로 보내면 대부분 전달되지 않았습니다(2026-11-02).
- 판정 노드가 꺼지면 웹 주차 패널은 `NODE_UNAVAILABLE`, 확정 판정과 `VERIFY_ARRIVAL`은 `PARKING_NODE_UNAVAILABLE`로 팔 단계를 시작하지 않습니다.
- 관제에는 주차 판정이 없습니다. `CONTROL_STATION_PARKING_SPOT`을 주지 않으면 주차 판정 없이 뜨고 `VERIFY_ARRIVAL`은 모의 성공입니다.
- 2026-11-03 같은 영상에 두 판정을 동시에 30초 돌린 비교, `usb_cam` 대신 자체 캡처 노드를 쓰는 이유는 [cpp/README.md](../../cpp/README.md)에 있습니다.

## 천장 카메라 Pinky 주차 판정

천장 카메라로 Pinky 윗면의 ArUco 마커를 찾고, 미리 등록한 정답 위치와 비교해 주차 상태를 판정합니다. 관제 서버와 연결되지 않은 C++ CLI `pinky_parking`(`cpp/parking_vision`)입니다. 2026-11-03에 Python `scripts/pinky_parking.py`를 같은 인자·파일 형식·종료 코드로 옮기고 Python은 지웠습니다. Pinky나 로봇팔에 명령을 보내지 않으며, 결과는 `DRAFT_ONLY` 관측 초안입니다.

준비할 것:

- 천장 카메라 내부 보정 파일 `.control-station/camera-calibrations/ceiling_camera.json`. `camera_id`가 `ceiling_camera`이고 `pixel_transform_id`가 `opencv-v4l2-capture-no-app-geometry-v1`이어야 합니다. 만드는 방법은 [카메라 내부 보정](camera-intrinsic-calibration.md)을 따릅니다. 캡처 해상도가 보정 해상도(현재 640×480)와 다르면 판정하지 않고 종료합니다.
- Pinky 윗면의 ArUco 마커. 기본 사전은 `DICT_4X4_50`입니다. 마커 한 변 길이는 인쇄물의 검은 테두리 기준으로 mm 단위로 잽니다.
- (선택) 화면에 함께 보이는 바닥 고정 ArUco 마커. 이 마커로 카메라 시야가 등록할 때와 같은지 확인합니다. 쓰려면 바닥에 테이프로 고정하고, Pinky가 지나가거나 서는 자리와 사람이 가리는 자리에서 떨어뜨려 둡니다. 놓아만 둔 카드나 Pinky 바퀴 옆 마커는 밀려서 판정이 멈춥니다. 쓰지 않으면 `--no-anchors`로 등록합니다.
- `cpp/`를 빌드하고 `source cpp/install/setup.bash`한 셸. `check --once`와 `register --headless`는 창을 띄우지 않고, 창이 있는 `register`와 실시간 `check`는 시스템 OpenCV(highgui) 창을 씁니다.
- 카메라 경로. `--camera` 또는 환경 변수 `CONTROL_STATION_CEILING_CAMERA`로 지정합니다. 천장 카메라는 관제의 Head Camera와 같은 실물 카메라(HCAM0)입니다. 2026-11-02 현장에서는 USB 포트 4에 연결됐고, 이름은 Head Camera 기본 by-id(`usb-Jieli_Technology_USB_Composite_Device-video-index0`)로 잡혔습니다. 같은 모델이 여러 대 꽂혀 있으면 by-id로는 구분되지 않으므로 포트 기준 `/dev/v4l/by-path/...` 경로를 씁니다. 실제로 19:36에 다른 HCAM0이 USB 포트 10에 연결되자 by-id 링크가 그 카메라(선반을 옆에서 보는 시점)로 바뀌었습니다. by-path는 꽂은 USB 포트가 바뀌면 달라집니다.
- 카메라는 한 프로그램만 열 수 있습니다. 관제 서버가 Head Camera로 이 카메라를 열고 있으면 서버를 먼저 종료합니다. 점유 여부는 `fuser -v /dev/videoN`으로 확인합니다.

### 정답 위치 등록

```bash
ros2 run parking_vision pinky_parking register --spot-id pinky_dock_1 \
  --camera /dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0 \
  --marker-id 21 --marker-side-mm 70 --marker-forward-axis -x
```

`--marker-forward-axis`는 Pinky 앞쪽을 향하는 마커 축입니다. `+x`는 마커 오른쪽 변, `+y`는 윗변이 앞쪽이라는 뜻이고, 마커 면이 위를 향한다고 가정합니다. 주면 판정 결과에 Pinky 기준 앞·왼쪽 차이가 함께 나옵니다. 생략할 수 있습니다.

Pinky를 정답 위치에 세운 뒤 창에서 Space를 누르면 다음 순서로 등록합니다. `--headless`를 주면 창 없이 바로 모읍니다.

1. 15프레임(`--frames`)을 모읍니다. 모서리 좌표는 프레임별 중앙값을 씁니다.
2. 이 모서리로 마커 평면을 정합니다(OpenCV IPPE).
3. 모으는 동안 위치가 5 mm 넘게, 또는 방향이 3° 넘게 흔들렸으면 저장하지 않고 다시 Space를 누르게 합니다.
4. 시야 확인용 고정 마커를 고릅니다. Pinky 마커가 아니고, 모든 프레임에 한 번씩 보이며, 1 px 이상 흔들리지 않은 마커만 씁니다. `--anchor-ids 1,20`처럼 직접 지정할 수도 있고, 지정한 마커가 조건에 맞지 않으면 등록을 거부합니다. 고정 마커가 하나도 없으면 등록하지 않습니다. `--no-anchors`를 주면 이 단계를 건너뛰고 `view_check.mode: OFF`로 저장합니다.

허용 오차 기본값은 30 mm / 5°이며 `--tolerance-mm`, `--tolerance-deg`로 바꿉니다. 등록 화면 왼쪽 가장자리에 세로 이음선이 보이면 영상이 밀린 상태이므로, 창을 닫고 다시 실행합니다(아래 "카메라 영상 밀림" 참고).

결과는 `.control-station/parking-spots/<spot-id>.json`에 schema v2로 저장되며 Git에서 제외됩니다. 같은 이름의 파일이 있으면 덮어쓰지 않고 종료하므로, 다시 등록하려면 기존 파일을 직접 다른 곳으로 옮깁니다. 파일에는 다음이 기록됩니다.

- 기준 모서리 픽셀과 마커 평면 자세
- 고정 마커 픽셀 위치와 시야 확인 방식(`view_check.mode`: `FIXED_MARKERS` 또는 `OFF`)
- 마커 ID·크기
- 내부 보정 파일 경로와 SHA-256
- 허용 오차
- `usable_for_robot_control: false`

schema v1 파일(2026-10-31 이전 등록)은 고정 마커 정보가 없어 판정하지 않으며 다시 등록하라고 안내합니다.

### 판정

```bash
# 실시간 창. 흰 외곽선은 등록 위치, 흰 원은 고정 마커의 등록 위치입니다. q로 종료합니다.
ros2 run parking_vision pinky_parking check --spot-id pinky_dock_1 \
  --camera /dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0

# 창 없이 한 번 판정하고 JSON 한 줄을 출력합니다.
ros2 run parking_vision pinky_parking check --spot-id pinky_dock_1 \
  --camera /dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0 --once
```

프레임마다 다음 순서로 확인합니다.

1. 고정 마커를 등록했으면 고정 마커들의 픽셀 이동 중앙값을 구합니다(`--no-anchors` 등록이면 1·2단계를 건너뜁니다). 등록 파일의 `view_check.max_anchor_shift_px`(기본 15 px)를 넘으면 시야가 바뀐 것으로 보고 판정을 멈춥니다. 보이는 고정 마커가 없는 프레임은 Pinky 마커를 놓친 프레임처럼 건너뜁니다.
2. 그 이하의 이동은 카메라 거치대가 조금 흐른 것으로 보고, 그만큼 Pinky 마커 좌표에서 뺍니다. 단, 보이는 고정 마커끼리 이동량이 3 px 넘게 다르면 하나가 밀린 것으로 보고 판정을 멈춥니다. 고정 마커가 하나만 보이면 카메라 흐름과 밀린 마커를 구분할 수 없으므로 3 px까지만 빼고, 그보다 크면 판정을 멈춥니다(2026-11-03 추가). 거치대 흐름을 넓게 보정하려면 서로 떨어진 고정 마커를 두 개 이상 등록합니다. 마커가 하나뿐이어도 바닥에 고정돼 있음을 운영자가 확인했다면 등록 때 `--trust-single-anchor`(등록 파일 `view_check.single_anchor_trusted: true`)를 주어 기준(기본 15 px)까지 보정합니다. `pinky_dock_1`의 1번 마커는 나무판에 고정돼 있다는 확인을 받아 이 표시를 넣었습니다. 2026-11-03 실제 프레임(카메라 흐름 6.6 px)에서 표시가 있으면 정상 판정, 없으면 `VIEW_UNVERIFIED`였습니다. 처음에는 기준이 3 px였는데, 2026-11-02 21:12에 정상 주차 상태에서 거치대가 약 5 px 흘러(1번 마커·Pinky 마커·선반이 같은 방향으로 (−2, +5) px) 판정이 막혔습니다. 그래서 기본값을 15 px로 올렸습니다. 영상 밀림(약 96 px)은 여전히 막습니다. 넓힌 뒤 같은 장면은 고정 마커 이동 5.7 px를 보정해 정상 위치(앞 −0.6 mm, 왼쪽 +0.4 mm)로 판정했습니다.
3. Pinky 마커의 네 모서리 광선을 등록된 마커 평면과 교차시켜 평면 위 위치와 방향을 구합니다. 프레임마다 PnP로 깊이를 다시 추정하지 않습니다.
4. 평면 위에서 잰 마커 한 변이 인쇄 크기와 10 % 넘게 다르면, 마커가 등록 평면 위에 있지 않은 것으로 보고 그 프레임을 쓰지 않습니다.

판정은 최근 15프레임 오프셋의 중앙값으로 합니다. 등록 파일에 기록된 내부 보정 SHA-256이 지금 파일과 다르면 판정하지 않고 재등록을 요구합니다. `--once`는 15프레임이 모이거나 `--timeout`(기본 3초)이 지나면, 그때까지 모인 프레임으로 판정하고 사용한 프레임 수를 `frames`에 출력합니다. `--tolerance-mm`, `--tolerance-deg`를 주면 등록 파일의 허용 오차 대신 이번 판정에만 그 값을 씁니다.

| 상태 | 뜻 | `--once` 종료 코드 |
| --- | --- | --- |
| `ARRIVED` | 위치·방향 오차가 모두 허용 범위 안 | 0 |
| `MISALIGNED` | 마커는 보이지만 위치 또는 방향이 허용 범위 밖 | 1 |
| `NOT_FOUND` | 판정에 쓸 프레임이 없음. 마커가 안 보이거나 등록 평면 밖으로 측정되는 상태가 15프레임(`--frames`) 연속되면 이전 값을 버리고 이 상태가 됨 | 2 |
| `AMBIGUOUS` | 같은 ID 마커가 화면에 둘 이상 보여 어느 쪽이 Pinky인지 모름 | 2 |
| `VIEW_UNVERIFIED` | 고정 마커로 등록한 경우에만 나옴. 카메라 시야가 등록 때와 같은지 확인하지 못함. 고정 마커가 등록 위치에서 기준(기본 15 px) 넘게 움직였거나, 고정 마커가 안 보이는 프레임만 이어져 판정할 값이 없음 | 2 |

`AMBIGUOUS` 프레임이나 고정 마커가 기준보다 많이 움직인 프레임이 나오면 이전 프레임 값도 버립니다. 출력 JSON의 `view_check`는 시야 확인 여부(`ON`/`OFF`)이고, `view_shift_px`는 고정 마커 이동량입니다. `view_check`가 `OFF`인 판정은 영상 밀림이나 카메라 이동을 확인하지 않은 결과입니다. 등록 파일 오류나 카메라 열기 실패처럼 판정을 시작하지 못한 경우에도 종료 코드는 2이고, 사유는 표준 오류로 출력합니다.

오프셋은 등록된 마커 축을 기준으로 합니다. `dx`는 마커 오른쪽 변 방향, `dy`는 윗변 방향이 +이고, `dyaw`는 천장 카메라에서 볼 때 반시계 방향이 +입니다. 위치 판정에는 평면 거리 `distance_mm`를 씁니다. 등록할 때 `--marker-forward-axis`를 줬으면 Pinky 기준 `forward_mm`(앞 +)와 `left_mm`(왼쪽 +)도 출력합니다. Pinky 기준으로도 `dyaw`가 +이면 왼쪽(반시계)으로 돌아간 것입니다.

어긋난 정도는 허용 오차 대비 비율로도 출력합니다. `position_ratio`는 `distance_mm / 허용 위치 오차`, `yaw_ratio`는 `|dyaw| / 허용 방향 오차`입니다. 두 값 중 큰 값 r로 `deviation_level`을 정합니다.

| `deviation_level` | 기준 | 뜻 |
| --- | --- | --- |
| `WITHIN` | r ≤ 1 | 허용 범위 안. `ARRIVED`와 같음 |
| `NEAR` | 1 < r ≤ 2 | 조금 벗어남 |
| `FAR` | r > 2 | 많이 벗어남. 다른 위치에 섰거나 이동이 잘못됐을 수 있음 |

경계 2는 초안 값입니다. 판정 불가 상태에서는 오프셋, 비율, 등급을 출력하지 않습니다.

### 관제 API로 판정

관제 서버를 켤 때 `CONTROL_STATION_PARKING_CAMERA`에 판정에 쓸 feed ID를 지정하면, 관제가 그 실물 카메라를 열고 주차 판정 API를 제공합니다. 천장 카메라는 Head Camera와 같은 카메라이므로 `head_camera`를 쓰고, Head Camera 경로는 포트 기준 by-path로 지정합니다. 2026-11-02 기준 Head Camera 기본 by-id 경로는 포트 10에 연결된 다른 HCAM0을 가리킵니다.

```bash
CONTROL_STATION_HEAD_CAMERA=/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0 \
CONTROL_STATION_PARKING_CAMERA=head_camera \
./run-control-station.sh

curl -s http://127.0.0.1:8000/api/parking-spots/pinky_dock_1/check
```

- 요청을 받은 뒤에 찍힌 프레임만 최대 3초 동안 15프레임까지 모아 CLI와 같은 방식으로 판정합니다. 판정에 쓴 프레임이 5장보다 적으면 `NOT_FOUND`입니다.
- 응답은 `{"observation": {...}}`이며, 필드는 [주차 정렬 관측 계약 초안](../spec/interface-contracts.md#pinky-주차-정렬-관측-초안)을 따릅니다. 오류는 `error_code`/`message`/`error`로 반환합니다.
- 이 API 호출은 판정 결과를 작업 이력에 남기지 않고, 로봇 명령도 만들지 않습니다.
- 관제가 카메라를 열고 있는 동안에는 CLI로 같은 카메라를 열 수 없습니다.
- 코드의 기본 카메라 경로(by-id)는 바꾸지 않았습니다. 실습 PC의 포트 설정(포트 4 천장, 포트 10 선반)과 YOLO 모델 경로는 PC마다 다르므로 Git에서 제외되는 `.control-station/control-station.env`에 두고 `set -a; source .control-station/control-station.env; set +a; ./run-control-station.sh`로 실행합니다. 이 설정으로 띄운 화면은 [관제 화면 캡처](../images/control-station-overview-2026-11-02.png)에 있습니다.
- 관제 가상환경의 `opencv-python-headless`와 CLI로 등록할 때 쓴 GUI OpenCV의 버전이 다르면 모서리 검출이 조금 달라집니다. 2026-11-02 같은 화면에서 OpenCV 4.6(등록)과 4.14(판정) 사이에 약 1 mm·0.2°·고정 마커 0.8 px 차이가 났습니다. 응답의 `opencv_version`과 `registration_opencv_version`으로 확인합니다.

2026-11-02에 mock 모드, 임시 DB, 포트 8765로 관제를 띄워 실제 천장 카메라로 확인했습니다. `/api/cameras`의 `head_camera`는 640×480, 29.7 fps였습니다. 판정 API를 5회 호출했고 모두 약 0.5초 걸렸습니다. 결과는 모두 `ARRIVED`/`WITHIN`(앞 0.4 mm·왼쪽 1.5 mm·yaw 0.23° 이내)이었고, 15프레임 모두 요청 시각 이후 프레임이었으며, 호출 간 프레임 sequence는 겹치지 않았습니다. 없는 등록 ID는 404, 형식이 틀린 ID는 400으로 응답했습니다. 이때 밀림은 일어나지 않았습니다.

### 작업 흐름의 도착 확인 단계

`CONTROL_STATION_PARKING_SPOT`에 주차 위치 ID를 더 지정하면, 모의 작업의 `VERIFY_ARRIVAL` 단계가 위 API와 같은 방식으로 판정합니다. 판정 결과는 그 단계의 `PARKING_OBSERVED` event로 작업 이력에 남고 화면의 이벤트 목록에 표시됩니다.

```bash
CONTROL_STATION_HEAD_CAMERA=/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0 \
CONTROL_STATION_PARKING_CAMERA=head_camera \
CONTROL_STATION_PARKING_SPOT=pinky_dock_1 \
./run-control-station.sh
```

| 판정 | 단계·작업 | 오류 코드 |
| --- | --- | --- |
| `ARRIVED` | `VERIFY_ARRIVAL` 성공 → 팔 단계 진행 | 없음 |
| `MISALIGNED` | `FAILED`, 팔 단계 미실행 | `PARKING_MISALIGNED` |
| `NOT_FOUND`/`AMBIGUOUS`/`VIEW_UNVERIFIED` | `FAILED`, 팔 단계 미실행 | `PARKING_NOT_VERIFIED` |
| 판정 시작 실패(등록 파일·카메라 오류) | `FAILED`, 팔 단계 미실행, `PARKING_OBSERVED` 없음 | 관측 오류 코드(예: `PARKING_CAMERA_UNAVAILABLE`) |

자동 재주행이나 재판정은 하지 않습니다. 주차 위치만 지정하고 `CONTROL_STATION_PARKING_CAMERA`를 지정하지 않으면 서버가 시작을 거부합니다. 이동 단계는 아직 모의이므로, 판정은 Pinky가 실제로 서 있는 위치를 봅니다.

2026-11-02 실제 천장 카메라로 확인했습니다. `pinky_dock_1`로 작업을 실행하면 `ARRIVED`(1.1 mm, 0.24°, 15프레임, 시야 확인 ON)로 모든 단계가 성공했습니다. 허용 오차를 0.1 mm/0.01°로 줄인 임시 등록으로 실행하면 `MISALIGNED`/`FAR`로 `VERIFY_ARRIVAL`이 실패하고, 팔 작업 이후 단계는 `NOT_EXECUTED_AFTER_FAILURE`로 끝났습니다.

관제 화면 **현황** 탭 아래쪽의 **Pinky 도착 확인** 패널에서 볼 수 있습니다.

- **실시간 상태:** `CONTROL_STATION_PARKING_SPOT`까지 설정하면 관제가 그 위치를 계속 추적합니다(`GET /api/parking-spots/{spot_id}/live`, 화면은 0.5초마다 갱신). 상태는 화면 밖 → 이동 중 → 정상 위치 / 위치 벗어남이며, 같은 마커 여러 개·시야 미확인·카메라 없음·등록 파일 오류는 확인 불가로 표시합니다. 최근 1초 동안 5 mm·2° 안에서 멈춰 있어야 정상 위치/위치 벗어남을 판정합니다. 멈춘 Pinky도 프레임 하나하나는 약 ±1.5 mm·±0.55° 흔들려서, 처음 정한 2 mm·0.5°로는 계속 이동 중으로 나왔습니다(2026-11-02).
- **화면 구성:** 왼쪽은 천장 카메라 실시간 영상입니다. 그 위에 등록 위치(흰 점선)와 지금 Pinky 마커·앞쪽 방향(상태 색)을 겹쳐 그립니다. 오른쪽은 진행 단계, 상태, 앞·왼쪽·회전 수치, 허용 오차 대비 배수, 위치 벗어남일 때의 보정 방향 안내, 위에서 본 그림입니다. 색은 정상 위치 초록, 조금 벗어남 주황, 많이 벗어남 빨강, 이동 중 파랑, 확인 불가 회색입니다.
- **확정 판정:** 버튼을 누르면 `/check`로 요청 이후 프레임 15장을 판정해 결과를 한 줄로 보여 줍니다. 실시간 추적이 꺼져 있으면 이 결과로 패널 전체를 그립니다.
- 실시간 상태는 화면 표시용입니다. 지나가는 도중 잠깐 범위 안에 든 프레임을 도착으로 착각하지 않도록, 팔 단계 진행은 작업 흐름의 도착 확인(도착 신호 이후 프레임으로 하는 판정)만 씁니다. 패널은 로봇 명령을 보내지 않습니다.

2026-11-02에 실행 중인 관제의 천장 카메라 스트림을 추적기에 넣어 확인했습니다. 시작 약 1.8초 뒤 `IN_POSITION`(앞 −1.0 mm, 왼쪽 +1.0 mm, 회전 −0.3°)이 됐고 이후 유지됐습니다. 이동 중·위치 벗어남 화면은 예시 데이터로 렌더링만 확인했습니다.

### 카메라 영상 밀림

2026-11-02에 카메라를 10번 열었을 때, 한 번은 영상 전체가 가로로 96 px 말린 채 들어왔습니다. 오른쪽 끝 96 px가 왼쪽에 붙어 보였고, 같은 시점에 OpenCV가 `Corrupt JPEG data` 경고를 냈습니다. 이 상태는 카메라를 닫을 때까지 이어지므로 프레임 중앙값으로 걸러지지 않습니다. 이전 판정 방식에서 Pinky가 정지해 있는데 dy가 −66 mm와 +85 mm로 갈렸던 원인입니다.

고정 마커로 등록했으면, 밀린 상태로 열린 프레임에서 고정 마커가 97 px 이동으로 측정되어 `VIEW_UNVERIFIED`가 됩니다. `--no-anchors`로 등록했으면 이를 감지하지 못하고, Pinky가 약 150 mm 옮겨진 것처럼 측정됩니다. 정답 위치에 서 있으면 `MISALIGNED`로 나오는 쪽이 대부분입니다. 하지만 실제로 어긋난 Pinky가 우연히 허용 범위 안으로 측정될 수도 있습니다. 카메라를 다시 열면(명령 재실행) 정상으로 돌아왔습니다. 관제 서버도 Head Camera를 시작할 때 한 번만 열기 때문에, 밀린 상태로 열리면 서버가 켜져 있는 동안 Head Camera 영상 좌표 전체가 어긋날 수 있습니다. 원인(USB 전송 또는 카메라 firmware)은 확인하지 않았습니다.

### 2026-11-02 현장 확인

Pinky를 움직이지 않은 상태에서 확인했습니다.

- 같은 프레임에 이전 PnP 방식을 적용하면 dz −45 mm, dx +9.5 mm였습니다. 평면 투영 방식은 dx −12.9 mm였고, 평면 위에서 잰 마커 한 변이 69.6 mm(인쇄 70 mm)로 맞았습니다. 마커가 약 40 px로 작아 PnP 깊이가 흔들리고, 카메라가 바닥을 26° 기울여 보기 때문에 그 오차가 평면 방향으로 섞인 것으로 판단합니다.
- 지금 위치를 임시 폴더에 등록한 뒤 카메라를 매번 새로 열어 `--once`를 15회 실행했습니다. 15회 모두 `ARRIVED`였고 오프셋은 0.2 mm, 0.06° 이내, 고정 마커 이동은 0.2 px 이하였습니다. 이 15회 중에는 영상 밀림이 일어나지 않았습니다. 앞서 저장한 밀린 프레임은 `VIEW_UNVERIFIED`(97.1 px)로 판정했습니다.
- 약 30분 전 프레임에서는 고정 마커 전체가 1.9 px 움직여 있었습니다. 보정 전에는 4 mm, 시야 흐름 보정 후에는 1 mm 이내의 오프셋으로 측정됐습니다.
- 19:36에 `pinky_dock_1`을 schema v2로 다시 등록했습니다. 고정 마커는 3번과 20번이고 허용 오차는 20 mm / 2°입니다. 약 20분 뒤 판정에서는 `VIEW_UNVERIFIED`가 나왔습니다. 20번은 Pinky 옆에 놓인 카드였는데 Pinky 뒤로 밀려 가려졌습니다. 3번 카드와 Pinky 마커는 같은 방향으로 각각 5.6 px, 6.6 px 움직여 있었습니다. 카메라가 움직였는지, 카드와 Pinky가 각각 옮겨졌는지는 확인하지 않았습니다. 3번은 작고 기울어진 카드라 30프레임 중 26프레임에서만 검출됐습니다.
- 배경 화면 전체를 등록 때 화면과 위상 상관(`cv2.phaseCorrelate`)으로 비교해 고정 마커를 대신하는 방법도 시험했습니다. 밀린 화면은 96 px로 잡았습니다. 하지만 팔·판자·선반 물건·사람이 바뀐 두 화면에서는 실제와 다른 30 px 이동이 나와(응답 0.35), 이 작업장에서는 쓰지 않았습니다.
- 카메라 위치를 고정한다는 운영 결정에 따라, 19:58에 Pinky를 정답 위치에 세우고 `pinky_dock_1`을 `--no-anchors`로 다시 등록했습니다. 허용 오차는 20 mm / 2°로 등록했다가, 팔 작업 허용치를 정하기 전까지 임시로 50 mm / 10°로 넓혔습니다(등록 파일의 `tolerance`와 `tolerance_note`). 마커 장착 방향은 처음에 천장 영상만 보고 `+x`로 기록했는데 틀렸습니다. Pinky를 손으로 앞으로 약 5 cm 밀었더니 마커 −x 방향(천장 영상 아래쪽)으로 53.7 mm, 옆으로 2.2 mm 이동으로 측정됐습니다. 그래서 `marker_forward_axis: -x`로 고쳤습니다. 이 위치에서 판정은 `MISALIGNED`/`NEAR`(앞 +49.3 mm, 왼쪽 −12.4 mm, 허용 50 mm의 1.02배)였습니다.
- 위 확인 중 한 번은 카메라가 밀린 상태로 열려, Pinky가 그대로인데 왼쪽 +150.9 mm, `FAR`로 판정됐습니다. 시야 확인을 끈 등록에서 영상 밀림이 실제로 나타난 사례입니다. 다시 열어 판정한 3회는 모두 정상값이었습니다. 장착 방향은 화면만 보고 정하지 말고, Pinky를 실제로 움직여 확인합니다.
- 이 사례를 보고 고정 마커를 다시 쓰기로 했습니다. 팔이 놓인 나무판(바닥에 고정) 위, Pinky 경로에서 떨어진 곳에 1번 마커를 붙였습니다. Pinky를 정답 위치에 다시 세우고 `--anchor-ids 1 --marker-forward-axis -x`, 허용 50 mm / 10°로 `pinky_dock_1`을 다시 등록했습니다(19:58 등록본은 `rejected/pinky_dock_1-20261102T1958-no-anchors.json`). Pinky 바퀴 옆의 3번은 Pinky에 가려지거나 밀려서, 17번은 40프레임 중 1프레임만 검출돼서 기준에서 뺐습니다.
- 다시 등록한 뒤 카메라를 매번 새로 열어 `--once`를 10회 실행했습니다. 9회는 `ARRIVED`/`WITHIN`(앞·왼쪽 0.1 mm, yaw 0.06° 이내, 고정 마커 이동 0.1 px 이하)였습니다. 1회는 카메라가 밀린 상태로 열렸고, 1번 마커가 96.0 px 이동으로 측정되어 `VIEW_UNVERIFIED`로 판정을 거부했습니다. 기존 19:36 등록은 `rejected/pinky_dock_1-20261102T1936-loose-anchors.json`으로 옮겼습니다. 등록 흔들림은 0.28 mm / 0.32°였고, 평면 위 마커 한 변은 70.0 mm였습니다.
- 이어서 카메라를 매번 새로 열어 `--once`를 11회 실행했습니다. 10회는 `ARRIVED`(최대 2.5 mm, 0.68°)였습니다. 1회는 사람이 Pinky 옆에서 작업하며 마커를 가린 순간이라 `NOT_FOUND`였습니다. 사람이 옆에 있는 동안 측정된 1.3–2.5 mm 변화는 Pinky를 건드린 것인지 측정 오차인지 구분하지 않았습니다.
- 이 결과는 정지 상태의 측정 반복성입니다. Pinky를 다시 세웠을 때의 주차 반복성이나 팔 작업에 필요한 허용 오차는 측정하지 않았습니다.
- 2026-11-03 19:20쯤 Pinky가 정답 위치에서 뒤로 44 mm 밀린 상태가 허용 오차 50 mm / 10° 안이라 `IN_POSITION`으로 표시됐습니다. 화면에 등록 위치와 현재 마커를 겹쳐 그려 약 26 px(마커 한 변의 약 60 %) 어긋난 것을 확인한 뒤, 허용 오차를 25 mm / 5°로 좁혔습니다(등록 파일의 `tolerance`만 수정, 등록 기준은 그대로). 같은 상태가 `OFF_POSITION`, 확정 판정 `MISALIGNED`/`NEAR`(position 44 mm > 25 mm, 비율 1.78)로 바뀌었습니다. 여전히 측정 근거가 없는 임시 값입니다.

### 해석할 때 주의할 점

- `ARRIVED`는 마커가 허용 범위 안에 보인다는 뜻일 뿐입니다. Nav2 이동 성공, 로봇 정지, 팔 작업 가능 여부를 뜻하지 않습니다.
- 기본 허용 오차 30 mm / 5°는 측정 반복성(1 mm, 0.1° 수준)보다 충분히 큰 출발값입니다. 실제 값은 Pinky 재주차 반복성과 팔 작업이 감당할 수 있는 어긋남을 측정한 뒤 정합니다. Nav2 goal checker의 기본 허용값(위치 0.25 m, yaw 0.25 rad)을 그대로 쓰면 Nav2가 목표 도착으로 끝내도 이 판정은 `MISALIGNED`일 수 있습니다. Pinky의 실제 Nav2 설정은 확인하지 않았습니다.
- 마커 평면은 등록할 때 한 번 PnP로 정합니다. 평면 깊이가 틀린 만큼 오프셋 크기가 비례해 달라집니다. 예를 들어 깊이가 3 % 틀리면 90 mm 오프셋이 약 93 mm로 측정됩니다.
- 시야 흐름 보정은 고정 마커와 Pinky 마커가 같은 바닥 높이에 있고 이동이 작다는 가정에 기댑니다. 기준을 넘는 이동은 보정하지 않고 거부합니다. 고정 마커가 하나뿐이면 그 마커가 밀린 것과 카메라가 흐른 것을 구분하지 못하므로, 고정 마커는 단단히 붙여 둡니다.
- `--no-anchors` 등록에서는 카메라 시야가 흐른 만큼 오프셋이 그대로 판정에 들어갑니다. 70 mm 마커가 약 40 px로 보이므로 1 px는 약 1.75 mm입니다. 카메라나 거치대를 건드렸으면 다시 등록합니다.

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
