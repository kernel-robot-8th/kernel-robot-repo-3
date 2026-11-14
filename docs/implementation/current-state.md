# 현재 구현 현황

이 문서는 현재 코드가 실제로 제공하는 기능을 기록합니다. 목표 요구사항은 [시스템 명세](../spec/system-spec.md), 주문 한 건의 단계와 모의/실제 경계는 [주문 흐름](order-flow.md)을 참고하세요.

## 코드 구성

| 파일 | 현재 책임 |
|---|---|
| `run-control-station.sh` | 관제 실행. `--arm`이면 LeRobot 텔레오퍼레이션 시작 API를 켬. 주차 위치나 `ros:` 카메라를 설정하면 ROS 2와 `cpp/install`을 source. 세션 로그는 `.control-station/logs/` |
| `src/control_station/app.py` | 서버 조립(`main`)과 요청 분배. 경로 표는 `api/routes.py` |
| `src/control_station/api/` | `tasks.py`(주문·작업 목록·생성·시작·취소, 명령 해석), `flow.py`(주문 진행 화면, 경보 확인, 모의 세계 조작), `cameras.py`(카메라 목록, MJPEG 스트림), `arm.py`(텔레오퍼레이션·정책 한 번 실행, OMX 명령 큐, 선반 칸, 정책 목록), `status.py`(health, 로봇 상태, 주차 `/live`·`/check`), `common.py`(교차 출처 확인, JSON 본문, 작업 조건·주문 물체 검증). 경로별 상태·오류 코드는 `tests/test_api_routes.py`로 고정 |
| `src/control_station/order_desk.py`, `api/orders.py` | 주문 접수 창구: 주문 창용 catalog, 핑키 없이 접수(`PENDING`), 배정 미리보기, 시작할 때 핑키 확정(`PINKY_ASSIGNED`), 거절. 수령 구역은 `receiving_zones.py`·`config/receiving-zones.json` |
| `src/control_station/assignment.py` | 핑키·OMX 배정 규칙(순수 함수): 핑키는 제외 조건 뒤 배터리 구간 → 오늘 운행 → 쉰 시간 → 번호, 팔은 제약이 많은 물체부터 성공률 → 남은 재고 → 설정 순서. 고른 이유를 함께 돌려줌 |
| `src/control_station/order_flow.py` | 주문 단계 실행: 선반 재고 → 배정 규칙으로 팔 배정 → 핑키 도착 대기 → 물체별 집기·옮김 확인 → 총량 → 핑키 관제 신호. 적재 자리 하나를 주문끼리 나눠 씀 |
| `src/control_station/mock_world.py` | 선반 재고·핑키 위 물건·핑키 임무 단계·팔 집기를 흉내 내는 모의 세계와 사고 주입(모든 결과 `simulated: true`) |
| `src/control_station/pinky_fleet.py` | 핑키 팀 관제(`pinky_fleet_dashboard`) HTTP 클라이언트: 임무 단계 읽기, `dock_complete`·`received` 신호 |
| `src/control_station/camera_counts.py` | 선반 캠(칸 영역 안)과 탑 캠(핑키 쟁반 영역 안)의 YOLO 검출로 종류별 개수 세기(새 프레임 5장 중앙값) |
| `src/control_station/arms.py`, `config/arms.json` | 팔과 담당 선반·재고 카메라 (왼쪽 선반=arm-01, 오른쪽 선반=arm-02) |
| `src/control_station/flow_view.py` | 주문 진행 화면 요약(주문별 진행 막대, 핑키·팔 상태, 경보) |
| `src/control_station/task_orchestrator.py` | 작업 실행. 주문은 `order_flow.py`에 넘김. 이전 형식 작업(이동·팔 7단계)은 모의 단계이고, 설정에 따라 `VERIFY_ARRIVAL`은 실제 주차 판정(`CONTROL_STATION_PARKING_SPOT`), `EXECUTE_ARM_TASK`는 실제 `PICK_WITH_POLICY` 명령(`CONTROL_STATION_TASK_REAL_ARM=1`), `VERIFY_LOAD`는 모의 적재 관측(`CONTROL_STATION_MOCK_LOAD_OBSERVED`) |
| `src/control_station/arm_commands.py` | OMX 명령 큐(`arm_commands` 테이블): `MOVE_TO_SLOT`·`RETURN_HOME`·`PICK_WITH_POLICY`, 팔당 미완료 1건, ACK 제한 시간 뒤 `UNKNOWN`, 자동 재전송 없음 ([HTTP 인터페이스](arm-command-api.md)) |
| `src/control_station/act_policies.py`, `config/omx-act-policies.json` | 물체 이름(YOLO 클래스·한글·별칭) → 그 물체용 ACT 정책(모델, front/wrist 카메라 역할, 최대 실행 시간); 파일 SHA-256을 명령에 고정 |
| `src/control_station/shelf_slot_map.py`, `config/shelf-slots.json` | 선반 칸 정의와 그 SHA-256 버전 |
| `src/control_station/shelf_slot_zones.py` | 정면 카메라의 칸 영역([선반 위치 보정](shelf-pose-calibration.md))과 검출별 칸 배정, 물체가 한 칸에서만 보일 때의 칸 근거(`select_slot_for_class`) |
| `src/control_station/load_check.py` | 이전 형식 작업의 `VERIFY_LOAD` 판정. 관측기는 모의(`CONTROL_STATION_MOCK_LOAD_OBSERVED`)뿐 |
| `src/control_station/parking_ros.py` | C++ 주차 노드의 `/pinky_parking/state`·`/pinky_parking/check`를 관제 JSON(`/live`, `/check`, 도착 단계)으로 바꿈. 관제 자체 판정은 없음 |
| `src/control_station/camera_adapters.py` | 카메라 feed(Head·Joint·Front·선반)를 V4L2(OpenCV) 또는 `ros:<토픽>`으로 받고, 카메라별 선택 YOLO/YOLO-World 검출 표시. 정책 실행·정책 명령 동안 정책 카메라를 닫았다가 다시 엶 |
| `src/control_station/robot_adapters.py` | `/api/robots`: 모의 상태, 또는 ROS 2 모드에서 Nav2 `navigate_to_pose` action server 발견 여부만. 팔은 항상 "미설정" |
| `src/control_station/arm_supervisor.py` | 고정 LeRobot 텔레오퍼레이션 프로세스 시작·종료·상태 |
| `src/control_station/policy_run_supervisor.py` | LeRobot async ACT client를 총 액션 상한으로 한 번 실행(기본 비활성). 정책 목록은 `config/omx-act-policies.json`, 정책별 `cameras`에 따라 위·그리퍼 카메라를 `front`/`wrist`에 넣음, 끝나도 토크 유지. 프로세스 결과만 기록 |
| `src/control_station/json_files.py` | 설정 파일과 요청 본문의 JSON 읽기(중복 키 거부, 크기 제한 일반 파일만) |
| `src/control_station/command_parser.py` | 제한 문법 초안 parser: 집기 조건(물체·색상·선반 층·목적지)과 주문 문장("당근 1개랑 사과 두 개") |
| `models.py`, `database.py`, `store.py`, `migrations/` | 상태 어휘, SQLAlchemy ORM·SQLite, 작업·단계·이벤트 저장, Alembic revision |
| `src/control_station/web/` | 관제 화면 탭: 주문 진행(맨 위 주문 접수), 현황, 수동 조작, 작업 이력, API. 손님용 주문 창(목업) `/order` |
| `cpp/` | Pinky 주차 판정의 유일한 구현(C++ 라이브러리·CLI `pinky_parking`·ROS 2 노드)과 `omx_interfaces`. [cpp/README.md](../../cpp/README.md) |
| `scripts/` | 보정(`calibrate_camera_intrinsics.py`, `calibrate_shelf_pose.py`, 공용 `calibration_artifacts.py`), OMX 워커(`mock_omx_worker.py`, `omx_lerobot_worker.py`, `omx_ros_worker.py`), 정책 도우미(`omx_act_policy_test.sh`, `omx_act_policy_run.py`), `record_omx_pose.py`, `probe_omx_readonly.py`, `check_setup.sh` |
| `simulation/ros2_ws/src/toktak_warehouse/` | Gazebo Harmonic 창고 asset과 bridge ([ROS 2 인터페이스 목록](ros2-inventory.md)) |
| `requirements-ros.txt` | ROS 2 Python 연동용 선택 의존성 `PyYAML` |

## 구현된 API

| 메서드 | 경로 | 동작 |
|---|---|---|
| GET | `/api/flow` | 주문 진행 화면 요약: 최근 주문의 단계별 상태·지금 하는 일·팔 순서, 핑키 3대 단계, 팔 2대 상태, 경보, 적재 자리, 모의 세계 상태 |
| POST | `/api/flow/alerts/{task_id}/ack` | 멈춘 주문의 경보를 운영자가 확인함(`ALERT_ACKNOWLEDGED` 이벤트). 작업 상태는 그대로이고, 그 주문이 잡고 있던 적재 자리·핑키를 놓음 |
| POST | `/api/mock-world` | 모의 세계 조작: `{"action":"inject","fault":...,"argument":...}` 또는 `{"action":"restock"}` |
| GET | `/api/health` | 서버 모드와 주차 설정(`parking.camera_id`(패널이 보여 줄 feed), `parking.spot_id`, `parking.live_tracking`) |
| GET | `/api/robots` | 모의 상태 또는 ROS 2 Nav2 action server 발견 여부. 발견은 goal 전송·준비·안전 판정이 아님 |
| GET | `/api/cameras` | 카메라별 연결·스트림·freshness, 프레임 sequence·해상도·SHA-256·`pixel_transform_id`, YOLO/YOLO-World 검출(`detector_state`, `detections`)과 모델 provenance. 선반 위치 보정이 활성화된 카메라는 `slot_zones`와 검출별 `slot_assignment`(표시 전용) 포함 |
| GET | `/streams/{camera_id}.mjpg` | 카메라 MJPEG 스트림 |
| GET | `/api/parking-spots/{spot_id}/live` | 주차 노드가 추적하는 위치의 화면 표시용 실시간 상태. 추적하지 않는 위치면 `PARKING_TRACKING_NOT_CONFIGURED`(404). 진행 판단에 쓰지 않음 |
| GET | `/api/parking-spots/{spot_id}/check` | 노드의 `/pinky_parking/check`로 요청 이후 프레임만 판정해 `{"observation": ...}` 반환. 오류 코드는 [주차 정렬 관측 계약](../spec/interface-contracts.md#pinky-주차-정렬-관측-초안). 작업 이력에 기록하지 않음 |
| GET | `/api/arm/status` | 텔레오퍼레이션과 학습 정책 한 번 실행 프로세스 상태 |
| POST | `/api/arm/start`, `/api/arm/stop` | 고정 프로필 텔레오퍼레이션 시작, 프로세스 그룹에 `SIGINT` 종료 요청 |
| POST | `/api/arm/policy/start?profile_id=<정책 ID>`, `/api/arm/policy/stop` | `config/omx-act-policies.json`의 정책 하나(예: `carrot_act_v2`, 없으면 첫 정책)를 총 액션 상한으로 실행, 중단 요청. 프로세스 종료 상태만 기록 |
| GET | `/api/arm/policies`, `/api/arm/shelf-slots` | 물체별 학습 정책 목록, 선반 칸 정의 |
| GET | `/api/arm/commands`, `/api/arm/commands/{id}`, `/api/arm/{robot_id}/commands/next` | OMX 명령 목록·조회, 워커의 long-poll |
| POST | `/api/arm/commands`, `/api/arm/commands/from-camera`, `/api/arm/commands/{id}/events`·`cancel`·`clear-unknown` | OMX 명령 등록(정면 카메라 칸 확인 포함), 워커 이벤트 보고, 취소, 결과 불명 정리 ([HTTP 인터페이스](arm-command-api.md)) |
| GET | `/api/tasks`, `/api/tasks/{task_id}` | 작업 요약 목록, 작업 단계·이벤트 이력과 비저장 `local_execution_owner` |
| POST | `/api/tasks` | 주문: `order_items`(1–5종, 종류별 1–5개, 학습 정책이 있는 물체)와 선택 `pinky_id`(`10`/`11`/`12`, 없으면 비어 있는 핑키 배정, 모두 바쁘면 409 `PINKY_UNAVAILABLE`). 이전 형식: `destination`, `arm_job`, 선택 `pick_request`. 그 밖의 필드는 거부. `PENDING` 작업·단계·`TASK_CREATED` 이벤트 저장 |
| POST | `/api/tasks/parse-command` | 제한 문법 문장을 집기 조건·주문 물체 초안으로 해석. 저장이나 명령 없음 |
| POST | `/api/tasks/{task_id}/start`, `/cancel` | 작업 시작(비동기), 취소 요청. 모의 모드에서만 받고 ROS 2 모드는 `EXECUTION_MODE_UNSUPPORTED`. 오류는 `error_code`/`message`/`error` |

task start/cancel과 주차 판정 GET만 안정된 오류 envelope(`error_code`/`message`/`error`)를 씁니다. POST JSON body는 중복 object key를 거부합니다(HTTP 400).

## 현재 동작의 경계

- 서버는 기본 `127.0.0.1`에 바인딩합니다(`CONTROL_STATION_HOST`로 바꿀 수 있고 인증은 없음). 기본 실행은 모의 모드이며 로봇에 접속하지 않습니다.
- 주문 흐름에서 실제로 쓸 수 있는 것은 주차 판정, 핑키 관제 HTTP 신호, 카메라 개수 세기이고, 팔 집기(`PICK_ITEM`)는 아직 모의입니다([주문 흐름](order-flow.md#무엇이-실제이고-무엇이-모의인가)).
- 이전 형식 작업은 `CONTROL_STATION_TASK_REAL_ARM=1`일 때만 `EXECUTE_ARM_TASK`가 OMX 명령 큐로 `PICK_WITH_POLICY`를 보냅니다. 정면 카메라가 물체를 한 칸에서만 볼 때만 명령을 만들고, 그 밖의 단계(주행 등)는 모의입니다([HTTP 인터페이스](arm-command-api.md#학습-정책으로-집기-pick_with_policy)). 실제 정면 카메라 검출 상태에서 이 경로로 실물 팔을 움직이는 시험은 하지 않았습니다.
- Pinky 주차 판정은 C++ 노드만 합니다. 관제는 `CONTROL_STATION_PARKING_SPOT`을 지정했을 때 노드에 묻고, 노드가 꺼져 있거나 응답하지 않으면 판정 불가로 다음 단계를 시작하지 않습니다. `ARRIVED`는 마커가 허용 범위 안이라는 관측일 뿐 Nav2 이동 성공을 대신하지 않습니다. 운영 절차와 현장 기록은 [실행 방법](runbook.md#천장-카메라-pinky-주차-판정), 결과 형식과 남은 결정은 [주차 정렬 관측 계약](../spec/interface-contracts.md#pinky-주차-정렬-관측-초안)에 있습니다.
- 핑키 주행과 임무는 핑키 팀 관제가 맡습니다. 관제는 그 관제의 `GET /api/status`로 단계를 읽고 `POST /api/robots/<id>/operator-confirm`으로 `dock_complete`·`received`만 보냅니다. Nav2 goal은 보내지 않습니다.
- 팔 제어(`--arm`, `CONTROL_STATION_POLICY_ENABLED`)는 기본 비활성입니다. LeRobot 프로세스 생존이나 정책의 액션 상한 도달은 장치 준비나 집기·적재 성공 신호가 아닙니다.
- async inference client는 별도 LeRobot checkout `~/Desktop/lerobot`의 수정된 `robot_client.py`(`--max_action_steps` 지원)가 필요합니다. 관제는 OpenCV와 OMX config 모듈을 먼저 import한 뒤 client를 실행합니다.
- 카메라 stale 판정은 카메라 단위이며, 소비자는 프레임 freshness와 detector 상태를 함께 확인해야 합니다. `pixel_transform_id`는 소프트웨어 영상 경로만 식별하고 driver/firmware 설정은 확인하지 않습니다. 같은 실물 장치를 두 feed에 지정하면 둘 다 열지 않고 경로 충돌 오류를 표시합니다.
- Joint Camera 기본값은 Innomaker by-id, Head Camera 기본값은 HCAM0 by-id입니다. HCAM0 두 대는 by-id가 같아 현장에서는 by-path로 지정합니다([실행 방법](runbook.md#선반-카메라-표시)). 카메라의 물리 장착 역할은 화면으로 확인해야 합니다.
- `probe_omx_readonly.py`는 수동 실행 때만 follower 레지스터를 읽으며, 관제의 READY 상태로 쓰지 않습니다. LeRobot `OmxFollower.send_action()`은 관절 목표를 쓸 뿐 파지 성공 결과를 주지 않고, `connect()`는 모터 설정을 쓸 수 있어 읽기 전용 확인에 쓰지 않습니다.
- 작업 생성은 `PENDING` 작업·단계·`TASK_CREATED` 이벤트를 한 트랜잭션으로 저장하며 로봇 명령을 보내지 않습니다. 이동·팔 작업은 7단계(2026-11-03 이전에 저장된 작업은 6단계), 주문은 물체 수에 따라 단계가 늘어납니다.
- 시작 요청은 한 서버 프로세스 안에서 활성 작업 하나만 허용합니다. 주문은 핑키가 다르면 동시에 진행하고, 적재 자리는 주문 흐름이 하나씩 나눠 줍니다. 같은 작업의 반복 시작은 worker를 더 만들지 않고, 재시작 뒤 worker가 없는 활성 작업은 자동 재개하거나 명령을 다시 보내지 않습니다. `local_execution_owner`(`NOT_ACTIVE`, `LOCAL_WORKER_ACTIVE`, `NO_LOCAL_WORKER`, `MODE_NOT_MANAGED`)는 이 프로세스의 worker 소유만 보여 줍니다.
- 단계 실패 시 남은 단계는 `NOT_EXECUTED_AFTER_FAILURE`로 닫히고 작업은 `FAILED`가 됩니다. 시작 전 취소는 한 트랜잭션으로, 진행 중 취소는 worker가 멈춘 뒤 기록합니다. 단계 시작 기록과 취소·실패 종결은 같은 프로세스 lock으로 직렬화합니다. 프로세스 중단 복구와 장치 timeout 처리는 없습니다.
- `CONTROL_STATION_MOCK_FAIL_STEP`은 지정한 단계를 `MOCK_SCENARIO_FAILURE`로 실패시키는 모의 전용 설정입니다.
- 모의 단계는 저장소에서 task/step/command 일치를 확인한 뒤 `PROGRESS: 50%`를 한 번 기록합니다. 장치 feedback이 아닙니다. mock `robot_id`(`mobile-01`/`arm-01`)는 실제 장치 식별자가 아닙니다.
- Alembic revision은 `0001_task_history`, `0002_pick_request`, `0003_arm_commands`, `0004_order_items`이며 기존 작업 행을 보존합니다.

## 테스트

저장소 루트에서 `.venv/bin/python -m pytest -q`로 `tests/`를 돌립니다(`pytest.ini`). C++ 테스트는 [cpp/README.md](../../cpp/README.md)를 따릅니다.

## 다음 작업

우선순위는 [개발 로드맵](../development-plan.md)과 [주문 흐름의 남은 일](order-flow.md#남은-일)에서 관리합니다.
