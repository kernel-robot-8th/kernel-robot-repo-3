# 현재 구현 현황

이 문서는 현재 코드가 실제로 제공하는 기능을 기록합니다. 목표 요구사항은 [시스템 명세](../spec/system-spec.md)를 참고하세요.

## 코드 구성

| 파일 | 현재 책임 |
|---|---|
| `run-control-station.sh` | mock/ROS 실행 mode와 팔 process flag를 분리하고 Python entry point를 선택 |
| `src/control_station/app.py` | localhost HTTP 서버 조립(`main`)과 요청 분배. 경로 판별은 `api/routes.py`의 표, 처리 코드는 영역별 `api/` 모듈 |
| `src/control_station/api/` | HTTP 경로 처리: `tasks.py`(작업 목록·생성·시작·취소, 명령 해석), `observations.py`(대상 관측·선택, 투영 초안, 관측 이미지), `cameras.py`(카메라 목록, MJPEG 스트림, 카메라 대상 관측), `arm.py`(팔 텔레오퍼레이션, OMX 명령 큐, 선반 칸, 좌표 초안), `status.py`(health, 로봇 상태, 주차 `/live`·`/check`), `common.py`(교차 출처 확인, JSON 본문 규칙, 작업 조건·관측 프레임 검증). 경로별 상태 코드·오류 코드는 `tests/test_api_routes.py`로 고정 |
| `src/control_station/robot_adapters.py` | mock 상태, ROS 2 Nav2 action server discovery, 설정된 OMX `JointState` freshness 표시; position 단위 미확정; ROS 명령 없음 |
| `src/control_station/arm_supervisor.py` | 고정 LeRobot 텔레오퍼레이션 프로세스 시작·종료·상태 |
| `src/control_station/policy_run_supervisor.py` | LeRobot async ACT client를 총 액션 상한으로 한 번 실행; 기본 비활성, process 결과만 기록하며 작업 성공을 추정하지 않음 |
| `src/control_station/omx_pose.py` | OMX follower 정규화 관절 목표 JSON의 오프라인 검증; bool schema version과 float 변환 overflow 거부; 장치 연결·명령 없음 |
| `src/control_station/coordinate_target_drafts.py` | 좌표 입력을 숨김폴더의 `DRAFT_ONLY` JSON으로 저장; IK/장치 명령 없음 |
| `scripts/calibrate_camera_intrinsics.py` | 저장된 체커보드 이미지(각 최대 64 MiB, symlink 거부) bytes의 SHA-256과 OpenCV 내부 파라미터 schema v2 초안을 계산하고 공통 validator로 출력 계약 확인; camera API의 `pixel_transform_id` 입력 필요; 카메라/로봇 연결 없음 |
| `scripts/calibration_artifacts.py` | calibration input regular-file bounded reader(이미지 64 MiB, TCP/JSON evidence 계약별 limit), 중복 JSON key 거부, intrinsic/plane/ROI 및 link6↔TCP 검토 draft의 구조·provenance validator와 산출물 no-overwrite atomic publisher; TCP는 물리 모델 revision, frame mapping reference/hash, 측정 source/hash, uncertainty, normalized quaternion을 검증; 측정 품질/사실성 승인 없음 |
| `scripts/calibrate_shelf_plane.py` | 최대 1 MB의 비-symlink schema v1 측정 대응점 JSON과 내부 보정 입력에서 중복 key 거부, 경로 구성요소 symlink 거부/최대 64 MiB·실제 decode 해상도 확인을 적용한 참조 원본 이미지별 SHA-256, 일치하는 pixel transform ID 및 내부 보정 초안으로 고정 선반 평면 homography schema v4 초안을 계산; OMX 좌표나 명령 없음 |
| `scripts/calibrate_shelf_regions.py` | 최대 64 MiB task JSON 및 최대 1 MB ROI JSON bounded reader, 저장된 task observation event의 task/frame provenance와 최대 64 MiB PNG/pixel SHA-256 검증, symlink 경로·입력 JSON 중복 키 거부를 거쳐 운영자 측정 층 polygon을 `DRAFT_ONLY`·선택 비활성 artifact로 생성; 활성 파일로 명시 배치하면 관측 API가 검토 전용 비교 |
| `scripts/shelf_region_editor.html` | 저장 observation PNG에 원본 해상도 픽셀 좌표 polygon을 수동 입력하고 PNG file SHA-256/해상도를 포함한 `regions.json` schema v2로 다운로드하는 브라우저 도구; 물리 측정·승인 없음, CLI가 PNG provenance를 대조 |
| `scripts/project_observation_to_shelf_plane.py` | 최대 64 MiB task JSON 및 최대 1 MB calibration JSON bounded reader와 duplicate-key 거부, task 저장 조건·유일한 observation event·camera raw-frame SHA-256/schema v2/pixel transform ID·plane correspondence provenance를 확인한 뒤 bbox 하단 중앙 proxy를 고정 선반 평면 2D `DRAFT_ONLY`에 투영; CLI index에는 frame-age 제한, 운영자 선택 event에는 최대 64 MiB 보관 PNG hash 검증·symlink 경로 거부와 review-only snapshot policy 적용; 선택적 PNG overlay 및 opt-in localhost task review event 기록; z/OMX/명령 없음 |
| `src/control_station/projection_drafts.py` | 저장된 2D projection JSON과 원본 plane/intrinsic calibration JSON의 SHA-256, pending task/step 및 event/step provenance를 교차 검증하는 review-only API 경계; robot command 생성 없음 |
| `scripts/probe_omx_readonly.py` | LeRobot 버스의 ID/model handshake 후 OMX register `sync_read`; 목표·설정 쓰기 없이 한 시점 값을 표준 출력 |
| `scripts/omx_act_policy_test.sh` | LeRobot ACT 정책 서버, 정책 파일의 카메라 역할에 맞춘 수동 정책 클라이언트, OMX 워커 실행 도우미; 종료 시 토크 유지, 다른 팔 프로세스·카메라 사용 중이면 시작 거부 ([runbook](runbook.md#act-집기-정책-실물-시험)) |
| `config/omx-act-policies.json`, `src/control_station/act_policies.py` | 물체 이름(YOLO 클래스·한글·별칭) → 그 물체용 ACT 정책(모델, front/wrist 카메라 역할, 최대 실행 시간); 중복 이름·잘못된 카메라 대응 거부; 파일 SHA-256을 명령에 고정 |
| `src/control_station/task_orchestrator.py` (팔 단계) | `CONTROL_STATION_TASK_REAL_ARM=1`이면 `EXECUTE_ARM_TASK`가 정면 카메라에서 물체가 칸 하나에 보일 때만(SR-13, 아니면 팔 명령 없이 실패) 팔 작업 문구의 물체 정책으로 `PICK_WITH_POLICY`를 카메라 근거와 함께 보내고 결과를 기다림; home 복귀·적재 미확인이면 성공으로 넘기고 적재 확인이 판단; 기본은 기존 모의 단계 ([HTTP 인터페이스](arm-command-api.md#학습-정책으로-집기-pick_with_policy)) |
| `src/control_station/camera_to_arm.py` | 정면 캠 선반 좌표 → 이 팔 좌표(현장 터치 2점으로 맞춘 평면 회전·이동, PC별 파일 `CONTROL_STATION_CAMERA_TO_ARM`); 물체 위(손가락 끝 6 cm 위·2 cm 앞, 손목 45° 아래) `MOVE_TO_POSITION` 목표 계산; 관제 화면 "정면 캠으로 물체 위로 이동" |
| `scripts/omx_act_policy_run.py` | OMX 워커 안에서 LeRobot `RobotClient`로 정책 실행; 정책이 쓴 관절 목표로 "home을 벗어났다가 돌아와 머묾"을 판정해 정지, 최대 시간·취소·오류 처리; 정지 후 토크 유지. `PICK_WITH_POLICY`([HTTP 인터페이스](arm-command-api.md#학습-정책으로-집기-pick_with_policy)): 관제 화면 수동 조작 탭의 물체별 버튼("당근 집기"·"물티슈 집기", 정책 파일에서 자동 생성)이 정면 카메라 칸 확인 뒤 명령 생성(확인 실패 시 운영자 확인을 거쳐 근거 없이 생성), 워커가 정책 실행 → home 정렬 → `RESULT_UNKNOWN`/`PLACE_NOT_VERIFIED`. 실물 실행은 미시험 |
| `simulation/ros2_ws/src/toktak_warehouse/` | 기존 Gazebo Harmonic 창고 asset 및 bridge; ROS 2 Jazzy에서 해당 package colcon build 성공, launch와 action server는 현재 세션에서 실행/확인하지 않음 |
| `scripts/create_gripper_geometry_draft.py`, `scripts/validate_gripper_geometry.py`, geometry measurement input/output schemas | `DRAFT_ONLY` 입력 계약에 맞는 운영자 측정 JSON record와 hash-verified TCP calibration artifact를 연결해 no-overwrite geometry artifact 작성; 검증기는 managed evidence hash, closing/approach unit axes 직교성, TCP frame ID, min/max measured gap, TCP 기준 좌/우 finger contact polygon의 서로 다른 3점과 vertex 순서 기준 0이 아닌 signed area, 측정 uncertainty를 확인; 물리 측정 정확도·접촉 가능성·grasp·robot readiness는 확인하지 않음; 현장 record/artifact 부재, CLI 실행 전, no command |
| `scripts/create_link6_tcp_transform_draft.py` | 운영자가 제공한 실측값·검토된 frame mapping과 관리 디렉터리의 두 근거 파일로 덮어쓰기 불가 `DRAFT_ONLY` artifact를 생성; 실측값/근거 자료가 없어 실행하지 않았고, 생성물은 로봇 제어에 사용할 수 없음 |
| `.control-station/omx-calibrations/` | artifact와 frame-mapping/measurement evidence 저장 위치; 현재 폴더 또는 실측 artifact 없음 |
| `cpp/` | Pinky 주차 판정의 유일한 구현([전환 계획](../plans/pinky-parking-ros2-node.md) 순서 1~5, 2026-11-03에 Python 판정을 옮기고 지움). `parking_vision`: ArUco 검출·평면 투영 판정, 고정 마커 시야 확인(흐름 보정, 마커끼리 3 px 넘게 다르면 거부, 단일 마커는 신뢰 표시 없으면 3 px까지), 요청 이후 프레임 수집, 화면 표시용 상태 기계, schema v2 등록·로더, 등록·판정 CLI `pinky_parking`; `parking_vision_interfaces`(`ParkingState.msg`, `CheckParking.srv`); `parking_vision_ros`: 천장 카메라 캡처 컴포넌트(끊기면 2초마다 다시 엶)와 판정 컴포넌트(`/pinky_parking/state`, `/pinky_parking/check`), `ceiling_parking.launch.py`. 옮기기 전 Python 결과로 만든 기대값과의 비교 테스트(같은 OpenCV 4.6이면 1e-9 이내), localhost ROS end-to-end 테스트, 실물 천장 카메라 확인. 자세한 내용은 [cpp/README.md](../../cpp/README.md) |
| `src/control_station/parking_ros.py` | `CONTROL_STATION_PARKING_SPOT`이 있으면 C++ 노드의 상태 토픽과 판정 서비스를 관제 JSON(`/live`, `/check`, `VERIFY_ARRIVAL`)으로 바꿈; 노드 상태가 없거나 2초 넘게 멈추면 `NODE_UNAVAILABLE`, 서비스 연결 실패·6초 timeout은 판정 불가(재시도 없음); 단위 테스트 12개 |
| `cpp/parking_vision/tools/pinky_parking.cpp` | `ros2 run parking_vision pinky_parking`. 주차 정답 위치 `register`(흔들림 5 mm/3° 초과 또는 고정 마커 없음이면 거부, `--no-anchors`, `--marker-forward-axis`, `--trust-single-anchor`, 창 없는 `--headless`, 덮어쓰기 불가 `DRAFT_ONLY` schema v2 JSON)와 `check`(실시간 창 또는 `--once` JSON·종료 코드 0/1/2); 등록 결과가 옮기기 전 Python CLI와 같은지 비교 테스트; Pinky·팔 명령 없음 |
| `scripts/validate_omx_pose.py` | 최대 64 KiB 비-symlink JSON의 중복 키/관절 목표 형식 검사 CLI |
| `src/control_station/camera_adapters.py` | OpenCV physical camera manager: Head·Joint·Front·선반 카메라 feed, 선택 YOLO/YOLO-World 검출 표시, 최근 raw frame 보관, `ros:<topic>` 카메라 구독. 학습 정책 client(관제의 정책 한 번 실행)가 도는 동안 Front Camera를 닫고, 정책 집기 명령(`PICK_WITH_POLICY`)이 워커에 있는 동안 정책 카메라와 같은 V4L2 피드를 닫았다가 끝나면 다시 엶 |
| `src/control_station/shelf_regions.py` | 명시된 camera별 ROI artifact의 source/camera/해상도/pixel transform 계약과 공통 shelf-level 범위(1–999,999,999)를 확인해 후보 `center_px`의 층 비교 상태/index를 계산; 1 MB 이내로 읽은 artifact bytes SHA-256을 성공·검증실패 결과에 보존; 경로 구성요소 symlink/non-file 입력 거부; 미설정·불일치·모호 상태 구분, 후보 선택 없음 |
| `src/control_station/models.py` | 작업·단계 상태 어휘 및 카메라 스냅샷 |
| `src/control_station/database.py` | SQLAlchemy ORM 모델, SQLite 엔진과 세션 설정; optional task `pick_request` JSON |
| `src/control_station/store.py` | ORM 세션으로 작업/단계 상태 및 감사 이벤트를 저장; 활성 task/step/command 일치가 확인된 step 진행 이벤트도 상태 전이 없이 추가 |
| `src/control_station/task_orchestrator.py` | 장치 연결 없이 작업 단계와 상태/진행 이벤트를 기록하는 mock 전용 실행기; mock robot ID 연결, 선택 step의 명시적 실패 주입 및 나머지 단계 미실행 기록 |
| `requirements-ros.txt` | ROS 2 Python 연동용 선택 의존성 `PyYAML`; system `dist-packages` 전체를 `.venv`보다 앞에 두지 않도록 분리 |

| `src/control_station/command_parser.py` | 제한된 한국어/영어 문법에서 상자·병, 색상, 숫자/일부 한국어 서수 선반 층, 목적지를 추출하는 사람 검토용 초안 parser; 부정/제외 표현은 category/color/shelf 자동 선택에서 제외 |
| `migrations/` | Alembic schema revision |
| `src/control_station/web/` | 간단한 상태 화면과 텔레오퍼레이션 제어 |

## 구현된 API

| 메서드 | 경로 | 동작 |
|---|---|---|
| GET | `/api/health` | 서버 모드와 주차 판정 설정(`parking.source`(`ros` 또는 `null`), `parking.camera_id`(패널이 보여 줄 feed), `parking.spot_id`, `parking.live_tracking`) 반환 |
| GET | `/api/parking-spots/{spot_id}/live` | `CONTROL_STATION_PARKING_SPOT`으로 지정한 위치의 화면 표시용 실시간 상태(`NOT_IN_VIEW`/`MOVING`/`IN_POSITION`/`OFF_POSITION`/판정 불가) 반환; 추적하지 않는 위치면 `PARKING_TRACKING_NOT_CONFIGURED`(404); 팔 단계 진행에 쓰지 않음 |
| GET | `/api/robots` | mock 상태 또는 ROS 2 Nav2 discovery, 선택 설정의 OMX `/joint_states` 관측, 선택 설정의 arm trajectory action-server discovery 반환; action discovery는 goal 전송/준비·안전 판정이 아님 |
| GET | `/api/cameras` | Head·Joint 카메라와 선택 설정한 선반 카메라(`shelf_camera`, `CONTROL_STATION_SHELF_CAMERA`) 상태, 설정된 스트림 주소, 각 feed에 설정된 YOLO/선택 YOLO-World 검출 결과(`detector_state`, `detections`) 및 model type·prompt·가중치·추론 provenance 반환; 활성화한 선반 위치 보정이 있는 카메라는 `slot_zones`(6칸 영상 영역)와 검출별 `slot_assignment`(bbox 아래 가운데 점이 든 칸, 표시 전용 — [선반 위치 보정](shelf-pose-calibration.md#관제-화면에-연결)) 포함; MuJoCo mode는 fixed camera detector 상태/provenance와 depth/CameraInfo 및 선택형 PointCloud2 metadata도 노출 |
| GET | `/api/cameras/{camera_id}/target-observation?class_name=...` | 대상 클래스의 최신 카메라 관측을 미설정/준비/오류, fixed frame 미추론, stale/프레임 없음, 후보 없음/단일/복수로 분류; 선택적 `color` 힌트와 `shelf_level` 영상 영역 비교를 검토용으로 표시; 후보 필터·선택 및 명령 없음 |
| GET | `/api/tasks/{task_id}/observations/{event_id}/image` | 저장된 `TARGET_OBSERVED` 이벤트에 연결된 PNG만 task/event 경로 및 SHA-256 확인 후 제공 |
| GET | `/api/tasks/{task_id}/projection-drafts/{event_id}/artifact` | projection review event의 경로/hash와 draft/task ID를 확인한 뒤 저장된 `DRAFT_ONLY` JSON을 제공; 파일 변경/누락 시 거부 |
| POST | `/api/tasks/{task_id}/target-selection` | 대기 작업의 지정된 fresh `TARGET_OBSERVED` event와 새 선택일 때 저장 PNG/hash를 확인하고 명시한 0-based 후보를 pending `SELECT_TARGET` step에 연결된 `TARGET_SELECTED` event로 append; 동일 요청 재전송은 저장 트랜잭션에서 중복 확인 후 기존 event ID 반환; 관측/candidate/frame provenance와 요청 조건 보존; operator review 기록만 남기며 task/step 상태나 로봇 명령은 변경하지 않음; 오류는 `error_code`/`message`와 호환용 `error` 제공 |
| POST | `/api/tasks/{task_id}/projection-draft` | 관리된 calibration 디렉터리의 2D 선반 평면 초안 JSON과 SHA-256, 원본 관측·선택 event/step 링크를 검증하고 pending `ESTIMATE_TARGET_POSE`에 검토 이벤트로 append; 재요청은 초안 ID/hash로 멱등 처리; task/step 상태와 로봇 명령은 변경하지 않음; 오류는 `error_code`/`message`와 호환용 `error` 제공 |
| GET | `/api/parking-spots/{spot_id}/check` | C++ 판정 노드의 `/pinky_parking/check`로 요청 이후 프레임만 판정해 `{"observation": ...}` 반환; 오류는 `PARKING_SPOT_ID_INVALID`(400), `PARKING_SPOT_NOT_FOUND`(404), `PARKING_SPOT_INVALID`(409), `PARKING_CAMERA_UNAVAILABLE`(503), 판정 요청이 이미 4개 진행 중이면 `PARKING_CHECK_BUSY`(429), 노드 미설정·연결 실패 `PARKING_NODE_UNAVAILABLE`(503), 6초 timeout `PARKING_NODE_TIMEOUT`(503)의 `error_code`/`message`/`error`; 작업 이력에 기록하지 않음 |
| GET | `/api/arm/status` | 팔 텔레오퍼레이션과 학습 정책 1회 실행 프로세스 상태 반환 |
| POST | `/api/arm/start` | 고정 프로필 프로세스 시작 요청 |
| POST | `/api/arm/stop` | 프로세스 그룹에 `SIGINT` 종료 요청 |
| POST | `/api/arm/policy/start?profile_id=apple_pick` | 선택한 LeRobot ACT async policy profile을 총 액션 상한으로 시작; 현재 `apple_pick`은 UI 이름 `사과 집기`를 학습 task `Pick up Doll`과 연결; 프로세스 종료 상태만 기록 |
| POST | `/api/arm/policy/stop` | 학습 정책 client 중단 요청; 실제 동작 결과 미확정 |
| POST | `/api/arm/coordinate-target-drafts` | 좌표 입력을 미실행 초안 파일로 저장 |
| POST/GET | `/api/arm/commands`, `/api/arm/commands/{id}`, `/api/arm/commands/{id}/events`·`cancel`·`clear-unknown`, `/api/arm/{robot_id}/commands/next` | OMX 명령 큐([HTTP 인터페이스](arm-command-api.md)): 운영자가 고른 선반 칸(`pick_slot_id`, `config/shelf-slots.json`의 사용 칸 6개) 또는 좌표 1건을 `arm_commands` 테이블에 고정해 두고 외부 워커가 long-poll로 가져가 이벤트를 보고. 팔당 미완료 1건, ACK 제한 시간 뒤 `UNKNOWN`, 자동 재전송 없음. 작업 단계와 미연결이며 확인한 워커는 팔을 움직이지 않는 `scripts/mock_omx_worker.py`뿐 |
| GET | `/api/tasks` | SQLite 작업 요약 목록 반환 |
| GET | `/api/tasks/{task_id}` | 작업 요약, 단계 및 이벤트 이력과 비저장 `local_execution_owner` 관측 반환; 없으면 `404` |
| POST | `/api/tasks` | `destination`, `arm_job`, 선택적 `pick_request` 외의 필드는 거부; `PENDING` 작업, 표준 단계 및 `TASK_CREATED` 이벤트 저장 |
| POST | `/api/tasks/parse-command` | 제한 문법 자연어를 구조화 초안으로 해석; 저장이나 명령 전송 없음 |
| POST | `/api/tasks/{task_id}/start` | mock 모드에서 비동기 작업을 시작; 단일 활성 작업과 반복 요청을 처리; 오류 응답은 안정적 `error_code`/`message`와 호환용 `error` 제공 |
| POST | `/api/tasks/{task_id}/cancel` | mock worker의 취소 요청 및 정지 확인; 오류 응답은 안정적 `error_code`/`message`와 호환용 `error` 제공 |

위의 task start/cancel, target-observation, target-selection, projection-draft POST 경로와 주차 판정 GET만 현재 안정된 오류 envelope를 제공합니다. 카메라 GET 관측 API와 다른 경로는 아직 전역 계약으로 마이그레이션되지 않았습니다.

POST JSON body는 중복 object key를 허용하지 않으며, 중복 키가 있으면 task 저장이나 event append 전에 HTTP 400으로 거부합니다.



작업 이력 화면에서 목적지·팔 작업과 선택 대상 조건(종류·색상·선반 층)을 등록하고, 대기 중인 작업에서 각 카메라의 현재 관측을 task 이벤트로 남길 수 있습니다. Head 이벤트는 pending `DETECT_TARGETS`, Joint 이벤트는 pending `REFINE_TARGET`에 review 목적으로 연결됩니다; 이는 실제 실행 단계 수행이나 접근 시점 재인식 증거가 아닙니다. `TARGET_OBSERVED` 이벤트에는 `task_id`, 요청 조건, detector model type/고정 prompt vocabulary와 요청별 one-shot prompt scope/version/가중치 hash, frame SHA-256/`pixel_transform_id` 출처, 추론 threshold·이미지 크기·요청 device·Ultralytics 버전, 픽셀 후보와 명령 미전송 표시를 저장합니다. 선택 색상은 후보의 `color_hint`와 비교해 `color_evaluation` 상태와 후보 index를 남깁니다. `shelf_level`은 운영자가 명시적으로 활성화한 camera별 ROI artifact가 있고 camera ID·해상도·`pixel_transform_id`가 맞을 때 후보 중심점의 `shelf_level_evaluation`을 남깁니다. ROI 파일을 1 MB 이내로 모두 읽으면 유효 비교 여부와 관계없이 JSON bytes SHA-256을 기록하고 관측/이벤트 UI에 표시합니다; 읽기 실패나 크기 초과 시 불완전한 hash는 만들지 않습니다. 두 비교 모두 미보정/검토 자료이며 후보 필터·자동 선택에는 쓰지 않습니다. 작업이 `PENDING`인 동안 운영자는 특정 관측 event의 후보 index를 `TARGET_SELECTED`로 별도 기록할 수 있습니다. 이 기록은 해당 event의 candidate/frame/detector provenance와 요청 조건을 보존하지만 추론 결과를 승인하거나 task/step 상태·팔 명령에 연결하지 않습니다. frame SHA-256은 검출 직전 OpenCV 원본 BGR 픽셀 buffer를, detector SHA-256은 로드 시 가중치 file bytes를 식별합니다. device가 null이면 backend 자동 선택을 요청한 것이며 실제 선택된 실행 장치를 뜻하지 않습니다. confidence threshold는 모델 점수 기준이며 확률 보정값이 아닙니다. 기본 동작은 이미지 저장 없음이며, 사용자가 `save_frame: true`를 선택하면 일치하는 frame을 `.control-station/observations/{task_id}/`에 lossless PNG로 보존하고 파일 경로와 별도 PNG hash를 기록합니다. 관측 schema v2/v3이며 schema v1 과거 event는 현재 projection CLI가 거부합니다. 이 이벤트는 위치 추정 또는 집기 단계의 완료 증거가 아닙니다. `pick_request`가 있는 작업은 인식→선택→위치 추정→접근→파지→배치 순서의 Pick & Place mock 단계를 저장합니다. mock 단계 성공은 진행 시뮬레이션일 뿐 실제 인식·위치 추정·로봇 동작이 아닙니다.

`POST /api/tasks/parse-command`는 현재 규칙 기반 제한 parser입니다. 한국어 문형에서 숫자 또는 첫째~열째 일부 한국어 서수 선반 층·목적지·집기 동사를, 한국어 별칭과 일부 영어 물체/색상어에서 상자·병·색상을 추출합니다. 목적지는 `집어서 [목적지]에 넣어줘` 문형과 fallback `상자를 [목적지]로 옮겨줘` 문형을 인식합니다. `ready` 상태 카메라의 detector class가 제공되면 문장에 명시된 class label을 먼저 대조하고, 상자/병 별칭도 해당 모델 label과 연결합니다. Head Camera가 ready YOLO-World인 경우에 한해 vocabulary 밖의 상자/병 별칭 또는 단일 한국어 목적격 대상구를 요청 prompt 검토 초안으로 채울 수 있습니다. 오류·미준비 카메라의 클래스는 사용 가능 목록에서 제외합니다. 모호하거나 detector class와 일치하지 않는 category, 부정·제외 표현과 연결된 category/color/shelf_level, 대조·선택 표현이 포함된 destination, 복수·수량으로 물체가 한 개보다 많다고 명시된 요청은 해당 필드를 비워 둡니다. 결과에는 항상 `review_required: true`, 누락/경고가 포함되며 폼을 채울 뿐 작업을 만들거나 시작하지 않습니다. 색상과 선반 층은 각각 미보정 hint/명시적 ROI draft 비교만 검토용으로 표시하며 후보 필터·선택에는 사용하지 않습니다. 일반 영어 문장 이해는 지원하지 않습니다. LLM은 연결되지 않았습니다.

## 현재 동작의 경계

- 서버는 `127.0.0.1`에 바인딩합니다. 기본 실행은 모의 모드이며 로봇에 접속하지 않습니다.
- `CONTROL_STATION_ARM_JOINT_STATES_TOPIC`은 기본 미설정이며, 토픽 출처를 ROS graph에서 확인한 뒤에만 절대 이름으로 설정합니다. `/joint_states`는 현재 ROS inventory에서 주행 시뮬레이터 모델에도 사용되므로 이름만 보고 OMX 관절 상태로 간주하지 않습니다. 유효 샘플은 2초 기본 freshness 한계로 `observed`/`stale`을 표시하며 `CONTROL_STATION_ARM_JOINT_STATES_MAX_AGE`로 조정할 수 있습니다. name/position 길이, 선택적 velocity/effort 배열 길이, 이름과 유한 position 검증에 실패하면 freshness와 별도로 `invalid`로 표시합니다. `JointState`에는 joint type이 없어 숫자 position의 단위는 `UNSPECIFIED`이며 API가 임의로 라디안 표기를 붙이지 않습니다.
- 팔 제어는 기본 비활성화입니다. 활성화 시 LeRobot CLI 프로세스의 시작·생존·종료를 관리합니다.
- LeRobot CLI의 프로세스 생존은 장치 준비 완료나 작업 완료 신호가 아닙니다. 별도의 작업 완료 이벤트는 아직 없습니다.
- async inference client는 별도 LeRobot checkout `~/Desktop/lerobot`의 수정된 `robot_client.py`에서 `--max_action_steps`를 지원해야 합니다. 관제는 OpenCV와 OMX config 모듈을 먼저 import한 뒤 client를 실행하고, 이 소스 경로를 `PYTHONPATH`에 우선 추가합니다. 다른 장비/모델에서는 정책·카메라·액션 상한 환경변수를 별도로 검토해야 합니다.
- 카메라 API는 프레임 sequence·해상도·원본 BGR pixel buffer SHA-256·앱 내부 `pixel_transform_id`와 검출의 카메라 ID·캡처 시각·프레임 sequence·frame hash·transform ID·픽셀 중심/경계 상자 및 미보정 `color_hint`를 반환합니다. 관측 이벤트에는 캡처와 동기화된 관절 상태나 camera-to-base/tool extrinsic이 포함되지 않습니다. 색상 힌트는 bbox 안쪽 crop의 HSV rule 기반 pixel fraction이며 `hsv-inner-box-thresholds-v1` 버전을 기록하고, 확률이나 물체 색상 판정으로 보정되지 않았습니다. 관측 및 작업 이력 화면은 색상 추정·낮은 픽셀 비율·혼합색·계산 불가 상태와 해당 힌트 버전을 표시합니다. 요청 색상 비교에는 `exact-estimated-hint-match-v1` 비교 버전을 보존하며, 관측 및 작업 이력 화면에서 해당 버전을 표시합니다. 선택 색상 조건이 있으면 별도의 `color_evaluation` 요약을 추가하지만 원 후보 목록과 선택 결과는 바꾸지 않습니다. transform ID는 software geometry path만 식별하며 driver/firmware 설정은 확인하지 않습니다. stale 판정은 카메라 단위이며, 소비자는 프레임 fresh 상태와 detector 상태를 함께 확인해야 합니다. frame hash는 해당 픽셀 데이터를 저장하거나 재현할 수 있게 하지 않습니다.
- 대상 관측 API는 schema v2에서 `class_name`을 받아 `DETECTOR_NOT_CONFIGURED`, `DETECTOR_LOADING`, `DETECTOR_ERROR`, `CLASS_UNKNOWN`, `CAMERA_DISCONNECTED`, `CAMERA_STREAM_UNAVAILABLE`, `PIXEL_TRANSFORM_UNKNOWN`, `FRAME_PROVENANCE_UNKNOWN`, `FRAME_STALE`, `FRAME_UNAVAILABLE`, `TARGET_NOT_FOUND`, `SINGLE_CANDIDATE`, `AMBIGUOUS` 등으로 분류하고 화면에 표시합니다. 검출 모델이 제공한 클래스 이름을 입력 자동완성에 표시합니다. 카메라 연결/스트림이 오류이거나 stale 또는 frame hash/transform 출처 불일치인 경우 후보를 내보내지 않습니다. 단일 후보도 집기 가능 판정이 아니며, 이 경로는 관측 전용입니다.
- `TARGET_OBSERVED`의 observation snapshot은 `camera_connection_state`와 `camera_stream_state`의 원래 enum 값도 함께 보존해 연결 끊김·스트림 오류를 사후 확인할 수 있습니다.
- 별도 projection CLI가 대기 중 task의 fresh `TARGET_OBSERVED` event에서 명시된 candidate의 bbox 하단 중앙 proxy를 맞는 intrinsic/plane calibration으로 2D shelf-plane draft에 투영할 수 있습니다. 하단 중앙은 검증된 물체 지지점이 아니며, calibration fit/validation 및 intrinsic 재투영 지표를 결과에 함께 보존합니다. 입력 calibration의 대응점·RMS와 이미지별 intrinsic 재투영 기록도 산출 수치와 일치하는지 재계산합니다. 이는 내부 일관성 확인이며 자동 허용 오차는 정하지 않았습니다. 실제 보정/event 자료가 없어 실제 후보에 적용하지 않았습니다. 범위 밖 anchor는 거절하고 결과는 `.control-station` 초안으로만 저장합니다.
- `project_observation_to_shelf_plane.py --record-review-event`를 명시했을 때만 저장된 JSON 초안의 경로/hash를 localhost API에 제출합니다. 서버는 관리 디렉터리 안의 JSON, task의 pending 상태, 원본 관측·선택 event와 step provenance, 보정 참조, 2D 좌표 계약을 재검증하고 `TARGET_PLANE_PROJECTION_RECORDED`를 pending `ESTIMATE_TARGET_POSE`에 연결합니다. 이는 사람이 검토할 수 있는 2D 투영 초안 이력이며 위치 추정 단계의 완료가 아닙니다. UI에도 proxy 좌표와 로봇 목표 사용 불가를 표시합니다. API는 task/step 상태를 바꾸거나 명령을 만들지 않습니다.
- 작업 관측 기록 endpoint는 task가 `PENDING`일 때만 기록합니다. task 상태 전이를 같은 트랜잭션 경계에서 재확인해 작업이 진행된 뒤의 늦은 관측을 거부합니다. 기록된 픽셀 후보는 추적 가능한 관측 근거이며, 깊이/TF/색상/선반 추론이나 실행 허가가 아닙니다.
- Pinky 주차 판정은 C++로만 합니다. 등록과 단독 판정은 CLI `pinky_parking`, 관제는 `CONTROL_STATION_PARKING_SPOT`을 지정했을 때 C++ 판정 노드(`ceiling_parking.launch.py`)에 묻습니다([실행 방법](runbook.md#c-주차-판정-노드로-관제-실행)). 이때 모의 작업의 `VERIFY_ARRIVAL` 단계가 그 주차 위치를 판정해 `PARKING_OBSERVED` event로 남기고, `ARRIVED`일 때만 성공합니다. `MISALIGNED`(`PARKING_MISALIGNED`)나 판정 불가(`PARKING_NOT_VERIFIED`), 판정을 시작하지 못한 경우(관측 오류 코드)에는 단계와 작업이 `FAILED`가 되고 팔 단계는 실행하지 않으며, 자동 재주행은 없습니다. 작업 이력 화면에 판정 결과를 표시합니다. 판정 노드가 꺼져 있거나 응답하지 않으면 판정 불가로 팔 단계를 시작하지 않습니다. 주차 위치를 지정하지 않으면 `VERIFY_ARRIVAL`은 기존처럼 모의 성공입니다. `ARRIVED`는 등록 기준 대비 마커 위치가 허용 범위 안이라는 관측이며, Nav2 이동 성공을 대신하지 않고 그 뒤의 추가 조건으로만 씁니다. 천장 카메라는 Head Camera와 같은 실물 카메라(HCAM0)라서 한 프로그램만 열 수 있습니다. 판정 노드 구성에서는 노드의 캡처 컴포넌트가 열고 관제는 `CONTROL_STATION_HEAD_CAMERA=ros:/ceiling_camera/image_raw/compressed`로 영상을 받으며, CLI를 쓰려면 노드를 먼저 끕니다. 2026-11-02 현장에서 이 카메라는 열릴 때 가끔 영상 전체가 가로로 96 px 말린 채 세션 내내 유지됐습니다. 고정 마커로 등록한 주차 판정은 이를 `VIEW_UNVERIFIED`로 거부하지만, `--no-anchors` 등록은 감지하지 않습니다. 19:36 이후 다른 HCAM0이 포트 10에 연결되어 Head Camera 기본 by-id 경로가 천장 카메라가 아닌 그 카메라를 가리킵니다. 정지 상태에서 측정한 반복성은 1 mm·0.1° 수준이고, 재주차 반복성은 측정하지 않았습니다. 현재 `pinky_dock_1`은 팔 나무판 위의 1번 고정 마커로 시야를 확인하도록 등록돼 있습니다. 마커 장착 방향은 `-x`(Pinky를 밀어 확인), 허용 오차는 임시 25 mm/5°입니다(2026-11-03에 50 mm/10°에서 좁힘). 관제에 올릴 결과 형식은 [주차 정렬 관측 계약 초안](../spec/interface-contracts.md#pinky-주차-정렬-관측-초안)에 있습니다. 실행 방법과 측정 기록은 [실행 방법](runbook.md#천장-카메라-pinky-주차-판정)을 참고합니다.
- `probe_omx_readonly.py`는 수동 실행 시 실제 follower 레지스터를 읽지만, 관제의 READY 상태로 소비하지 않습니다. 2026-10-30 현장 probe에서 `/dev/omx_follower`의 ID/model handshake와 위치·torque·error·mode·limit register 읽기가 성공했습니다. 당시 torque는 모두 0, hardware error는 모두 0, 팔 다섯 관절 mode는 4, gripper mode는 5였습니다. 이는 당시 값이며 안전 준비나 현재 상태를 보장하지 않습니다.
- 로컬 LeRobot OMX-F 소스에서 `OmxFollower.get_observation()`은 현재 관절 상태를 읽고, `send_action()`은 `*.pos` 목표를 `Goal_Position`으로 씁니다. 자동 작업 완료나 파지 성공 결과를 반환하지 않습니다. `OmxFollower.connect()`는 보정 불일치 시 보정값을 쓰고 모터 설정을 적용할 수 있으므로 read-only readiness probe로 사용하지 않습니다. 기본 config의 `max_relative_target`은 제한값이 없을 수 있습니다.
- 텔레오퍼레이션 시작은 저장된 관제 작업을 실행하지 않습니다. `/api/tasks`는 계속 `PENDING`만 저장하며, 별도 `/start` 요청만 mock 작업 실행을 시작합니다.
- 시작 요청은 한 서버 프로세스 안에서 활성 작업 하나만 허용합니다. 같은 작업의 반복 시작은 추가 worker를 만들지 않고 현재 상태를 반환합니다. 취소 완료는 worker가 진행 중 단계를 멈춘 뒤 기록합니다. 재시작 뒤 worker가 없는 활성 task는 자동 재개나 명령 재전송을 하지 않습니다.
- 작업 상세의 `local_execution_owner`는 현재 task/프로세스 상태에서 `NOT_ACTIVE`, `LOCAL_WORKER_ACTIVE`, `NO_LOCAL_WORKER`, `MODE_NOT_MANAGED` 중 하나를 계산해 반환합니다. 저장되거나 장치 상태로 해석되지 않으며, 활성 상태와 로컬 worker 부재가 함께 나타나면 화면에서 운영자 확인이 필요하다고 표시합니다.
- 시작 전 대기 작업의 즉시 취소는 task terminal state와 모든 pending step의 `NOT_EXECUTED` 결과/event를 하나의 DB transaction으로 저장합니다. 진행 중 mock worker 취소는 worker가 정지를 확인한 뒤 완료로 기록합니다.
- mock 단계 시작의 `COMMAND_SENT`/`ACCEPTED`/`RUNNING` 기록은 취소 요청과 같은 프로세스 lock으로 직렬화합니다. 취소 상태가 먼저 기록되면 해당 단계 command event를 만들지 않고, 단계 시작 기록이 먼저면 취소는 그 뒤에 접수됩니다.
- mock worker가 단계 오류를 기록하면 남은 pending step은 `CANCELED`와 `NOT_EXECUTED_AFTER_FAILURE`로 남기고 task를 `FAILED`로 마무리합니다. 오류 종결 처리는 취소 요청과 같은 프로세스 lock으로 직렬화합니다. 이는 mock 경로의 상태 일관성만 다루며 프로세스 중단 복구, 장치 timeout, 실제 정지 확인을 구현한 것은 아닙니다.
- `CONTROL_STATION_MOCK_FAIL_STEP`은 기본 비활성인 mock 전용 실패 주입 설정입니다. 지정 step은 `MOCK_SCENARIO_FAILURE` event/error code로 실패하고 후속 pending step은 실행되지 않습니다. 이는 장치 오류나 실제 단계 결과를 나타내지 않습니다.
- 이동·팔 작업은 `VERIFY_ARM_RESULT` 뒤에 관제 단계 `VERIFY_LOAD`(적재 확인, `robot_id: null`)를 둡니다. 2026-11-03 이후 만든 작업부터 7단계이며, 이전에 저장된 작업은 원래 6단계를 유지합니다. 적재 관측기(`src/control_station/load_check.py`)가 연결되면 수레 위 개수를 기대 수량(작업당 1개 고정)과 비교해 `LOAD_OBSERVED` event를 남기고, 같을 때만 성공(`LOAD_CONFIRMED`)합니다. 적으면 `LOAD_INCOMPLETE`, 많으면 `LOAD_EXCESS`, 개수가 없으면 `LOAD_NOT_VERIFIED`로 작업이 `FAILED`가 되고 `FINALIZE_TASK`는 실행하지 않습니다. 현재 관측기는 `CONTROL_STATION_MOCK_LOAD_OBSERVED`로 켜는 모의 관측기뿐이며 카메라를 열지 않습니다. 천장 카메라 YOLO 적재 관측기는 미구현입니다. 설정하지 않으면 `VERIFY_LOAD`는 모의 성공입니다.
- 새 mock 작업의 로봇 step 및 command event는 동일한 설정 mock `robot_id`(`mobile-01`/`arm-01`)를 사용합니다. 기존 DB의 `MOBILE`/`ARM` step 값은 API 응답에서 canonical ID로 정규화하고, 관제 자체 단계는 `robot_id: null`로 둡니다. 실행 전 취소된 단계의 `command_id`도 null입니다. DB event ID와 시각을 보존하지만 장치 sequence나 ROS action goal ID를 생성하지 않으며, mock ID는 실제 장치 이벤트의 증거가 아닙니다.
- mock 단계는 실행 대기 중 task 상태가 `VALIDATING`/`RUNNING`이고 `active_step_id`, step 상태, `command_id`가 현재 명령과 일치하는지 저장소에서 확인한 뒤 `PROGRESS` 이력을 한 번 기록합니다. 기본 진행률 50%는 모의 지연 구간을 표시할 뿐 장치 feedback이나 실측 작업량이 아닙니다. 이 이벤트는 task/step 상태를 바꾸지 않습니다.
- `pick_request`가 없는 새 작업은 기존 이동·팔 작업 단계 순서를 유지합니다. 대상 조건을 포함한 새 작업은 Pick & Place 단계 템플릿을 사용합니다. 과거 작업에 단계를 추가/재해석하지 않습니다.
- 기존 설정은 `/dev/video4`를 Joint Camera로, HCAM0 by-id 경로를 Head Camera로 사용했습니다. 2026-10-31 read-only V4L2 열거에서 HCAM0는 `/dev/video4`, Innomaker U20CAM-720P는 `/dev/video6`이므로 numeric 기본 경로는 두 슬롯을 같은 장치로 만들 수 있었습니다.
- 이전에 `/dev/video4`로 선택했던 Joint Camera 장치가 Innomaker였다는 기록을 따라 Joint 기본값을 Innomaker by-id 경로로 바꿨습니다. `OpenCVArmCameraStreamManager`는 두 slot의 realpath가 같으면 둘 다 열지 않고 경로 충돌 오류를 표시합니다. Head/Joint의 물리 장착 역할은 최종 육안 확인 전이며, 배포 시 관제 화면과 장착 위치를 대조해야 합니다.
- read-only V4L2 조회에서 HCAM0는 MJPG 1280×720 capture node로 열거됐지만 1-frame stream file은 JPEG frame으로 해석할 수 없었습니다. Innomaker는 MJPG 1280×720 stream을 내보냈고, 저장한 단일 frame에는 가까운 평면과 가장자리의 붉은/검은 물체 일부가 보였으나 선반 장면과 물리 장착 역할은 판별되지 않았습니다. RGB 프레임은 측정표 아래 `.control-station/physical-camera-readonly/`에만 있으며 저장소 추적 대상이 아닙니다.
- SQLAlchemy ORM과 Alembic revision이 SQLite schema를 관리합니다. 기존 task 열과 행을 보존하고 task steps/events 테이블을 추가하며, 과거 단계·이벤트는 만들어내지 않습니다.
- 새 작업 생성은 작업 행·표준 단계 6개·`TASK_CREATED` 이벤트를 단일 트랜잭션으로 저장합니다. 생성 응답에서 단계와 이벤트를 확인할 수 있습니다.
- Alembic revision `0002_pick_request`가 기존 작업 행을 보존하면서 nullable JSON 대상 명세 열을 추가합니다. 기존 작업의 `pick_request`는 `null`입니다.
- 작업·단계 상태 전이는 저장소 서비스에서 허용 전이를 검증하고 이벤트와 단일 트랜잭션으로 기록합니다. mock 실행 오류의 code/message는 task 요약과 이력 화면에도 보입니다. mock 전용 오케스트레이터가 이 경로에 연결됐습니다. 실제 장치 adapter, 장치 이벤트 수신기, 실행/취소 결과 계약은 아직 구현되지 않았습니다.

## 다음 작업

우선순위와 완료 조건은 [개발 로드맵](../development-plan.md) 및 각 에픽 문서에서 관리합니다.
