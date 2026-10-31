# 데이터·인터페이스 계약 초안

이 문서는 주문, 관제 작업, 로봇 상태, 명령 및 결과 이벤트의 데이터 의미와 경계를 정리합니다. 목표 시스템 구성은 [데이터 흐름도](data-flow.md), 현재 코드가 제공하는 기능은 [구현 현황](../implementation/current-state.md)을 참고하세요.

주문 흐름과 상태는 [Confluence 시스템 요구사항](https://techinwoong.atlassian.net/wiki/spaces/JJJS/pages/1998932/System+Requirements), 추적 티켓은 [JJS-15](https://techinwoong.atlassian.net/browse/JJS-15)입니다. 장치에서 확인되지 않은 필드·프로토콜은 **미확정**으로 표시하며 구현 계약으로 간주하지 않습니다.

## 1. 식별자와 데이터 경계

| 식별자 | 의미 | 생성·소유 주체 | 규칙 |
|---|---|---|---|
| `order_id` | 고객 주문 | 주문 서비스 | 주문 상태·재고 예약·픽업 위치에 연결합니다. 표시는 요구사항의 `TK-날짜-순번` 형식을 따릅니다. |
| `task_id` | 관제가 실행 순서와 결과를 추적하는 작업 | 관제 저장소 | 이동·팔 단계를 연결합니다. 한 주문이 여러 운송 작업으로 나뉠 수 있으나, DB 관계와 분할 정책은 미확정입니다. |
| `step_id` | 작업 내 단계 | 관제 저장소 | 한 `task_id` 안에서 식별하고 `sequence`로 순서를 보존합니다. |
| `command_id` | 장치 명령 시도 | 오케스트레이터 | 명령 수락·결과 추적의 상관관계 키입니다. 새 재시도는 별도 명령 ID를 사용하며 결과 불명 명령은 자동 재전송하지 않습니다. |
| `event_id` | 수신 이벤트 | 이벤트 생산자 | 중복 반영 방지 키입니다. 장치가 제공하지 않으면 어댑터의 중복 제거 방식이 정해지기 전까지 중복 제거를 보장하지 않습니다. |
| `robot_id` | Pinky 또는 OMX 식별자 | 장치/관제 설정 | 장치 종류, 상태, 명령의 대상을 구분합니다. |

현재 코드의 `Task.destination`과 `Task.arm_job`은 입력 문자열입니다. 선택 필드 `pick_request`는 `category`(필수), `color`(선택), `shelf_level`(선택 1–999,999,999 정수)를 보존하지만, 검출된 대상 ID·물체 위치/자세, 주문 픽업 위치 또는 OMX 장치 명령 payload로 해석되지 않습니다.

## 2. 상태 계약

주문, 관제 작업, 단계, 로봇은 각각 별도 리소스입니다. 한쪽 상태를 다른 쪽의 완료 근거로 사용하지 않습니다.

### 주문 상태

아래 영문 표기는 제품 요구사항의 한글 상태를 표현하기 위한 관제 초안 enum입니다. 주문 서비스의 실제 wire 값과 보류 해제 전이는 아직 합의되지 않았습니다.

| 상태 | 전환 근거 | 고객 표시 |
|---|---|---|
| `RECEIVED` | 주문 검증·저장과 재고 예약 성공 | 접수 |
| `PREPARING` | 주문의 운송 준비 시작 | 준비 중 |
| `LOADING` | B 구역에서 적재 시작 | 적재 중 |
| `IN_TRANSIT` | 목표 수량의 적재 성공이 확인된 뒤 Pinky 출발 | 운송 중 |
| `AWAITING_PICKUP` | 선택된 A 픽업 위치 도착 확인 | 수령 대기 |
| `PARTIALLY_COMPLETED` | 수령 확인 후 미수령 운송분이 남음 | 부분 완료 |
| `COMPLETED` | 모든 운송분의 수령 확인 | 완료 |
| 보류 (초안 enum `ON_HOLD`) | 인식·정렬·집기·적재 실패, timeout 또는 통신 이상 | 내부 사유 대신 `DELAYED`(지연)로 표시 |

상태 이름은 제품 요구사항의 의미를 나타냅니다. 주문 서비스와 관제 API는 아직 연결되지 않았으므로 JSON 필드·wire 값·동시 갱신 책임은 **미확정**입니다. 수령 확인은 해당 운송분에서 한 번만 반영합니다.

### 관제 작업·단계 및 로봇 상태

관제 `Task`와 `TaskStep`은 [시스템 명세의 상태 전이](system-spec.md#4-상태와-전이-규칙)를 따릅니다. 주문의 `ON_HOLD`는 관제 작업의 `NEEDS_INTERVENTION`과 동일한 enum이 아닙니다. 고객 화면에는 내부 오류 사유를 직접 노출하지 않습니다.

| 로봇 필드 | 값 | 생산자 | 의미 |
|---|---|---|---|
| `connection_state` | `CONNECTED`, `DISCONNECTED`, `UNKNOWN` | 로봇 어댑터 | 마지막 통신 관측에 따른 연결 여부 |
| `readiness_state` | `READY`, `BUSY`, `NOT_READY`, `UNKNOWN` | 로봇 어댑터 | 새 명령 수락 가능 여부 |
| `execution_state` | `IDLE`, `NAVIGATING`, `EXECUTING`, `STOPPING`, `UNKNOWN` | 로봇 어댑터 | 확인된 실행 상태 |
| `last_heartbeat_at` | UTC 시각 또는 `null` | 상태 관리자 | 마지막 유효 상태 관측 시각; stale 한계는 측정 후 결정 |
| `active_task_id` | 작업 ID 또는 `null` | 오케스트레이터 | 현재 로봇과 연결된 작업 |

HTTP 서버 생존, 카메라 프레임 수신, Nav2 action server 발견, LeRobot 프로세스 생존만으로 로봇 `READY`나 작업 완료를 판단하지 않습니다.

## 3. 생산자·소비자와 결과 신호

| 생산자 → 소비자 | 데이터 | 결과·실패 신호 | 합의 상태 |
|---|---|---|---|
| 고객 화면 → 주문 서비스 | 물품, 양수 수량, 픽업 위치 `A-1`/`A-2`/`A-3` | 주문 ID 반환; 검증·재고 실패 시 저장하지 않고 사유 반환 | 제품 요구사항 확정; 이 저장소와 미연동 |
| 주문 서비스 → 관제 | 주문 ID, 물품·수량, 선택 픽업 위치, 배정 운송 작업 | 배정/대기 또는 거절·보류 사유 | 의미는 요구사항에 있음; endpoint·전송 방식·schema 미확정 |
| 관제 UI → HTTP API | 작업 생성·조회, 구조화된 대상 조건, 카메라 관측 기록, mock 작업 시작·취소 | HTTP 접수와 작업 ID; 관측은 task event이며 상태 전이 아님; 완료/취소는 별도 상태·이벤트 | localhost 구현; `pick_request`는 저장 전용, 관측 이벤트는 픽셀 후보 근거, `/start`와 `/cancel`은 mock 전용, 실제 장치 실행 미연결 |
| Mock start/cancel API → UI/client | task-control 결과 또는 오류 | 오류는 `error_code`, 사용자용 `message`, 호환용 `error`를 함께 반환 | `TASK_NOT_FOUND`, `TASK_STATE_CONFLICT`, `TASK_NOT_STARTABLE`, `TASK_NOT_CANCELLABLE`, `TASK_WORKER_OWNERSHIP_UNCERTAIN`, `EXECUTION_MODE_UNSUPPORTED`, `MOCK_WORKER_START_FAILED`, `TASK_CONTROL_ERROR`, `CROSS_ORIGIN_REQUEST_REJECTED`; 다른 API는 아직 공통 오류 envelope 적용 전 |
| Mock 오케스트레이터 → 작업 이력 저장소 | 현재 task/active-step/step/command/robot ID와 mock progress | task가 `VALIDATING`/`RUNNING`이고 같은 step이 `active_step_id` 및 `RUNNING` 상태이며 command ID가 일치할 때만 `PROGRESS` event를 추가; task/step 상태는 유지 | mock 지연 구간의 50% 표시; 장치 feedback이나 실측 작업량 아님; 모든 상관 ID와 상태 선행 조건을 저장소가 확인 |
| 오케스트레이터 → Pinky 어댑터 | task/step/command ID, 목적지, deadline | 수락/거절, 진행, terminal 성공·실패, 취소 결과 | 공통 의미 확정; ROS 2 action 이름·타입·namespace·frame 미확정 |
| 오케스트레이터 → OMX 어댑터 | task/step/command ID, 작업 종류·파라미터 | 수락/거절, 진행, 파지·적재 최종 결과 또는 실패 | 공통 의미 확정; 관제 자동 명령은 미연결. 로컬 LeRobot `OmxFollower.send_action()`은 `*.pos` 관절 목표를 쓰지만 task 수락/terminal 파지 결과를 제공하지 않음 |
| 로봇 어댑터 → 이벤트 처리기 | robot/task/step/command ID, 장치 결과 | `ACCEPTED`, `EXECUTION_STARTED`, `PROGRESS`, `SUCCEEDED`, `FAILED`, `CANCELED` 또는 미확정 | 공통 이벤트 의미 확정; 장치별 실제 메시지 미확정 |
| Head/Joint Camera → 관제 UI | 카메라별 프레임·연결·스트림·프레임 sequence/해상도·마지막 프레임 시각, closed-set YOLO 또는 선택 YOLO-World의 camera/frame 출처·고정 prompt vocabulary·픽셀 영역 | 카메라별 오류; 다른 카메라나 로봇 상태에 전파하지 않음; 검출 후보는 자동 선택/명령 신호가 아님 | API 관측 필드 구현; 가중치 미포함; 실제 장착 위치·실물 객체별 검출 적합성은 미검증 |
| 설정된 ROS 2 `sensor_msgs/msg/JointState` → `/api/robots` | 명시적으로 설정한 topic의 관절명·유효한 position, ROS 메시지 원본 `header.stamp`의 sec/nanosec와 `frame_id`, 관제 수신시각/나이 | 미설정, 아직 미수신, fresh `observed`, stale, 또는 배열 불일치·이름 오류·비유한 위치가 있는 `invalid` 관측 | ROS 2 상태 adapter의 읽기 전용 subscription; topic source는 운영자가 확인해야 하며 joint-state 수신은 arm readiness/safety/command acceptance가 아님; ROS goal/command 전송 없음 |
| 선택 설정 ROS 2 `FollowJointTrajectory` action → `/api/robots` | absolute action name과 ROS graph의 server 발견 여부 | `unconfigured`, `not_discovered`, `discovered` | 선택형 discovery-only `ActionClient`; goal을 보내지 않으며 server discovery는 controller/arm readiness, safety 또는 command acceptance가 아님 |

`/api/robots`의 `source_stamp`는 메시지에 실려 온 숫자를 그대로 보존하며 ROS clock 설정, 유효성, 실제 샘플 시각을 검증하지 않습니다. 0 또는 null일 수 있고 관제 `received_at`과 같은 시간축이라고 보장하지 않습니다. 카메라 캡처 시각과의 차이를 계산하거나 joint pose를 카메라 관측에 결합하는 데 사용하지 않습니다. `frame_id`도 원문 메타데이터이며 TF 변환을 수행했다는 뜻이 아닙니다.

샘플은 joint name/position 개수 일치, 선택적 velocity/effort 배열이 비어 있거나 name 개수와 일치하는지, 비어 있지 않은 유일한 이름, 유한한 모든 position을 확인합니다. 실패하면 `state: "invalid"`, `sample_valid: false`, 설명용 `sample_error`를 반환하고 `positions`는 `null`입니다. 검증에 성공했을 때만 `sample_valid: true`와 원본 숫자 배열을 반환하며, `position_unit_status`는 `UNSPECIFIED`입니다. ROS `JointState`에서 position 단위는 revolute joint는 rad, prismatic joint는 m이지만 메시지 자체에 joint type이 없으므로 관제는 단위를 임의로 붙이지 않습니다 ([Jazzy `JointState.msg`](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg)). 이는 관절 한계·모터 상태·안전 상태를 검증하지 않습니다.

## 4. 명령·이벤트 공통 envelope

다음은 **관제 내부 논리 계약 예시**입니다. Pinky나 OMX가 이 JSON을 그대로 수신한다는 뜻은 아닙니다. `parameters` schema, deadline 정책, 실제 wire protocol은 장치 조사 후 정합니다.

```json
{
  "task_id": "task-123",
  "step_id": "step-2",
  "command_id": "cmd-8b17",
  "robot_id": "pinky-01",
  "command_type": "NAVIGATE",
  "parameters": { "destination_id": "A-1" },
  "deadline_at": null
}
```

```json
{
  "event_id": "evt-91c2",
  "robot_id": "pinky-01",
  "task_id": "task-123",
  "step_id": "step-2",
  "command_id": "cmd-8b17",
  "event_type": "SUCCEEDED",
  "occurred_at": null,
  "received_at": "2026-10-28T10:00:01Z",
  "sequence": null,
  "payload": {}
}
```

- `occurred_at`은 장치 발생 시각, `received_at`은 관제 수신 시각입니다. 장치가 발생 시각이나 sequence를 주지 않으면 `null`로 두고 수신 순서를 장치 실행 순서로 간주하지 않습니다.
- `ACCEPTED`는 명령 접수일 뿐 실행 완료가 아닙니다. `PROGRESS` 역시 terminal 결과를 대체하지 않습니다.
- 결과는 현재 활성 명령과 `robot_id`, `task_id`, `step_id`, `command_id`가 모두 일치할 때만 반영합니다. 중복 `event_id`는 한 번만 반영합니다.
- 새 mock 작업의 step과 event는 설정된 mock `robot_id`(`mobile-01`/`arm-01`)를 동일하게 사용하고, 관제 자체 단계는 `robot_id: null`입니다. 기존 DB에 남은 `MOBILE`/`ARM` step 종류값은 API 응답에서 대응하는 ID로 정규화합니다. DB event ID와 task/step/command ID도 이력에 기록하지만, 이 mock 식별자는 장치 sequence나 ROS action goal ID를 생성하지 않으며 실제 장치가 제공한 식별자로 간주하지 않습니다.
- OMX 적재 수량은 파지와 적재 완료가 모두 확인된 경우에만 증가합니다. 이동·적재 실패는 주문 완료로 전이되지 않습니다.
- timeout은 deadline 내 결과 미수신을 뜻합니다. 장치가 멈췄거나 실패했다고 추정하지 않고, 결과 불명 시 `UNKNOWN`/`NEEDS_INTERVENTION`으로 두며 자동 재전송하지 않습니다.
- 실패·무응답 뒤 후속 로봇 명령은 멈춥니다. 장치 상태와 안전한 재개 지점을 확인한 운영자의 명시적 재개가 필요합니다.

## 5. 좌표 계약

좌표값만으로 명령을 만들지 않습니다. 좌표에는 최소한 frame과 단위를 명시해야 합니다.

| 데이터 | 좌표계·단위 | 결정 상태 |
|---|---|---|
| 카메라 검출 위치 | 이미지 `(u, v)` / `pixel`, 카메라 ID와 프레임 시각 동반 | 픽셀 표현은 개념상 정함; 해상도·원점·crop/resize 보정은 카메라별 확인 필요 |
| OMX 집기 목표 | OMX가 명령에서 요구하는 기준 frame과 길이 단위 | **미확정**: frame 이름, 축 방향, 원점, 허용 작업 범위를 문서/장치로 확인해야 함 |
| Pinky 목적지 | Nav2가 요구하는 map frame과 pose 단위 | **미확정**: map/frame, yaw 표현, waypoint/action schema 조사 필요 |
| 카메라→OMX 변환 | 카메라 calibration과 OMX 기준 frame 사이 변환 | 현재 MVP 범위 밖; 변환 오차·허용값은 실험 후 합의 필요 |

추후 좌표 명령에는 `source_camera_id`, `captured_at`, 원본 해상도, source/target `frame_id`, 단위와 calibration 버전을 연결해야 합니다. 정확한 필드명은 공급자 인터페이스를 확인한 뒤 고정합니다. frame·단위·calibration이 없거나 작업 범위를 벗어난 좌표는 집기 명령으로 보내지 않습니다.

현재 오프라인 `project_observation_to_shelf_plane.py`는 명시된 observation event candidate의 detection bbox 하단 중앙을, calibration correspondence 범위 안에서 고정 선반 평면의 2D `DRAFT_ONLY`로 투영합니다. 후보를 CLI index로 지정하면 기본 2초의 frame age 제한(최대 60초까지 명시 조정)을 적용합니다. 운영자 `TARGET_SELECTED` event를 참조하면 존재하는 `step_id`가 pending `SELECT_TARGET`를 가리키는지 확인하고 task/frame/request/candidate snapshot과 저장 PNG file/pixel hash를 검증한 뒤, 캡처 당시 fresh였던 정확한 스냅샷을 경과 시간 제한 없이 오프라인 투영하고 review-only freshness policy 및 나이를 결과에 기록합니다. task는 두 경우 모두 `PENDING`이어야 합니다. event ID가 task 이력에 유일하게 존재해야 하며 존재하는 `step_id`는 pending `DETECT_TARGETS`와 일치해야 합니다(과거 null link는 허용). event 안의 `pick_request`는 task에 저장된 명세와 일치해야 합니다. 선택 후보의 event/camera/frame/`pixel_transform_id` 출처와 plane schema v4에 기록된 대응점 입력 및 각 source image의 SHA-256도 교차 확인합니다. transform ID는 앱 내부 geometry 경로만 식별합니다. 이 anchor는 실제 물체 지지점을 보증하지 않는 proxy임을 결과에 기록하고 calibration fit/validation 지표와 provenance를 함께 보존합니다. 잔차의 자동 합격 기준은 없습니다. 색상/선반 조건이 있으면 미적용 조건으로 남기며, 평면 `(x,y)`는 OMX base/tool pose의 `(x,y,z)`가 아니고 작업 명령으로 직접 사용할 수 없습니다.

### 작업 생성과 대상 명세 (현재 API)

`POST /api/tasks`는 `destination`, `arm_job`, 선택적 `pick_request`만 받습니다. 나머지 최상위 필드는 HTTP 400으로 거부해 필드명 오타나 아직 지원하지 않는 값을 조용히 무시하지 않습니다. `destination`과 `arm_job`은 필수 문자열이고, `pick_request`는 생략/`null`이거나 `{category, color?, shelf_level?}` 계약을 따라야 합니다. `category`는 1–80자, `color`는 1–40자이며 두 필드 모두 ASCII C0 및 DEL 제어 문자를 포함할 수 없습니다. 성공 응답은 저장된 `PENDING` 작업이며 인식·작업 단계 실행·로봇 명령을 시작하지 않습니다.

### 카메라 관측 결과 (현재 API)

schema v2/v3의 observation object는 `camera_connection_state`와 `camera_stream_state`에 관측 당시 카메라 연결/스트림 enum을 보존합니다. `GET /api/cameras/{camera_id}/target-observation` 응답과 `TARGET_OBSERVED` task event의 중첩 observation에 모두 포함되며, 판정 status만으로 원래 장치 상태를 추정할 필요가 없습니다.

`GET /api/cameras` 응답의 각 카메라에는 `frame_sequence`, `frame_width`, `frame_height`, `frame_sha256`, `pixel_transform_id`, `last_frame_at`, `frame_freshness`가 포함됩니다. MuJoCo mode RGB subscriber는 추가로 `source_frame_id`와 ROS `source_stamp: {sec, nanosec}`를 내보냅니다. 이는 sensor message header의 원문 숫자이며 관제 수신 시각/ROS clock과 같은 시간축이라고 보장하지 않습니다. `frame_sequence`는 해당 관제 프로세스의 캡처 카운터이며 첫 프레임은 1입니다. MuJoCo mode에서만 `depth_observation`은 고정 depth topic의 encoding, dimensions, payload byte count, 원본 stamp/frame ID와 2초 수신 freshness를, `camera_info_observation`은 고정 CameraInfo topic의 D/K/R/P, distortion model, dimensions, binning/ROI, 원본 stamp/frame ID와 freshness를 반환합니다. Point cloud가 선택적으로 실행되면 `point_cloud_observation`은 `/mujoco_omx/fixed_camera/points`의 XYZ field layout, dimensions, point/row stride, payload byte count, source stamp/frame, freshness 및 최신 RGB header/dimensions 필드 비교 상태를 기록합니다. point cloud payload 자체는 `/api/cameras`에 넣지 않습니다. 메시지 검증은 형식·크기·유한 숫자 검증이며 캘리브레이션 정확성 확인이 아닙니다. 각 관측의 `rgb_header_dimension_match`는 최신 RGB 대비 source stamp/frame ID 및 dimensions의 필드 동일 여부만 `HEADER_AND_DIMENSIONS_MATCH`, `HEADER_MISMATCH`, `DIMENSIONS_MISMATCH`, `RGB_UNAVAILABLE`, `SOURCE_UNAVAILABLE`로 표시합니다. 이는 transport/sample synchronization을 보증하지 않습니다. depth, CameraInfo와 point cloud는 최신 메시지별 기록이고 depth unit/invalid-pixel semantics, camera calibration validity 또는 object segmentation을 보장하지 않습니다. `calibration_use: OBSERVATION_ONLY_UNVALIDATED`는 CameraInfo를 검증된 보정으로 사용할 수 없음을 나타냅니다. MuJoCo mode는 추가로 `tf_observation`에 RGB 원본 stamp에서 fixed camera frame→`mujoco_omx/link0` lookup 결과를 `AVAILABLE`, `LOOKUP_FAILED`, `NO_RGB`, `RGB_STALE` 상태로 노출합니다. 성공 시 source/target frame, 요청 RGB stamp, TF message stamp, translation/rotation, RGB host 수신 age를 기록합니다. 결과는 `transform_basis: SIMULATION_ONLY_PLACEHOLDER`, `target_pose_use: false`로 표시해 시뮬레이션 카메라 장착 변환의 provenance로만 취급합니다. 이는 실물 외부보정, RGB-D sample synchronization, image-time-aligned joint state 또는 물체 위치를 검증하지 않으며 target/object pose나 grasp target은 계산하지 않습니다. Physical camera에서는 depth, CameraInfo, point cloud, TF 관측 필드가 null입니다. Physical camera의 `frame_sha256`는 OpenCV에서 decode한 원본 BGR 픽셀 C-order bytes hash입니다. MuJoCo mode에서는 ROS `rgb8`을 BGR8 contiguous pixel로 채널 변환한 뒤, spatial transform 없이 그 bytes를 hash합니다. `opencv-v4l2-capture-no-app-geometry-v1`과 `mujoco-ros-rgb8-to-bgr8-no-app-geometry-v1` ID는 이 서로 다른 입력 경로를 구분합니다. 두 hash 모두 프레임을 저장하거나 복원하지 않습니다. 해당 카메라의 detector가 구성된 경우 `detector_confidence_threshold`, `detector_weights_sha256`, `detector_inference_image_size`, `detector_device`, `detector_ultralytics_version`도 반환합니다. threshold는 해당 process가 모델 추론에 전달하는 값이고 확률 보정 기준은 아닙니다. 모델 SHA-256은 로드 전후 파일 hash가 일치한 경우에만 제공합니다. 이미지 크기는 `predict()`의 `imgsz`, device는 설정값이며 null이면 backend 자동 선택 요청입니다. 실제로 선택된 device를 나타내지 않습니다. Ultralytics version은 설치 package metadata에서 읽습니다. `detections` 각 항목은 `class_name`, `confidence`, `center_px`, `bbox_xyxy_px`, `color_hint`와 함께 `camera_id`, `captured_at`, `frame_sequence`, `frame_sha256`, `frame_width`, `frame_height`, `pixel_transform_id`를 가집니다. `color_hint`는 bbox 안쪽의 HSV threshold별 픽셀 비율로 산출하는 `UNCALIBRATED_HSV_PIXEL_FRACTION`이며 `algorithm_version: hsv-inner-box-thresholds-v1`, `status`, `color`, `pixel_fraction`, `sampled_pixel_count`, `review_required: true`, `used_for_selection: false`를 포함합니다. `pixel_fraction`은 색상 분류 확률이 아닙니다. 검출 위치·조명·반사·배경 영향을 보정하지 않으므로 자동 색상 조건 필터 또는 대상 선택 근거로 사용할 수 없습니다. transform ID는 입력 경로를 구분하지만 camera driver/firmware 물리 회전, crop, extrinsic을 증명하지 않습니다. 현재 관제 API는 image-time-aligned joint state를 반환하거나 point cloud를 base/tool frame으로 재표현하지 않습니다. 실제 영상 좌표를 base frame으로 바꾸려면 센서 보정 및 시간/TF provenance가 별도로 필요합니다. 픽셀 원점은 영상 좌상단이며 `bbox_xyxy_px` 순서는 `[x1, y1, x2, y2]`입니다.

색상 힌트는 bbox 중앙 60% crop에 OpenCV HSV를 적용합니다. black은 `V<=45`, white는 `S<=45 and V>=200`; red/yellow/green/blue는 각각 `S>=65`, `V>=50`이며 H 범위는 red `<=10 or >=170`, yellow `16–37`, green `38–85`, blue `90–130`입니다. 가장 많은 지원 색상이 crop 전체 픽셀의 20% 미만이면 `LOW_COLOR_COVERAGE`, 1·2위 차가 5% 미만이면 `MIXED_COLOR`, 그 외에만 `ESTIMATED`를 반환합니다. `pixel_fraction`은 그 색상의 픽셀 수를 crop 전체 픽셀 수로 나눈 값입니다.

이 데이터는 화면 표시와 관측에만 사용합니다. 검출 결과가 비어 있으면 해당 프레임에서 후보가 없거나 detector가 설정되지 않은 것일 수 있으므로 `detector_state`를 함께 확인해야 합니다. 카메라 프레임이 stale이면 이 결과를 집기 목표로 사용하지 않습니다. 좌표 변환·작업영역 판정·로봇 명령은 이 API에 연결되어 있지 않습니다.

`GET /api/cameras/{camera_id}/target-observation?class_name=...`은 지정 클래스의 후보 수를 최신 프레임에서 분류합니다. 선택적 `color`와 1–999,999,999 정수 `shelf_level` query는 검토용 힌트 비교를 요청합니다. 결과의 `observation_schema_version`은 3이며, 상태는 detector 미설정/준비 중/오류, MuJoCo FIXED feed에서 현재 frame inference가 수행되지 않은 `DETECTOR_NOT_RUN`, 모델에 없는 클래스(`CLASS_UNKNOWN`), 카메라 연결 끊김(`CAMERA_DISCONNECTED`), 스트림 사용 불가(`CAMERA_STREAM_UNAVAILABLE`), 픽셀 변환 버전 미확인(`PIXEL_TRANSFORM_UNKNOWN`), 프레임 fingerprint 미확인(`FRAME_PROVENANCE_UNKNOWN`), 프레임 stale/사용 불가, `TARGET_NOT_FOUND`, `SINGLE_CANDIDATE`, `AMBIGUOUS`를 구분합니다. detector가 준비된 뒤에는 연결 상태, 스트림 상태, 프레임 freshness, pixel transform/frame hash provenance, 요청 class 순으로 판정하며, 선행 조건이 충족되지 않으면 후보 목록은 비웁니다. 따라서 연결·스트림·프레임이 준비되지 않은 상태는 hash/class 오류보다 먼저 표시됩니다. 결과는 모델의 `available_classes`, `frame_sha256`, `pixel_transform_id`, `detector_inference_frame_sequence`, `decision_scope: OBSERVATION_ONLY`, `arm_command_sent: false`를 포함합니다. FIXED의 일반 GET 관측은 inference를 시작하지 않으며 마지막 inference sequence가 현재 frame sequence와 다르면 `DETECTOR_NOT_RUN`을 반환합니다. task observation POST에서 해당 frame을 실제 추론한 뒤 후보가 없을 때만 `TARGET_NOT_FOUND`를 반환합니다. 현재 검출 프레임이 카메라의 최신 frame sequence·capture time·frame hash·pixel transform ID와 일치하지 않아도 후보 목록을 비웁니다. 선택적 `color_evaluation`은 `algorithm_version: exact-estimated-hint-match-v1`으로 red/blue/green/yellow/black/white 및 지원 한국어 별칭을 각 후보의 `ESTIMATED` `color_hint`와 정확히 대조하고 결과 상태와 `candidates` 기준 0부터 시작하는 `candidate_indices`를 반환합니다. 색상 힌트/비교 algorithm version은 API와 UI에서 확인할 수 있으며, UI는 후보 번호를 1부터 표시합니다. `shelf_level_evaluation`은 요청 층과 명시적으로 선택한 camera ROI 초안을 대조합니다. 영역 파일이 없거나 계약이 잘못됐거나 camera ID·해상도·`pixel_transform_id`가 일치하지 않으면 상태를 구분하고 층 일치를 만들지 않습니다. 파일을 1 MB 이내로 모두 읽은 경우에는 유효 비교 여부와 관계없이 해당 JSON 원본 bytes의 `calibration_artifact_sha256`를 반환해 task event에서 검사한 정확한 파일 내용을 추적할 수 있습니다. 읽기 실패 또는 크기 초과로 전체 내용을 확보하지 못하면 hash를 만들지 않습니다. 일치할 때는 detection `center_px`를 polygon에 대조하고 후보 index/층 판정 상태를 반환합니다. 상태에는 `CALIBRATION_NOT_CONFIGURED`, `CALIBRATION_UNREADABLE`, `CALIBRATION_INVALID`, `CALIBRATION_CAMERA_MISMATCH`, `CALIBRATION_PIXEL_TRANSFORM_MISMATCH`, `CALIBRATION_RESOLUTION_MISMATCH`, `REQUESTED_LEVEL_NOT_CONFIGURED`, `NO_CANDIDATES`, `NO_REGION_MATCH`, `NO_SHELF_LEVEL_MATCH`, `AMBIGUOUS_REGION_ASSIGNMENT`, `INVALID_CANDIDATE_POSITION`, `SINGLE_SHELF_LEVEL_MATCH`, `MULTIPLE_SHELF_LEVEL_MATCHES`가 있습니다. 두 평가는 `review_required: true`, `used_for_selection: false`이며 원래 `candidates` 배열을 필터·변경하지 않습니다. 색상 hint는 물체 색상 확정이 아니며 2D ROI는 가림/원근을 해결하지 않습니다. `SINGLE_CANDIDATE` 역시 픽업 안전성이나 위치 보정을 검증하지 않으며 실행 허가가 아닙니다. 클래스 이름 비교는 대소문자를 구분하지 않습니다.

`POST /api/tasks/{task_id}/target-observation`은 JSON body의 `camera_id`로 해당 작업의 `pick_request.category`를 현재 카메라 관측에 대조하고, 요청에 `color`/`shelf_level`이 있으면 각각 색상 hint와 명시적 ROI 초안에 대한 검토 비교를 수행해 반환된 동일 observation을 `TARGET_OBSERVED` 이벤트 payload에 저장합니다. MuJoCo `FIXED` feed는 `CONTROL_STATION_HEAD_YOLO_WEIGHTS` 및 선택적 `CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS`로 detector를 설정할 수 있습니다. detector가 ready이면 이 POST가 snapshot의 frame sequence/hash와 같은 최근 보관 raw frame에만 추론을 요청하고, 각 candidate에 동일한 camera/frame/hash/pixel-transform provenance를 붙이고 observation에는 ROS `source_stamp`와 `source_frame_id`를 원문으로 보존합니다. 이 sensor header 시간은 `source_stamp_clock_status: ROS_SOURCE_HEADER_NOT_ALIGNED_WITH_HOST_CLOCK`로 표시하며 host `captured_at`과 같은 시간축으로 비교하지 않습니다. 그 프레임이 더 이상 유지되지 않거나 provenance가 바뀌면 추론은 실패 상태로 기록하며 새 프레임으로 조용히 대체하지 않습니다. detector가 구성되지 않았거나 로딩/오류 상태면 관측 이벤트는 해당 상태를 그대로 남깁니다. fixed camera 추론은 task POST 요청 때에만 수행하고 ROS image callback에서는 모델 inference를 돌리지 않습니다. 해당 camera가 ready YOLO-World이고 요청 category가 시작 시 고정 vocabulary에 없으면 task POST에 한해 요청 class one-shot inference를 수행하며 기존 관측 snapshot의 frame sequence/hash가 같은 raw frame에 적용합니다. `save_frame: true`이면 이 raw frame을 inference 전에 먼저 저장합니다. detector inference lock으로 상시 추론과 직렬화하며 원 vocabulary 복원까지 완료한 뒤 반환합니다; `GET` 관측과 closed-set YOLO는 변경하지 않습니다. 요청별 추론 실패는 `REQUEST_PROMPT_INFERENCE_ERROR`로 해당 observation/event에 기록하고 기본 vocabulary 복원이 실패한 경우에만 detector 상태도 오류로 전환합니다. fixed camera 추론 실패는 `DETECTOR_INFERENCE_ERROR`로 기록합니다. 두 비교는 후보를 필터링하거나 선택하지 않습니다. detector provenance에는 가중치 file SHA-256, confidence threshold, 추론 이미지 크기, 설정 device, Ultralytics 버전을 담으며, 요청별 one-shot 적용 시 `prompt_scope`, `requested_prompt`, configured vocabulary 및 policy version `yolo-world-request-class-one-shot-v1`도 기록하고, 모델 파일이나 package version을 안정적으로 식별할 수 없는 detector 오류 상태에서는 일부 또는 전체 값이 null일 수 있습니다. device가 null이면 backend 자동 선택 요청이며 실제 선택 장치가 아닙니다. ROI 파일 디렉터리는 `CONTROL_STATION_SHELF_REGION_DIR`로 지정하며 기본값은 `.control-station/shelf-region-calibrations`; 파일명은 `{camera_id}.json`입니다. 이는 명시적으로 배치한 `DRAFT_ONLY` artifact만 읽어 관측용 비교를 하며, 소스 camera ID·해상도·`pixel_transform_id`가 맞지 않으면 결과를 만들지 않습니다. 선택적 `save_frame: true`는 해당 sequence/hash가 아직 메모리의 최근 5개 프레임 안에 있을 때만 그 원본 frame을 lossless PNG로 `.control-station/observations/{task_id}/`에 저장합니다. 원본 PNG는 64 MiB 상한을 적용하고 저장·검증·조회에서 managed directory symlink와 파일 symlink를 거부합니다. 기본값은 `false`입니다. 응답과 이벤트에는 `frame_saved`와 `frame_artifact`(상대 경로, PNG 파일 SHA-256, 원본 frame SHA-256)가 포함됩니다. 파일은 task 삭제 시 자동 삭제되지 않으며 사용자가 `.control-station/observations`에서 관리합니다. 요청은 카메라 position에 따라 HEAD 또는 FIXED의 unique pending `DETECT_TARGETS`, JOINT의 unique pending `REFINE_TARGET` step을 찾아 review 목적으로 `step_id`에 연결합니다. Joint 관측을 대기 중에 기록해도 실제 접근 시점의 재인식을 증명하지 않으며, task/step 상태를 변경하거나 command ID를 만들지 않습니다. 이벤트에는 전체 `pick_request`, detector/frame hash/pixel-transform 및 camera ROS source stamp/frame ID provenance, 현재 프레임의 픽셀 후보 및 선택적 색상/층 평가, `task_state_changed: false`, `arm_command_sent: false`가 포함됩니다. 이는 관측 감사 이력이며 인식 단계 완료, 위치 추정 또는 집기 허가가 아닙니다. schema v1로 저장된 과거 event는 source frame hash가 없어 projection CLI가 거부합니다.


MuJoCo FIXED 카메라의 `depth_region_observation`은 일반 GET에서 `NOT_REQUESTED`이고, task observation POST는 exact RGB source stamp/frame ID와 해상도가 일치하며 최근 2초 이내 수신된 depth frame이 보관된 경우에만 후보별 수치를 계산합니다. RGB와 depth 각각에서 최근 64개의 서로 다른 source stamp 값의 occurrence를 추적하며, 그 범위에서 동일 stamp가 반복되면 `RGB_STAMP_NOT_UNIQUE` 또는 `DEPTH_STAMP_NOT_UNIQUE`로 요약을 거부합니다. 잘못된 depth 메시지도 기존 반복 이력을 지우지 않습니다. 이 제한 이력은 과거 전체의 stamp 유일성을 보장하지 않으며, occurrence 검사는 stamp의 반복 여부만 확인해 노출 동기화를 증명하지 않습니다. 이 경로는 raw depth payload를 task event/API로 노출하지 않고 최근 최대 5 frame 및 16 MiB 한도에서 보관하며 observation에는 후보 최대 32개와 총 8,000,000 bbox pixel 처리 한계를 기록합니다. 후보별 `bbox_xyxy_px` 영역에서 finite positive raw sample의 min/median/max와 finite/positive/zero/nonpositive/nonfinite pixel count를 계산하고 source header, encoding, endian, depth payload SHA-256 및 frame provenance를 기록합니다. `32FC1`/`16UC1` 숫자는 원시 인코딩 값으로만 표시하고 `numeric_unit_status: NOT_INTERPRETED`를 유지합니다. bbox 영역에는 배경·가림 픽셀이 포함될 수 있습니다. 이 결과는 `review_required: true`, `used_for_selection: false`, `target_pose_use: false`이며 RGB-D synchronization이나 calibration 검증, camera/base 3D 좌표, 물체 중심·자세, grasp target을 뜻하지 않습니다. 조건 미충족 상태는 `RGB_FRAME_NOT_RETAINED`, `RGB_HEADER_MISMATCH`, `RGB_STAMP_NOT_UNIQUE`, `RGB_FRAME_STALE`, `NO_EXACT_DEPTH_FRAME`, `DEPTH_FRAME_STALE`, `DEPTH_HEADER_MISMATCH`, `DEPTH_STAMP_NOT_UNIQUE`, `DEPTH_DIMENSIONS_MISMATCH` 등으로 구분되며 depth 수치로 후보를 선택하지 않습니다. 일반 GET observation은 `NOT_REQUESTED`를 반환합니다.

관측, 후보 선택, projection-draft API는 저장된 task의 `pick_request`도 현재 계약으로 다시 확인합니다. 관측에 대상 명세가 없으면 `PICK_REQUEST_MISSING`, 값이 잘못됐거나 비정규 형식이면 `PICK_REQUEST_INVALID`를 HTTP 409로 반환하고 검토 event를 기록하지 않습니다. 선택과 projection 경로의 누락/잘못된 명세도 `PICK_REQUEST_INVALID`로 거부합니다.

`GET /api/tasks/{task_id}/observations/{event_id}/image`는 task 이력에서 해당 event ID가 유일한 `TARGET_OBSERVED`이며 저장된 PNG metadata의 원본 frame hash·픽셀 형식·크기가 event 관측과 일치할 때만 image를 제공합니다. path는 `.control-station/observations/{task_id}/{filename}.png` 형태와 실제 task 전용 디렉터리 경계를 확인하고, 64 MiB 상한 및 symlink 거부 뒤 전달 전에 PNG 파일 SHA-256을 검증합니다. 파일이 없거나 경로·메타데이터·hash 검증에 실패하면 image를 반환하지 않습니다.

`POST /api/tasks/{task_id}/target-selection`은 `{"observation_event_id":"EVT-...","candidate_index":0}`를 받아 대기 task의 유일한 `TARGET_OBSERVED` event에서 해당 후보를 사람이 선택했다는 `TARGET_SELECTED` 감사 이벤트를 기록합니다. source observation은 schema v2 또는 v3, `FRESH`, `SINGLE_CANDIDATE` 또는 `AMBIGUOUS`, `OBSERVATION_ONLY`여야 하고, event에 `step_id`가 있으면 pending `DETECT_TARGETS`를 가리켜야 합니다(과거 `null` 링크는 허용). task에 저장된 `pick_request` 및 candidate의 camera/frame/hash/transform 출처와 일치해야 합니다. 새 선택 기록에는 같은 event에 연결된 원본 PNG가 필요하며 artifact 경로와 파일 SHA-256을 검증합니다. `candidate_index`는 0-based입니다. 선택 이벤트는 pending `SELECT_TARGET` step의 `step_id`, 관측 ID, 후보 snapshot, 요청 조건, 색상/선반 hint 평가를 보존하고 `source: operator_review`, `selection_scope: HUMAN_REVIEW_ONLY`, `task_state_changed: false`, `arm_command_sent: false`를 표시합니다. 새 기록은 `201`, 이미 저장된 동일 요청의 재전송은 기존 ID를 돌려주는 `200`과 `already_recorded: true`를 반환합니다. 재전송은 동일한 관측 ID와 후보 index, 저장된 provenance를 모두 만족해야 하며 이미 기록된 경우 source PNG를 다시 읽을 필요가 없습니다. 중복 확인과 새 이벤트 추가는 저장 트랜잭션 안에서 수행해 동시 동일 요청도 같은 이벤트 ID로 수렴시킵니다. 작업이 `PENDING`이 아니거나 후보 provenance가 맞지 않으면 거부합니다. 이 엔드포인트는 후보를 다시 인식하거나 결과를 검증·승인하지 않고, task/step 상태 또는 downstream 실행 선택을 변경하지 않습니다.

`POST /api/tasks/{task_id}/target-observation`과 `POST /api/tasks/{task_id}/target-selection`이 반환하는 오류 JSON은 `error_code`, 사람이 읽을 수 있는 `message`, 하위 호환용 `error`를 포함합니다. 검증·상태·출처 오류는 안정된 코드로 구분하며 웹 화면은 코드를 메시지와 함께 표시합니다. 이는 task 관련 POST 두 경로에만 적용된 현재 구현 범위이고 GET 관측 API를 포함한 전체 API의 공통 오류 envelope를 뜻하지 않습니다.

`POST /api/tasks/{task_id}/projection-draft`는 `{"artifact_path":".control-station/camera-calibrations/...json","artifact_sha256":"..."}`를 받습니다. 이 API는 임의 파일 경로 대신 저장소의 관리 calibration 디렉터리 바로 아래 JSON만 읽고, 1 MB 크기 제한·SHA-256·초안 schema v2·`DRAFT_ONLY`/2D 전용 플래그·pending task·고유 pending `DETECT_TARGETS`/`ESTIMATE_TARGET_POSE` 단계·관측/선택 event ID와 step 링크·candidate snapshot·camera/frame/transform·보정 ID·미적용 요청 조건을 다시 확인합니다. 초안에 연결된 plane/intrinsic calibration artifact도 같은 디렉터리의 상대 경로와 SHA-256으로 다시 읽어 camera, frame size, pixel transform, calibration ID 교차 일치를 확인하며, 초안의 표본 수·RMS·checkerboard·source-image 요약도 원본 calibration JSON과 대조합니다. 이 비교는 파일 간 불일치 검사이지 calibration 품질 합격 판정이 아닙니다. 과거 event에서 `step_id: null`인 provenance는 해당 artifact도 null을 그대로 보존한 경우에만 허용합니다. 검증되면 `TARGET_PLANE_PROJECTION_RECORDED`를 pending `ESTIMATE_TARGET_POSE`에 연결해 기록하고 event ID를 돌려줍니다. 같은 draft ID와 artifact hash의 재요청은 기존 event를 반환하며, 같은 ID에 다른 hash를 제출하면 거부합니다. 이 이벤트는 선반 기준 2D `(x,y)` 검토 자료만 보존하며 `z`, camera-to-robot transform, IK, 작업영역/충돌 검사 또는 파지 목표를 뜻하지 않습니다. `task_state_changed: false`, `arm_command_sent: false`, `usable_for_robot_control: false`이며 task/step 상태, 명령, 선택 결과를 바꾸지 않습니다. offline CLI의 `--record-review-event`가 기본 꺼짐이며 localhost에만 이 요청을 보냅니다.

`GET /api/tasks/{task_id}/projection-drafts/{event_id}/artifact`는 해당 task에 유일하게 존재하는 `TARGET_PLANE_PROJECTION_RECORDED` event를 찾아 저장된 상대 경로와 SHA-256으로 artifact를 다시 읽습니다. draft ID/task ID/`DRAFT_ONLY` 상태 및 review-only event 플래그까지 일치해야 JSON을 반환하며, 파일이 바뀌었거나 없어졌으면 거부합니다. task 단계가 이후 진행되어도 과거 review artifact의 hash 검증 열람은 가능합니다. 이 조회는 task나 로봇 상태를 변경하지 않습니다.

`POST /api/tasks/{task_id}/projection-draft` 오류 응답은 `error_code`, 사람이 읽을 수 있는 `message`, 하위 호환용 `error`를 포함합니다. 요청 형식, task/step 상태 충돌, 초안 provenance 검증 거부를 구분하며, projection review 화면이 없는 현재는 API/CLI 호출자에게 이 코드를 제공합니다.

관제의 JSON POST body parser는 object 내 중복 key를 거부하고 HTTP 400을 반환합니다. 요청 값이 parser 선택 순서에 따라 달라지지 않도록 하는 입력 계약이며, task/event 저장이나 장치 동작을 시작하지 않습니다.

`GET /api/tasks/{task_id}`는 응답에 비저장 `local_execution_owner` 관측을 덧붙입니다. 현재 활성 task에 대한 값은 `LOCAL_WORKER_ACTIVE`, `NO_LOCAL_WORKER`, `MODE_NOT_MANAGED` 중 하나이며 terminal/pending task는 `NOT_ACTIVE`입니다. 이는 이 Python 프로세스가 가진 mock worker 여부만 설명하며, 다른 프로세스의 worker나 장치 상태·정지·작업 결과를 증명하지 않습니다. worker가 없는 활성 task를 자동 재개하거나 종료하지 않습니다.

`POST /api/tasks/parse-command`는 `{"text":"..."}`를 받아 `RULE_BASED_DRAFT`와 구조화 후보, 누락/모호 필드 및 warning을 반환합니다. 현재는 제한된 한국어 문형에서 지원 동사·목적지(동사 우선 및 간단한 목적어 우선 배치/이동 문형), 일부 색상어, 아라비아 숫자와 첫째부터 열째까지 일부 한국어 서수의 선반 층을 추출하며 일반 영어 문장은 지원하지 않습니다. 한 작업의 parser는 물체 한 개를 전제로 하며, 대상 주변의 복수형·2개 이상 수량 표현이 있으면 category를 비우고 `MULTIPLE_TARGETS_UNSUPPORTED`로 표시합니다. `ready` 상태 카메라의 detector class 목록이 있으면 문장에 쓰인 label을 대소문자 구분 없이 대조하고, `box`/`bottle` 한국어 별칭도 해당 label과 연결합니다. Head Camera가 ready YOLO-World인 경우에만 고정 vocabulary에 없는 별칭 또는 단일 한국어 목적격 대상구를 `REQUEST_PROMPT_REVIEW_REQUIRED` category 초안으로 채웁니다. 이는 요청별 class prompt를 사람이 검토하도록 돕는 규칙이며 object detection 결과가 아닙니다. 복수/수량, conjunction, 부정·대조 표현이 대상구에 있거나 대상구가 단일하지 않으면 자동 채우지 않습니다. 오류·미준비 카메라의 클래스는 사용 가능 목록으로 보지 않습니다. 물체·색상·선반 층의 부정/제외 표현이나 목적지의 대조/선택 표현이 추출값에 걸리면 해당 필드를 자동 선택하지 않고 모호/누락으로 표시합니다. 색상과 선반 층은 관측 때 검토용 hint 비교만 할 수 있고 후보 필터·선택에는 사용하지 않는다고 경고합니다. 결과는 항상 `review_required: true`, `task_created: false`, `arm_command_sent: false`이며 DB나 로봇 상태를 바꾸지 않습니다. 해석 결과는 task 등록 전에 사용자가 확인·수정해야 합니다.

## 6. 전송 방식: 합의된 범위와 미정 항목

| 경계 | 현재 합의·관측된 방식 | 미정·미구현 |
|---|---|---|
| UI ↔ 관제 | localhost HTTP API | 원격 배포 인증·TLS·접근 제어 |
| 관제 ↔ 저장소 | SQLite, SQLAlchemy ORM, Alembic | 원격/다중 사용자 배포 DB |
| 관제 ↔ Pinky | ROS 2/Nav2 action client를 기본 방향으로 검토 | 배포판, domain, action 이름·타입, namespace, frame, lifecycle 진단; 현재는 action server discovery만 구현 |
| 관제 ↔ OMX | 공통 명령·결과 모델을 어댑터에서 사용; 현장 read-only register probe 확인 | 실제 관제 어댑터, READY/정지 신호, 안전 작업 영역, 최종 파지 결과/오류 경로; LeRobot `send_action()`은 관절 목표 쓰기 함수이며 이 계약을 대신하지 않음 |
| 카메라 ↔ 관제 | 현재 OpenCV 캡처와 브라우저용 MJPEG | 배포 장치 ID, 동시 스트림 성능, 장착 위치 최종 확인 |
| 주문 서비스 ↔ 관제 | 주문 ID·물품·수량·픽업 위치가 필요 | endpoint, HTTP/event 방식, 인증, payload, 재시도와 저장 책임자 |
| 로봇 결과 이벤트 | 관제 내부 식별자와 이벤트 의미 | 실제 topic/action result 형식, event ID·sequence·시각, 순서·재전송 보장 |

## 7. 남은 확인 사항

- [ ] 주문 API schema, 인증·전송 방식, 주문/운송 상태 소유자
- [ ] 한 주문을 여러 운송 작업으로 나누는 조건과 `order_id`-`task_id` 관계
- [ ] Pinky Nav2 action 계약과 map frame
- [ ] OMX 작업 명령, READY, 파지·적재 완료와 오류 신호
- [ ] 카메라→OMX 좌표 frame·단위·축·calibration 및 검증 허용 오차
- [ ] 장치 이벤트 ID·sequence·시각 제공 여부와 heartbeat 주기
- [ ] timeout/stale 기준과 운영자의 보류·재개 절차
