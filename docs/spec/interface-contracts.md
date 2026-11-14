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
| 관제 UI → HTTP API | 주문·작업 생성·조회, 구조화된 대상 조건, 작업 시작·취소 | HTTP 접수와 작업 ID; 완료/취소는 별도 상태·이벤트 | localhost 구현; 단계별로 무엇이 모의인지는 [구현 현황](../implementation/current-state.md)과 [주문 흐름](../implementation/order-flow.md) |
| Mock start/cancel API → UI/client | task-control 결과 또는 오류 | 오류는 `error_code`, 사용자용 `message`, 호환용 `error`를 함께 반환 | `TASK_NOT_FOUND`, `TASK_STATE_CONFLICT`, `TASK_NOT_STARTABLE`, `TASK_NOT_CANCELLABLE`, `TASK_WORKER_OWNERSHIP_UNCERTAIN`, `EXECUTION_MODE_UNSUPPORTED`, `MOCK_WORKER_START_FAILED`, `TASK_CONTROL_ERROR`, `CROSS_ORIGIN_REQUEST_REJECTED`; 다른 API는 아직 공통 오류 envelope 적용 전 |
| Mock 오케스트레이터 → 작업 이력 저장소 | 현재 task/active-step/step/command/robot ID와 mock progress | task가 `VALIDATING`/`RUNNING`이고 같은 step이 `active_step_id` 및 `RUNNING` 상태이며 command ID가 일치할 때만 `PROGRESS` event를 추가; task/step 상태는 유지 | mock 지연 구간의 50% 표시; 장치 feedback이나 실측 작업량 아님; 모든 상관 ID와 상태 선행 조건을 저장소가 확인 |
| 오케스트레이터 → Pinky 어댑터 | task/step/command ID, 목적지, deadline | 수락/거절, 진행, terminal 성공·실패, 취소 결과 | 공통 의미 확정; ROS 2 action 이름·타입·namespace·frame 미확정 |
| 오케스트레이터 → OMX 어댑터 | task/step/command ID, 작업 종류·파라미터 | 수락/거절, 진행, 파지·적재 최종 결과 또는 실패 | 공통 의미 확정; 관제 자동 명령은 미연결. 로컬 LeRobot `OmxFollower.send_action()`은 `*.pos` 관절 목표를 쓰지만 task 수락/terminal 파지 결과를 제공하지 않음 |
| 로봇 어댑터 → 이벤트 처리기 | robot/task/step/command ID, 장치 결과 | `ACCEPTED`, `EXECUTION_STARTED`, `PROGRESS`, `SUCCEEDED`, `FAILED`, `CANCELED` 또는 미확정 | 공통 이벤트 의미 확정; 장치별 실제 메시지 미확정 |
| 천장 카메라 주차 판정 → 관제 | spot·marker ID, 등록 위치 대비 dx·dy·dyaw와 허용 오차·비율, 시야 확인 여부, 프레임 출처 | `ARRIVED`, `MISALIGNED`, 판정 불가(`NOT_FOUND`/`AMBIGUOUS`/`VIEW_UNVERIFIED`); Nav2 결과와 별개의 관측 | 초안: C++ 노드(`/pinky_parking/state`, `/pinky_parking/check`)와 CLI, 관제 GET 관측, `VERIFY_ARRIVAL`·주문 `WAIT_PINKY_ARRIVAL`의 `PARKING_OBSERVED` event·진행 조건 구현 ([주차 정렬 관측](#pinky-주차-정렬-관측-초안)) |
| Head/Joint Camera → 관제 UI | 카메라별 프레임·연결·스트림·프레임 sequence/해상도·마지막 프레임 시각, closed-set YOLO 또는 선택 YOLO-World의 camera/frame 출처·고정 prompt vocabulary·픽셀 영역 | 카메라별 오류; 다른 카메라나 로봇 상태에 전파하지 않음; 검출 후보는 자동 선택/명령 신호가 아님 | API 관측 필드 구현; 가중치 미포함; 실제 장착 위치·실물 객체별 검출 적합성은 미검증 |

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

OMX에는 좌표를 보내지 않습니다(2026-11-11 결정). 팔은 기록한 관절 자세(칸, home)와 물체별 학습 정책으로만 움직이며, 카메라 좌표는 표시와 근거 기록에만 씁니다.

| 데이터 | 좌표계·단위 | 결정 상태 |
|---|---|---|
| 카메라 검출 위치 | 이미지 `(u, v)` / `pixel`, 카메라 ID와 프레임 시각 동반 | 픽셀 표현은 개념상 정함; 해상도·원점·crop/resize 보정은 카메라별 확인 필요 |
| OMX 집기 목표 | 좌표 없음. 칸 ID(`pick_slot_id`) 또는 물체 이름으로 고른 학습 정책 ([OMX 명령 HTTP 인터페이스](../implementation/arm-command-api.md)의 `target`) | 정함 |
| Pinky 목적지 | Nav2가 요구하는 map frame과 pose 단위 | **미확정**: map/frame, yaw 표현, waypoint/action schema 조사 필요 |


### 작업 생성과 대상 명세 (현재 API)

`POST /api/tasks`는 주문(`order_items`, 선택 `pinky_id`, 아래 "주문과 핑키 관제 신호") 또는 이전 형식인 `destination`, `arm_job`, 선택적 `pick_request`를 받습니다. 나머지 최상위 필드는 HTTP 400으로 거부해 필드명 오타나 아직 지원하지 않는 값을 조용히 무시하지 않습니다. `destination`과 `arm_job`은 필수 문자열이고, `pick_request`는 생략/`null`이거나 `{category, color?, shelf_level?}` 계약을 따라야 합니다. `category`는 1–80자, `color`는 1–40자이며 두 필드 모두 ASCII C0 및 DEL 제어 문자를 포함할 수 없습니다. 성공 응답은 저장된 `PENDING` 작업이며 인식·작업 단계 실행·로봇 명령을 시작하지 않습니다.

### 주문과 핑키 관제 신호 (현재 API, 2026-11-11)

주문은 `POST /api/tasks`의 `order_items`(`[{"object_class": "carrot", "quantity": 1}, ...]`, 1–5종·종류별 1–5개)와 선택 `pinky_id`(`10`/`11`/`12`)로 등록합니다. 물체 이름은 학습 정책 파일의 클래스·이름·별칭(당근, carrot)으로 받고 클래스 이름으로 저장합니다. 같은 물체는 수량을 합칩니다. `pinky_id`가 없으면 끝나지 않은 주문이나 경보를 확인하지 않은 멈춘 주문이 없는 핑키를 10→11→12 순서로 배정하고, 없으면 HTTP 409 `PINKY_UNAVAILABLE`입니다. 단계 목록과 멈춤 코드는 [주문 흐름](../implementation/order-flow.md)에 있습니다.

핑키 팀 관제(`origin/pinky_fleet`)와는 HTTP로 주고받습니다.

| 관제가 보내거나 읽는 것 | 핑키 관제 | 의미 |
|---|---|---|
| 핑키 단계 | `GET /api/status` → `robots[].route.{status, phase, mission_token}` | `docking_manual`=5번 정지·도킹완료 대기, `docking_anchor`=도킹완료 뒤 위치 재설정(최대 18초, 실패 시 경로 정지), `docked_manual`=정지 유지·수령 대기, `dock_exit`=수령 뒤 후진, 경로 없음·끝남=`parked` |
| 도킹완료 | `POST /api/robots/<id>/operator-confirm` `{"action":"dock_complete","token":<docking_manual 토큰>}` | 천장 카메라 주차 판정이 `ARRIVED`일 때만 보냄 |
| 수령 완료 | 같은 경로 `{"action":"received","token":<docked_manual 토큰>}` | 핑키 위 총량이 주문과 같을 때만 보냄 |

응답 202는 핑키 관제가 신호를 받아 로봇에 기록했다는 뜻입니다. 핑키가 실제로 움직였는지는 다음 `GET /api/status` 단계로 확인합니다. 409(단계·토큰 불일치)는 `PINKY_SIGNAL_REJECTED`, 502(로봇 전달 실패)는 `PINKY_SIGNAL_NOT_DELIVERED`, 무응답은 `PINKY_FLEET_UNREACHABLE`로 기록하고 자동으로 다시 보내지 않습니다.

### 카메라 검출 결과 (현재 API)

`GET /api/cameras`의 검출 결과는 화면 표시와 개수 세기·칸 근거에만 씁니다. 검출 결과가 비어 있으면 해당 프레임에서 후보가 없거나 detector가 설정되지 않은 것일 수 있으므로 `detector_state`를 함께 확인해야 합니다. 카메라 프레임이 stale이면 개수 세기와 칸 근거에 쓰지 않습니다. 좌표 변환은 없습니다.

관제의 JSON POST body parser는 object 내 중복 key를 거부하고 HTTP 400을 반환합니다. 요청 값이 parser 선택 순서에 따라 달라지지 않도록 하는 입력 계약이며, task/event 저장이나 장치 동작을 시작하지 않습니다.

`GET /api/tasks/{task_id}`는 응답에 비저장 `local_execution_owner` 관측을 덧붙입니다. 현재 활성 task에 대한 값은 `LOCAL_WORKER_ACTIVE`, `NO_LOCAL_WORKER`, `MODE_NOT_MANAGED` 중 하나이며 terminal/pending task는 `NOT_ACTIVE`입니다. 이는 이 Python 프로세스가 가진 mock worker 여부만 설명하며, 다른 프로세스의 worker나 장치 상태·정지·작업 결과를 증명하지 않습니다. worker가 없는 활성 task를 자동 재개하거나 종료하지 않습니다.

`POST /api/tasks/parse-command`는 `{"text":"..."}`를 받아 `RULE_BASED_DRAFT`와 구조화 후보, 누락/모호 필드 및 warning을 반환합니다. 현재는 제한된 한국어 문형에서 지원 동사·목적지(동사 우선 및 간단한 목적어 우선 배치/이동 문형), 일부 색상어, 아라비아 숫자와 첫째부터 열째까지 일부 한국어 서수의 선반 층을 추출하며 일반 영어 문장은 지원하지 않습니다. 한 작업의 parser는 물체 한 개를 전제로 하며, 대상 주변의 복수형·2개 이상 수량 표현이 있으면 category를 비우고 `MULTIPLE_TARGETS_UNSUPPORTED`로 표시합니다. `ready` 상태 카메라의 detector class 목록이 있으면 문장에 쓰인 label을 대소문자 구분 없이 대조하고, `box`/`bottle` 한국어 별칭도 해당 label과 연결합니다. Head Camera가 ready YOLO-World인 경우에만 고정 vocabulary에 없는 별칭 또는 단일 한국어 목적격 대상구를 `REQUEST_PROMPT_REVIEW_REQUIRED` category 초안으로 채웁니다. 이는 요청별 class prompt를 사람이 검토하도록 돕는 규칙이며 object detection 결과가 아닙니다. 복수/수량, conjunction, 부정·대조 표현이 대상구에 있거나 대상구가 단일하지 않으면 자동 채우지 않습니다. 오류·미준비 카메라의 클래스는 사용 가능 목록으로 보지 않습니다. 물체·색상·선반 층의 부정/제외 표현이나 목적지의 대조/선택 표현이 추출값에 걸리면 해당 필드를 자동 선택하지 않고 모호/누락으로 표시합니다. 색상과 선반 층은 `pick_request`에 저장만 하며 후보 필터·선택에는 쓰지 않습니다. 결과는 항상 `review_required: true`, `task_created: false`, `arm_command_sent: false`이며 DB나 로봇 상태를 바꾸지 않습니다. 해석 결과는 task 등록 전에 사용자가 확인·수정해야 합니다.

### Pinky 주차 정렬 관측 (초안)

Pinky가 멈춘 뒤 천장 카메라로 "등록된 주차 위치와 얼마나 다른지"를 재서 관제에 올리는 계약 초안입니다. 관제는 이 값을 화면에 보여 주고, 다음 단계를 진행할지 판단하는 입력 중 하나로 씁니다. 판정은 C++ ROS 2 노드(`cpp/parking_vision_ros`)만 하고, 관제는 `CONTROL_STATION_PARKING_SPOT`을 지정했을 때 노드의 결과를 아래 JSON으로 바꿉니다. `GET /api/parking-spots/{spot_id}/check`가 아래 관측 객체를 `{"observation": ...}`로 반환하고, `VERIFY_ARRIVAL`과 주문의 `WAIT_PINKY_ARRIVAL`은 같은 관측을 `PARKING_OBSERVED` event로 남긴 뒤 `ARRIVED`일 때만 진행합니다. 운영 절차와 현장 기록은 [실행 방법](../implementation/runbook.md#천장-카메라-pinky-주차-판정), 노드 내부는 [cpp/README.md](../../cpp/README.md)에 있습니다.

천장 카메라는 노드의 캡처 컴포넌트가 열고, 관제 Head Camera는 같은 영상을 `ros:/ceiling_camera/image_raw/compressed`로 받습니다. 결과에는 패널이 보여 주는 관제 feed(`feed_camera_id`, 기본 `head_camera`), 노드가 구독한 영상 토픽(`node_image_topic`), 등록 파일의 보정 카메라(`calibration_camera_id`, `ceiling_camera`)를 함께 기록합니다.

관측 객체 예시입니다.

```json
{
  "observation_schema_version": 1,
  "spot_id": "pinky_dock_1",
  "marker_id": 21,
  "status": "MISALIGNED",
  "reason": "position 62 mm > 50 mm",
  "offset": {
    "frame": "REGISTERED_SPOT_MARKER_AXES",
    "dx_mm": -60.8,
    "dy_mm": 12.0,
    "distance_mm": 62.0,
    "dyaw_deg": 1.3,
    "marker_forward_axis": "-x",
    "forward_mm": 60.8,
    "left_mm": -12.0
  },
  "tolerance": { "position_mm": 50.0, "yaw_deg": 10.0 },
  "position_ratio": 1.24,
  "yaw_ratio": 0.13,
  "deviation_level": "NEAR",
  "view_check": "OFF",
  "view_shift_px": null,
  "frames_used": 15,
  "feed_camera_id": "head_camera",
  "node_image_topic": "/ceiling_camera/image_raw",
  "calibration_camera_id": "ceiling_camera",
  "frame_sequence_range": [10234, 10248],
  "captured_from": "2026-11-02T11:05:01.120Z",
  "captured_until": "2026-11-02T11:05:03.010Z",
  "requested_at": "2026-11-02T11:05:01.000Z",
  "frames_examined": 15,
  "spot_registration_sha256": "…",
  "intrinsic_calibration_sha256": "…",
  "opencv_version": "4.6.0",
  "registration_opencv_version": "4.6.0",
  "image_overlay": { "...": "화면에 그릴 픽셀 좌표" },
  "usable_for_robot_control": false,
  "judgment_source": "ros:/pinky_parking/check"
}
```

| 필드 | 의미 |
|---|---|
| `status` | `ARRIVED`, `MISALIGNED`, `NOT_FOUND`, `AMBIGUOUS`, `VIEW_UNVERIFIED` 중 하나. 의미는 CLI와 같음. 오프셋이 유한한 값이 아니면(측정 실패) `NOT_FOUND`로 두고 도착으로 보지 않음 |
| `offset` | 등록된 주차 마커 축 기준 차이. `dx_mm`는 마커 오른쪽 변 방향, `dy_mm`는 윗변 방향, `dyaw_deg`는 카메라에서 볼 때 반시계가 +. 판정 불가 상태에서는 `null` |
| `forward_mm`, `left_mm` | Pinky 기준 앞(+)·왼쪽(+) 차이. 등록 파일에 마커 장착 방향 `marker_forward_axis`(`+x`/`-x`/`+y`/`-y`, 마커 면이 위를 향한다고 가정)가 있을 때만 포함 |
| `position_ratio`, `yaw_ratio` | "얼마나 다른가"를 허용 오차 대비 비율로 나타낸 값(`distance_mm / position_mm`, `abs(dyaw_deg) / yaw_deg`). 1 이하면 범위 안. 허용 오차를 바꿔도 화면과 이력에서 어긋난 정도를 같은 척도로 비교하기 위한 값 |
| `deviation_level` | 두 비율 중 큰 값 r로 정하는 등급. `WITHIN`(r ≤ 1, `ARRIVED`와 같음), `NEAR`(1 < r ≤ 2, 조금 벗어남), `FAR`(r > 2, 많이 벗어남). 경계 2는 초안 값이며 허용 오차와 함께 조정 |
| `view_check` | `ON`이면 바닥 고정 마커로 카메라 시야를 확인한 결과, `OFF`이면 확인하지 않은 결과. 화면에 항상 표시 |
| `frames_used`, `frames_examined`, `frame_sequence_range`, `captured_from`/`captured_until` | 판정에 쓴(중앙값에 들어간) 프레임 수, 살펴본 요청 이후 프레임 수와 그 출처 |
| `opencv_version`, `registration_opencv_version` | 판정·등록에 쓴 OpenCV 버전(버전이 다르면 결과가 조금 달라짐, [cpp/README.md](../../cpp/README.md)). 등록 파일에 없으면 `null` |
| `image_overlay`, `judgment_source` | 영상 위에 그릴 등록 위치·마커 픽셀(표시 전용)과 결과를 낸 노드 경로(`ros:/pinky_parking/check`, `/live`는 `ros:/pinky_parking/state`) |
| `spot_registration_sha256`, `intrinsic_calibration_sha256` | 어떤 등록·보정 파일로 판정했는지 나타내는 provenance |

규칙 초안:

1. **신선도.** 요청 시각(`requested_at`) 이후에 캡처된 프레임만 씁니다. 요청 전에 버퍼에 있던 프레임은 쓰지 않습니다. 최대 3초 동안 최대 15프레임을 모으고, 5프레임 미만이면 `NOT_FOUND`(사유: 프레임 부족)로 판정합니다. Nav2 terminal 결과를 받은 뒤 요청하면 정차 후 프레임만 쓰게 됩니다.
2. **상태별 관제 처리.**

   | 상태 | 관제 처리 |
   |---|---|
   | `ARRIVED` | 팔 단계를 시작하는 조건 중 하나입니다([시스템 명세 4.4](system-spec.md#44-이동에서-팔로-넘어가는-조건)). Nav2 이동 성공, 로봇 정지, 팔 작업 가능 여부를 대신하지 않으며 그 조건들과 함께 만족해야 합니다. |
   | `MISALIGNED` + `NEAR` | 조금 벗어남. 팔 단계를 시작하지 않습니다. 자동 재주행이나 명령 재전송은 하지 않고, `forward_mm`/`left_mm`(없으면 `dx`/`dy`)와 `dyaw_deg`를 운영자에게 보여 줍니다. |
   | `MISALIGNED` + `FAR` | 많이 벗어남. 다른 위치에 섰거나 이동이 잘못됐을 수 있습니다. 처리는 `NEAR`와 같고, 화면에서 더 눈에 띄게 구분합니다. |
   | `NOT_FOUND`, `AMBIGUOUS`, `VIEW_UNVERIFIED` | 판정 불가입니다. 팔 단계를 시작하지 않고, "어긋남"과 구분해 "확인 못 함"으로 표시합니다. |

3. **관측을 시작하지 못한 경우**는 관측 객체 대신 오류로 반환하고 task event를 남기지 않습니다. `spot_id` 형식이 틀리면 `PARKING_SPOT_ID_INVALID`(400), 등록 파일이 없으면 `PARKING_SPOT_NOT_FOUND`(404), schema·보정 SHA-256이 맞지 않으면 `PARKING_SPOT_INVALID`(409), 노드의 영상 토픽 발행자가 없거나 3초 안에 새 프레임이 없거나 해상도가 보정과 다르면 `PARKING_CAMERA_UNAVAILABLE`(503)입니다. 영상은 들어오지만 stamp가 모두 요청 전이면(예: rosbag을 `use_sim_time` 없이 재생) 같은 코드에 그 사실을 적은 메시지가 붙습니다. 그 밖의 노드 처리 오류는 `PARKING_OBSERVATION_ERROR`(503), 노드가 이미 4개 요청을 처리 중이면 `PARKING_CHECK_BUSY`(429)입니다. 관제가 노드에 연결하지 못하면 `PARKING_NODE_UNAVAILABLE`(503), 6초 안에 응답이 없으면 `PARKING_NODE_TIMEOUT`(503)이고 자동 재시도는 없습니다. `/live`는 노드 상태가 없거나 2초 넘게 갱신되지 않으면 `NODE_UNAVAILABLE`입니다.
4. **화면 표시용 실시간 상태.** `GET /api/parking-spots/{spot_id}/live`는 같은 판정 방식으로 최신 프레임을 계속 보며 `NOT_IN_VIEW`/`MOVING`/`IN_POSITION`/`OFF_POSITION`/판정 불가(`AMBIGUOUS`, `VIEW_UNVERIFIED`, `CAMERA_UNAVAILABLE`, `SPOT_INVALID`, 추적 코드 오류 `TRACKING_ERROR`) 상태와 같은 오프셋·등급 필드, `since`(그 상태가 시작된 시각), `display_only: true`를 반환합니다. 이 값은 팔 단계 진행 조건으로 쓰지 않습니다.
5. **작업 이력 연결.** 작업 실행 중의 관측은 `PARKING_OBSERVED` event로 task/step/robot ID와 함께 남깁니다. 이 event는 task·step 상태를 바꾸지 않습니다.
6. **방향 표현.** `dx`/`dy`는 마커 축 기준입니다. 등록 파일에 `marker_forward_axis`가 있으면 Pinky 기준 `forward_mm`/`left_mm`를 함께 냅니다. 현재 `pinky_dock_1`은 `-x`(마커 왼쪽 변이 Pinky 앞쪽)로 기록돼 있습니다(정한 방법은 [실행 방법](../implementation/runbook.md#2026-11-02-현장-확인)).
7. 이 관측은 `usable_for_robot_control: false`입니다. 좌표로 Pinky를 보정 이동시키는 명령은 만들지 않습니다.

## 6. 전송 방식: 합의된 범위와 미정 항목

| 경계 | 현재 합의·관측된 방식 | 미정·미구현 |
|---|---|---|
| UI ↔ 관제 | localhost HTTP API | 원격 배포 인증·TLS·접근 제어 |
| 관제 ↔ 저장소 | SQLite, SQLAlchemy ORM, Alembic | 원격/다중 사용자 배포 DB |
| 관제 ↔ Pinky | 임무 단계와 도킹완료·수령 완료 신호는 핑키 팀 관제 HTTP(`origin/pinky_fleet`, 위 "주문과 핑키 관제 신호"). 주차 판정은 관제 쪽 C++ ROS 2 노드 | 핑키 관제 외부 접속(기본 127.0.0.1만 열림)과 인증; 주행 명령은 핑키 팀이 담당 |
| 관제 ↔ OMX | HTTP 명령 큐: 워커가 long-poll로 명령을 가져가고 이벤트를 보고하며, `ACCEPTED`에 200을 받아야 실행 ([OMX 명령 HTTP 인터페이스](../implementation/arm-command-api.md)); 현장 read-only register probe 확인 | 주문 흐름의 `PICK_ITEM` 연결(지금은 모의), 실행 중 결과 제한 시간, 안전 작업 영역, 원격 배포 인증 |
| 카메라 ↔ 관제 | 현재 OpenCV 캡처와 브라우저용 MJPEG | 배포 장치 ID, 동시 스트림 성능, 장착 위치 최종 확인 |
| 주문 서비스 ↔ 관제 | 주문 ID·물품·수량·픽업 위치가 필요 | endpoint, HTTP/event 방식, 인증, payload, 재시도와 저장 책임자 |
| 로봇 결과 이벤트 | 관제 내부 식별자와 이벤트 의미 | 실제 topic/action result 형식, event ID·sequence·시각, 순서·재전송 보장 |

## 7. 남은 확인 사항

- [ ] 주문 API schema, 인증·전송 방식, 주문/운송 상태 소유자
- [ ] 한 주문을 여러 운송 작업으로 나누는 조건과 `order_id`-`task_id` 관계
- [ ] Pinky Nav2 action 계약과 map frame
- [ ] OMX 작업 명령, READY, 파지·적재 완료와 오류 신호 (전달 방식과 이벤트는 [HTTP 인터페이스](../implementation/arm-command-api.md)로 초안 구현; `reason_code` 목록·READY 신호·실물 워커는 미정)
- [ ] 핑키 관제 외부 접속 주소·인증과, 관제가 `dock_complete`를 자동으로 보내는 것에 대한 핑키 팀 합의
- [ ] 장치 이벤트 ID·sequence·시각 제공 여부와 heartbeat 주기
- [ ] timeout/stale 기준과 운영자의 보류·재개 절차
- [ ] 주차 정렬 `MISALIGNED`·판정 불가 때 운영자 처리 절차(재주행 요청 방식, 재판정 시점). `ARRIVED`를 팔 단계 조건에 넣는 것은 2026-11-02에 결정
- [ ] 주차 허용 오차(팔 작업 허용치와 Pinky 재주차 반복성 기준; 현재 `pinky_dock_1`은 임시 25 mm / 5°)와 `NEAR`/`FAR` 경계
- [ ] `spot_id` ↔ 목적지(주차·적재 위치), `marker_id` ↔ `robot_id` 매핑 (지금 `pinky_dock_1`은 마커 21 핑키 한 대 기준)
- [ ] 시야 확인 없는(`view_check: OFF`) 판정을 진행 근거로 받아들일지
