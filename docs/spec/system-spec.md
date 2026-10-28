# 시스템 개발 명세

## 문서 범위와 기준

이 문서는 관제시스템의 **완성 목표와 외부 동작 계약**을 정의합니다. 현재 코드의 사실은 [구현 현황](../implementation/current-state.md), 명령 실행 방법은 [실행 방법](../implementation/runbook.md)을 기준으로 확인하세요. 목표 컴포넌트 간 상호작용은 [목표 데이터 흐름](data-flow.md)에 모았습니다.

요구사항의 원문은 저장소 루트의 [robot-control-flow.md](../../robot-control-flow.md)입니다. 이 문서는 원문의 내용을 개발자가 구현하고 검토할 수 있도록 상태, API 의미, 실패 규칙으로 정리한 명세입니다. 실제 장비 프로토콜이 확인되지 않은 값은 확정하지 않습니다.

## 1. 제품 목표와 범위

운영자가 이동 로봇과 로봇팔을 한 관제 UI에서 관리하고, 이동 완료 확인 후 팔 작업을 실행하는 순서를 안전하게 조율합니다. 관제시스템은 명령을 전달한 사실과 로봇이 실제 작업을 완료한 사실을 별도로 기록합니다.

### MVP에 포함할 기능

- 이동 로봇과 로봇팔의 연결 및 실행 상태 조회
- 작업 생성, 명시적 실행, 진행 상태·최종 결과·이력 조회
- 이동 명령의 접수와 최종 결과 확인
- 이동 성공이 확인된 작업에서만 팔 작업 시작
- 작업·단계별 오류, timeout, 취소 및 결과 불명 처리
- Head Camera와 Joint Camera의 개별 연결·영상 상태 관리 및 영상 표시
- 실제 로봇이 없는 환경의 모의 어댑터

### MVP에서 제외할 기능

- 비전 추론, 객체 검출·분류·위치 추정, 카메라 캘리브레이션
- 확인되지 않은 로봇 토픽, 서비스, 액션 또는 SDK를 가정한 연동
- 다중 작업 동시 스케줄링과 우선순위 큐
- 여러 대의 동일 종류 로봇을 대상으로 하는 fleet 관리
- 관제 UI를 하드웨어 비상정지나 안전 인터록으로 사용하는 것

## 2. 구성요소 책임

| 구성요소 | 입력 | 책임과 출력 |
|---|---|---|
| 관제 UI | 사용자 입력, API 응답 | 작업 요청, 명시적 실행·취소, 로봇·카메라·작업 상태 표시 |
| HTTP API | UI 요청 | 입력 검증, 명령 접수, 리소스 조회; 로봇 동작 완료까지 HTTP 요청을 붙잡지 않음 |
| 작업 오케스트레이터 | task ID, 어댑터 이벤트 | 작업 선점, 순서·전이 관리, 단계 실행, 결과 기록 |
| 이동 로봇 어댑터 | 목적지, command ID | ROS 2/Nav2 action goal 전달 및 수락·feedback·최종 결과를 공통 이벤트로 변환 |
| 로봇팔 어댑터 | 팔 작업, command ID | 팔 작업 전달 및 수락·진행·최종 결과를 공통 이벤트로 변환 |
| 로봇 상태 관리자 | heartbeat, 장치 상태 | 로봇별 연결·준비·실행 상태 및 마지막 갱신 시각 관리 |
| 카메라 관리자 | 카메라 프레임과 연결 이벤트 | 카메라마다 연결·스트림·신선도 상태 관리 및 프레임 전달 |
| 이력 저장소 | 작업 및 이벤트 | 작업·단계 상태, 타임스탬프, 결과, 오류 저장 |

이동 로봇, 로봇팔, 카메라는 서로 다른 상태 리소스입니다. 예를 들어 카메라 끊김을 로봇 연결 끊김으로 바꾸거나, 프로세스 생존을 팔 준비 완료로 해석하지 않습니다.

## 3. 핵심 도메인 객체

### Task

필수 필드: `id`, `status`, `destination`, `arm_task`, `created_at`. 실행 후 `started_at`, `finished_at`, `active_step_id`, `error`를 기록합니다. `destination`과 `arm_task.parameters`의 구체적인 형식은 실제 장비 인터페이스를 확인한 후 확정합니다.

### TaskStep

필수 필드: `id`, `task_id`, `step_type`, `status`. 실제 로봇이 수행하는 단계는 `robot_id`를 지정하고, `VALIDATE_ROBOTS`, `FINALIZE_TASK`처럼 관제 시스템이 수행하는 단계는 `robot_id: null`로 둡니다. 실행한 명령에는 `command_id`를 붙이고 `started_at`, `finished_at`, 결과 또는 오류를 기록합니다. 한 작업 안에서 단계 순서를 고정해 저장합니다.

목표 단계의 기본 순서는 다음과 같습니다.

1. `VALIDATE_ROBOTS`
2. `NAVIGATE_TO_DESTINATION`
3. `VERIFY_ARRIVAL`
4. `EXECUTE_ARM_TASK`
5. `VERIFY_ARM_RESULT`
6. `FINALIZE_TASK`

작업이 이동만 요구하는 경우 등 선택 단계의 존재 여부는 작업 유형에 따라 정합니다. MVP에서 기본 시나리오는 위 순서를 따릅니다.

### Robot

필수 개념은 `id`, `kind`(`MOBILE` 또는 `ARM`), `connection_state`, `readiness_state`, `execution_state`, `last_heartbeat_at`, `active_task_id`입니다. `connection_state`와 준비·실행 상태를 하나의 표시 문자열로 합치지 않습니다.

### Camera

필수 개념은 `id`, `robot_id`, `position`(`HEAD` 또는 `JOINT`), `connection_state`, `stream_state`, `frame_freshness`, `last_frame_at`, 선택적 `frame_rate`, `error_message`, `reconnect_state`입니다.

## 4. 상태와 전이 규칙

### 4.1 작업 상태

| 상태 | 의미 | 종료 상태 |
|---|---|---:|
| `PENDING` | 생성되어 실행을 기다림 | 아니오 |
| `VALIDATING` | 로봇·입력·중복 실행 조건 확인 중 | 아니오 |
| `RUNNING` | 하나 이상의 단계가 진행 중 | 아니오 |
| `CANCELING` | 취소 요청을 장치에 전달하고 정지 결과를 확인 중 | 아니오 |
| `NEEDS_INTERVENTION` | 실제 장치 상태나 명령 결과를 자동으로 확정할 수 없음 | 아니오 |
| `SUCCEEDED` | 모든 필수 단계의 성공 결과 확인 | 예 |
| `FAILED` | 작업을 계속할 수 없는 실패가 확인됨 | 예 |
| `CANCELED` | 장치 정지 또는 미실행이 확인되어 취소가 완료됨 | 예 |

허용되는 상위 전이는 다음과 같습니다.

```text
PENDING → VALIDATING | CANCELED
VALIDATING → RUNNING | FAILED | CANCELED | NEEDS_INTERVENTION
RUNNING → SUCCEEDED | FAILED | CANCELING | NEEDS_INTERVENTION
CANCELING → CANCELED | FAILED | NEEDS_INTERVENTION
NEEDS_INTERVENTION → CANCELED | FAILED | RUNNING
```

`NEEDS_INTERVENTION → RUNNING`은 운영자가 실제 상태를 확인하고 오케스트레이터가 안전한 재개 지점을 확인한 경우에만 허용합니다. 자동 재시작이나 미확인 명령 재전송은 금지합니다. 종료 상태는 별도 명시적 재시도 작업을 만들지 않는 한 바꾸지 않습니다.

### 4.2 단계 상태

| 상태 | 의미 |
|---|---|
| `PENDING` | 선행 조건 대기 또는 실행 전 |
| `COMMAND_SENT` | 어댑터가 명령 전송을 시도함 |
| `ACCEPTED` | 장치가 해당 명령을 수락함 |
| `RUNNING` | 장치가 실제 실행 중임을 보고함 |
| `SUCCEEDED` | 해당 단계의 최종 성공 결과를 받음 |
| `FAILED` | 해당 단계의 최종 실패 결과를 받음 |
| `TIMED_OUT` | 정해진 시간 안에 단계 완료를 확인하지 못함 |
| `CANCELED` | 단계가 실행되지 않았거나 장치 정지 완료를 확인함 |
| `UNKNOWN` | 명령이 실행됐는지 또는 결과가 무엇인지 판정할 수 없음 |

일반 경로는 `PENDING → COMMAND_SENT → ACCEPTED → RUNNING → SUCCEEDED`입니다. 장치가 즉시 결과를 반환하면 `ACCEPTED` 또는 `RUNNING`을 생략할 수 있습니다. `FAILED`와 확인된 `CANCELED`는 종료 상태입니다. `TIMED_OUT`은 deadline 내 결과를 받지 못했다는 사실을 기록하지만 장치 동작이 끝났다는 뜻은 아닙니다. `UNKNOWN`도 결과 미확정 상태이며 자동 후속 명령을 멈춥니다. 이후 동일한 `command_id`에 대한 권위 있는 최종 결과 또는 운영자 확인이 들어오면 상태를 조정할 수 있고, 그 변경과 근거를 이력에 남깁니다.

선행 단계가 끝나기 전 후속 단계는 `PENDING`으로 유지하고 `blocked_by_step_id` 또는 동등한 사유를 표시합니다. UI에서 이를 “대기”로 표시할 수 있지만 `BLOCKED`는 저장 상태 어휘에 포함하지 않습니다.

### 4.3 작업 시작 및 단일 실행 규칙

- 작업 생성은 저장만 하며 로봇 명령을 보내지 않습니다.
- 실행은 별도의 명시적 요청으로 시작합니다.
- MVP는 한 번에 하나의 관제 작업만 `VALIDATING`, `RUNNING`, `CANCELING`, `NEEDS_INTERVENTION` 상태로 실행합니다.
- 작업 선점은 원자적으로 처리해 동시에 도착한 두 실행 요청 중 하나만 성공시킵니다.
- 이미 다른 명령을 수행 중이거나 상태가 오래된 로봇에는 새 작업을 보내지 않습니다.
- 같은 `task_id`에 대한 실행 재요청은 중복 로봇 명령을 만들지 않습니다.
- 시작 전 선점 충돌은 `409 Conflict`로 거절하고 작업은 `PENDING`으로 유지합니다. 실행 요청이 접수된 뒤 검증에 실패하면 단계·원인을 이력에 남기고 `FAILED` 또는 상태 불명 시 `NEEDS_INTERVENTION`으로 끝냅니다.

### 4.4 이동에서 팔로 넘어가는 조건

오케스트레이터는 아래 조건을 모두 만족한 경우에만 `EXECUTE_ARM_TASK` 명령을 보냅니다.

1. 주행 명령이 현재 `task_id`, `step_id`, `command_id`로 수락되었음.
2. 해당 명령의 최종 이동 성공 결과를 받았고 결과가 현재 작업과 일치함.
3. 이동 로봇 상태가 연결되고 새 작업을 허용할 수 있음.
4. 작업이 취소·실패·개입 필요 상태가 아님.
5. 팔 로봇이 연결되고 작업 가능 상태임.

수락 이벤트, 진행률 100%, Nav2 서버 발견만으로 도착 성공을 추정하지 않습니다. timeout이나 연결 단절로 결과가 불명확하면 팔 단계를 실행하지 않습니다.

## 5. 명령·이벤트 계약

### 5.1 명령 봉투

어댑터에 전달하는 논리 명령은 다음 정보를 가집니다.

```json
{
  "task_id": "task-123",
  "step_id": "step-2",
  "command_id": "cmd-8b17",
  "robot_id": "mobile-01",
  "command_type": "NAVIGATE",
  "parameters": {},
  "deadline_at": "2026-10-24T10:01:00Z"
}
```

예시는 필드 의미를 설명하기 위한 것입니다. 장치별 `parameters`는 어댑터가 검증하며, UI에서 임의 셸 명령이나 raw ROS 명령을 제출하지 않습니다. 실제 deadline은 작업 유형별 설정으로 정합니다.

### 5.2 결과 이벤트 봉투

```json
{
  "event_id": "evt-91c2",
  "robot_id": "mobile-01",
  "task_id": "task-123",
  "step_id": "step-2",
  "command_id": "cmd-8b17",
  "event_type": "COMMAND_ACCEPTED",
  "occurred_at": "2026-10-24T10:00:01Z",
  "sequence": 4,
  "payload": {}
}
```

필수 이벤트 종류는 `COMMAND_ACCEPTED`, `EXECUTION_STARTED`, `PROGRESS`, `SUCCEEDED`, `FAILED`, `CANCELED`, `HEARTBEAT`, `CONNECTION_CHANGED`입니다. 전송 오류나 결과 대기는 관제 측 `TIMED_OUT` 또는 `UNKNOWN`으로 기록할 수 있습니다. 장치 프로토콜별 메시지는 어댑터가 이 공통 모델로 변환합니다.

이벤트 처리 규칙:

- `event_id` 중복은 한 번만 반영합니다.
- `task_id`, `step_id`, `command_id`가 현재 활성 명령과 일치해야 단계 결과에 적용합니다.
- 이전 작업의 늦은 완료 이벤트는 현재 작업 상태를 변경하지 않습니다.
- `occurred_at`은 장치가 보고한 시각, `received_at`은 관제 서버 수신 시각으로 각각 보존합니다. 장치가 sequence 번호를 제공하면 함께 기록하고, 미지원이면 서버 수신 순서만으로 장치 실행 순서를 추정하지 않습니다.
- 이벤트 순서가 불명확하면 성공으로 보정하지 않고 `UNKNOWN` 또는 `NEEDS_INTERVENTION`으로 전환합니다.

## 6. 관제 API 계약

아래는 UI와 관제 백엔드 사이의 목표 계약입니다. 실제 장치 통신 프로토콜을 정하는 API가 아닙니다.

| 메서드 | 경로 | 성공 의미 |
|---|---|---|
| `GET` | `/api/health` | 서버 가동 및 실행 모드 조회 |
| `GET` | `/api/robots` | 각 로봇의 연결·준비·실행 상태 조회 |
| `GET` | `/api/cameras` | 카메라마다 연결·스트림·프레임 상태 조회 |
| `POST` | `/api/tasks` | 입력 검증 후 `PENDING` 작업·표준 단계·`TASK_CREATED` 이벤트 생성; 성공은 `201 Created`; 명령은 보내지 않음 |
| `GET` | `/api/tasks` | 작업 요약 목록 조회 |
| `GET` | `/api/tasks/{task_id}` | 작업과 단계 이력 조회 |
| `POST` | `/api/tasks/{task_id}/start` | 실행 요청 접수; `202 Accepted`와 작업 ID·현재 상태 반환 |
| `POST` | `/api/tasks/{task_id}/cancel` | 취소 요청 접수; `202 Accepted`; 정지 결과는 별도 확인 |

비동기 실행 API의 `202` 응답은 “접수됨”을 뜻하며 작업 완료를 뜻하지 않습니다. 잘못된 필드·형식은 `400`, 없는 작업 ID는 `404`, 로봇 사용 중·상태 충돌·중복 실행은 `409`, 지원하지 않는 미디어 형식은 `415`로 응답합니다. 응답에 안정적인 `error_code`와 사람이 읽을 수 있는 `message`를 함께 담습니다. 작업 실행이 접수된 뒤 생기는 장치 오류는 HTTP 오류가 아니라 task/step 상태와 이력 이벤트로 반환합니다.

`destination`, `arm_task`의 세부 JSON 스키마, 페이지네이션, 인증, API 버저닝은 실제 목적지 표현과 배포 범위를 확정한 후 정합니다. 로컬 개발 단계의 localhost 바인딩을 외부 네트워크 접근이 가능한 서비스로 간주하지 않습니다.

## 7. 로봇 상태 계약

로봇별로 다음 세 관점을 분리합니다.

| 필드 | 값 예시 | 의미 |
|---|---|---|
| `connection_state` | `CONNECTED`, `DISCONNECTED`, `UNKNOWN` | 통신 상대를 확인할 수 있는지 |
| `readiness_state` | `READY`, `BUSY`, `NOT_READY`, `UNKNOWN` | 새 명령을 받을 준비가 되었는지 |
| `execution_state` | `IDLE`, `NAVIGATING`, `EXECUTING`, `STOPPING`, `UNKNOWN` | 현재 알려진 동작 |

상태 snapshot은 `robot_id`, `last_heartbeat_at`, `active_task_id`, 선택적 오류 정보를 포함합니다. heartbeat 허용 주기와 stale 임계값은 실제 장치가 제공하는 주기를 확인해 설정합니다. Nav2 서버 discovery는 통신 상대 발견일 뿐 `READY`나 작업 성공 근거가 아닙니다.

### 7.1 ROS 2/Nav2 action과 lifecycle의 역할

Nav2가 로봇 쪽 action server를 제공하고 관제 백엔드의 이동 어댑터가 action client가 되는 구성을 기본으로 합니다. 관제 UI와 백엔드 사이에는 이미 HTTP API가 있으므로, 관제 자체를 감싸는 ROS 2 action server는 현재 범위에서 만들지 않습니다. 다른 ROS 2 노드가 관제 작업 전체를 ROS action으로 시작·취소해야 한다는 요구가 생기면, 동일한 오케스트레이터를 호출하는 별도 ROS 인터페이스로 검토합니다.

모바일 로봇의 목적지 이동은 확인된 Nav2 action(현재 후보는 `NavigateToPose`)의 goal로 요청하고, action goal handle의 수락/거부, feedback, 최종 result를 관제 공통 이벤트로 변환합니다. ROS 2 action의 goal ID는 ROS 통신 추적에 사용하고, 관제의 `task_id`, `step_id`, `command_id`와 함께 저장해 어느 작업에서 나온 결과인지 연결합니다. ROS goal ID를 관제 명령 ID 대신 사용하지 않습니다.

관제의 이동 단계는 action의 최종 성공 result와 현재 활성 명령 식별자가 일치할 때만 성공합니다. goal 수락은 실행 시작의 근거일 뿐 도착 완료가 아니며, feedback과 거리/진행률도 최종 성공을 대신하지 않습니다. 취소는 action cancel 요청을 보내고 cancel 응답만으로 정지 완료로 보지 않습니다. 최종 action 결과와 로봇 상태를 확인해 취소 결과를 기록합니다. 결과 대기 중 관제 timeout이 발생해도 action을 무조건 재전송하지 않습니다.

ROS 2 lifecycle 상태는 Nav2의 `lifecycle_manager`와 실제 노드 구성을 확인할 수 있을 때만 readiness 진단에 사용합니다. 필요한 노드가 `active`인지 상태 화면과 작업 시작 전 조건에 반영할 수 있지만, action server 발견이나 lifecycle `active`만으로 로봇이 안전하거나 이동을 성공했다고 판정하지 않습니다. 관제가 lifecycle 전이를 임의로 호출해 Nav2를 configure/activate/shutdown하는 것은 기본 범위에 포함하지 않습니다. 운영자가 Nav2 수명 주기를 명시적으로 관제해야 한다는 요구와 지원 인터페이스가 확인될 때 별도 기능으로 계획합니다.

Nav2 action 이름·타입, namespace, goal frame 및 좌표/방향 변환, result 성공 판정, feedback 사용 범위, lifecycle node 목록과 상태 조회 방법은 실제 로봇의 ROS 2 배포판과 Nav2 구성을 확인한 후 확정합니다. 현재 구현된 action server discovery는 이 계약을 확인하는 사전 탐색이며 실제 goal 전송은 하지 않습니다.

## 8. 카메라 및 영상 계약

### 8.1 카메라 리소스

로봇팔의 `head_camera`와 `joint_camera`를 독립적으로 조회합니다. 최소 상태는 다음과 같습니다.

- `connection_state`: `CONNECTED`, `DISCONNECTED`, `UNKNOWN`
- `stream_state`: `STREAMING`, `NO_FRAMES`, `STOPPED`, `ERROR`, `UNKNOWN`
- `frame_freshness`: `FRESH`, `STALE`, `UNKNOWN`
- `last_frame_at`, 지원하는 경우 `frame_rate`, `error_message`, `reconnect_state`

프레임을 받은 직후 timestamp를 갱신합니다. timestamp가 기준 시간보다 오래되면 stale로 표시합니다. stale 기준 시간은 실제 FPS·전송 지연을 측정한 후 설정하며 임의의 고정값을 명세하지 않습니다.

### 8.2 영상 경로

영상 바이트는 작업 API 및 작업 상태 JSON과 분리된 카메라별 미디어 경로로 전달합니다. 구체적 전송 방식(예: ROS 이미지 토픽, HTTP/MJPEG 등), 인코딩, 해상도, 브라우저 endpoint는 장비 조사 뒤 결정합니다. 미연결 상태에서 샘플 프레임을 실영상처럼 표시하지 않습니다.

카메라 하나의 오류는 다른 카메라와 로봇 상태를 변경하지 않습니다. 특정 작업이 특정 카메라를 필수로 선언한 경우에만 해당 카메라 이상이 작업의 진행 조건에 영향을 줍니다.

## 9. 오류, 취소, 복구

| 상황 | 작업/단계 처리 | 후속 팔 단계 |
|---|---|---|
| 로봇이 명령 거부 | 해당 단계 `FAILED`, 작업 `FAILED` | 실행 금지 |
| 결과 대기 시간 초과 | 단계 `TIMED_OUT`, 작업 `NEEDS_INTERVENTION`; 실제 상태 확인 | 확인 전 실행 금지 |
| 통신 끊김 또는 명령 결과 불명 | 단계 `UNKNOWN`, 작업 `NEEDS_INTERVENTION` | 실행 금지 |
| 이동 실패 | 이동 단계 `FAILED`, 작업 `FAILED` | 실행 금지 |
| 팔 작업 실패 | 팔 단계 `FAILED`, 작업 `FAILED` | 완료 처리 금지 |
| 취소 요청 접수 | 작업 `CANCELING`; 장치 정지 결과 대기 | 새 명령 금지 |
| 정지 확인 | 활성 단계와 작업을 `CANCELED` 처리 | 실행 금지 |
| 취소 결과 불명 | `NEEDS_INTERVENTION` | 자동 재시도 금지 |
| 카메라 오류 | 해당 카메라 상태·오류만 갱신 | 작업이 카메라를 필수로 쓸 때만 판단 |

관제 프로세스는 장치 명령을 보내기 전에 명령 의도와 ID를 저장합니다. 명령 후 응답 전에 프로세스가 재시작하면 장치 동작을 추측하지 않습니다. 작업을 복구 확인 대상으로 표시하고, 장치 상태와 이벤트를 대조한 뒤 운영자 또는 검증된 복구 절차로 처리합니다. 같은 명령을 무조건 재전송하지 않습니다.

## 10. 비기능 요구와 안전 경계

- 상태 조회는 명령 완료를 기다리지 않고 응답해야 합니다.
- 같은 작업의 중복 시작과 오래된 이벤트가 중복 명령·잘못된 성공 판정을 만들면 안 됩니다.
- 상태 변경과 오류에는 작업·단계·명령 식별자 및 시각을 남깁니다.
- 로컬 개발 서버는 기본 `127.0.0.1`에서만 수신합니다.
- 로봇별 안전 제한, 비상정지, 충돌 방지와 작업 영역 보호는 장비의 안전 메커니즘이 담당합니다.
- UI의 정지/취소 버튼은 하드웨어 비상정지를 대체하지 않습니다.

## 11. MVP 수용 기준

1. 작업 생성만으로 로봇 명령이 발생하지 않고, 명시적 실행 요청이 있어야 시작됩니다.
2. 주행 명령의 전송·수락·실행·최종 결과가 서로 구분됩니다.
3. 주행 성공이 현재 작업 결과로 확인된 때에만 팔 단계가 시작됩니다.
4. 주행 거절·실패·timeout·연결 단절·결과 불명·취소 상황에서 팔 명령이 전송되지 않습니다.
5. 팔 작업은 최종 성공 결과를 받기 전까지 성공 처리되지 않습니다.
6. 중복 시작 및 이전 작업의 지연 이벤트가 현재 작업을 잘못 변경하지 않습니다.
7. 작업과 단계별 결과·오류·시각을 다시 조회할 수 있습니다.
8. 두 카메라의 상태와 프레임 신선도가 독립적으로 표시되고, 한 카메라 장애가 다른 카메라 상태를 덮어쓰지 않습니다.
9. 각 핵심 실패 흐름을 mock adapter로 재현하고 자동화된 검증으로 확인합니다.
10. 확인되지 않은 실제 장치 기능은 지원하는 것처럼 표시하지 않습니다.

## 12. 확정 전 결정사항

- 이동 로봇의 배포 환경, Nav2 목표/결과 및 상태 신선도 판정 기준
- 팔 로봇 자동 작업 명령과 장치 준비·작업 완료 이벤트 경로
- 현재 LeRobot 수동 텔레오퍼레이션을 최종 관제 기능에 포함할지 여부
- 목적지의 waypoint/좌표 표현, 좌표계, 허용 범위
- 팔 작업 종류와 파라미터 schema
- Head Camera 장치 및 주소, Joint Camera의 실제 노드(`/dev/video4`/`video5`) 식별
- 카메라 전송 규격, FPS, stale 임계값, 재연결 정책
- 작업 유형별 timeout과 수동 복구 정책
- 배포 시 인증, 접근 제어, 로그 보존과 사용자 감사 범위
