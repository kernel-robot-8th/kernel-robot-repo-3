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

현재 코드의 `Task.destination`과 `Task.arm_job`은 입력 문자열입니다. 주문 픽업 위치, 물품 ID·수량, OMX 좌표 또는 장치 명령 payload로 해석되지 않습니다.

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
| 관제 UI → HTTP API | 작업 생성·조회, 향후 실행·취소 요청 | HTTP 접수와 작업 ID; 비동기 완료는 별도 상태/이벤트 | 현재 localhost HTTP 구현; 실행·취소 미구현 |
| 오케스트레이터 → Pinky 어댑터 | task/step/command ID, 목적지, deadline | 수락/거절, 진행, terminal 성공·실패, 취소 결과 | 공통 의미 확정; ROS 2 action 이름·타입·namespace·frame 미확정 |
| 오케스트레이터 → OMX 어댑터 | task/step/command ID, 작업 종류·파라미터 | 수락/거절, 진행, 파지·적재 최종 결과 또는 실패 | 공통 의미 확정; 자동 명령·완료 회신 미확정. LeRobot CLI는 현재 수동 텔레오퍼레이션 |
| 로봇 어댑터 → 이벤트 처리기 | robot/task/step/command ID, 장치 결과 | `ACCEPTED`, `EXECUTION_STARTED`, `PROGRESS`, `SUCCEEDED`, `FAILED`, `CANCELED` 또는 미확정 | 공통 이벤트 의미 확정; 장치별 실제 메시지 미확정 |
| Head/Joint Camera → 관제 UI | 카메라별 프레임·연결·스트림·마지막 프레임 시각 | 카메라별 오류; 다른 카메라나 로봇 상태에 전파하지 않음 | 팔에 카메라 2대라는 점 확정; 실제 식별·전송 정보는 구현 현황 참조 |

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

## 6. 전송 방식: 합의된 범위와 미정 항목

| 경계 | 현재 합의·관측된 방식 | 미정·미구현 |
|---|---|---|
| UI ↔ 관제 | localhost HTTP API | 원격 배포 인증·TLS·접근 제어 |
| 관제 ↔ 저장소 | SQLite, SQLAlchemy ORM, Alembic | 원격/다중 사용자 배포 DB |
| 관제 ↔ Pinky | ROS 2/Nav2 action client를 기본 방향으로 검토 | 배포판, domain, action 이름·타입, namespace, frame, lifecycle 진단; 현재는 action server discovery만 구현 |
| 관제 ↔ OMX | 공통 명령·결과 모델을 어댑터에서 사용 | 자동 작업 API, READY 신호, 최종 결과/오류 경로; 수동 LeRobot 프로세스 관리는 이 계약을 대신하지 않음 |
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
