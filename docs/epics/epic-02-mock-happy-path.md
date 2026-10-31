# 에픽 2: 모의 정상 작업 흐름

## 목표

실제 장비 없이 관제 화면에서 작업 시작, 이동 완료, 팔 작업 완료, 작업 성공까지의 전체 정상 흐름을 확인합니다. 다음 단계는 앞 단계의 최종 성공 이벤트가 확인된 경우에만 시작합니다.

## 선행 조건

- [에픽 1: 작업·단계 이력](epic-01-task-history.md) 완료
- 명령·이벤트의 식별자와 상태 전이 규칙 확정

## 범위

- 공통 명령·이벤트 봉투
- 비동기 실행과 동시 작업 선점 규칙
- mock 이동/팔 어댑터
- 화면의 진행 상태와 저장된 이력

장치 adapter의 실패, timeout, 프로세스 재시작 뒤 결과 불명 복구는 [개발 로드맵](../development-plan.md)의 실패·취소·복구 에픽에서 다룹니다. 이 slice의 취소는 in-process mock worker에 한정합니다.

## 작업 순서와 확인 지점

### 1. 공통 명령·이벤트 모델

- [ ] 내부 명령 모델에 `task_id`, `step_id`, `command_id`, `robot_id`, 종류, `parameters`, `deadline` 정의
- [ ] 이벤트 모델에 `event_id`, 같은 작업/단계/명령 식별자, 이벤트 종류, `occurred_at`, `received_at`, `sequence`, `payload` 정의
- [ ] ROS 2 action goal ID를 보조 추적 정보로 둘 수 있게 adapter 이벤트 payload/metadata 경계를 마련하고 관제 `command_id`와 분리
- [ ] 장치별 payload가 mock/실제 adapter 바깥 공통 모델에 섞이지 않도록 경계 설정
- [ ] 명령 수락과 명령 최종 성공을 서로 다른 이벤트로 표현

**확인:** 콘솔이나 API 예시에서 공통 필드와 이벤트 봉투를 검토합니다.

### 2. 비동기 실행과 단일 작업 선점

- [x] `POST /api/tasks/{task_id}/start` 추가; 신규 접수에 `202`, 반복 활성 요청에 현재 상태 반환
- [x] 단일 서버 프로세스 안에서 활성 작업을 선점해 동시에 한 작업만 mock 실행
- [x] 다른 작업이 이미 활성 상태면 `409` 반환, 요청 작업 상태 유지
- [x] 동일 작업의 반복 시작 요청은 현재 상태를 반환하고 worker를 중복 생성하지 않음
- [x] 장치 readiness 대신 mock mode gate와 `VALIDATE_ROBOTS` mock 단계를 사용; ROS 2/실물 모드에서는 실행 요청을 거부
- [x] 단계 실행을 HTTP 요청과 분리한 daemon worker에서 진행
- [x] 시작 시 `VALIDATING` 상태와 `START_REQUESTED` 이벤트를 기록
- [x] mock 단계 이벤트 payload에 `task_id`, `step_id`, `command_id`를 연결하고, 실행되지 않은 취소 단계는 `command_id: null`로 남김

**확인:** 화면에 시작 상태가 보이고, 같은 작업 재요청이 중복 실행을 만들지 않는지 확인합니다. 다른 작업의 동시 실행은 `409`가 되고 해당 작업이 `PENDING`으로 남는지 확인합니다.

### 3. 이동 모의 어댑터

- [x] mock 실행기가 이동 단계를 접수, 실행 시작, 최종 성공 순서로 저장
- [x] step 이력과 event payload에 현재 `task_id`, `step_id`, `command_id`, `source: mock`을 연결
- [x] mock 결과를 단계 상태와 이력에 트랜잭션으로 반영
- [x] terminal step 성공 결과를 확인한 뒤 다음 단계로 이동
- [ ] goal 수락/feedback/final result를 공통 command accepted/progress/terminal result로 매핑하고 ROS action 최종 성공만 단계 성공으로 반영
- [ ] Nav2 action discovery 및, 구성이 확인된 경우 lifecycle `active` 상태를 readiness 입력으로 취급하되 둘 중 어느 하나만으로 goal 결과를 추정하지 않음

**확인:** 이동 단계가 접수 → 실행 중 → 성공으로 바뀌고 단계 이벤트 시각이 추가됩니다.

### 4. 이동 완료 확인 및 팔 모의 어댑터

- [ ] 현재 작업·단계·명령 ID가 모두 일치하는 이동 최종 성공만 수용
- [x] 이동 단계 최종 성공을 저장한 뒤 도착 확인 및 팔 단계를 시작
- [ ] 이동 단계가 성공이 아니거나 이벤트 식별자가 맞지 않으면 팔 adapter를 호출하지 않음
- [x] 팔 mock 단계가 접수, 실행 시작, 최종 성공 이벤트를 생성
- [x] `POST /api/tasks/{task_id}/cancel`을 추가해 in-process mock worker의 정지 확인 후 취소 상태를 기록
- [x] 아직 시작되지 않은 작업 취소는 task와 모든 pending step 결과/event를 단일 저장소 트랜잭션으로 기록

**확인:** 이동 성공 이벤트 뒤에 팔 단계가 표시되고, 두 단계의 시간 순서를 타임라인에서 확인합니다.

### 5. 최종 상태와 화면 갱신

- [x] 선택 작업 상세와 목록 상태를 1초 간격으로 갱신
- [x] 필수 단계의 최종 성공을 확인한 뒤 작업을 `SUCCEEDED`로 기록
- [x] `started_at`, `finished_at`, `active_step_id`, 단계별 결과를 상세 응답에 반영
- [x] 저장된 작업·단계·이벤트를 새로고침 뒤 다시 조회할 UI/API 경로 유지

**완료 조건:** 실제 장비 없이 화면과 저장된 이력에서 `PENDING → VALIDATING → RUNNING → SUCCEEDED` 및 이동 후 팔 실행 순서를 확인할 수 있습니다.

실제 Nav2 action 및 lifecycle 인터페이스의 확정·연결은 [개발 로드맵](../development-plan.md)의 ROS 2·장치 인터페이스 조사와 별도 수용 단계에서 수행합니다.

**구현 메모:** mock 실행 API, 단일 프로세스 선점, 시작·취소 UI, 구조화된 `pick_request`, optional DB migration을 추가했습니다. 새 task step과 mock command event는 `mobile-01`/`arm-01` 식별자를 동일하게 사용하며, 기존 DB에 저장된 `MOBILE`/`ARM` 값은 조회 응답에서 정규화합니다. 단계 상태 변경마다 DB event ID와 `occurred_at`/`received_at`을 기록합니다. 현재 실행 중인 동일 command에 대해서만 task 상태·`active_step_id`·step 상태·command ID를 저장소에서 재확인한 뒤 비전이 없는 모의 `PROGRESS: 50%` 이력을 한 번 추가합니다. start/cancel 및 task target-observation/target-selection 오류는 안정적인 `error_code`와 `message`를 반환하고 기존 `error` 응답 필드도 유지합니다. ID와 진행률은 실제 장치 식별자·feedback 또는 ROS action goal ID가 아닙니다. Python 구문 확인은 수행했지만 테스트 suite와 브라우저 수동 확인은 미수행이며, 공통 event envelope 전체 구현과 완료 조건의 end-to-end 확인은 남아 있습니다.
