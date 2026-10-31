# 프로젝트 문서

문서는 **목표로 만들 시스템**과 **코드가 지금 제공하는 기능**을 분리합니다.

## 개발 목표

- [시스템 명세](spec/system-spec.md): 범위, 작업 규칙, 상태 및 카메라 요구사항
- [데이터·인터페이스 계약 초안](spec/interface-contracts.md): 주문·작업·로봇 상태, 메시지 경계와 미확정 항목
- [목표 데이터 흐름](spec/data-flow.md): 완성 목표의 컴포넌트와 메시지 흐름
- [원본 제어 흐름 요구사항](../robot-control-flow.md): 사용자가 관리하는 상세 요구사항

## 현재 코드

- [구현 현황](implementation/current-state.md): API, 코드 위치, 구현된 기능과 미구현 기능
- 제한 규칙 기반 자연어 해석은 [실행 방법](implementation/runbook.md)의 작업 이력 절차를 참고하세요. 결과는 task 등록 전 검토가 필요한 초안입니다.
- [ROS 2·관제 인터페이스 조사](implementation/ros2-inventory.md): 현재 workspace와 관제 경계, 정적 토픽/action 목록 및 확인되지 않은 runtime 정보
- [시뮬레이션 설정](implementation/simulation-setup.md): MuJoCo ROS 2 선행조건, 분리 domain 실행과 관제 read-only 상태 확인
- [좌표 목표 초안](implementation/coordinate-target-draft.md): 관제 입력을 미실행 JSON 초안으로 저장하는 임시 기능
- [카메라 좌표 초안](implementation/camera-intrinsic-calibration.md): 체커보드/선반 평면 보정 및 fresh task 관측 후보를 2D 평면 draft로 투영하는 오프라인 도구와 제한
- [선반 층 영상 영역 초안](implementation/shelf-image-regions.md): 저장된 관측 frame에 polygon 좌표를 입력하는 브라우저 도구와 provenance를 확인해 review-only ROI artifact를 만드는 CLI
- [Pick & Place 현장 입력표](implementation/field-measurement-sheet.md): 첫 고정 시나리오, 카메라/선반 측정 및 OMX 기준 확인을 위한 빈 기록표
- [OMX MuJoCo 실행 안내](../simulation/mujoco_omx/README.md): 공식 OMX MJCF 기반 단독 시뮬레이터, Python 제어 및 카메라 이미지 실행 방법
- [OMX MuJoCo 기술 기록](implementation/mujoco-omx.md): 모델 출처, 기존 IK/TCP 계산 및 실물 적용 제한
- [실물 관측 자세의 MuJoCo 실험 — 2026-10-31](implementation/mujoco-live-pose-experiment-2026-10-31.md): 실물 encoder snapshot의 임시 MuJoCo 환산, 합성 카메라 한계와 접근 IK/경로 접촉 관측
- [IK 후보 JSON Schema](implementation/mujoco-ik-candidate.schema.json) 및 [link6↔TCP artifact Schema](implementation/link6-tcp-transform.schema.json): 오프라인 solver 결과와 검토 전용 TCP 측정/frame 대응 artifact의 데이터 계약
- TCP artifact 초안 생성은 실측값·frame mapping 검토·근거 파일을 받은 `scripts/create_link6_tcp_transform_draft.py`에서 수행합니다. 생성물은 `DRAFT_ONLY`로 남으며, 파일 계약과 근거 해시는 MuJoCo 비의존 `scripts/validate_link6_tcp_transform.py`에서 확인합니다.
- [그리퍼 파지 기하 측정 schema](implementation/gripper-geometry-measurement.schema.json): 측정한 closing/approach axes, finger contact patches와 opening gap 범위를 TCP calibration 및 evidence hash에 연결하는 초안 계약; `scripts/create_gripper_geometry_draft.py`가 operator record로 `DRAFT_ONLY` artifact를 만들고 `scripts/validate_gripper_geometry.py`가 형식/provenance만 확인
- [그리퍼 측정 입력 schema](implementation/gripper-geometry-measurement-input.schema.json): 실제 계측값으로 작성기 입력 JSON을 만들 때 사용할 strict field contract
- [OMX 관절 목표 파일 형식](implementation/omx-joint-pose-format.md): 장치에 연결하지 않는 첫 관절 목표 입력 계약 및 검사 방법
- [OMX 실물 자세 기록 — 2026-10-31](implementation/omx-live-pose-record-2026-10-31.md): 딸기를 손목 카메라 시야에 넣기까지의 초기/현재 관측, 엔코더 snapshot 및 재현 제한
- [실행 방법](implementation/runbook.md): 모의 서버와 팔 텔레오퍼레이션 실행
- [개발 로드맵](development-plan.md): 에픽 순서, 목적, 의존성 및 전체 진행 상황
- [OMX 로봇팔 좌표 제어 계획](plans/omx-coordinate-control.md): 관절 목표부터 검증된 말단 좌표 제어까지의 단계별 계획
- [에픽 1 — 작업·단계 이력](epics/epic-01-task-history.md): 첫 구현 묶음의 세부 작업과 확인 기준
- [에픽 2 — 모의 정상 작업 흐름](epics/epic-02-mock-happy-path.md): mock 기반 실행 흐름의 세부 작업과 확인 기준
- [에픽 7 — 에피소드 없는 선반 Pick & Place](epics/epic-07-episode-free-shelf-picking.md): 기존 관제와 카메라 관측·좌표 초안·MuJoCo 검증의 단계별 갭

**읽는 기준:** 목표 동작은 시스템 명세를, 현재 동작은 구현 현황과 실행 방법을 기준으로 확인합니다. 둘이 다르면 현재 구현 현황이 실제 코드 상태를 설명합니다.
