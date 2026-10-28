# 프로젝트 문서

문서는 **목표로 만들 시스템**과 **코드가 지금 제공하는 기능**을 분리합니다.

## 개발 목표

- [시스템 명세](spec/system-spec.md): 범위, 작업 규칙, 상태 및 카메라 요구사항
- [데이터·인터페이스 계약 초안](spec/interface-contracts.md): 주문·작업·로봇 상태, 메시지 경계와 미확정 항목
- [목표 데이터 흐름](spec/data-flow.md): 완성 목표의 컴포넌트와 메시지 흐름
- [원본 제어 흐름 요구사항](../robot-control-flow.md): 사용자가 관리하는 상세 요구사항

## 현재 코드

- [구현 현황](implementation/current-state.md): API, 코드 위치, 구현된 기능과 미구현 기능
- [좌표 목표 초안](implementation/coordinate-target-draft.md): 관제 입력을 미실행 JSON 초안으로 저장하는 임시 기능
- [OMX 관절 목표 파일 형식](implementation/omx-joint-pose-format.md): 장치에 연결하지 않는 첫 관절 목표 입력 계약 및 검사 방법
- [실행 방법](implementation/runbook.md): 모의 서버와 팔 텔레오퍼레이션 실행
- [개발 로드맵](development-plan.md): 에픽 순서, 목적, 의존성 및 전체 진행 상황
- [에픽 1 — 작업·단계 이력](epics/epic-01-task-history.md): 첫 구현 묶음의 세부 작업과 확인 기준
- [에픽 2 — 모의 정상 작업 흐름](epics/epic-02-mock-happy-path.md): mock 기반 실행 흐름의 세부 작업과 확인 기준

**읽는 기준:** 목표 동작은 시스템 명세를, 현재 동작은 구현 현황과 실행 방법을 기준으로 확인합니다. 둘이 다르면 현재 구현 현황이 실제 코드 상태를 설명합니다.
