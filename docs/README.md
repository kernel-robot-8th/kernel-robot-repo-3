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
- [정면 카메라 선반 위치 보정](implementation/shelf-pose-calibration.md): 선반 모서리를 찍어 solvePnP로 정면 카메라 위치를 구하고 6칸 영역을 화면에 투영하는 도구
- [OMX 명령 HTTP 인터페이스](implementation/arm-command-api.md): 관제가 OMX 명령(칸 자세, 원상복귀, 학습 정책)을 넘기고 OMX 워커가 long-poll로 받아 결과를 보고하는 계약, 모의 워커 실행 방법
- [카메라 내부 보정](implementation/camera-intrinsic-calibration.md): 체커보드로 카메라 내부 파라미터를 구하는 오프라인 도구 (천장 카메라 주차 판정과 정면 카메라 선반 위치 보정에 사용)
- [캘리브레이션 파일 지도](implementation/calibration-files.md): 카메라별 보정 스크립트, 결과 파일 위치와 현황, 팀 체커보드 인쇄 파일
- [선반 층 영상 영역 초안](implementation/shelf-image-regions.md): 저장된 관측 frame에 polygon 좌표를 입력하는 브라우저 도구와 provenance를 확인해 review-only ROI artifact를 만드는 CLI
- [OMX 실물 자세 기록 — 2026-10-31](implementation/omx-live-pose-record-2026-10-31.md): 딸기를 손목 카메라 시야에 넣기까지의 초기/현재 관측, 엔코더 snapshot 및 재현 제한
- [실행 방법](implementation/runbook.md): 모의 서버와 팔 텔레오퍼레이션 실행
- [개발 로드맵](development-plan.md): 에픽 순서, 목적, 의존성 및 전체 진행 상황
- [Pinky 도착 판정과 로봇 진행 계획](plans/pinky-arrival-gate.md): 천장 카메라 주차 판정을 팔 단계 시작 조건으로 쓰는 흐름, 미정 항목과 실물 적용 전 조건
- [Pinky 주차 판정 C++ ROS 2 노드 전환 계획](plans/pinky-parking-ros2-node.md): 카메라 드라이버·C++ 판정 노드·YOLO를 영상 토픽 하나로 나눠 쓰고, 관제는 토픽·서비스로 받아 웹 화면을 유지하는 구조
- [에픽 1 — 작업·단계 이력](epics/epic-01-task-history.md): 첫 구현 묶음의 세부 작업과 확인 기준
- [에픽 2 — 모의 정상 작업 흐름](epics/epic-02-mock-happy-path.md): mock 기반 실행 흐름의 세부 작업과 확인 기준

**읽는 기준:** 목표 동작은 시스템 명세를, 현재 동작은 구현 현황과 실행 방법을 기준으로 확인합니다. 둘이 다르면 현재 구현 현황이 실제 코드 상태를 설명합니다.
