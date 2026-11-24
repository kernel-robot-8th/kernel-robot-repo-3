# 프로젝트 문서

문서는 **목표로 만들 시스템**(명세)과 **코드가 지금 하는 일**(구현)을 나눠 적습니다. 둘이 다르면 구현 문서가 실제 코드 상태를 설명합니다.

## 처음 읽는 순서

1. [저장소 README](../README.md): 시스템 구성, 주문 한 건의 순서, 실행 방법
2. [주문 흐름](implementation/order-flow.md): 주문 → 선반 확인 → 팔 배정 → 핑키 도착 → 적재 → 확인 → 배송, 모의/실제 경계
3. [구현 현황](implementation/current-state.md): 코드 위치, API 목록, 현재 동작의 경계
4. [시스템 명세](spec/system-spec.md): 목표 동작과 필수 제어 규칙

## 개발 목표 (명세)

| 문서 | 내용 |
|---|---|
| [시스템 명세](spec/system-spec.md) | 범위, 작업 규칙, 상태와 카메라 요구사항 |
| [데이터·인터페이스 계약 초안](spec/interface-contracts.md) | 주문·작업·로봇 상태, 메시지 경계, 미확정 항목 |
| [목표 데이터 흐름](spec/data-flow.md) | 완성 목표의 컴포넌트와 메시지 흐름 |
| [원본 제어 흐름 요구사항](../robot-control-flow.md) | 초기 요구사항 기록 (이동 로봇 1대, 팔 1대, 좌표 기반 팔 제어 전제) |

## 현재 구현

### 관제 서버와 주문

| 문서 | 내용 |
|---|---|
| [구현 현황](implementation/current-state.md) | 코드 구성, 구현된 API, 현재 동작의 경계 |
| [주문 흐름](implementation/order-flow.md) | 단계와 실패 코드, 배정 규칙, 자동 출발, 큰 주문 운송분, 핑키 관제 HTTP 연결, 적재 자리 도착 순서 |
| [가상 핑키 관제](implementation/virtual-pinky-fleet.md) | 핑키 실물 없이 주문 → 적재 → 수령 → 주차 흐름을 끝까지 돌리기 |
| [실행 방법](implementation/runbook.md) | 모의 서버, 주문 흐름, 팔 정책, 카메라, 주차 판정 노드 실행 절차 |

자연어 주문 해석(제한 문법 초안)은 [실행 방법](implementation/runbook.md)의 작업 이력 절차를 참고하세요. 결과는 작업 등록 전에 검토가 필요한 초안입니다.

### 로봇팔

| 문서 | 내용 |
|---|---|
| [OMX 명령 HTTP 인터페이스](implementation/arm-command-api.md) | 관제가 팔 명령(칸 자세, 원상복귀, 학습 정책)을 넘기고 워커가 long-poll로 받아 보고하는 계약, ROS 2 집기 액션, 모의 워커 |
| [OMX 팔 ROS 2 단계별 실습](implementation/omx-ros-practice.md) | `/omx/move_to_slot` action으로 팔을 단계별로 다뤄 보는 실습 |

### 핑키 (이동 로봇)

| 문서 | 내용 |
|---|---|
| [핑키 관제 README](../pinky_fleet_dashboard/README.md) | 핑키 관제의 기능, 처음 세팅, 실행, 세부 동작 기록 |
| [핑키 관제 운영](../pinky_fleet_dashboard/OPERATIONS.md) | 사용자 서비스 실행, 로봇 코드 적용, 공유 구역, 로그 |
| [라이다·초음파 센서 퓨전](implementation/sensor-fusion.md) | 확률 점유 격자 모델, Nav2 연결, 녹화·평가 방법 |

### 카메라와 비전

| 문서 | 내용 |
|---|---|
| [주차 판정 C++ / ROS 2](../cpp/README.md) | 천장 카메라 ArUco 주차 판정 패키지, 노드 인터페이스, 빌드·테스트 |
| [ROS 2 인터페이스 목록](implementation/ros2-inventory.md) | `cpp/` 패키지, 토픽·서비스·액션, 관제가 ROS에 붙는 방식 |
| [카메라 내부 보정](implementation/camera-intrinsic-calibration.md) | 체커보드로 카메라 내부 파라미터를 구하는 오프라인 도구 |
| [정면 카메라 선반 위치 보정](implementation/shelf-pose-calibration.md) | 선반 모서리로 solvePnP를 풀어 정면 카메라 위치를 구하고 칸 영역을 화면에 투영 |
| [캘리브레이션 파일 지도](implementation/calibration-files.md) | 카메라별 보정 스크립트, 결과 파일 위치와 현황, 체커보드 인쇄 파일 |
| [YOLO 모델](../yolo/README.md) | 물체 검출 데이터 준비·학습 스크립트, 팀 모델 |

### 시뮬레이션

| 문서 | 내용 |
|---|---|
| [창고 시뮬레이션](../simulation/README.md) | Gazebo Harmonic 창고 월드, 가상 핑키 관제 |

## 계획과 지난 기록

| 문서 | 내용 |
|---|---|
| [개발 로드맵](development-plan.md) | 에픽 순서, 목적, 의존성, 전체 진행 상황 |
| [Pinky 주차 판정 C++ ROS 2 노드 — 남은 계획](plans/pinky-parking-ros2-node.md) | 끝난 1~5단계는 [cpp/README.md](../cpp/README.md), 남은 YOLO 노드 분리와 확인 사항 |
| [Pinky 도착 판정과 로봇 진행 계획](plans/pinky-arrival-gate.md) | 지난 계획. 지금은 주문 흐름에 구현됨 |
| [에픽 1 — 작업·단계 이력](epics/epic-01-task-history.md) | 첫 구현 묶음의 세부 작업과 확인 기준 |
| [에픽 2 — 모의 정상 작업 흐름](epics/epic-02-mock-happy-path.md) | 모의 기반 실행 흐름의 세부 작업과 확인 기준 |
