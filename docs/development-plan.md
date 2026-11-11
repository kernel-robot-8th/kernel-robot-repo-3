# 개발 로드맵

이 로드맵은 기존 Python 관제시스템과 ROS 2/Gazebo workspace를 보존하면서, 에피소드 없는 제한적 Pick & Place를 단계적으로 추가하는 계획입니다. 목표 요구사항은 [시스템 개발 명세](spec/system-spec.md), 현재 확인된 기능은 [구현 현황](implementation/current-state.md)을 기준으로 합니다. 2026-11-11부터 팔은 좌표 없이 칸 자세 재생과 학습(ACT) 정책으로만 움직이고, 작업 흐름은 핑키 도착 확인 → OMX 물체 배정 → 적재 확인 → 핑키 완료 신호 순서입니다.

## 계획 원칙

- 기존 관제 API, 작업 이력, 카메라 모니터링, 모의 기본 동작을 유지하고 필요한 경계만 확장합니다.
- 새 ROS 2 패키지·토픽·서비스·액션은 현재 인터페이스 조사를 마친 뒤에만 결정합니다. 이름과 성공 의미를 추측해 계약으로 고정하지 않습니다.
- 작업 생성/계획, 장치 명령 접수, 장치 실행, 최종 결과 확인을 구분하고 기존 task/step/event 이력에 연결합니다.
- 인식과 좌표 관측은 명령 없는 경로에서 먼저 확인합니다. 모의 결과와 실물 결과는 별도 상태와 화면으로 구분합니다.
- 실제 장치 명령은 사용자 요청/승인과 장치별 인터페이스·안전 경계 확인을 모두 전제로 합니다. discovery, 프로세스 생존, 카메라 프레임, 시뮬레이션은 장치 준비나 동작 성공의 증거가 아닙니다.
- 각 확인 지점은 작은 수직 단위로 완료하고, 정해진 에픽을 한 번에 모두 구현하지 않습니다.

## 저장소 기준선과 재사용 경계

현재 저장소에는 localhost HTTP 기반 Python 관제 서버, SQLite/SQLAlchemy 작업·단계·이벤트 이력, Head/Joint 카메라 모니터링, 관측 전용 YOLO 결과, Nav2 action server discovery, 비활성화 기본값의 LeRobot 텔레오퍼레이션 프로세스 관리가 있습니다. 작업 생성은 `PENDING`을 저장하고, mock 모드의 명시적 시작 요청은 별도 mock worker로 진행합니다. 실제 장치 실행 adapter는 아직 연결되지 않았습니다. 상세 사실은 [현재 구현 현황](implementation/current-state.md)을 기준으로 합니다.

`simulation/ros2_ws`에는 Gazebo Harmonic 창고 모델을 담은 `toktak_warehouse` 패키지가 있습니다. 확인된 bridge 토픽은 `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/scan`, `/imu/data`, `/joint_states`입니다. 이는 현재 Gazebo 이동로봇 모델의 브리지 계약입니다. 이 workspace에서 Nav2 action server, OMX 제어 노드, 카메라 드라이버, Pick & Place 인터페이스는 확인되지 않았습니다. 실행 중인 ROS graph의 전체 인터페이스 목록으로 확대 해석하지 않습니다.


## 에픽 순서 및 상태

| 순서 | 에픽 | 목적 | 선행 조건 | 현재 상태 |
| --- | --- | --- | --- | --- |
| 1 | [작업·단계 이력](epics/epic-01-task-history.md) | 표준 단계, 이벤트, 상태 전이와 추적 가능한 조회 기반 | 기존 저장 기능 | 진행 중 |
| 2 | [모의 정상 작업 흐름](epics/epic-02-mock-happy-path.md) | 관제 화면에서 작업 시작부터 mock terminal 결과까지 확인 | 에픽 1 | 진행 중 — 구조화 대상 조건과 시작/취소 slice 구현, 확인 미실행 |
| 3 | 실패·취소·복구 | 중복, 거절, timeout, 통신 단절, 미확정 결과를 안전하게 보존 | 에픽 2 | 진행 중 — mock 단계 시작 command-state와 취소 요청, 실패/취소 종결 처리를 같은 프로세스 lock으로 직렬화; mock 실패에서 미실행 단계 표시; 설정 가능한 mock step 실패 주입 추가; worker 소유 상태를 읽기 전용 표시; 수동 재시작 복구와 장치 이벤트 복구 미구현 |
| 4 | 기존 ROS 2·장치 인터페이스 조사 및 어댑터 계약 | workspace, 배포판, 장치별 실제 action/topic 및 상태 신호 목록을 확인하고 공통 관제 경계 결정 | 코드/설정 및 실행 환경 관측 | 진행 중 — 정적 인벤토리와 2026-10-30 domain 0/31, 10-31 domain 0 및 31의 읽기 전용 graph 관측 기록; 관제의 명시 설정 `JointState` 구독 경계 구현(기본 비활성), 실제 OMX publisher source와 runtime 미확인 |
| 5 | 카메라 관측 경로 완성 | 두 카메라 동시 관측, 장착 위치·스트림 특성·대상 선택 상태 확인 | 카메라 접근 정보 | 진행 중 — 픽셀 후보와 task 감사 이벤트, 색상 hint 및 명시적 ROI draft의 층 비교를 검토 전용으로 연결하고 저장 PNG의 polygon JSON 입력 도구 추가; Head/Joint feed별 선택형 YOLO 및 YOLO-World prompt mode를 같은 provenance UI/API에 연결(각 camera 별도 weights 필요, 모델 없음); 자연어 초기 대상 클래스는 Head detector로 한정; YOLO-World task observation에 요청 class one-shot prompt와 직렬화/복원 provenance를 추가했으나 모델 weights가 없어 runtime 미확인; 2026-10-31 V4L2 열거에서 numeric `/dev/video4`가 HCAM0로 재배정되어 Joint/Head alias 충돌을 확인, 이전 Joint Innomaker 선택을 보존하는 by-id 기본값과 duplicate realpath 보호를 추가; Innomaker 단일 RGB frame은 선반/역할을 식별하지 못했고 HCAM0 stream artifact는 유효 JPEG가 아니었음; 실제 물리 장착·역할은 화면/장착 대조 미완료; 실제 생활용품 정확도·분할 mask·실제 층 polygon·색상 보정·실측 3D 위치·wrist extrinsic 및 camera/joint timestamp alignment 미구현 |
| 8 | Pick & Place mock 오케스트레이션 및 관제 표시 | 기존 작업 이력/API/UI에 인식·계획·실행 단계를 추가하고 mock 경로 확인 | 에픽 1–7, 데이터 계약 | 진행 중 — 대상 조건 task 단계 템플릿, mock task/step/command/robot ID, 저장소 확인을 거친 mock `PROGRESS: 50%`, 로봇별 step UI, 검토용 자연어 초안 및 읽기 전용 joint-state 표시 구현; 실제 카메라/좌표/물리 sim과 연결되지 않았고 end-to-end 확인 미실행 |
| 9 | 제한된 실물 adapter 수용 | 명시적으로 승인된 현장 절차에서 확인된 인터페이스만 연결 | 에픽 3–8, 안전·완료 신호 확인 | 차단 조건 충족 전 미착수 |
| 10 | 통합 수용 및 범위 확장 | 첫 고정 시나리오를 검증한 뒤 조건을 하나씩 확대 | 에픽 9 | 계획 |

에픽 4의 결과로 새 ROS 2 패키지가 필요하다고 확인되기 전까지 새 패키지나 중복 인터페이스를 만들지 않습니다. 관제 작업 전체를 ROS 2 action으로 노출하는 것도 실제 소비자 요구가 확인된 경우에만 검토합니다. 긴 장치 작업은 이미 확인된 장치 action이 있으면 우선 사용합니다.

## 6주 기준 개발안

기간은 가용 장비·작업 시간과 기존 조사 상태에 따라 조정합니다. 각 주의 완료 기준이 충족되지 않으면 다음 주의 의존 작업을 시작하지 않습니다.

| 주차 | 초점 | 주요 산출물 | 완료 확인 |
| --- | --- | --- | --- |
| 1 | 기존 시스템 기준선·인터페이스 조사 | 패키지/launch/config/URDF/bridge 및 관제 API·상태 흐름 목록, 확인/미확인 구분, 첫 고정 작업 시나리오 | 기존 기능 재사용 경계와 미확정 인터페이스가 문서화됨; 코드 변경 없이 리뷰 가능 |
| 2 | 데이터 계약과 모의 작업 수직 단위 | optional `pick_request`, 카메라 관측 task 이벤트, 검토 전용 제한 자연어 초안 parser, 작업·단계 이력과 연결한 mock 작업 시작·진행·취소 API/UI | 장치 없이 초안→사용자 검토→저장 및 mock 결과·오류 흐름을 구분; 기존 모의 실행 보존 |
| 5 | 단일 물체 grasp/motion 모의 흐름 | 실측에 맞춘 그리퍼/대상 collision geometry, tool orientation을 보존하는 접근·lift waypoint, 연속 경로/접촉 유지 검사, 이후 제한 대상 하나의 retreat·place 모의 단계 | 상자가 선반에서 유의미하게 분리되고 이동·안정화 뒤에도 jaw 지지가 유지됨을 simulator에서 확인하기 전에는 후속 place 성공을 주장하지 않음; 대상 없음·복수·stale·범위 밖·계획 실패가 후속 실행을 막고 이력에 남음 |
| 6 | 통합 수용·실물 준비도 판정 | 기존 관제 UI/API와 mock 흐름 통합, runbook 및 위험·미확정 목록 갱신, 실물 현장 수용 체크리스트 | mock 수용 기준 통과 여부가 확인됨; 하드웨어 단계는 별도 승인·운영 절차 이후로 분리 |

6주 종료 시점의 목표는 검증된 단일 물체 시나리오와 다음 실물 수용 단계의 준비 상태입니다. 미확정 장치 인터페이스나 안전 경계를 일정 때문에 추정해 실물 동작을 완료 조건으로 약속하지 않습니다.

말단 좌표 제어(카메라→팔 좌표, IK)는 2026-11-11에 계획에서 뺐습니다. 팔은 기록한 관절 자세(칸, home)와 물체별 학습 정책으로만 움직입니다.

## 주요 위험과 대응

| 위험 | 대응 |
| --- | --- |
| ROS workspace 파일에서 실행 중인 graph나 배포 환경의 실제 action/topic 전체를 알 수 없음 | 패키지·launch·bridge 정적 목록과 실행 시 관측을 구분해 기록합니다. 장치 인터페이스는 확인한 endpoint/type/feedback/result만 계약화합니다. |
| 카메라 장착·Depth 지원·시야가 미확정 | 카메라별 관측만으로 역할을 단정하지 않고, 물리 위치·RGB/Depth·동시 프레임 조건을 확인한 뒤 보정합니다. |
| RGB-only 또는 선반 평면 보정은 높이·가림·배치 변화에 취약 | 대상/선반 범위를 고정해 오차를 측정하고, 3D 요구가 확인되면 depth 또는 추가 센서 조건을 검토합니다. |
| 공식 MJCF와 실물 OMX 간 기구학/마찰/그리퍼 차이 | 모델을 검증 기준으로 취급하고 sim 한계를 문서화합니다. 실물 한계는 공식 사양·URDF·장치 확인값으로 별도 관리합니다. |
| LeRobot 관절 목표 쓰기는 작업 수락·취소·파지 성공 결과를 제공하지 않을 수 있음 | teleop 프로세스 관리와 자동 작업 adapter를 분리합니다. 확인된 terminal result 경로 전에는 성공 판정을 만들지 않습니다. |
| timeout/연결 단절 시 실제 동작 결과가 불명확 | `UNKNOWN`/`NEEDS_INTERVENTION`으로 남기고 자동 재전송·후속 팔 동작을 막습니다. |
| mock과 sim/hardware 상태가 UI에서 혼동됨 | 각 관측/명령/result에 실행 mode와 출처를 표시하고, mock을 실물 완료로 표현하지 않습니다. |

## Pick & Place 작업 현황과 다음 gate


| 작업 | 예정 문서/파일 | 의존성 | 확인 방법 | 완료 조건 |
| --- | --- | --- | --- | --- |
| 핑키 도착 → 정책 집기 → 적재 확인 → 핑키 완료 신호 | `src/control_station/task_orchestrator.py`, `src/control_station/load_check.py`, `docs/implementation/runbook.md` | 천장 카메라 주차 판정, 물체별 ACT 정책, 적재 확인 YOLO 모델, 핑키 팀과 합의한 완료 신호 토픽 | 모의 흐름 테스트, 실물은 사용자 승인 뒤 단계별 확인 | 적재 확인을 통과한 작업만 완료 신호를 한 번 보내고, 실패하면 신호 없이 멈춤 |

저장소 기록상 simulator-only ROS runtime prerequisite는 2026-10-31 임시 apt overlay에서 launch/controller activation, JointState·RGB-D·CameraInfo·TF read-only 관측, FollowJointTrajectory server discovery까지 확인했습니다. action goal이나 joint command는 보내지 않았으며 system prefix 설치나 장기 안정성은 확인하지 않았습니다. Fixed-camera task observation과 raw depth bbox 통계도 코드/문서에 연결됐지만 detector weights가 없어 실제 모델 inference는 미확인입니다. schema v3/v4 camera capture와 다섯 IK endpoint wrist-view 검토 및 virtual camera attachment sweep도 simulation-only 자료로 남아 있습니다. scene baseline/aimed camera는 다섯 endpoint에서 target depth pixel을 만들지 못했고, 가상 local camera position 60개 중 34개가 다섯 상태 모두에서 bbox edge margin 5 px 이상을 보였습니다. 이 결과는 physical camera 장착, 카메라/그리퍼/선반 충돌, 실제 접근 가시성을 입증하지 않습니다.


## 실제 장치 연동 전 확인 항목

- ROS 2 배포판/domain, 실제 graph의 action/topic/service 이름과 타입, namespace 및 frame
- Nav2 action 수락·feedback·terminal result와 lifecycle/상태 확인 경로
- OMX 모델/관절 제한 출처, 명령 권한자, 물리 정지 절차, READY·취소·파지 결과 제공자
- Head/Joint 물리 장착 매핑, RGB/Depth 지원, calibration frame/단위와 오차 허용 기준
- sim/hardware 실행 모드 분리 및 동시 제어 차단
- 승인된 제한 시험의 대상·작업영역·감독자·중단 조건

각 항목은 확인 방법과 근거를 함께 기록합니다. 미확정 값은 기본값이나 추측으로 채우지 않습니다.
