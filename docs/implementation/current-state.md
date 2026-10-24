# 현재 구현 현황

이 문서는 현재 코드가 실제로 제공하는 기능을 기록합니다. 목표 요구사항은 [시스템 명세](../spec/system-spec.md)를 참고하세요.

## 코드 구성

| 파일 | 현재 책임 |
|---|---|
| `src/control_station/app.py` | localhost HTTP 서버, API 라우팅 |
| `src/control_station/robot_adapters.py` | 모의 상태와 Nav2 action server discovery |
| `src/control_station/arm_supervisor.py` | 고정 LeRobot 프로세스 시작·종료·상태 |
| `src/control_station/camera_adapters.py` | `--arm` 모드에서 OpenCV 카메라 단독 점유 및 MJPEG 프레임 제공 |
| `src/control_station/models.py` | 작업·단계 상태 어휘 및 카메라 스냅샷 |
| `src/control_station/store.py` | SQLite 작업 생성·조회 |
| `src/control_station/web/` | 간단한 상태 화면과 텔레오퍼레이션 제어 |

## 구현된 API

| 메서드 | 경로 | 동작 |
|---|---|---|
| GET | `/api/health` | 서버 모드 반환 |
| GET | `/api/robots` | mock 상태 또는 ROS 2 Nav2 discovery 반환 |
| GET | `/api/cameras` | Head·Joint 카메라 상태 및 설정된 스트림 주소 반환 |
| GET | `/api/arm/status` | 팔 텔레오퍼레이션 프로세스 상태 반환 |
| POST | `/api/arm/start` | 고정 프로필 프로세스 시작 요청 |
| POST | `/api/arm/stop` | 프로세스 그룹에 `SIGINT` 종료 요청 |
| GET | `/api/tasks` | SQLite 작업 목록 반환 |
| POST | `/api/tasks` | `PENDING` 작업 저장 |

## 현재 동작의 경계

- 서버는 `127.0.0.1`에 바인딩합니다. 기본 실행은 모의 모드이며 로봇에 접속하지 않습니다.
- ROS 2 모드는 Nav2 `NavigateToPose` 액션 서버를 찾는 것까지만 합니다. 주행 목표를 보내지 않습니다.
- 팔 제어는 기본 비활성화입니다. 활성화 시 LeRobot CLI 프로세스의 시작·생존·종료를 관리합니다.
- LeRobot CLI의 프로세스 생존은 장치 준비 완료나 작업 완료 신호가 아닙니다. 별도의 작업 완료 이벤트는 아직 없습니다.
- 텔레오퍼레이션 시작은 저장된 관제 작업을 실행하지 않습니다. `/api/tasks`는 계속 `PENDING`만 저장합니다.
- `--arm` 모드에서는 관제 서버가 기본 `/dev/video4`를 단독으로 열어 Joint Camera 슬롯에 MJPEG로 보냅니다. LeRobot에는 카메라 설정을 전달하지 않고 Rerun 표시도 끕니다.
- `/dev/video4`는 Joint Camera, HCAM0의 by-id 경로는 Head Camera로 설정했습니다. 두 경로 모두 영상 캡처 장치이며 `/dev/video5`는 메타데이터 노드입니다.
- Head Camera의 HCAM0 경로에서 프레임 수신을 확인했습니다. 두 카메라 동시 표시와 실제 장착 위치의 최종 육안 확인은 남아 있습니다.
- 작업 상태·단계 상태 이름은 모델에 정의되어 있으나 실제 오케스트레이션과 전이는 구현되지 않았습니다.

## 다음 작업

우선순위와 완료 조건은 [개발 TODO](../development-plan.md)에서 관리합니다.
