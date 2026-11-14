# 주문 흐름 (주문 → 선반 확인 → 팔 배정 → 핑키 도착 → 적재 → 확인 → 완료 신호)

2026-11-11에 좌표 기반 팔 제어를 계획에서 빼고, 관제가 주문 한 건을 끝까지 이끄는 흐름으로 바꿨습니다. 이 문서는 지금 코드가 하는 일과, 아직 모의인 부분을 정리합니다.

## 현장 구성 (2026-11-11 기준)

- 선반 두 개: 왼쪽 선반 `shelf_a`, 오른쪽 선반 `shelf_b`.
- 팔 두 대. 왼쪽 팔 `arm-01`은 왼쪽 선반, 오른쪽 팔 `arm-02`는 오른쪽 선반을 맡습니다. 설정 파일은 [config/arms.json](../../config/arms.json)입니다.
  - 팔은 물체 종류가 아니라 **물체가 놓인 선반**으로 정합니다.
- 핑키는 세 대(P0=10, P1=11, P2=12)입니다. 주문마다 한 대를 배정하고, 정차 위치는 두 팔 가운데 한 곳입니다.
- 카메라 역할:
  - **탑 캠(포트 4, HCAM0):** 핑키 도착 판정, 핑키 위 물건 세기.
    - C++ 주차 노드(`ceiling_parking.launch.py`)가 이 카메라를 열고, 관제는 그 ROS 토픽(`ros:/ceiling_camera/image_raw/compressed`)을 받습니다.
  - **선반 캠(포트 10):** 선반 재고 세기.
    - 두 선반, 두 팔, 가운데 핑키 쟁반까지 한 화면에 보입니다.
  - 팔 ACT 정책용 위 카메라는 따로 붙일 예정입니다.
  - 두 캠 모두 인웅님 YOLO `best19`(12클래스)를 씁니다.

## 단계

주문을 등록하면 단계 목록을 주문마다 만듭니다(`store.order_step_definitions`). 물체 한 개마다 `PICK_ITEM`과 `VERIFY_TRANSFER`가 한 쌍씩 들어갑니다. 실행은 [order_flow.py](../../src/control_station/order_flow.py)가 맡습니다.

| 단계 | 하는 일 | 멈추는 경우 |
|---|---|---|
| `VALIDATE_ORDER` | 정책·팔 설정 확인. 모의 핑키는 이때 출발 | `ORDER_POLICY_NOT_FOUND`, `ARMS_NOT_CONFIGURED` |
| `CHECK_SHELF_STOCK` | 선반별·종류별 재고가 주문 수량 이상인지 | `SHELF_STOCK_SHORT`, `COUNT_*` |
| `PLAN_ARM_SEQUENCE` | 물체마다 팔을 배정하고 팔별로 묶어 순서를 정함([배정 규칙](#주문-접수와-배정)). 결과와 물체별 배정 이유는 뒤 단계의 `robot_id`·`detail`과 `PLAN_CREATED` 이벤트 | `POLICY_NOT_FOR_ARM`, `SHELF_STOCK_SHORT` |
| `WAIT_PINKY_ARRIVAL` | 핑키가 5번에 정지(`docking_manual`)하고, 적재 자리가 비고, 주차 판정이 `ARRIVED`일 때 통과. 위치가 어긋나 있으면 경보만 띄우고 계속 기다리다가, 정상 위치가 되면 자동으로 다음 단계로 감 | `PINKY_ARRIVAL_TIMEOUT` (기본 600초) |
| `CONFIRM_DOCKING` | 핑키 관제에 `dock_complete` 전송 → `docking_anchor`(5번 지도 위치 재설정, 최대 18초)를 거쳐 `docked_manual`(정지 유지) 확인. 기본 30초 안에 안 바뀌면 실패 | `PINKY_SIGNAL_REJECTED`, `PINKY_SIGNAL_NOT_DELIVERED`, `PINKY_SIGNAL_NOT_APPLIED` |
| `PICK_ITEM` | 배정된 팔이 물체 하나를 집어 핑키 위에 놓음 (**지금은 모의**) | — |
| `VERIFY_TRANSFER` | 집기 전과 비교해 선반 −1, 핑키 +1인지 확인 | `PICK_FAILED_STILL_ON_SHELF`(선반에 그대로), `ITEM_LOST`(선반에도 핑키에도 없음), `TRANSFER_NOT_VERIFIED` |
| `VERIFY_LOAD_TOTAL` | 핑키 위 종류별 개수가 주문과 같고 다른 물건이 없는지 | `LOAD_INCOMPLETE`, `LOAD_EXTRA_ITEM`, `LOAD_MISMATCH` |
| `NOTIFY_PINKY_RECEIVED` | 핑키 관제에 `received` 전송. 응답 202는 핑키 관제가 받았다는 뜻이고, 핑키가 출발했다는 뜻은 아님 | `PINKY_SIGNAL_REJECTED` 등 |
| `FINALIZE_TASK` | 마무리 | — |

공통 규칙:
- 어떤 단계든 실패하면 주문을 그 자리에서 멈춥니다. 자동 재시도는 없고, 핑키에 `received`도 보내지 않습니다. 남은 단계는 `NOT_EXECUTED_AFTER_FAILURE`가 됩니다.
- 팔은 `WAIT_PINKY_ARRIVAL`을 통과하기 전에는 움직이지 않습니다.
- **적재 자리는 하나입니다.**
  - 정류장에 먼저 정지한 핑키의 주문이 자리를 씁니다. 다른 주문은 "적재 자리 대기"로 기다립니다.
  - 멈춘 주문의 핑키는 아직 정류장에 서 있을 수 있습니다. 그래서 운영자가 경보를 **확인**하거나 그 핑키가 떠날 때까지, 자리와 핑키 배정을 그대로 잡아 둡니다.

## 주문 접수와 배정

주문 창에서 들어온 주문은 핑키 없이 접수 상태(`PENDING`)로 기다리고, 작업자가 시작할 때 핑키를 정합니다. 배정 규칙은 `src/control_station/assignment.py`의 순수 함수라서 모의·실제 모드가 같습니다.

| 경로 | 하는 일 |
| --- | --- |
| `GET /api/catalog` | 주문할 수 있는 물체(정책이 있는 물체), 선반 재고 합계, 수령 구역 |
| `POST /api/orders` | `{items: [{object_class, quantity}], receiving_zone}` 접수. 핑키는 아직 없음 |
| `GET /api/orders` | 시작을 기다리는 주문, 오래된 순 |
| `GET /api/orders/{id}/assignment` | 지금 시작하면 맡을 핑키와 팔, 그 이유. 아무것도 바꾸지 않음 |
| `POST /api/orders/{id}/start` | 핑키를 다시 골라 확정(`PINKY_ASSIGNED` 이벤트)하고 주문 흐름 시작 |
| `POST /api/orders/{id}/reject` | 시작 전 주문 거절(`CANCELED`) |

- **핑키:** 진행 중 주문이 있거나, 핑키 관제가 답하지 않거나, 주차장 대기가 아니거나, 운행 준비가 안 됐거나, 배터리가 30% 미만이면 제외합니다. 남은 핑키는 배터리 10% 구간이 높은 순 → 오늘 운행 횟수가 적은 순 → 오래 쉰 순 → 번호 순으로 고릅니다. 남은 핑키가 없으면 `PINKY_UNAVAILABLE`이고 주문은 접수 상태로 남습니다.
- **팔:** 팔은 자기 선반에서, 정책이 있는 물체만 집습니다. 집을 수 있는 팔이 적은 물체부터 정하고, 두 팔이 다 되면 집기 성공률((성공+1)/(시도+2)) → 그 선반에 남은 수량 → 설정 순서로 고릅니다. 실행은 맡은 개수가 많은 팔부터 한 팔씩 합니다. 성공률은 지금 집기가 모의라서 모의 결과로만 셉니다.
- **수령 구역:** `config/receiving-zones.json`에 구역 이름과 핑키 경로의 웨이포인트 순서를 둡니다. 주문에는 `receiving_zone`으로 저장합니다. 핑키를 그 경로로 보내고 수령 확인을 받는 단계는 아직 없습니다.
- 이전 경로(`POST /api/tasks` + `order_items`)는 그대로 만들 때 핑키를 정합니다. 핑키 없이 접수된 주문을 `/api/tasks/{id}/start`로 시작하면 거절합니다.

## 무엇이 실제이고 무엇이 모의인가

| 부분 | 실제로 쓰는 조건 | 아니면 |
|---|---|---|
| 핑키 도착(주차 판정) | `CONTROL_STATION_PARKING_SPOT` 설정 + 주차 노드 실행 | 모의(`CONTROL_STATION_ORDER_PARKING=mock`이면 강제 모의) |
| 핑키 단계·신호 | `CONTROL_STATION_PINKY_FLEET_URL=http://<핑키 관제 PC>:8765` | 모의 세계 |
| 선반·핑키 위 개수 | `CONTROL_STATION_ORDER_COUNTS=camera` | 모의 세계 |
| 팔 집기 | 아직 없음 (다음 작업) | 모의 세계 |

모의 결과는 이벤트와 결과에 `simulated: true`로 남습니다. 화면에는 "모의" 표시가 붙습니다.

카메라로 개수를 셀 때, 팔 집기는 아직 모의라는 점을 주의해야 합니다. 집기 단계 동안 사람이 물체를 선반에서 핑키 위로 옮겨야 옮김 확인을 통과합니다. 집기 단계 시간은 `CONTROL_STATION_MOCK_PICK_S`(기본 4초)로 늘릴 수 있습니다.

## 핑키 관제 연결 (HTTP)

핑키 팀 관제(`origin/pinky_fleet`의 `pinky_fleet_dashboard/app.py`)에 HTTP로 붙습니다. 클라이언트는 [pinky_fleet.py](../../src/control_station/pinky_fleet.py)입니다.

ROS가 아니라 HTTP인 이유:
- 로봇 쪽에는 이 확인 신호를 받는 ROS 인터페이스가 없습니다. 핑키 관제가 SSH로 로봇에 파일을 써서 전달합니다.
- 핑키 관제가 단계마다 토큰을 새로 발급하기 때문에, 늦게 온 신호나 다른 핑키로 간 신호는 거절됩니다.

| 핑키 관제 | 관제의 뜻 |
|---|---|
| `GET /api/status`의 `robots[].route.phase` | `docking_manual`(5번 정지, 도킹완료 대기), `docking_anchor`(도킹완료 뒤 위치 재설정, 토큰 없음), `docked_manual`(정지 유지, 수령 대기), `dock_exit`(수령 뒤 20 cm 후진), `recovering`(장애물 우회), `driving` 등. 경로가 없거나 끝났으면 `parked` |
| `POST /api/robots/<id>/operator-confirm` `{"action": "dock_complete"\|"received", "token": <mission_token>}` | 202 받음, 409 `PINKY_SIGNAL_REJECTED`, 502 `PINKY_SIGNAL_NOT_DELIVERED`, 무응답 `PINKY_FLEET_UNREACHABLE` |

핑키 관제는 기본으로 그 PC의 `127.0.0.1:8765`에서만 열려 있습니다. 다른 PC의 관제가 붙으려면 핑키 팀이 접속을 열어 줘야 합니다.

## 카메라로 개수 세기

[camera_counts.py](../../src/control_station/camera_counts.py)가 셉니다. 공통으로 새 프레임 5장을 받아 종류별 중앙값을 씁니다.
- 프레임이 최신이 아니면(`COUNT_CAMERA_NOT_FRESH`) 0개로 치지 않고 "셀 수 없음"으로 멈춥니다.
- 검출기가 준비되지 않거나(`COUNT_DETECTOR_NOT_READY`) 칸 영역을 쓸 수 없을 때(`COUNT_SLOT_ZONES_NOT_READY`)도 마찬가지입니다.

**선반 재고**
- 팔마다 `stock_camera_id`의 검출 중 그 선반 칸 안(`IN_SLOT`)에 있는 것만 셉니다.
- 칸 영역은 [정면 카메라 선반 위치 보정](shelf-pose-calibration.md)에서 옵니다.

**핑키 위 물건**
- `CONTROL_STATION_LOAD_CAMERA`(기본 `head_camera`) 화면에서 쟁반 영역 안의 검출만 셉니다.
- 쟁반 영역 정하기:
  - `CONTROL_STATION_PINKY_TRAY_REGION=x1,y1,x2,y2`(픽셀)를 주면 그 영역을 씁니다.
  - 없으면 주차 위치 파일에 등록된 핑키 마커 둘레를 씁니다. 범위는 `CONTROL_STATION_PINKY_TRAY_MARGIN` × 마커 한 변이고, 기본 1.6입니다.
- `CONTROL_STATION_LOAD_IGNORE_CLASSES`(기본 `car`)는 세지 않습니다.

2026-11-11 실제 프레임 시험:
- 선반 캠 재고는 선반별로 셌습니다.
- 탑 캠 쟁반 영역 개수는 0개였습니다. 그보다 앞서 찍은 탑 캠 화면에서도 best19가 쟁반 위 물건을 잡지 못했습니다. 위에서 본 쟁반 사진을 더 학습해야 할 것으로 보입니다.

## 화면 (주문 진행 탭과 주문 창)

주문 진행 탭:

- **경보 띠**
  - 빨강: 멈춘 주문. **확인** 버튼을 누르면 자리와 핑키를 놓습니다.
  - 노랑: 핑키 위치 이상, 적재 자리 막힘, 핑키 관제 연결 오류.
- **주문 접수:** 주문 창에서 들어와 시작을 기다리는 주문. `GET /api/orders`를 2초마다 읽습니다.
  - 카드마다 접수 시각, 주문 번호, 물건, 수령 구역과 배정 미리보기(`/assignment`, 3초마다)를 보여 줍니다.
  - 미리보기의 핑키: 고를 핑키와 이유, 세 대의 순위 또는 제외 이유
  - 미리보기의 팔: 실행 순서대로 "팔 · 물체 #n · 고른 이유". 배정할 수 없으면 그 이유
  - **시작**은 `POST /api/orders/{id}/start`입니다. 시작하지 못하면(예: `PINKY_UNAVAILABLE`) 사유와 그때의 핑키별 판단을 카드에 남깁니다.
  - **거절**은 확인 창을 거쳐 `POST /api/orders/{id}/reject`를 보냅니다.
  - 새 주문이 들어오면 "주문 진행" 메뉴에 대기 건수 배지가 붙고, 화면 구석에 짧은 알림이 뜹니다. 기다리는 주문이 없으면 주문 창 링크를 보여 줍니다.
- **로봇 상태:**
  - 핑키 3대: 단계 메시지, 진행 중인 주문, 적재 자리 표시, 멈춘 주문 표시
  - 팔 2대: 대기, 또는 "사과 #1 집는 중"
- **주문 목록:** 시작한 주문만 보입니다. 진행 막대(주문 접수 → 선반 확인 → 팔 배정 → 핑키 도착 → 적재 n/N → 총량 확인 → 완료 신호), "지금" 한 줄, 팔 순서 칩, 멈추기·단계 보기 버튼.
- **모의 세계:** 선반 재고와 핑키 위 물건을 보여 주고, 사고를 넣을 수 있습니다.
  - 넣을 수 있는 사고: `STOCK_SHORT`, `PICK_FAILED`, `ITEM_LOST`, `EXTRA_ON_PINKY`, `PINKY_MISALIGNED`, `FLEET_REJECT`
  - 사고는 한 번 일어나고 사라집니다.
  - 화면 대신 `POST /api/mock-world`나 `CONTROL_STATION_MOCK_FAULTS`로도 넣을 수 있습니다.

주문 창(`/order`, 목업): 손님 쪽 화면이며 휴대폰 폭에서도 씁니다. 결제·로그인은 없고, 접수 뒤 진행 상황은 보여 주지 않습니다.

- `GET /api/catalog`를 5초마다 읽어 물건별 남은 수량을 보여 줍니다. 재고를 읽지 못하면 "재고 확인 불가", 0개면 품절입니다.
- 물건마다 1–5개를 고르고, 남은 수량을 알면 그보다 많이 고를 수 없습니다. 수령 구역이 하나면 고정으로 보여 줍니다.
- **주문하기**는 `POST /api/orders`입니다. 접수되면 주문 번호와 물건을 보여 주고, 접수되지 않으면 서버 메시지를 폼 아래에 보여 줍니다.

## 실행

```bash
set -a; source .control-station/control-station.env; set +a
# 터미널 A: 탑 캠 + 주차 판정 노드 (카메라는 이 노드만 엽니다)
source /opt/ros/jazzy/setup.bash; source cpp/install/setup.bash
ros2 launch parking_vision_ros ceiling_parking.launch.py repository_root:=$PWD spot_id:=pinky_dock_1
# 터미널 B: 관제
./run-control-station.sh
```

모의 시간 조절:
- `CONTROL_STATION_MOCK_PINKY_DRIVE_S`: 모의 핑키 이동 시간, 기본 8초
- `CONTROL_STATION_MOCK_PICK_S`: 모의 집기 시간, 기본 4초
- `CONTROL_STATION_ORDER_STEP_DWELL_S`: 빠른 단계를 화면에 잠깐 보여 주는 시간, 기본 0.8초
- `CONTROL_STATION_MOCK_SHELF_STOCK`: 모의 선반 재고. 예: `shelf_a:carrot=3;shelf_b:apple=2`

카메라를 다시 꽂거나 옮겨 주차 판정이 `VIEW_UNVERIFIED`(고정 마커가 등록 때보다 15 px 넘게 이동)가 되면, 핑키를 정차 위치에 세우고 다시 등록합니다. 절차는 [실행 방법](runbook.md#정답-위치-등록)에 있습니다.
- 2026-11-11에 이 절차로 `pinky_dock_1`을 다시 등록했습니다(마커 21, 70 mm, 고정 마커 1 신뢰, 25 mm / 5°).
- 지금 등록은 마커 21번 핑키 한 대 기준입니다. 핑키 세 대를 구분하려면 핑키별로 마커를 등록해야 합니다.

## 남은 일

- 팔 집기를 실제 `PICK_WITH_POLICY` 명령으로 바꾸기. 팔별 정책(`arm_id`)을 고르고, 잡았는지는 `VERIFY_TRANSFER`가 카메라로 판단합니다.
- 탑 캠에서 핑키 쟁반 위 물건 검출 개선(YOLO 학습).
- 핑키 관제 외부 접속, 핑키별 주차 마커 등록.
