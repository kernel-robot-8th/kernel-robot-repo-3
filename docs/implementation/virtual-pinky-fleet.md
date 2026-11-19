# 가상 핑키 관제 (로봇 없이 주문 흐름 돌리기)

핑키 실물이 꺼져 있어도 주문 → 적재 자리 → 수령 구역 → 주차장 흐름을 끝까지 돌려 볼 수 있습니다.
가상 핑키 관제는 실제 핑키 관제(`pinky_fleet_dashboard/app.py`)와 같은 HTTP 주소로 답하고, 모든 응답에 `simulated: true`가 붙습니다.

## 실행

```bash
# 터미널 1: 가상 핑키 세 대 (8766번 포트, 실제 관제는 8765)
python3 simulation/virtual_pinky_fleet/virtual_pinky_fleet.py --time-scale 4

# 터미널 2: 관제를 가상 핑키에 붙여서 따로 띄우기 (실제 관제 8000번과 DB를 섞지 않음)
CONTROL_STATION_DB=/tmp/virtual-orders.sqlite3 CONTROL_STATION_PORT=8001 \
CONTROL_STATION_PINKY_FLEET_URL=http://127.0.0.1:8766 \
PYTHONPATH=src .venv/bin/python -m control_station.app
```

`http://127.0.0.1:8001`에서 주문을 넣으면 가상 핑키가 지도 위를 움직입니다.
`CONTROL_STATION_DB`를 주지 않으면 실제 관제와 같은 `data/tasks.sqlite3`를 쓰므로 꼭 따로 줍니다.

## 실물과 같은 것

- 지도, 웨이포인트, 주차 칸: `simulation/virtual_pinky_fleet/`의 사본 (2026-11-17에 실제 파일과 다시 맞춤)
- 임무 단계: `docking_manual` → `dock_complete` → `docking_anchor` → `docked_manual` → `received` → 주행 → `course_complete` → `mission-complete` → 복귀
- 가운데 벽은 통로 점(`navigation.json`의 `wall_passages`)으로 지나가고, 5번 앞에서는 정면 진입점에 먼저 섭니다.
- `dock_exit_reverse`가 false면 5번에서 후진 없이 나갑니다.
- 다시 맞추기(`redock`): 35 cm 물러났다가 정면 진입점에서 다시 들어옵니다. 세 번까지입니다.

## 가상에만 있는 것

| 주소 | 하는 일 |
|---|---|
| `GET /api/robots/<id>/parking-check` | 가상 천장 캠. 적재 자리 기준으로 `ARRIVED` / `MISALIGNED` / `NOT_IN_VIEW`와 오프셋을 답합니다. |
| `POST /api/virtual/dock-bias {robot?, forward_mm, left_mm, dyaw_deg}` | 핑키가 "자기가 있다고 믿는 자리"와 실제 자리의 차이를 넣습니다. `--dock-bias 30,-45,6`으로도 줄 수 있습니다. |

관제는 핑키 관제가 가상이고 주차 판정 노드가 없을 때 이 가상 천장 캠으로 도착을 판정합니다.

## 카메라 보정 다시 맞추기 (2026-11-17)

천장 캠이 "앞으로 30 mm, 오른쪽 45 mm, 왼쪽으로 6° 틀어짐"이라고 하면, 그 값을 `redock` 신호에 같이 보냅니다.

1. 관제(`order_flow`)가 판정의 `forward_mm`, `left_mm`, `dyaw_deg`를 `operator-confirm {action: redock, offset}`으로 보냅니다.
2. 핑키 관제가 핑키의 현재 위치와 그 오프셋으로 "진짜 적재 자리"를 계산합니다 (`route_geometry.corrected_dock`).
3. 핑키는 그 자리 기준의 정면 진입점으로 가서 다시 직진해 들어옵니다.

오프셋이 15 cm·30°보다 크거나, 고친 자리가 저장된 5번에서 12 cm·25° 넘게 벗어나면 쓰지 않고 예전처럼 같은 자리로 다시 들어옵니다.
핑키 화면의 [다시 맞추기] 버튼도 카메라 확정 판정을 받아 같이 보냅니다.

가상 확인: `--dock-bias 30,-45,6`으로 54 mm 어긋나게 세운 뒤 주문하면, 다시 맞추기 한 번에 약 6 mm로 들어옵니다.
오프셋 없이 다시 맞추면 같은 어긋남이 반복됩니다 (`tests/test_virtual_pinky_fleet.py`).
실물에서는 아직 확인하지 않았습니다.

## 가상으로 하지 않는 것

Nav2 경로 계획(점 사이는 직선), 장애물·충돌, 통로·적재 자리 예약, SSH, 배터리 소모.
