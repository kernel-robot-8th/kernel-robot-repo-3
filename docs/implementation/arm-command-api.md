# OMX 명령 HTTP 인터페이스

관제가 OMX 명령(칸 자세, 원상복귀, 학습 정책)을 넘기고, OMX 쪽 워커가 그것을 가져가 실행한 뒤 결과를 알려 주는 HTTP 계약입니다. 관제와 OMX가 한 노트북에 있든 서버와 노트북으로 나뉘든 같은 API를 씁니다. 관제는 팔을 직접 움직이지 않습니다. 팔을 움직이는 것은 OMX 쪽 워커(LeRobot)입니다.

현재 구현 범위: 명령 큐·상태 기록 API, 관제 화면의 **OMX 명령** 섹션(수동 조작 탭), 팔을 움직이지 않는 [모의 워커](../../scripts/mock_omx_worker.py). 작업 흐름(작업 이력의 단계)과는 아직 연결하지 않았고, 목표는 운영자가 직접 고릅니다(`source.kind: OPERATOR_INPUT`). 좌표(XYZ) 목표는 쓰지 않습니다(2026-11-11 결정). 실물 워커와 작업 흐름 연결은 아래 절에 있습니다.

## 왜 HTTP인가

- OMX는 LeRobot으로 제어하고, LeRobot은 ROS 2 노드가 아닙니다. 관제는 이미 HTTP 서버이고 작업·명령 기록 DB를 갖고 있습니다.
- 워커가 관제에 요청하는 방식(pull)이라 관제는 OMX PC 주소를 몰라도 됩니다. 워커 쪽은 Python 표준 라이브러리만 있으면 됩니다.
- ROS 2 action은 처음에는 OMX PC의 ROS 2 설치와 LeRobot 가상환경의 `rclpy` 사용 가능 여부를 몰라 보류했습니다. 2026-11-04 이 PC에서 둘 다 확인했고, 칸 이동만 먼저 action으로 옮기는 중입니다(아래 [ROS 2 action 워커](#ros-2-action-워커-omxmove_to_slot)). 관제 쪽 action 클라이언트는 아직 없으므로, 관제 화면에서 보내는 명령은 지금도 이 HTTP 계약을 씁니다.

## 전체 흐름

```mermaid
sequenceDiagram
    participant Op as 운영자/관제
    participant API as 관제 API
    participant W as OMX 워커 (LeRobot)
    W->>API: GET /api/arm/arm-01/commands/next (long-poll, 최대 30초 대기)
    Op->>API: POST /api/arm/commands (명령 1건)
    API-->>W: 200 명령 (상태 DISPATCHED)
    W->>W: 칸 정의 버전·유효 시간·기록 자세 확인
    alt 실행 불가
        W->>API: REJECTED + reason_code
    else 실행 가능
        W->>API: ACCEPTED
        API-->>W: 200 이면 실행, 그 외(409)면 실행 금지
        loop 단계마다
            W->>API: PROGRESS (phase) — 응답의 cancel_requested 확인
        end
        W->>API: SUCCEEDED / FAILED / RESULT_UNKNOWN / CANCELED
    end
```

**핵심 규칙: 워커는 `ACCEPTED`에 HTTP 200을 받은 뒤에만 팔을 움직입니다.** 409나 연결 오류면 그 명령은 실행하지 않습니다. 이 규칙 덕분에 관제는 "ACCEPTED가 안 왔다 = 팔이 안 움직였다"로 판단할 근거가 생깁니다. 그래도 워커 버그 가능성 때문에 그 경우는 `UNKNOWN`으로 두고 사람이 확인합니다.

## 명령 (관제 → 워커)

워커가 `GET /api/arm/{robot_id}/commands/next`로 받는 `command` 객체입니다. 대상은 명령을 만든 시점에 고정되며, 실행 중에 바뀌지 않습니다. 명령 종류는 `MOVE_TO_SLOT`(칸 자세 재생), `RETURN_HOME`(home 자세), `PICK_WITH_POLICY`(물체별 학습 정책) 세 가지입니다. 좌표(XYZ) 명령은 2026-11-11에 없앴습니다.

```json
{
  "schema_version": 1,
  "command_id": "CMD-E7792665603A",
  "robot_id": "arm-01",
  "operation": "MOVE_TO_SLOT",
  "state": "DISPATCHED",
  "created_at": "2026-11-02T13:14:44.120000+00:00",
  "valid_until": "2026-11-02T13:15:44.120000+00:00",
  "worker_id": "omx-laptop",
  "cancel_requested": false,
  "target": {
    "object_class": "strawberry",
    "pick_slot_id": "slot_a_2_1",
    "slot_label": "왼쪽 선반 아래 단 좌",
    "slot_map_version": "<config/shelf-slots.json의 SHA-256>"
  },
  "place": {"slot_id": "CART_1"},
  "source": {"kind": "OPERATOR_INPUT", "note": null}
}
```

그 밖에 상태 확인용 필드(`dispatched_at`, `accepted_at`, `finished_at`, `last_event_at`, `reason_code`, `result` 등)가 함께 옵니다.

| 필드 | 의미 | 워커가 할 일 |
|---|---|---|
| `target.pick_slot_id` | 선반 칸([칸 정의](#선반-칸-정의)의 사용 칸 6개 중 하나). `PICK_WITH_POLICY`에서는 정면 카메라가 물체를 본 칸(근거)이며 없을 수 있습니다 | 그 칸에 기록해 둔 관절 자세를 재생. 모르는 칸이면 `REJECTED` / `SLOT_UNKNOWN` |
| `target.slot_label`, `slot_map_version` | 칸 이름(예: "왼쪽 선반 아래 단 좌")과 명령을 만들 때 쓴 칸 정의 파일의 SHA-256 | 자기 칸 정의와 버전이 다르면 `REJECTED` / `SLOT_MAP_MISMATCH` |
| `target.policy` | `PICK_WITH_POLICY`만: 정책 ID·모델·정책 정의 파일 버전 | 자기 정책 정의와 다르면 `REJECTED` / `POLICY_REGISTRY_MISMATCH` |
| `valid_until` | 이 시각 이후에는 실행하지 않음 | 지났으면 `REJECTED` / `EXPIRED` |
| `cancel_requested` | 운영자가 취소를 요청함 | 단계 사이에 확인하고 멈춘 뒤 `CANCELED` |

## 이벤트 (워커 → 관제)

`POST /api/arm/commands/{command_id}/events`

```json
{
  "event_id": "ACCEPTED-1f3a9c20",
  "robot_id": "arm-01",
  "worker_id": "omx-laptop",
  "event": "ACCEPTED",
  "simulated": false,
  "occurred_at": "2026-11-02T13:14:45.010+00:00",
  "phase": null,
  "reason_code": null,
  "message": null,
  "grasp_confirmed": null,
  "place_confirmed": null,
  "details": {}
}
```

- 필수: `event_id`, `robot_id`, `worker_id`, `event`, `simulated`. 실물 워커는 `simulated: false`를 명시합니다.
- `event_id`는 워커가 정합니다. 네트워크 오류로 같은 이벤트를 다시 보낼 때는 **같은 `event_id`**를 쓰면 한 번만 반영되고 `applied: false`로 응답합니다. 같은 `event_id`로 다른 이벤트를 보내면 409입니다.
- `worker_id`는 명령을 가져간 워커와 같아야 합니다.
- 응답 `200 {"command": ..., "applied": true}`의 `command.cancel_requested`로 취소 요청을 확인할 수 있습니다.

| event | 허용되는 현재 상태 → 다음 상태 | 조건 |
|---|---|---|
| `ACCEPTED` | `DISPATCHED` → `ACCEPTED` | 취소 요청됐거나 `valid_until`이 지났으면 409, 명령은 `CANCELED`/`EXPIRED`로 닫힘 |
| `REJECTED` | `DISPATCHED` → `REJECTED` | `reason_code` 필수 |
| `PROGRESS` | `ACCEPTED`/`RUNNING` → `RUNNING` | `phase` 필수 |
| `SUCCEEDED` | `ACCEPTED`/`RUNNING` → `SUCCEEDED` | `grasp_confirmed`와 `place_confirmed`가 모두 `true`일 때만 (SR-14) |
| `FAILED` | `ACCEPTED`/`RUNNING` → `FAILED` | `reason_code` 필수 |
| `RESULT_UNKNOWN` | `ACCEPTED`/`RUNNING` → `UNKNOWN` | 동작은 끝났지만 파지·적재를 확인할 수 없을 때. `reason_code` 필수 |
| `CANCELED` | `ACCEPTED`/`RUNNING` → `CANCELED` | 운영자 취소 요청이 있을 때만. 스스로 멈췄으면 `FAILED` |

`phase` 값은 관제의 집기 단계 이름과 같습니다: `APPROACH_TARGET`, `REFINE_TARGET`, `GRASP_TARGET`, `VERIFY_GRASP`, `LIFT_TARGET`, `MOVE_TO_DESTINATION`, `PLACE_TARGET`, `VERIFY_PLACEMENT`. `reason_code`는 대문자·숫자·`_`로 된 1–60자이며, 값 목록은 OMX 팀과 정합니다.

## 상태와 관제 규칙

| 상태 | 뜻 | 새 명령 |
|---|---|---|
| `QUEUED` | 워커가 아직 가져가지 않음. `valid_until`이 지나면 `EXPIRED` | 막힘 |
| `DISPATCHED` | 워커가 가져감. `CONTROL_STATION_ARM_ACK_TIMEOUT_S`(기본 10초) 안에 `ACCEPTED`/`REJECTED`가 없으면 `UNKNOWN` (`ACK_TIMEOUT`) | 막힘 |
| `ACCEPTED`, `RUNNING` | 실행 중. 결과 제한 시간은 아직 없으며 화면에 마지막 이벤트 시각을 표시합니다 | 막힘 |
| `SUCCEEDED`, `FAILED`, `REJECTED`, `CANCELED`, `EXPIRED` | 종료 | 가능 |
| `UNKNOWN` | 결과를 알 수 없음. 자동 재전송하지 않습니다 | 운영자가 현장 확인 내용을 기록할 때까지 막힘 |

- 팔 하나에 끝나지 않은 명령은 1건만 둡니다. 두 번째 명령은 409 `ARM_COMMAND_ACTIVE`입니다.
- `UNKNOWN`은 운영자가 화면의 **현장 확인 완료**(`POST .../clear-unknown`, 확인 내용 필수)를 눌러야 다음 명령을 보낼 수 있습니다. 결과는 계속 `UNKNOWN`으로 남고 성공·실패로 바꾸지 않습니다.
- 마감 시각 검사는 API를 호출할 때 함께 수행합니다. 화면이 1초마다 목록을 다시 불러오므로 표시는 거의 바로 바뀝니다.
- 워커의 long-poll 요청이 접속 신호 역할을 합니다. 화면 제목 옆에 "워커 대기 중" 또는 "마지막 접속 n초 전"을 표시합니다.

## API 목록

| Method | Path | 호출자 | 내용 |
|---|---|---|---|
| POST | `/api/arm/commands` | 관제 화면 | 명령 등록. body: `robot_id`, `operation`, `valid_for_s`(5–600), `target`, `place`, `source` |
| GET | `/api/arm/commands` | 관제 화면 | 최근 20건, 워커 접속 상태, ACK 제한 시간 |
| GET | `/api/arm/shelf-slots` | 둘 다 | 관제가 불러온 칸 정의 파일 내용과 버전(SHA-256). 파일이 없으면 404 `SHELF_SLOTS_NOT_CONFIGURED` |
| GET | `/api/arm/policies` | 관제 화면 | 물체별 학습 정책 목록과 정책 파일 버전(SHA-256). 파일이 없으면 404 `ACT_POLICIES_NOT_CONFIGURED` |
| POST | `/api/arm/commands/from-camera` | 관제 화면 | body `{"object_class": "당근", "operation": "PICK_WITH_POLICY"}`(operation 생략 시 `MOVE_TO_SLOT`). 정면 카메라에서 한 칸에만 보일 때 명령 등록. 정책 없는 물체는 409 `POLICY_NOT_FOUND` |
| GET | `/api/arm/commands/{command_id}` | 둘 다 | 명령과 전체 이벤트 이력 |
| POST | `/api/arm/commands/{command_id}/cancel` | 관제 화면 | body `{}`. `QUEUED`면 바로 취소, 실행 중이면 취소 요청 표시 |
| POST | `/api/arm/commands/{command_id}/clear-unknown` | 관제 화면 | body `{"note": "..."}`. `UNKNOWN` 현장 확인 기록 |
| GET | `/api/arm/{robot_id}/commands/next?worker_id=...&wait_s=20` | 워커 | long-poll. 200 명령 / 204 없음. `wait_s` 0–30 |
| POST | `/api/arm/commands/{command_id}/events` | 워커 | 이벤트 보고 |

오류 응답은 `error_code`, `message`를 담습니다. 400은 형식 오류, 404는 없는 명령·팔, 409는 현재 상태에서 허용되지 않는 요청이며 409에는 현재 `command`가 함께 옵니다. 브라우저의 다른 출처(Origin) 요청은 403으로 막습니다.

## 선반 칸 정의

[config/shelf-slots.json](../../config/shelf-slots.json)은 팀 3D 모델(`shelf_a.glb`, `shelf_b.glb`)에서 [derive_shelf_slots.py](../../scripts/derive_shelf_slots.py)로 만든 칸 목록입니다. 모델에 들어 있는 칸 표시(`slot_<선반>_<단>_<번호>`, 각 단 바닥판 윗면의 칸 중앙)와 메시의 바닥판 면을 읽어, 두 선반을 하나의 선반 기준 좌표(`shelf_front`)로 표현합니다.

- 원점은 두 선반이 맞닿는 선의 밑면, 선반 정면 평면 위입니다. x는 선반 안쪽, y는 정면에서 볼 때 왼쪽, z는 위입니다. 높이는 선반 밑면 기준이며 책상 높이는 들어 있지 않습니다.
- 칸마다 칸 표시 위치(`marker_m`), 바닥판 영역(`footprint_m`), 바닥판 높이, 위 단까지 높이(`clear_height_m`, 위가 열려 있으면 `null`)를 담습니다.
- 사용하는 칸(`active: true`)은 6개로 정했습니다(2026-11-03): 왼쪽 선반 위 단 좌·우, 아래 단 좌·우, 오른쪽 선반 아래 단 좌·우. 오른쪽 선반 위 단(`slot_b_1_*`)은 모델에는 있지만 쓰지 않습니다.

| 칸 | `slot_id` | x | y | z (바닥판) | 물건 최대 높이 |
|---|---|---|---|---|---|
| 왼쪽 위 단 좌 | `slot_a_1_1` | 7.0 | 30.5 | 25.1 | 열림 |
| 왼쪽 위 단 우 | `slot_a_1_2` | 7.0 | 10.5 | 25.1 | 열림 |
| 왼쪽 아래 단 좌 | `slot_a_2_1` | 4.5 | 30.5 | 8.3 | 16.6 |
| 왼쪽 아래 단 우 | `slot_a_2_2` | 4.5 | 10.5 | 8.3 | 16.6 |
| 오른쪽 아래 단 좌 | `slot_b_2_1` | 4.5 | −10.5 | 8.3 | 16.6 |
| 오른쪽 아래 단 우 | `slot_b_2_2` | 4.5 | −30.5 | 8.3 | 16.6 |

단위는 cm이며 파일에는 m로 저장됩니다. 2026-11-03 자로 잰 실물은 폭 41 cm, 위 단 턱 위 27 cm로 모델과 같았지만, 아래 단은 모델보다 3.5 cm 낮았습니다(선반 밑면부터 아래 단 턱 위 약 10 cm, 모델 13.5 cm). 그래서 아래 단과 그 앞 턱에만 −3.5 cm 보정을 넣었고, 파일 `status`는 `MODEL_DERIVED_WITH_MEASURED_ADJUSTMENTS`, 보정 내역은 `measured_adjustments`에 남깁니다. 아래 단 바닥판은 8.3 cm, 아래 칸 높이는 16.6 cm가 됩니다. 정면 카메라 위치 보정에서 이 값으로 재투영 오차 1.3 px를 얻었고, 정면 카메라에서 `shelf_a`가 왼쪽인 것도 확인했습니다([선반 위치 보정](shelf-pose-calibration.md)). 깊이와 칸 좌우 위치는 실측하지 않은 모델 값입니다.

모델을 다시 받으면 이렇게 새로 만듭니다. 출력 파일이 있으면 `--force` 없이는 덮어쓰지 않습니다.

```bash
.venv/bin/python scripts/derive_shelf_slots.py shelf_a.glb shelf_b.glb \
  --inactive slot_b_1_1,slot_b_1_2 --force \
  --lower-deck-shift -0.035 --adjustment-note "2026-11-03 자 실측: …"
```

파일 내용이 바뀌면 버전(SHA-256)도 바뀝니다. 관제는 시작할 때 한 번 읽으므로(`CONTROL_STATION_SHELF_SLOTS`로 경로 변경 가능) 다시 시작해야 하고, OMX 워커의 칸별 자세도 새 버전 기준으로 다시 확인해야 합니다. 그 전까지 워커는 버전이 다른 명령을 `SLOT_MAP_MISMATCH`로 거부합니다. 파일을 읽지 못하면 관제는 칸 명령을 거부합니다(원상복귀와 학습 정책 명령은 받습니다).

## 실행해 보기

```bash
# 터미널 1: 관제 (모의 모드)
./run-control-station.sh

# 터미널 2: 모의 워커 (팔을 움직이지 않음)
.venv/bin/python scripts/mock_omx_worker.py --server http://127.0.0.1:8000
```

관제 화면 **수동 조작 → OMX 명령**에서 칸을 고르고 **칸 앞으로 이동**을 누르면, 모의 워커가 받아 단계별 진행을 보고하고 "성공 · 모의"로 끝납니다. 다른 경로는 `--result failed|unknown|reject|silent`로 확인합니다. `silent`는 명령만 가져가고 `ACCEPTED`를 보내지 않으므로 10초 뒤 `UNKNOWN`이 됩니다.

**OMX가 다른 PC에 있을 때:** 관제는 기본적으로 `127.0.0.1`에만 열립니다. 같은 네트워크의 노트북에서 접속하려면 `CONTROL_STATION_HOST=0.0.0.0`으로 실행하고, 워커는 `--server http://<관제 PC IP>:8000`을 씁니다. **인증이 없으므로** 믿을 수 있는 로컬 네트워크에서만 사용합니다. 관제 화면을 볼 수 있는 사람은 누구나 명령을 보낼 수 있습니다.

## 실물 OMX: 칸 앞으로 이동 (`MOVE_TO_SLOT`)

관제 쪽 범위는 **기록해 둔 칸 앞 자세로 팔을 데려가기**까지입니다. 집어서 넣는 동작은 LeRobot으로 학습한 정책이 이어받습니다(다른 파트). 그래서 워커는 명령을 처리할 때만 팔로워 포트를 열고, 도착하면 토크를 켠 채 포트를 놓습니다. 정책 프로그램은 그 자세에서 바로 같은 포트로 연결할 수 있습니다. 명령 종류 `MOVE_TO_SLOT`은 `target.pick_slot_id`가 필수이고 `object_class`는 비워도 됩니다. `SUCCEEDED`에는 측정한 관절값이 목표에 도착했다는 `pose_reached: true`가 있어야 합니다(`PICK_WITH_POLICY`의 `SUCCEEDED`는 `grasp_confirmed`·`place_confirmed`가 필요).

환경 (2026-11-03, 이 PC): LeRobot 0.4.4 + Dynamixel SDK 3.8.4가 `~/lerobot-venv`에 있고, 팔로워는 OpenRB-150 `F58AE686…`(모터 ID 11–16), 리더는 `24D48EBF…`(ID 1–6)입니다. 포트는 꽂는 순서에 따라 `ttyACM` 번호가 바뀌므로 `/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F58AE686…-if00`을 씁니다. 계정이 `dialout` 그룹이어야 하며, 다시 로그인하기 전에는 `sg dialout -c "…"`로 실행합니다. 두 팔 모두 LeRobot OMX 공장 보정값(원점 보정 0, 위치 한계 0–4095)을 갖고 있음을 읽기 전용으로 확인했습니다.

1. **자세 기록** ([record_omx_pose.py](../../scripts/record_omx_pose.py), 읽기만 함): 팔로워 토크가 꺼진 상태에서 손으로 팔을 칸 앞 자세로 옮긴 뒤 실행합니다. `config/omx-slot-poses.json`에 다섯 관절과 그리퍼의 Dynamixel 위치값(1회전 4096)을 저장하고, 팔로워 보드 일련번호와 칸 정의 버전을 함께 남깁니다. 같은 이름을 다시 기록하려면 `--replace`.

   ```bash
   sg dialout -c "~/lerobot-venv/bin/python scripts/record_omx_pose.py slot_a_2_1"
   ```

2. **워커 실행** ([omx_lerobot_worker.py](../../scripts/omx_lerobot_worker.py)): `sg dialout -c "~/lerobot-venv/bin/python scripts/omx_lerobot_worker.py --server http://127.0.0.1:8000"`. 모터에 쓰지 않고 흐름만 보려면 `--dry-run`(실물 모터를 읽기만 함, 결과는 `RESULT_UNKNOWN`/`DRY_RUN_NO_MOTION`), 팔이 연결돼 있지 않으면 `--fake-follower`(이 프로세스 안의 가짜 모터가 프로파일 속도로 움직임, 모든 이벤트 `simulated: true`). 이동 로직과 안전장치는 ROS 2 워커와 함께 [omx_slot_motion.py](../../scripts/omx_slot_motion.py)에 있습니다.
3. **관제에서 명령**: **수동 조작 → OMX 명령**에서 칸을 고르고 **칸 앞으로 이동**.
4. **원상복귀** (`RETURN_HOME`): 같은 섹션의 **원상복귀** 버튼. 명령에는 칸·놓을 곳이 없고(`target` 필드 모두 `null`, `place: null`), 워커가 자기 자세 파일의 `home`으로 갑니다. 안전장치와 성공 조건(`pose_reached: true`)은 칸 이동과 같습니다. `home`이 기록되지 않았으면 `POSE_NOT_RECORDED`로 거부합니다. API: `POST /api/arm/commands` body `{"robot_id": "arm-01", "operation": "RETURN_HOME", "valid_for_s": 60, "source": {"kind": "OPERATOR_INPUT"}}`. 2026-11-04 확인: 임시 관제에서 모의 워커와 `--fake-follower` 워커로 `SUCCEEDED`를 확인했습니다. 같은 날 관제 화면에서 실물 팔 원상복귀(`CMD-35E067AE79B9`, `--profile-velocity 5 --max-delta 1500`, 어깨 들기 1289틱 이동)가 `SUCCEEDED`였고 도착 오차는 최대 13틱이었습니다.

워커의 동작과 안전장치:
- LeRobot의 `OMXFollower` 대신 LeRobot Dynamixel 버스를 직접 씁니다. `OMXFollower.connect()`는 모터 설정을 다시 쓰고, 목표 위치를 맞추지 않은 채 토크를 켜기 때문입니다.
- `ACCEPTED`에 200을 받은 뒤에만 움직입니다. 토크를 켜기 전에 목표 위치를 현재 위치로 맞추고, 모터 자체의 속도 프로파일(`Profile_Velocity` 기본 10 ≈ 14°/s, `Profile_Acceleration` 기본 2 ≈ 43°/s², 약 0.3초에 걸쳐 가속·감속)로 천천히 움직입니다. 처음 쓴 15/10은 실물에서 출발·정지가 너무 급했습니다. 그리퍼는 명령하지 않습니다.
- 어느 관절이든 변화량이 `--max-delta`(기본 1200 = 약 105°)를 넘으면 `TARGET_TOO_FAR`로 거부합니다. 자세가 기록되지 않았으면 `POSE_NOT_RECORDED`, 칸 정의 버전이 다르면 `SLOT_MAP_MISMATCH`입니다.
- 다섯 관절이 목표에서 `--tolerance`(기본 15틱 ≈ 1.3°) 안에 들어오고 `Moving`이 0이면 `SUCCEEDED`, `--move-timeout`(30초)을 넘기면 그 자리에 멈추고 `FAILED`/`MOVE_TIMEOUT`, 통신 오류면 멈추고 `RESULT_UNKNOWN`입니다. 관제에서 취소하면 1초 안에 그 자리에 멈추고 `CANCELED`입니다.
- 관절 공간에서 바로 이동하므로 경로 중간의 선반 충돌을 검사하지 않습니다. 시작 자세와 목표 자세 사이에 선반이 없게 기록하고, 현장에서 전원을 끊을 수 있게 지켜봅니다.
- 이동 후와 워커 종료 후에도 토크를 켜 둡니다(팔이 떨어지지 않게). 손으로 다시 옮기려면 팔을 받친 채 `--release-torque`로 토크를 끕니다.
- 다른 프로그램(학습 정책, 텔레오퍼레이션)이 포트를 쓰고 있으면 모터에 아무것도 보내지 않고 `REJECTED`/`FOLLOWER_PORT_UNAVAILABLE`로 거부합니다.
- 2026-11-03 확인: `--dry-run`으로 관제 명령 → 검사 → 수락 → 쓰기 순서(목표=현재 → 속도 → 토크 → 목표)와, 명령 전후에 포트를 놓는 것까지 확인했습니다. 같은 날 현장 감독 하에 관제에서 `MOVE_TO_SLOT`(`slot_a_2_1`)을 보내 실물 팔로워가 쉬는 자세에서 기록 자세까지 움직였습니다. 가장 큰 이동은 어깨 들기 978틱(약 86°)이었고, 도착 오차는 최대 10틱(약 0.9°)으로 `SUCCEEDED`였습니다(`CMD-804B25BCB2B0`). 기록하지 않은 칸(`slot_a_2_2`) 명령은 움직이지 않고 `POSE_NOT_RECORDED`로 거부됐습니다. 경로 중 충돌 검사는 없으며, 나머지 칸 자세는 아직 기록하지 않았습니다.

## 학습 정책으로 집기 (`PICK_WITH_POLICY`)

운영자가 관제 화면 **수동 조작** 탭의 물체별 버튼(**당근 집기**, **물티슈 집기**. 정책 파일에서 자동으로 만듦)을 누르면, 그 물체만 집도록 학습한 LeRobot ACT 정책을 OMX 워커가 실행합니다. 정면 카메라에서 물체를 확인하지 못하면(카메라 미설정, 미검출, 여러 칸 등) 이유를 보여 주고, 운영자가 확인하면 칸 근거 없이(`source.note`에 이유 기록) 명령을 만듭니다. **ACT 정책은 위치를 입력으로 받지 않습니다.** 카메라 영상과 관절 값만 보고 학습한 물체를 스스로 찾습니다. 그래서 관제는 물체 이름으로 **정책을 고르고**, 정면 카메라가 본 칸은 물체가 있다는 근거로만 명령에 남깁니다.

```mermaid
sequenceDiagram
    participant Op as 운영자 (관제 화면)
    participant API as 관제 API
    participant W as OMX 워커
    participant P as 정책 서버 (LeRobot)
    Op->>API: from-camera {object_class: "당근", operation: PICK_WITH_POLICY}
    API->>API: 당근 → carrot → 정책 carrot_act_v2, 정면 카메라에서 carrot이 한 칸에만 보이는지
    API-->>Op: 201 명령 (칸은 근거)
    W->>API: 명령 가져감 → 검사 → ACCEPTED (200일 때만 진행)
    W->>P: RobotClient로 정책 실행 (카메라 2대 + 관절)
    W->>API: PROGRESS APPROACH_TARGET, 이후 1초마다 취소 확인
    W->>W: 팔이 home을 벗어났다가 돌아와 1.5초 머물면 정책 정지 (토크 유지)
    W->>W: home 자세로 정렬 (칸 이동과 같은 안전장치)
    W->>API: RESULT_UNKNOWN / PLACE_NOT_VERIFIED, pose_reached true
```

- 정책 정의: [config/omx-act-policies.json](../../config/omx-act-policies.json). 물체 이름(YOLO 클래스, 한글 이름, 별칭) → 모델(Hugging Face repo), 정책 입력 `front`/`wrist`에 넣을 카메라 역할(`OVERHEAD` 선반을 위에서 봄, `GRIPPER` 그리퍼), 최대 실행 시간. 관제와 워커가 같은 파일을 읽고, 명령에는 파일의 SHA-256(`target.policy.registry_version`)이 고정됩니다. 다르면 워커가 `POLICY_REGISTRY_MISMATCH`로 거부합니다.
- 명령: `target`은 `object_class`(필수)와 선택적 `pick_slot_id`만 받습니다. 관제가 `target.object_class`를 검출기 클래스 이름으로 바꾸고 `target.policy`(`policy_id`, `label`, `model`, `registry_version`)를 붙입니다. 정책이 없는 물체는 400입니다. `place`는 다른 집기 명령과 같이 필요합니다(화면에서는 `CART_1`).
- 워커 검사(거부 사유): 가짜·dry-run 팔(`POLICY_NEEDS_REAL_ARM`), 정책 파일 버전·정책 ID(`POLICY_REGISTRY_MISMATCH`, `POLICY_UNKNOWN`), home 미기록(`POSE_NOT_RECORDED`), 만료(`EXPIRED`), 카메라 장치 없음(`CAMERA_UNAVAILABLE`), 다른 프로그램이 카메라 사용 중(`CAMERA_BUSY`), 정책 서버 응답 없음(`POLICY_SERVER_UNAVAILABLE`), 현재 자세가 home에서 400틱 넘게 떨어짐(`NOT_AT_REST`, 정책은 쉬는 자세에서 시작하도록 학습됨).
- 카메라 넘겨주기: V4L2 카메라는 한 프로그램만 열 수 있습니다. 관제가 정책 카메라(`CONTROL_STATION_ACT_OVERHEAD_CAMERA`, `CONTROL_STATION_JOINT_CAMERA`와 같은 장치)를 화면에 띄우고 있으면, 워커가 정책 명령을 가져가는 순간(`DISPATCHED`) 그 카메라를 닫고 명령이 끝나면(`DISPATCHED`·`ACCEPTED`·`RUNNING`이 아니게 되면) 다시 엽니다. 그동안 화면에는 "학습 정책 실행 중"이 표시됩니다. 워커는 카메라를 열기 전에 다른 프로세스가 장치를 열고 있지 않은지 최대 5초 기다리고, 그래도 열려 있으면 `CAMERA_BUSY`로 거부합니다. 2026-11-07 임시 관제에서 실제 헤드캠(허브 4.4)으로 확인: 명령을 가져가는 응답(0.04초) 안에 닫히고, 명령이 끝난 뒤 다시 `STREAMING`.
- 실행: ACCEPTED 뒤 서보 속도 프로파일을 0(학습 때와 같은 전원 기본값)으로 되돌리고, LeRobot `RobotClient`를 워커 프로세스 안에서 돌립니다. LeRobot이 연결하면서 모터 설정을 쓰느라 토크가 잠깐 꺼집니다. 정지할 때는 토크를 유지합니다(`disable_torque_on_disconnect=False`).
- 멈추는 조건: 정책에는 완료 신호가 없어서, 워커가 정책이 쓴 관절 목표를 보고 정합니다. 어느 관절이든 home에서 300틱 넘게 벗어난 뒤 다섯 관절이 모두 150틱 안으로 돌아와 1.5초 머물면 `RETURNED_TO_REST`입니다(값은 워커 옵션 `--policy-depart-ticks`, `--policy-rest-ticks`, `--policy-settle-s`). 학습 에피소드가 쉬는 자세에서 끝나도록 녹화돼 있어서 이 조건을 씁니다.
- 결과:

| 정책 정지 이유 | 그 뒤 동작 | 보고 |
|---|---|---|
| 쉬는 자세로 돌아옴 | 실제 관절이 home에서 300틱 안이면 home으로 정렬 | `RESULT_UNKNOWN` / `PLACE_NOT_VERIFIED`, `pose_reached: true`. 300틱 넘으면 움직이지 않고 `FAILED` / `REST_NOT_CONFIRMED` |
| 최대 시간 안에 움직이지 않음 | 없음 (쉬는 자세) | `FAILED` / `POLICY_DID_NOT_MOVE` |
| 최대 시간 초과, 쉬는 자세 아님 | **자동 복귀 안 함.** 그 자리에서 토크 유지 | `FAILED` / `POLICY_TIMEOUT_NOT_AT_REST` |
| 운영자 취소 | 그 자리에서 토크 유지 | `CANCELED` |
| 통신·카메라·모터 오류 | 그 자리 | `RESULT_UNKNOWN` / `POLICY_RUN_ERROR` |

  쉬는 자세로 돌아온 것은 집기·적재 성공을 뜻하지 않습니다. 그래서 `SUCCEEDED`를 보내지 않고(관제도 `grasp_confirmed`·`place_confirmed` 없이는 받지 않음), 운영자가 현장을 보고 **현장 확인 완료**로 닫습니다. 시간 초과 때 자동 복귀하지 않는 것은 물건을 든 채나 선반 근처에서 관절 공간으로 크게 움직이지 않기 위해서입니다. 이때는 현장 확인 뒤 원상복귀 버튼을 씁니다.
- 실행 방법: [ACT 집기 정책 실물 시험](runbook.md#act-집기-정책-실물-시험)의 `server`와 `worker`.
- 작업 흐름 연결(선택, `CONTROL_STATION_TASK_REAL_ARM=1`): 이동·팔 작업(`TRANSPORT_AND_ARM`)의 `EXECUTE_ARM_TASK`가 작업의 팔 작업 문구에 들어 있는 물체 이름(예: "당근 집기")으로 정책을 고릅니다. 명령을 만들기 전에 **정면 카메라(`CONTROL_STATION_PICK_EVIDENCE_CAMERA`, 기본 `shelf_camera`)가 그 물체를 칸 하나에서만 보고 있어야** 합니다(SR-13, 수동 "정면 캠으로 칸 찾기"와 같은 판정). 보이면 작업 이력에 `TARGET_CONFIRMED`(칸·신뢰도·촬영 시각)를 남기고 그 관측을 `source.kind: CAMERA`·`target.pick_slot_id`로 붙여 `PICK_WITH_POLICY`를 만듭니다. 못 보면(`CAMERA_NOT_FOUND`, `CAMERA_NOT_FRESH`, `TARGET_NOT_FOUND`, `TARGET_AMBIGUOUS`, `TARGET_MULTIPLE` 등) **팔 명령을 만들지 않고** 그 코드로 작업이 실패하며 실패 원천은 `front_camera`로 기록됩니다. 수동 조작 탭의 "확인 없이 실행"과 달리 작업 흐름에는 우회가 없습니다. 명령이 만들어지면 워커가 가져가 실행하고, 결과를 기다립니다(0.5초마다 조회, 정책 최대 시간+180초 넘으면 `ARM_RESULT_TIMEOUT`). `RESULT_UNKNOWN`/`PLACE_NOT_VERIFIED`+`pose_reached`면 단계 성공(`ARM_HOME_PLACEMENT_NOT_VERIFIED`), 그 밖의 결과는 그 사유 코드로 작업 실패입니다. `VERIFY_ARM_RESULT`는 판단을 `VERIFY_LOAD`로 넘긴다고 기록합니다. 실제(모의가 아닌) 적재 관측이 확인되면 팔 명령에 현장 확인을 기록하고, 모의 적재 관측이면 팔 명령을 `UNKNOWN`으로 남겨 운영자가 닫습니다. 작업을 취소하면 팔 명령도 취소합니다. 나머지 단계(주행·도착 확인 등)는 기존과 같습니다. 2026-11-07 가짜 워커로 성공·정책 시간 초과·정책 없는 작업 문구를 단위 테스트했고, 실물로는 아직 실행하지 않았습니다. 2026-11-11 정면 카메라 확인을 추가해 단위 테스트(카메라 근거가 명령 source로 전달, 미검출·확인 경로 없음이면 팔 명령 0건)와, 카메라 없는 임시 관제+모의 워커로 HTTP 작업 실행(`CAMERA_NOT_FOUND`, 원천 `front_camera`, 팔 명령 0건)을 확인했습니다. 관제 화면 작업 이력에는 정면 카메라 확인, OMX 명령 등록(명령 ID·정책·칸), OMX 명령 상태(워커 대기열→워커가 가져감→접수→실행 중→결과)가 표시됩니다. **실제 정면 카메라가 물체를 본 상태에서 작업 흐름으로 실물 팔을 움직이는 시험은 아직 하지 않았습니다.**
- 2026-11-07 확인: 단위 테스트(정책 선택, 명령 고정값, 성공 조건, 정지 판정, 물체별 카메라 대응), 임시 관제(카메라 없음)에서 "당근" 명령 등록 → 모의 워커 `RESULT_UNKNOWN`/`PLACE_NOT_VERIFIED`, 가짜 팔 워커 `POLICY_NEEDS_REAL_ARM` 거부, 실물 팔 포트에서 정책 서버 주소를 틀리게 줘 `POLICY_SERVER_UNAVAILABLE` 거부(팔 읽기만). **관제에서 실물 정책 실행, 정면 카메라의 당근 검출, 정지 판정과 home 정렬은 아직 실물로 시험하지 않았습니다.**

## ROS 2 action 워커 (`/omx/move_to_slot`)

같은 칸 이동을 ROS 2 action으로 받는 워커입니다. 관제(또는 `ros2 action send_goal`)가 action 클라이언트, [omx_ros_worker.py](../../scripts/omx_ros_worker.py)가 action 서버입니다. 이동 로직과 안전장치는 HTTP 워커와 같은 [omx_slot_motion.py](../../scripts/omx_slot_motion.py)를 씁니다. 메시지 정의는 [cpp/omx_interfaces/action/MoveToSlot.action](../../cpp/omx_interfaces/action/MoveToSlot.action)입니다.

- 목표: `command_id`, `slot_id`, `slot_map_version`(칸 정의 SHA-256), `valid_until`(필수, 0이면 `VALID_UNTIL_MISSING`). 좌표 필드는 없고, 워커는 그 칸에 기록한 자세를 재생합니다.
- 진행: `phase: MOVING`, 관절 이름, 현재 관절값, 목표와의 최대 오차. 첫 진행은 목표 위치를 쓰기 **직전**에 보냅니다. 따라서 첫 진행이 오기 전에는 팔이 움직이지 않습니다.
- 결과: `outcome`(`SUCCEEDED`/`FAILED`/`RESULT_UNKNOWN`/`CANCELED`/`REJECTED`), `pose_reached`, `reason_code`, `message`, 최종 관절값, 최대 오차, `simulated`.
- ROS 상태: `SUCCEEDED`만 succeeded, 취소는 canceled, 나머지(검사 거부 포함)는 aborted입니다. 판단은 `outcome`과 `pose_reached`로 합니다.
- 한 번에 목표 하나만 받습니다. 이동 중에 온 목표는 ROS 단계에서 받지 않습니다(`Goal was rejected`). 결과가 불명확해도 워커가 다시 시도하지 않습니다.

```bash
cd cpp && colcon build --packages-select omx_interfaces && cd ..   # 처음 한 번
source /opt/ros/jazzy/setup.bash && source cpp/install/setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
sg dialout -c "~/lerobot-venv/bin/python scripts/omx_ros_worker.py"      # 실물 팔
~/lerobot-venv/bin/python scripts/omx_ros_worker.py --fake-follower      # 팔 없이 (모의)

# 다른 터미널 (같은 source/export 후)
ros2 action send_goal --feedback /omx/move_to_slot omx_interfaces/action/MoveToSlot \
  "{command_id: TEST-1, slot_id: slot_a_2_1, slot_map_version: $(sha256sum config/shelf-slots.json | cut -d' ' -f1), \
    valid_until: {sec: $(( $(date +%s) + 60 ))}}"
```

`ros2 action send_goal`은 Ctrl+C로 끊어도 취소를 보내지 않았습니다(Jazzy, 2026-11-04). 취소는 action 클라이언트 코드에서 `cancel_goal_async()`로 시험했습니다. Ctrl+C를 누르면 취소를 보내는 클라이언트 [omx_move_to_slot_client.py](../../scripts/omx_move_to_slot_client.py)와 단계별 실습 절차는 [OMX 팔 ROS 2 단계별 실습](omx-ros-practice.md)에 있습니다.

2026-11-04 확인 (`--fake-follower`만, 팔 미연결): `slot_a_2_1` 이동 `SUCCEEDED`(`simulated: true`)를 확인했습니다. 기록 안 된 칸, 만료, 칸 정의 다름, 유효 시각 없음, 다른 좌표계(이 확인은 2026-11-14에 목표의 좌표 필드와 함께 지움)는 `REJECTED`였고 진행 메시지 없이 끝났습니다. 이동 중 두 번째 목표는 ROS가 거부했고, 이동 중 취소는 그 자리에 멈춘 뒤 `CANCELED`였습니다. 같은 날 HTTP 워커도 공용 모듈로 바꾼 뒤 `--fake-follower`로 임시 관제에 붙여 `POSE_NOT_RECORDED` 거부와 `SUCCEEDED`를 다시 확인했습니다. **실물 팔과 관제 action 클라이언트는 아직 시험하지 않았습니다.**

## 실물 워커를 만들 때 (OMX 팀)

[mock_omx_worker.py](../../scripts/mock_omx_worker.py)가 틀입니다. 표준 라이브러리만 쓰므로 LeRobot 가상환경에서 그대로 실행됩니다.

1. `check()`에서 칸 정의 버전, `valid_until`, 기록 자세 유무를 확인하고 문제가 있으면 `REJECTED`를 보냅니다.
2. `execute_phase()`에 기록 자세 재생이나 학습 정책 실행을 넣습니다. 실물 구현은 [omx_lerobot_worker.py](../../scripts/omx_lerobot_worker.py)입니다.
3. 모든 이벤트에 `simulated: false`를 보냅니다.
4. 파지와 적재를 모두 확인한 경우에만 `SUCCEEDED`, 확인 수단이 없으면 `RESULT_UNKNOWN`을 보냅니다.
5. 이벤트 POST가 200이 아니면(특히 `ACCEPTED`) 그 명령을 더 진행하지 않습니다.
6. 텔레오퍼레이션과 워커를 동시에 켜지 않습니다. 같은 모터 포트를 두 프로그램이 쓰게 됩니다.

## 남은 일

- 작업 흐름 실물 시험: 실제 도착 확인(`CONTROL_STATION_PARKING_SPOT`)·정면 카메라 검출 상태에서 `EXECUTE_ARM_TASK` → 실물 워커 → 적재 확인까지
- 정면 카메라 검출 신뢰도 하한(현재 검출기 기본 0.25) 팀 합의
- 실행 중 결과 제한 시간과 그때의 처리
- `reason_code` 목록을 OMX 팀과 확정
- 원격 배포 시 인증
