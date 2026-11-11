# OMX 팔 ROS 2 단계별 실습

LeRobot으로 움직이는 OMX 팔을 ROS 2 action `/omx/move_to_slot`으로 다뤄 보는 실습 절차입니다. 앞 단계는 모터에 아무것도 쓰지 않고, 마지막 단계에서만 실물 팔이 움직입니다. 각 단계의 결과를 확인한 뒤 다음 단계로 넘어갑니다.

- 서버(워커): [omx_ros_worker.py](../../scripts/omx_ros_worker.py). LeRobot의 Dynamixel 버스로 팔을 칸 자세까지만 옮깁니다. 집기는 하지 않습니다.
- 클라이언트: [omx_move_to_slot_client.py](../../scripts/omx_move_to_slot_client.py). 목표 하나를 보내고 진행과 결과를 출력합니다. **Ctrl+C를 누르면 취소를 보냅니다.** `ros2 action send_goal`은 Ctrl+C를 눌러도 취소를 보내지 않습니다.
- 도우미: [omx_ros_practice.sh](../../scripts/omx_ros_practice.sh). 모든 단계에 같은 ROS 환경을 맞춥니다(Jazzy + `omx_interfaces`, `ROS_DOMAIN_ID=87`, localhost만). domain은 `OMX_ROS_DOMAIN_ID`로 바꿉니다.
- 메시지 정의, 결과 의미, 안전장치는 [팔 명령 API의 ROS 2 절](arm-command-api.md#ros-2-action-워커-omxmove_to_slot)을 따릅니다.

이 실습은 하드웨어 전원 차단이나 비상정지를 대신하지 않습니다. 실물 단계에서는 전원 차단 스위치에 손이 닿게 두세요.

## 0단계: 준비 확인 (모터와 통신하지 않음)

```bash
scripts/omx_ros_practice.sh check
```

ROS 2, `omx_interfaces` 빌드, LeRobot 가상환경, dialout 권한, 팔로워 포트, 기록된 자세를 보여 줍니다. 포트가 보여도 팔이 움직일 준비가 됐다는 뜻은 아닙니다.

## 1단계: 가짜 모터로 action 익히기 (팔 불필요)

터미널 A:

```bash
scripts/omx_ros_practice.sh worker fake
```

터미널 B:

```bash
scripts/omx_ros_practice.sh shell                 # ROS 환경이 맞춰진 셸
ros2 action list -t
ros2 action info /omx/move_to_slot
ros2 interface show omx_interfaces/action/MoveToSlot
exit

scripts/omx_ros_practice.sh goal slot_a_2_1       # 성공: SUCCEEDED, 모의 결과
```

한 번 이동하면 가짜 팔이 이미 목표에 있으므로 다시 보내면 거리 0으로 바로 끝납니다. 취소를 연습하려면 워커를 다시 시작하세요.

| 해 볼 것 | 명령 | 기대 결과 |
|---|---|---|
| 기록 안 된 칸 | `goal slot_a_2_2` | `REJECTED POSE_NOT_RECORDED`, 진행 메시지 없음 |
| 만료된 목표 | `goal slot_a_2_1 --valid-for -5` | `REJECTED EXPIRED` |
| 다른 칸 정의 | `goal slot_a_2_1 --slot-map-version deadbeef` | `REJECTED SLOT_MAP_MISMATCH` |
| 이동 중 취소 | `goal slot_a_2_1` 후 Ctrl+C, 또는 `--cancel-after 1` | `CANCELED`, 그 자리에서 멈춤 |
| 이동 중 두 번째 목표 | 이동 중 다른 터미널에서 `goal ...` | ROS 단계에서 받지 않음 |
| 워커 없음 | 워커를 끄고 `goal slot_a_2_1` | 서버를 찾지 못함, 보낸 목표 없음 |

결과 줄의 `모의 결과`는 실물 모터가 움직이지 않았다는 뜻입니다. 판단은 ROS 상태가 아니라 `outcome`과 `pose_reached`로 합니다.

## 2단계: 실물 모터 읽기만

팔로워 보드를 USB로 연결하고 팔 전원을 켭니다. 리더 텔레오퍼레이션이나 LeRobot 정책이 같은 포트를 쓰고 있지 않아야 합니다.

```bash
scripts/omx_ros_practice.sh probe
```

현재 관절값, 토크, 하드웨어 오류, 전압을 읽기만 합니다. `write_or_motion_command_sent: false`를 확인합니다. 이 결과로 팔이 안전하거나 준비됐다고 판단할 수는 없습니다.

포트는 칸 자세를 기록한 보드의 by-id 경로(`…OpenRB-150_F58AE686…`)를 씁니다. udev 규칙의 `/dev/omx_follower`는 다른 시리얼 번호를 가리키므로, 워커가 보드 불일치로 시작을 거부합니다.

## 3단계: home 자세 기록 (읽기만)

칸으로 갔다가 돌아올 자세를 하나 기록해 둡니다. 토크가 꺼진 상태에서 팔을 손으로 칸 자세 **근처**로 옮긴 뒤 기록합니다.

```bash
scripts/omx_ros_practice.sh record home
```

워커는 한 관절이라도 1200 ticks(약 105°)보다 많이 움직여야 하면 `TARGET_TOO_FAR`로 거부합니다. 따라서 home은 `slot_a_2_1`에서 모든 관절이 그보다 가깝게 있어야 왕복할 수 있습니다. 처음에는 수십 도 이내로 잡으세요. 다시 기록하려면 `--replace`를 붙입니다. 기록 결과는 `config/omx-slot-poses.json`에 저장됩니다.

## 4단계: 실물 dry-run (읽기만, 쓰지 않음)

```bash
scripts/omx_ros_practice.sh worker dry      # 터미널 A
scripts/omx_ros_practice.sh goal slot_a_2_1 # 터미널 B
```

워커는 실물의 현재 관절값을 읽고 계획한 목표와 변화량을 출력합니다. 모터에 쓸 내용은 `(dry-run)` 줄로만 보여 줍니다. 결과는 `RESULT_UNKNOWN DRY_RUN_NO_MOTION`입니다. 실제로 움직이지 않았으므로 성공이라고 하지 않습니다. 변화량이 예상한 크기인지 이 단계에서 확인합니다.

## 5단계: 실물 이동

시작 전 확인:

- 팔 주변에 사람이나 물건이 없습니다. 선반이 있으면 팔이 지나갈 경로가 비어 있습니다.
- 전원 차단 스위치에 손이 닿습니다.
- 4단계에서 본 변화량이 작습니다.

```bash
scripts/omx_ros_practice.sh worker real     # 터미널 A, "실물" 입력 후 시작
scripts/omx_ros_practice.sh goal slot_a_2_1 # 터미널 B
scripts/omx_ros_practice.sh goal home
```

- 속도는 서보 프로파일로 약 14°/s입니다. 처음에는 더 느리게 해 보세요: `worker real --profile-velocity 5`
- 이상하면 클라이언트 터미널에서 Ctrl+C를 누릅니다. 워커가 현재 위치를 목표로 써서 팔이 멈추고, 결과는 `CANCELED`입니다. 이 경로로 멈추지 않으면 전원을 끊습니다.
- 결과 줄이 `실물 모터 결과`이고 `SUCCEEDED pose_reached=True`이면, 측정 관절값이 허용 오차(15 ticks) 안에 들어왔다는 뜻입니다. 팔 끝이 칸의 물건에 맞게 놓였다는 확인은 아닙니다.
- 워커를 끝내도 토크는 켜진 채로 남아 팔이 처지지 않습니다. 끄려면 팔을 손으로 받친 뒤 `scripts/omx_ros_practice.sh release`를 실행합니다.

## 확인 기록

- 2026-11-04: 1단계 전부(성공, 세 가지 거부, `--cancel-after`와 Ctrl+C 취소, 이동 중 두 번째 목표 거부, 서버 없음)와 0단계 `check`를 확인했습니다. 취소 시험에는 home을 임시로 넣은 자세 파일(`--poses`)을 썼습니다.
- 2026-11-04 실물 팔로워(`F58AE686…`) 2단계: 여섯 모터 모두 응답했고 토크 꺼짐, 하드웨어 오류 0, 읽기 오류 없음, 쓰기 없음.
- 2026-11-04 실물 4단계: `slot_a_2_1` 계획 변화량 최대 289 ticks(약 25°), `RESULT_UNKNOWN DRY_RUN_NO_MOTION`. 처음에는 `sg dialout`이 `LD_LIBRARY_PATH`를 지워 워커가 rclpy를 불러오지 못했습니다. 도우미가 `sg` 안에서 ROS 환경을 다시 맞추도록 고쳤고, `check`가 같은 경로를 검사합니다.
- 2026-11-04 실물 3단계: 토크가 꺼진 채 놓여 있던 자세를 `home`으로 기록했습니다. `slot_a_2_1`과의 관절 차이는 최대 289 ticks입니다.
- 2026-11-04 실물 5단계 첫 시도(`home`→`slot_a_2_1`, `--profile-velocity 5`): 실패, 원인 미확인. 출발 2~3초 뒤 `shoulder_lift`가 1957 ticks 부근에서 멈췄고(목표 2153), 다른 관절은 목표 근처까지 갔습니다. 이후 `shoulder_lift`가 1807로 내려갔고 읽는 값이 바뀌지 않다가 `RESULT_UNKNOWN MOTOR_COMM_ERROR`로 끝났습니다. 같은 시각 팔로워 보드 USB 장치가 다시 잡혔습니다. 직후 읽기 전용 조회에서 여섯 모터 모두 토크 꺼짐, 하드웨어 오류 0이었습니다. 모터가 재시작된 것으로 보이지만 전원 문제인지 충돌인지는 확인하지 못했습니다. `home` 복귀는 보내지 않았습니다.
