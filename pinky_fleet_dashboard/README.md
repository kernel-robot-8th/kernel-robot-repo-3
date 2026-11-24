# 핑키 관제 (Pinky Fleet Dashboard)

PC에서 Pinky Pro 세 대(10·11·12번)를 주행준비하고, 경로 주행·적재 자리 진입·수령 구역 배송 임무를 맡기는 관제입니다. 주문을 이끄는 관제 서버(`src/control_station`)는 이 관제에 HTTP로 붙어 임무 단계를 읽고 `dock_complete`·`received` 신호만 보냅니다.

운영 기준(서비스 실행, 로봇 코드 적용, 공유 구역, 로그)은 [OPERATIONS.md](OPERATIONS.md)를 따릅니다.

## 하는 일

- **주행준비:** SSH로 로봇의 bringup, ADC 센서, IMU, Nav2를 띄우고, Nav2 lifecycle 노드가 모두 활성인지 확인합니다. 이동 목표는 보내지 않습니다. 준비는 한 대씩 차례로 진행합니다. (`nav_readiness.py`, `install_nav2_runtime.py`)
- **위치 찾기:** 라이다 스캔 한 번을 지도와 맞춰 위치를 찾습니다. 대칭 후보가 모호하면 운영자가 고르거나, 지도에서 핑키 위치와 방향을 끌어 다시 잡습니다. (`scan_localizer.py`)
- **임무 주행:** 주차장 → 적재 자리(5번) → 수령 구역 → 주차장 경로를 Nav2 목표로 차례로 보냅니다. 적재 자리에서는 천장 카메라 안내를 받아 들어가고, 관제 서버의 신호를 기다립니다. (`navigate_route.py`, `mission_behaviors.py`, `route_geometry.py`)
- **공유 구역과 양보:** 통로, 적재 자리, 주차장 출입은 예약으로 한 대씩만 들어갑니다. 두 대가 마주치면 우선순위에 따라 한 대가 물러납니다. (`traffic.py`, `yielding.py`)
- **안전 경로:** 모터 명령은 `Nav2 → 행동 선택 → 독립 최종 감시기 → /cmd_vel` 순서로 나갑니다. 최종 감시기는 20 Hz로 명령·센서·관제 허가가 끊기면 멈춥니다. (`velocity_guard.py`, `motion_gate.py`, `ultrasonic_safety.py`)
- **라이다·초음파 퓨전:** 로봇 둘레 확률 점유 격자로 라이다 평면보다 낮은 장애물을 Nav2 지역 costmap에 넣습니다. (`fusion_grid.py`, [설명](../docs/implementation/sensor-fusion.md))
- **센서 표시:** 라이다, IR, 초음파, 배터리, IMU, 오도메트리, AMCL 위치와 각 값의 신선도를 화면에 보여 줍니다.
- **핑키 램프와 화면:** 상태 색과 지금 하는 일, 표정, 배터리를 핑키의 LED와 LCD에 보냅니다. (`status_lamp.py`, `status_display.py`)

관제는 `127.0.0.1:8765`에서만 열립니다. SSH 비밀번호는 실행 중 메모리에만 두고 저장하지 않습니다.

## 구조

| 위치 | 내용 |
|---|---|
| `app.py` | 관제 HTTP 서버와 화면 API, 로봇별 상태, SSH로 로봇 프로세스 시작·정리 |
| `web/` | 관제 화면 (`index.html`, `app.js`), IR 보정 페이지 |
| `scripts/` (PC 쪽) | 준비 확인 `nav_readiness.py`, 위치 찾기 `scan_localizer.py`, 구역 예약 `traffic.py`, 양보 `yielding.py`, 운영 상태 `fleet_ops.py`, 기록 `control_journal.py`, 퓨전 녹화·평가 `fusion_record.py`·`fusion_eval.py` |
| `scripts/` (로봇에 올라감) | 경로 주행 `navigate_route.py`, 행동 `mission_behaviors.py`, 최종 감시기 `velocity_guard.py`, 초음파 게이트 `ultrasonic_safety.py`, IR 보조 `line_following.py`, 퓨전 `fusion_grid.py`, 램프 `status_lamp.py`, 화면 `status_display.py`. 주행준비 때 `runtime_bundle.py`가 묶어 올림. `motion_gate.py`·`traffic.py`·`control_journal.py`는 PC와 로봇 양쪽에서 씀 |
| `config/` | 지도(`map/`), 금지 구역(`keepout/`), Nav2·초음파·통로 설정(`navigation.json`), 주차 칸(`parking.json`), 차체·센서 치수(`robot_geometry.json`) |
| `data/waypoints.json` | 웨이포인트 (5번 적재 자리, 4번 대기 자리, 수령 1·2 등) |
| `tests/` | 오프라인 회귀 테스트 (ROS 모터 발행·SSH 없음) |

## 요구 사항

- ROS 2 Jazzy가 설치된 Ubuntu PC (`rclpy`, 표준 ROS 메시지 패키지, OpenSSH 클라이언트, `python3-scipy`). `domain_bridge`는 `./run.sh --bridges`로 실행할 때만 필요
- PC에서 핑키 세 대의 SSH에 접속할 수 있는 네트워크
- 로봇 계정이 sudo 비밀번호 없이 ROS 2를 실행할 수 있어야 함

로봇의 ROS overlay는 원격으로 `/home/pinky/pinky_pro/install/setup.bash`를 source합니다. `map-proto.yaml`을 고르면 Nav2를 띄우기 전에 이 PC의 `~/Documents/Pinky_backup/`에서 지도 YAML·PGM을 각 로봇으로 복사합니다.

## 처음 세팅 (새 PC)

1. **준비물**: Ubuntu, ROS 2 Jazzy, `python3-scipy`, OpenSSH 클라이언트. PC가 핑키 세 대와 같은 네트워크에 있어야 합니다.
2. **핑키 주소 확인**: `app.py`의 `ROBOTS`에 IP가 적혀 있습니다(10번 `192.168.0.9`, 11번 `192.168.0.8`, 12번 `192.168.0.11`). 네트워크가 다르면 여기를 고칩니다.
3. **지도 복사**: 관제는 지도를 저장소가 아니라 홈 폴더에서 읽습니다.

   ```bash
   mkdir -p ~/Documents/Pinky_backup
   cp config/map/map-proto.yaml config/map/map-proto.pgm ~/Documents/Pinky_backup/
   ```

4. **실행**: `./run.sh` 뒤 <http://127.0.0.1:8765>를 엽니다.
5. **SSH 비밀번호 입력**: 저장소에 없습니다. 팀원에게 받아 관제 화면에 넣습니다.
6. **주행준비**: 핑키를 자기 주차 칸에 세우고 [주행준비]를 누릅니다. 전원만 켠 핑키는 ROS가 뜨지 않습니다.

저장소에 이미 들어 있는 값:

| 파일 | 내용 |
|---|---|
| `config/map/` | 셀 지도(`map-proto`) — 3번에서 복사하는 원본 |
| `data/waypoints.json` | 웨이포인트: 5번 적재 자리, 4번 대기 자리, 수령 1·2 등 |
| `config/parking.json` | 핑키별 주차 칸 좌표 |
| `config/navigation.json` | Nav2·초음파·통로·대기 자리 설정 |
| `config/robot_geometry.json` | 차체·센서 치수 |
| `config/keepout/` | 들어가면 안 되는 구역 |

웨이포인트와 주차 좌표는 이 지도의 좌표입니다. 지도를 새로 만들면 웨이포인트도 다시 기록해야 합니다. 지도를 고쳤을 때는 `~/Documents/Pinky_backup/`의 파일을 `config/map/`에도 복사해 커밋합니다.

IR 보정값(`data/ir_profiles.json`, `data/ir_calibration/`)은 저장소에 없습니다. IR 조향(`ir_steering`)이 꺼져 있는 동안은 필요하지 않습니다.

## 실행

평소에는 사용자 서비스 `pinky-fleet.service`로 실행합니다([OPERATIONS.md](OPERATIONS.md#pc-관제)). 직접 실행하려면 이 폴더에서:

```bash
./run.sh            # 이미 실행 중이면 주소만 출력하고 끝남
./run.sh --bridges  # 도메인 0의 RViz 등에서 핑키 토픽을 봐야 할 때 (서비스를 멈춘 뒤)
```

<http://127.0.0.1:8765>를 열고 SSH 비밀번호를 넣은 뒤 핑키마다 [주행준비]를 누릅니다. 핑키를 자기 주차 칸에 세워 두고 시작합니다. 주행준비는 로봇의 바퀴에 힘을 줄 수 있으니 바퀴 주변을 비워 두고, 필요하면 물리 정지를 씁니다.

## 테스트

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

ROS 모터 발행이나 SSH 없이 돕니다. ROS 노드 스모크 테스트는 로봇 도메인과 분리된 도메인 83에서 돕니다: `source /opt/ros/jazzy/setup.bash && PYTHONPATH=".:$PYTHONPATH" python3 tests/ros_fusion_smoke.py`. 오프라인 통과를 실제 주행 성공으로 보지 않습니다.

## 세부 동작과 변경 기록

아래는 날짜별 구현 기록입니다. 앞쪽 영어 부분은 초기 구현(2026-11-09 무렵) 설명이라 속도·허용 오차·브리지 방식 같은 값이 지금과 다를 수 있습니다. 현재 값은 `config/navigation.json`, [OPERATIONS.md](OPERATIONS.md), 날짜가 붙은 최신 항목을 기준으로 봅니다.

### 초기 구현 개요 (영어)


- Shows Pinky C76C / D631 / 1E3E as separate cards.
- Starts one robot's base bringup, ADC sensors, BNO055 IMU, then Nav2.
- Uses the supplied `map-proto.yaml` / `map-proto.pgm` map by default (also supports `my_map.yaml` and `pinklab.yaml`).
- Receives `/scan`, IR, ultrasonic, battery, IMU, odometry, map, and AMCL pose through ROS 2 Domain Bridge.
- Uses best-effort QoS for LiDAR scans at the robot bridge and dashboard subscriber, matching sensor-data publishers.
- Displays per-sensor freshness so a missing/stale feed is visible.
- Records named waypoints from the live `map → odom → base_footprint` TF pose, per robot; exports CSV.
- Applies the `map-proto` Keepout Filter mask to both Nav2 costmaps; the original map stays unchanged.
- Sends Pinky 10's saved CSV waypoints to Nav2 in the order `1 → 2 → 3 → 7 → 4 → 5 → 6 → 7 → 9 → 8 → 1`; each goal is sent after the previous one succeeds.
- Verifies Nav2 publishes the `/navigate_to_pose` action before marking navigation ready; the dashboard can restart Nav2 in non-composed mode if the action server is missing.
- Listens on `127.0.0.1:8765` only. The SSH password is entered at runtime and held in memory only.

The route start button sends navigation goals through Nav2. Nav2 plans around the active map and obstacles, so straight segments are followed where its planner considers them safe. Preparing Nav2 can energize the robot's base; keep the wheels clear during setup and use the physical stop if needed.

Dashboard-managed Nav2 starts use a runtime copy of the installed configuration. Goal tolerances are 0.03 m and 0.12 rad; target forward speed is 0.048 m/s (24% of the package default), and the velocity smoother caps forward speed at 0.06 m/s. Inflation radius is reduced from 0.15 m to 0.08 m; global footprint padding is reduced from 0.03 m to 0.01 m while the robot footprint is preserved. The ultrasonic safety gate sits between Nav2 and `/cmd_vel`: a fresh reading at or below 0.10 m causes a 0.5 s stop, then a 0.25 rad/s turn until readings remain clear (at least 0.15 m, or positive infinity) for 0.75 s, followed by a 0.03 m/s forward creep for up to 0.5 s. A renewed obstacle interrupts motion; stale ultrasound data commands zero velocity. The updated detour described below replaces the original timed creep.

The live Nav2 map shows the TF pose as the robot center and heading, plus the AMCL pose as a center point with a 95% covariance radius. The readout includes the TF-to-AMCL position difference and the estimated error radius when fresh covariance data is available. Nav2 readiness waits up to about 90 seconds for `/navigate_to_pose` during lifecycle activation. The initial-pose publisher waits for AMCL's `/initialpose` subscription and uses best-effort QoS, then publishes the origin estimate three times.

Each prepared robot card has **초기 위치로 이동**. After physically placing that robot at the map origin, the button publishes an AMCL initial pose `(0, 0, 0°)`; it does not drive the robot. AMCL may adjust the estimate from LiDAR after the reset.


### 초기 실행 방식 (영어)

From this directory:

```bash
./run.sh
```

Open <http://127.0.0.1:8765>. Enter the Pinky SSH password in the dashboard, then press **주행준비** on one robot. The dashboard blocks another start until the current robot's bringup sequence finishes or reports a fault.

The app starts three one-way Domain Bridge processes from ROS domains 10, 11, and 12 into PC domain 0. Topic names are remapped to `/pinky_10/...`, `/pinky_11/...`, and `/pinky_12/...` so robot data stays separate. The bridge configuration is in `config/bridge/`. The run script starts the bridge executable directly and cleans up leftover bridge processes from earlier runs.

### Sensor launch details

`pinky_bringup bringup_robot.launch.xml` starts the chassis, LiDAR, odometry, joint states, robot TF, and battery percentage/voltage publishers. It does not start the ADC sensor or IMU executables, so this dashboard starts these separately:

```bash
ros2 run pinky_sensor_adc main_node
ros2 run pinky_imu_bno055 main_node
```

The ADC node publishes `ir_sensor/range` (`UInt16MultiArray`), `us_sensor/range` (`Range`), and `batt_state` (`BatteryState`). The IMU node publishes `imu_raw` (`Imu`). The IR channel order is shown as CH1–CH3 until its physical left/center/right mapping is verified.

The Pinky Pro camera is not included because the checked real-robot bringup does not expose a camera ROS topic. Add it only after confirming its driver and topic on the robot.

### Runtime files

SSH launch logs and PID files are written under `/home/pinky/.cache/pinky-fleet/` on each robot. The local process writes bridge logs to a temporary directory and removes that directory on exit.

The dashboard reads battery percentage and voltage topics; it does not publish or modify battery values. The battery percentage is shown to one decimal place, with the ROS topic source and sample age.

Use **전체 초기화** to stop dashboard-tracked bringup, ADC, IMU, and Nav2 launch processes on all three robots, and to stop a lingering `teleop_twist_keyboard` process. The dashboard and PC-side Domain Bridge remain running. The reset first publishes a zero velocity command.

Recorded waypoints persist in `data/waypoints.json` in this directory. A waypoint is only recorded while a map has arrived and the live `map → odom → base_footprint` TF chain is available; stop the robot before recording each point. The map is static and may publish once, so its age is not treated as sensor staleness. Pinky 10's route is sent as sequential `NavigateToPose` goals. Nav2 may choose a detour when needed to avoid obstacles; it does not force a collision-prone straight line. The route can be cancelled from the same card. The dashboard checks for the `/navigate_to_pose` action server before declaring Nav2 ready and offers a Nav2 restart control after route failures. Pinky Pro's separate web interface on port 8080 can set an initial pose and send Nav2 goals; do not launch its `web_nav2.launch.xml` alongside this dashboard's Nav2 bringup, because that launch file also starts Nav2.

### Manual waypoint collection

1. Start the selected robot with its saved map. The dashboard needs the localized TF pose in the `map` frame before recording.
2. In a separate SSH terminal on that robot, source ROS and the Pinky overlay, then run keyboard teleop:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/pinky/pinky_pro/install/setup.bash
   export ROS_DOMAIN_ID=10 # use 11 or 12 for the other robots
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. Drive to a point, stop, enter a label in that robot's dashboard card, and click **현재 위치 기록**. Saved points can be removed individually or exported as CSV.

### Keepout zones

When `map-proto.yaml` is selected, Nav2 loads `config/keepout/keepout_mask.yaml` and applies the associated mask in both global and local costmaps. Dark cells in `keepout_mask.pgm` are forbidden cells; white cells remain traversable. The mask is aligned to the map's 5 cm resolution and `[-0.349, -0.295, 0]` origin. The yellow boxes from the supplied map screenshot are represented as filled zones. The original occupancy map is not edited.

Regenerate the mask after changing the map frame or zone bounds with `python3 scripts/create_keepout_mask.py`. The script reads the source map metadata and derives the mask dimensions, resolution, and origin. Keepout is enabled only for `map-proto.yaml`; the other supported maps are not assumed to share its coordinates. This does not add charging or docking behavior.

Routes are expanded to checkpoints at most 0.10 m apart. Measured anchor positions are preserved. Yaw is chosen per visit from the outgoing segment; the final stop retains the incoming heading. Intermediate yaw follows the segment tangent, avoiding turns caused by recorded headings on straight sections. Each successful Nav2 action must also have a fresh robot-local map TF within 0.05 m of its target before advancing. AMCL update_min_d is 0.05 m. The dashed preview is an intended geometric route; obstacle avoidance may change the actual path. Dense CSV exports are available from each route card.

### IR steering assistance and LiDAR

IR assistance is optional and disabled until calibrated and explicitly enabled on a prepared robot. CH1/CH2/CH3 map to physical left/center/right as confirmed by the operator. The dashboard's **IR 차선 보조 + LiDAR** panel captures live, multi-sample ADC medians and robust noise estimates separately for the floor and each channel's tape reading. ADC polarity is inferred independently for each channel. Insufficient contrast or unstable readings block activation. Calibration files are stored per robot in `data/ir_profiles.json`; enabling uploads that robot's profile and waits for its revision to be acknowledged.

1. With navigation stopped and IR assistance off, put all three sensors over bare floor for at least one second and click **바닥값 저장**.
2. Put the tape under CH1, hold for one second, then click **CH1 선값**. Repeat for CH2 and CH3 individually.
3. Prepare the robot to install the integrated guard if not already running. Click **IR 보조 켜기**, check its live status, then use the route start button.

The existing ultrasonic guard is the sole dashboard-owned `/cmd_vel` publisher. Nav2/velocity smoother provide `/cmd_vel_nav2`; the guard adds bounded IR PD steering only to forward, nearly straight commands with a visible line and clear frontal LiDAR. Left line strength produces positive/left angular correction; right strength produces negative/right correction. The IR correction is limited to 0.15 rad/s, total angular command to 0.30 rad/s, and normal forward target to 0.048 m/s. Larger line errors reduce speed. Turns, reverse commands, nearby LiDAR obstacles, missing line and ambiguous all-dark patterns hand steering back to Nav2. This is assistance to the existing Nav2 route, not a standalone line-only graph navigator.

IR-assisted trajectories are checked against LiDAR points transformed into `base_footprint` using the actual scan TF. A 14 cm square footprint is swept over one second; detected collisions stop the corrected command. While IR assistance is enabled, stale IR, stale/untransformable LiDAR, or invalid calibration stops motion. Ultrasound remains higher priority. The recovery sequence also requires a fresh, nonzero Nav2 command, so placing an obstacle near an idle robot does not initiate recovery. Sensor blind spots and localization errors remain physical limits; no autonomous body movement is used for calibration.

`/line/status` (JSON in `std_msgs/String`) is bridged separately for each robot. It reports enabled state, acknowledged profile revision, line error/strength, and whether Nav2, IR or a safety stop is active. All-dark readings are marked as unclassified marker candidates; they do not identify a node or reset odometry/AMCL. Returning from an obstacle detour can resume IR correction when the line is visible and Nav2 is moving forward again.

To revert to the prior Nav2 control behavior, stop the route and click **IR 보조 끄기**; ultrasound protection remains active. P10's prior guard was backed up on the robot as `ultrasonic_safety.before-ir.py`. Do not run the pasted standalone line follower alongside this guard, because it would add a competing `/cmd_vel` publisher.

Validation: `python3 -m unittest discover -s tests -v`. The ROS adapter smoke check is explicitly isolated from robot domains: `source /opt/ros/jazzy/setup.bash && PYTHONPATH=".:$PYTHONPATH" python3 tests/ros_ir_smoke.py` uses localhost-only domain 83.

Center-first policy: CH2 is the line reference. With only CH2 on the line, small Nav2 steering corrections are suppressed and the candidate straight command is checked by LiDAR. CH2 plus either side is a valid corner pattern and leaves the planned turn to Nav2. Side-directed PD recovery is used only while CH2 is off the line. Center detection uses 0.55/0.35 hysteresis; side detection uses 0.50 normalized line strength. Both sides on with CH2 off is ambiguous and hands back to Nav2. All-dark still does not identify a node.

#### 임시 IR 횡방향 캘리브레이션 페이지

관제 상단의 **IR 캘리브레이션 수집** 또는 `http://127.0.0.1:8765/ir-calibration`에서 P10/P11/P12를 선택합니다.

- 로봇을 차선과 평행하게 놓고, 선이 로봇 중심의 오른쪽이면 `offset_cm > 0`으로 입력합니다. 로봇을 왼쪽으로 밀면 양의 오프셋입니다.
- 정지한 뒤 수집 버튼을 누르면 요청 이후의 새 IR 원시 메시지 20개를 최대 5초간 수집합니다. IR·오도메트리가 신선해야 하며, 관제 주행/준비 중 또는 바퀴 움직임 감지 시 거부합니다. 손으로 옆으로 미는 움직임까지 오도메트리가 보장해 감지하는 것은 아니므로 수집 동안 위치를 유지해야 합니다.
- 테이프/센서 높이/조명 조건을 바꿀 때 새 세션을 생성합니다. 기체별 기록은 `data/ir_calibration/pinky_<domain>.json`에 저장하며 기존 IR 주행 보정값과 독립적입니다.
- 추정기용 평균 CSV는 BOM 없이 `offset_cm,ch1,ch2,ch3` 4열로 내보냅니다. 같은 세션·오프셋에서 체크된 재측정 기록은 모든 샘플을 합쳐 평균합니다.
- 원시 CSV에는 20개 샘플과 PC 수신 시각을, 통계 CSV에는 수집별 평균·모집단 표준편차·최솟값·최댓값을 담습니다. 잘못 측정한 기록은 사용 체크를 해제하면 내보내기에서 제외되며 원본은 보관합니다.
- 페이지는 이동 명령을 발행하지 않으며, 추정기/자동 조향에 CSV를 자동 적용하지 않습니다.

#### 2026-11-09: 2cm–2cm–2cm 차선 / 횡방향 CSV / 연속 구간 제어

- 실제 차선은 검정 20mm–흰색 20mm–검정 20mm, 센서 중심 간격은 10mm(CH1~CH3 20mm)입니다.
- P10의 `s20261109121154748_9399b540` 자료(9개 오프셋, 10회 수집, 200개 raw)를 `data/ir_templates/`에 원본 그대로 보존했습니다. 모델에는 파일 SHA256과 측정 기체를 기록합니다. P10 모델을 P11/P12에 자동 복제하지 않습니다.
- 모델은 -2~+2cm 측정값을 1mm 격자로 보간하되 이를 1mm 정확도로 표시하지 않습니다. 중앙 ±5mm는 센서 포화/재측정 변동으로 세밀한 위치를 구별하기 어려워 안정 구간으로 처리합니다. 불일치·모호성·보정 범위 끝에서는 외삽 조향 대신 제한 탐색/정지를 사용합니다.
- 양의 오프셋은 선이 오른쪽에 있다는 의미이며 ROS 조향은 음수입니다. 필터된 이탈량 P, 이탈 변화율 D, 오도메트리 회전속도 감쇠를 사용합니다. 중앙 안정 주행 중만 방향 참고값을 천천히 갱신하며, 처음 방향을 유지하는 절대 yaw 보정을 CSV 조향과 합산하지 않습니다.
- Nav2 목표는 기준 웨이포인트 사이 구간당 1개입니다. 기존 91개 10cm 보간점은 통과 시 TF 좌표 기록으로 남기며 개별 도착 목표로 보내지 않습니다. 5cm 초과 통과 오차는 화면에 표시하고, 기준점 도착 오차는 5cm 한도를 유지합니다. 최종 목표 yaw는 해당 구간의 진입 방향으로 지정하며 다음 구간이 실제 코너 회전을 담당합니다.
- 경로 컨텍스트는 주행 중 갱신되며 실제 코너 반경 12cm 안에서만 Nav2 조향을 허용합니다. 컨텍스트가 1.5초 이상 끊기면 초음파 회복 중이어도 정지합니다. `/cmd_vel`에 수동 키보드 등 다른 발행자가 있으면 경로 시작을 거부합니다.
- 관제에서 정지 중에도 추정 이탈량·후보 범위·패턴 불일치도를 볼 수 있습니다. 표시되는 후보 범위는 측정 패턴 유사도 기반이며 통계적 신뢰구간이 아닙니다. 실제 주행 성능 검증과는 별개입니다.

#### 교차로 통과·차선 재획득 / 5번 도킹

- `three_surface_pd_v2`: 교차로 출구에 도달한 뒤에도 공통 IR 제어기를 계속 실행합니다. 옆 채널의 흰선 또는 검정 테두리·바닥 조합으로 저속 복귀하며, 방향 단서가 없으면 전진하지 않고 좌우 탐색합니다. CH2 흰선을 0.3초 연속 확인하면 일반 추종으로 돌아가고, 탐색 기억은 인계 중 유지합니다. 지도 방향에서 벗어났다는 이유로 재탐색을 취소하지 않습니다.
- 방향 단서 없는 탐색 각도는 좌우 약 11.5도로, 6초 제한 안에 양쪽을 탐색하도록 조정했습니다. 6초/약 23도 제한, 교차로 전체 이동·회전 제한 및 초음파·LiDAR 정지는 유지합니다. 검정 3채널만으로는 흰선 방향을 확정하지 않습니다.
- 주차장→1번 진입을 마친 첫 직선은 교차로 회전으로 분류하지 않습니다. 각 로봇의 기존 IR 보정값은 유지합니다. 구형 감시기가 실행 중이면 경로 시작을 막으며, 재연결 후 **주행준비**로 공통 소스를 배포해야 합니다.

- 경로 컨텍스트에 시작·끝 위치, 진입/출구 방향과 교차로 여부를 전달합니다. 회전 지점과 경로에서 반복 방문하는 교차점(현재 7번)에 진입하면 통과 상태를 유지합니다. 다음 구간으로 넘어가 출구 방향 오차 약 11도 이내, 교차점에서 출구 방향으로 5cm 이상 진행, 사용 가능한 차선 패턴을 0.3초 연속 확인 후 IR 제어로 돌아옵니다. IR 보조가 꺼져 있으면 위치·방향 조건으로 통과를 완료합니다.
- 접근 20초·회전 20초·차선 재획득 12초를 각각 적용하고 전체 최대 60초·누적 이동 65cm·누적 회전 3.6rad 안에서 저속으로 수행하고, 실패 시 정지하며 경로를 중단합니다. 초음파 장애물 정지/회복과 센서 신선도 조건은 계속 적용합니다.
- **도킹 시작:** 현재 목표가 5번이고 지도상 5번 반경 5cm 이내, 보정된 세 IR 선 강도 모두 0.5 이상인 새 메시지가 3개 연속 들어오면 `도킹중`입니다. 고정 ADC 기준은 사용하지 않습니다.
- **도킹중:** Nav2 목표를 취소하고 경로를 일시정지한 채 1cm/s로 진입합니다. 최대 10cm 또는 10초 후에는 정지하여 확인을 기다립니다. IR 이탈 역시 새 메시지 3개로 확인하며, 도킹 중에만 한 번 최대 2cm/2초를 1cm/s로 후진한 뒤 정지합니다. 후진도 LiDAR footprint와 센서 신선도를 확인합니다.
- **도킹완료:** 관제의 `도킹완료 확인` 입력과 세 IR의 연속 흰색 확인이 함께 충족되면 정지합니다. `물건 수령 완료` 버튼 전까지 계속 정지를 유지합니다. 이 상태에서는 IR 색이 바뀌어도 자동 후진하지 않습니다.
- **재개:** `물건 수령 완료` 입력을 받으면 5번 서비스를 완료 처리하고 다음 구간(6번 방향)을 진행합니다. 계획된 도킹 이동은 5번의 일반적인 5cm 도착 검증과 별도로 기록합니다.
- 관제 명령은 `POST /api/robots/<id>/docking`의 JSON `{ "action": "complete" | "received", "dock_id": "현재 도킹 회차 ID" }`입니다. 현재 회차 ID는 `/api/status`의 `topics.line_control.value.docking.dock_id`에서 읽습니다. 현재는 PC 내부 주소의 API이며, 외부 장치 연결은 후속 연동이 필요합니다. 지금은 화면 버튼이 같은 API를 사용합니다.
- 버튼은 해당 기체의 진행 중인 경로·상태·센서 수신을 확인하며, 명령은 현재 도킹 회차에만 적용되고 3초 후 만료합니다. 관제는 로봇의 적용 응답을 받은 뒤 성공을 표시합니다.
- 도킹 정지 상태는 로봇의 `dock_latch.json`에 기록합니다. 감시기 재시작 시 이동 중이던 도킹은 자동 재진입하지 않고 정지 확인 대기로, 도킹완료는 정지 유지로 복구합니다. 경로 취소/통신 끊김에는 자동 경로 재개를 하지 않습니다.

#### 통합 관제와 동시 경로 주행

- P0=DOMAIN 10(핑크), P1=11(초록), P2=12(파랑). 공용 지도에 같은 지도 메타데이터를 사용하는 로봇의 최신 map TF 위치·방향과 AMCL 95% 오차 반경을 겹쳐 표시합니다. 오래된 좌표는 화살표로 표시하지 않습니다.
- 상단 `세 대 동시 경로 시작`은 준비·센서·기체별 IR 보정·새 감시기가 모두 확인된 세 로봇에 시작 요청을 병렬 전송합니다. `주행 중인 로봇 전체 정지`도 로봇별 취소 요청을 병렬 전송합니다. 경로 실행 상태는 로봇별로 관리하여 세 대를 동시에 시작할 수 있습니다. 준비·설정 작업의 짧은 예약과 장시간 경로 주행을 분리했습니다. 정지·도킹 확인·수령은 해당 로봇에만 적용합니다. 초기 위치 설정은 현재 각 로봇 map (0, 0, 0°)를 유지합니다.
- 이것은 각 로봇의 Nav2·LiDAR·초음파에 의한 독립 주행입니다. 교차로 우선권·공용 도킹 자리 예약·상호 경로 조율은 구현되어 있지 않습니다. 각 로봇의 실제 위치와 초기 추정 좌표가 일치해야 공용 지도 위치를 비교할 수 있습니다.
- 5번 도착 후 흰색 패턴이 없으면 Nav2 목표를 취소한 상태에서 오도메트리 기준 3cm 전진 → 3cm 후방 위치까지 후진 → 탐색 출발 위치로 복귀합니다. 속도 1cm/s, 최대 26초·누적 15cm·출발점 반경 4.5cm·방향 오차 약 11도 제한입니다. 전방 초음파·전후방 LiDAR·센서/TF 신선도와 경로 heartbeat를 계속 확인합니다.
- 새 IR 메시지에서 세 채널 흰색을 3회 연속 확인하면 도킹으로 전환합니다. 탐색 소진 시 경로 오류로 종료하지 않고 `도킹 탐색 종료 · 정지` 상태에서 기다립니다. IR 보정 오류 자체는 탐색으로 해결하지 않으며 보정을 수정하려면 경로를 먼저 정지해야 합니다.

#### 주차장 출발·종료 임무 (2026-11-10)

- 설정은 `config/parking.json`: P0 (0.40,0.40)m, P1 (0.70,0.40)m, P2 (1.00,0.70)m. 사용자가 세 기체 모두 평행한 180° 방향임을 확인했습니다.
- 초기 위치 버튼은 물리 이동 없이 각 기체의 지정 주차장 좌표·방향을 AMCL `/initialpose`에 발행합니다. 출발 때 주차장 위치 오차 5cm·방향 오차 15°를 확인합니다.
- 경로는 자기 주차장 → 기존 코스(1→2→3→7→4→5→6→7→9→8→1) → 정지·완료 신호 대기 → 자기 주차장입니다. 주차장 출입에는 강제 직진을 사용하지 않고 Nav2를 사용하며 복귀 목표의 yaw도 주차장 yaw입니다.
- 관제의 `주행완료 · 주차장 복귀` 버튼은 해당 기체의 코스 완주 후에만 활성화됩니다. 향후 서버 연동은 `POST /api/robots/<id>/mission-complete` JSON `{ "token": "현재 mission_token" }`를 사용합니다. 다른 임무 토큰과 완주 전 신호는 거부합니다. 현재 HTTP 서버는 localhost에만 열립니다.
- 완료 신호를 기다리는 동안 route heartbeat와 명시적인 정지 상태를 유지합니다. 자동 복귀나 시간 초과 이동은 하지 않습니다. 임무 완료 상태는 주차장 도착 이후에만 기록합니다.
- IR 보정을 미룬 현재 세 기체는 IR 보조를 꺼서 Nav2로 주행합니다. IR 조향·흰색 탐색·5번 도킹은 비활성입니다. 같은 센서 모델이어도 설치 높이·채널 편차에 따라 기체별 보정값은 달라질 수 있어 다른 기체의 값을 복제하지 않았습니다.
- 검증: `python3 -m unittest discover -s tests -p test_parking_mission.py -v`. HTTP 병렬 요청·로봇별 시작/취소 상태·완주 전 신호 거부·토큰 격리와 세 runner의 완주 대기/복귀 흐름을 모의 실행합니다. 이 테스트는 물리적인 동시 주행 검증을 대체하지 않습니다.

- 관제 센서 수신은 각 ROS 도메인에 독립적인 rclpy context/노드/executor를 두고 직접 구독합니다. 브릿지 재기동 뒤 관제에서 일부 도메인이 재발견되지 않는 현상을 피하고, 기존 domain_bridge는 다른 ROS 도구의 domain 0 수신을 위해 유지합니다. 실시간 데이터는 best-effort, 지도와 정적 TF는 transient-local QoS를 사용합니다.
- 실기 검증 기록은 `data/verification/parking_20261110.json`: 세 Nav2 실제 목표 동시 접수·독립 취소를 정지 출력 상태에서 확인했습니다. P0 2초, P1 4초, P2 5초 뒤 각자 취소됐고 모두 canceled 상태였습니다. 물리적인 코스 완주/복귀 검증은 수행하지 않았습니다.

#### 원점 정렬·필수 도킹 대기 수정

- P2 주차장은 (1.00,0.40)m로 정정했습니다. 세 기체의 주차장 yaw는 180°입니다.
- 주차장→1번 원점의 도착 yaw는 코스 출발 방향 0°로 지정합니다. Nav2 목표 허용치는 위치 2cm·yaw 0.05rad입니다. 원점 도착 후 수동 확인 없이 다음 코스 목표를 자동으로 전송합니다. IR 보정 전에는 Nav2의 위치·방향 추정을 사용하며 실제 선 중앙 정렬을 센서로 보증하지는 않습니다.
- IR 비활성은 도킹 생략을 뜻하지 않습니다. 5번에서 반드시 정지하여 `도킹완료 확인`을 받고, 다시 `물건 수령 완료`까지 정지합니다. IR 활성 상태의 센서 기반 도킹은 기존처럼 도킹완료+흰색 유지·수령 확인을 모두 요구합니다.
- 작업자 확인은 `/api/robots/<id>/operator-confirm`으로 action(dock_complete,received)과 현재 대기 단계의 mission_token을 보냅니다. 단계마다 새 토큰을 발급하여 이전 단계·다른 로봇·중복 지연 입력으로 다음 단계를 해제할 수 없습니다.

#### 검정·흰색·회색 바닥을 구분한 복귀 제어

- 채널의 `line` 값이 바닥보다 높은 경우 검정 테두리 기준으로 해석합니다. 기존 CH1/CH3의 4095를 흰선으로 취급하지 않습니다. 흰선 강도와 검정 강도는 각각 바닥과의 보정된 상대 거리로 계산합니다. CH1/CH3에 별도 흰색 기준이 없으면 CH2 실측 흰색을 임시 공통 참조하고 상태에 표시합니다. 향후 `white`/`black` 채널별 기준을 지원하며 실제 채널별 측정을 권장합니다.
- 흰색이 한쪽에만 보이면 그 방향으로 저속 복귀합니다. 흰색이 안 보이더라도 `CH1 검정 + CH2/CH3 회색 바닥`이면 왼쪽으로, 대칭 패턴이면 오른쪽으로 1.2cm/s 전진하며 회전합니다. 바닥만 보이면 맹목적으로 전진하지 않고 마지막 방향을 먼저 회전 탐색합니다.
- 복귀 한도는 6초·출발 방향 대비 약 23도입니다(진행 속도에 의한 최대 전진 약 7.2cm). LiDAR footprint, 전방 장애물 우선권, 초음파 및 센서 신선도 검사는 계속 적용합니다. 한계 초과 시 정지하고 경로에 코드16으로 원인을 알립니다. 기존 구간 전체 240초 시간 초과를 연장하지 않았습니다.
- CH2 흰색이 0.25초 연속 확인된 뒤 복귀 상태를 끝내고 현재 방향을 새 기준으로 사용합니다. 중앙 안정 시 방향 기준을 서서히 갱신하고, 흰선 위치의 PD와 조향 변화율 제한으로 좌우 흔들림을 줄입니다.
- 참고한 제조사 알고리즘: https://www.pololu.com/docs/0j18/all (보정값·폐루프 PID·마지막 검출 방향 기억). 세 색의 경계 패턴 상태기는 이 트랙을 위한 별도 구현입니다.

#### 정지 및 초기위치 복구

- 로봇별 정지 버튼은 경로가 없거나 취소 대기 중이어도 사용할 수 있습니다. 전체 정지도 연결된 모든 로봇에 요청합니다. 정지 요청은 로봇의 `stop_latch.json`에 저장되며 안전 감시기는 다른 조건보다 먼저 0 속도를 발행합니다. 관리 중인 경로 프로세스를 종료하고 해당 ROS 도메인의 Nav2 목표만 취소합니다.
- 정지 유지 해제는 새로운 명시적 경로 실행에서 새 Nav2 목표가 수락된 다음에만 합니다. 시작과 동시에 정지가 들어오면 run_id가 일치하는 정지 요청을 새 실행이 제거하지 않습니다. 경로 취소 상태는 원격 적용 응답으로 종료하며 SSH 로그의 끝을 무한히 기다리지 않습니다.
- 위치 재설정 때 센서·지도·안전 감시기가 없거나 AMCL 구독자가 없으면 해당 기체의 주행준비를 한 번 복구합니다(주행준비가 위치까지 잡음). 실패하면 준비 완료 대신 오류 상태로 표시합니다. 현재 위치가 지정 주차장과 일치할 때 위치 재설정 버튼을 사용해야 합니다.
- 주행준비는 Nav2 액션 확인 뒤 자기 주차장 좌표(`config/parking.json`)를 AMCL `/initialpose`로 보내고, 그 뒤에 받은 `/amcl_pose` 또는 map 기준 TF 위치가 주차장에서 15 cm·약 20° 안에 12초 안에 들어오면 위치를 잡은 것으로 봅니다(`_localize_at_parking`). 오도메트리가 움직이는 중이면 보내지 않습니다. 실패해도 Nav2는 준비된 상태로 두고 오류 내용을 남기며, 지도 위치(`tf_pose`)가 없으므로 주행 가능으로 보이지 않습니다. 이 확인은 AMCL이 좌표를 받았다는 뜻이지 로봇이 실제로 그 칸에 있다는 증거가 아니므로, 주행준비 전에 로봇을 자기 주차 칸에 둡니다. Nav2 복구는 주행 중일 수 있어 위치를 넣지 않습니다.
- `POST /api/prepare-all {map}`은 주행준비가 안 된 기체(주행 중·준비됨·SSH 포트 미응답 제외)를 하나의 작업(`busy_robot="all"`)으로 차례로 준비합니다. 한 기체가 실패해도 다음 기체를 계속 준비합니다. `GET /api/status`의 `password_set`은 SSH 비밀번호를 메모리에 갖고 있는지만 알려 줍니다.


#### 초음파 차체 폭 우회 및 정지 경합 수정 (2026-11-11)

- 공통 감시기 `three_surface_pd_v3`: 10cm 이내 감지 → 0.5초 정지 → LiDAR로 덜 막힌 방향을 선택해 약 60도 회전 → 오도메트리 기준 원래 경로 옆으로 최소 10cm 확보 → 원래 진행 방향 복원 → 최초 감지 거리와 차체 여유를 포함해 전방 최소 22cm 통과 후 IR/Nav2에 인계합니다. 전진은 3cm/s입니다. 이전 0.5초/1.5cm 전진은 제거했습니다.
- 차체는 기존 14cm 사각형으로 검사합니다. 요청한 센서 좌우 각 5cm를 포함하며, 회전·측면 확보·방향 복원·통과 모두 LiDAR와 초음파 감지점의 예상 궤적을 검사합니다. LiDAR에 보이지 않는 낮은 장애물도 초음파 감지 위치를 오도메트리 좌표로 기억해 검사합니다. 좁아서 회전/전진할 수 없으면 정지하며 60초 또는 출발점 반경 70cm 한도에서 실패 처리합니다. 단일 초음파만으로 장애물 전체 폭을 알 수 없으므로 실제 통과는 재연결 후 확인해야 합니다.
- 오도메트리·LiDAR·초음파가 끊기면 정지합니다. 경로 신호 만료·작업자 정지가 회피보다 우선하며, 교차로 감시기는 회피 시간과 이동량을 별도로 취급합니다.
- 완주 대기에서 잘못된 튜플 파일 경로를 수정했습니다. 정지 확인 전에는 `cancel_requested`를 유지하고 재출발을 차단하며, 확인 실패 시 정지 재시도를 허용합니다. 큐에 대기하던 출발 작업도 취소 상태에서는 실행하지 않습니다.
- 오프라인 회귀 테스트는 ROS 모터 명령·SSH 없이 수행합니다. 로봇별 IR 보정값은 변경하지 않습니다. 재연결 후 세 기체의 주행준비로 새 공통 소스를 적용합니다.


#### 스펀지 도킹 스테이션 / 5번 기준 위치 보정

- P0/P1/P2 모두 5번은 `(0.39, 1.64)m`, 진입 방향은 4→5의 −X(yaw 180°)입니다.
- `도킹완료 확인` 후 (IR 사용 시 세 채널 연속 흰색 확인 후) AMCL `/initialpose`에 5번 위치와 진입 방향을 전달합니다. 최신 AMCL 및 map→base TF가 3cm/5° 안으로 반영되어야 수령 버튼이 활성화됩니다. 바퀴 `odom` 자체는 연속성을 유지합니다. 구독/반영 실패 시 오류 코드 18로 정지하며 수령 대기를 건너뛰지 않습니다.
- `물건 수령 완료` 후 2cm/s로 약 20cm 직선 후진합니다. 바퀴 odom 진행량을 사용하며 횡방향 2cm/방향 약 7° 이탈 또는 30초 초과 시 정지합니다. 로봇 제어 프로세스가 후진 중 재시작되어도 후진 원점을 새로 잡아 추가 후진하지 않습니다.
- 경로는 `5 (0.39,1.64) → 도킹 탈출 (0.59,1.64) → 6 (0.59,1.24)`입니다. 세 기체의 저장된 6번 좌표와 10cm 경로 미리보기/CSV가 함께 변경되었습니다. 후진 중에도 도킹 구역 예약과 독립 최종 충돌 감시를 유지합니다. 후방 장애물이나 오래된 센서 데이터가 있으면 후진하지 않습니다.
- 새 소스 적용 여부는 `topics.line_control.value.dock_protocol == sponge_v1`로 확인하며, 이전 감시기로는 경로를 시작할 수 없습니다. 연결 후 각 기체의 주행준비 또는 Nav2 복구를 실행해야 합니다.
- 통합 지도는 `config/keepout/keepout_mask.yaml`의 원점·해상도·회전과 PGM의 상하 방향을 반영해 금지 셀을 보라색 사선으로 표시합니다. 노란 웨이포인트, 청록색 도킹 후진 경로와 구분합니다. 이 표시는 마스크 파일의 표시이며 로봇 Nav2 필터의 실행 상태를 뜻하지 않습니다. 파일 변경은 다음 관제 갱신에 반영됩니다.


#### 네트워크 재연결 후 PC에서만 scan/odom이 안 보일 때

로봇의 직접 DDS 샘플이 모두 정상인데 PC 관제의 센서가 전부 미수신이면 로봇 bringup보다 PC 수신 상태를 먼저 확인합니다. 오래 실행된 DDS 참여자는 네트워크 재연결 후 재시작이 필요할 수 있습니다. 다른 기체의 진행 중인 임무가 없는 상태에서 `systemctl --user restart pinky-fleet.service`를 실행한 후 SSH 연결 설정과 주행준비를 다시 실행합니다. 브라우저 새로고침만으로는 DDS 프로세스가 갱신되지 않습니다.


#### 목표 즉시 거부 / SSH 종료 코드 5

2026-11-14 수정: keepout `lethal_override_cost`는 ROS 정수 파라미터이므로 `252`로 생성합니다. 실수 `252.0`은 local_costmap 활성화를 실패시키고 BT 액션을 inactive 상태로 남깁니다. 준비 확인은 액션 목록뿐 아니라 map_server/amcl/controller_server/planner_server/bt_navigator/behavior_server의 GetState가 모두 active인지 확인합니다. Nav2 정리 시 설치 패키지 launch와 캐시 경로 launch를 모두 종료합니다. 주행 준비 확인 과정에서는 이동 목표를 전송하지 않습니다.

#### Nav2 주행, 라이다·초음파 퓨전, IR은 보조값 (2026-11-14)

- 주행은 Nav2가 맡습니다. `config/navigation.json`의 `ir_steering: false`이면 IR 보정 프로필이 켜져 있어도 경로의 `use_ir`은 false이고, IR 값은 화면과 기록에만 쓰입니다.
- `ultrasonic_fusion: true`이면 주행준비 때 로봇에 `fusion_grid.py`를 띄웁니다(2026-11-16, 이전 `ultrasonic_cloud.py`를 대신함). 라이다와 초음파를 로봇 둘레 2 m 확률 점유 격자(log-odds, 2 cm 칸)에서 합칩니다. 칸마다 라이다 높이(약 12.5 cm)와 초음파 높이(약 3.7 cm) 두 층을 두고 각 센서는 자기 높이 층만 고칩니다. 초음파 반사 호에 라이다가 이미 보는 장애물이 있으면 반사를 그 장애물 칸에만 주고, 라이다가 그 호를 비었다고 보면 라이다 평면보다 낮은 장애물로 분류합니다. 자세한 모델·실험 방법은 [docs/implementation/sensor-fusion.md](../docs/implementation/sensor-fusion.md).
- Nav2에는 낮은 층만 넘깁니다. 지역 costmap의 `VoxelLayer`(5 cm 높이 칸)에 `/scan`과 함께 `/fusion/low_marks`(낮은 층의 점유 칸, 표시)와 `/fusion/low_clear`(격자 가장자리 링, 지우기)를 넣어, Nav2의 맨 아래 칸이 퓨전 격자를 따릅니다. 전역 costmap에는 `ultrasonic_fusion_global: true`일 때만 넣습니다(기본 false): 전역에 잘못 찍힌 표시는 로봇이 그쪽을 다시 볼 때까지 남아 2.2 × 2.6 m 칸에서 경로를 막을 수 있습니다. `publish_voxel_map`은 끕니다. 높이는 URDF 설계값이며 실측이 아닙니다.
- 설정은 `navigation.json`의 `fusion_grid`(기본값은 `scripts/fusion_grid.py`의 `DEFAULTS`). `lidar_sigma_m`·`us_sigma_m`은 임시값이며 노이즈 측정으로 바꿉니다. 잘못된 설정이면 주행준비가 시작 전에 알립니다. 상태는 `/fusion/status`(JSON), 격자는 `/fusion/grid`(`OccupancyGrid`, odom)로 나오고 브리지가 PC의 `/pinky_<id>/fusion/*`로 넘깁니다.
- 녹화·평가: `scripts/fusion_record.py`(PC에서 브리지 토픽을 읽기만 함)로 기록하고 `scripts/fusion_eval.py noise|compare`로 노이즈와 세 방식(라이다만·초음파 단순 표시·퓨전) 비교를 냅니다. 검증: `python3 -m unittest tests.test_fusion_grid tests.test_fusion -v`, ROS 노드는 `source /opt/ros/jazzy/setup.bash && PYTHONPATH=".:$PYTHONPATH" python3 tests/ros_fusion_smoke.py`(도메인 83, 로봇 없음).
- 최종 안전장치(`velocity_guard.py`)는 퓨전 노드를 쓰지 않고 라이다 점과 초음파 기억을 직접 봅니다. 퓨전 노드가 죽어도 마지막 정지는 따로 동작하게 하려는 것입니다.
- `ultrasonic_detour: false`이면 `ultrasonic_safety`는 10 cm 이내에서 자체 우회를 하지 않고 전진 명령만 막습니다. 회전·후진은 Nav2 명령대로 통과합니다. 설정이 없으면 기존 우회 동작 그대로입니다.
- 지역 위치는 라이다로 찾습니다(`scripts/scan_localizer.py`, `/api/robots/<id>/localize`, `/locate`). 경기장이 직사각형이라 180° 대칭 후보가 나오면 주차 칸·5번 같은 알려진 자리에 맞는 후보가 하나일 때만 자동으로 고르고, 아니면 운영자가 후보를 고릅니다.
- 위치 다시 잡기: 주행준비 끝에 라이다로 자동으로 찾고, 지도에 보이는 자리가 틀리면 운영자가 관제 화면 큰 지도에서 핑키가 있는 곳을 누른 채 바라보는 방향으로 끌어 다시 잡습니다(RViz 2D Pose Estimate, 기똥차 관제와 같은 방식). `POST /api/robots/<id>/set-pose {pose}`는 그 자세 주변 30 cm·30° 안에서만 라이다로 다듬어 AMCL에 넣으므로 대칭인 반대편으로 튀지 않습니다. 경로 주행 중에는 거절합니다.
- Nav2 bringup: Jazzy 1.3의 lifecycle 관리자는 노드 상태 응답을 2초만 기다리고, 바쁜 라즈베리파이가 늦으면 bringup을 통째로 그만둡니다. `scripts/nav_readiness.py`가 Nav2 로그에서 이를 보면 그 관리자의 `node_names`를 configure→activate 순서로 직접 이어서 켜고, 그것도 안 되면 관제가 Nav2를 한 번 다시 띄웁니다. 로그는 이어 쓰므로 실행마다 `=== PINKY_FLEET_START` 줄로 구분합니다.
- 상태 램프: 관제가 `/fleet/indicator`로 색을 보내고 로봇의 `status_lamp.py`가 `set_lamp`/`set_led`에 반영합니다(주황 준비·위치 없음, 파랑 대기, 초록 주행, 빨강 정지·오류, 3초 끊기면 주황 깜빡임). `pinky_lamp_control`이 `WS2811_STRIP_GBR`이라 빨강·파랑을 바꿔 보냅니다.
- 마주침 양보 (2026-11-24, `scripts/yielding.py`): 통로·적재 자리는 구역 예약이 한 대씩 들여보내지만, 그 밖에서 두 핑키가 코를 맞대면 서로 멈췄다 물러났다를 되풀이했고, 통로 입구에서는 들어가려고 기다리는 핑키가 나오는 핑키를 막았습니다. 관제가 0.25초마다 두 핑키가 50 cm 안에서 서로를 앞(±55°)에 두고 있는지 보고, 그러면 한 대에게 `/fleet/yield`(`{"id","to","back_m"}`)를 계속 보냅니다. 받은 핑키는 라이다로 뒤가 비었을 때만 최대 `yield_back_m`(기본 25 cm, 한도 40 cm) 직선 후진한 뒤 명령이 끊길 때까지 서 있습니다(`mission_behaviors.GiveWay`, 상태 `yield_back`·`yield_wait`). 구역을 기다리며 서 있던 핑키도 물러날 수 있게 최종 게이트는 이 명령이 있는 동안의 순수 후진만 예약 없이 통과시킵니다(충돌 검사는 그대로).
  - 계속 가는 쪽(먼저 갈리는 규칙): ① 적재 자리에 드나들거나 서 있는 핑키·수령 대기 중인 핑키, ② 통로를 잡고 있는 핑키, ③ 적재 자리를 다녀온(물건을 실은) 핑키, ④ 구역을 먼저 요청한 핑키, ⑤ 번호가 작은 핑키. ①·②는 양보시키지 않습니다.
  - 끝나는 때: 두 핑키가 75 cm 넘게 떨어지거나 상대가 지나쳐 갔을 때. 계속 가야 할 쪽이 8초 동안 5 cm도 못 움직이면 역할을 한 번 바꾸고, 25초가 지나면 풀어 주고 8초 동안 그 둘은 다시 묶지 않습니다.
  - 끄기: `config/navigation.json`에 `"head_on_yield": false`. 거리: `yield_meet_m`(기본 0.5), `yield_back_m`.
  - 화면에는 "양보 중", 임무 메시지에는 "다른 핑키에게 길을 비켜 주는 중"으로 나옵니다. 핑키 쪽 코드는 주행준비 때 올라갑니다.
  - **확인한 것: 판정·동작 단위 테스트뿐입니다. 실제 핑키로는 아직 한 번도 돌려 보지 않았습니다.**
- 핑키 화면: 같은 `/fleet/indicator`(1초에 4번)에 지금 하는 일(`scripts/fleet_ops.activity`)과 배터리, 그릴 화면 종류를 함께 실어 보내고, 로봇의 `status_display.py`가 화면에 그립니다. 바탕색은 램프 색이고 이름과 배터리는 늘 위에 나옵니다.
  - 화면 종류: **표정**(주행 중: 보통은 웃는 얼굴, 주차장으로 돌아갈 때는 눈을 반달로 뜨고 크게 웃는 얼굴, 적재 장소 드나들 때는 집중한 얼굴, 장애물을 피할 때는 놀란 얼굴, 아래에 "적재 장소로 이동 중" 같은 한 줄), **배터리**(20% 이하이면 다른 무엇보다 먼저, 하던 일은 둘째 줄. 25%로 올라올 때까지 유지), **STOPPED**(정지를 누른 때와 정지 뒤 명령을 기다리는 동안), **글자**(그 밖: 이름을 크게, "대기 중"·"물건 싣는 중"·"교차로 대기"·"양보 중"·"위치 확인 필요" 등). 장소는 웨이포인트 번호가 아니라 쓰임으로 부릅니다: 5번 = 적재 장소, 4번 = 적재 대기 자리, 수령 1 = 수령 장소 1, 통로 위·아래 = 통로. 3초 끊기면 NO LINK. 로봇에 한글 글꼴이 있으면 한글로, 없으면 짧은 영어로 씁니다.
  - 배터리 숫자는 최근 값의 가운데 값을 쓰고 2%p 이상 달라질 때만 바꿉니다(`BatteryView`). 주행 중에는 값이 1분 안에 65 → 58처럼 출렁여서 그때마다 화면을 다시 그렸습니다.
  - 그리기: pinky_pro의 `pinky_emotion/pinky_lcd.py`를 씁니다. 그 드라이버의 `img_show`는 한 화면(153,600바이트)을 한 번에 보내는데, 주행 중인 핑키에서는 이 전송이 `[Errno 110] Connection timed out`으로 자주 실패해 화면이 한참 예전 그림에 머물렀습니다(2026-11-24, 세 대 모두). 그래서 드라이버의 기본 함수로 **바뀐 줄만 8줄씩** 보내고, 실패한 묶음만 다시 보내며, 남은 것은 다음 틱(0.2초)에 이어 보냅니다. 큰 전송이 여섯 번 연달아 실패하면 32바이트씩 끊어 보냅니다. 실패 횟수와 마지막 그리기 시간은 `/fleet/display_status`의 `write_failures`·`last_draw_s`로 나옵니다.
  - 화면을 못 열면 이유를 20초 동안 `/fleet/display_status`에 알린 뒤 끝납니다. `pinky_emotion`의 `emotion_server`와 같은 화면을 쓰므로 함께 켜지 않습니다.
  - 화면 스크립트만 다시 올리기: `POST /api/robots/<id>/display/reload`. 주행준비를 다시 하지 않고 이 PC의 `scripts/status_display.py`를 로봇에 올려 다시 시작합니다(bringup·Nav2·경로는 건드리지 않음).
  - **2026-11-24 확인한 것:** 실제 핑키 세 대에서 글자 화면이 그려짐(한글 글꼴 있음). 줄 묶음 전송·표정·배터리·STOPPED 화면은 이 PC에서 가짜 화면(전송 3번에 1번 실패)으로만 돌려 봤고, 실제 핑키 화면에서는 아직 확인하지 않았습니다.
