# MuJoCo 시뮬레이션 설정

이 절차는 기존 창고 Gazebo와 분리된 `omx_mujoco_sim` MuJoCo 시뮬레이션만 준비합니다. 실물 OMX, Dynamixel 포트, 카메라 장치에는 연결하지 않습니다. 모델 구조와 MJCF 한계는 [MuJoCo OMX 설계 문서](mujoco-omx.md), ROS workspace 조사 결과는 [ROS 2 inventory](ros2-inventory.md)를 참고하세요.

## 선행 조건

- Ubuntu 24.04와 ROS 2 Jazzy 환경
- 프로젝트 `.venv`에 ROS 연동 Python 의존성 설치: `.venv/bin/python -m pip install -r requirements-ros.txt`
- `mujoco_ros2_control`, `mujoco_ros2_control_plugins`, `controller_manager`, `control_msgs`, `joint_state_broadcaster`, `joint_trajectory_controller`, `launch`, `launch_ros`, `robot_state_publisher`, `tf2_ros`, `xacro` ROS 패키지
- `simulation/ros2_ws`에서 빌드된 `omx_mujoco_sim` workspace overlay
- 실물 로봇과 다른 simulator가 사용하지 않는지 운영자가 확인한 ROS domain ID (`1`–`232`)

현재 시스템 prefix에는 ROS 2 Jazzy, MuJoCo Python, `tf2_ros`가 있지만 `mujoco_ros2_control`, `mujoco_ros2_control_plugins`, `joint_trajectory_controller`는 설치되지 않았습니다. 세 package와 필요한 apt dependency를 `/tmp/ros-mujoco-prefix`에 `.deb` 다운로드·추출해 만든 임시 overlay에서 runtime을 확인했습니다. 이 과정은 시스템 package를 설치하거나 저장소 파일을 변경하지 않습니다. 기본 실행 wrapper는 시스템 prefix 기준이므로 해당 local overlay를 환경에 추가하지 않으면 누락 패키지에서 launch 전에 종료합니다. 재현 가능한 로컬 overlay 준비 도구는 아직 없습니다.

Ubuntu Noble의 ROS Jazzy apt 저장소를 쓰는 환경에서는 필요한 ROS package를 다음처럼 설치합니다. 시스템 package 설치는 관리자 권한이 필요한 운영 작업입니다.

```bash
sudo apt-get install \
  ros-jazzy-control-msgs \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-mujoco-ros2-control \
  ros-jazzy-mujoco-ros2-control-plugins \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-xacro
```

선택형 XYZ PointCloud2 변환을 사용하려면 추가로 설치합니다.

```bash
sudo apt-get install ros-jazzy-depth-image-proc
```

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd simulation/ros2_ws
colcon build --packages-select omx_mujoco_sim
```

이 패키지는 MJCF 장면을 simulation geometry 기준으로 사용하고, ROS 2 controller description과 launch 파일을 설치합니다. 빌드 성공만으로 backend plugin 로드, controller 활성화, ROS topic/action 수신이 검증되지는 않습니다.

launch 설정은 [Jazzy CameraPlugin](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control_plugins/doc/plugins.html)의 `camera_publish_rate: 5.0`을 사용해 `placeholder_fixed_camera` RGB-D를 `/mujoco_omx/fixed_camera/{color/image_raw,depth/image_raw,camera_info}`에 게시하도록 요청합니다. 공식 문서는 이 rate가 모든 streaming camera에 공통 적용된다고 설명합니다. 5 Hz는 설정값이며 실제 subscriber 전달률은 아래 timing 관측에 별도로 기록합니다. 임시 손목 카메라 두 개는 비활성화합니다. MJCF fixed camera에는 640×480 resolution metadata를 지정했습니다. 이는 시뮬레이션 설정이며 물리 카메라의 해상도나 intrinsic 측정값이 아닙니다. `world`에서 `mujoco_omx/link0`와 `placeholder_fixed_camera_optical_frame`으로 가는 simulation-only 정적 TF도 launch합니다. 카메라 위치는 MJCF placeholder pose에서, optical 축 회전은 MuJoCo `+X/+Y/-Z`와 ROS optical `+X/-Y/+Z` convention 차이에서 가져왔습니다. 따라서 simulation localization의 TF 입력을 연결하지만, 실물 extrinsic 보정값은 아닙니다. 이 호스트는 `DISPLAY`가 설정되어 있어 plugin log에 GLFW 렌더링이 기록됐습니다. EGL 라이브러리와 그래픽 드라이버는 display-less headless 실행 시 별도 조건입니다.

## 현재 runtime 관측

2026-10-31에 먼저 ROS domain의 topic graph를 확인한 뒤 빈 domain `231`에서 임시 local overlay로 launch했습니다. MuJoCo backend와 CameraPlugin 초기화, GLFW camera renderer, 두 controller의 `active` 상태를 확인했습니다. `FollowJointTrajectory` action endpoint 발견, JointState 6개 수신, RGB `640×480`, depth `32FC1`, CameraInfo `K=[461.0357, 0, 320, 0, 461.0357, 240, 0, 0, 1]`을 읽었습니다. `world→placeholder_fixed_camera_optical_frame`과 `world→mujoco_omx/link0` TF lookup도 성공했습니다. 관제 MuJoCo mode를 임시 SQLite DB로 실행해 `/api/cameras`에서 RGB `STREAMING`, depth/CameraInfo `OBSERVED`와 `FRESH`, 최신 RGB 대비 `HEADER_AND_DIMENSIONS_MATCH`를 확인하고 MJPEG HTTP 200을 받았습니다.

같은 격리 domain에서 MuJoCo와 관제 프로세스를 동시에 실행한 뒤 `/api/health`, `/api/cameras`, 작업 생성 및 `POST /api/tasks/{task_id}/target-observation`을 확인했습니다. Fixed RGB는 `640×480 STREAMING`이었고 모델 설정을 주지 않은 기본 상태는 `DETECTOR_NOT_CONFIGURED`를 기록했습니다. 관측 event에는 ROS `source_stamp`/`source_frame_id`와 host `captured_at`이 별도 field로 보존되며 시간축 일치 상태는 미확인으로 표시됐습니다. event는 `DETECT_TARGETS`에 연결되고 task는 `PENDING`, `arm_command_sent: false`였습니다. 이 smoke에서는 FollowJointTrajectory goal과 관절 명령을 보내지 않았습니다.

선택형 `depth_image_proc` launch도 통합 확인했습니다. `/mujoco_omx/fixed_camera/points`에서 640×480 XYZ `PointCloud2`(point step 16 bytes, row step 10,240 bytes, 4,915,200 data bytes)를 수신했고 관제 API가 field layout, frame/stamp, freshness를 표시했습니다. 좌표 payload는 API에 포함하지 않습니다. 한 관측에서 RGB와 depth/CameraInfo stamp는 `1.202 s`였고 cloud stamp는 `1.002 s`여서 RGB 대비 `HEADER_MISMATCH`가 표시됐습니다. 이는 converter 결과가 최신 RGB보다 늦을 수 있음을 보여주며 이미지와 cloud 간 동기화 계약이 아직 없다는 구현 제한과 일치합니다. 해당 실행에서는 cloud가 `FRESH`였고 640×480 구조 검증은 통과했습니다. `depth_image_proc`는 기동 초기에 depth와 CameraInfo의 동기화 대기 경고도 출력했으므로 연속 동기화 성능은 검증 완료로 간주하지 않습니다.

이 header/dimension 필드 비교는 sample synchronization을 증명하지 않습니다. 어떤 action goal, service command, joint command도 보내지 않았고 실제 장치에 연결하지 않았습니다. launch와 관제 서버를 종료했습니다. `ros2_control` realtime scheduling 경고(권한 없음)와 종료 때 statistics publisher context 경고가 남아 있어 정상 운영 배포 검증은 아닙니다.

## Simulation-only launch

먼저 선택할 domain이 실물 로봇이나 다른 simulator와 분리됐는지 ROS graph와 운영 환경에서 확인합니다. 숫자 선택이나 wrapper의 입력 검사만으로 DDS 격리를 증명할 수는 없습니다.

```bash
source /opt/ros/jazzy/setup.bash
cd simulation/ros2_ws
ROS_DOMAIN_ID=15 ./run_mujoco_sim.sh
```

`15`는 예시입니다. wrapper는 명시적 domain `1`–`232`, Jazzy setup, workspace overlay 및 필요한 package index를 확인한 뒤 MuJoCo launch를 실행합니다. domain을 자동 변경하거나 격리 상태를 판정하지 않습니다. 이 호스트의 system prefix에는 MuJoCo ROS backend와 controller package가 없어 위 기본 명령만으로는 launch할 수 없습니다. runtime smoke는 필요한 `.deb`를 `/tmp` overlay로 추출해 `AMENT_PREFIX_PATH`와 `LD_LIBRARY_PATH`에 연결한 환경에서 수행했습니다. simulator wrapper는 그 임시 overlay를 자동 설정하지 않으며, 재현 가능한 overlay 준비 도구는 아직 없습니다.

`ROS_DOMAIN_ID=15 ./run_mujoco_sim.sh --point-cloud`는 표준 `depth_image_proc` XYZ node를 함께 실행하며 해당 package가 필요합니다. 기본 simulator는 이 optional node를 실행하지 않습니다. output에 subscriber가 없을 때 node input subscription도 나타나지 않는 lazy 경로를 runtime graph에서 관측했습니다. 관제 MuJoCo camera manager 또는 timing 조사 도구가 cloud topic을 구독하면 입력 처리가 시작됩니다. `ROS_DOMAIN_ID=15 ./run_mujoco_sim.sh --check-only --point-cloud`는 동일한 domain 형식, Jazzy/overlay 및 필수 package-index 선행 조건만 확인하고 ROS node를 실행하지 않습니다. 누락된 package가 있으면 모두 한 줄에 표시하고 종료 code `2`를 반환합니다. 통과 결과도 DDS graph 격리, plugin 로딩 또는 controller 활성화를 검증하지 않습니다.

RGB-D 도착 간격과 source header stamp 차이를 읽기 전용으로 조사하려면 simulator를 켠 별도 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
# 저장소 루트에서 실행하고, 15 대신 격리를 확인한 시뮬레이션 domain을 사용합니다.
ROS_DOMAIN_ID=15 python3 scripts/measure_mujoco_camera_sync.py --duration 60 \
  > /tmp/mujoco-camera-sync.json
```

`--point-cloud` 없이 실행하면 cloud 표본은 없어도 되며, RGB/depth/CameraInfo가 하나라도 관측되지 않으면 report를 출력한 뒤 종료 code `1`을 반환합니다. 구독은 관제 MuJoCo camera manager와 같은 reliable/volatile/keep-last 1 QoS(`QoSProfile(depth=1)`)를 사용합니다. XYZ·pixel payload를 읽지 않고 host 도착 간격과 ROS header의 최근접 stamp 차이만 요약합니다. 최근접 표본은 동기화 pair가 아니며, 허용 가능한 최대 skew를 추정하거나 판정하지 않습니다.

2026-10-31에 임시 ROS binary overlay와 격리된 domain `231`에서 관제와 같은 QoS로 cloud on/off를 각각 120초씩 두 번 관측했습니다. 네 run에서 RGB callback rate는 4.407–4.556 Hz였고 RGB host arrival p50은 약 200–201 ms, p95는 400–407 ms, 최댓값은 610–1,010 ms였습니다. Depth/CameraInfo는 4.980–4.997 Hz, cloud on PointCloud2는 4.981–4.982 Hz였습니다. 반복 측정에서도 cloud 유무에 따른 뚜렷한 RGB 전달률 차이는 보이지 않았고, cloud on의 RGB–PointCloud exact-stamp match는 1,076개 중 1,074개였습니다. RGB–Depth/RGB–CameraInfo는 cloud on/off의 각 run에서 받은 모든 RGB 표본이 일치했고 Depth–CameraInfo도 받은 표본에서 일치했습니다. 이는 구독자 callback과 header 수치의 단기 비교일 뿐 노출 시각 동기화, publisher 무손실, 장기 전달 보증, 물리 카메라를 증명하지 않습니다. 따라서 PointCloud 변환이 RGB 전달률 저하 원인이라는 증거는 현재 없지만, 이 표본만으로 인과를 확정하지 않습니다. 이전 sensor-best-effort QoS 값은 관제 reliable QoS와 다른 조건이라 대체했습니다. 설정 목표는 5 Hz이며 측정값은 publisher 발행률이나 장기 보증이 아닙니다.

## 관제에서 상태만 확인

시뮬레이션 launch가 별도 터미널에서 동작하고 동일 domain을 사용하도록 설정한 뒤 관제 서버를 read-only ROS 2 상태 모드로 실행할 수 있습니다.

```bash
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
CONTROL_STATION_MODE=mujoco \
CONTROL_STATION_MOBILE_DOMAIN=15 \
./run-control-station.sh
```

ROS가 제공하는 `rclpy`는 활성 Python interpreter에서 PyYAML을 import하므로, optional `requirements-ros.txt`를 프로젝트 가상환경에 설치합니다. 시스템 `dist-packages` 전체를 가상환경보다 앞에 두지 않아 OpenCV/Numpy 버전이 섞이지 않게 합니다. ROS setup과 workspace overlay는 관제 실행 전에 source해야 합니다.

```bash
.venv/bin/python -m pip install -r requirements-ros.txt
```

`mujoco` mode는 ROS domain 1–232를 명시하도록 요구하고, 현재 `omx_mujoco_sim` package의 joint-state topic과 trajectory action, fixed camera RGB/depth/CameraInfo topic만 고정 read-only endpoint로 사용합니다. 일반 ROS topic/action override 환경변수는 이 mode에서 적용하지 않습니다. 이 모드에서는 Pinky/Nav2 discovery를 하지 않고, `--arm`과 LeRobot 텔레오퍼레이션 및 OpenCV 실물 카메라를 비활성화합니다. 관제는 `JointState`를 관측하고 action server 발견 여부를 표시합니다. 카메라 RGB는 기존 MJPEG preview로 제공하고, depth/CameraInfo는 dimensions, encoding/calibration coefficients, source stamp/frame, 수신 freshness metadata로만 보여줍니다. optional point cloud가 활성화되면 XYZ field layout·dimensions·source frame/stamp·freshness도 보여주며 raw cloud와 좌표는 API에 반환하지 않습니다. 최신 RGB와의 source stamp/frame/dimensions 필드 비교 결과와 RGB source stamp 기준 camera→`mujoco_omx/link0` TF lookup 결과를 보여줍니다. TF는 simulation placeholder provenance이며 sample synchronization, depth 단위·유효 픽셀 의미, 보정 유효성 또는 object 3D pose를 확인하지 않습니다. action goal, service request, joint command는 보내지 않습니다. topic 수신이나 server 발견은 controller readiness, 안전 또는 Pick & Place 성공을 뜻하지 않습니다.

## 확인 범위와 제한

- `omx_mujoco_sim`은 Python task orchestrator나 실물 OMX adapter와 연결되지 않습니다. ROS RGB-D/CameraInfo publisher와 fixed camera TF는 임시 binary package overlay에서 runtime 확인했습니다. 관제 MuJoCo mode는 RGB를 MJPEG camera API/UI로 전달하고 depth/CameraInfo는 validation-limited read-only metadata로 노출합니다. RGB stamp 기준 simulation camera→base TF lookup provenance와 optional `depth_image_proc` XYZ cloud publisher가 추가됐지만 object 3D localization은 아직 없습니다. 관제는 PointCloud2 바이트 수와 field layout만 검사하며 XYZ를 역직렬화하거나 색상 mask와 연결하지 않습니다. Runtime에서 최신 RGB와 cloud stamp mismatch도 관측했습니다.
- Controller runtime smoke는 isolated domain과 임시 local overlay에서 완료했지만, base host에 ROS package를 설치한 실행, 장시간 안정성, shutdown 경고 분석, task orchestration 연동은 미완료입니다.
- MuJoCo controller의 position 단위는 시뮬레이션 관절 radian입니다. LeRobot normalized action과의 변환은 확인되지 않았습니다.
- MJCF 선반·물체 치수와 wrist/fixed camera pose는 placeholder입니다. simulation 좌표나 렌더 결과를 실물 캘리브레이션 값으로 사용하지 않습니다.
- 관제 상태 화면은 지정 topic과 action server의 관측만 보여 줍니다. 자동 작업 실행·취소·결과 경로는 구현되어 있지 않습니다.
- 실제 장치 포트나 hardware launch를 이 절차에 추가하지 않습니다. 실물 연결은 별도 장치 계약, 안전 확인 및 승인된 현장 절차가 필요합니다.

현재 호스트의 package 설치 상태와 마지막 정적 확인은 [구현 현황](current-state.md) 및 [개발 로드맵](../development-plan.md)에 기록합니다.
