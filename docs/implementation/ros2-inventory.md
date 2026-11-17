# ROS 2 인터페이스 목록

저장소 코드가 제공하거나 쓰는 ROS 2(Jazzy) 인터페이스입니다. 코드에서 확인한 사실만 적었고, 실행 중인 graph를 관측한 기록은 아닙니다. 관제가 ROS로 로봇에 명령을 보내는 곳은 집기 액션 `/arm_0N/pick_object` 하나이며, `CONTROL_STATION_ORDER_REAL_ARM=1`과 `CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2`를 함께 켰을 때만 씁니다.

## 패키지 (`cpp/` colcon 워크스페이스)

| 패키지 | 내용 |
|---|---|
| `parking_vision` | ROS 의존 없는 주차 판정 C++ 라이브러리와 등록·판정 CLI `pinky_parking` |
| `parking_vision_interfaces` | `msg/ParkingState.msg`, `srv/CheckParking.srv` |
| `parking_vision_ros` | 컴포넌트 `ParkingVisionNode`(판정)와 `CeilingCameraNode`(천장 카메라 캡처), launch `ceiling_parking.launch.py`·`parking_vision.launch.py` |
| `omx_interfaces` | `action/MoveToSlot.action`(칸 앞 자세), `action/PickObject.action`(학습 정책으로 집기) |

빌드·테스트와 노드 파라미터는 [cpp/README.md](../../cpp/README.md)에 있습니다.

## 토픽·서비스·액션

| 이름 | 타입 | 제공 | 쓰는 곳 |
|---|---|---|---|
| `/ceiling_camera/image_raw` | `sensor_msgs/msg/Image` | `CeilingCameraNode` (`ceiling_parking.launch.py`) | 판정 노드 (같은 컨테이너, 프로세스 내부 전달) |
| `/ceiling_camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 같음 | 관제 Head Camera (`CONTROL_STATION_HEAD_CAMERA=ros:/ceiling_camera/image_raw/compressed`) |
| `/pinky_parking/state` | `parking_vision_interfaces/msg/ParkingState` (reliable, keep-last 1, transient_local) | `ParkingVisionNode` | 관제 `parking_ros.py` → `GET /api/parking-spots/{spot_id}/live` |
| `/pinky_parking/check` | `parking_vision_interfaces/srv/CheckParking` | `ParkingVisionNode` | 관제 `parking_ros.py` → `/check`, `VERIFY_ARRIVAL`, 주문 `WAIT_PINKY_ARRIVAL` |
| `/omx/move_to_slot` | `omx_interfaces/action/MoveToSlot` | `scripts/omx_ros_worker.py` | `scripts/omx_move_to_slot_client.py`, `ros2 action send_goal` ([실습](omx-ros-practice.md)). 관제에는 이 action 클라이언트가 없음 |
| `/arm_01/pick_object`, `/arm_02/pick_object` | `omx_interfaces/action/PickObject` | `scripts/omx_pick_action_server.py --robot-id arm-0N` (팔마다 하나) | 관제 `arm_action_ros.py` → 주문 `PICK_ITEM` (`CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2`일 때). [설명](arm-command-api.md#ros-2-집기-액션-arm_0npick_object) |
| `navigate_to_pose` (상대 이름) | `nav2_msgs/action/NavigateToPose` | 외부 Nav2 | `CONTROL_STATION_MODE=ros2`일 때 `/api/robots`가 server 발견 여부만 봄. goal을 보내지 않음 |

## 관제가 ROS에 붙는 방식

- 관제(Python, `rclpy`)는 ROS를 쓰는 기능마다 자기 context를 엽니다. domain은 `CONTROL_STATION_MOBILE_DOMAIN`, 없으면 `ROS_DOMAIN_ID`입니다.
  - 주차 판정(`CONTROL_STATION_PARKING_SPOT`)과 `ros:<토픽>` 카메라(`CONTROL_STATION_*_CAMERA=ros:...`, `Image` 또는 `CompressedImage`)는 domain이 없으면 시작하지 않습니다.
  - Nav2 발견(`CONTROL_STATION_MODE=ros2`)은 domain이 없으면 0을 씁니다.
- 현장에서는 주차 노드와 관제를 domain 87, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 띄웁니다([실행 방법](runbook.md#c-주차-판정-노드로-관제-실행)). 노드 테스트는 domain 97·98을 씁니다.
- 핑키 세 대는 로봇마다 ROS domain 10·11·12를 쓰고, 핑키 팀 관제(`pinky_fleet_dashboard`)가 주행과 임무를 맡습니다. 관제는 핑키 ROS graph에 붙지 않고, 핑키 관제 HTTP로 단계를 읽고 신호를 보냅니다([주문 흐름](order-flow.md#핑키-관제-연결-http)).
- OMX 팔은 기본으로 LeRobot 워커가 관제의 HTTP 명령 큐에서 명령을 가져갑니다([OMX 명령 HTTP 인터페이스](arm-command-api.md)). `CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2`면 주문의 집기를 팔마다 띄운 `PickObject` 액션 서버로 보냅니다. domain은 `CONTROL_STATION_ARM_ACTION_DOMAIN`, 없으면 `ROS_DOMAIN_ID`이며, 어느 쪽이든 같은 팔 명령 기록에 남습니다.

## 시뮬레이션 (`simulation/ros2_ws`)

`toktak_warehouse`(ament_cmake)는 Gazebo Harmonic 창고 world, 이동 플랫폼 SDF, 지도, `ros_gz_bridge` 설정을 설치합니다. `config/bridge.yaml`이 선언한 토픽은 `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/scan`, `/imu/data`, `/joint_states`이고, `simulation/run.sh`는 `ROS_DOMAIN_ID` 기본값 31로 띄웁니다. 이 workspace에는 Nav2 bringup이 없고, 관제에도 이 시뮬레이터 전용 연결은 없습니다.
