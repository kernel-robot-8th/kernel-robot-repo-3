# Pinky 주차 판정 C++ / ROS 2 워크스페이스

천장 카메라의 ArUco 마커로 Pinky가 등록된 주차 위치에 섰는지 판정하는 **유일한 구현**입니다. 관제 서버에 있던 Python 판정(`parking_check`, `parking_observation`, `parking_tracker`)과 등록 CLI(`scripts/pinky_parking.py`)를 C++17로 옮기고 ROS 2 Jazzy 노드로 감쌌으며, 2026-11-03에 Python 쪽은 지웠습니다(커밋 `8ea4c84`가 마지막 Python 버전).

**관측만 합니다.** Pinky나 로봇팔에 명령을 보내지 않고, 천장 카메라는 읽기 전용으로 엽니다. 결과의 `ARRIVED`는 마커가 등록 위치의 허용 범위 안에 있다는 관측일 뿐이며, Nav2 이동 성공을 대신하지 않습니다.

## 현재 상태 (2026-11-03)

[전환 계획](../docs/plans/pinky-parking-ros2-node.md)의 순서 1~5에 해당합니다.

| 항목 | 상태 |
|---|---|
| 판정 로직 C++ 이식 | 완료, Python 판정 삭제. 옮기기 전 Python 결과로 만든 기대값과 비교하는 테스트 통과 |
| 등록·판정 CLI `pinky_parking` | 완료, Python CLI 삭제. 등록 결과가 Python과 같은지 비교, 비디오 파일로 register→check 확인 |
| ROS 2 컴포넌트 | 합성 영상 토픽으로 end-to-end 테스트 통과 (localhost 한정). 단일 스레드 `component_container`에서도 서비스 응답 확인 |
| 카메라 캡처·실물 천장 카메라 (순서 3) | 자체 캡처 `CeilingCameraNode` + 판정 컴포넌트를 한 컨테이너에서 실물 확인(`ceiling_parking.launch.py`). USB가 끊겨도 2초마다 다시 엶. 96 px 영상 밀림 재현 여부는 **판단하지 못함** |
| rosbag 검증 | **하지 않음** |
| 관제 서버(`app.py`)·웹 연결 (순서 5) | `CONTROL_STATION_PARKING_SPOT`과 `CONTROL_STATION_HEAD_CAMERA=ros:<토픽>`으로 연결. 관제에는 주차 판정이 없습니다. 2026-11-03부터 현장 관제가 이 구성으로 동작. 실행 방법은 [runbook](../docs/implementation/runbook.md#c-주차-판정-노드로-관제-실행) |
| Python 판정과 동시 비교 (검증 3) | 같은 영상 30초, 60/60 같은 상태, 차이 dx 1.0 mm·dy 1.3 mm·yaw 0.15°(중앙값) |

## 패키지

| 패키지 | 내용 |
|---|---|
| `parking_vision` | ROS 의존성 없는 C++ 라이브러리 (ament_cmake). 검출과 평면 투영 판정·고정 마커 시야 확인(`parking_check`), 요청 이후 프레임 수집(`ParkingObservationCollector`), 화면 표시용 상태 기계(`ParkingTracker`), 주차 위치 파일 재로딩(`SpotReloader`), 등록(`build_registration`), `spot_id` 규칙. 스레드, 시계, 카메라 feed는 호출하는 쪽이 맡습니다. 등록·판정 CLI `pinky_parking`(`ros2 run parking_vision pinky_parking`)도 이 패키지에 있습니다. |
| `parking_vision_interfaces` | `ParkingState.msg`, `CheckParking.srv`. 필드는 관제 JSON 계약(`/live`, `/check`의 `judgment_fields`·`image_overlay`)과 1:1입니다. JSON에서 `null`인 값은 `has_*` 플래그로 표시합니다. |
| `parking_vision_ros` | `rclcpp_components` 컴포넌트 `parking_vision_ros::ParkingVisionNode`. 단독 실행 파일 `parking_vision_node`로도 쓸 수 있습니다. |

### 노드 인터페이스 (노드 이름 `pinky_parking`)

| 이름 | 종류 | 내용 |
|---|---|---|
| `image_topic` 파라미터 (기본 `image_raw`) | 구독 `sensor_msgs/Image` | 천장 카메라 영상. `bgr8`, `rgb8`, `mono8` 등 cv_bridge가 변환할 수 있는 형식 |
| `/pinky_parking/state` | 발행 `ParkingState` | `spot_id` 파라미터로 지정한 주차 위치를 10Hz로 추적한 화면 표시용 상태입니다. QoS는 reliable, keep-last 1, transient_local이라 나중에 붙은 구독자도 마지막 상태를 바로 받습니다. 팔 단계 진행 조건으로 쓰지 않습니다. |
| `/pinky_parking/check` | 서비스 `CheckParking` | 요청한 `spot_id`에 대해, 요청 뒤에 찍힌 프레임만 최대 3초, 15프레임 모아서 판정합니다. 5프레임 미만이면 `NOT_FOUND`입니다. |

그 밖의 파라미터는 `repository_root`(기본 `.`)와 `feed_camera_id`(기본은 구독 토픽 이름)입니다. 주차 위치 파일은 `<repository_root>/.control-station/parking-spots/<spot_id>.json`에서 읽고, 요청마다 다시 읽습니다. 파일 안의 내부 보정 경로도 `repository_root` 기준입니다.

**결과 해석**

- `ok=true`는 판정을 마쳤다는 뜻입니다. Pinky가 도착했는지는 `status`(`ARRIVED`, `MISALIGNED`, `NOT_FOUND`, `AMBIGUOUS`, `VIEW_UNVERIFIED`)로 확인합니다.
- `ok=false`이면 판정을 시작하지 못했거나 끝내지 못한 것이고, `error_code`는 Python API와 같습니다.
  - `PARKING_SPOT_ID_INVALID`: `spot_id` 형식이 틀림
  - `PARKING_SPOT_NOT_FOUND`: 주차 위치 파일이 없음
  - `PARKING_SPOT_INVALID`: schema가 맞지 않거나, 등록 이후 내부 보정 파일이 바뀜
  - `PARKING_CAMERA_UNAVAILABLE`: 영상 토픽 발행자가 없거나, 3초 안에 새 프레임이 없거나, 해상도가 보정값과 다름
  - `PARKING_OBSERVATION_ERROR`: 그 밖의 처리 오류
- 응답에는 주차 위치 파일과 내부 보정 파일의 SHA-256, 판정 때와 등록 때의 OpenCV 버전이 함께 들어갑니다.
- 서비스는 deferred response로 별도 스레드에서 응답합니다. 프레임을 모으는 동안에도 executor가 막히지 않아서, 단일 스레드 컨테이너에서도 영상 구독이 계속 돌아갑니다.

**프레임 촬영 시각 판단**

"요청 이후에 찍힌 프레임"인지는 이미지 header stamp와 노드가 프레임을 받은 시각 중 **더 이른 쪽**을 기준으로 판단합니다. 카메라 드라이버를 다른 호스트에서 띄운다면 두 호스트의 시계가 동기화돼 있어야 합니다.

## 빌드, 테스트, 실행

```bash
cd cpp
source /opt/ros/jazzy/setup.bash
# 테스트 토픽과 서비스가 같은 네트워크의 로봇에 닿지 않게 discovery를 이 호스트로 제한한다.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=87
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
```

`parking_vision_ros`의 테스트는 CMake에서 직접 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_DOMAIN_ID=87`을 지정해서 돌립니다.

실행은 저장소 루트에서 합니다.

```bash
source cpp/install/setup.bash
ros2 launch parking_vision_ros parking_vision.launch.py \
  repository_root:=. spot_id:=pinky_dock_1 image_topic:=/ceiling_camera/image_raw
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /pinky_parking/state
ros2 service call /pinky_parking/check parking_vision_interfaces/srv/CheckParking "{spot_id: pinky_dock_1}"
```

천장 카메라 캡처(`CeilingCameraNode`)와 판정 노드를 한 컨테이너에 올리는 launch입니다. 영상이 프로세스 안에서 전달되고, 관제용 JPEG(`/ceiling_camera/image_raw/compressed`)도 함께 나갑니다. 관제가 같은 카메라를 V4L2로 열고 있으면 먼저 꺼야 합니다.

```bash
ros2 launch parking_vision_ros ceiling_parking.launch.py repository_root:=. spot_id:=pinky_dock_1
```

**캡처를 `usb_cam` 대신 직접 만든 이유 (2026-11-03)**

처음에는 표준 드라이버 `usb_cam` 0.8.1을 썼습니다. 그런데 현장에서 헤드 카메라 USB가 2분 사이에 두 번 끊겼다 다시 잡히자 다음 문제가 생겼습니다.

- 깨진 MJPEG 프레임에서 예외로 컨테이너 전체가 종료되어, 판정 노드까지 함께 꺼졌습니다.
- 카메라가 다른 `/dev/videoN`으로 다시 잡히면 따라가지 못했습니다.
- 상대 경로 by-path 링크를 `/dev/../../videoN`으로 잘못 풀었습니다.
- brightness를 지정하지 않으면 카메라에 50을 써서 남겼습니다.

`CeilingCameraNode`는 관제가 오래 쓰던 캡처 방식(OpenCV V4L2, MJPG 640×480)을 그대로 옮긴 것입니다.

- 읽기가 실패하면 장치를 닫고, 2초마다 by-path를 다시 풀어서 다시 엽니다. 캡처 스레드에서 예외가 밖으로 나가지 않습니다.
- 카메라 설정값은 바꾸지 않습니다.
- 카메라가 빠진 동안에도 판정 노드는 살아 있고, `/pinky_parking/state`는 `CAMERA_UNAVAILABLE`("새 프레임이 들어오지 않습니다")을 냅니다.

비디오 파일로 카메라를 대신하는 테스트(`test_ceiling_camera_node.cpp`)에서 다음을 확인합니다. 영상이 사라졌다가 돌아오는 경우와, 시작할 때 장치가 없는 경우입니다. 실물에서도 확인했습니다. 확정 판정은 `MISALIGNED` 15프레임이었고, 웹 영상은 초당 약 29장이었으며, 카메라 설정은 바뀌지 않았습니다.

같은 프레임에서 C++과 Python(OpenCV 4.6)은 소수 6자리까지 같았습니다. 관제(OpenCV 4.14) 결과와의 약 1 mm, 0.2° 차이는 OpenCV 버전 차이입니다(2026-11-02 드라이버 프레임 5장 비교).

판정 노드만 기존 컨테이너에 올릴 때는 이렇게 합니다.

```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager parking_vision_ros parking_vision_ros::ParkingVisionNode \
  -p repository_root:=. -p spot_id:=pinky_dock_1 -p image_topic:=/ceiling_camera/image_raw
```

이 노드는 카메라 드라이버를 포함하지 않습니다. 천장 카메라는 관제 서버의 Head Camera와 같은 실물 장치(HCAM0)라서, 관제 서버와 카메라 드라이버 노드가 동시에 열 수 없습니다.

**프로세스 사이 영상 전달 주의 (2026-11-02)**

같은 PC의 다른 프로세스에서 640×480 raw `bgr8` 영상을 best-effort로 보냈더니, Fast DDS 기본 설정에서 대부분 전달되지 않았습니다(`ros2 topic hz`는 7초간 0개, 판정 노드는 3초 동안 8프레임). 드라이버와 판정 노드를 한 컨테이너에 두는 것을 기본으로 하고, 프로세스를 나눈다면 전송 설정을 먼저 확인합니다.

## 테스트

| 테스트 | 확인하는 것 |
|---|---|
| `parking_vision/test/test_parking_check.cpp`, `test_parking_tracker.cpp` | 지운 Python 단위 테스트의 케이스를 C++로 옮긴 것과 `SpotReloader` |
| `parking_vision/test/test_pinky_parking_cli.cpp` | 등록 결과가 Python CLI와 같은지(정상, 고정 마커 지정, 시야 확인 끔, 단일 마커 신뢰, 움직임 거부, 흔들린 고정 마커), 비디오 파일을 카메라 삼아 CLI register→check(종료 코드 0/1/2, 덮어쓰기 거부) |
| `parking_vision/test/test_python_parity.cpp` | 같은 입력에서 C++ 결과가 옮기기 전 Python 결과와 같은지. 왜곡 렌즈의 평면 투영과 자세 추정, 프레임별 판정(고정 마커 하나만 밀림, 단일 마커 신뢰 포함), `ParkingJudge` 순서, 요청 이후 수집 7개 시나리오, 추적기 123틱 |
| `parking_vision_ros/test/test_parking_vision_node.cpp` | 영상 토픽 → `/pinky_parking/state`와 `/pinky_parking/check`. 정위치, 늦게 붙은 구독자의 마지막 상태 수신, 96px 밀림, 카메라 없음, 어긋남, 요청 전 stamp 프레임 무시, `spot_id` 오류·없음·보정 변경, 잘못된 추적 주차 위치 |

기대값(`test/fixtures/`)은 Python 판정을 지우기 직전(커밋 `8ea4c84`의 Python 모듈, 시스템 OpenCV 4.6)에 그 모듈을 실제로 돌려서 만든 값이며, 이제 **고정된 회귀 기준**입니다. 생성 스크립트는 Python과 함께 지웠습니다. 판정 규칙을 의도적으로 바꾸면 기대값도 C++ 쪽에서 근거를 남기고 고쳐야 합니다.

- **같은 OpenCV 버전**: 1e-9 이내로 비교하고 reason 문자열도 정확히 비교합니다.
- **다른 버전으로 만든 기대값**(`PARKING_VISION_FIXTURES=<dir>`): 0.2px, 0.3mm, 0.2° 이내로 비교합니다. 2026-11-02에 OpenCV 4.14로 만든 기대값과 비교했을 때 차이는 최대 0.12px, 0.2mm, 0.16°였습니다.

## 성능 측정 (2026-11-02)

같은 주차 위치와 같은 프레임으로 `observe_frame` 한 번에 걸리는 시간을 쟀습니다. 측정 조건은 Intel Core Ultra 9 275HX, OpenCV 스레드 1개, 500회 반복의 중앙값입니다.

| 프레임 | C++ (OpenCV 4.6) | Python (OpenCV 4.6) | Python (OpenCV 4.14) |
|---|---|---|---|
| 렌더링, 등록 위치 | 1.02 ms | 1.13 ms | 1.32 ms |
| 블러와 노이즈, 등록 위치 | 2.86 ms | 2.97 ms | 1.78 ms |
| 렌더링, 26 mm 이동 | 0.99 ms | 1.14 ms | 1.03 ms |
| 96 px 밀림 (판정 거부) | 1.01 ms | 1.05 ms | 0.96 ms |

같은 OpenCV 버전에서 C++은 프레임당 약 0.1ms(4~13%) 빠를 뿐입니다. 시간 대부분이 양쪽에서 똑같이 쓰는 OpenCV C++ 코드(ArUco 검출과 subpixel 보정)에서 쓰이기 때문입니다. 노이즈 낀 프레임에서는 오히려 OpenCV 버전 차이(4.6 → 4.14에서 1.6배)가 언어 차이보다 컸습니다. 10Hz 추적이나 30fps 영상에 비하면 어느 쪽이든 여유가 충분합니다.

C++로 옮겨서 얻는 이점은 이 측정에 나타나는 판정 속도가 아닙니다. 판정 로직을 ROS 2 컴포넌트로 분리해서, 관제 서버 없이도 토픽과 서비스로 쓸 수 있게 한 점입니다. 이미지 전송과 Python 관제 서버 쪽 프레임 복사 비용은 측정하지 않았습니다.

```bash
cpp/build/parking_vision/parking_vision_benchmark cpp/parking_vision/test/fixtures 500 1
```

Python 측정 스크립트는 Python 판정과 함께 지웠습니다. 위 표의 Python 값은 2026-11-02 측정 기록입니다.
