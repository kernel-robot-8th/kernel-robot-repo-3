# Pinky 주차 판정 C++ ROS 2 노드 전환 계획

천장 카메라 Pinky 주차 판정을 C++ ROS 2 노드로 옮기고, 관제는 그 결과를 토픽·서비스로 받아 지금 웹 화면과 작업 흐름을 그대로 유지하는 계획입니다. 판정 규칙과 결과 필드는 [주차 정렬 관측 계약](../spec/interface-contracts.md#pinky-주차-정렬-관측-초안)과 [실행 방법](../implementation/runbook.md#천장-카메라-pinky-주차-판정)을 따릅니다.

## 현재 상태 (2026-11-03)

- 순서 1~5를 구현했습니다. `CONTROL_STATION_PARKING_SOURCE=ros`로 띄운 관제는 `/live`, `/check`, `VERIFY_ARRIVAL`을 C++ 노드(`/pinky_parking/state`, `/pinky_parking/check`)로 답하고, `CONTROL_STATION_HEAD_CAMERA=ros:/ceiling_camera/image_raw/compressed`로 Head Camera 영상과 YOLO 표시를 드라이버 토픽에서 받습니다. 설정하지 않으면 예전처럼 관제 안의 Python 판정(`parking_observation.py`, `parking_tracker.py`)이 Head Camera를 직접 엽니다.
- 2026-11-03 19:5x부터 현장 관제가 이 구성으로 돌고 있습니다(드라이버 컨테이너 `ceiling_parking.launch.py`, ROS는 localhost·domain 87). 선반 카메라는 여전히 관제가 OpenCV로 엽니다.
- 검증 1·3을 했습니다. 3은 같은 영상을 Python 판정과 C++ 판정에 동시에 넣어 30초간 0.5초마다 비교했고, 60회 모두 같은 상태(`OFF_POSITION`)였으며 차이는 dx 중앙값 1.0 mm(최대 1.4), dy 1.3 mm(최대 1.7), yaw 0.15°(최대 0.3)로 OpenCV 4.14와 4.6의 차이 수준이었습니다. 합성 영상으로 `VERIFY_ARRIVAL`이 `ARRIVED`일 때만 팔 단계로 넘어가고, `MISALIGNED`와 노드 정지(`PARKING_NODE_UNAVAILABLE`)일 때 팔 단계가 취소되는 것도 확인했습니다. 검증 2(rosbag)와 4(실물 위치 시험), 순서 6(YOLO 노드 분리)은 아직입니다. 자세한 내용은 [cpp/README.md](../../cpp/README.md)에 있습니다.

## 목표 구조

```
천장 카메라 ─→ [카메라 드라이버 노드] ─→ /ceiling_camera/image_raw (+ /compressed), /ceiling_camera/camera_info
                         │  같은 컴포넌트 컨테이너, 프로세스 내부 전달
                         ├─→ [C++ 주차 판정 노드]  ─→ /pinky_parking/state (실시간)
                         │                          └─ /pinky_parking/check (확정 판정 서비스)
                         ├─→ [YOLO 노드 (2단계)]   ─→ /ceiling_camera/detections (vision_msgs/Detection2DArray)
                         └─→ [Python 관제 (rclpy)] ─→ 웹: 영상(MJPEG) + YOLO 박스 + 주차 패널, /live·/check API 그대로
```

카메라는 한 프로세스만 열 수 있습니다. 드라이버 노드 하나가 카메라를 열고, 판정 노드·YOLO·관제는 같은 영상 토픽을 구독합니다. 관제는 더 이상 Head Camera 장치를 직접 열지 않습니다. 선반 카메라도 같은 방식(`/shelf_camera/...`)으로 옮길 수 있지만, 이 계획의 범위는 천장 카메라입니다.

## 패키지

`cpp/`를 colcon 작업 공간의 소스 루트로 씁니다.

| 패키지 | 내용 |
|---|---|
| `cpp/parking_vision` | 지금의 판정 라이브러리(ROS 의존 없음). ament_cmake로도 빌드되게 맞춤 |
| `cpp/parking_vision_interfaces` | `ParkingState.msg`, `CheckParking.srv` |
| `cpp/parking_vision_ros` | `rclcpp_components` 컴포넌트: 영상 구독 → 판정 → 상태 발행, 확정 판정 서비스. 실행 launch와 파라미터 |

## 인터페이스

필드는 지금 JSON 계약과 1:1로 맞춥니다. 그래야 관제 API 응답과 웹 화면을 바꾸지 않습니다.

`ParkingState.msg` (`/pinky_parking/state`)

| 필드 | 내용 |
|---|---|
| `std_msgs/Header header` | 판정에 쓴 마지막 프레임의 stamp와 `frame_id` |
| `string spot_id`, `int32 marker_id` | 주차 위치와 Pinky 마커 |
| `string state` | `NOT_IN_VIEW`, `MOVING`, `IN_POSITION`, `OFF_POSITION`, `AMBIGUOUS`, `VIEW_UNVERIFIED`, `CAMERA_UNAVAILABLE`, `SPOT_INVALID` |
| `string reason`, `builtin_interfaces/Time since` | 사유와 그 상태가 시작된 시각 |
| `bool has_offset`, `float64 dx_mm, dy_mm, distance_mm, dyaw_deg, forward_mm, left_mm` | 등록 위치 대비 차이(마커 축, Pinky 축) |
| `float64 position_ratio, yaw_ratio`, `string deviation_level` | 허용 오차 대비 배수, `WITHIN`/`NEAR`/`FAR` |
| `bool view_check`, `float64 view_shift_x_px, view_shift_y_px` | 고정 마커 시야 확인과 카메라 흐름 |
| `float64[8] registered_corners_px`, `float64[8] marker_corners_px`, `bool has_marker_corners` | 웹에서 영상 위에 그릴 좌표 |
| `bool display_only` | 항상 true. 팔 진행 판단에 쓰지 않음 |

QoS는 reliable, keep-last 1, transient_local입니다. 관제가 나중에 붙어도 마지막 상태를 바로 받습니다.

`CheckParking.srv` (`/pinky_parking/check`)

- 요청: `spot_id`
- 응답: `bool ok`, `string error_code`(`PARKING_SPOT_ID_INVALID`, `PARKING_SPOT_NOT_FOUND`, `PARKING_SPOT_INVALID`, `PARKING_CAMERA_UNAVAILABLE`, `PARKING_OBSERVATION_ERROR`), 확정 판정 결과(`status`, 오프셋, 등급, `frames_used`, `frames_examined`, 프레임 stamp 범위, 등록·보정 SHA-256, OpenCV 버전)
- 요청을 받은 뒤의 프레임만 최대 3초·15프레임 모읍니다. 5프레임 미만이면 `NOT_FOUND`입니다. 0.5초 안팎으로 끝나므로 액션 대신 서비스로 둡니다.

등록 파일(`.control-station/parking-spots/*.json`)과 내부 보정 파일은 C++ 노드도 같은 JSON을 읽습니다. 설정 원본은 하나로 유지합니다.

## 카메라 드라이버

- 먼저 표준 드라이버(`usb_cam` 0.8.1 또는 `v4l2_camera` 0.7.3, 2026-11-02 설치 확인)가 HCAM0을 MJPG 640×480 30 fps로 안정적으로 내는지 확인합니다. 두 카메라가 USB 2.0 허브를 같이 써서 비압축 YUYV는 대역폭이 모자랄 수 있습니다.
- 표준 드라이버로 안 되면, 지금 검증된 설정(OpenCV V4L2, MJPG 640×480)으로 캡처하는 작은 C++ 컴포넌트를 둡니다.
- 2026-11-03 결정: 아래 `usb_cam` 문제에 더해 헤드 카메라 USB가 끊겼다 다시 잡힐 때 `usb_cam`이 깨진 프레임에서 컨테이너째 종료되고 새 `/dev/videoN`을 따라가지 못해, 관제의 검증된 캡처 방식(OpenCV V4L2, MJPG)을 옮긴 `CeilingCameraNode`를 만들어 대신 씁니다. 실패하면 2초마다 by-path를 다시 풀어 다시 열고, 카메라 설정은 바꾸지 않습니다.
- 2026-11-02 22시 확인: `usb_cam`(`mjpeg2rgb`, 640×480, 30 fps)과 판정 컴포넌트를 한 컨테이너에 올려 프로세스 내부로 전달하니 확정 판정이 0.5초 안팎에 15프레임으로 끝났습니다(`cpp/parking_vision_ros/launch/ceiling_parking.launch.py`). 같은 드라이버 프레임에서 C++ 판정은 Python(OpenCV 4.6)과 소수 6자리까지 같았고, 관제(OpenCV 4.14) 결과와의 약 1 mm·0.2° 차이는 OpenCV 버전 차이였습니다. `usb_cam` 0.8.1은 상대 경로 by-path 링크를 `/dev/../../videoN`으로 잘못 풀고(launch에서 실제 장치로 변환), brightness를 지정하지 않으면 카메라에 50을 써서 남기며(작은 고정 마커가 안 보이게 됨, `brightness: -1`로 회피, 값은 0으로 되돌림), 컴포넌트를 내려도 장치를 놓지 않습니다(컨테이너를 다시 시작).
- 장치는 by-path로 지정합니다. HCAM0 두 대가 같은 by-id를 씁니다.
- 판정 노드와 드라이버를 한 컴포넌트 컨테이너에 올려 프로세스 내부 전달로 영상 복사를 없앱니다. 관제가 다른 PC에 있으면 웹 표시는 `/compressed` 토픽을 씁니다.

## YOLO와 관제 화면

Head Camera를 관제에서 계속 보면서 YOLO 박스도 보이게 합니다.

1. **1단계 (작은 변경)**: 관제가 영상 토픽을 구독하는 실물 카메라 매니저를 새로 둡니다. MuJoCo 모드의 `Ros2SimulationCameraStreamManager`를 본뜹니다. YOLO는 지금처럼 관제 안의 `_YoloDetector`가 그 영상에 돌립니다. 웹 화면은 바뀌지 않습니다.
2. **2단계 (분리)**: YOLO를 별도 노드로 옮겨 `vision_msgs/Detection2DArray`를 발행합니다. 관제는 박스만 받아 주차 패널처럼 웹에서 영상 위에 그립니다. 나중에 C++ 추론(ONNX Runtime 또는 TensorRT)으로 바꿀 수 있습니다.

## 관제 변경

- `CONTROL_STATION_PARKING_SOURCE=ros`이면 `/live`는 `/pinky_parking/state` 구독 값으로, `/check`와 `VERIFY_ARRIVAL`은 `/pinky_parking/check` 서비스 호출로 답합니다. 응답 JSON 형식은 지금과 같습니다. 설정하지 않으면 지금처럼 프로세스 안에서 판정합니다.
- 서비스가 응답하지 않거나 timeout이면 판정 불가로 처리하고 팔 단계를 시작하지 않습니다. 자동 재시도는 하지 않습니다. 구현에서는 연결 실패를 `PARKING_NODE_UNAVAILABLE`(503), 6초 timeout을 `PARKING_NODE_TIMEOUT`(503)으로 돌려주고, `/live`는 노드 상태가 없거나 2초 넘게 갱신되지 않으면 마지막 상태 대신 `NODE_UNAVAILABLE`을 보여 줍니다. ROS 판정 결과에는 `judgment_source`(`ros:/pinky_parking/state` 또는 `ros:/pinky_parking/check`)가 붙습니다.
- mock 모드에서도 이 설정이면 rclpy를 초기화합니다. ROS domain은 `CONTROL_STATION_MOBILE_DOMAIN` 규칙을 따릅니다.

## 검증

1. 라이브러리: 지금의 Python 비교 테스트를 유지합니다(같은 OpenCV 4.6에서 1e-9 이내).
2. 노드: 오늘 저장한 실제 프레임(정상, 96 px 영상 밀림, 5 px 거치대 흐름, 마커 가림)을 rosbag으로 재생해 상태 전환과 확정 판정을 Python 결과와 비교합니다.
3. 통합: 같은 시간에 Python 판정(`/live`)과 ROS 판정을 함께 띄워 30초간 상태·오프셋을 비교한 뒤 전환합니다.
4. 실물: Pinky를 정답 위치, 앞으로 5 cm, 옆으로 2 cm, 10° 돌린 상태로 두고 화면·판정을 확인합니다.

## 순서

1. `parking_vision_interfaces` 정의와 빌드
2. `parking_vision`을 ament_cmake 패키지로 맞춤
3. 카메라 드라이버 확인(표준 드라이버 또는 자체 캡처 컴포넌트)
4. `parking_vision_ros` 컴포넌트(상태 발행, 확정 판정 서비스)와 rosbag 검증
5. 관제의 ROS 카메라 매니저와 `CONTROL_STATION_PARKING_SOURCE=ros`
6. YOLO 노드 분리(2단계)

## 남은 확인 사항

- 이 카메라의 96 px 영상 밀림이 `usb_cam`에서도 나는지. 2026-11-02 22:30~22:40 드라이버를 다시 여는 시험 중 천장 카메라 영상이 검게 나오다 선반 정면 화면으로 바뀌어(카메라를 옮기거나 가린 것으로 보임) 판단하지 못했습니다.
- 2026-11-02 같은 PC의 다른 프로세스로 640×480 raw `bgr8` 영상을 best-effort로 보냈을 때 Fast DDS 기본 설정에서 대부분 전달되지 않았습니다(`ros2 topic hz` 7초간 0개, 판정 노드는 3초에 8프레임). 드라이버와 판정 노드를 한 컨테이너에 두거나, 프로세스를 나눈다면 전송 설정을 먼저 확인합니다.
- 관제와 노드를 같은 PC에서 돌릴지, 다른 PC면 압축 영상 대역폭
- ROS domain ID와 Pinky·MuJoCo graph와의 분리
- 선반 카메라도 같은 구조로 옮길 시점
