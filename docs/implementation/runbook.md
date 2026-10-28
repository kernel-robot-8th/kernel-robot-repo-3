# 실행 방법

## 대시보드와 API만 실행

Python 3.10 이상 환경에서 저장소 의존성을 설치한 뒤 실행합니다. 기본 모드에는 별도 LeRobot 폴더가 필요하지 않습니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
./run-control-station.sh
```

브라우저에서 <http://127.0.0.1:8000>을 엽니다. 기본 모드에서는 로봇 제어를 시작하지 않습니다. 서버를 종료하려면 실행 중인 터미널에서 `Ctrl+C`를 누릅니다.

런처로 시작할 때마다 `.control-station/logs/session-<시각>-<프로세스ID>.log`에 관제 서버 출력과 HTTP 요청 로그를 저장하고, 터미널에도 함께 표시합니다. `--arm` 실행의 LeRobot 출력은 같은 세션 ID를 가진 `arm-<시각>-<프로세스ID>.log`에 따로 저장합니다. 이 폴더는 Git에서 제외됩니다.

## 팔 텔레오퍼레이션 제어 켜기

먼저 `omx_follower`와 `omx_leader` 지원 코드가 포함된 LeRobot 환경을 활성화한 뒤 실행합니다.

```bash
source /경로/lerobot/venv/bin/activate
./run-control-station.sh --arm
```

또는 실행 파일 경로를 직접 지정할 수 있습니다.

```bash
CONTROL_STATION_ARM_PROGRAM=/경로/lerobot-teleoperate ./run-control-station.sh --arm
```

그 다음 화면의 **텔레오퍼레이션 시작** 버튼을 누르면 팔 프로세스를 시작합니다. `--arm`은 시작 API를 활성화할 뿐 서버 실행만으로 팔을 움직이지 않습니다. 종료 버튼은 프로세스에 `SIGINT`를 보냅니다.

카메라 캡처와 JPEG 인코딩은 관제 서버가 담당하므로 `opencv-python-headless`도 관제 가상환경에 설치되어야 합니다. `requirements.txt` 설치 후 `--arm`으로 실행하면 됩니다. LeRobot 가상환경에만 OpenCV가 설치된 경우 관제 서버에서는 사용할 수 없습니다.

기본 장치 설정은 follower `/dev/omx_follower`, leader `/dev/omx_leader`입니다. LeRobot은 카메라를 열거나 별도 Rerun 화면을 띄우지 않습니다. `--arm` 모드에서 관제 서버가 Joint Camera `/dev/video4`와 Head Camera `/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0`를 열어 MJPEG 영상으로 표시합니다. 각 경로는 `CONTROL_STATION_JOINT_CAMERA`, `CONTROL_STATION_HEAD_CAMERA`로 바꿀 수 있습니다. 환경 변수 `CONTROL_STATION_FOLLOWER_PORT`, `CONTROL_STATION_LEADER_PORT`로 모터 포트를 바꿀 수 있습니다. 런처에서 시작한 팔 프로세스 로그는 `.control-station/logs/arm-<세션ID>.log`에 저장하며 `CONTROL_STATION_ARM_LOG`로 경로를 지정할 수 있습니다.

장치 조사에서 `/dev/video4`는 Innomaker 영상 캡처, `/dev/video5`는 해당 장치의 메타데이터 인터페이스로 확인됐습니다. 별도로 `HCAM0` 장치가 `/dev/video6`으로 나타났고 캡처 프레임을 읽었습니다. 안정적인 by-id 경로를 Head Camera 기본값으로 사용합니다. 화면의 Head Camera 피드가 로봇 머리 위치와 맞는지 확인하고, 다르면 환경 변수로 장치 경로를 조정하세요.

실제 팔 제어는 운영체제의 장치 권한, LeRobot 의존성 및 사용자별 OMX 지원 코드가 있어야 합니다. 이 구성은 현재 관제 저장소에 포함되지 않습니다. 저장소만 복제한 환경에서는 첫 번째 절차로 대시보드와 API를 실행할 수 있습니다.

## Head Camera YOLO 검출 표시

Head Camera 영상에 학습한 YOLO 모델(딸기·자동차·공)의 검출 박스를 그려 표시합니다. 모델 파일 경로를 지정하면 `--arm` 없이도 Head Camera만 엽니다. Joint Camera는 `--arm` 모드에서만 엽니다.

```bash
source .venv/bin/activate
python -m pip install -r requirements-vision.txt   # 처음 한 번. PyTorch가 함께 설치되어 용량이 큽니다.
CONTROL_STATION_HEAD_YOLO_WEIGHTS=/경로/best.pt ./run-control-station.sh
```

| 환경 변수 | 기본값 | 내용 |
|---|---|---|
| `CONTROL_STATION_HEAD_YOLO_WEIGHTS` | 없음 (꺼짐) | YOLO 가중치 `.pt` 경로 |
| `CONTROL_STATION_HEAD_YOLO_CONF` | `0.25` | 표시할 최소 신뢰도 |
| `CONTROL_STATION_HEAD_YOLO_DEVICE` | 자동 | `0`(첫 번째 GPU) 또는 `cpu` |

화면의 Head Camera 카드에 박스가 그려진 영상과 **YOLO 검출** 행(클래스·신뢰도)이 나타나고, `/api/cameras` 응답의 `detector_state`·`detections`(클래스, 신뢰도, 픽셀 중심 좌표)로도 확인할 수 있습니다. 가중치 파일이 없거나 `ultralytics`가 설치되지 않았으면 원본 영상을 그대로 보여 주고 카드에 오류를 표시합니다.

검출 결과는 표시 전용입니다. 로봇 명령이나 작업 상태 판단에 쓰지 않습니다. 카메라는 한 프로그램만 열 수 있으므로 다른 카메라 프로그램을 먼저 종료합니다.

## ROS 2 Nav2 검색 모드

Nav2 action server discovery만 확인할 때 사용합니다. 이동 목표는 보내지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source /경로/ros2_워크스페이스/install/setup.bash
CONTROL_STATION_MODE=ros2 CONTROL_STATION_MOBILE_DOMAIN=15 ./run-control-station.sh
```

domain을 지정하지 않으면 `ROS_DOMAIN_ID`, 없으면 `0`을 사용합니다.
