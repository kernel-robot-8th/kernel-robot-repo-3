# 실행 방법

## 대시보드와 API만 실행

Python 3.10 이상이 있는 저장소 복제본에서 실행합니다. 추가 Python 패키지나 별도 LeRobot 폴더가 필요하지 않습니다.

```bash
./run-control-station.sh
```

브라우저에서 <http://127.0.0.1:8000>을 엽니다. 기본 모드에서는 로봇 제어를 시작하지 않습니다. 서버를 종료하려면 실행 중인 터미널에서 `Ctrl+C`를 누릅니다.

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

기본 장치 설정은 follower `/dev/omx_follower`, leader `/dev/omx_leader`입니다. LeRobot은 카메라를 열거나 별도 Rerun 화면을 띄우지 않습니다. `--arm` 모드에서 관제 서버가 Joint Camera `/dev/video4`와 Head Camera `/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0`를 열어 MJPEG 영상으로 표시합니다. 각 경로는 `CONTROL_STATION_JOINT_CAMERA`, `CONTROL_STATION_HEAD_CAMERA`로 바꿀 수 있습니다. 환경 변수 `CONTROL_STATION_FOLLOWER_PORT`, `CONTROL_STATION_LEADER_PORT`로 모터 포트를 바꿀 수 있습니다. 로그는 `data/arm-teleop.log`에 쌓입니다.

장치 조사에서 `/dev/video4`는 Innomaker 영상 캡처, `/dev/video5`는 해당 장치의 메타데이터 인터페이스로 확인됐습니다. 별도로 `HCAM0` 장치가 `/dev/video6`으로 나타났고 캡처 프레임을 읽었습니다. 안정적인 by-id 경로를 Head Camera 기본값으로 사용합니다. 화면의 Head Camera 피드가 로봇 머리 위치와 맞는지 확인하고, 다르면 환경 변수로 장치 경로를 조정하세요.

실제 팔 제어는 운영체제의 장치 권한, LeRobot 의존성 및 사용자별 OMX 지원 코드가 있어야 합니다. 이 구성은 현재 관제 저장소에 포함되지 않습니다. 저장소만 복제한 환경에서는 첫 번째 절차로 대시보드와 API를 실행할 수 있습니다.

## ROS 2 Nav2 검색 모드

Nav2 action server discovery만 확인할 때 사용합니다. 이동 목표는 보내지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source /경로/ros2_워크스페이스/install/setup.bash
CONTROL_STATION_MODE=ros2 CONTROL_STATION_MOBILE_DOMAIN=15 \
  PYTHONPATH=src python3 -m control_station.app
```

domain을 지정하지 않으면 `ROS_DOMAIN_ID`, 없으면 `0`을 사용합니다.
