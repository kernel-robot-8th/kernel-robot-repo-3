"""천장 카메라 캡처와 Pinky 주차 판정을 컴포넌트 컨테이너 하나에 올린다(영상은 프로세스 안에서 전달, intra-process).

관측만 한다. 천장 카메라를 읽기 전용으로 열고 Pinky나 로봇팔에 명령하지 않는다. 관제의 Head Camera가 V4L2 장치이면
관제가 같은 카메라(HCAM0)를 열므로, 대신 CONTROL_STATION_HEAD_CAMERA=ros:/ceiling_camera/image_raw/compressed를 준다.

캡처는 usb_cam이 아니라 parking_vision_ros::CeilingCameraNode다. 2026-11-03에 usb_cam 0.8.1이 깨진 MJPEG 프레임에서
이 컨테이너를 종료시켰고, 카메라가 다시 잡혔을 때(re-enumerate) 따라가지 못해서 판정 노드까지 함께 꺼졌다.
CeilingCameraNode는 실패하면 2초마다 by-path 장치를 다시 열기 때문에, 카메라가 빠진 동안에도 판정 노드는 살아 있고
CAMERA_UNAVAILABLE을 알린다.

카메라 장치는 video_device:=... 인자, 없으면 CONTROL_STATION_CEILING_CAMERA(pinky_parking CLI도 읽는 같은 변수이며,
git에서 제외된 .control-station/control-station.env에 PC마다 둔다), 그것도 없으면 아래 공용 기본값이다. HCAM0 카메라
두 대가 by-id와 serial이 같아서 by-path 포트로만 구분할 수 있다.

판정 노드 하나는 주차 위치 하나만 실시간으로 따라가고, Pinky마다 적재 위치를 따로 등록해 둔다(마커가 붙은 위치가 조금씩
달라서다, config/pinky-markers.json). 그래서 spot_id의 노드("pinky_parking") 말고도, 그 파일에 있고 등록 파일
(.control-station/parking-spots/<spot>.json)이 있는 다른 주차 위치마다 노드를 하나 더 띄운다. 이름은
"pinky_parking_<spot>"이고, 영문자와 숫자가 아닌 글자는 모두 "_"로 바꾼다(control_station/parking_ros.parking_node_name과
같은 규칙). 관제는 Pinky가 들어오는 동안 그 Pinky의 노드를 읽는다. 실행 중에 등록한 주차 위치는 다음 시작부터 따라간다.
"""

import json
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# 현장 PC에서 천장 카메라가 꽂힌 USB 포트 4의 by-path. 장치 번호(/dev/videoN)는 다시 잡힐 때 바뀔 수 있어서 쓰지 않는다.
DEFAULT_DEVICE = "/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0"


def live_spots(repository_root: str, primary_spot_id: str, markers_path: str = "") -> list[str]:
    """실시간으로 따라갈 주차 위치 목록: 기본 주차 위치, 그다음 config/pinky-markers.json에 있고 등록된 주차 위치들."""
    spots = [primary_spot_id]
    try:
        with open(markers_path or os.path.join(repository_root, "config", "pinky-markers.json"), encoding="utf-8") as handle:
            entries = json.load(handle).get("pinkies", {}).values()
        for entry in entries:
            spot = entry.get("spot_id") if isinstance(entry, dict) else None
            registration = os.path.join(repository_root, ".control-station", "parking-spots", f"{spot}.json")
            if isinstance(spot, str) and spot and spot not in spots and os.path.isfile(registration):
                spots.append(spot)
    except (OSError, ValueError, AttributeError):
        pass  # 마커 파일이 없으면: 예전처럼 기본 주차 위치 하나만
    return spots


def parking_node_name(spot_id: str, primary_spot_id: str) -> str:
    # 기본 주차 위치는 기존 이름(pinky_parking)을 그대로 쓴다. 관제와 문서가 /pinky_parking/...을 가리키기 때문이다.
    if spot_id == primary_spot_id:
        return "pinky_parking"
    return "pinky_parking_" + re.sub(r"[^A-Za-z0-9]", "_", spot_id)


# OpaqueFunction으로 실행 시점에 launch 인자 값을 읽어서(perform) 노드 목록을 만든다.
# 주차 위치 개수가 등록 파일에 따라 달라지므로 정적인 LaunchDescription으로는 만들 수 없다.
def container(context) -> list:
    intra = {"use_intra_process_comms": True}

    def text(value: str) -> ParameterValue:  # spot_id:=1이 정수가 되지 않고 문자열로 남는다
        return ParameterValue(value, value_type=str)

    root = LaunchConfiguration("repository_root").perform(context)
    primary = LaunchConfiguration("spot_id").perform(context)
    spots = live_spots(root, primary, LaunchConfiguration("pinky_markers").perform(context))
    return [ComposableNodeContainer(
        name="ceiling_parking_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",  # 단일 스레드 executor. 판정 서비스는 자기 스레드에서 답하므로 충분하다
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="parking_vision_ros",
                plugin="parking_vision_ros::CeilingCameraNode",
                name="ceiling_camera",
                namespace="ceiling_camera",
                parameters=[{"device": text(LaunchConfiguration("video_device").perform(context)),
                             "frame_id": "ceiling_camera"}],
                extra_arguments=[intra],
            ),
            # 주차 위치마다 판정 노드 하나. 모두 같은 /ceiling_camera/image_raw를 프로세스 안에서 받는다.
            *[ComposableNode(
                package="parking_vision_ros",
                plugin="parking_vision_ros::ParkingVisionNode",
                name=parking_node_name(spot, primary),
                parameters=[{
                    "repository_root": text(root),
                    "spot_id": text(spot),
                    "image_topic": "/ceiling_camera/image_raw",
                }],
                extra_arguments=[intra],
            ) for spot in spots],
        ],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("video_device",
                              default_value=os.environ.get("CONTROL_STATION_CEILING_CAMERA", "").strip() or DEFAULT_DEVICE,
                              description="Ceiling camera by-path device (default: CONTROL_STATION_CEILING_CAMERA)"),
        DeclareLaunchArgument("repository_root", default_value="."),
        DeclareLaunchArgument("spot_id", default_value="pinky_dock_1"),
        DeclareLaunchArgument("pinky_markers", default_value=os.environ.get("CONTROL_STATION_PINKY_MARKERS", "").strip(),
                              description="Pinky marker file (default: <repository_root>/config/pinky-markers.json)"),
        OpaqueFunction(function=container),
    ])
