"""Pinky 주차 판정 노드(parking_vision_node, 노드 이름 pinky_parking)만 띄운다. 관측만 하며 Pinky나 로봇팔에 명령하지 않는다.

영상은 다른 노드가 image_topic(기본 /ceiling_camera/image_raw)으로 내야 한다. 캡처와 판정을 한 컨테이너에 올리려면
ceiling_parking.launch.py를 쓴다.

rosbag은 --clock으로 재생하고 use_sim_time:=true를 넘긴다. 판정은 요청 뒤에 찍힌 프레임만 쓰므로, 녹화된 stamp를
벽시계와 비교하면 모두 오래된 프레임으로 세어진다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # launch 인자 이름 -> 기본값. 같은 이름의 노드 파라미터로 그대로 넘긴다.
    arguments = {
        "repository_root": ".",
        "spot_id": "pinky_dock_1",
        "image_topic": "/ceiling_camera/image_raw",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when replaying a rosbag with --clock"),
        Node(
            package="parking_vision_ros",
            executable="parking_vision_node",
            name="pinky_parking",
            # value_type=str이면 spot_id:=1이 문자열로 남는다. 그렇지 않으면 launch가 정수로 읽는다.
            parameters=[{
                **{name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in arguments},
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
            output="screen",
        ),
    ])
