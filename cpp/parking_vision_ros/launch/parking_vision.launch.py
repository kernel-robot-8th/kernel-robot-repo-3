"""Start the Pinky parking judgment node. It only observes; it never commands Pinky or the arm.

For a rosbag, play it with --clock and pass use_sim_time:=true: the check only judges frames stamped after the
request, so recorded stamps compared against the wall clock all count as old.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
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
            # value_type=str keeps spot_id:=1 a string; launch would otherwise read it as an integer.
            parameters=[{
                **{name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in arguments},
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
            output="screen",
        ),
    ])
