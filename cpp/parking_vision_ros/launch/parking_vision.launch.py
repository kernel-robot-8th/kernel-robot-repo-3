"""Start the Pinky parking judgment node. It only observes; it never commands Pinky or the arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = {
        "repository_root": ".",
        "spot_id": "pinky_dock_1",
        "image_topic": "/ceiling_camera/image_raw",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        Node(
            package="parking_vision_ros",
            executable="parking_vision_node",
            name="pinky_parking",
            parameters=[{name: LaunchConfiguration(name) for name in arguments}],
            output="screen",
        ),
    ])
