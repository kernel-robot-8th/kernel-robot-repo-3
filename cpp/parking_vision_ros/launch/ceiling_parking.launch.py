"""Ceiling camera capture and Pinky parking judgment in one component container (intra-process images).

Only observes: it opens the ceiling camera read-only and never commands Pinky or the arm. The control station
opens the same camera (HCAM0) when its Head Camera is a V4L2 device, so give it
CONTROL_STATION_HEAD_CAMERA=ros:/ceiling_camera/image_raw/compressed instead.

The capture is parking_vision_ros::CeilingCameraNode rather than usb_cam: on 2026-11-03 usb_cam 0.8.1 aborted
this container on a corrupt MJPEG frame and could not follow the camera when it re-enumerated, which also took
the parking node down. CeilingCameraNode reopens the by-path device every 2 s after a failure, so the parking
node stays up and reports CAMERA_UNAVAILABLE while the camera is gone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

DEFAULT_DEVICE = "/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0"


def generate_launch_description() -> LaunchDescription:
    intra = {"use_intra_process_comms": True}
    return LaunchDescription([
        DeclareLaunchArgument("video_device", default_value=DEFAULT_DEVICE,
                              description="Ceiling camera by-path device (two HCAM0 share one by-id)"),
        DeclareLaunchArgument("repository_root", default_value="."),
        DeclareLaunchArgument("spot_id", default_value="pinky_dock_1"),
        ComposableNodeContainer(
            name="ceiling_parking_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="parking_vision_ros",
                    plugin="parking_vision_ros::CeilingCameraNode",
                    name="ceiling_camera",
                    namespace="ceiling_camera",
                    parameters=[{"device": LaunchConfiguration("video_device"), "frame_id": "ceiling_camera"}],
                    extra_arguments=[intra],
                ),
                ComposableNode(
                    package="parking_vision_ros",
                    plugin="parking_vision_ros::ParkingVisionNode",
                    name="pinky_parking",
                    parameters=[{
                        "repository_root": LaunchConfiguration("repository_root"),
                        "spot_id": LaunchConfiguration("spot_id"),
                        "image_topic": "/ceiling_camera/image_raw",
                    }],
                    extra_arguments=[intra],
                ),
            ],
        ),
    ])
