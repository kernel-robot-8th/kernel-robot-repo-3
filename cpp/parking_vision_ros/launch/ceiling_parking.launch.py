"""Ceiling camera capture and Pinky parking judgment in one component container (intra-process images).

Only observes: it opens the ceiling camera read-only and never commands Pinky or the arm. The control station
opens the same camera (HCAM0) when its Head Camera is a V4L2 device, so give it
CONTROL_STATION_HEAD_CAMERA=ros:/ceiling_camera/image_raw/compressed instead.

The capture is parking_vision_ros::CeilingCameraNode rather than usb_cam: on 2026-11-03 usb_cam 0.8.1 aborted
this container on a corrupt MJPEG frame and could not follow the camera when it re-enumerated, which also took
the parking node down. CeilingCameraNode reopens the by-path device every 2 s after a failure, so the parking
node stays up and reports CAMERA_UNAVAILABLE while the camera is gone.

The camera device is video_device:=..., else CONTROL_STATION_CEILING_CAMERA (the same variable the pinky_parking CLI
reads, kept per PC in the git-ignored .control-station/control-station.env), else the shared default below. Two HCAM0
cameras share one by-id and serial, so only the by-path port tells them apart.

One parking node follows one spot live, and each Pinky has its own registration of the loading spot (its marker sits a
little differently, config/pinky-markers.json). So besides the node of spot_id ("pinky_parking") one more node is
started for every other spot in that file whose registration exists (.control-station/parking-spots/<spot>.json),
named "pinky_parking_<spot>" with every character that is not a letter or a digit turned into "_" (the same rule as
control_station/parking_ros.parking_node_name). The control station reads each Pinky's own node while it drives in.
A spot registered while this runs is followed after the next start.
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

DEFAULT_DEVICE = "/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0"


def live_spots(repository_root: str, primary_spot_id: str, markers_path: str = "") -> list[str]:
    """The spots to follow live: the primary one, then every registered spot of config/pinky-markers.json."""
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
        pass  # no marker file: the primary spot alone, as before
    return spots


def parking_node_name(spot_id: str, primary_spot_id: str) -> str:
    if spot_id == primary_spot_id:
        return "pinky_parking"
    return "pinky_parking_" + re.sub(r"[^A-Za-z0-9]", "_", spot_id)


def container(context) -> list:
    intra = {"use_intra_process_comms": True}

    def text(value: str) -> ParameterValue:  # spot_id:=1 stays a string instead of becoming an integer
        return ParameterValue(value, value_type=str)

    root = LaunchConfiguration("repository_root").perform(context)
    primary = LaunchConfiguration("spot_id").perform(context)
    spots = live_spots(root, primary, LaunchConfiguration("pinky_markers").perform(context))
    return [ComposableNodeContainer(
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
                parameters=[{"device": text(LaunchConfiguration("video_device").perform(context)),
                             "frame_id": "ceiling_camera"}],
                extra_arguments=[intra],
            ),
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
