import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_gazebo(context):
    package_share = get_package_share_directory('toktak_warehouse')
    gazebo_share = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(package_share, 'worlds', 'toktak_warehouse.world')
    gui_config = os.path.join(package_share, 'config', 'toktak_warehouse_gui.config')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    args = ['-r']
    if headless:
        args.insert(0, '-s')
    else:
        args.extend(['--gui-config', shlex.quote(gui_config)])
    args.append(shlex.quote(world_file))

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ' '.join(args)}.items(),
    )]


def generate_launch_description():
    package_share = get_package_share_directory('toktak_warehouse')
    bridge_config = os.path.join(package_share, 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='False',
            description='Run Gazebo server without opening the GUI',
        ),
        OpaqueFunction(function=_launch_gazebo),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='toktak_warehouse_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_frame',
            arguments=[
                '--x', '0.215', '--y', '0', '--z', '0.215',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
        ),
    ])
