"""토크탁 창고 Gazebo Harmonic 월드와 ROS 2 bridge를 띄우는 launch 파일이다.

    ros2 launch toktak_warehouse toktak_warehouse.launch.py               # Gazebo GUI 포함
    ros2 launch toktak_warehouse toktak_warehouse.launch.py headless:=True  # 서버만

보통은 `simulation/run.sh`가 빌드 후 이 파일을 실행한다. 띄우는 것:
- Gazebo(`ros_gz_sim`의 gz_sim.launch.py)로 `worlds/toktak_warehouse.world`를 실행 상태(-r)로 연다.
- `ros_gz_bridge` parameter_bridge: `config/bridge.yaml`대로 /clock, /cmd_vel, /odom, /tf, /scan, /imu/data,
  /joint_states를 Gazebo 토픽과 잇는다.
- 로봇 링크 사이 고정 TF(base_footprint→base_link, base_link→laser_frame, base_link→imu_link).

시뮬레이터 안의 바퀴 두 개짜리 가상 로봇만 움직이며 실물 핑키는 움직이지 않는다.
"""
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_gazebo(context):
    """headless 인자 값에 따라 Gazebo 실행 인자(gz_args)를 만들어 gz_sim.launch.py를 포함한다.

    launch 인자 값은 실행 시점에야 정해지므로 OpaqueFunction 안에서 `perform(context)`로 읽는다.
    """
    package_share = get_package_share_directory('toktak_warehouse')
    gazebo_share = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(package_share, 'worlds', 'toktak_warehouse.world')
    gui_config = os.path.join(package_share, 'config', 'toktak_warehouse_gui.config')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    args = ['-r']  # 열자마자 시뮬레이션을 실행한다(일시정지 상태로 열지 않음).
    if headless:
        args.insert(0, '-s')  # 서버만 실행하고 GUI 창은 열지 않는다.
    else:
        args.extend(['--gui-config', shlex.quote(gui_config)])
    args.append(shlex.quote(world_file))  # gz_args는 한 문자열로 넘어가므로 경로에 공백이 있어도 되게 따옴표를 친다.

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ' '.join(args)}.items(),
    )]


def generate_launch_description():
    """Gazebo, ROS↔Gazebo bridge, 로봇 링크 사이 고정 TF를 띄우는 LaunchDescription을 만든다."""
    package_share = get_package_share_directory('toktak_warehouse')
    bridge_config = os.path.join(package_share, 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='False',
            description='Run Gazebo server without opening the GUI',  # GUI 창 없이 Gazebo 서버만 실행한다.
        ),
        OpaqueFunction(function=_launch_gazebo),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='toktak_warehouse_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
        # 바닥 기준 프레임과 몸체 프레임은 같은 위치로 둔다.
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
            # 라이다는 몸체 중심에서 앞 0.215 m, 위 0.215 m에 있다. 로봇 모델(model.sdf)의 lidar 센서 위치와 같다.
            # /scan의 frame_id(laser_frame)를 Nav2가 몸체 기준으로 바꿀 수 있게 이 TF가 필요하다.
            name='base_link_to_laser_frame',
            arguments=[
                '--x', '0.215', '--y', '0', '--z', '0.215',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # IMU는 몸체 중심과 같은 위치로 둔다(/imu/data의 frame_id가 imu_link).
            name='base_link_to_imu_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
        ),
    ])
