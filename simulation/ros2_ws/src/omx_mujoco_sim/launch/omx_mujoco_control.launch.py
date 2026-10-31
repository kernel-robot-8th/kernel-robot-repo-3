from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("omx_mujoco_sim"), "urdf", "omx_mujoco_control.urdf.xacro"]
    )
    robot_description = Command(["xacro ", robot_description_file])
    controller_config = PathJoinSubstitution(
        [FindPackageShare("omx_mujoco_sim"), "config", "controllers.yaml"]
    )
    camera_plugin_config = PathJoinSubstitution(
        [FindPackageShare("omx_mujoco_sim"), "config", "camera_plugins.yaml"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    simulation_namespace = "mujoco_omx"
    simulation_joint_states_topic = "/mujoco_omx/joint_state_broadcaster/joint_states"

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        namespace=simulation_namespace,
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time},
            controller_config,
            camera_plugin_config,
        ],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=simulation_namespace,
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
                "frame_prefix": "mujoco_omx/",
            }
        ],
        remappings=[("joint_states", simulation_joint_states_topic)],
        output="both",
    )
    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mujoco_world_to_omx_base_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "mujoco_omx/link0",
        ],
        output="both",
    )
    world_to_fixed_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mujoco_world_to_fixed_camera_tf",
        arguments=[
            "--x", "0.05", "--y", "0.25", "--z", "0.85",
            "--roll", "3.141592653589793", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "placeholder_fixed_camera_optical_frame",
        ],
        output="both",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=simulation_namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/mujoco_omx/controller_manager",
            "--param-file",
            controller_config,
        ],
        output="both",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=simulation_namespace,
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/mujoco_omx/controller_manager",
            "--param-file",
            controller_config,
        ],
        output="both",
    )
    fixed_camera_point_cloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="mujoco_fixed_camera_point_cloud",
        condition=IfCondition(enable_point_cloud),
        remappings=[
            ("image_rect", "/mujoco_omx/fixed_camera/depth/image_raw"),
            (
                "/mujoco_omx/fixed_camera/depth/camera_info",
                "/mujoco_omx/fixed_camera/camera_info",
            ),
            ("points", "/mujoco_omx/fixed_camera/points"),
        ],
        output="both",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_point_cloud", default_value="false"),
            robot_state_publisher,
            world_to_base_tf,
            world_to_fixed_camera_tf,
            control_node,
            joint_state_spawner,
            trajectory_spawner,
            fixed_camera_point_cloud,
        ]
    )
