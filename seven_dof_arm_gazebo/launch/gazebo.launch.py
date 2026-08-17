"""
gazebo.launch.py
================
Full Gazebo Harmonic simulation launch for the 7-DOF arm.

Sequence
--------
1. Parse URDF via xacro and publish robot_description
2. Start Gazebo Harmonic (gz sim) with our empty world
3. Spawn the robot into Gazebo via ros_gz_sim create service
4. Start the controller_manager's spawner for:
     - joint_state_broadcaster
     - arm_controller  (JointTrajectoryController)
     - gripper_controller (JointTrajectoryController)
5. Bridge /clock from Gazebo to ROS 2

Usage:
    ros2 launch seven_dof_arm_gazebo gazebo.launch.py
    ros2 launch seven_dof_arm_gazebo gazebo.launch.py gui:=false   # headless
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── package share directories ──────────────────────────────────────────────
    pkg_desc   = get_package_share_directory("seven_dof_arm_description")
    pkg_gazebo = get_package_share_directory("seven_dof_arm_gazebo")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    # ── launch arguments ───────────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Set to 'false' to run Gazebo headless (no GUI)",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_gazebo, "worlds", "empty_world.sdf"),
        description="Absolute path to Gazebo world SDF",
    )

    gui   = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")

    # ── robot description (xacro → URDF string) ────────────────────────────────
    urdf_xacro = os.path.join(pkg_desc, "urdf", "seven_dof_arm.urdf.xacro")
    controllers_yaml = os.path.join(pkg_desc, "config", "ros2_controllers.yaml")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", urdf_xacro,
            " controllers_yaml:=", controllers_yaml,
        ]
    )
    robot_description = {"robot_description": ParameterValue(
        robot_description_content, value_type=str
    )}

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── Gazebo Harmonic (gz sim) ───────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", world],          # -r = start running immediately
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Spawn robot into Gazebo ────────────────────────────────────────────────
    controllers_yaml = os.path.join(pkg_desc, "config", "ros2_controllers.yaml")

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_seven_dof_arm",
        arguments=[
            "-name",  "seven_dof_arm",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        parameters=[{"ros_params_file": controllers_yaml}],
        output="screen",
    )

    # ── Clock bridge: Gazebo → ROS 2 ──────────────────────────────────────────
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ── Camera bridge: Gazebo → ROS 2 ──────────────────────────────────────────
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    # ── Controller spawners (started after robot is spawned) ─────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_controller_spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="gripper_controller_spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # After robot spawned → start joint_state_broadcaster
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # After joint_state_broadcaster active → start arm + gripper controllers
    start_arm_ctrl_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        gui_arg,
        world_arg,
        robot_state_publisher,
        gazebo,
        clock_bridge,
        camera_bridge,
        spawn_robot,
        start_jsb_after_spawn,
        start_arm_ctrl_after_jsb,
    ])
