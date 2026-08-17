"""
demo.launch.py
==============
All-in-one launch:
  1. Gazebo Harmonic  (full physics simulation)
  2. robot_state_publisher  (publishes /tf from /joint_states)
  3. Spawns robot into Gazebo + starts ros2_control controllers
  4. move_group  (MoveIt2 planning server)
  5. RViz2 with the MoveIt motion planning panel

Run with:
    ros2 launch seven_dof_arm_moveit_config demo.launch.py

To skip Gazebo (standalone MoveIt + fake hardware):
    ros2 launch seven_dof_arm_moveit_config demo.launch.py use_gazebo:=false
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    pkg_desc    = get_package_share_directory("seven_dof_arm_description")
    pkg_gazebo  = get_package_share_directory("seven_dof_arm_gazebo")
    pkg_moveit  = get_package_share_directory("seven_dof_arm_moveit_config")
    pkg_ros_gz  = get_package_share_directory("ros_gz_sim")

    # ── arguments ─────────────────────────────────────────────────────────────
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo", default_value="true",
        description="Launch Gazebo simulation",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_gazebo, "worlds", "empty_world.sdf"),
        description="Absolute path to Gazebo world SDF",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2 with MoveIt panel",
    )
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_rviz   = LaunchConfiguration("use_rviz")
    world      = LaunchConfiguration("world")

    # ── robot_description ──────────────────────────────────────────────────────
    urdf_xacro = os.path.join(pkg_desc, "urdf", "seven_dof_arm.urdf.xacro")
    controllers_yaml = os.path.join(pkg_desc, "config", "ros2_controllers.yaml")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", urdf_xacro,
            " controllers_yaml:=", controllers_yaml,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ── robot_description_semantic (SRDF) ──────────────────────────────────────
    srdf_path = os.path.join(pkg_moveit, "srdf", "seven_dof_arm.srdf")
    with open(srdf_path) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # ── config files ──────────────────────────────────────────────────────────
    robot_description_kinematics = {"robot_description_kinematics": load_yaml("seven_dof_arm_moveit_config", "config/kinematics.yaml")}
    ompl_planning_pipeline_config = {"ompl": load_yaml("seven_dof_arm_moveit_config", "config/ompl_planning.yaml")}
    moveit_controllers_dict = load_yaml("seven_dof_arm_moveit_config", "config/moveit_controllers.yaml")
    robot_description_planning = {"robot_description_planning": load_yaml("seven_dof_arm_moveit_config", "config/joint_limits.yaml")}
    rviz_cfg                 = os.path.join(pkg_moveit, "config", "moveit.rviz")

    # ══════════════════════════════════════════════════════════════════════════
    # WITH GAZEBO  (physics simulation)
    # ══════════════════════════════════════════════════════════════════════════

    # Gazebo launch (includes robot_state_publisher + controller spawners)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world}.items(),
        condition=IfCondition(use_gazebo),
    )

    # ══════════════════════════════════════════════════════════════════════════
    # WITHOUT GAZEBO  (fake/mock hardware for pure MoveIt testing)
    # ══════════════════════════════════════════════════════════════════════════
    robot_state_publisher_fake = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        condition=UnlessCondition(use_gazebo),
    )

    # ══════════════════════════════════════════════════════════════════════════
    # MOVE GROUP
    # ══════════════════════════════════════════════════════════════════════════
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            moveit_controllers_dict,
            robot_description_planning,
            trajectory_execution,
            planning_scene_monitor_params,
            {
                "use_sim_time": True,
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
            },
        ],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # RVIZ2
    # ══════════════════════════════════════════════════════════════════════════
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            robot_description_planning,
            {"use_sim_time": True},
        ],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_gazebo_arg,
        use_rviz_arg,
        world_arg,
        # Simulation
        gazebo_launch,
        robot_state_publisher_fake,
        # Planning
        move_group_node,
        # Visualisation
        rviz_node,
    ])
