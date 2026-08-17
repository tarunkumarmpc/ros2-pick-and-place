"""
move_group.launch.py
Starts the MoveIt2 move_group node with all required parameters.
Does NOT start Gazebo or RViz — use together with gazebo.launch.py
or demo.launch.py.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
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
    pkg_desc   = get_package_share_directory("seven_dof_arm_description")
    pkg_moveit = get_package_share_directory("seven_dof_arm_moveit_config")

    # ── robot_description ──────────────────────────────────────────────────────
    urdf_xacro = os.path.join(pkg_desc, "urdf", "seven_dof_arm.urdf.xacro")
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_xacro]
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

    # ── trajectory execution ──────────────────────────────────────────────────
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

    return LaunchDescription([move_group_node])
