import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory("seven_dof_arm_gazebo")
    pkg_moveit = get_package_share_directory("seven_dof_arm_moveit_config")

    # The custom colored objects world
    colored_world_path = os.path.join(pkg_gazebo, "worlds", "colored_objects_world.sdf")

    # Launch demo.launch.py but with our new world
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "demo.launch.py")
        ),
        launch_arguments={"world": colored_world_path}.items(),
    )

    # Launch the vision grasping YOLO node
    vision_node = Node(
        package="seven_dof_arm_gazebo",
        executable="vision_grasp.py",
        name="vision_grasp_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Launch rqt_image_view automatically to see the annotated feed
    rqt_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        arguments=["/vision/annotated_image"],
    )

    return LaunchDescription([
        demo_launch,
        vision_node,
        rqt_node
    ])
