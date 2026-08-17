cmake_minimum_required(VERSION 3.8)
project(seven_dof_arm_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
    scripts/test_gripper.py 
    scripts/vision_grasp.py
    scripts/pick_and_place.py
    scripts/tf_debug.py
    scripts/gui_pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
