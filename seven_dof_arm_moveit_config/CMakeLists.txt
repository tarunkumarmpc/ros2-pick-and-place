cmake_minimum_required(VERSION 3.8)
project(seven_dof_arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
