# Seven DOF Arm Description

This package serves as the **Single Source of Truth** for the physical configuration of the 7-DOF arm. 

It contains absolutely no runtime nodes, planners, or physics simulation configurations. Its sole responsibility is defining the robot's physical properties so that other packages (Gazebo, MoveIt, RViz) can reliably import it.

## Architecture & XACRO Modularity

The URDF is broken into highly modular XACRO files to maintain clean separation of concerns:

1. **`seven_dof_arm.urdf.xacro`**: The core blueprint. Defines links, joints, geometric primitives, inertias, and colors (carbon black, neon blue, sleek white).
2. **`seven_dof_arm.gazebo.xacro`**: Contains the Gazebo Harmonic plugin definitions (`gz_ros2_control-system`) required to inject the robot into a physics engine.
3. **`seven_dof_arm.ros2_control.xacro`**: Defines the ROS 2 Hardware Interfaces (command/state mapping and joint limits) for the controllers to read.

At compile-time, the main `urdf.xacro` uses `<xacro:include>` to stitch the simulation and control parameters into a single unified robot description.

## Important Note on Materials
This package utilizes native URDF `<color>` tags for vibrant visualization. Legacy `<gazebo reference="..."><material>` tags have been explicitly removed to ensure Gazebo Harmonic natively maps the URDF colors to its modern Physically Based Rendering (PBR) engine, matching RViz perfectly.
