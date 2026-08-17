# Seven DOF Arm MoveIt Config

This is the motion planning "brain" of the 7-DOF arm simulation pipeline, built for **ROS 2 Jazzy**.

## Architecture & Responsibilities

This package is responsible for all intelligent path planning, inverse kinematics, and collision avoidance before any execution commands are sent to the Gazebo simulation.

* **`demo.launch.py`**: The master launch file. It orchestrates the entire simulation by importing Gazebo, booting up RViz, starting the `move_group` AI planner, and tying all nodes to `use_sim_time:=True`.
* **SRDF (`seven_dof_arm.srdf`)**: Defines the robot's semantic planning groups (`arm` and `gripper`), named positions (like `home` or `ready`), and critically, the **Self-Collision Matrix** that tells MoveIt which links are allowed to touch. 
* **Kinematics (`kinematics.yaml`)**: Configured to use `KDLKinematicsPlugin` to resolve end-effector Cartesian targets back into 7 joint angles.
* **OMPL Config (`ompl_planning.yaml`)**: Defines the specific probabilistic search algorithms (e.g., RRTConnect) used to navigate the arm around obstacles.
* **Controllers Map (`moveit_controllers.yaml`)**: Maps MoveIt's abstract planning outputs to the actual ROS 2 Action Server topics (e.g., `follow_joint_trajectory`) exposed by Gazebo's hardware controllers.

## Collision Tuning Note
For ease of testing and to prevent the `move_group` node from constantly aborting goals when the arm is dragged into a tight curl, self-collision checking between the gripper fingers and the upper arm (`link_4`) has been intentionally disabled in the SRDF.
