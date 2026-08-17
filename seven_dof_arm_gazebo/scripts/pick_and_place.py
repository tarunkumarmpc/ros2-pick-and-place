#!/usr/bin/env python3
"""
pick_and_place.py
=================
"""
pick_and_place.py
=================
Implementation of a pick-and-place state machine with a two-layer depth strategy:

  Layer 1 - Known-surface Z:
      Table height and object dimensions are referenced from the world SDF.
      X,Y coordinates are derived from the vision system (HSV color tracking -> TF).

  Layer 2 - Continuous TCP distance monitoring:
      During descent, the node queries the TF distance between the arm's TCP link
      and the target frame. The grasp triggers when the distance falls below GRASP_THRESHOLD.
      This logic accounts for physical disturbances or IK settling errors.

Usage:
    ros2 run seven_dof_arm_gazebo pick_and_place.py --target "Red Box"
    ros2 run seven_dof_arm_gazebo pick_and_place.py --target "Blue Cylinder"
    ros2 run seven_dof_arm_gazebo pick_and_place.py --target "Green Sphere"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import argparse
import time
import math

from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

from tf2_ros import Buffer, TransformListener

from rclpy.parameter import Parameter

# ── World-geometry constants (from colored_objects_world.sdf) ───────────────
# Table: pose z=0.05, half-height=0.05 → table top at Z=0.10 m
TABLE_TOP_Z = 0.10

# Object centroid Z above ground (= table_top + shape_half_height)
OBJECT_GRASP_Z = {
    'red_box':       TABLE_TOP_Z + 0.025,   # box 0.05 tall
    'blue_cylinder': TABLE_TOP_Z + 0.050,   # cylinder 0.10 tall
    'green_sphere':  TABLE_TOP_Z + 0.030,   # sphere radius 0.03
}

# During monitored descent, stop when TCP is within this Z-distance of object center
GRASP_THRESHOLD = 0.015   # 1.5 cm above the object center

# Step size and duration for the slow, monitored approach phase
APPROACH_STEP_M   = 0.020   # 2.0 cm per step
APPROACH_STEP_SEC = 0.80    # seconds per step


class PickAndPlaceNode(Node):
    def __init__(self, target_name):
        super().__init__('pick_and_place_node')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.target_name = target_name.lower().replace(" ", "_")
        self.target_frame = f"target_{self.target_name}"

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service / action clients
        self.ik_client      = self.create_client(GetPositionIK, '/compute_ik')
        self.arm_client     = ActionClient(self, FollowJointTrajectory,
                                           '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory,
                                           '/gripper_controller/follow_joint_trajectory')

    # ── Primitive: move arm TCP to (x, y, z) in base_link ───────────────────
    def move_to_pose(self, x, y, z, duration_sec=2.0):
        """
        Compute IK for tcp at (x,y,z) in base_link and execute the trajectory.
        duration_sec controls how long the controller has to complete the move.
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z

        # Gripper pointing straight down: quaternion = (x=0, y=1, z=0, w=0)
        req.ik_request.pose_stamped.pose.orientation.x = 0.0
        req.ik_request.pose_stamped.pose.orientation.y = 1.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 0.0

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp.error_code.val != 1:
            self.get_logger().error(
                f"IK Failed (code {resp.error_code.val}) for pose "
                f"({x:.3f}, {y:.3f}, {z:.3f})"
            )
            return False

        joint_names = resp.solution.joint_state.name
        joint_positions = resp.solution.joint_state.position

        arm_joints = ['joint_1','joint_2','joint_3','joint_4',
                      'joint_5','joint_6','joint_7']
        target_positions = []
        for j in arm_joints:
            idx = joint_names.index(j)
            target_positions.append(joint_positions[idx])

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = arm_joints

        pt = JointTrajectoryPoint()
        pt.positions = target_positions
        # Allow sub-second durations using nanoseconds
        full_secs  = int(duration_sec)
        nanosecs   = int((duration_sec - full_secs) * 1e9)
        pt.time_from_start = RosDuration(sec=full_secs, nanosec=nanosecs)
        goal_msg.trajectory.points.append(pt)

        self.arm_client.wait_for_server()
        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm trajectory goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(0.3)
        return True

    # ── Primitive: move gripper ──────────────────────────────────────────────
    def move_gripper(self, width):
        """
        Command both gripper fingers to joint position 'width'.
        Convention (matches SRDF named states):
            width = 0.0  → OPEN  (fingers fully apart, default/travel state)
            width = 0.04 → CLOSED (fingers pressed together, grasp state)
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_left_joint', 'gripper_right_joint']

        pt = JointTrajectoryPoint()
        pt.positions = [width, width]
        pt.time_from_start = RosDuration(sec=1, nanosec=0)
        goal_msg.trajectory.points.append(pt)

        self.gripper_client.wait_for_server()
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(0.4)

    # ── Layer 2: TCP distance monitor ────────────────────────────────────────
    def get_tcp_to_target_z_distance(self):
        """
        Returns the Z-axis distance (metres) between the arm's TCP link and
        the detected target TF frame, or None if the transform is unavailable.
        Since TCP Z points down, a positive Z means the target is below the TCP.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                "tcp",
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return tf.transform.translation.z
        except Exception:
            return None

    # ── Two-layer monitored descent ──────────────────────────────────────────
    def approach_with_monitoring(self, x, y, grasp_z):
        """
        Descend from (hover_z) toward (grasp_z) in small increments.
        After each step, check the TCP→target distance.
        Returns when either:
          (a) distance < GRASP_THRESHOLD  [proximity trigger]
          (b) grasp_z is reached          [geometry trigger / fallback]
        """
        hover_z   = grasp_z + 0.15
        current_z = hover_z

        self.get_logger().info(
            f"  Monitored descent: {hover_z:.3f} → {grasp_z:.3f} m  "
            f"(threshold={GRASP_THRESHOLD*100:.0f}cm, step={APPROACH_STEP_M*100:.0f}cm)"
        )

        current_z = hover_z
        while True:
            # Step down, but never below grasp_z
            current_z = max(current_z - APPROACH_STEP_M, grasp_z)
            ok = self.move_to_pose(x, y, current_z, duration_sec=APPROACH_STEP_SEC)
            if not ok:
                break

            # Spin once so TF buffer is fresh
            rclpy.spin_once(self, timeout_sec=0.05)

            # ── Layer 2: proximity check ────────────────────────────────────
            dist_z = self.get_tcp_to_target_z_distance()
            if dist_z is not None:
                self.get_logger().info(
                    f"    z={current_z:.3f}m  |  TCP→target Z={dist_z*100:.1f}cm"
                )
                if dist_z < GRASP_THRESHOLD:
                    self.get_logger().info(
                        f"  ✓ Proximity trigger at z={current_z:.3f}m  "
                        f"(dist_z={dist_z*100:.1f}cm < {GRASP_THRESHOLD*100:.0f}cm)"
                    )
                    return   # ← grasp here
            else:
                self.get_logger().warn(
                    f"    z={current_z:.3f}m  |  TCP→target TF unavailable, continuing..."
                )

            # ── Geometry trigger: reached bottom of range ───────────────────
            if current_z <= grasp_z:
                self.get_logger().info(
                    f"  ✓ Geometry trigger: reached grasp_z={grasp_z:.3f}m"
                )
                break

    # ── Full pick-and-place sequence ─────────────────────────────────────────
    def execute_pick_and_place(self, x, y, grasp_z):
    def execute_pick_and_place(self, x, y, grasp_z):
        self.get_logger().info("Initializing pick and place sequence.")
        self.get_logger().info(f"Target XY: ({x:.3f}, {y:.3f})")
        self.get_logger().info(f"Grasp Z: {grasp_z:.3f} m")
        self.get_logger().info(f"Proximity threshold: {GRASP_THRESHOLD*100:.0f} cm")

        # Gripper convention (from SRDF):
        #   0.0  = OPEN  (fingers fully apart — default/travel state)
        #   0.04 = CLOSED (fingers pressed together — grasp state)

        # 1. Ensure gripper is OPEN, then move to hover above object
        # 1. Hover
        self.get_logger().info("Opening gripper and moving to hover position.")
        self.move_gripper(0.0)
        self.move_to_pose(x, y, grasp_z + 0.15, duration_sec=4.0)

        # 2. Descend
        self.get_logger().info("Executing monitored descent.")
        self.approach_with_monitoring(x, y, grasp_z)

        # 3. Grasp
        self.get_logger().info("Closing gripper.")
        self.move_gripper(0.05)
        time.sleep(0.5)

        # 4. Lift
        self.get_logger().info("Lifting object.")
        self.move_to_pose(x, y, grasp_z + 0.35, duration_sec=6.0)

        # 5. Move
        self.get_logger().info("Moving to drop-off table.")
        drop_x, drop_y = 0.0, 0.7
        self.move_to_pose(drop_x, drop_y, grasp_z + 0.35, duration_sec=8.0)

        # 6. Place
        self.get_logger().info("Descending to place object.")
        self.move_to_pose(drop_x, drop_y, grasp_z + 0.02, duration_sec=4.0)

        # 7. Release
        self.get_logger().info("Opening gripper to release object.")
        self.move_gripper(0.0)
        time.sleep(0.5)

        # 8. Retract
        self.get_logger().info("Retracting arm.")
        self.move_to_pose(drop_x, drop_y, grasp_z + 0.35, duration_sec=4.0)

        # 9. Return
        self.get_logger().info("Returning to scan pose.")
        self.move_to_pose(0.65, 0.0, 0.55, duration_sec=5.0)

        self.get_logger().info("Pick and place sequence complete.")


# ── Main ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--target', type=str, required=True,
                        help="Target object name, e.g. 'Red Box'")
    parsed_args, _ = parser.parse_known_args()

    node = PickAndPlaceNode(parsed_args.target)

    # ── Step 1: Scan Pose ────────────────────────────────────────────────────
    # Move tcp above table centre so the eye-in-hand camera looks straight
    # down at all objects. X=0.65 aligns with the table, Z=0.55 gives enough
    # clearance for a wide field-of-view.
    node.get_logger().info("Moving to scan pose.")
    node.move_to_pose(0.65, 0.0, 0.55, duration_sec=4.0)
    time.sleep(2.0)   # let camera stabilise and vision node publish TF

    # ── Step 2: Wait for a valid TF ─────────────────────────────────────────
    node.get_logger().info(f"Waiting for TF frame: {node.target_frame} ...")
    target_tf = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            target_tf = node.tf_buffer.lookup_transform(
                "base_link",
                node.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            t = target_tf.transform.translation
            node.get_logger().info(
                f"  TF received: X={t.x:.3f}  Y={t.y:.3f}  Z={t.z:.3f}"
            )
            # Sanity check: within robot workspace
            if 0.3 < t.x < 1.1 and abs(t.y) < 0.5 and 0.05 < t.z < 0.5:
                break
            else:
                node.get_logger().warn("  TF out of workspace bounds, retrying...")
                target_tf = None
        except Exception:
            pass

    # ── Step 3: Execute ──────────────────────────────────────────────────────
    if target_tf:
        t = target_tf.transform.translation

        # Layer 1: known-surface Z (primary reliable source)
        grasp_z = OBJECT_GRASP_Z.get(node.target_name, TABLE_TOP_Z + 0.03)

        node.get_logger().info(
            f"Target located  —  XY from vision: ({t.x:.3f}, {t.y:.3f})"
            f"  |  Z from world geometry: {grasp_z:.3f} m"
        )
        node.execute_pick_and_place(t.x, t.y, grasp_z)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
