#!/usr/bin/env python3
"""
tf_debug.py — Analytical TF verification for vision grasping system.

Cross-checks the camera's reported object positions against world SDF ground truth
and FK-derived camera world pose, to isolate whether errors come from:
  (a) wrong camera world position   (FK / URDF extrinsics bug)
  (b) wrong depth reading           (sensor encoding bug)
  (c) wrong pixel deprojection      (camera intrinsics bug)

Run while simulation is live (arm in scan pose):
    ros2 run seven_dof_arm_gazebo tf_debug.py

Output every second includes:
  - Camera world position (from FK via TF)
  - For each detected target:
      * Detected position in world
      * Expected position (from world SDF)
      * XY error
      * Vector from camera to object (lets you verify depth and angle)
"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.parameter import Parameter
import math

# World SDF ground-truth object positions in base_link (robot at world origin)
WORLD_TRUTH = {
    'red_box':       (0.650,  0.100, 0.125),
    'blue_cylinder': (0.750, -0.100, 0.150),
    'green_sphere':  (0.650, -0.200, 0.130),
}


class TFDebugNode(Node):
    def __init__(self):
        super().__init__('tf_debug_node')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.5, self.debug_callback)
        self.get_logger().info("TF Debug Node started. Printing analysis every 1.5s...")

    def lookup_xyz(self, parent, child):
        """Returns (x,y,z) of child origin in parent frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3))
            t = tf.transform.translation
            return t.x, t.y, t.z
        except Exception:
            return None

    def debug_callback(self):
        sep = "═" * 60
        self.get_logger().info("\n" + sep)

        # ── 1. Camera world position (pure FK from /joint_states) ────────────
        cam = self.lookup_xyz("base_link", "camera_link_optical")
        if cam:
            self.get_logger().info(
                f"  CAMERA  (FK→world) : X={cam[0]:.4f}  Y={cam[1]:.4f}  Z={cam[2]:.4f}"
            )
        else:
            self.get_logger().warn("  CAMERA TF unavailable (arm not spawned yet?)")
            return

        # ── 2. TCP world position (to verify IK accuracy) ────────────────────
        tcp = self.lookup_xyz("base_link", "tcp")
        if tcp:
            self.get_logger().info(
                f"  TCP     (FK→world) : X={tcp[0]:.4f}  Y={tcp[1]:.4f}  Z={tcp[2]:.4f}"
            )

        self.get_logger().info("  " + "─" * 58)
        self.get_logger().info(
            f"  {'Object':16s}  {'Det.XY':>14s}  {'SDF.XY':>14s}  {'XYerr':>7s}  {'Cam→Obj':>22s}"
        )
        self.get_logger().info("  " + "─" * 58)

        for name, (wx, wy, wz) in WORLD_TRUTH.items():
            det = self.lookup_xyz("base_link", f"target_{name}")
            if det is None:
                self.get_logger().info(
                    f"  {name:16s}  (not detected yet)"
                )
                continue

            # XY error vs world SDF
            err_x  = det[0] - wx
            err_y  = det[1] - wy
            err_xy = math.sqrt(err_x**2 + err_y**2)

            # Vector from camera to detected object (in world frame)
            # This is what the depth sensor should be reporting
            v = (det[0] - cam[0], det[1] - cam[1], det[2] - cam[2])
            depth_reported = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

            # Angle between camera→object vector and straight-down (-Z)
            # If camera is pointing straight down, this should be small
            angle_from_down_deg = math.degrees(
                math.acos(max(-1.0, min(1.0, -v[2] / depth_reported)))
            )

            # What SHOULD the cam→obj vector be using world SDF truth?
            v_truth = (wx - cam[0], wy - cam[1], wz - cam[2])
            depth_expected = math.sqrt(v_truth[0]**2 + v_truth[1]**2 + v_truth[2]**2)

            self.get_logger().info(
                f"  {name:16s}  "
                f"({det[0]:.3f},{det[1]:.3f})  "
                f"({wx:.3f},{wy:.3f})  "
                f"{err_xy*100:6.1f}cm  "
                f"depth={depth_reported:.3f}m(exp={depth_expected:.3f}m)  "
                f"angle={angle_from_down_deg:.1f}°"
            )
            # Detailed breakdown for easy diagnosis
            self.get_logger().info(
                f"    ↳ det-sdf: dX={err_x*100:+.1f}cm  dY={err_y*100:+.1f}cm  "
                f"dZ={(det[2]-wz)*100:+.1f}cm | "
                f"cam→obj vector: ({v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f})"
            )

        self.get_logger().info(sep)
        self.get_logger().info(
            "  LEGEND: XYerr = 2D error vs SDF truth | "
            "angle = deviation of cam-to-obj from straight down"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TFDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
