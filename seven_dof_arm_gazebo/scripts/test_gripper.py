#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys

class TestGripperNode(Node):
    def __init__(self, action):
        super().__init__('test_gripper_node')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        self.action = action
        self.get_logger().info(f'Preparing to {action} the gripper...')
        self.timer = self.create_timer(1.0, self.send_trajectory)
        self.timer_called = False

    def send_trajectory(self):
        if self.timer_called:
            return
        self.timer_called = True

        msg = JointTrajectory()
        msg.joint_names = ['gripper_left_joint', 'gripper_right_joint']

        point = JointTrajectoryPoint()
        
        # 0.0 is fully open, 0.04 is fully closed
        if self.action == 'open':
            point.positions = [0.0, 0.0]
        else:
            point.positions = [0.04, 0.04]
            
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)

        self.get_logger().info(f'Commanding gripper to {self.action} in Gazebo!')
        self.publisher_.publish(msg)
        
        # Exit cleanly
        self.create_timer(1.0, lambda: sys.exit(0))

def main(args=None):
    if len(sys.argv) < 2 or sys.argv[1] not in ['open', 'close']:
        print("Usage: ./test_gripper.py [open|close]")
        sys.exit(1)
        
    rclpy.init(args=args)
    node = TestGripperNode(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
