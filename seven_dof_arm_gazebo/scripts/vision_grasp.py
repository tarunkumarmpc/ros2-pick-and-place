#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionGraspNode(Node):
    def __init__(self):
        super().__init__('vision_grasp_node')
        self.bridge = CvBridge()
        
        self.get_logger().info("Vision grasp node initialized.")

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth_image', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        # Publishers & Broadcasters
        self.image_pub = self.create_publisher(Image, '/vision/annotated_image', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.cv_image = None
        self.depth_image = None
        self.camera_info = None

    def info_callback(self, msg):
        # We only need the camera intrinsics once
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("Received camera intrinsic matrix.")

    def depth_callback(self, msg):
        # Gazebo Harmonic RGBD sensor publishes depth as 32-bit float in METERS (32FC1)
        # Explicitly request this encoding - passthrough can misinterpret the format
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            return

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # Define HSV color ranges for Red, Blue, Green
        color_ranges = {
            'Red Box': [(0, 120, 70), (10, 255, 255)],
            'Blue Cylinder': [(100, 150, 0), (140, 255, 255)],
            'Green Sphere': [(36, 25, 25), (86, 255, 255)]
        }

        # Z-axis depth offsets for volumetric centering
        depth_offsets = {
            'Red Box': 0.025,
            'Blue Cylinder': 0.050,
            'Green Sphere': 0.030
        }

        # Process object contours and broadcast TF frames
        for label, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_contour = None
            max_area = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500 and area > max_area:
                    max_area = area
                    best_contour = contour
            
            if best_contour is not None:
                # Calculate 2D bounding box centroid
                x, y, w, h = cv2.boundingRect(best_contour)
                u, v = x + w//2, y + h//2

                # Annotate image frame
                cv2.rectangle(self.cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(self.cv_image, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(self.cv_image, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Extract depth value
                depth = float(self.depth_image[v, u])
                if not np.isnan(depth) and depth > 0.0 and depth <= 8.0:
                    # Deproject to 3D Cartesian space
                    fx, fy = self.camera_info.k[0], self.camera_info.k[4]
                    cx, cy = self.camera_info.k[2], self.camera_info.k[5]

                    Z = depth + depth_offsets[label]
                    X = (u - cx) * Z / fx
                    Y = (v - cy) * Z / fy

                    # Broadcast TF frame
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = "camera_link_optical"
                    t.child_frame_id = "target_" + label.lower().replace(" ", "_")
                    t.transform.translation.x = X
                    t.transform.translation.y = Y
                    t.transform.translation.z = Z
                    t.transform.rotation.w = 1.0

                    self.tf_broadcaster.sendTransform(t)

        # Publish the fully annotated image with all detected objects
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = VisionGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
