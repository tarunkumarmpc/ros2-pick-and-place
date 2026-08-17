#!/usr/bin/env python3
import sys
import subprocess
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSizePolicy
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont

class PickPlaceGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Operator Dashboard")
        self.resize(800, 700)

        # Main Layout
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # Title
        self.title_label = QLabel("Camera Feed & Object Selection")
        font = QFont("Arial", 16, QFont.Bold)
        self.title_label.setFont(font)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.title_label)

        # Video Frame
        self.video_label = QLabel("Waiting for camera feed...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.layout.addWidget(self.video_label)

        # Buttons Layout
        self.button_layout = QHBoxLayout()
        self.layout.addLayout(self.button_layout)

        # Buttons
        self.btn_red = QPushButton("Pick Red Box")
        self.btn_red.setStyleSheet("background-color: #ff4c4c; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
        self.btn_red.clicked.connect(lambda: self.run_pick_and_place("Red Box"))
        self.button_layout.addWidget(self.btn_red)

        self.btn_blue = QPushButton("Pick Blue Cylinder")
        self.btn_blue.setStyleSheet("background-color: #4c4cff; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
        self.btn_blue.clicked.connect(lambda: self.run_pick_and_place("Blue Cylinder"))
        self.button_layout.addWidget(self.btn_blue)

        self.btn_green = QPushButton("Pick Green Sphere")
        self.btn_green.setStyleSheet("background-color: #4cff4c; color: black; font-weight: bold; padding: 15px; font-size: 14px;")
        self.btn_green.clicked.connect(lambda: self.run_pick_and_place("Green Sphere"))
        self.button_layout.addWidget(self.btn_green)

        # Status Label
        self.status_label = QLabel("Status: Ready")
        self.status_label.setAlignment(Qt.AlignCenter)
        font_status = QFont("Arial", 12)
        self.status_label.setFont(font_status)
        self.layout.addWidget(self.status_label)

        # ROS 2 Node setup
        rclpy.init()
        self.node = Node('gui_pick_place_node')
        self.bridge = CvBridge()
        self.sub = self.node.create_subscription(
            Image,
            '/vision/annotated_image',
            self.image_callback,
            10
        )

        # Process running state
        self.current_process = None

        # Timer to spin ROS 2
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(30)  # ~33Hz

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # Check if process has finished
        if self.current_process is not None:
            ret = self.current_process.poll()
            if ret is not None:
                self.current_process = None
                self.status_label.setText("Status: Ready")
                self.enable_buttons(True)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convert to QImage
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w
            q_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            # Scale to fit label
            pixmap = QPixmap.fromImage(q_img)
            self.video_label.setPixmap(pixmap.scaled(
                self.video_label.width(), 
                self.video_label.height(), 
                Qt.KeepAspectRatio
            ))
        except Exception as e:
            self.node.get_logger().error(f"Failed to process image: {e}")

    def enable_buttons(self, enabled):
        self.btn_red.setEnabled(enabled)
        self.btn_blue.setEnabled(enabled)
        self.btn_green.setEnabled(enabled)
        if not enabled:
            self.btn_red.setStyleSheet("background-color: gray; color: white; padding: 15px;")
            self.btn_blue.setStyleSheet("background-color: gray; color: white; padding: 15px;")
            self.btn_green.setStyleSheet("background-color: gray; color: white; padding: 15px;")
        else:
            self.btn_red.setStyleSheet("background-color: #ff4c4c; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
            self.btn_blue.setStyleSheet("background-color: #4c4cff; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
            self.btn_green.setStyleSheet("background-color: #4cff4c; color: black; font-weight: bold; padding: 15px; font-size: 14px;")

    def run_pick_and_place(self, target_name):
        if self.current_process is not None:
            return
            
        self.status_label.setText(f"Status: Executing pick and place sequence for {target_name}")
        self.enable_buttons(False)
        
        # Launch the pick and place script as a background subprocess
        self.current_process = subprocess.Popen([
            'ros2', 'run', 'seven_dof_arm_gazebo', 'pick_and_place.py',
            '--target', target_name
        ])

    def closeEvent(self, event):
        if self.current_process is not None:
            self.current_process.terminate()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = PickPlaceGUI()
    gui.show()
    sys.exit(app.exec_())
