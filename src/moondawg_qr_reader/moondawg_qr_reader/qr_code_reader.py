#!/usr/bin/env python3
"""
qr_code_reader.py  –  ROS 2 Jazzy node:
    • subscribes to /image_raw (sensor_msgs/Image)
    • detects and decodes a QR code with OpenCV
    • publishes the text on /qr_code (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')

        self.bridge   = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_cb, 10)
        self.publisher = self.create_publisher(String, '/qr_code', 10)

        self.get_logger().info('QR-Code reader ready (sub=/image_raw pub=/qr_code)')

    # ------------------------------------------------------------ #
    def image_cb(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        text, bbox, _ = self.detector.detectAndDecode(frame)

        if text:                                   # non-empty string → QR found
            self.get_logger().info(f'QR detected: "{text}"')
            self.publisher.publish(String(data=text))


def main():
    rclpy.init()
    node = QRCodeReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
