#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            1)
        self.bridge = CvBridge()
        

    def listener_callback(self, msg):
        # 이미지를 OpenCV 포맷으로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # 이미지를 파일로 저장
        cv2.imwrite('/home/dlee/Pictures/saved_image.png', cv_image)
        

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
