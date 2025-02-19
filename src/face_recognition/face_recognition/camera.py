#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # 建立 ROS 影像發布器
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # OpenCV 影像轉換工具
        self.bridge = CvBridge()

        # 開啟攝像機 (0 表示默認攝影機)
        self.cap = cv2.VideoCapture(0)

        # 設定影像發送頻率 (30 Hz)
        self.timer = self.create_timer(1.0 / 30, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            # 轉換 OpenCV 影像到 ROS 影像
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
            self.get_logger().info("發佈影像到 /camera/image_raw")
        else:
            self.get_logger().warn("無法讀取攝影機影像")

    def destroy_node(self):
        self.cap.release()  # 釋放攝影機
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
