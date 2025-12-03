#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("webcam_publisher")
        self.publisher_ = self.create_publisher(Image, "/webcam/image_raw", 10)

        # カメラを開く（0はデフォルトカメラ）
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Webカメラが開けませんでした")
            raise RuntimeError("Cannot open webcam")

        # 30fps 相当
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("フレーム取得に失敗しました")
            return

        # OpenCVのデフォルトは BGR8
        height, width, channels = frame.shape

        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "webcam_frame"

        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * channels
        msg.data = frame.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
