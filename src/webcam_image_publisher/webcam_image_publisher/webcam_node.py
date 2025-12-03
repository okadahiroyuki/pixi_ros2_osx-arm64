#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("webcam_publisher")

        # パラメータ宣言
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("frame_id", "webcam_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)

        self.camera_id = self.get_parameter("camera_id").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value

        # OpenCV カメラ初期化
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Webカメラ {self.camera_id} が開けませんでした")
            raise RuntimeError("Cannot open webcam")

        # 解像度設定（macOS では効かない場合もあります）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self.cap.set(cv2.CAP_PROP_FPS, float(self.fps))

        # Publisher
        self.image_pub = self.create_publisher(Image, "image_raw", 10)
        self.cinfo_pub = self.create_publisher(CameraInfo, "camera_info", 10)

        # タイマー（fps Hz）
        period = 1.0 / self.fps if self.fps > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"WebcamPublisher started: camera_id={self.camera_id}, "
            f"resolution={self.width}x{self.height}, fps={self.fps}"
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("フレーム取得に失敗しました")
            return

        # 実際のサイズを更新（ドライバ側で丸められることがある）
        height, width, channels = frame.shape
        self.width = width
        self.height = height

        now = self.get_clock().now().to_msg()

        # Image メッセージ
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id

        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = "bgr8"  # OpenCVデフォルト
        img_msg.is_bigendian = 0
        img_msg.step = width * channels
        img_msg.data = frame.tobytes()

        # CameraInfo メッセージ（簡易な pinhole モデル）
        cinfo = CameraInfo()
        cinfo.header = img_msg.header
        cinfo.width = width
        cinfo.height = height

        # 単純な「画素=1」の焦点距離（適当）:
        fx = fy = float(width)  # かなりテキトーだが、形だけ欲しいとき用
        cx = width / 2.0
        cy = height / 2.0

        cinfo.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        cinfo.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        cinfo.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # 歪みはとりあえず 0
        cinfo.distortion_model = "plumb_bob"
        cinfo.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Publish
        self.image_pub.publish(img_msg)
        self.cinfo_pub.publish(cinfo)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        self.get_logger().info("WebcamPublisher shutdown")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
