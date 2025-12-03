# macOS arm64でPixiを使ってROS2を動かす
pixi_ros2_osx-arm64


## 環境
- Apple M2 Max
- macOS Sequoia 15.3.2
- pixi 0.59.0

- ROS2 Humble




## Webカメラの画像取得
RoboStack のパッケージ一覧を見ると、ros-humble-usb-cam は
- linux-64：✔
- linux-aarch64：✔
- win-64：❌
- osx-64：❌
- osx-arm64：❌

osx-arm64 で使える ros-humble-usb-cam はありません。

### OpenCVでカメラ画像の取得
```
pixi add opencv
python cam_test.py
```

### ROS2 ノードとして /image_raw を publish する（usb_cam の自前実装）
usb_cam パッケージの代わりに、自作の Python ノードで画像トピックを publish する。
```
pixi add opencv ros-humble-rclpy ros-humble-sensor-msgs
python webcam_publisher.py
```
```
ros2 topic list
ros2 topic echo /webcam/image_raw
```
