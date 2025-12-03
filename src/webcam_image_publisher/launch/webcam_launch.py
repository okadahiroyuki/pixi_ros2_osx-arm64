from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="webcam_image_publisher",
            executable="webcam_node",
            name="webcam_node",
            output="screen",
            parameters=[
                {
                    "camera_id": 0,
                    "frame_id": "webcam_frame",
                    "width": 640,
                    "height": 480,
                    "fps": 30.0,
                }
            ],
        )
    ])
