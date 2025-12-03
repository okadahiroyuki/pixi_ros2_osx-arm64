from setuptools import setup

package_name = "webcam_image_publisher"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/webcam_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hiroyuki Okada",
    maintainer_email="okdhryk@gmail.com",
    description="Simple webcam image publisher using OpenCV and rclpy",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "webcam_node = webcam_image_publisher.webcam_node:main",
        ],
    },
)
