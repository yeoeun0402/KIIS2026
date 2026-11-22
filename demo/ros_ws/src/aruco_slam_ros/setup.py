from setuptools import find_packages, setup

package_name = "aruco_slam_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    description="[demo] ArUco EKF SLAM ROS 2 노드",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "env_check = aruco_slam_ros.env_check:main",
            "cam_probe = aruco_slam_ros.cam_probe:main",
            "video_publisher = aruco_slam_ros.video_publisher:main",
            "aruco_slam_node = aruco_slam_ros.aruco_slam_node:main",
            "track_aligner = aruco_slam_ros.track_aligner:main",
            "bag_export = aruco_slam_ros.bag_export:main",
        ],
    },
)
