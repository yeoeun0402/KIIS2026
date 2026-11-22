"""rviz2 (car.rviz): 궤적 · 차량 자세 · TF · SLAM 마커 · 실측 마커 · 카메라(검출 표시).

  ros2 launch car_bringup rviz.launch.py [fixed_frame:=map|track]
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    src = os.path.join(get_package_share_directory("car_bringup"), "rviz", "car.rviz")
    frame = LaunchConfiguration("fixed_frame").perform(context)
    fd, cfg = tempfile.mkstemp(prefix="car_", suffix=".rviz")
    with os.fdopen(fd, "w") as f:
        f.write(open(src).read().replace("__FIXED_FRAME__", frame))
    return [Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", cfg], output="log")]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("fixed_frame", default_value="map"), OpaqueFunction(function=_setup)])
