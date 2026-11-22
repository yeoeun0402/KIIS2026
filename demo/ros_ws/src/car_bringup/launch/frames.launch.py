"""정적 TF: base_link → camera_link (장착 실측) → camera_optical (고정 축 회전).

  ros2 launch car_bringup frames.launch.py                       # config/mount.yaml 값
  ros2 launch car_bringup frames.launch.py x:=0.28 z:=0.14 pitch_down_deg:=12
  ros2 launch car_bringup frames.launch.py mount_file:=<yaml>
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

KEYS = ("x", "y", "z", "pitch_down_deg", "yaw_deg", "roll_deg")


def _nodes(context):
    f = LaunchConfiguration("mount_file").perform(context) or os.path.join(
        get_package_share_directory("car_bringup"), "config", "mount.yaml")
    cfg = yaml.safe_load(open(os.path.expanduser(f))) or {}
    v = {}
    for k in KEYS:                                   # 인자로 준 값 > 파일 값 > 0
        arg = LaunchConfiguration(k).perform(context)
        v[k] = float(arg) if arg != "" else float(cfg.get(k, 0.0))
    # static_transform_publisher: --yaw/--pitch/--roll 은 라디안, ROS 피치 + = 아래로 숙임
    mount = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_base_to_camera",
                 arguments=["--x", str(v["x"]), "--y", str(v["y"]), "--z", str(v["z"]),
                            "--yaw", str(math.radians(v["yaw_deg"])), "--pitch", str(math.radians(v["pitch_down_deg"])),
                            "--roll", str(math.radians(v["roll_deg"])),
                            "--frame-id", "base_link", "--child-frame-id", "camera_link"])
    # camera_link(x 앞 y 왼 z 위) → camera_optical(x 오른 y 아래 z 앞): REP-103 표준 rpy (−90°, 0, −90°)
    optical = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_camera_to_optical",
                   arguments=["--roll", str(-math.pi / 2), "--pitch", "0", "--yaw", str(-math.pi / 2),
                              "--frame-id", "camera_link", "--child-frame-id", "camera_optical"])
    return [mount, optical]


def generate_launch_description():
    # 기본값은 config/mount.yaml. mount_file:=<yaml> 이면 그 파일 (SLAM 노드와 같은 파일을 써야 TF 와 /car/pose 가 맞음)
    return LaunchDescription([DeclareLaunchArgument("mount_file", default_value="")]
                             + [DeclareLaunchArgument(k, default_value="") for k in KEYS]
                             + [OpaqueFunction(function=_nodes)])
