"""SLAM 묶음: 영상 소스 + aruco_slam_node + 정적 TF (base_link → camera_link → camera_optical).

  # 카메라 없이 녹화 영상으로 (영상 끝나면 전체 종료)
  ros2 launch car_bringup slam.launch.py source:=video video:=<mkv> [rate:=1.0] [out_dir:=<dir>]
  # 실제 C920
  ros2 launch car_bringup slam.launch.py source:=camera [size:=1920x1080]
  # bag 재생: ros2 launch car_bringup slam.launch.py source:=none  +  ros2 bag play <bag> --topics /camera/image_raw/compressed /camera/camera_info
  # 공통: mount_file:=<yaml> (기본 config/mount.yaml) · decode_gray:=true · landmarks:=<csv> (트랙 정합) · rviz:=true
  #       state:=true (car_state → /car/state [x y θ v δ]) · steer_calib:=<yaml> (기본 config/steer_calib.yaml)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory("car_bringup")
    cfg = os.path.join(share, "config")
    g = {k: LaunchConfiguration(k).perform(context)
         for k in ("source", "video", "rate", "size", "mount_file", "out_dir", "decode_gray", "reliable", "landmarks", "rviz", "align_yaw_only", "state", "steer_calib")}
    mount = g["mount_file"] or os.path.join(cfg, "mount.yaml")
    acts = [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "frames.launch.py")),
                                     launch_arguments={"mount_file": mount}.items())]     # 노드와 같은 장착값
    if g["source"] == "video":
        video = os.path.expanduser(g["video"])
        # 영상 해상도에 맞는 camera_info
        import json
        import subprocess
        s = json.loads(subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                                       "stream=width,height", "-of", "json", video], capture_output=True, text=True).stdout)["streams"][0]
        info = os.path.join(cfg, f"camera_info_c920_{s['height']}p.yaml")
        src = Node(package="aruco_slam_ros", executable="video_publisher", name="video_publisher", namespace="camera",
                   output="screen", parameters=[{"video": video, "rate": float(g["rate"]),
                                                 "camera_info_file": info if os.path.exists(info) else ""}])
        acts += [src, RegisterEventHandler(OnProcessExit(target_action=src, on_exit=[EmitEvent(event=Shutdown(reason="영상 끝"))]))]
    elif g["source"] == "camera":
        acts.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "camera.launch.py")),
                                             launch_arguments={"size": g["size"]}.items()))
    acts.append(Node(package="aruco_slam_ros", executable="aruco_slam_node", name="aruco_slam", output="screen",
                     parameters=[{"mount_file": mount, "out_dir": g["out_dir"],
                                  "decode_gray": g["decode_gray"].lower() in ("1", "true", "yes"),
                                  "reliable": g["reliable"].lower() in ("1", "true", "yes")}]))
    lmk = os.path.expanduser(g["landmarks"]) if g["landmarks"] else ""
    if lmk:
        acts.append(Node(package="aruco_slam_ros", executable="track_aligner", name="track_aligner", output="screen",
                         parameters=[{"landmarks": lmk, "yaw_only": g["align_yaw_only"].lower() in ("1", "true", "yes")}]))
    if g["state"].lower() in ("1", "true", "yes"):
        # 차량 상태 [x, y, θ, v, δ] — 트랙 정합 있으면 트랙 좌표, 없으면 map
        sc = os.path.expanduser(g["steer_calib"]) if g["steer_calib"] else os.path.join(cfg, "steer_calib.yaml")
        acts.append(Node(package="car_state", executable="car_state_node", name="car_state", output="screen",
                         parameters=[{"pose_topic": "/car/pose_track" if lmk else "/car/pose", "mount_file": mount,
                                      "steer_calib": sc if os.path.exists(sc) else ""}]))
    if g["rviz"].lower() in ("1", "true", "yes"):
        acts.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "rviz.launch.py")),
                                             launch_arguments={"fixed_frame": "track" if lmk else "map"}.items()))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="camera", description="camera | video | none (영상은 밖에서, 예: ros2 bag play)"),
        DeclareLaunchArgument("video", default_value=""),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("size", default_value="1920x1080"),
        DeclareLaunchArgument("mount_file", default_value=""),
        DeclareLaunchArgument("out_dir", default_value=""),
        DeclareLaunchArgument("decode_gray", default_value="true"),
        DeclareLaunchArgument("landmarks", default_value="", description="실측 마커 CSV → track_aligner (TF track → map)"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("align_yaw_only", default_value="false", description="장착 pitch/roll 실측 후 true 권장"),
        DeclareLaunchArgument("state", default_value="true", description="car_state 노드 (/car/state · /car/odom)"),
        DeclareLaunchArgument("steer_calib", default_value="", description="조향 보정 yaml (기본 config/steer_calib.yaml, 없으면 δ=NaN)"),
        DeclareLaunchArgument("reliable", default_value="true", description="SLAM 구독 QoS reliable(기본)/best-effort"),
        OpaqueFunction(function=_setup),
    ])
