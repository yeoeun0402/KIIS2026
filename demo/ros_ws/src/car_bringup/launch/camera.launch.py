"""카메라 → /camera/image_raw/compressed (MJPG 그대로) · /camera/camera_info.

  ros2 launch car_bringup camera.launch.py                         # C920 1920x1080 30 fps, 초점 19 · 노출 156 고정
  ros2 launch car_bringup camera.launch.py size:=1280x720
  ros2 launch car_bringup camera.launch.py device:=/dev/video0 controls:=false   # 다른 카메라로 파이프라인만 시험
  ros2 launch car_bringup camera.launch.py driver:=usb_cam          # 비교용 (stamp 가 0.7~1 s 어긋남 — 쓰지 말 것)

driver
  c920    (기본) car_bringup/c920_camera — stamp = V4L2 드라이버 프레임 시각, 컨트롤 주기 점검
  usb_cam usb_cam 0.8.1 raw_mjpeg — demo/out/p1 에서 stamp 오류 확인됨
camera_info: config/camera_info_c920_<H>p.yaml (1080p 캘리브레이션 × 해상도 비). header.frame_id = camera_optical.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

C920 = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"


def _setup(context):
    cfg = os.path.join(get_package_share_directory("car_bringup"), "config")
    g = {k: LaunchConfiguration(k).perform(context) for k in ("size", "device", "fps", "controls", "focus", "exposure", "driver")}
    w, h = map(int, g["size"].lower().split("x"))
    controls = g["controls"].lower() in ("1", "true", "yes")
    info = os.path.join(cfg, f"camera_info_c920_{h}p.yaml")
    acts = []
    if not os.path.exists(g["device"]):
        acts.append(LogInfo(msg=f"[camera] 장치 없음: {g['device']} — USB 연결 확인"))
    if not os.path.exists(info):
        acts.append(LogInfo(msg=f"[camera] {h}p camera_info 없음 → K 비어 있음 (1080p/720p 만 준비됨)"))
        info = ""
    if g["driver"] == "usb_cam":
        params = [os.path.join(cfg, "usb_cam_c920.yaml"),
                  {"video_device": g["device"], "image_width": w, "image_height": h, "framerate": float(g["fps"]),
                   "camera_info_url": ("file://" + info) if info else ""}]
        if not controls:
            params.append({"autoexposure": True, "exposure": -1, "autofocus": False, "focus": -1})
        acts.append(Node(package="usb_cam", executable="usb_cam_node_exe", name="usb_cam", namespace="camera",
                         parameters=params, output="screen"))
        if controls:
            dev = g["device"]
            acts.append(TimerAction(period=3.0, actions=[ExecuteProcess(cmd=["v4l2-ctl", "-d", dev, "-c",
                        f"focus_automatic_continuous=0,focus_absolute={g['focus']},auto_exposure=1,exposure_time_absolute={g['exposure']}"])]))
        return acts
    acts.append(Node(package="car_bringup", executable="c920_camera", name="c920_camera", namespace="camera", output="screen",
                     parameters=[{"device": g["device"], "width": w, "height": h, "fps": int(float(g["fps"])),
                                  "camera_info_file": info, "set_controls": controls,
                                  "focus": int(g["focus"]), "exposure": int(g["exposure"])}]))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("size", default_value="1920x1080"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("device", default_value=C920),
        DeclareLaunchArgument("controls", default_value="true", description="C920 초점·노출 고정 (다른 카메라면 false)"),
        DeclareLaunchArgument("focus", default_value="19"),
        DeclareLaunchArgument("exposure", default_value="156"),
        DeclareLaunchArgument("driver", default_value="c920", description="c920 (기본) | usb_cam (비교용)"),
        OpaqueFunction(function=_setup),
    ])
