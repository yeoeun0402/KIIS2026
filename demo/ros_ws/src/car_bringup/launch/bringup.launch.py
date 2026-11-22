"""주행 중 실시간 로컬라이제이션 묶음: 영상 소스 + SLAM + 정적 TF + ros2 bag 녹화. (주행 노드는 drive_ros.sh 가 전경에서 따로)

  ros2 launch car_bringup bringup.launch.py run_dir:=<dir> [source:=camera|video] [video:=<mkv>] [record:=true]

run_dir/
  bag/    ros2 bag (sqlite3): 영상(MJPG 그대로)·camera_info·자세·마커·주행 명령·TF  ← 나중에 그대로 다시 틀 수 있음
  slam/   aruco_slam_node 기록 (trajectory_ros.csv · map.txt · summary.json)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BAG_TOPICS = [
    "/camera/image_raw/compressed", "/camera/camera_info",          # 입력 (다시 틀면 SLAM 재현)
    "/car/pose", "/car/cam_pose", "/aruco/visible", "/aruco/markers", "/aruco_slam/stats",   # SLAM 출력
    "/car/pose_track", "/track/align", "/track/landmarks",                                 # 트랙 정합 (landmarks 줄 때)
    "/car/pwm", "/car/steer_cmd", "/car/steering_pot", "/car/board_state",                 # 주행 명령·보드 응답
    "/car/wheel_encoder",                                                                  # 뒷바퀴 엔코더 누적 카운트
    "/car/state", "/car/odom",                                                             # 차량 상태 [x y θ v δ] (car_state)
    "/tf", "/tf_static",
]


def _setup(context):
    share = get_package_share_directory("car_bringup")
    g = {k: LaunchConfiguration(k).perform(context)
         for k in ("run_dir", "source", "video", "rate", "size", "mount_file", "record", "landmarks", "rviz", "steer_calib")}
    run = os.path.expanduser(g["run_dir"])
    os.makedirs(run, exist_ok=True)
    acts = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "slam.launch.py")),
        launch_arguments={"source": g["source"], "video": g["video"], "rate": g["rate"], "size": g["size"],
                          "mount_file": g["mount_file"], "out_dir": os.path.join(run, "slam"),
                          "landmarks": g["landmarks"], "rviz": g["rviz"],
                          "steer_calib": g["steer_calib"]}.items())]
    if g["record"].lower() in ("1", "true", "yes"):
        # 발행 QoS(reliable)에 맞춰 구독 → 영상 유실 없음. 이미 압축된 MJPG 라 bag 압축은 하지 않음
        acts.append(ExecuteProcess(cmd=["ros2", "bag", "record", "-o", os.path.join(run, "bag")] + BAG_TOPICS,
                                   output="screen", sigterm_timeout="10", sigkill_timeout="10"))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("run_dir", default_value=os.path.expanduser("~/demo/sessions/manual")),
        DeclareLaunchArgument("source", default_value="camera"),
        DeclareLaunchArgument("video", default_value=""),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("size", default_value="1920x1080"),
        DeclareLaunchArgument("mount_file", default_value=""),
        DeclareLaunchArgument("record", default_value="true"),
        DeclareLaunchArgument("landmarks", default_value=""),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("steer_calib", default_value=""),
        OpaqueFunction(function=_setup),
    ])
