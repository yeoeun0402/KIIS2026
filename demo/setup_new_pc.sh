#!/usr/bin/env bash
# [demo] 새 PC 설치 — Ubuntu 22.04 + ROS 2 Humble, 또는 Ubuntu 24.04 + ROS 2 Jazzy (2026-11-21 추가)
#
#   cd ~ && unzip -o demo_bundle_*.zip && ~/demo/setup_new_pc.sh
#
# 하는 일: 시스템 패키지 확인/설치 → ~/aruco_slam_deps (SLAM 전용 파이썬 패키지, 이 PC 와 같은 버전) →
#          dialout 그룹 → 작업공간 빌드 + 테스트 → 환경 점검 → 현장 준비 점검
# 인터넷 필요 (apt, pip). 원래 PC 의 원본 레포(~/aruco-slam, ~/car_drive_exp)는 필요 없음.
set -u
red() { printf '\033[31m✗ %s\033[0m\n' "$*"; }; grn() { printf '\033[32m✓ %s\033[0m\n' "$*"; }; ylw() { printf '\033[33m! %s\033[0m\n' "$*"; }
cd "$HOME" || exit 1
[ -d "$HOME/demo/ros_ws/src" ] || { red "~/demo/ros_ws/src 없음 — zip 을 홈(~)에서 풀었는지 확인"; exit 1; }

echo "━━ 1. OS · ROS ━━"
. /etc/os-release
case "$VERSION_ID" in
  22.04) DISTRO=humble ;;
  24.04) DISTRO=jazzy ;;
  *) red "Ubuntu $VERSION_ID — 22.04(Humble) 또는 24.04(Jazzy)만 지원"; exit 1 ;;
esac
grn "Ubuntu $VERSION_ID → ROS 2 $DISTRO · $(python3 -V)"
if [ -f "/opt/ros/$DISTRO/setup.bash" ]; then grn "ROS 2 $DISTRO 설치돼 있음"
else
  # 공식 설치 방법 (https://docs.ros.org/en/$DISTRO/Installation/Ubuntu-Install-Debs.html): ros2-apt-source 패키지로 저장소·키 등록
  ylw "ROS 2 $DISTRO 없음 → 설치 (ros-$DISTRO-desktop, 수 GB · 수십 분)"
  sudo apt-get update && sudo apt-get install -y software-properties-common curl || { red "apt 실패"; exit 1; }
  sudo add-apt-repository -y universe
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  [ -n "$V" ] || { red "ros-apt-source 버전을 못 가져옴 (인터넷 확인)"; exit 1; }
  curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${VERSION_CODENAME}_all.deb" || { red "ros2-apt-source 다운로드 실패"; exit 1; }
  sudo dpkg -i /tmp/ros2-apt-source.deb || { red "ros2-apt-source 설치 실패"; exit 1; }
  sudo apt-get update && sudo apt-get install -y "ros-$DISTRO-desktop" || { red "ROS 2 설치 실패"; exit 1; }
  grn "ROS 2 $DISTRO 설치함"
fi

echo "━━ 2. 시스템 패키지 ━━"
PKGS="python3-serial python3-sympy python3-yaml python3-matplotlib python3-pip python3-colcon-common-extensions
      socat ffmpeg v4l-utils ros-$DISTRO-rviz2 ros-$DISTRO-tf2-ros ros-$DISTRO-rosbag2-storage-default-plugins
      ros-$DISTRO-rosidl-default-generators ros-$DISTRO-nav-msgs ros-$DISTRO-visualization-msgs"
MISS=""
for p in $PKGS; do dpkg -s "$p" >/dev/null 2>&1 || MISS="$MISS $p"; done
if [ -n "$MISS" ]; then
  ylw "없는 패키지:$MISS"
  sudo apt-get update && sudo apt-get install -y $MISS || { red "apt 설치 실패"; exit 1; }
fi
grn "시스템 패키지 OK"

echo "━━ 3. ~/aruco_slam_deps (SLAM 전용, 시스템 파이썬과 분리) ━━"
# 원래 PC 와 같은 버전 고정. --no-deps: 시스템 패키지를 건드리지 않게 필요한 것만 정확히
DEPS="dill==0.3.9 numpy==1.26.4 opencv-contrib-python==4.10.0.84 PyOpenGL==3.1.7
      pyparsing==3.3.3 scipy==1.14.1 typing_extensions==4.12.2"
if PYTHONPATH="$HOME/aruco_slam_deps" python3 -c "import cv2, numpy, scipy, dill; assert cv2.__version__.startswith('4.10') and numpy.__version__ == '1.26.4'" 2>/dev/null; then
  grn "이미 설치됨"
else
  python3 -m pip install --no-deps --upgrade --target "$HOME/aruco_slam_deps" $DEPS || { red "pip 설치 실패"; exit 1; }
  grn "설치함 → $HOME/aruco_slam_deps"
fi
# gtsam 은 factor_graph 필터(검증 안 됨)에만 씀 → 실패해도 계속 (python 3.12 용 4.2 휠이 없을 수 있음)
PYTHONPATH="$HOME/aruco_slam_deps" python3 -c "import gtsam" 2>/dev/null || \
  python3 -m pip install --no-deps --target "$HOME/aruco_slam_deps" gtsam==4.2.1 >/dev/null 2>&1 || \
  ylw "gtsam 설치 안 됨 (factor_graph 필터만 못 씀, EKF·주행은 상관없음)"

echo "━━ 4. 시리얼 권한 ━━"
if id -nG | grep -qw dialout; then grn "dialout 그룹"
else sudo usermod -aG dialout "$USER" && ylw "dialout 추가함 — 로그아웃/재로그인 후 적용"; fi

echo "━━ 5. 빌드 + 테스트 ━━"
rm -rf "$HOME/demo/ros_ws/build" "$HOME/demo/ros_ws/install" "$HOME/demo/ros_ws/log"
"$HOME/demo/ros_ws/build.sh" --test 2>&1 | grep -E "Summary|failed|error" | tail -4
source "$HOME/demo/ros_ws/env.sh"
ros2 run aruco_slam_ros env_check >/dev/null 2>&1 && grn "SLAM 환경 점검 통과" || { red "env_check 실패 → ros2 run aruco_slam_ros env_check"; }

echo "━━ 6. 현장 준비 점검 ━━"
"$HOME/demo/ros_ws/field_check.sh"
echo
echo "다음: 명령어 모음 ~/demo/ros_ws/COMMANDS.md · 계획/기록 ~/demo/PLAN_ROS.md · 보고서·체크리스트 ~/demo/REPORT.md"
echo "하드웨어 없는 시험(복도 영상 필요: ~/car_calibration/corridor_map/videos/corridor_mapping_01.mkv, zip 에 없음)"
echo "  ~/demo/ros_ws/p3_drive_check.sh q · ~/demo/ros_ws/p5_calib_check.sh"
