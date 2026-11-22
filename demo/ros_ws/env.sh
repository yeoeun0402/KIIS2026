# [demo] source ~/demo/ros_ws/env.sh — ROS 2 (Humble 또는 Jazzy, 설치된 것) + 이 작업공간 (원본 car_drive_exp 와 섞이지 않게 여기만)
if [ -n "${CONDA_PREFIX:-}" ]; then
  PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -e conda -e miniconda -e anaconda | paste -sd:)"; unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME
fi
export PYTHONNOUSERSITE=1
# 2026-11-21: Ubuntu 24.04(Jazzy) 지원 — 설치된 배포판을 찾음 (jazzy 우선, 없으면 humble)
# ROS setup.bash 는 set -u(없는 변수 = 오류) 상태에서 source 하면 죽음 → 잠깐 끄고 원래대로
case $- in *u*) _nu=1; set +u;; *) _nu=0;; esac
for _d in jazzy humble; do [ -f "/opt/ros/$_d/setup.bash" ] && { source "/opt/ros/$_d/setup.bash"; break; }; done; unset _d
[ -f "$HOME/demo/ros_ws/install/setup.bash" ] && source "$HOME/demo/ros_ws/install/setup.bash"
[ "$_nu" = 1 ] && set -u; unset _nu
export CAR_DRIVE_LOG="${CAR_DRIVE_LOG:-$HOME/demo/sessions}"
# SLAM 노드는 aruco_slam_ros/__init__.py 가 ~/aruco_slam_deps 를 sys.path 앞에 넣는다 (PYTHONPATH 불필요)
