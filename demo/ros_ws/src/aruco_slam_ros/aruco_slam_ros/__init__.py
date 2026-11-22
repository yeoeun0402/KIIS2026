"""[demo] aruco_slam_ros — 이 패키지를 import 하는 순간 SLAM 용 파이썬 환경을 맞춘다.

ROS Humble 의 시스템 파이썬에는 OpenCV 4.5.4 / numpy 1.21 이 깔려 있지만, aruco-slam 은
OpenCV ≥ 4.7(cv2.aruco.ArucoDetector) · numpy 1.26 · scipy ≥ 1.14(from_quat scalar_first) 가 필요하다.
그래서 ~/aruco_slam_deps 를 sys.path 맨 앞에 넣는다 — numpy·cv2 를 import 하기 **전에** 해야 하므로
노드 모듈은 항상 이 패키지를 가장 먼저 import 한다. cv_bridge(시스템 OpenCV 로 빌드됨)는 쓰지 않는다.

환경변수로 바꿀 수 있음: ARUCO_SLAM_DEPS (기본 ~/aruco_slam_deps), ARUCO_SLAM_ROOT (기본 ~/demo/aruco-slam-fixed)
"""
import os
import sys

# EKF 행렬(상태 70~190차원)에서 OpenBLAS 멀티스레드는 오히려 느림: 마커 40개 기준 16.9 ms → 1스레드 5.3 ms (demo P2 측정).
# numpy 를 import 하기 전에 정해야 적용된다. 바꾸려면 실행 전에 환경변수로 지정.
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

DEPS = os.path.expanduser(os.environ.get("ARUCO_SLAM_DEPS", "~/aruco_slam_deps"))
SLAM_ROOT = os.path.expanduser(os.environ.get("ARUCO_SLAM_ROOT", "~/demo/aruco-slam-fixed"))

if "numpy" in sys.modules and not getattr(sys.modules["numpy"], "__file__", "").startswith(DEPS):
    # 이미 시스템 numpy 가 올라와 있으면 deps 의 cv2(numpy 1.26 빌드)가 깨진다 → 일찍, 분명하게 알린다
    sys.stderr.write("[aruco_slam_ros] 경고: numpy 가 먼저 import 됨 (%s). aruco_slam_ros 를 가장 먼저 import 할 것\n"
                     % sys.modules["numpy"].__file__)
for p in (SLAM_ROOT, DEPS):
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
sys.dont_write_bytecode = True
