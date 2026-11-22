"""demo 공용: 원본 aruco-slam 코드를 '읽기 전용'으로 import 하고, 상대경로(calibration/…)는 demo/work 로 돌린다.

원본 레포 파일은 절대 쓰지 않는다 — 출력은 전부 demo/out/ 아래.
"""
import os
import sys

HOME = os.path.expanduser("~")
DEMO = os.path.join(HOME, "demo")
WORK = os.path.join(DEMO, "work")          # calibration/camera_matrix.npy 등의 사본이 여기
OUT = os.path.join(DEMO, "out")
ARUCO = os.path.join(HOME, "aruco-slam")
CALIB = os.path.join(HOME, "car_calibration")

for p in (os.path.join(HOME, "aruco_slam_deps"), os.path.join(ARUCO, "thirdparty", "pangolin"), ARUCO):
    if p not in sys.path:
        sys.path.insert(0, p)

os.chdir(WORK)                 # BaseFilter 가 "calibration/camera_matrix.npy" 를 상대경로로 읽는다
os.makedirs(OUT, exist_ok=True)
sys.dont_write_bytecode = True  # 원본 레포에 __pycache__ 를 새로 만들지 않게
