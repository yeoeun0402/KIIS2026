"""[demo 수정본] 녹화 영상으로 SLAM 실행 — 2D/3D 화면은 옵션, 결과는 --out 폴더에 (원본 outputs/ 를 덮어쓰지 않음).

  cd ~/demo/aruco-slam-fixed
  PYTHONPATH=~/aruco_slam_deps:~/aruco-slam/thirdparty/pangolin python3 -B -m main.run_slam \\
      --video <영상> [--filter ekf] [--out 폴더] [--no-2d] [--3d] [--save-2d] [--save-3d] [--max-frames N]

원본과 다른 점
  * EKF.add_marker 회전 규약 수정 (filters/extended_kalman_filter.py)
  * 결과(trajectory.txt · map.txt · 내보낸 영상)를 --out 에 저장. 기본 ~/demo/out/slam_<영상이름>_<시각>/
  * 어느 폴더에서 실행해도 calibration/ 을 이 레포 기준으로 읽는다
  * 720p 영상이면 1080p 로 키워서 처리 (원본과 같음 — 720p 는 1080p 와 같은 화각, demo T8)
  * trajectory.txt 열 순서는 원본 그대로: t x y z qw qx qy qz (TUM 표준 qx qy qz qw 아님)
"""

import argparse
import datetime as dt
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import tqdm  # noqa: E402

IMAGE_SIZE = 1920, 1080
np.set_printoptions(precision=7, suppress=True)


def init_tracker(filter_type: str, initial_pose: np.ndarray):
    from filters.ekf_with_rotations import EKF_Rotations
    from filters.extended_kalman_filter import EKF
    from filters.factor_graph import FactorGraph
    if filter_type == "ekf":
        return EKF(initial_pose)
    if filter_type == "ekf_rotations":
        print("[경고] ekf_rotations 는 demo 에서 수정·검증하지 않았다 (원본과 같은 회전 버그가 있음)", file=sys.stderr)
        return EKF_Rotations(initial_pose)
    if filter_type == "factorgraph":
        print("[경고] factorgraph 는 demo 에서 검증하지 않았다", file=sys.stderr)
        return FactorGraph(initial_pose)
    raise ValueError(f"Unknown filter type: {filter_type}")


def main(a: argparse.Namespace) -> None:
    video = str(Path(a.video).expanduser().resolve())
    out = Path(a.out).expanduser().resolve() if a.out else Path.home() / "demo" / "out" / (
        f"slam_{Path(video).stem}_{dt.datetime.now():%Y%m%d_%H%M%S}")
    out.mkdir(parents=True, exist_ok=True)
    os.chdir(ROOT)                                   # calibration/ · outputs/images/ 상대경로 기준
    (ROOT / "outputs" / "images").mkdir(parents=True, exist_ok=True)

    import viewers.viewer_2d as v2d
    import viewers.viewer_3d as v3d
    from outputs.trajectory_writer import TrajectoryWriter

    tracker = init_tracker(a.filter, np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        sys.exit(f"영상을 열 수 없음: {video}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or None

    viewer3d = v3d.Viewer3D(export_video=a.save_3d) if a.show_3d else None
    viewer2d = v2d.Viewer2D(export_video=a.save_2d) if a.show_2d else None

    n = det_frames = 0
    t0 = time.perf_counter()
    with TrajectoryWriter(str(out / "trajectory.txt")) as w:
        for _ in tqdm.tqdm(range(total) if total else iter(int, 1), desc="frames", unit="f"):
            ok, frame = cap.read()
            if not ok or (a.max_frames and n >= a.max_frames):
                break
            if (frame.shape[1], frame.shape[0]) != IMAGE_SIZE:
                frame = cv2.resize(frame, IMAGE_SIZE)
            frame, cam, marks, det = tracker.process_frame(frame)
            det_frames += int(det.shape != (0,))
            w.write(cap.get(cv2.CAP_PROP_POS_MSEC), cam)
            n += 1
            if viewer3d:
                viewer3d.view(cam, marks, det)
            if viewer2d and viewer2d.view(frame, cam, marks, det):
                break
    tracker.save_map(str(out / "map.txt"))
    for v in (viewer3d, viewer2d):
        if v:
            v.close()
    cap.release()
    for name in ("output_2d.mp4", "output_3d.mp4"):
        p = ROOT / "outputs" / "images" / name
        if p.exists() and ((name == "output_2d.mp4" and a.save_2d) or (name == "output_3d.mp4" and a.save_3d)):
            p.replace(out / name)
    dt_s = time.perf_counter() - t0
    print(f"완료: {n}프레임 (마커 검출 {det_frames}) · {n / dt_s:.1f} fps · 랜드마크 {sorted(tracker.landmarks)} → {out}/")


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    sys.dont_write_bytecode = True
    p = argparse.ArgumentParser(description="[demo] ArUco SLAM on a video")
    p.add_argument("--video", required=True)
    p.add_argument("--filter", default="ekf", help="ekf (수정·검증됨) | ekf_rotations | factorgraph")
    p.add_argument("--out")
    p.add_argument("--no-2d", dest="show_2d", action="store_false")
    p.add_argument("--3d", dest="show_3d", action="store_true")
    p.add_argument("--save-2d", action="store_true")
    p.add_argument("--save-3d", action="store_true")
    p.add_argument("--max-frames", type=int, default=0)
    main(p.parse_args())
