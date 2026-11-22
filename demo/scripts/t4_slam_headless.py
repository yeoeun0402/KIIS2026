"""T4. 녹화 영상으로 SLAM 을 GUI 없이 돌린다(원본 run_slam.py 와 같은 처리, 출력만 demo/out).

  python3 -B t4_slam_headless.py <video> <tag> [--fixed] [--size 1920x1080] [--landmarks csv]

- run_slam.py 와 똑같이 프레임을 1920x1080 으로 리사이즈 → tracker.process_frame()
- 원본은 outputs/trajectory.txt·map.txt 를 덮어쓰지만, 여기서는 demo/out/<tag>/ 에 쓴다
- 프레임당 처리 시간(검출+EKF)을 재서 실시간 가능 여부 판단
- --landmarks 가 있으면 실측 마커 좌표와 Kabsch 정합(복도 alignment_01 과 같은 방식) → 잔차
"""
import common  # noqa: F401
import argparse
import json
import os
import time

import cv2
import numpy as np

from filters.extended_kalman_filter import EKF
from outputs.trajectory_writer import TrajectoryWriter
from t1_ekf_convention import EKFFixedInit


def kabsch(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cd - R @ cs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video"); ap.add_argument("tag")
    ap.add_argument("--fixed", action="store_true")
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--landmarks")
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args()
    out = os.path.join(common.OUT, a.tag); os.makedirs(out, exist_ok=True)
    W, H = map(int, a.size.split("x"))
    cls = EKFFixedInit if a.fixed else EKF
    tr = cls(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    cap = cv2.VideoCapture(a.video)
    src_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    dt, det_frames, n, cams = [], 0, 0, []
    t_read = []
    with TrajectoryWriter(os.path.join(out, "trajectory.txt")) as w:
        while True:
            r0 = time.perf_counter()
            ok, fr = cap.read()
            if not ok or (a.max_frames and n >= a.max_frames):
                break
            fr = cv2.resize(fr, (W, H))
            t_read.append(time.perf_counter() - r0)
            t0 = time.perf_counter()
            fr, cam, marks, det = tr.process_frame(fr)
            dt.append(time.perf_counter() - t0)
            det_frames += int(det.shape != (0,))
            w.write(cap.get(cv2.CAP_PROP_POS_MSEC), cam)
            cams.append(cam[:3].copy())
            n += 1
    tr.save_map(os.path.join(out, "map.txt"))
    dt = np.array(dt) * 1000
    res = {"video": a.video, "src_size": src_size, "proc_size": [W, H], "filter": cls.__name__,
           "frames": n, "frames_with_marker": det_frames, "landmarks": sorted(int(k) for k in tr.landmarks),
           "ms_per_frame_median": round(float(np.median(dt)), 1), "ms_per_frame_p95": round(float(np.percentile(dt, 95)), 1),
           "decode+resize_ms_median": round(float(np.median(t_read) * 1000), 1),
           "fps_possible(proc only)": round(1000 / float(np.median(dt)), 1)}
    _, mk = tr.get_poses()
    if a.landmarks:
        gt = {int(r[0]): r[1:] for r in np.loadtxt(a.landmarks, delimiter=",", skiprows=1)}
        ids = [k for k in sorted(tr.landmarks) if k in gt]
        src = np.array([mk[tr.landmarks[k]] for k in ids]); dst = np.array([gt[k] for k in ids])
        R, t = kabsch(src, dst)
        resid = np.linalg.norm(src @ R.T + t - dst, axis=1)
        cams = np.array(cams) @ R.T + t
        res["align"] = {"ids": [int(i) for i in ids], "residual_cm": {int(k): round(float(e) * 100, 1) for k, e in zip(ids, resid)},
                        "rms_cm": round(float(np.sqrt((resid ** 2).mean())) * 100, 1), "det": round(float(np.linalg.det(R)), 3),
                        "cam_start_corridor": np.round(cams[0], 3).tolist(), "cam_end_corridor": np.round(cams[-1], 3).tolist(),
                        "path_len_xy_m": round(float(np.linalg.norm(np.diff(cams[:, :2], axis=0), axis=1).sum()), 3),
                        "net_disp_xy_m": round(float(np.linalg.norm(cams[-1, :2] - cams[0, :2])), 3)}
        np.savez(os.path.join(out, "slam_to_landmarks_rigid.npz"), R=R, t=t)
    print(json.dumps(res, ensure_ascii=False, indent=1, default=lambda o: o.item()))
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=lambda o: o.item())


if __name__ == "__main__":
    main()
