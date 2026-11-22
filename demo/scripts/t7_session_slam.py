"""T7. 주행 세션 폴더 → SLAM 궤적을 drive_log 와 같은 시간축(t_s)으로.

  python3 -B t7_session_slam.py <session_dir> [--fixed] [--landmarks track_landmarks.csv]

출력: demo/out/session_<이름>/
  trajectory.txt              (TUM 비슷: t x y z qw qx qy qz — 원본 writer 순서 그대로)
  trajectory_session.csv      t_s, x, y, z, yaw_deg, pwm, n_markers   (카메라 광학 좌표; --landmarks 면 트랙 좌표)
  summary.json

video_frames.csv 가 비어 있으면(Ctrl+C 로 끝난 세션) mkv 패킷 PTS + session.json 의 first_frame_epoch 로 복구.
"""
import common  # noqa: F401
import argparse
import csv
import json
import os
import subprocess

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from filters.extended_kalman_filter import EKF
from t1_ekf_convention import EKFFixedInit
from t4_slam_headless import kabsch


def frame_times(sess):
    meta = json.load(open(os.path.join(sess, "session.json")))
    rows = list(csv.reader(open(os.path.join(sess, "video_frames.csv"))))[1:]
    if rows:
        return np.array([float(r[1]) for r in rows]), "video_frames.csv"
    v = meta.get("video", {})
    pts = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time",
                          "-of", "csv=p=0", os.path.join(sess, "video.mkv")], capture_output=True, text=True).stdout.split()
    pts = np.array(sorted(float(p) for p in pts if p not in ("", "N/A")))
    return np.round(v["first_frame_epoch"] + pts - meta["t0_epoch"], 3), "recovered(mkv pts + first_frame_epoch)"


def pwm_at(sess, ts):
    ev = [(float(r["t_s"]), int(r["value"][1:])) for r in csv.DictReader(open(os.path.join(sess, "drive_log.csv")))
          if r["kind"] == "tx" and r["value"].startswith("V")]
    if not ev:
        return np.zeros(len(ts), int)
    et = np.array([e[0] for e in ev]); ev_v = np.array([e[1] for e in ev])
    idx = np.searchsorted(et, ts, side="right") - 1
    return np.where(idx >= 0, ev_v[np.clip(idx, 0, None)], 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session"); ap.add_argument("--fixed", action="store_true"); ap.add_argument("--landmarks")
    a = ap.parse_args()
    name = os.path.basename(os.path.normpath(a.session))
    out = os.path.join(common.OUT, "session_" + name); os.makedirs(out, exist_ok=True)
    ts, src = frame_times(a.session)
    tr = (EKFFixedInit if a.fixed else EKF)(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    cap = cv2.VideoCapture(os.path.join(a.session, "video.mkv"))
    W, H = int(cap.get(3)), int(cap.get(4))
    poses, nmk = [], []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        if (W, H) != (1920, 1080):
            fr = cv2.resize(fr, (1920, 1080))   # run_slam.py 와 동일 (캘리브레이션이 1080p)
        _, cam, _, det = tr.process_frame(fr)
        poses.append(cam[:7].copy()); nmk.append(0 if det.shape == (0,) else len(det))
    n = min(len(poses), len(ts))
    P = np.array(poses[:n]); ts = ts[:n]; nmk = np.array(nmk[:n])
    xyz = P[:, :3]; R = Rotation.from_quat(P[:, 3:7], scalar_first=True)
    fwd = R.apply([0, 0, 1])                          # 카메라 광축
    frame = "camera_optical(x右,y下,z前)"
    if a.landmarks:
        gt = {int(r[0]): r[1:] for r in np.loadtxt(a.landmarks, delimiter=",", skiprows=1, ndmin=2)}
        _, mk = tr.get_poses()
        ids = [k for k in sorted(tr.landmarks) if k in gt]
        if len(ids) >= 3:
            Ra, ta = kabsch(np.array([mk[tr.landmarks[k]] for k in ids]), np.array([gt[k] for k in ids]))
            xyz = xyz @ Ra.T + ta; fwd = fwd @ Ra.T; frame = "track(landmarks csv)"
    yaw = np.degrees(np.arctan2(fwd[:, 1], fwd[:, 0])) if a.landmarks else np.degrees(np.arctan2(fwd[:, 0], fwd[:, 2]))
    pwm = pwm_at(a.session, ts)
    with open(os.path.join(out, "trajectory_session.csv"), "w") as f:
        f.write("t_s,x,y,z,yaw_deg,pwm,n_markers\n")
        for i in range(n):
            f.write(f"{ts[i]:.3f},{xyz[i,0]:.4f},{xyz[i,1]:.4f},{xyz[i,2]:.4f},{yaw[i]:.2f},{pwm[i]},{nmk[i]}\n")
    tr.save_map(os.path.join(out, "map.txt"))
    res = {"session": a.session, "frame_times_from": src, "video_size": [W, H], "frames": n,
           "frames_with_marker": int((nmk > 0).sum()), "landmarks": sorted(int(k) for k in tr.landmarks),
           "output_frame": frame, "filter": type(tr).__name__,
           "pwm_nonzero_frames": int((pwm != 0).sum())}
    print(json.dumps(res, ensure_ascii=False, indent=1))
    json.dump(res, open(os.path.join(out, "summary.json"), "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
