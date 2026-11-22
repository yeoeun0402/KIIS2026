"""P2 정확도: ROS 노드 결과(map.txt · trajectory_ros.csv)를 오프라인 결과(demo/out/corridor_fixed)와 비교.

  python3 -B p2_compare.py <ros_out_dir> [--offline ~/demo/out/corridor_fixed] [--landmarks aruco_landmarks.csv]

- 지도: 각자 실측 마커에 Kabsch 정합 → 마커별 잔차(cm) 비교 · 두 지도끼리 정합한 차이
- 궤적: 영상 시각(첫 프레임 기준)으로 짝지어, 두 궤적을 실측 좌표로 옮긴 뒤 위치 차이(cm)·yaw 차이(°)
통과 기준(PLAN_ROS P2): 마커 잔차 차이 ≤ 1 cm
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/aruco_slam_deps"))
from scipy.spatial.transform import Rotation  # noqa: E402


def read_map(p):
    lines = [l.strip() for l in open(p) if l.strip() and not l.startswith("#")]
    out, i = {}, 0
    while i < len(lines):
        k = int(lines[i]); xyz = np.array(lines[i + 1].replace(",", " ").split(), float); out[k] = xyz; i += 3
    return out


def kabsch(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cd - R @ cs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ros")
    ap.add_argument("--offline", default="~/demo/out/corridor_fixed")
    ap.add_argument("--landmarks", default="~/car_calibration/corridor_map/aruco_landmarks.csv")
    a = ap.parse_args()
    ros, off = os.path.expanduser(a.ros), os.path.expanduser(a.offline)
    gt = {int(r[0]): r[1:] for r in np.loadtxt(os.path.expanduser(a.landmarks), delimiter=",", skiprows=1, ndmin=2)}
    res = {}
    maps = {"ros": read_map(os.path.join(ros, "map.txt")), "offline": read_map(os.path.join(off, "map.txt"))}
    align = {}
    for name, m in maps.items():
        ids = sorted(k for k in m if k in gt)
        S, D = np.array([m[k] for k in ids]), np.array([gt[k] for k in ids])
        R, t = kabsch(S, D)
        r = np.linalg.norm(S @ R.T + t - D, axis=1) * 100
        align[name] = (R, t)
        res[name] = {"ids": ids, "residual_cm": dict(zip(ids, np.round(r, 1).tolist())), "rms_cm": round(float(np.sqrt((r ** 2).mean())), 1)}
    ids = sorted(set(maps["ros"]) & set(maps["offline"]))
    R, t = kabsch(np.array([maps["ros"][k] for k in ids]), np.array([maps["offline"][k] for k in ids]))
    dm = np.linalg.norm(np.array([maps["ros"][k] for k in ids]) @ R.T + t - np.array([maps["offline"][k] for k in ids]), axis=1) * 100
    res["map_ros_vs_offline_cm"] = dict(zip(ids, np.round(dm, 2).tolist()))
    res["residual_diff_cm"] = {k: round(res["ros"]["residual_cm"][k] - res["offline"]["residual_cm"][k], 1) for k in res["ros"]["ids"]}

    # 궤적: 실측 좌표로 옮겨 시각으로 짝짓기
    rr = list(csv.DictReader(open(os.path.join(ros, "trajectory_ros.csv"))))
    t_r = np.array([float(x["stamp"]) for x in rr]); t_r -= t_r[0]
    P_r = np.array([[float(x[c]) for c in ("cam_x", "cam_y", "cam_z")] for x in rr])
    Q_r = np.array([[float(x[c]) for c in ("cam_qw", "cam_qx", "cam_qy", "cam_qz")] for x in rr])
    o = np.loadtxt(os.path.join(off, "trajectory.txt"))
    t_o, P_o, Q_o = o[:, 0] - o[0, 0], o[:, 1:4], o[:, 4:8]                 # TUM 순서 t x y z qw qx qy qz (demo 형식)
    Rr, tr_ = align["ros"]; Ro, to_ = align["offline"]
    Wr, Wo = P_r @ Rr.T + tr_, P_o @ Ro.T + to_
    j = np.clip(np.searchsorted(t_o, t_r), 0, len(t_o) - 1)
    d = np.linalg.norm(Wr - Wo[j], axis=1) * 100

    def heading(Q, R):
        f = Rotation.from_quat(Q, scalar_first=True).apply([0, 0, 1]) @ R.T
        return np.degrees(np.arctan2(f[:, 1], f[:, 0]))
    dy = (heading(Q_r, Rr) - heading(Q_o[j], Ro) + 180) % 360 - 180
    res["trajectory"] = {"pairs": int(len(d)), "pos_diff_cm_median": round(float(np.median(d)), 2),
                         "pos_diff_cm_p95": round(float(np.percentile(d, 95)), 2), "pos_diff_cm_max": round(float(d.max()), 2),
                         "yaw_diff_deg_median": round(float(np.median(np.abs(dy))), 2), "yaw_diff_deg_max": round(float(np.abs(dy).max()), 2)}
    s = json.load(open(os.path.join(ros, "summary.json")))
    res["ros_summary"] = {k: s.get(k) for k in ("frames_received", "frames_processed", "frames_dropped", "proc_ms_median",
                                                 "latency_ms_median", "latency_ms_p95")}
    worst = max(abs(v) for v in res["residual_diff_cm"].values())
    res["pass_residual_within_1cm"] = worst <= 1.0
    json.dump(res, open(os.path.join(ros, "compare_offline.json"), "w"), ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
