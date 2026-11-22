#!/usr/bin/env python3
"""[twin] ArUco 칼만 필터 잡음 시험 — "localization 이 잘 잡혔다" 고 보고 그 궤적(참값)에 무작위 잡음을 섞은 뒤
칼만(dashboard/loc_kf.py, live_loc 과 같은 코드)을 씌워 잡음만 있을 때(필터 전)와 필터 뒤의 오차를 비교한다.

  python3 analysis/loc_kf/noise_test.py                                  (기본: 세션 20261122_002434 build_map 궤적, 10회)
  python3 analysis/loc_kf/noise_test.py --truth ~/demo/sessions/<세션>/map/trajectory.csv --seeds 20
  python3 analysis/loc_kf/noise_test.py --r-from results/loc_kf/loc_<시각>.csv   (실제 측정 공분산 R 순서를 그대로 씀)
  옵션  --sigma-xy 0.05 --sigma-th-deg 2  (R 을 직접 정할 때)  --outlier-rate 0.02 --outlier-m 0.5  --drop-rate 0.05
        --r-scale 1.0  (칼만에 알려 주는 R = 실제 잡음 R × 이 값: 1 이 아니면 'R 이 틀린 경우' 시험)

참값: build_map 번들 조정 궤적 (trajectory.csv: t_s, x_m, y_m, theta_deg) → 15 Hz 로 보간 (live_loc 실시간과 같은 간격)
잡음: 프레임마다 N(0, R) (x, y, θ 상관 포함) + 가끔 큰 튐(이상치, 무작위 방향) + 가끔 측정 없음(마커 안 보임)
출력 (--out, 기본 version__1/results/loc_kf/noise_test_<시각>/):
  seed<k>.csv     live_loc 기록과 같은 열 + x_true, y_true, th_true, outlier  → plot_loc_kf.py 로 그대로 그림
  seed0.png       첫 시행 그림
  summary.json    시행별·평균: 위치 RMSE (필터 전/후), 줄어든 %, 방향 RMSE, NIS·NEES, 이상치 잡은 비율
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
V1 = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(V1, "dashboard"))
sys.path.insert(0, HERE)
from loc_kf import LocKF, CSV_COLS, csv_row, wrap, fmt  # noqa: E402
import plot_loc_kf as plk  # noqa: E402

DEFAULT_TRUTH = os.path.expanduser("~/demo/sessions/20261122_002434/map/trajectory.csv")


def load_truth(path, hz):
    g = np.genfromtxt(path, delimiter=",", names=True)
    t = g["t_s"] - g["t_s"][0]
    th = np.unwrap(np.radians(g["theta_deg"]))
    tt = np.arange(0, t[-1], 1.0 / hz)
    return tt, np.c_[np.interp(tt, t, g["x_m"]), np.interp(tt, t, g["y_m"]), wrap(np.interp(tt, t, th))]


def r_sequence(args, n):
    """프레임마다 실제 잡음 공분산 R (3×3) 과 보이는 마커 수."""
    if args.r_from:
        d = np.genfromtxt(args.r_from, delimiter=",", names=True)
        d = d[d["meas"] == 1]
        R = np.zeros((len(d), 3, 3))
        R[:, 0, 0], R[:, 0, 1], R[:, 1, 1] = d["Rxx"], d["Rxy"], d["Ryy"]
        R[:, 2, 2], R[:, 0, 2], R[:, 1, 2] = d["Rtt"], d["Rxt"], d["Ryt"]
        R[:, 1, 0], R[:, 2, 0], R[:, 2, 1] = R[:, 0, 1], R[:, 0, 2], R[:, 1, 2]
        idx = np.arange(n) % len(d)
        return R[idx], d["n_markers"][idx].astype(int)
    R = np.diag([args.sigma_xy ** 2, args.sigma_xy ** 2, np.radians(args.sigma_th_deg) ** 2])
    return np.repeat(R[None], n, 0), np.full(n, 2)


def run_once(tt, X, R, nmk, args, seed, path):
    rng = np.random.default_rng(seed)
    kf = LocKF()
    rows, out_flags, outl = [], [], []
    n = len(tt)
    drop = rng.random(n) < args.drop_rate
    is_out = (rng.random(n) < args.outlier_rate) & ~drop
    drop[0] = is_out[0] = False                                   # 첫 측정은 정상 (필터 시작점)
    for i in range(n):
        z = None
        if not drop[i]:
            e = rng.multivariate_normal(np.zeros(3), R[i])
            if is_out[i]:
                a = rng.uniform(0, 2 * np.pi)
                e[:2] += args.outlier_m * np.array([np.cos(a), np.sin(a)])
                e[2] += np.radians(rng.uniform(-20, 20))
            z = X[i] + e
            z[2] = wrap(z[2])
        Rk = R[i] * args.r_scale if z is not None else None
        o = kf.step(tt[i], z, Rk)
        rows.append(csv_row(0, tt[i], z, int(nmk[i]) if z is not None else 0, 1.0 if z is not None else np.nan, Rk, o)
                    + [*X[i], int(is_out[i])])
        out_flags.append(o["flag"] if o else -2)
        outl.append(is_out[i])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLS + ["x_true", "y_true", "th_true", "outlier"])
        for r in rows:
            w.writerow([fmt(v) for v in r])
    d = plk.load(path)
    s = plk.summary(d)
    fl, ol = np.array(out_flags), np.array(outl)
    s["outliers_injected"] = int(ol.sum())
    s["outliers_rejected"] = int((ol & (fl == 0)).sum())
    s["good_rejected"] = int((~ol & (fl == 0)).sum())
    # 이상치 빼고 (정상 잡음만) 비교
    m = (d["meas"] == 1) & (d["outlier"] == 0)
    for nm in ("raw", "kf"):
        e = np.hypot(d[f"x_{nm}"] - d["x_true"], d[f"y_{nm}"] - d["y_true"])[m]
        s[f"rmse_pos_{'meas' if nm == 'raw' else 'kf'}_inliers_m"] = float(np.sqrt(np.mean(e ** 2)))
    return d, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--truth", default=DEFAULT_TRUTH, help="참값 궤적 (build_map trajectory.csv)")
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--r-from", help="실제 live_loc 기록 csv: 그 측정 공분산 R 순서를 잡음으로")
    ap.add_argument("--sigma-xy", type=float, default=0.05)
    ap.add_argument("--sigma-th-deg", type=float, default=2.0)
    ap.add_argument("--outlier-rate", type=float, default=0.02)
    ap.add_argument("--outlier-m", type=float, default=0.5)
    ap.add_argument("--drop-rate", type=float, default=0.05)
    ap.add_argument("--r-scale", type=float, default=1.0)
    ap.add_argument("--out")
    a = ap.parse_args()
    out = a.out or os.path.join(V1, "results", "loc_kf", "noise_test_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    tt, X = load_truth(os.path.expanduser(a.truth), a.hz)
    R, nmk = r_sequence(a, len(tt))
    per = []
    for k in range(a.seeds):
        d, s = run_once(tt, X, R, nmk, a, k, os.path.join(out, f"seed{k}.csv"))
        per.append(s)
        if k == 0:
            plk.plot(d, os.path.join(out, "seed0.png"), 0.5, f"noise test seed 0 ({os.path.basename(a.truth)})")
        print(f"seed {k}: pos RMSE meas {s['rmse_pos_meas_m'] * 100:.1f} cm -> KF {s['rmse_pos_kf_m'] * 100:.1f} cm "
              f"({s['pos_rmse_reduction_pct']:.0f}% less) · inliers {s['rmse_pos_meas_inliers_m'] * 100:.1f} -> "
              f"{s['rmse_pos_kf_inliers_m'] * 100:.1f} cm · θ {s['rmse_theta_meas_deg']:.2f} -> {s['rmse_theta_kf_deg']:.2f}° · "
              f"outliers {s['outliers_rejected']}/{s['outliers_injected']} rejected (good rejected {s['good_rejected']}) · "
              f"NIS {s.get('nis_mean', float('nan')):.2f} · 95% ellipse coverage {s['kf_p95_ellipse_coverage']:.2f}")
    keys = [k for k, v in per[0].items() if isinstance(v, float)]
    mean = {k: float(np.mean([p[k] for p in per])) for k in keys}
    json.dump({"args": vars(a), "truth_samples": len(tt), "mean": mean, "per_seed": per},
              open(os.path.join(out, "summary.json"), "w"), indent=1)
    print("\n평균 (%d회)" % a.seeds)
    for k in ("rmse_pos_meas_m", "rmse_pos_kf_m", "pos_rmse_reduction_pct", "median_pos_meas_m", "median_pos_kf_m", "rmse_pos_meas_inliers_m", "rmse_pos_kf_inliers_m",
              "rmse_theta_meas_deg", "rmse_theta_kf_deg", "nis_mean", "nees_pos_mean", "kf_p95_ellipse_coverage"):
        print(f"  {k:28s} {mean[k]:.4g}")
    print("결과 폴더:", out)


if __name__ == "__main__":
    main()
