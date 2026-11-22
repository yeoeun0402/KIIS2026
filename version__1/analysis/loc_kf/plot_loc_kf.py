#!/usr/bin/env python3
"""[twin] ArUco 칼만 필터 기록 그림 — 필터 넣기 전(원 측정) · 후(칼만) 비교.

  python3 analysis/loc_kf/plot_loc_kf.py <loc_*.csv> [--out 그림.png] [--every 0.5]

입력: live_loc.py 기록 (version__1/results/loc_kf/loc_<시각>.csv) 또는 noise_test.py 결과 csv (x_true 열이 있으면 참값과의 오차도 그림)
출력: 같은 이름 .png (4칸) + _summary.json
  ① 트랙 위 경로: 원 측정(주황) · 칼만(하늘색) · 95% 타원 (R 주황, P 하늘색, every 초마다) · 버린 측정(빨강)
  ② x, y, θ 시간 그래프 (원 측정 점 · 칼만 선)
  ③ 위치 불확실성 σ_pos = sqrt(xx+yy): 측정 R vs 칼만 P, 보이는 마커 수
  ④ NIS (측정이 칼만 예측에서 얼마나 벗어났나): 평균 3 근처면 R·과정잡음이 맞음, 16.27 넘으면 버림
"""
import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Ellipse  # noqa: E402

CL = os.path.expanduser("~/demo/track/lane2_centerline.csv")
K95 = np.sqrt(5.991)          # 2차원 95%
C_MEAS, C_KF, C_TRUE, C_REJ = "#ff8c00", "#00a5d6", "#222222", "#e00000"


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    return d


def ell(ax, x, y, a, b, c, color, lw=0.8, alpha=1.0):
    if not (np.isfinite(a) and np.isfinite(c)) or a <= 0 or c <= 0:
        return
    w, v = np.linalg.eigh(np.array([[a, b], [b, c]]))
    w = np.maximum(w, 0)
    ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
    ax.add_patch(Ellipse((x, y), 2 * K95 * np.sqrt(w[1]), 2 * K95 * np.sqrt(w[0]), angle=ang,
                         fill=False, ec=color, lw=lw, alpha=alpha))


def wrapd(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def summary(d):
    m = d["meas"] == 1
    used = d["kf_flag"] == 1
    s = {"rows": int(len(d)), "measurements": int(m.sum()), "kf_used": int(used.sum()),
         "kf_rejected": int((d["kf_flag"] == 0).sum()), "kf_predict_only": int((d["kf_flag"] == -1).sum()),
         "kf_restarts": int((d["kf_flag"] == 2).sum())}
    nis = d["nis"][used & np.isfinite(d["nis"])]
    if len(nis):
        s["nis_mean"] = float(nis.mean())
        s["nis_frac_above_7.81"] = float((nis > 7.81).mean())      # χ²₃ 95%: 맞으면 약 5%
    s["sigma_pos_meas_median_m"] = float(np.nanmedian(np.sqrt(d["Rxx"] + d["Ryy"])[m]))
    s["sigma_pos_kf_median_m"] = float(np.nanmedian(np.sqrt(d["Pxx"] + d["Pyy"])))
    # 흔들림: 연속 측정 위치의 2차 차분 (참값 없이도 볼 수 있는 잡음 크기)
    for nm, x, y in (("meas", d["x_raw"][m], d["y_raw"][m]), ("kf", d["x_kf"][m], d["y_kf"][m])):
        if len(x) > 3:
            s[f"jitter_{nm}_m"] = float(np.sqrt(np.mean(np.diff(x, 2) ** 2 + np.diff(y, 2) ** 2)))
    if "x_true" in d.dtype.names:
        for nm, x, y, th in (("meas", d["x_raw"], d["y_raw"], d["th_raw"]), ("kf", d["x_kf"], d["y_kf"], d["th_kf"])):
            e = np.hypot(x - d["x_true"], y - d["y_true"])[m]
            et = np.degrees(wrapd(th - d["th_true"]))[m]
            s[f"rmse_pos_{nm}_m"] = float(np.sqrt(np.nanmean(e ** 2)))
            s[f"median_pos_{nm}_m"] = float(np.nanmedian(e))
            s[f"p95_pos_{nm}_m"] = float(np.nanpercentile(e, 95))
            s[f"max_pos_{nm}_m"] = float(np.nanmax(e))
            s[f"rmse_theta_{nm}_deg"] = float(np.sqrt(np.nanmean(et ** 2)))
        s["pos_rmse_reduction_pct"] = 100 * (1 - s["rmse_pos_kf_m"] / s["rmse_pos_meas_m"])
        # NEES (칼만이 말하는 불확실성 P 가 실제 오차와 맞는지): 위치 2차원 평균 2 가 이상적
        ex, ey = d["x_kf"] - d["x_true"], d["y_kf"] - d["y_true"]
        det = d["Pxx"] * d["Pyy"] - d["Pxy"] ** 2
        nees = (d["Pyy"] * ex ** 2 - 2 * d["Pxy"] * ex * ey + d["Pxx"] * ey ** 2) / det
        s["nees_pos_mean"] = float(np.nanmean(nees[m]))
        s["kf_p95_ellipse_coverage"] = float(np.nanmean(nees[m] <= 5.991))   # 95% 타원 안에 참값이 든 비율 (≈0.95 이상적)
    return s


def plot(d, out, every=0.5, title=""):
    t = d["epoch"] - d["epoch"][0]
    m = d["meas"] == 1
    rej = d["kf_flag"] == 0
    has_true = "x_true" in d.dtype.names
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.35, 1])
    ax = fig.add_subplot(gs[:, 0])
    try:
        cl = np.loadtxt(CL, delimiter=",", skiprows=1)
        ax.plot(cl[:, 1], cl[:, 2], "--", color="#bbbbbb", lw=1, label="lane 2 centre")
    except OSError:
        pass
    if has_true:
        ax.plot(d["x_true"], d["y_true"], "-", color=C_TRUE, lw=1.0, label="truth")
    ax.plot(d["x_raw"][m], d["y_raw"][m], ".-", color=C_MEAS, ms=2, lw=0.5, alpha=0.8, label="measurement (no KF)")
    ax.plot(d["x_kf"], d["y_kf"], "-", color=C_KF, lw=1.6, label="Kalman")
    ax.plot(d["x_raw"][rej], d["y_raw"][rej], "x", color=C_REJ, ms=6, mew=1.5, label="rejected (NIS>16.27)")
    last = -1e9
    for i in range(len(d)):
        if t[i] - last < every:
            continue
        last = t[i]
        if m[i]:
            ell(ax, d["x_raw"][i], d["y_raw"][i], d["Rxx"][i], d["Rxy"][i], d["Ryy"][i], C_MEAS, 0.6, 0.6)
        ell(ax, d["x_kf"][i], d["y_kf"][i], d["Pxx"][i], d["Pxy"][i], d["Pyy"][i], C_KF, 0.9)
    ax.set_aspect("equal"); ax.set_xlim(0, 16); ax.set_ylim(0, 11); ax.grid(alpha=0.3)
    ax.set_xlabel("x track [m]"); ax.set_ylabel("y track [m]")
    ax.set_title(f"{title}\n95% ellipses every {every:g} s: orange = measurement R, cyan = Kalman P")
    ax.legend(loc="center", fontsize=8)

    a1 = fig.add_subplot(gs[0, 1])
    for k, nm in ((0, "x"), (1, "y")):
        if has_true:
            a1.plot(t, d[f"{nm}_true"], color=C_TRUE, lw=0.8)
        a1.plot(t[m], d[f"{nm}_raw"][m], ".", color=C_MEAS, ms=2)
        a1.plot(t, d[f"{nm}_kf"], color=C_KF, lw=1.2)
    a1t = a1.twinx()
    a1t.plot(t[m], np.degrees(d["th_raw"][m]), ".", color="#c9a0dc", ms=1.5)
    a1t.plot(t, np.degrees(d["th_kf"]), color="#7b3fa0", lw=0.8)
    a1.set_ylabel("x, y [m]  (dots meas, line KF)"); a1t.set_ylabel("θ [deg] (purple)"); a1.grid(alpha=0.3)

    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    a2.plot(t[m], 100 * np.sqrt(d["Rxx"] + d["Ryy"])[m], ".", color=C_MEAS, ms=2, label="measurement R")
    a2.plot(t, 100 * np.sqrt(d["Pxx"] + d["Pyy"]), color=C_KF, lw=1.2, label="Kalman P")
    if has_true:
        a2.plot(t[m], 100 * np.hypot(d["x_raw"] - d["x_true"], d["y_raw"] - d["y_true"])[m], ",", color="#8a5a00", alpha=0.6, label="|meas − truth|")
        a2.plot(t, 100 * np.hypot(d["x_kf"] - d["x_true"], d["y_kf"] - d["y_true"]), color="#005f7a", lw=0.7, label="|KF − truth|")
    top = 2.5 * np.nanpercentile(100 * np.sqrt(d["Rxx"] + d["Ryy"])[m], 90) if m.any() else 20
    a2.set_ylim(0, top)                                  # 큰 튐(이상치)은 잘려 보임
    a2.set_ylabel("σ_pos / error [cm]"); a2.grid(alpha=0.3); a2.legend(fontsize=7, loc="upper right")
    a2t = a2.twinx(); a2t.plot(t[m], d["n_markers"][m], ".", color="#999999", ms=1.5, alpha=0.5)
    a2t.set_ylim(0, max(6, np.nanmax(d["n_markers"]) + 1)); a2t.set_ylabel("markers seen (grey)", color="#999999")

    a3 = fig.add_subplot(gs[2, 1], sharex=a1)
    used = d["kf_flag"] == 1
    a3.semilogy(t[used], np.maximum(d["nis"][used], 1e-3), ".", color="#444444", ms=2, label="NIS (used)")
    a3.semilogy(t[rej], d["nis"][rej], "x", color=C_REJ, ms=5, label="rejected")
    for v, ls in ((3, "-"), (7.81, "--"), (16.27, ":")):
        a3.axhline(v, color="#2a9d3a", ls=ls, lw=0.8)
    a3.set_ylabel("NIS  (— 3 mean, -- 7.81 95%, ·· 16.27 gate)"); a3.set_xlabel("t [s]"); a3.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out")
    ap.add_argument("--every", type=float, default=0.5, help="타원 간격 [s]")
    a = ap.parse_args()
    d = load(a.csv)
    out = a.out or os.path.splitext(a.csv)[0] + ".png"
    s = summary(d)
    plot(d, out, a.every, os.path.basename(a.csv))
    json.dump(s, open(os.path.splitext(out)[0] + "_summary.json", "w"), indent=1)
    for k, v in s.items():
        print(f"{k:32s} {v:.4g}" if isinstance(v, float) else f"{k:32s} {v}")
    print("그림:", out)


if __name__ == "__main__":
    sys.exit(main())
