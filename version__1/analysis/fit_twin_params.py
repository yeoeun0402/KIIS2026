#!/usr/bin/env python3
"""[twin] 실차 주행 기록으로 시뮬레이션 차량 모델(twin_model_step) 파라미터 맞추기.

  python3 fit_twin_params.py --real ~/demo/sessions/<세션> [--sim ~/version__1/results/<run>]

방법: 실차에 실제로 보낸 명령(drive_log.csv 의 V·L·R·F, 실차 시각)을 그대로 시뮬레이션 모델에 다시 넣고(replay),
      나온 궤적이 실차 localization 궤적(map/trajectory.csv)과 가장 가까워지는 파라미터를 찾음 (scipy least_squares).
      출발 자세 = 실차 궤적 첫 점, 모델은 MATLAB twin_model_step.m 과 같은 식 (+ 출발 지연·조향 중립 오프셋).
맞추는 파라미터
  kv [m/s per PWM]   pwmDead [PWM]   tauV [s]   delay [s] (PWM 바뀐 뒤 바퀴가 반응하기까지)
  steerRate [deg/s]  deltaMax [deg]  delta0 [deg] (조향 중립 오프셋: 키를 안 눌러도 한쪽으로 휘는 정도)
출력: <세션>/twin_fit/  fit.json · fit_replay.png (실차 vs 맞추기 전 모델 vs 맞춘 모델) · replay.csv
      --sim 을 주면 그 결과 폴더에도 fit.json 을 복사
"""
import argparse
import csv
import json
import math
import os
import shutil

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

L_WB = 0.56          # 축간거리 (mount.yaml wheelbase, MATLAB vehicle_params p.L)
PULSE = 0.12         # 조향 펄스 [s] (펌웨어 STEER_PULSE_MS)
DT = 0.01
NAMES = ["kv", "pwmDead", "tauV", "delay", "steerRate_deg", "deltaMax_deg", "delta0_deg"]
BEFORE = dict(kv=0.015, pwmDead=0.0, tauV=0.3, delay=0.0, steerRate_deg=60.0, deltaMax_deg=30.0, delta0_deg=0.0)
LO = [0.001, 0.0, 0.02, 0.0, 5.0, 5.0, -5.0]
HI = [0.05, 45.0, 2.0, 1.5, 300.0, 45.0, 5.0]


def load_commands(sess):
    cmds = []
    for r in csv.DictReader(open(os.path.join(sess, "drive_log.csv"))):
        if r["kind"] != "tx":
            continue
        v = r["value"]
        t = float(r["t_s"])
        if v.startswith("V"):
            cmds.append((t, "V", int(v[1:])))
        elif v in ("L", "R", "F"):
            cmds.append((t, v, 0))
    return cmds


def replay(par, cmds, t_eval, pose0):
    """MATLAB twin_model_step.m 과 같은 모델. par: NAMES 순서. 반환: t_eval 시각의 (x, y, yaw)."""
    kv, dead, tau, delay, srate, dmax, d0 = par
    srate, dmax, d0 = math.radians(srate), math.radians(dmax), math.radians(d0)
    x, y, yaw = pose0
    v = delta = 0.0
    pwm_cmd = 0
    pend = []                     # (적용 시각, pwm): delay 뒤에 바퀴에 반영
    pwm = 0
    sdir, sleft = 0, 0.0
    ci = 0
    out = np.full((len(t_eval), 3), np.nan)
    ei = 0
    t = min(t_eval[0], cmds[0][0] if cmds else t_eval[0])
    t_end = t_eval[-1]
    while ei < len(t_eval) and t_eval[ei] <= t:
        out[ei] = (x, y, yaw); ei += 1
    while t < t_end and ei < len(t_eval):
        while ci < len(cmds) and cmds[ci][0] <= t:
            _, k, val = cmds[ci]; ci += 1
            if k == "V":
                if val != pwm_cmd:
                    pwm_cmd = val
                    pend.append((cmds[ci - 1][0] + delay, val))
            elif k == "L":
                sdir, sleft = 1, PULSE
            elif k == "R":
                sdir, sleft = -1, PULSE
            elif k == "F":
                sdir, sleft = 0, 0.0
        while pend and pend[0][0] <= t:
            pwm = pend.pop(0)[1]
        vref = kv * math.copysign(max(abs(pwm) - dead, 0.0), pwm) if pwm else 0.0
        v = vref + (v - vref) * math.exp(-DT / tau)
        if sleft > 0:
            on = min(DT, sleft)
            delta = min(max(delta + sdir * srate * on, -dmax), dmax)
            sleft -= on
            if sleft <= 0:
                sdir, sleft = 0, 0.0
        yr = v * math.tan(delta + d0) / L_WB
        ym = yaw + 0.5 * yr * DT
        x += v * math.cos(ym) * DT
        y += v * math.sin(ym) * DT
        yaw += yr * DT
        t += DT
        while ei < len(t_eval) and t_eval[ei] <= t + 1e-9:
            out[ei] = (x, y, yaw); ei += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", required=True)
    ap.add_argument("--sim")
    ap.add_argument("--t-max", type=float, default=None, help="이 실차 시각까지만 맞춤 (기본: 궤적 끝)")
    a = ap.parse_args()
    sess = os.path.expanduser(a.real)
    cmds = load_commands(sess)
    tr = list(csv.DictReader(open(os.path.join(sess, "map", "trajectory.csv"))))
    T = np.array([float(r["t_s"]) for r in tr])
    XY = np.array([[float(r["x_m"]), float(r["y_m"])] for r in tr])
    TH = np.radians([float(r["theta_deg"]) for r in tr])
    if a.t_max:
        m = T <= a.t_max; T, XY, TH = T[m], XY[m], TH[m]
    pose0 = (XY[0, 0], XY[0, 1], TH[0])
    moving = np.r_[0, np.hypot(*np.diff(XY, axis=0).T)].cumsum()       # 실차 누적 거리 (가중용)

    def resid(p):
        q = replay(p, cmds, T, pose0)
        d = q[:, :2] - XY
        return np.nan_to_num(d.ravel(), nan=10.0)

    p_before = np.array([BEFORE[n] for n in NAMES])
    best = None
    # 여러 시작점 (속도·조향은 서로 얽혀 있어 국소해가 많음)
    for kv0 in (0.006, 0.01):
        for sr0 in (20.0, 60.0):
            for dm0 in (15.0, 30.0):
                p0 = np.array([kv0, 20.0, 0.3, 0.3, sr0, dm0, 0.0])
                r = least_squares(resid, p0, bounds=(LO, HI), x_scale=[0.005, 10, 0.3, 0.3, 30, 10, 1],
                                  diff_step=1e-3, max_nfev=400)
                if best is None or r.cost < best.cost:
                    best = r
    p_fit = best.x
    q_before = replay(p_before, cmds, T, pose0)
    q_fit = replay(p_fit, cmds, T, pose0)
    e_before = np.hypot(*(q_before[:, :2] - XY).T)
    e_fit = np.hypot(*(q_fit[:, :2] - XY).T)

    out = os.path.join(sess, "twin_fit")
    os.makedirs(out, exist_ok=True)
    fit = {
        "session": sess, "samples": int(len(T)), "t_range_s": [round(float(T[0]), 2), round(float(T[-1]), 2)],
        "real_distance_m": round(float(moving[-1]), 3),
        "params_before": BEFORE,
        "params_fit": {n: round(float(v), 5) for n, v in zip(NAMES, p_fit)},
        "position_error_m": {"before_rms": round(float(np.sqrt(np.mean(e_before ** 2))), 3),
                             "before_end": round(float(e_before[-1]), 3),
                             "fit_rms": round(float(np.sqrt(np.mean(e_fit ** 2))), 3),
                             "fit_end": round(float(e_fit[-1]), 3)},
        "speed_at_pwm40_mps": {"before": round(BEFORE["kv"] * 40, 3),
                               "fit": round(float(p_fit[0] * max(40 - p_fit[1], 0)), 3)},
        "note": "같은 명령(drive_log tx)을 모델에 다시 넣어 실차 궤적과 위치 오차 최소화. 한 번의 주행(약 %.0f m)으로 맞춘 값이라 "
                "다른 속도·조향 범위에서는 다를 수 있음. 바닥·배터리 상태에 따라서도 달라짐." % moving[-1],
    }
    json.dump(fit, open(os.path.join(out, "fit.json"), "w"), ensure_ascii=False, indent=1)
    with open(os.path.join(out, "replay.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_real", "y_real", "x_before", "y_before", "x_fit", "y_fit"])
        for i in range(len(T)):
            w.writerow([f"{T[i]:.3f}"] + [f"{v:.4f}" for v in (*XY[i], *q_before[i, :2], *q_fit[i, :2])])

    fig, axs = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw={"width_ratios": [1.3, 1]})
    ax = axs[0]
    cl = os.path.expanduser("~/demo/track/lane2_centerline.csv")
    if os.path.exists(cl):
        c = np.genfromtxt(cl, delimiter=",", names=True)
        ax.plot(c["x_m"], c["y_m"], "-", color="#e0b000", lw=1.2, label="lane 2 centerline")
    ax.plot(XY[:, 0], XY[:, 1], "-", color="#1f5fd6", lw=2.2, label="real car (localization)")
    ax.plot(q_before[:, 0], q_before[:, 1], ":", color="#d62728", lw=1.6, label="model before fit (same commands)")
    ax.plot(q_fit[:, 0], q_fit[:, 1], "--", color="#2ca02c", lw=2, label="model after fit (same commands)")
    ax.plot(*XY[0], "ko", mfc="#2ca02c", ms=8)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=9)
    lo = np.nanmin(np.r_[XY, q_fit[:, :2]], 0) - 0.5; hi = np.nanmax(np.r_[XY, q_fit[:, :2]], 0) + 0.5
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
    ax.set_xlabel("x_track [m]"); ax.set_ylabel("y_track [m]")
    ax.set_title(f"replay of real commands: position RMS before {fit['position_error_m']['before_rms']} m"
                 f" -> after {fit['position_error_m']['fit_rms']} m")
    axs[1].plot(T, e_before, ":", color="#d62728", label="before fit")
    axs[1].plot(T, e_fit, "--", color="#2ca02c", label="after fit")
    axs[1].set_xlabel("real time [s]"); axs[1].set_ylabel("position error vs real [m]"); axs[1].grid(alpha=0.3); axs[1].legend()
    txt = "\n".join(f"{n}: {BEFORE[n]:g} -> {fit['params_fit'][n]:g}" for n in NAMES)
    axs[1].text(0.02, 0.97, txt, transform=axs[1].transAxes, va="top", family="monospace", fontsize=9,
                bbox=dict(fc="white", ec="0.7"))
    fig.tight_layout(); fig.savefig(os.path.join(out, "fit_replay.png"), dpi=120); plt.close(fig)

    if a.sim:
        shutil.copy(os.path.join(out, "fit.json"), os.path.join(os.path.expanduser(a.sim), "fit.json"))
    print(f"[fit] 결과: {out}/")
    print("  " + " · ".join(f"{n} {BEFORE[n]:g}→{fit['params_fit'][n]:g}" for n in NAMES))
    pe = fit["position_error_m"]
    print(f"  같은 명령 재생 위치 오차 RMS: 맞추기 전 {pe['before_rms']} m → 후 {pe['fit_rms']} m (끝점 {pe['before_end']} → {pe['fit_end']} m)")


if __name__ == "__main__":
    main()
