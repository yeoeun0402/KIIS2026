#!/usr/bin/env python3
"""[twin] 여러 실차 주행으로 차량 모델 파라미터를 한꺼번에 맞추기 (짧은 구간 예측 오차 기준).

  python3 fit_twin_multi.py --real ~/demo/sessions/<세션1> ~/demo/sessions/<세션2> ... [--window 6]

fit_twin_params.py 와의 차이
  한 주행 전체를 처음부터 끝까지 다시 돌리면 작은 조향 차이가 쌓여 몇 m 씩 벌어지고, 그 벌어짐에 맞춰져
  다른 주행에서는 안 맞는 값이 나옴 (20261121_212500 으로 맞춘 값이 214854 에서 더 나빠짐).
  그래서 window 초마다 모델을 실차 상태(위치·방향·속도·조향각)로 다시 놓고, 그 뒤 window 초 동안의 예측 오차만 씀.
  조향각 시작값 = 실차 궤적의 곡률에서: δ = atan(L κ) − δ0.  속도 = 궤적 속도 (부호는 그때 PWM 부호).
  명령(PWM·조향 펄스)과 지연 큐는 처음부터 이어서 돌리므로 실차와 같음.
출력: ~/version__1/analysis/fit_multi/fit.json · fit_windows.png
"""
import argparse
import csv
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.ndimage import gaussian_filter1d  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from fit_twin_params import L_WB, PULSE, DT, NAMES, LO, HI, load_commands  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fit_multi")


def load_session(sess, extra=3.0):
    cmds = load_commands(sess)
    pv = [(t, v) for t, k, v in cmds if k == "V"]
    t_end = max(t for t, v in pv if v != 0) + extra
    tr = np.genfromtxt(os.path.join(sess, "map", "trajectory.csv"), delimiter=",", names=True)
    m = tr["t_s"] <= t_end
    T = np.arange(tr["t_s"][m][0], tr["t_s"][m][-1], 0.1)
    x = np.interp(T, tr["t_s"], tr["x_m"]); y = np.interp(T, tr["t_s"], tr["y_m"])
    th = np.interp(T, tr["t_s"], np.unwrap(np.radians(tr["theta_deg"])))
    xs, ys = gaussian_filter1d(x, 3), gaussian_filter1d(y, 3)
    spd = np.hypot(np.gradient(xs, 0.1), np.gradient(ys, 0.1))
    w = np.gradient(gaussian_filter1d(th, 3), 0.1)
    kappa = np.where(spd > 0.08, w / np.maximum(spd, 1e-3), 0.0)
    pwm_at = np.array([next((v for tt, v in reversed(pv) if tt <= t), 0) for t in T])
    return dict(name=os.path.basename(sess.rstrip("/")), cmds=cmds, T=T, x=x, y=y, th=th, spd=spd, kappa=kappa, pwm=pwm_at)


def replay_windows(par, S, window):
    kv, dead, tau, delay, srate, dmax, d0 = par
    srate, dmax, d0 = math.radians(srate), math.radians(dmax), math.radians(d0)
    T, cmds = S["T"], S["cmds"]
    starts = set(np.flatnonzero(np.r_[True, np.diff(np.floor((T - T[0]) / window)) > 0]).tolist())
    out = np.full((len(T), 2), np.nan)
    pwm_cmd = pwm = 0; pend = []; sdir, sleft = 0, 0.0; ci = 0
    x = y = yaw = v = delta = 0.0
    t = min(T[0], cmds[0][0])
    i = 0
    while i < len(T):
        while ci < len(cmds) and cmds[ci][0] <= t:
            tc, k, val = cmds[ci]; ci += 1
            if k == "V":
                if val != pwm_cmd:
                    pwm_cmd = val; pend.append((tc + delay, val))
            elif k == "L": sdir, sleft = 1, PULSE
            elif k == "R": sdir, sleft = -1, PULSE
            elif k == "F": sdir, sleft = 0, 0.0
        while pend and pend[0][0] <= t:
            pwm = pend.pop(0)[1]
        if t + 1e-9 >= T[i]:
            if i in starts:                                   # 실차 상태로 다시 놓기
                x, y, yaw = S["x"][i], S["y"][i], S["th"][i]
                sgn = 1.0 if S["pwm"][i] >= 0 else -1.0
                v = sgn * S["spd"][i]
                delta = min(max(math.atan(L_WB * S["kappa"][i] * sgn) - d0, -dmax), dmax)
            out[i] = (x, y); i += 1
            continue
        vref = kv * math.copysign(max(abs(pwm) - dead, 0.0), pwm) if pwm else 0.0
        v = vref + (v - vref) * math.exp(-DT / tau)
        if sleft > 0:
            on = min(DT, sleft)
            delta = min(max(delta + sdir * srate * on, -dmax), dmax)
            sleft -= on
            if sleft <= 0: sdir, sleft = 0, 0.0
        yr = v * math.tan(delta + d0) / L_WB
        ym = yaw + 0.5 * yr * DT
        x += v * math.cos(ym) * DT; y += v * math.sin(ym) * DT; yaw += yr * DT
        t += DT
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", nargs="+", required=True)
    ap.add_argument("--window", type=float, default=6.0)
    ap.add_argument("--before", help="비교할 파라미터 JSON (예: twin_params 지금 값)")
    a = ap.parse_args()
    SS = [load_session(os.path.expanduser(s)) for s in a.real]

    def resid(p):
        r = []
        for S in SS:
            q = replay_windows(p, S, a.window)
            r.append(np.nan_to_num((q - np.c_[S["x"], S["y"]]).ravel(), nan=5.0))
        return np.concatenate(r)

    best = None
    for kv0 in (0.007, 0.011):
        for sr0 in (25.0, 50.0):
            for dm0 in (20.0, 35.0):
                p0 = np.array([kv0, 10.0, 0.4, 0.3, sr0, dm0, -2.0])
                r = least_squares(resid, p0, bounds=(LO, HI), x_scale=[0.005, 10, 0.3, 0.3, 30, 10, 1],
                                  diff_step=1e-3, max_nfev=300, loss="soft_l1", f_scale=0.3)
                if best is None or r.cost < best.cost:
                    best = r
    p = best.x
    before = np.array([0.0080, 0.0, 0.42, 0.30, 30.5, 20.5, -1.6])     # 지금 twin_params (180841 로 맞춘 값)
    os.makedirs(OUT, exist_ok=True)
    per = {}
    fig, axs = plt.subplots(1, len(SS), figsize=(7 * len(SS), 6))
    axs = np.atleast_1d(axs)
    for ax, S in zip(axs, SS):
        R = np.c_[S["x"], S["y"]]
        qb, qf = replay_windows(before, S, a.window), replay_windows(p, S, a.window)
        eb, ef = np.hypot(*(qb - R).T), np.hypot(*(qf - R).T)
        per[S["name"]] = {"before_rms": round(float(np.sqrt(np.nanmean(eb ** 2))), 3),
                          "fit_rms": round(float(np.sqrt(np.nanmean(ef ** 2))), 3),
                          "before_p95": round(float(np.nanpercentile(eb, 95)), 3), "fit_p95": round(float(np.nanpercentile(ef, 95)), 3)}
        ax.plot(R[:, 0], R[:, 1], "-", color="#1f5fd6", lw=2, label="real (build_map)")
        ax.plot(qb[:, 0], qb[:, 1], ".", color="#999", ms=2, label="model before (per %.0f s window)" % a.window)
        ax.plot(qf[:, 0], qf[:, 1], ".", color="#d62728", ms=2, label="model fit")
        ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(fontsize=8)
        ax.set_title("%s  window RMS %.2f → %.2f m" % (S["name"], per[S["name"]]["before_rms"], per[S["name"]]["fit_rms"]))
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fit_windows.png"), dpi=110)
    res = {"sessions": [S["name"] for S in SS], "window_s": a.window,
           "params_before": dict(zip(NAMES, before.round(5).tolist())), "params_fit": dict(zip(NAMES, [round(float(v), 5) for v in p])),
           "per_session": per, "speed_mps_at_pwm": {str(w): round(float(p[0] * max(w - p[1], 0)), 3) for w in (10, 20, 30, 40, 50, 60)},
           "steer_deg_per_key": round(float(p[4] * PULSE), 2)}
    json.dump(res, open(os.path.join(OUT, "fit.json"), "w"), ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
