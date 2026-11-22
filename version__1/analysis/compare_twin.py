#!/usr/bin/env python3
"""[twin] Unity 시뮬레이션 궤적 vs 실차 localization 궤적 → 횡방향 오차 분석.

  python3 compare_twin.py --real ~/demo/sessions/<세션>        시뮬 결과는 summary.json 의 car_session 으로 자동으로 찾음
  python3 compare_twin.py --sim  ~/version__1/results/<run>     실차 세션은 summary.json 의 car_session
  python3 compare_twin.py --sim <run> --real <세션>             둘 다 직접 지정

입력
  시뮬 (MATLAB twin_drive.m):  timeseries.csv (트랙 좌표 x_track, y_track, theta_track · epoch_s) · summary.json · unity_top.png/.json
  실차 (drive.sh --twin):      map/trajectory.csv (build_map.py: 트랙 좌표 base_link x, y, θ · 세션 시각 t_s)
                               session.json (t0_epoch) · drive_log.csv (보낸 PWM)
  기준선: 도면 2차선 중심선 (~/demo/track/lane2_centerline.csv, MATLAB/twin 에도 같은 파일)

좌표: 모두 트랙 좌표 (원점 매트 왼쪽 아래, x 오른쪽 16 m, y 위 11 m), 기준점 = 뒷축 중앙 바닥 투영
횡방향 오차 세 가지 (모두 왼쪽 +, 단위 m)
  e_lat_sim, e_lat_real   각각 기준선(2차선 중심선)에서 옆으로 벗어난 거리
  d_lat_s     같은 진행 거리 s 에서  e_lat_real(s) - e_lat_sim(s)     ← 주 지표 (두 차가 같은 곳을 지날 때 옆으로 얼마나 다른가)
  d_lat_t     같은 시각 t 에서 실차 위치를 시뮬 차의 진행 방향 기준으로 본 옆 거리 (속도 차이가 있으면 앞뒤 차이 d_lon_t 가 커짐)

출력: <시뮬 결과 폴더>/compare/
  compare_trajectories.png   트랙 위 두 궤적 + 기준선
  compare_lateral.png        s 기준 횡오차 (시뮬·실차·차이) + 시각 기준 옆/앞뒤 차이
  compare_on_unity.png       Unity Top 이미지(unity_top.png) 위에 실차 궤적(파랑)을 겹친 그림
  compare.csv                실차 샘플마다 비교 숫자
  report.json                RMS·최대·평균 + 속도 보정 추정값(est_kv)
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                                   # ~/version__1
CENTERLINE = [os.path.expanduser("~/demo/track/lane2_centerline.csv"),
              os.path.join(ROOT, "MATLAB", "twin", "lane2_centerline.csv")]
MAT = (16.0, 11.0)


# ── 입력 ─────────────────────────────────────────────────────────
def load_csv(path):
    with open(path) as f:
        r = csv.DictReader(f)
        rows = list(r)
    return {k: np.array([float(x[k]) if x[k] not in ("", "NaN", "nan") else np.nan for x in rows]) for k in r.fieldnames}


def find_sim_for_real(real):
    real = os.path.realpath(real)
    best = None
    for sj in sorted(glob.glob(os.path.join(ROOT, "results", "*_twin", "summary.json"))):
        s = json.load(open(sj))
        if s.get("car_session") and os.path.realpath(s["car_session"]) == real:
            best = os.path.dirname(sj)
    return best


def load_centerline():
    for p in CENTERLINE:
        if os.path.exists(p):
            c = load_csv(p)
            return np.c_[c["x_m"], c["y_m"]], p
    sys.exit("기준선 파일 없음: " + " / ".join(CENTERLINE))


# ── 기준선에 투영 (닫힌 폴리라인) ──────────────────────────────────
class Centerline:
    def __init__(self, xy):
        self.p = xy
        self.q = np.roll(xy, -1, axis=0)                       # 다음 점 (닫힘)
        seg = self.q - self.p
        self.len = np.hypot(seg[:, 0], seg[:, 1])
        self.u = seg / self.len[:, None]
        self.s0 = np.r_[0, np.cumsum(self.len)[:-1]]
        self.L = self.len.sum()

    def project(self, pts):
        """각 점 → (s, e_lat 왼쪽 +)"""
        d = pts[:, None, :] - self.p[None, :, :]               # N x M x 2
        a = np.clip((d * self.u[None]).sum(-1), 0, self.len[None])
        foot = self.p[None] + a[..., None] * self.u[None]
        dist2 = ((pts[:, None, :] - foot) ** 2).sum(-1)
        k = dist2.argmin(1)
        i = np.arange(len(pts))
        s = self.s0[k] + a[i, k]
        r = pts - foot[i, k]
        e = self.u[k, 0] * r[:, 1] - self.u[k, 1] * r[:, 0]   # 진행 방향 기준 왼쪽 +
        return s, e


def unwrap_s(s, L):
    """한 바퀴를 넘어가도 계속 늘어나는 진행 거리로"""
    out = s.copy()
    for i in range(1, len(s)):
        ds = s[i] - s[i - 1]
        if ds < -L / 2:
            out[i:] += L
        elif ds > L / 2:
            out[i:] -= L
    return out


def stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return {"n": 0}
    return {"n": int(len(x)), "rms": round(float(np.sqrt(np.mean(x ** 2))), 4), "mean": round(float(np.mean(x)), 4),
            "max_abs": round(float(np.max(np.abs(x))), 4), "p95_abs": round(float(np.percentile(np.abs(x), 95)), 4)}


def pwm_series(sess):
    p = os.path.join(sess, "drive_log.csv")
    t, v = [], []
    if os.path.exists(p):
        for r in csv.DictReader(open(p)):
            if r["kind"] == "tx" and r["value"].startswith("V"):
                t.append(float(r["t_s"])); v.append(int(r["value"][1:]))
    return np.array(t), np.array(v)


# ── 메인 ─────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", help="twin_drive.m 결과 폴더 (~/version__1/results/<run>)")
    ap.add_argument("--real", help="실차 세션 폴더 (~/demo/sessions/<세션>)")
    ap.add_argument("--real-traj", help="실차 궤적 CSV 직접 지정 (기본 <세션>/map/trajectory.csv)")
    a = ap.parse_args()

    sim = a.sim and os.path.expanduser(a.sim)
    real = a.real and os.path.expanduser(a.real)
    if not sim and real:
        sim = find_sim_for_real(real)
        if not sim:
            sys.exit(f"[compare] 이 실차 세션과 연결된 시뮬 결과가 없음 ({ROOT}/results/*_twin/summary.json 의 car_session). --sim 으로 지정")
    if not sim:
        sys.exit("--sim 또는 --real 이 필요함")
    summ = json.load(open(os.path.join(sim, "summary.json")))
    if not real:
        real = summ.get("car_session") or ""
        if not real:
            sys.exit("[compare] summary.json 에 실차 세션이 없음 (실차 없이 돌린 시뮬). --real 로 지정")
    traj_p = a.real_traj or os.path.join(real, "map", "trajectory.csv")
    if not os.path.exists(traj_p):
        sys.exit(f"[compare] 실차 궤적 없음: {traj_p}\n  먼저: python3 -B ~/demo/track/tools/build_map.py --session {real} --out {real}/map")

    S = load_csv(os.path.join(sim, "timeseries.csv"))
    R = load_csv(traj_p)
    # 주행이 끝난 뒤(마지막으로 PWM 이 0 이 아니었던 시각 + 5 s)의 실차 궤적은 뺀다:
    # 주행 뒤 차를 손으로 들어 출발선에 옮기면 그것도 궤적에 남아 거리·오차가 크게 부풀려짐 (20261121_212500: 21.8 m 중 12 m)
    tp_all, pv_all = pwm_series(real)
    moving = tp_all[pv_all != 0] if len(tp_all) else np.array([])
    t_drive_end = float(moving.max()) + 5.0 if len(moving) else np.inf
    keep = R["t_s"] <= t_drive_end
    if (~keep).any():
        print(f"[compare] 주행 끝(PWM 마지막 0 아님 + 5 s = {t_drive_end:.1f} s) 뒤 실차 궤적 {int((~keep).sum())}개 제외")
        R = {k: v[keep] for k, v in R.items()}
    sess_meta = json.load(open(os.path.join(real, "session.json"))) if os.path.exists(os.path.join(real, "session.json")) else {}
    t0_real = float(sess_meta.get("t0_epoch") or summ.get("car_t0_epoch") or np.nan)
    cl_xy, cl_path = load_centerline()
    C = Centerline(cl_xy)

    sim_xy = np.c_[S["x_track"], S["y_track"]]
    sim_th = S["theta_track_rad"]
    sim_ep = S["epoch_s"]
    real_xy = np.c_[R["x_m"], R["y_m"]]
    real_th = np.radians(R["theta_deg"])
    real_ep = t0_real + R["t_s"]

    # 1) 각자 기준선 횡오차
    s_sim, e_sim = C.project(sim_xy)
    s_real, e_real = C.project(real_xy)
    s_sim_u, s_real_u = unwrap_s(s_sim, C.L), unwrap_s(s_real, C.L)
    s_sim_u -= s_sim_u[0]; s_real_u -= s_real_u[0]               # 출발점 기준 진행 거리 (둘 다 같은 출발 자세)
    s_sim_u += s_sim[0]; s_real_u += s_real[0]

    # 2) 같은 s 에서 비교 (시뮬 e_lat 를 s 로 보간; s 가 늘어나는 구간만)
    order = np.argsort(s_sim_u, kind="stable")
    ss, es = s_sim_u[order], e_sim[order]
    keep = np.r_[True, np.diff(ss) > 1e-4]
    ss, es = ss[keep], es[keep]
    in_range = (s_real_u >= ss[0]) & (s_real_u <= ss[-1])
    e_sim_at_s = np.where(in_range, np.interp(s_real_u, ss, es), np.nan)
    d_lat_s = e_real - e_sim_at_s

    # 3) 같은 시각에서 비교 (시뮬 차 진행 방향 기준 옆/앞뒤)
    ok_t = np.isfinite(real_ep) & (real_ep >= sim_ep[0]) & (real_ep <= sim_ep[-1])
    sx = np.interp(real_ep, sim_ep, sim_xy[:, 0]); sy = np.interp(real_ep, sim_ep, sim_xy[:, 1])
    sth = np.interp(real_ep, sim_ep, np.unwrap(sim_th))
    dx, dy = real_xy[:, 0] - sx, real_xy[:, 1] - sy
    d_lon_t = np.where(ok_t, np.cos(sth) * dx + np.sin(sth) * dy, np.nan)
    d_lat_t = np.where(ok_t, -np.sin(sth) * dx + np.cos(sth) * dy, np.nan)
    d_head_t = np.where(ok_t, np.degrees(np.angle(np.exp(1j * (real_th - sth)))), np.nan)

    # 4) 속도 보정 추정: 실차 속도 vs 보낸 PWM (|PWM| > 0, 속도가 거의 일정한 구간)
    est = {}
    tp, vp = pwm_series(real)
    if len(R["t_s"]) > 5 and len(tp):
        tt = R["t_s"]
        v_real = np.r_[np.nan, np.hypot(np.diff(real_xy[:, 0]), np.diff(real_xy[:, 1])) / np.maximum(np.diff(tt), 1e-3)]
        idx = np.searchsorted(tp, tt, side="right") - 1
        pwm_at = np.where(idx >= 0, vp[np.clip(idx, 0, None)], 0)
        m = (np.abs(pwm_at) > 0) & np.isfinite(v_real)
        if m.sum() > 10:
            est["est_kv_mps_per_pwm"] = round(float(np.median(v_real[m] / np.abs(pwm_at[m]))), 5)
            est["samples"] = int(m.sum())
            est["note"] = "실차 속도 / |PWM| 중앙값 — twin_params.m 의 kv 를 이 값으로 바꾸면 시뮬 속도가 실차에 가까워짐 (pwmDead·지연 무시한 대략값)"
    est["sim_kv_used"] = summ.get("params", {}).get("kv")

    out = os.path.join(sim, "compare")
    os.makedirs(out, exist_ok=True)

    # ── 숫자 ──
    with open(os.path.join(out, "compare.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_real_s", "epoch_s", "x_real", "y_real", "theta_real_deg", "s_real_m", "e_lat_real_m",
                    "e_lat_sim_at_s_m", "d_lat_s_m", "x_sim_at_t", "y_sim_at_t", "d_lat_t_m", "d_lon_t_m", "d_head_t_deg"])
        for i in range(len(real_xy)):
            w.writerow([f"{R['t_s'][i]:.3f}", f"{real_ep[i]:.3f}", f"{real_xy[i, 0]:.4f}", f"{real_xy[i, 1]:.4f}",
                        f"{R['theta_deg'][i]:.2f}", f"{s_real_u[i]:.4f}", f"{e_real[i]:.4f}", f"{e_sim_at_s[i]:.4f}",
                        f"{d_lat_s[i]:.4f}", f"{sx[i]:.4f}" if ok_t[i] else "", f"{sy[i]:.4f}" if ok_t[i] else "",
                        f"{d_lat_t[i]:.4f}", f"{d_lon_t[i]:.4f}", f"{d_head_t[i]:.2f}"])
    dist = lambda xy: float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))  # noqa: E731
    rep = {
        "sim_run": sim, "real_session": real, "real_trajectory": traj_p, "centerline": cl_path,
        "time_sync": "epoch (같은 PC 벽시계): 시뮬 epoch_s vs 실차 session.json t0_epoch + t_s",
        "duration_s": {"sim": round(float(S["t_s"][-1]), 2), "real": round(float(R["t_s"][-1] - R["t_s"][0]), 2)},
        "distance_m": {"sim": round(dist(sim_xy), 3), "real": round(dist(real_xy), 3)},
        "end_point_gap_m": round(float(np.hypot(*(real_xy[-1] - sim_xy[-1]))), 3),
        "e_lat_sim_vs_centerline_m": stats(e_sim),
        "e_lat_real_vs_centerline_m": stats(e_real),
        "d_lat_same_s_m (주 지표: 실차 - 시뮬)": stats(d_lat_s),
        "d_lat_same_time_m": stats(d_lat_t),
        "d_lon_same_time_m": stats(d_lon_t),
        "d_heading_same_time_deg": stats(d_head_t),
        "calibration_estimate": est,
        "build_map_report": os.path.join(os.path.dirname(traj_p), "report.json"),
    }
    json.dump(rep, open(os.path.join(out, "report.json"), "w"), ensure_ascii=False, indent=1)

    # ── 그림 1: 트랙 위 궤적 ──
    fig, ax = plt.subplots(figsize=(12, 8.6))
    ax.plot([0, MAT[0], MAT[0], 0, 0], [0, 0, MAT[1], MAT[1], 0], "k-", lw=1, label="mat 16 x 11 m")
    ax.plot(np.r_[cl_xy[:, 0], cl_xy[0, 0]], np.r_[cl_xy[:, 1], cl_xy[0, 1]], "-", color="#e0b000", lw=1.5, label="lane 2 centerline")
    ax.plot(sim_xy[:, 0], sim_xy[:, 1], "-", color="#d62728", lw=2, label="Unity sim")
    ax.plot(real_xy[:, 0], real_xy[:, 1], "-", color="#1f5fd6", lw=2, label="real car (localization)")
    ax.plot(*sim_xy[0], "o", ms=9, mfc="#2ca02c", mec="k", label="start")
    ax.plot(*sim_xy[-1], "s", ms=8, mfc="#d62728", mec="k", label="sim end")
    ax.plot(*real_xy[-1], "s", ms=8, mfc="#1f5fd6", mec="k", label="real end")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlim(-0.3, 16.3); ax.set_ylim(-0.3, 11.3)
    ax.set_xlabel("x_track [m]"); ax.set_ylabel("y_track [m]")
    r = rep["d_lat_same_s_m (주 지표: 실차 - 시뮬)"]
    ax.set_title(f"Twin compare  sim {os.path.basename(sim)} vs real {os.path.basename(real)}\n"
                 f"lateral diff at same s: RMS {r.get('rms', float('nan')):.3f} m, max {r.get('max_abs', float('nan')):.3f} m"
                 f"   distance sim {rep['distance_m']['sim']:.2f} m / real {rep['distance_m']['real']:.2f} m")
    ax.legend(loc="center", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(out, "compare_trajectories.png"), dpi=130); plt.close(fig)

    # ── 그림 2: 횡오차 ──
    fig, axs = plt.subplots(2, 1, figsize=(12, 8))
    axs[0].plot(s_sim_u, e_sim, color="#d62728", label="sim e_lat (vs centerline)")
    axs[0].plot(s_real_u, e_real, color="#1f5fd6", label="real e_lat (vs centerline)")
    axs[0].plot(s_real_u, d_lat_s, color="k", lw=1.2, ls="--", label="real - sim at same s")
    axs[0].axhline(0, color="gray", lw=0.8); axs[0].grid(alpha=0.3)
    axs[0].set_xlabel("progress along lane 2 s [m]"); axs[0].set_ylabel("lateral [m] (left +)"); axs[0].legend()
    tr = R["t_s"] - R["t_s"][0]
    axs[1].plot(tr, d_lat_t, color="k", label="lateral diff at same time (sim heading frame)")
    axs[1].plot(tr, d_lon_t, color="#8c564b", label="longitudinal diff at same time")
    axs[1].axhline(0, color="gray", lw=0.8); axs[1].grid(alpha=0.3)
    axs[1].set_xlabel("real time [s]"); axs[1].set_ylabel("real - sim [m]"); axs[1].legend()
    fig.tight_layout(); fig.savefig(os.path.join(out, "compare_lateral.png"), dpi=130); plt.close(fig)

    # ── 그림 3: Unity Top 이미지 위에 실차 궤적 ──
    up, uj = os.path.join(sim, "unity_top.png"), os.path.join(sim, "unity_top.json")
    if os.path.exists(up) and os.path.exists(uj):
        meta = json.load(open(uj))
        img = plt.imread(up)
        k = float(meta["pixels_per_meter"]); h = img.shape[0]
        if meta.get("orientation") == "track":
            px = lambda xy: (xy[:, 0] * k, h - xy[:, 1] * k)          # noqa: E731
        else:
            px = lambda xy: ((8.0 - xy[:, 0] + 8.0) * k, h - (5.5 - xy[:, 1] + 5.5) * k)  # sim 방향  # noqa: E731
        fig, ax = plt.subplots(figsize=(img.shape[1] / 100, img.shape[0] / 100), dpi=100)
        ax.imshow(img); ax.axis("off")
        u, v = px(real_xy); ax.plot(u, v, "-", color="#1f9fff", lw=2.2, label="real car (localization)")
        u, v = px(sim_xy); ax.plot(u, v, ":", color="#ff4040", lw=1.2, label="Unity sim (MATLAB log)")
        ax.legend(loc="lower right", fontsize=12)
        fig.subplots_adjust(0, 0, 1, 1); fig.savefig(os.path.join(out, "compare_on_unity.png")); plt.close(fig)
        rep["unity_overlay"] = "compare_on_unity.png"
    else:
        rep["unity_overlay"] = "unity_top.png 없음 (Unity 없이 돌렸거나 저장 실패)"
    json.dump(rep, open(os.path.join(out, "report.json"), "w"), ensure_ascii=False, indent=1)

    r = rep["d_lat_same_s_m (주 지표: 실차 - 시뮬)"]
    print(f"[compare] 결과: {out}/")
    print(f"  거리: 시뮬 {rep['distance_m']['sim']:.2f} m · 실차 {rep['distance_m']['real']:.2f} m · 끝점 차이 {rep['end_point_gap_m']:.2f} m")
    print(f"  기준선 횡오차 RMS: 시뮬 {rep['e_lat_sim_vs_centerline_m'].get('rms')} m · 실차 {rep['e_lat_real_vs_centerline_m'].get('rms')} m")
    print(f"  같은 s 에서 횡오차 차이(실차-시뮬): RMS {r.get('rms')} m · 최대 {r.get('max_abs')} m  (n={r.get('n')})")
    print(f"  같은 시각: 옆 RMS {rep['d_lat_same_time_m'].get('rms')} m · 앞뒤 RMS {rep['d_lon_same_time_m'].get('rms')} m")
    if "est_kv_mps_per_pwm" in est:
        print(f"  속도 보정 추정 kv = {est['est_kv_mps_per_pwm']} m/s per PWM (시뮬에 쓴 값 {est['sim_kv_used']})")


if __name__ == "__main__":
    main()
