"""[demo] 주행 bag 의 /car/state 점검 — 현장에서 주행 직후.

  ros2 run car_state state_report <run 폴더 또는 bag> [--plot]

보는 것 (움직일 때 = |v| ≥ 0.1 m/s)
  pose_valid 비율 · v 범위 · |v_lat| 중앙   → 0.05 m/s 넘으면 장착 yaw / 정합 / 미끄러짐 의심
  δ(POT) vs δ_from_motion(atan(L·ω/v)) 차이 → 조향 일정 구간(POT 흔들림 ≤ 4)에서 중앙 2° 넘으면 보정 다시 / 축간거리 확인
  전진 부호: PWM>0 인데 v<0 이 많으면 장착 yaw 180° 또는 자세 좌표 문제
출력: 터미널 요약 + <run>/state_report.json (+ --plot: state_report.png)
"""
import argparse
import json
import math
import os
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    p = os.path.abspath(os.path.expanduser(a.run))
    bag = p if os.path.exists(os.path.join(p, "metadata.yaml")) else os.path.join(p, "bag")
    run = os.path.dirname(bag)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if "/car/state" not in types or "CarState" not in types["/car/state"]:
        sys.exit(f"bag 에 /car/state(CarState) 없음: {types.get('/car/state')}")
    want = [t for t in ("/car/state", "/car/pwm") if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    S, PW = [], []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == "/car/state":
            S.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.x, m.y, m.theta, m.v, m.delta, m.yaw_rate,
                      m.v_lat, m.delta_from_motion, m.steer_pot, m.pose_valid, m.delta_valid, m.v_std))
        else:
            PW.append((t_ns * 1e-9, m.data))
    if not S:
        sys.exit("/car/state 0개")
    A = np.array(S, float)
    t, v, dl, vlat, dfm, pot, pv = A[:, 0], A[:, 4], A[:, 5], A[:, 7], A[:, 8], A[:, 9], A[:, 10].astype(bool)
    mv = pv & (np.abs(v) >= 0.1)
    rep = {"states": len(A), "duration_s": round(float(t[-1] - t[0]), 1), "pose_valid_pct": round(100 * pv.mean(), 1),
           "moving_pct": round(100 * mv.mean(), 1),
           "v_max": round(float(v[pv].max()), 3) if pv.any() else None, "v_min": round(float(v[pv].min()), 3) if pv.any() else None,
           "v_lat_abs_median_moving": round(float(np.nanmedian(np.abs(vlat[mv]))), 3) if mv.any() else None,
           "delta_valid_pct": round(100 * A[:, 11].mean(), 1)}
    warn = []
    if rep["pose_valid_pct"] < 90:
        warn.append(f"pose_valid {rep['pose_valid_pct']}% — 마커 안 보이는 구간 많음 (마커 보강·속도 낮추기)")
    if rep["v_lat_abs_median_moving"] and rep["v_lat_abs_median_moving"] > 0.05:
        warn.append(f"옆 방향 속도 중앙 {rep['v_lat_abs_median_moving']} m/s — 장착 yaw·정합 확인 (미끄러짐 없으면 ~0)")
    # 전진 부호: PWM>0 동안 v
    if PW:
        pt, pw = np.array(PW).T
        cur = pw[np.clip(np.searchsorted(pt, t) - 1, 0, None)] if len(pt) else np.zeros(len(t))
        fw = mv & (cur > 0)
        if fw.sum() > 10:
            rep["v_positive_while_pwm_forward_pct"] = round(100 * float((v[fw] > 0).mean()), 1)
            if rep["v_positive_while_pwm_forward_pct"] < 80:
                warn.append(f"전진 명령 중 v>0 이 {rep['v_positive_while_pwm_forward_pct']}% — 장착 yaw(180°?)·자세 부호 확인")
    # δ 검증: POT 일정 + 움직임 + 둘 다 값 있음
    if np.isfinite(dl).any() and np.isfinite(dfm).any():
        steady = np.zeros(len(A), bool)
        for i in range(len(A)):
            w = (t >= t[i] - 0.5) & (t <= t[i])
            steady[i] = np.ptp(pot[w]) <= 4
        g = mv & steady & np.isfinite(dl) & np.isfinite(dfm) & (np.abs(v) >= 0.15)
        if g.sum() > 10:
            e = np.degrees(dl[g] - dfm[g])
            rep["delta_check"] = {"n": int(g.sum()), "median_diff_deg": round(float(np.median(e)), 2),
                                  "median_abs_diff_deg": round(float(np.median(np.abs(e))), 2)}
            if abs(rep["delta_check"]["median_diff_deg"]) > 2:
                warn.append(f"δ(POT) − δ(운동) 중앙 {rep['delta_check']['median_diff_deg']}° — 조향 보정 다시 / 축간거리 확인")
    elif not np.isfinite(dl).any():
        warn.append("δ 전부 NaN — 조향 보정 없음 (calib_steer.sh)")
    rep["warnings"] = warn
    json.dump(rep, open(os.path.join(run, "state_report.json"), "w"), ensure_ascii=False, indent=1)
    print(f"[state_report] {rep['states']}개 · {rep['duration_s']} s · pose_valid {rep['pose_valid_pct']}% · 움직임 {rep['moving_pct']}% · "
          f"v {rep['v_min']}~{rep['v_max']} m/s · |v_lat| 중앙 {rep['v_lat_abs_median_moving']} · δ 유효 {rep['delta_valid_pct']}%"
          + (f" · δ 검증 중앙 {rep['delta_check']['median_diff_deg']}° (n={rep['delta_check']['n']})" if "delta_check" in rep else ""))
    for w in warn:
        print(f"  ⚠ {w}")
    if not warn:
        print("  ✓ 이상 없음")
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        tt = t - t[0]
        ax[0].plot(tt, v, label="v"); ax[0].plot(tt, vlat, label="v_lat", alpha=.6); ax[0].set_ylabel("m/s"); ax[0].legend()
        ax[1].plot(tt, np.degrees(dl), label="delta (POT)"); ax[1].plot(tt, np.degrees(dfm), ".", ms=2, label="delta from motion")
        ax[1].set_ylabel("deg"); ax[1].legend()
        ax[2].plot(tt, pv.astype(int), label="pose_valid"); ax[2].set_xlabel("t (s)"); ax[2].legend()
        for x in ax:
            x.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(run, "state_report.png"), dpi=100)
        print(f"  그림: {os.path.join(run, 'state_report.png')}")


if __name__ == "__main__":
    main()
