"""[demo] 조향 보정: 주행 bag (스윕 + 저속 원호) → steer_calib.yaml (POT → 유효 조향각 δ).

  ros2 run car_state steer_calib <run 폴더 또는 bag> [<bag2> ...] [--wheelbase 0.26] [--out <yaml>] [--install]

입력 토픽: /car/steering_pot · /car/steer_cmd · /car/pose_track (없으면 /car/pose)
  스윕 (바퀴 띄우고 A 끝까지 → D 끝까지 → 가운데) → POT 범위 · 펄스당 ADC · L 이 POT 를 올리는지 · 반응 지연 · 잡음
  원호 (조향 고정, 저속 W → Space) → 구간마다 κ = dθ/ds → δ = atan(L·κ) → 표, 직진 POT, 유격
축간거리: --wheelbase > run 폴더 mount.yaml 의 wheelbase > car_bringup/config/mount.yaml
출력 (기본 <run>/steer_calib/): steer_calib.yaml · report.json · steer_calib.png
--install: src/car_bringup/config/steer_calib.yaml 로 복사 (이전 것은 .bak) → ./build.sh 후 car_state 가 씀
"""
import argparse
import datetime
import json
import math
import os
import shutil
import sys

import numpy as np

from car_state.steer import analyze_sweep, find_segments, fit_table

WS = os.path.expanduser("~/demo/ros_ws")
SRC_CFG = os.path.join(WS, "src/car_bringup/config")


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    pose_topic = "/car/pose_track" if "/car/pose_track" in types else "/car/pose"
    want = [t for t in ("/car/steering_pot", "/car/steer_cmd", pose_topic) if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    d = {"pose_topic": pose_topic, "pose_t": [], "x": [], "y": [], "th": [], "pot_t": [], "pot": [], "cmd_t": [], "cmd": []}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == "/car/steering_pot":
            d["pot_t"].append(t_ns * 1e-9); d["pot"].append(m.data)
        elif topic == "/car/steer_cmd":
            d["cmd_t"].append(t_ns * 1e-9); d["cmd"].append(m.data)
        else:
            q = m.pose.orientation
            d["pose_t"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            d["x"].append(m.pose.position.x); d["y"].append(m.pose.position.y)
            d["th"].append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    return d


def resolve_bag(p):
    p = os.path.abspath(os.path.expanduser(p))
    if os.path.exists(os.path.join(p, "metadata.yaml")):
        return p, os.path.dirname(p)
    if os.path.exists(os.path.join(p, "bag", "metadata.yaml")):
        return os.path.join(p, "bag"), p
    sys.exit(f"bag 을 못 찾음: {p}")


def wheelbase_from(run, arg):
    if arg:
        return arg, "--wheelbase"
    import yaml
    for f in (os.path.join(run, "mount.yaml"), os.path.join(SRC_CFG, "mount.yaml")):
        if os.path.exists(f):
            L = float((yaml.safe_load(open(f)) or {}).get("wheelbase", 0) or 0)
            if L > 0:
                return L, f
    return 0.0, None


def plot(path, d, segs, tab, sweep):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    a = ax[0]
    for s in segs:
        c = {1: "tab:red", -1: "tab:blue"}.get(s.get("pot_dir", 0), "gray")
        a.scatter(s["pot"], math.degrees(math.atan(tab["wheelbase"] * s["kappa"])), s=20 + 40 * s["length_m"], c=c, alpha=.7)
    a.plot(tab["table"]["pot"], tab["table"]["delta_deg"], "k-", lw=1.5, label="calibration table (backlash-corrected POT)")
    for pe in tab["extrapolated_pot"]:
        a.axvline(pe, color="gray", ls=":", lw=1)
    if "pot_center" in tab:
        a.axvline(tab["pot_center"], color="green", ls="--", lw=1, label=f"straight POT {tab['pot_center']:.0f}")
    a.axhline(0, color="k", lw=.5)
    a.set_xlabel("POT (ADC)"); a.set_ylabel("delta (deg, left +)")
    a.set_title(f"POT -> delta  (dots: arc segments, red approached POT up / blue down) | backlash {tab['backlash_adc']} ADC", fontsize=10)
    a.legend(loc="best", fontsize=8); a.grid(alpha=.3)
    a = ax[1]
    x, y, t = np.array(d["x"]), np.array(d["y"]), np.array(d["pose_t"])
    a.plot(x, y, color="lightgray", lw=1)
    cm = plt.get_cmap("coolwarm")
    for s in segs:
        m = (t >= s["t0"]) & (t <= s["t1"])
        dd = math.degrees(math.atan(tab["wheelbase"] * s["kappa"]))
        a.plot(x[m], y[m], color=cm(0.5 - dd / 60), lw=2.5)
    a.set_aspect("equal"); a.set_title(f"trajectory ({d['pose_topic']}), color = segment delta"); a.grid(alpha=.3)
    a = ax[2]
    pt = np.array(d["pot_t"]) - (t[0] if len(t) else 0)
    a.plot(pt, d["pot"], lw=.8)
    for tt, c in zip(d["cmd_t"], d["cmd"]):
        if c in ("L", "R"):
            a.axvline(tt - (t[0] if len(t) else 0), color="tab:red" if c == "L" else "tab:blue", lw=.3, alpha=.4)
    for s in segs:
        a.axvspan(s["t0"] - t[0], s["t1"] - t[0], color="tab:green", alpha=.15)
    a.set_xlabel("t (s)"); a.set_ylabel("POT")
    a.set_title(f"POT | lines L(red)/R(blue) | green = used segments | ADC/pulse L {sweep.get('adc_per_pulse_L')} R {sweep.get('adc_per_pulse_R')}", fontsize=10)
    a.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--wheelbase", type=float)
    ap.add_argument("--out")
    ap.add_argument("--install", action="store_true")
    ap.add_argument("--pot-tol", type=float, default=4.0, help="구간 안 POT 허용 흔들림 (ADC)")
    ap.add_argument("--v-min", type=float, default=0.08)
    a = ap.parse_args()
    bags = [resolve_bag(b) for b in a.bags]
    run = bags[0][1]
    L, Lsrc = wheelbase_from(run, a.wheelbase)
    if L <= 0:
        sys.exit("축간거리 모름: mount.yaml 의 wheelbase 를 실측해 넣거나 --wheelbase 0.xx")
    out = os.path.abspath(os.path.expanduser(a.out)) if a.out else os.path.join(run, "steer_calib")
    os.makedirs(out, exist_ok=True)
    sweeps, segs, data = [], [], None
    for bag, _ in bags:
        d = read_bag(bag)
        print(f"[steer_calib] {bag}: 자세 {len(d['pose_t'])} ({d['pose_topic']}) · POT {len(d['pot'])} · 조향 명령 {len(d['cmd'])}")
        if d["pose_topic"] != "/car/pose_track":
            print("  ⚠ /car/pose_track 없음 → /car/pose(map) 사용. 장착 pitch/roll 이 0 이면 지도 평면이 기울어 κ 가 조금 틀릴 수 있음")
        if len(d["pot"]) < 10:
            print("  ⚠ POT 기록 없음 — 보드 연결/펌웨어 확인"); continue
        sw = analyze_sweep(d["pot_t"], d["pot"], d["cmd_t"], d["cmd"])
        sweeps.append(sw)
        sg = find_segments(d["pose_t"], d["x"], d["y"], d["th"], d["pot_t"], d["pot"], d["cmd_t"], d["cmd"],
                           wheelbase=L, pot_tol=a.pot_tol, v_min=a.v_min)
        print(f"  스윕: POT {sw.get('pot_min')}~{sw.get('pot_max')} · 펄스당 L {sw.get('adc_per_pulse_L')} / R {sw.get('adc_per_pulse_R')} ADC · "
              f"반응 {sw.get('response_ms_L')} ms · 잡음 {sw.get('noise_adc')} · 원호 구간 {len(sg)}")
        segs += sg
        data = d if data is None or len(d["pose_t"]) > len(data["pose_t"]) else data
    if not sweeps:
        sys.exit("POT 데이터 없음")
    sweep = max(sweeps, key=lambda s: s.get("pot_max", 0) - s.get("pot_min", 0))
    if len(segs) < 3:
        json.dump({"sweep": sweep, "segments": segs}, open(os.path.join(out, "report.json"), "w"), ensure_ascii=False, indent=1)
        print(f"[steer_calib] ✗ 원호 구간 {len(segs)}개 — 조향 고정 + 저속({a.v_min} m/s 이상) + 1 s·0.4 m 이상 구간이 3개 이상 필요. "
              f"스윕 결과만 저장: {out}/report.json")
        sys.exit(3)
    tab = fit_table(segs, L, sweep=sweep)
    warn = []
    if not tab["hysteresis"]:
        warn.append("같은 조향을 양쪽에서 다가간 구간이 없어 유격을 못 잼 (0 으로 둠)")
    if "max_left_deg" not in tab:
        warn.append("좌·우 둘 다 있어야 함 — 한쪽 원호만 있음")
    if tab.get("poly_rms_deg", 0) > 1.5:
        warn.append(f"구간들이 매끈한 곡선에서 {tab['poly_rms_deg']}° 벗어남 — 미끄러짐/지도 오차 의심, 더 느리게")
    if any(s.get("circle_rms_cm", 0) > 5 for s in segs):
        warn.append("원 맞춤 잔차 5 cm 넘는 구간 있음 — 조향이 구간 중 움직였거나 자세가 튐")
    if sweep.get("left_increases_pot") is not None and "max_left_deg" in tab:
        # 일관성: L 이 POT 를 올리면 δ 가 POT 와 같이 증가해야
        inc = tab["table"]["delta_deg"][-1] > tab["table"]["delta_deg"][0]
        if inc != sweep["left_increases_pot"]:
            warn.append("스윕(L 방향)과 원호(δ 부호)가 서로 반대 — 펌웨어 L/R 극성 또는 자세 좌표 확인")
    y = {"generated": datetime.datetime.now().isoformat(timespec="seconds"), "source": [b for b, _ in bags],
         "wheelbase": L, "wheelbase_from": Lsrc, "table": tab["table"], "backlash_adc": tab["backlash_adc"],
         "pot_center": tab.get("pot_center"), "deg_per_adc_center": tab.get("deg_per_adc_center"),
         "max_left_deg": tab.get("max_left_deg"), "max_right_deg": tab.get("max_right_deg"),
         "extrapolated_pot": tab["extrapolated_pot"],
         "sweep": {k: sweep.get(k) for k in ("pot_min", "pot_max", "adc_per_pulse_L", "adc_per_pulse_R",
                                              "left_increases_pot", "response_ms_L", "response_ms_R", "noise_adc")}}
    import yaml
    head = ("# [demo] 조향 보정 — ros2 run car_state steer_calib 가 만듦. 손으로 고치지 말고 다시 측정할 것\n"
            "# table: POT(유격 보정된 값) → 유효 조향각 δ (°, 좌 +). car_state 는 pot − dir·backlash/2 로 찾는다\n")
    yp = os.path.join(out, "steer_calib.yaml")
    with open(yp, "w") as f:
        f.write(head)
        yaml.safe_dump(y, f, allow_unicode=True, sort_keys=False, default_flow_style=None)
    json.dump({"sweep": sweep, "fit": tab, "segments": segs, "warnings": warn}, open(os.path.join(out, "report.json"), "w"),
              ensure_ascii=False, indent=1)
    try:
        plot(os.path.join(out, "steer_calib.png"), data, segs, tab, sweep)
    except Exception as e:  # 그림 실패해도 보정은 유효
        print(f"  (그림 실패: {e})")
    print(f"[steer_calib] 원호 {len(segs)}구간 → 표 {tab['points']}점 · 직진 POT {tab.get('pot_center')} · "
          f"좌 최대 {tab.get('max_left_deg')}° / 우 {tab.get('max_right_deg')}° · 유격 {tab['backlash_adc']} ADC · "
          f"곡선 잔차 {tab.get('poly_rms_deg')}° (L={L} m, {Lsrc})")
    for w in warn:
        print(f"  ⚠ {w}")
    print(f"[steer_calib] 저장: {yp} · report.json · steer_calib.png")
    if a.install:
        dst = os.path.join(SRC_CFG, "steer_calib.yaml")
        if os.path.exists(dst):
            shutil.copy2(dst, dst + ".bak")
        shutil.copy2(yp, dst)
        print(f"[steer_calib] 설치: {dst} (이전 것 .bak) → ~/demo/ros_ws/build.sh 후 적용")


if __name__ == "__main__":
    main()
