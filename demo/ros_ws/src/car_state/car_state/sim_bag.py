"""[demo] 가짜 차량(sim.SimCar)으로 '보정 주행' bag 만들기 — 실차 없이 steer_calib · car_state 를 끝까지 시험.

  ros2 run car_state sim_bag <out 폴더> [--seed 0] [--backlash 6] [--wheelbase 0.26]

out/bag      : /car/pose_track (30 Hz, 잡음 1 cm · 0.7°) · /car/steering_pot (20 Hz) · /car/steer_cmd · /car/pwm
out/mount.yaml (wheelbase) · out/truth.json (진짜 POT→δ 함수 표본, 유격, 직진 POT) · out/truth.csv (시각별 진짜 v, ω, δ)
"""
import argparse
import json
import math
import os
import shutil

import numpy as np

from car_state.sim import SimCar, calib_script, run_script


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--backlash", type=float, default=6.0)
    ap.add_argument("--wheelbase", type=float, default=0.26)
    ap.add_argument("--t0", type=float, default=1.8e9, help="bag 시작 epoch (s)")
    a = ap.parse_args()
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import serialize_message
    from std_msgs.msg import Int16, Int32, String

    out = os.path.abspath(os.path.expanduser(a.out))
    os.makedirs(out, exist_ok=True)
    if os.path.exists(os.path.join(out, "bag")):
        shutil.rmtree(os.path.join(out, "bag"))
    car = SimCar(wheelbase=a.wheelbase, backlash_adc=a.backlash, seed=a.seed)
    sc, tend = calib_script()
    d = run_script(car, sc, t_end=tend)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=os.path.join(out, "bag"), storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    for name, typ in (("/car/pose_track", "geometry_msgs/msg/PoseStamped"), ("/car/steering_pot", "std_msgs/msg/Int32"),
                      ("/car/steer_cmd", "std_msgs/msg/String"), ("/car/pwm", "std_msgs/msg/Int16")):
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format="cdr"))
    ev = []
    for t, x, y, th in zip(d["pose_t"], d["x"], d["y"], d["th"]):
        m = PoseStamped()
        tt = a.t0 + t
        m.header.stamp = Time(sec=int(tt), nanosec=int((tt - int(tt)) * 1e9))
        m.header.frame_id = "track"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        ev.append((t + 0.03, "/car/pose_track", m))                  # 수신 = 카메라 시각 + 처리 지연
    for t, p in zip(d["pot_t"], d["pot"]):
        ev.append((t, "/car/steering_pot", Int32(data=int(p))))
    for t, c in zip(d["cmd_t"], d["cmd"]):
        ev.append((t, "/car/steer_cmd", String(data=c)))
    for t, c in sc:
        if isinstance(c, tuple):
            ev.append((t, "/car/pwm", Int16(data=int(c[1] * 100))))
    ev.sort(key=lambda e: e[0])
    for t, topic, m in ev:
        w.write(topic, serialize_message(m), int((a.t0 + t) * 1e9))
    del w
    with open(os.path.join(out, "mount.yaml"), "w") as f:
        f.write(f"x: 0.0\ny: 0.0\nz: 0.0\npitch_down_deg: 0.0\nyaw_deg: 0.0\nroll_deg: 0.0\nwheelbase: {a.wheelbase}\n")
    pots = np.arange(car.lo, car.hi + 1, 5.0)
    json.dump({"wheelbase": a.wheelbase, "backlash_adc": a.backlash, "pot_center": car.c,
               "pot": pots.tolist(), "delta_deg": [math.degrees(car.true_delta(p)) for p in pots],
               "duration_s": tend}, open(os.path.join(out, "truth.json"), "w"), indent=1)
    with open(os.path.join(out, "truth.csv"), "w") as f:
        f.write("stamp,v,w,delta\n")
        for t, v, wv, dl in zip(d["pose_t"], d["v_true"], d["w_true"], d["delta_true"]):
            f.write(f"{a.t0 + t:.4f},{v:.4f},{wv:.5f},{dl:.5f}\n")
    print(f"[sim_bag] {out}/bag: {tend:.0f}s · 자세 {len(d['pose_t'])} · POT {len(d['pot'])} · 조향 명령 {len(d['cmd'])} "
          f"(유격 {a.backlash} ADC, L={a.wheelbase} m)")


if __name__ == "__main__":
    main()
