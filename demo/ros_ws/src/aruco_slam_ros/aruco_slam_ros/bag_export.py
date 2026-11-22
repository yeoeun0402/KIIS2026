"""[demo] ros2 bag → 세션 폴더(video.mkv · video_frames.csv · session.json · drive_log.csv) → 결과 영상.

  ros2 run aruco_slam_ros bag_export <bag 폴더> [--out <dir>] [--render clean|full|none] [--landmarks <csv>]

- 영상: /camera/image_raw/compressed 의 JPEG 바이트를 그대로 mkv 에 (ffmpeg -c copy, 재인코딩 없음)
- 프레임 시각: header.stamp (카메라 드라이버 시각) − 첫 프레임 stamp → video_frames.csv (t_s)
- 주행 명령: /car/pwm → "V<n>", /car/steer_cmd → L/R/F (bag 수신 시각) → drive_log.csv (render_localization 형식)
- --render: ~/demo/live/render_clean.py (기본, desiered.png 형식) 또는 render_localization.py (full 대시보드)
  렌더러는 같은 EKF 로 SLAM 을 다시 돌린다 — bag 재생 결과가 실시간과 같음은 P3 에서 확인 (0.000 cm)
출력 기본: <bag 의 run 폴더>/export/<run 이름>/ , 영상은 <run 폴더>/render/clean.mp4
"""
import aruco_slam_ros  # noqa: F401
import argparse
import csv
import json
import os
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out")
    ap.add_argument("--render", default="clean", choices=["clean", "full", "none"])
    ap.add_argument("--landmarks")
    a = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = os.path.abspath(os.path.expanduser(a.bag))
    run = os.path.dirname(bag)
    name = os.path.basename(run.rstrip("/"))
    out = os.path.abspath(os.path.expanduser(a.out)) if a.out else os.path.join(run, "export", name)   # 폴더 이름 = 영상 제목
    os.makedirs(out, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {"/camera/image_raw/compressed", "/car/pwm", "/car/steer_cmd"}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    if "/camera/image_raw/compressed" not in types:
        sys.exit(f"bag 에 영상 토픽 없음: {sorted(types)}")
    cls = {t: get_message(types[t]) for t in want if t in types}

    ff = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "mjpeg", "-framerate", "30", "-i", "-",
                           "-c:v", "copy", os.path.join(out, "video.mkv")], stdin=subprocess.PIPE)
    stamps, events = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == "/camera/image_raw/compressed":
            ff.stdin.write(bytes(m.data))
            stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        elif topic == "/car/pwm":
            events.append((t_ns * 1e-9, "tx", f"V{m.data}"))
        elif topic == "/car/steer_cmd":
            events.append((t_ns * 1e-9, "tx", m.data))
    ff.stdin.close()
    ff.wait()
    if not stamps:
        sys.exit("영상 프레임 0")
    t0 = stamps[0]
    with open(os.path.join(out, "video_frames.csv"), "w") as f:
        f.write("frame,t_s\n")
        for i, s in enumerate(stamps):
            f.write(f"{i},{s - t0:.4f}\n")
    with open(os.path.join(out, "drive_log.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "kind", "value"])
        for t, k, v in sorted(events):
            w.writerow([f"{t - t0:.4f}", k, v])
    json.dump({"session": name, "t0_epoch": t0, "source": bag, "video": {"frames": len(stamps), "from": "ros2 bag"}},
              open(os.path.join(out, "session.json"), "w"), ensure_ascii=False, indent=1)
    dur = stamps[-1] - t0
    print(f"[bag_export] {bag} → {out}: 영상 {len(stamps)}프레임 {dur:.1f}s ({(len(stamps) - 1) / max(dur, 1e-6):.1f} fps) · "
          f"주행 명령 {len(events)}개", flush=True)
    if a.render == "none":
        return
    script = os.path.expanduser("~/demo/live/render_clean.py" if a.render == "clean" else "~/demo/live/render_localization.py")
    suffix = "clean" if a.render == "clean" else "localization"
    video_out = os.path.join(run, "render", f"{suffix}.mp4")
    cmd = [sys.executable, "-B", script, "--session", out, "--out", video_out]
    if a.landmarks:
        cmd += ["--landmarks", os.path.abspath(os.path.expanduser(a.landmarks))]
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([aruco_slam_ros.DEPS, os.path.expanduser("~/aruco-slam/thirdparty/pangolin"),
                                         env.get("PYTHONPATH", "")])
    print(f"[bag_export] 렌더: {' '.join(cmd)}", flush=True)
    r = subprocess.run(cmd, env=env)
    if r.returncode == 0:
        print(f"[bag_export] 영상: {video_out}", flush=True)
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
