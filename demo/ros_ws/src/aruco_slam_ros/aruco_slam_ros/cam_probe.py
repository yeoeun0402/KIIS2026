"""P1 카메라 토픽 측정:  ros2 run aruco_slam_ros cam_probe [--topic /camera/image_raw/compressed] [--seconds 10] [--json out.json]

재는 것
  - 수신율(Hz)·프레임 간격(stamp 기준) 중앙값/최대 · 끊김(간격 > 1.5 × 중앙값) 수
  - 프레임 크기(KB)·대역폭(MB/s) · format 문자열 · JPEG 디코드 가능 여부와 해상도(앞 몇 장)
  - 지연: 수신 시각 − header.stamp (같은 PC 시계) 중앙값/95%
  - camera_info: 해상도·fx fy cx cy · frame_id
QoS 는 SLAM 노드와 같게 best-effort, depth 1 (센서 데이터 기본) — 이 QoS 로 받을 수 있어야 통과.
"""
import aruco_slam_ros  # noqa: F401  ← numpy/cv2 보다 먼저
import argparse
import json
import sys
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--info", default="/camera/camera_info")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--decode", type=int, default=5, help="앞에서 몇 장을 디코드해 볼지")
    ap.add_argument("--json")
    a, _ = ap.parse_known_args()

    import cv2
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage

    rclpy.init()
    node = rclpy.create_node("cam_probe")
    recv, stamps, sizes, fmts, decoded, info = [], [], [], set(), [], {}

    def on_img(m):
        t = time.time()
        recv.append(t)
        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        sizes.append(len(m.data))
        fmts.add(m.format)
        if len(decoded) < a.decode:
            img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            decoded.append(None if img is None else img.shape[:2][::-1])
        info.setdefault("frame_id", m.header.frame_id)

    def on_info(m):
        info.update({"info_size": [m.width, m.height], "K": [round(m.k[0], 1), round(m.k[4], 1), round(m.k[2], 1), round(m.k[5], 1)],
                     "D": [round(x, 4) for x in m.d], "info_frame_id": m.header.frame_id})

    node.create_subscription(CompressedImage, a.topic, on_img, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, a.info, on_info, qos_profile_sensor_data)
    t_end = time.time() + a.seconds + 30          # 첫 프레임까지 최대 30 s 기다림
    started = None
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if recv and started is None:
            started = recv[0]
            t_end = started + a.seconds
    node.destroy_node()
    rclpy.shutdown()

    if len(recv) < 2:
        print(f"[cam_probe] {a.topic} 수신 {len(recv)}장 — 발행 안 됨 / QoS 불일치?")
        sys.exit(1)
    recv, stamps, sizes = map(np.array, (recv, stamps, sizes))
    dur = recv[-1] - recv[0]
    dt = np.diff(stamps)
    med = float(np.median(dt))
    lat = (recv - stamps) * 1000
    r = {
        "topic": a.topic, "frames": int(len(recv)), "seconds": round(float(dur), 2),
        "rate_hz": round((len(recv) - 1) / dur, 2),
        "stamp_dt_ms_median": round(med * 1000, 2), "stamp_dt_ms_max": round(float(dt.max()) * 1000, 1),
        "gaps_over_1.5x": int((dt > 1.5 * med).sum()), "stamp_nonmonotonic": int((dt <= 0).sum()),
        "kb_per_frame": round(float(sizes.mean()) / 1024, 1), "mb_per_s": round(float(sizes.sum()) / dur / 1e6, 2),
        "format": sorted(fmts), "decoded_sizes": decoded,
        "first_stamp_epoch": round(float(stamps[0]), 3), "first_recv_epoch": round(float(recv[0]), 3),
        "latency_ms_median": round(float(np.median(lat)), 1), "latency_ms_p95": round(float(np.percentile(lat, 95)), 1),
        **info,
    }
    for k, v in r.items():
        print(f"  {k:22s} {v}")
    if a.json:
        json.dump(r, open(a.json, "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
