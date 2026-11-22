"""[demo] 녹화 영상 → 카메라 토픽 (하드웨어 없이 SLAM 노드 시험용, c920_camera 와 같은 토픽·QoS).

  ros2 run aruco_slam_ros video_publisher --ros-args -p video:=<mkv> -p camera_info_file:=<yaml> [-p rate:=1.0]

- MJPEG 영상이면 ffmpeg `-c:v copy` 로 **카메라가 보낸 JPEG 바이트 그대로** 발행 (재인코딩 없음).
  그 밖의 코덱은 디코드 후 JPEG(q95)로 인코딩해 발행.
- 프레임 시각: 영상 PTS 대로 (rate 배속). stamp = 발행 순간의 벽시계 (실제 카메라처럼).
- 끝나면 done 로그 후 종료 (launch 에서 on_exit 로 다른 노드도 내릴 수 있게).
"""
import aruco_slam_ros  # noqa: F401  ← numpy/cv2 보다 먼저
import array
import json
import os
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage


# 발행 QoS: reliable(유실 없이 — bag 녹화가 같은 QoS 로 받음), 최근 5장만 보관(느린 구독자가 발행을 막지 않게)
IMAGE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)


def probe(video):
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=codec_name,width,height",
                        "-of", "json", video], capture_output=True, text=True)
    s = json.loads(r.stdout)["streams"][0]
    pts = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time",
                          "-of", "csv=p=0", video], capture_output=True, text=True).stdout.split()
    pts = np.array(sorted(float(x) for x in pts if x not in ("", "N/A")))
    return s["codec_name"], int(s["width"]), int(s["height"]), pts


def mjpeg_frames(video):
    """ffmpeg 로 MJPEG 패킷을 그대로 꺼내 SOI(ffd8)~EOI(ffd9) 단위로 자른다 (엔트로피 데이터의 FF 는 FF00 으로 채워져 EOI 와 안 겹침)."""
    p = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-i", video, "-map", "0:v:0", "-c:v", "copy",
                          "-f", "mjpeg", "-"], stdout=subprocess.PIPE)
    buf = b""
    while True:
        chunk = p.stdout.read(1 << 20)
        if not chunk:
            break
        buf += chunk
        while True:
            s = buf.find(b"\xff\xd8")
            e = buf.find(b"\xff\xd9", s + 2) if s >= 0 else -1
            if e < 0:
                break
            yield buf[s:e + 2]
            buf = buf[e + 2:]
    p.wait()


def other_frames(video):
    cap = cv2.VideoCapture(video)
    while True:
        ok, f = cap.read()
        if not ok:
            break
        yield cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher")
        p = {k: self.declare_parameter(k, v).value for k, v in {
            "video": "", "camera_info_file": "", "rate": 1.0, "frame_id": "camera_optical", "start_delay": 1.0}.items()}
        self.p = p
        video = os.path.expanduser(p["video"])
        if not os.path.exists(video):
            raise RuntimeError(f"영상 없음: {video}")
        self.codec, self.w, self.h, self.pts = probe(video)
        self.frames = mjpeg_frames(video) if self.codec == "mjpeg" else other_frames(video)
        self.info = CameraInfo()
        self.info.header.frame_id = p["frame_id"]
        self.info.width, self.info.height = self.w, self.h
        if p["camera_info_file"]:
            c = yaml.safe_load(open(os.path.expanduser(p["camera_info_file"])))
            if (c["image_width"], c["image_height"]) != (self.w, self.h):
                raise RuntimeError(f"camera_info {c['image_width']}x{c['image_height']} ≠ 영상 {self.w}x{self.h}")
            self.info.distortion_model = c.get("distortion_model", "plumb_bob")
            self.info.d = [float(x) for x in c["distortion_coefficients"]["data"]]
            self.info.k = [float(x) for x in c["camera_matrix"]["data"]]
            self.info.r = [float(x) for x in c["rectification_matrix"]["data"]]
            self.info.p = [float(x) for x in c["projection_matrix"]["data"]]
        self.pub = self.create_publisher(CompressedImage, "image_raw/compressed", IMAGE_QOS)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", IMAGE_QOS)
        self.get_logger().info(f"{video} {self.codec} {self.w}x{self.h} {len(self.pts)}프레임 {self.pts[-1]:.1f}s "
                               f"→ {self.resolve_topic_name('image_raw/compressed')} ({p['rate']}배속, "
                               f"{'JPEG 그대로' if self.codec == 'mjpeg' else 'JPEG 재인코딩'})")

    def run(self):
        t_wait = time.monotonic() + 20.0                       # 구독자(SLAM 노드)가 붙을 때까지 최대 20 s
        while self.pub.get_subscription_count() == 0 and time.monotonic() < t_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            self.get_logger().warn("구독자 없음 — 그대로 시작")
        time.sleep(self.p["start_delay"])                     # 연결 안정화
        t0 = time.monotonic()
        n = 0
        for i, data in enumerate(self.frames):
            if i < len(self.pts):
                wait = t0 + (self.pts[i] - self.pts[0]) / self.p["rate"] - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
            now = time.time()
            m = CompressedImage()
            m.header.stamp.sec, m.header.stamp.nanosec = int(now), int((now % 1) * 1e9)
            m.header.frame_id = self.p["frame_id"]
            m.format = "jpeg"
            m.data = array.array("B", data)        # bytes 대입은 300 KB 에 33 ms (원소 검사) → array 는 즉시
            self.info.header.stamp = m.header.stamp
            self.pub.publish(m)
            self.pub_info.publish(self.info)
            n += 1
            rclpy.spin_once(self, timeout_sec=0)
            if not rclpy.ok():
                break
        el = time.monotonic() - t0
        self.get_logger().info(f"done: {n}프레임 {el:.1f}s ({n / el:.1f} fps)")


def main():
    rclpy.init()
    node = None
    try:
        node = VideoPublisher()
        node.run()
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"[video_publisher] {e}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)     # launch 의 이중 SIGINT
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
