"""[demo] C920 카메라 노드 — MJPG 를 재인코딩 없이 발행, stamp = 드라이버 프레임 시각.

  ros2 run car_bringup c920_camera --ros-args -p width:=1920 -p height:=1080 -p camera_info_file:=...
  (보통은 ros2 launch car_bringup camera.launch.py)

발행 (네임스페이스 /camera 기준)
  image_raw/compressed  sensor_msgs/CompressedImage  format "jpeg", data = 카메라가 보낸 MJPG 그대로
  camera_info           sensor_msgs/CameraInfo       같은 stamp
  stamp: V4L2 버퍼 시각(CLOCK_MONOTONIC) → 벽시계로 변환. usb_cam 0.8.1 은 stamp 가 실행마다 0.7~1 s 어긋나
         (첫 프레임 stamp 가 노드 시작보다 앞섬, demo/out/p1) 쓰지 않는다.

- 읽기 스레드가 프레임마다 바로 발행 (큐 없음 → 지연 = 드라이버 지연 + 발행). V4L2 버퍼 4개.
- DHT(허프만 테이블)가 없는 MJPG 프레임이면 표준 테이블을 끼워 넣는다 (일부 디코더가 못 읽음).
- v4l2 컨트롤(초점·노출)은 시작할 때 넣고, check_period 초마다 다시 읽어 바뀌면 경고 + 재설정.
- 5 초마다 fps · KB/프레임 · 지연(벽시계 − stamp) 로그.
"""
import array
import os
import re
import signal
import subprocess
import threading
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

C920 = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"


def standard_dht():
    """libjpeg 기본(=JPEG 표준 Annex K) 허프만 테이블 세그먼트들을 인코더 출력에서 떼어 온다."""
    ok, enc = cv2.imencode(".jpg", np.zeros((16, 16, 3), np.uint8), [cv2.IMWRITE_JPEG_OPTIMIZE, 0])
    d = enc.tobytes()
    out, i = b"", 2
    while i < len(d) - 4 and d[i] == 0xFF:
        m, ln = d[i + 1], int.from_bytes(d[i + 2:i + 4], "big")
        if m == 0xC4:
            out += d[i:i + 2 + ln]
        if m == 0xDA:
            break
        i += 2 + ln
    return out


def load_camera_info(path, w, h, frame_id):
    m = CameraInfo()
    m.header.frame_id = frame_id
    m.width, m.height = w, h
    if not path:
        return m
    c = yaml.safe_load(open(path))
    m.distortion_model = c.get("distortion_model", "plumb_bob")
    m.d = [float(x) for x in c["distortion_coefficients"]["data"]]
    m.k = [float(x) for x in c["camera_matrix"]["data"]]
    m.r = [float(x) for x in c["rectification_matrix"]["data"]]
    m.p = [float(x) for x in c["projection_matrix"]["data"]]
    if (c.get("image_width"), c.get("image_height")) != (w, h):
        raise ValueError(f"camera_info {c.get('image_width')}x{c.get('image_height')} ≠ 영상 {w}x{h}")
    return m


class C920Camera(Node):
    def __init__(self):
        super().__init__("c920_camera")
        p = {k: self.declare_parameter(k, v).value for k, v in {
            "device": C920, "width": 1920, "height": 1080, "fps": 30, "frame_id": "camera_optical",
            "camera_info_file": "", "set_controls": True, "focus": 19, "exposure": 156, "check_period": 5.0,
            "log_period": 5.0}.items()}
        self.p = p
        dev = os.path.realpath(p["device"])
        mt = re.match(r"/dev/video(\d+)$", dev)
        if not mt:
            raise RuntimeError(f"장치 없음/형식 이상: {p['device']} → {dev}")
        self.dev = dev
        if p["set_controls"]:
            self.set_controls()
        cap = cv2.VideoCapture(int(mt.group(1)), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, p["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, p["height"])
        cap.set(cv2.CAP_PROP_FPS, p["fps"])
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)        # 디코드하지 말고 MJPG 바이트 그대로
        if not cap.isOpened():
            raise RuntimeError(f"카메라 열기 실패 {dev} — 다른 프로그램이 쓰는 중?")
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="replace")
        if (w, h) != (p["width"], p["height"]) or fourcc != "MJPG":
            raise RuntimeError(f"요청 {p['width']}x{p['height']} MJPG → 실제 {w}x{h} {fourcc}")
        self.cap, self.w, self.h = cap, w, h
        if p["set_controls"]:
            self.set_controls()                     # 장치를 연 뒤 다시 (열면서 바뀌는 경우 대비)
        self.info = load_camera_info(p["camera_info_file"], w, h, p["frame_id"])
        if not p["camera_info_file"]:
            self.get_logger().warn("camera_info_file 없음 → K 가 빈 camera_info 발행")
        self.pub = self.create_publisher(CompressedImage, "image_raw/compressed", IMAGE_QOS)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", IMAGE_QOS)
        self.dht = standard_dht()
        self.n, self.n_dht, self.bytes, self.ages, self.t_log = 0, 0, 0, [], time.time()
        self.stop = threading.Event()
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()
        if p["set_controls"] and p["check_period"] > 0:
            self.create_timer(p["check_period"], self.check_controls)
        self.get_logger().info(f"{dev} {w}x{h} MJPG {p['fps']} fps → {self.resolve_topic_name('image_raw/compressed')}"
                               f" · 컨트롤 {'고정 초점 %d 노출 %d' % (p['focus'], p['exposure']) if p['set_controls'] else '건드리지 않음'}")

    # ── v4l2 컨트롤 ──
    def want(self):
        return {"focus_automatic_continuous": 0, "focus_absolute": self.p["focus"], "auto_exposure": 1,
                "exposure_time_absolute": self.p["exposure"]}

    def set_controls(self):
        c = ",".join(f"{k}={v}" for k, v in self.want().items())
        r = subprocess.run(["v4l2-ctl", "-d", self.dev, "-c", c], capture_output=True, text=True)
        if r.returncode:
            self.get_logger().warn(f"v4l2-ctl 실패: {r.stderr.strip()}")

    def read_controls(self):
        out = subprocess.run(["v4l2-ctl", "-d", self.dev, "-l"], capture_output=True, text=True).stdout
        return {m.group(1): int(m.group(2)) for m in re.finditer(r"^\s*(\w+) 0x\w+ \(\w+\)\s*:.*?value=(-?\d+)", out, re.M)}

    def check_controls(self):
        cur = self.read_controls()
        bad = {k: cur.get(k) for k, v in self.want().items() if cur.get(k) != v}
        if bad:
            self.get_logger().warn(f"카메라 컨트롤이 바뀜 {bad} → 다시 설정")
            self.set_controls()

    # ── 읽기·발행 ──
    def loop(self):
        while not self.stop.is_set():
            ok, buf = self.cap.read()
            if not ok:
                self.get_logger().error("프레임 읽기 실패 (USB 분리?)", throttle_duration_sec=2.0)
                time.sleep(0.05)
                continue
            mono_buf = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0     # V4L2 버퍼 시각 (monotonic)
            wall, mono = time.time(), time.monotonic()
            stamp = wall - (mono - mono_buf) if 0 < mono_buf <= mono else wall
            data = buf.tobytes()
            if b"\xff\xc4" not in data[:4096]:                        # DHT 없음 → SOS 앞에 표준 테이블
                i = data.find(b"\xff\xda")
                if i > 0:
                    data = data[:i] + self.dht + data[i:]
                    self.n_dht += 1
            m = CompressedImage()
            m.header.stamp.sec = int(stamp)
            m.header.stamp.nanosec = int((stamp % 1) * 1e9)
            m.header.frame_id = self.p["frame_id"]
            m.format = "jpeg"
            m.data = array.array("B", data)        # bytes 대입은 300 KB 에 33 ms (원소 검사) → array 는 즉시
            self.info.header.stamp = m.header.stamp
            try:
                self.pub.publish(m)
                self.pub_info.publish(self.info)
            except Exception:  # noqa: BLE001  Ctrl+C 로 컨텍스트가 먼저 닫힘 → 조용히 끝냄
                if not rclpy.ok():
                    break
                raise
            self.n += 1
            self.bytes += len(data)
            self.ages.append((time.time() - stamp) * 1000)
            if time.time() - self.t_log > self.p["log_period"]:
                el = time.time() - self.t_log
                self.get_logger().info(f"{self.n / el:4.1f} fps · {self.bytes / max(self.n, 1) / 1024:5.0f} KB/프레임 · "
                                       f"{self.bytes / el / 1e6:4.1f} MB/s · 지연 {np.median(self.ages):4.0f} ms"
                                       + (f" · DHT 삽입 {self.n_dht}" if self.n_dht else ""))
                self.n, self.n_dht, self.bytes, self.ages, self.t_log = 0, 0, 0, [], time.time()

    def destroy_node(self):
        self.stop.set()
        self.th.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = C920Camera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"[c920_camera] {e}")
    finally:
        # launch 는 프로세스 그룹 SIGINT + 직접 SIGINT 를 둘 다 보낸다 → 정리 중 두 번째 신호 무시
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
