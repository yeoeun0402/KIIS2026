"""[demo] ArUco EKF SLAM 노드 (aruco-slam-fixed) — 압축 영상 구독 → 차량 자세·지도 발행.

  ros2 run aruco_slam_ros aruco_slam_node --ros-args -p mount_file:=<mount.yaml> [-p out_dir:=<dir>]
  (보통은 ros2 launch car_bringup slam.launch.py)

구독  /camera/image_raw/compressed (best-effort, depth 1) · /camera/camera_info (K·D, 해상도에 맞는 값)
발행  /car/pose        PoseStamped  map 에서 본 base_link (차량 기준점 = 뒷축 중심 바닥)   ← 제어가 쓰는 값
      /car/cam_pose    PoseStamped  map 에서 본 camera_optical
      /car/path        nav_msgs/Path (base_link)
      /aruco/markers   MarkerArray  지도 마커 (보이는 것 빨강, 나머지 파랑, ID 글자)  ≤ 5 Hz
      /aruco/visible   Int32MultiArray 이번 프레임에 보인 ID
      /aruco/debug_image Image 640 폭 · 5 Hz (검출 테두리·ID, rviz 용)
      /aruco_slam/stats String(JSON) 1 Hz: 수신/처리 fps · 처리 ms · 지연 ms · 버린 프레임 · 마커 수
      TF   map → base_link (stamp = 영상 stamp).  base_link → camera_link → camera_optical 은 정적 TF(frames.launch.py)

좌표 (PLAN_ROS.md §2.1, frames.py)
  EKF 지도(slam) = 첫 프레임 camera_optical.  map = 첫 프레임 base_link.
  T_map_base = T_base_opt · T_slam_opt · T_base_opt⁻¹   (T_base_opt = 장착 실측 · 광학 축 회전)

처리 방식: 콜백은 최신 메시지만 보관, 작업 스레드가 꺼내 처리 → 처리가 느리면 중간 프레임을 버리고 지연은 쌓이지 않음.
out_dir 를 주면 trajectory_ros.csv · map.txt · summary.json 을 남긴다 (오프라인 결과와 비교용).
"""
import aruco_slam_ros  # noqa: F401  ← numpy/cv2 보다 먼저 (~/aruco_slam_deps)
import array
import json
import os
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from aruco_slam_ros.frames import FrameChain, T_base_link, yaw_deg

MOUNT_KEYS = ("x", "y", "z", "pitch_down_deg", "yaw_deg", "roll_deg")


def to_pose(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()                    # x y z w
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return p


class ArucoSlamNode(Node):
    def __init__(self):
        super().__init__("aruco_slam")
        defaults = {"image_topic": "/camera/image_raw/compressed", "info_topic": "/camera/camera_info",
                    "map_frame": "map", "base_frame": "base_link", "publish_tf": True,
                    "mount_file": "", "marker_size": 0.20, "decode_gray": True,
                    "path_max": 20000, "reliable": True, "debug_image_hz": 5.0, "debug_image_width": 640, "markers_hz": 5.0, "out_dir": ""}
        defaults.update({k: 0.0 for k in MOUNT_KEYS})
        p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        mount = {k: float(p[k]) for k in MOUNT_KEYS}
        if p["mount_file"]:
            y = yaml.safe_load(open(os.path.expanduser(p["mount_file"]))) or {}
            mount = {k: float(y.get(k, mount[k])) for k in MOUNT_KEYS}
        self.p, self.mount = p, mount
        self.chain = FrameChain(T_base_link(**mount))

        here = os.getcwd()
        os.chdir(aruco_slam_ros.SLAM_ROOT)                 # calibration/*.npy 상대경로
        from filters.extended_kalman_filter import EKF
        self.ekf = EKF(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
        os.chdir(here)
        self.K_file = self.ekf.calib_matrix.copy()          # 1080p 캘리브레이션 (camera_info 없을 때 대비)
        self.have_K = False

        self.pub_pose = self.create_publisher(PoseStamped, "/car/pose", 10)
        self.pub_cam = self.create_publisher(PoseStamped, "/car/cam_pose", 10)
        self.pub_path = self.create_publisher(Path, "/car/path", 1)
        self.pub_mk = self.create_publisher(MarkerArray, "/aruco/markers", 1)
        self.pub_vis = self.create_publisher(Int32MultiArray, "/aruco/visible", 10)
        self.pub_stats = self.create_publisher(String, "/aruco_slam/stats", 1)
        self.pub_dbg = self.create_publisher(Image, "/aruco/debug_image", 1)   # rviz 용 작은 영상 (검출 표시)
        self.t_dbg = 0.0
        self.tfb = TransformBroadcaster(self) if p["publish_tf"] else None
        self.path = Path()
        self.path.header.frame_id = p["map_frame"]

        self.lock, self.cv = threading.Lock(), threading.Condition()
        self.latest, self.n_recv, self.n_drop = None, 0, 0
        # reliable 로 받으면 전송 유실이 없고(재전송), depth 1 이라 오래된 프레임은 쌓이지 않음. false 면 best-effort
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=(
            ReliabilityPolicy.RELIABLE if p["reliable"] else ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CameraInfo, p["info_topic"], self.on_info, qos)
        self.create_subscription(CompressedImage, p["image_topic"], self.on_image, qos)
        self.create_timer(1.0, self.on_stats)

        self.csv = None
        if p["out_dir"]:
            os.makedirs(os.path.expanduser(p["out_dir"]), exist_ok=True)
            self.out = os.path.expanduser(p["out_dir"])
            self.csv = open(os.path.join(self.out, "trajectory_ros.csv"), "w")
            self.csv.write("seq,stamp,recv,done,cam_x,cam_y,cam_z,cam_qw,cam_qx,cam_qy,cam_qz,"
                           "base_x,base_y,base_z,base_yaw_deg,n_markers,ids,proc_ms,latency_ms\n")
        self.n_proc, self.n_det, self.proc_ms, self.lat_ms, self.t_mk = 0, 0, [], [], 0.0
        self.win = {"recv": 0, "proc": 0, "drop": 0, "proc_ms": [], "lat_ms": [], "t": time.time()}
        self.t_start = time.time()
        self.stop = threading.Event()
        self.th = threading.Thread(target=self.work, daemon=True)
        self.th.start()
        self.get_logger().info(f"구독 {p['image_topic']} · 장착 {mount} · 흑백 디코드 {p['decode_gray']}"
                               + (f" · 기록 {self.out}" if self.csv else ""))

    # ── 입력 ──
    def on_info(self, m):
        K = np.array(m.k, float).reshape(3, 3)
        if K[0, 0] <= 0:
            return
        if not self.have_K or not np.allclose(K, self.ekf.calib_matrix):
            self.ekf.calib_matrix = K
            if len(m.d):
                self.ekf.dist_coeffs = np.array(m.d, float).reshape(1, -1)
            self.have_K = True
            self.get_logger().info(f"camera_info {m.width}x{m.height}: fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} "
                                   f"cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}")

    def on_image(self, m):
        with self.cv:
            if self.latest is not None:
                self.n_drop += 1
                self.win["drop"] += 1
            self.latest = (m, time.time())
            self.n_recv += 1
            self.win["recv"] += 1
            self.cv.notify()

    # ── 처리 ──
    def work(self):
        warned = False
        while not self.stop.is_set():
            with self.cv:
                while self.latest is None and not self.stop.is_set():
                    self.cv.wait(0.2)
                if self.stop.is_set():
                    return
                m, recv = self.latest
                self.latest = None
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8),
                               cv2.IMREAD_GRAYSCALE if self.p["decode_gray"] else cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn("JPEG 디코드 실패", throttle_duration_sec=2.0)
                continue
            if not self.have_K:                            # camera_info 를 아직 못 받음 → 파일 K 를 해상도에 맞춰
                s = img.shape[1] / 1920.0
                self.ekf.calib_matrix = self.K_file.copy()
                self.ekf.calib_matrix[:2] *= s
                if not warned:
                    self.get_logger().warn(f"camera_info 없음 → 캘리브레이션 파일 K × {s:g} 사용")
                    warned = True
            t0 = time.perf_counter()
            corners, ids, _ = self.ekf.detector.detectMarkers(img)
            ids_l = [] if ids is None else ids.flatten().tolist()
            if ids_l:
                det = self.ekf.estimate_pose_of_markers(corners, ids.flatten(), self.p["marker_size"])
                self.ekf.observe(ids.flatten(), det)
            cam, _ = self.ekf.get_poses()
            dt = (time.perf_counter() - t0) * 1000
            if self.p["debug_image_hz"] > 0 and time.time() - self.t_dbg > 1.0 / self.p["debug_image_hz"]:
                self.t_dbg = time.time()
                try:
                    self.publish_debug(m.header, img, corners, ids_l)
                except Exception:  # noqa: BLE001
                    if not rclpy.ok():
                        return
            try:
                self.publish(m.header.stamp, stamp, recv, cam, ids_l, dt)
            except Exception:  # noqa: BLE001  종료 중 컨텍스트가 먼저 닫힘
                if not rclpy.ok():
                    return
                raise

    def publish_debug(self, header, img, corners, ids):
        w = int(self.p["debug_image_width"])
        s = w / img.shape[1]
        small = cv2.resize(img, (w, int(round(img.shape[0] * s))), interpolation=cv2.INTER_AREA)
        if small.ndim == 2:
            small = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
        for c, k in zip(corners, ids):
            q = (c.reshape(-1, 2) * s).astype(np.int32)
            cv2.polylines(small, [q], True, (61, 245, 168), 2, cv2.LINE_AA)
            cv2.putText(small, str(k), tuple(q[0] + [0, -6]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (61, 245, 168), 2, cv2.LINE_AA)
        m = Image()
        m.header = header
        m.height, m.width = small.shape[:2]
        m.encoding, m.step = "bgr8", small.shape[1] * 3
        m.data = array.array("B", small.tobytes())
        self.pub_dbg.publish(m)

    def publish(self, st, stamp, recv, cam, ids, proc_ms):
        T_base = self.chain.base_in_map(cam[:7])
        T_cam = self.chain.cam_in_map(cam[:7])
        mf = self.p["map_frame"]
        ps = PoseStamped(); ps.header.stamp, ps.header.frame_id, ps.pose = st, mf, to_pose(T_base)
        pc = PoseStamped(); pc.header.stamp, pc.header.frame_id, pc.pose = st, mf, to_pose(T_cam)
        self.pub_pose.publish(ps)
        self.pub_cam.publish(pc)
        if self.tfb:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = st, mf, self.p["base_frame"]
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T_base[:3, 3])
            q = Rotation.from_matrix(T_base[:3, :3]).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
            self.tfb.sendTransform(t)
        self.path.header.stamp = st
        self.path.poses.append(ps)
        if len(self.path.poses) > self.p["path_max"]:
            del self.path.poses[: len(self.path.poses) - self.p["path_max"]]
        self.pub_path.publish(self.path)
        self.pub_vis.publish(Int32MultiArray(data=[int(i) for i in ids]))
        now = time.time()
        if now - self.t_mk > 1.0 / self.p["markers_hz"]:
            self.publish_markers(st, set(ids))
            self.t_mk = now
        done = time.time()
        lat = (done - stamp) * 1000
        self.n_proc += 1
        self.n_det += bool(ids)
        self.proc_ms.append(proc_ms)
        self.lat_ms.append(lat)
        self.win["proc"] += 1
        self.win["proc_ms"].append(proc_ms)
        self.win["lat_ms"].append(lat)
        if self.csv:
            self.csv.write(f"{self.n_proc - 1},{stamp:.4f},{recv:.4f},{done:.4f},"
                           + ",".join(f"{v:.6f}" for v in cam[:7]) + ","
                           + f"{T_base[0, 3]:.4f},{T_base[1, 3]:.4f},{T_base[2, 3]:.4f},{yaw_deg(T_base):.2f},"
                           + f"{len(ids)},{' '.join(map(str, ids))},{proc_ms:.1f},{lat:.1f}\n")

    def publish_markers(self, st, visible):
        arr = MarkerArray()
        s = self.p["marker_size"]
        for k, i in self.ekf.landmarks.items():
            L = self.ekf.state[10 + 3 * i: 13 + 3 * i] if hasattr(self.ekf, "state") else None
            if L is None:
                continue
            P = self.chain.point_in_map(L)[0]
            vis = int(k) in visible
            for j, typ in enumerate((Marker.CUBE, Marker.TEXT_VIEW_FACING)):
                mk = Marker()
                mk.header.stamp, mk.header.frame_id = st, self.p["map_frame"]
                mk.ns, mk.id, mk.type, mk.action = ("aruco", "aruco_id")[j], int(k), typ, Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, P + (0, 0, 0.18 * j))
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = s if j == 0 else 0.12
                mk.scale.z = s if j == 0 else 0.12
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = ((0.84, 0.18, 0.13) if vis else (0.0, 0.34, 0.91)) + (1.0,)
                if j == 1:
                    mk.text = str(k)
                    mk.color.r = mk.color.g = mk.color.b = 0.1
                arr.markers.append(mk)
        self.pub_mk.publish(arr)

    def on_stats(self):
        w, now = self.win, time.time()
        el = max(now - w["t"], 1e-3)
        d = {"recv_fps": round(w["recv"] / el, 1), "proc_fps": round(w["proc"] / el, 1), "dropped": w["drop"],
             "proc_ms": round(float(np.median(w["proc_ms"])), 1) if w["proc_ms"] else None,
             "latency_ms": round(float(np.median(w["lat_ms"])), 1) if w["lat_ms"] else None,
             "landmarks": len(self.ekf.landmarks)}
        self.pub_stats.publish(String(data=json.dumps(d)))
        if w["proc"]:
            self.get_logger().info(f"수신 {d['recv_fps']} · 처리 {d['proc_fps']} fps · 처리 {d['proc_ms']} ms · "
                                   f"지연 {d['latency_ms']} ms · 버림 {d['dropped']} · 지도 마커 {d['landmarks']}",
                                   throttle_duration_sec=4.9)
        self.win = {"recv": 0, "proc": 0, "drop": 0, "proc_ms": [], "lat_ms": [], "t": now}

    def finish(self):
        self.stop.set()
        with self.cv:
            self.cv.notify_all()
        self.th.join(timeout=2.0)
        el = time.time() - self.t_start
        summ = {"frames_received": self.n_recv, "frames_processed": self.n_proc, "frames_dropped": self.n_drop,
                "frames_with_marker": self.n_det, "proc_ms_median": round(float(np.median(self.proc_ms)), 1) if self.proc_ms else None,
                "proc_ms_p95": round(float(np.percentile(self.proc_ms, 95)), 1) if self.proc_ms else None,
                "latency_ms_median": round(float(np.median(self.lat_ms)), 1) if self.lat_ms else None,
                "latency_ms_p95": round(float(np.percentile(self.lat_ms, 95)), 1) if self.lat_ms else None,
                "landmarks": sorted(int(k) for k in self.ekf.landmarks), "mount": self.mount,
                "decode_gray": self.p["decode_gray"], "wall_s": round(el, 1)}
        if self.csv:
            self.csv.close()
            try:
                self.ekf.save_map(os.path.join(self.out, "map.txt"))
            except Exception as e:  # noqa: BLE001
                summ["map_error"] = str(e)
            json.dump(summ, open(os.path.join(self.out, "summary.json"), "w"), ensure_ascii=False, indent=1)
        print(f"[aruco_slam] 종료: {json.dumps(summ, ensure_ascii=False)}", flush=True)


def main():
    rclpy.init()
    node = None
    try:
        node = ArucoSlamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)     # launch 의 이중 SIGINT
        if node is not None:
            node.finish()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
