"""[demo] 차량 상태 [x, y, θ, v, δ] — SLAM 자세 + 조향 POT → /car/state (car_msgs/CarState) · /car/odom.

  ros2 run car_state car_state_node --ros-args -p pose_topic:=/car/pose_track -p steer_calib:=<yaml> -p mount_file:=<yaml>

입력  pose_topic (PoseStamped, base_link 자세 — 트랙 정합 있으면 /car/pose_track, 없으면 /car/pose)
      /car/steering_pot (Int32, wasd_drive 가 펌웨어 "P <adc>" 를 발행)
출력  /car/state  CarState: x y θ (EKF 로 다듬은 자세) · v · yaw_rate (CTRV EKF) · δ (POT→steer_calib, 유격 보정)
                  · v_lat · delta_from_motion=atan(L·ω/v) (검증용) · 1σ · pose_valid/delta_valid · pose_age
      /car/odom   nav_msgs/Odometry (rviz·표준 도구용, twist = 차 좌표 v, ω)
발행  자세가 올 때마다 (카메라 속도, stamp = 카메라 프레임 시각) + 자세가 끊기면 10 Hz 로 pose_valid=false
θ     3D 자세의 base_link x 축을 바닥(xy)에 투영한 방향 — 장착 pitch/roll 이 맞아야 정확
"""
import math
import os

import numpy as np
import rclpy
from car_msgs.msg import CarState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32

from car_state.ekf import CtrvEkf
from car_state.steer import PotDirection, SteerCalib


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    R = quat_to_R(q.x, q.y, q.z, q.w)
    return math.atan2(R[1, 0], R[0, 0])


def read_wheelbase(mount_file):
    if not mount_file or not os.path.exists(os.path.expanduser(mount_file)):
        return 0.0
    import yaml
    return float((yaml.safe_load(open(os.path.expanduser(mount_file))) or {}).get("wheelbase", 0.0) or 0.0)


class CarStateNode(Node):
    def __init__(self):
        super().__init__("car_state")
        P = {k: self.declare_parameter(k, v).value for k, v in {
            "pose_topic": "/car/pose_track", "pot_topic": "/car/steering_pot", "steer_calib": "", "mount_file": "",
            "wheelbase": 0.0, "sigma_pos": 0.02, "sigma_theta_deg": 1.5, "sigma_acc": 2.0, "sigma_yaw_acc": 4.0,
            "pose_timeout": 0.3, "pot_timeout": 0.5, "v_min_for_check": 0.15, "publish_odom": True,
            "child_frame": "base_link"}.items()}
        self.P = P
        self.L = float(P["wheelbase"]) or read_wheelbase(P["mount_file"])
        self.ekf = CtrvEkf(sigma_pos=P["sigma_pos"], sigma_theta=math.radians(P["sigma_theta_deg"]),
                           sigma_acc=P["sigma_acc"], sigma_yaw_acc=P["sigma_yaw_acc"])
        self.calib = None
        f = os.path.expanduser(P["steer_calib"]) if P["steer_calib"] else ""
        if f and os.path.exists(f):
            self.calib = SteerCalib.load(f)
            self.get_logger().info(f"조향 보정 {f}: POT {self.calib.pot[0]:.0f}~{self.calib.pot[-1]:.0f} → "
                                   f"δ {math.degrees(self.calib.delta.min()):+.1f}~{math.degrees(self.calib.delta.max()):+.1f}° · "
                                   f"유격 {self.calib.backlash:.1f} ADC")
        else:
            self.get_logger().warn(f"조향 보정 파일 없음 ({f or '미지정'}) → delta = NaN. 현장에서 calib_steer.sh 로 만들 것")
        if self.L <= 0:
            self.get_logger().warn("축간거리(wheelbase) 0 → delta_from_motion = NaN. mount.yaml 의 wheelbase 실측할 것")
        self.pot, self.pot_rx, self.pdir = -1, None, PotDirection()
        self.hist = []                       # (t, x, y) 최근 0.3 s — v_lat
        self.last_pose_t = None
        self.last_pose_rx = None             # 수신 시각 (끊김 판정은 이걸로 — bag 재생 때 stamp 는 과거)
        self.frame = "map"
        self.last_msg = None
        self.n = 0
        self.vmax = 0.0
        self.vlat_abs = []
        self.pub = self.create_publisher(CarState, "/car/state", 10)
        self.pub_odom = self.create_publisher(Odometry, "/car/odom", 10) if P["publish_odom"] else None
        self.create_subscription(PoseStamped, P["pose_topic"], self.on_pose, 10)
        self.create_subscription(Int32, P["pot_topic"], self.on_pot, 50)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f"자세 {P['pose_topic']} · POT {P['pot_topic']} · L={self.L:.3f} m → /car/state"
                               + (" · /car/odom" if self.pub_odom else ""))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pot(self, m):
        self.pot = int(m.data)
        self.pot_rx = self.now_s()
        self.pdir.update(self.pot)

    def on_pose(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.position, m.pose.orientation
        th = yaw_of(q)
        r = self.ekf.update(t, [p.x, p.y, th])
        if r in ("skip", "rejected"):                 # 버린 측정이면 상태 그대로 → 발행 안 함
            return
        if r == "reset":
            self.get_logger().warn(f"EKF 다시 시작 ({r}) — 자세가 크게 튐/끊김", throttle_duration_sec=5.0)
        self.frame = m.header.frame_id or self.frame
        self.last_pose_t = t
        self.last_pose_rx = self.now_s()
        self.hist = [h for h in self.hist if t - h[0] <= 0.3] + [(t, p.x, p.y)]
        self.publish(t)

    def on_timer(self):
        if self.last_msg is None or self.last_pose_rx is None:
            return
        gap = self.now_s() - self.last_pose_rx
        if gap > self.P["pose_timeout"]:
            m = self.last_msg
            m.pose_valid = False
            m.pose_age = float(self.now_s() - self.last_pose_t)
            age = gap
            self.fill_delta(m)
            self.pub.publish(m)
            self.get_logger().warn(f"자세 {age:.1f}s 끊김 (마커 안 보임?) → pose_valid=false", throttle_duration_sec=5.0)

    def fill_delta(self, m):
        m.steer_pot = int(self.pot)
        fresh = self.pot_rx is not None and self.now_s() - self.pot_rx <= self.P["pot_timeout"]
        if self.calib is not None and self.pot >= 0:
            d, ok = self.calib(self.pot, self.pdir.dir)
            m.delta, m.delta_valid = d, bool(ok and fresh)
        else:
            m.delta, m.delta_valid = float("nan"), False

    def publish(self, t, valid=True):
        s, sd = self.ekf.s, self.ekf.std()
        m = CarState()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1_000_000_000
        m.header.frame_id = self.frame
        m.x, m.y, m.theta, m.v, m.yaw_rate = (float(v) for v in s)
        m.x_std, m.y_std, m.theta_std, m.v_std = (float(v) for v in sd[:4])
        # 옆 방향 속도: 최근 0.3 s 변위를 차 좌표 y 로
        if len(self.hist) >= 3 and self.hist[-1][0] - self.hist[0][0] > 0.1:
            (t0, x0, y0), (t1, x1, y1) = self.hist[0], self.hist[-1]
            m.v_lat = float((-(x1 - x0) * math.sin(s[2]) + (y1 - y0) * math.cos(s[2])) / (t1 - t0))
        else:
            m.v_lat = float("nan")
        if self.L > 0 and abs(s[3]) >= self.P["v_min_for_check"]:
            m.delta_from_motion = float(math.atan(self.L * s[4] / s[3]))
        else:
            m.delta_from_motion = float("nan")
        self.fill_delta(m)
        m.pose_valid, m.pose_age = bool(valid), max(0.0, float(self.now_s() - t))
        self.pub.publish(m)
        self.last_msg = m
        self.n += 1
        self.vmax = max(self.vmax, abs(m.v))
        if not math.isnan(m.v_lat) and abs(m.v) > 0.1:
            self.vlat_abs.append(abs(m.v_lat))
        if self.pub_odom:
            o = Odometry()
            o.header = m.header
            o.child_frame_id = self.P["child_frame"]
            o.pose.pose.position.x, o.pose.pose.position.y = m.x, m.y
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(m.theta / 2), math.cos(m.theta / 2)
            o.twist.twist.linear.x, o.twist.twist.angular.z = m.v, m.yaw_rate
            cov = [0.0] * 36
            cov[0], cov[7], cov[35] = m.x_std ** 2, m.y_std ** 2, m.theta_std ** 2
            o.pose.covariance = cov
            tc = [0.0] * 36
            tc[0], tc[35] = m.v_std ** 2, float(self.ekf.P[4, 4])
            o.twist.covariance = tc
            self.pub_odom.publish(o)

    def summary(self):
        st = self.ekf.stats
        vl = f"{np.median(self.vlat_abs):.3f}" if self.vlat_abs else "-"
        return (f"발행 {self.n} · 최고 |v| {self.vmax:.2f} m/s · |v_lat| 중앙 {vl} m/s (움직일 때) · "
                f"EKF 갱신 {st['updates']} / 버림 {st['rejected']} / 재시작 {st['resets']}")


def main():
    rclpy.init()
    node = None
    try:
        node = CarStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        import signal
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            print(f"[car_state] {node.summary()}", flush=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
