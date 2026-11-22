"""[demo] 트랙 정합: 실측 마커 CSV ↔ SLAM 지도 → TF track → map (+ 트랙 좌표 차량 자세).

  ros2 run aruco_slam_ros track_aligner --ros-args -p landmarks:=<track_landmarks.csv>

입력  /aruco/markers (aruco_slam_node, ns "aruco", id = 마커 ID, 위치 = map 좌표)  ·  /car/pose (map 의 base_link)
      landmarks CSV: id,x_m,y_m,z_m — 트랙 좌표(오른손, z 위). Unity 는 왼손이므로 x=z_u, y=−x_u, z=y_u 로 바꿔 적을 것
출력  TF  track → map (/tf_static, 갱신될 때마다 다시 발행)  → 제어기는 lookup_transform('track', 'base_link') 로 바로 사용
      /car/pose_track  PoseStamped (frame track) · /track/landmarks MarkerArray (실측 위치, 회색)
      /track/align     String(JSON): 쓴 마커 · 마커별 잔차 cm · RMS · 회전 각 · 갱신 횟수
규칙
  - 겹치는 마커 ≥ min_markers(3) 이고 일직선이 아니어야 정합 (특이값 비 검사)
  - 기본 3D 전체 회전. yaw_only:=true 면 z 축 회전만 (장착 pitch/roll 을 실측해 mount.yaml 에 넣은 뒤에만 권장 —
    안 넣으면 카메라 기울기가 지도에 남아 잔차가 커짐: 복도 3D RMS 8.4 cm vs yaw-only 10.4 cm, demo P4)
  - SLAM 지도가 계속 다듬어지므로 period 초마다 다시 계산. 잔차 RMS 가 warn_rms_cm 넘으면 경고
"""
import aruco_slam_ros  # noqa: F401
import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def kabsch(src, dst, yaw_only):
    """dst ≈ R src + t."""
    cs, cd = src.mean(0), dst.mean(0)
    A, B = src - cs, dst - cd
    if yaw_only:
        H = A[:, :2].T @ B[:, :2]
        U, _, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R2 = Vt.T @ np.diag([1, d]) @ U.T
        R = np.eye(3); R[:2, :2] = R2
    else:
        U, _, Vt = np.linalg.svd(A.T @ B)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cd - R @ cs


class TrackAligner(Node):
    def __init__(self):
        super().__init__("track_aligner")
        p = {k: self.declare_parameter(k, v).value for k, v in {
            "landmarks": "", "track_frame": "track", "map_frame": "map", "min_markers": 3,
            "yaw_only": False, "period": 2.0, "warn_rms_cm": 10.0}.items()}
        self.p = p
        f = os.path.expanduser(p["landmarks"])
        if not os.path.exists(f):
            raise RuntimeError(f"landmarks CSV 없음: {f}")
        rows = np.loadtxt(f, delimiter=",", skiprows=1, ndmin=2)
        self.gt = {int(r[0]): r[1:4] for r in rows}
        self.slam = {}
        self.T = None
        self.n_upd = 0
        self.last = None
        self.stb = StaticTransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pose = self.create_publisher(PoseStamped, "/car/pose_track", 10)
        self.pub_info = self.create_publisher(String, "/track/align", latched)
        self.pub_gt = self.create_publisher(MarkerArray, "/track/landmarks", latched)
        self.create_subscription(MarkerArray, "/aruco/markers", self.on_markers, 10)
        self.create_subscription(PoseStamped, "/car/pose", self.on_pose, 10)
        self.create_timer(p["period"], self.align)
        self.publish_gt()
        self.get_logger().info(f"실측 마커 {sorted(self.gt)} ({f}) · {'yaw 만' if p['yaw_only'] else '3D'} 정합 · "
                               f"마커 {p['min_markers']}개 이상 보이면 TF {p['track_frame']} → {p['map_frame']}")

    def on_markers(self, m):
        for mk in m.markers:
            if mk.ns == "aruco":
                self.slam[mk.id] = np.array([mk.pose.position.x, mk.pose.position.y, mk.pose.position.z])

    def align(self, publish=True):
        ids = sorted(k for k in self.slam if k in self.gt)
        if len(ids) < self.p["min_markers"]:
            return
        S = np.array([self.slam[k] for k in ids])            # map
        D = np.array([self.gt[k] for k in ids])              # track
        sv = np.linalg.svd(D[:, :2] - D[:, :2].mean(0), compute_uv=False)
        if sv[-1] < 0.05 * sv[0]:
            self.get_logger().warn(f"실측 마커 {ids} 가 거의 일직선 → 정합 보류", throttle_duration_sec=10.0)
            return
        R, t = kabsch(S, D, self.p["yaw_only"])               # p_track = R p_map + t
        res = np.linalg.norm(S @ R.T + t - D, axis=1) * 100
        rms = float(np.sqrt((res ** 2).mean()))
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        changed = self.T is None or np.abs(T - self.T).max() > 1e-4
        self.T = T
        if changed and publish:
            self.n_upd += 1
            ts = TransformStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id, ts.child_frame_id = self.p["track_frame"], self.p["map_frame"]
            ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = map(float, t)
            q = Rotation.from_matrix(R).as_quat()
            ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = map(float, q)
            self.stb.sendTransform(ts)
        yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        info = {"ids": ids, "residual_cm": {str(k): round(float(r), 1) for k, r in zip(ids, res)}, "rms_cm": round(rms, 1),
                "yaw_deg": round(yaw, 2), "t": [round(float(x), 3) for x in t], "updates": self.n_upd,
                "mode": "yaw_only" if self.p["yaw_only"] else "3d"}
        self.last = info
        if not publish:
            return
        self.pub_info.publish(String(data=json.dumps(info)))
        if changed and (self.n_upd == 1 or self.n_upd % 10 == 0):
            self.get_logger().info(f"정합 #{self.n_upd}: 마커 {ids} 잔차 cm {[round(float(r), 1) for r in res]} RMS {rms:.1f} · yaw {yaw:+.1f}°")
        if rms > self.p["warn_rms_cm"]:
            self.get_logger().warn(f"정합 RMS {rms:.1f} cm > {self.p['warn_rms_cm']} — 실측 좌표/마커 ID 확인", throttle_duration_sec=10.0)

    def on_pose(self, m):
        if self.T is None:
            return
        P = np.eye(4)
        o = m.pose.orientation
        P[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        P[:3, 3] = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
        Q = self.T @ P
        out = PoseStamped()
        out.header.stamp, out.header.frame_id = m.header.stamp, self.p["track_frame"]
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, Q[:3, 3])
        q = Rotation.from_matrix(Q[:3, :3]).as_quat()
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = map(float, q)
        self.pub_pose.publish(out)

    def publish_gt(self):
        arr = MarkerArray()
        for k, g in self.gt.items():
            for j, typ in enumerate((Marker.CUBE, Marker.TEXT_VIEW_FACING)):
                mk = Marker()
                mk.header.frame_id = self.p["track_frame"]
                mk.ns, mk.id, mk.type = ("track_gt", "track_gt_id")[j], k, typ
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(g[0]), float(g[1]), float(g[2]) + 0.25 * j
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = 0.17 if j == 0 else 0.1
                mk.scale.z = 0.02 if j == 0 else 0.1
                mk.color.r = mk.color.g = mk.color.b = 0.45
                mk.color.a = 0.6 if j == 0 else 1.0
                if j:
                    mk.text = f"gt {k}"
                arr.markers.append(mk)
        self.pub_gt.publish(arr)


def main():
    rclpy.init()
    node = None
    try:
        node = TrackAligner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"[track_aligner] {e}")
    finally:
        import signal
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.align(publish=False)                  # 마지막 지도로 한 번 더 계산만 (컨텍스트는 이미 닫힘)
            if node.T is not None:
                print(f"[track_aligner] 최종 정합 (갱신 {node.n_upd}회): {node.last}", flush=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
