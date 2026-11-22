"""[demo] 실시간 ArUco SLAM — 주행 중 카메라를 이 프로세스가 맡는다 (주행 노드는 -p camera:=none).

  PYTHONPATH=~/aruco_slam_deps:~/aruco-slam/thirdparty/pangolin python3 -B live_slam.py --out DIR \\
      [--size 1280x720|1920x1080] [--no-2d] [--3d] [--no-video] [--ros] [--seconds N]

- 주행(차량 제어)과 완전히 분리된 프로세스: 이게 죽거나 느려져도 차는 wasd_drive·펌웨어 워치독대로 동작
- EKF = ~/demo/aruco-slam-fixed (add_marker 회전 수정본)
- 720p 로 찍으면 1080p 캘리브레이션 K 를 2/3 로 축소해 쓴다 (720p = 1080p 와 같은 화각, demo T8)
- 출력 DIR/
    trajectory_live.csv   frame,epoch,x,y,z,qw,qx,qy,qz,yaw_deg,n_markers,proc_ms   (카메라 광학 좌표: x右 y下 z前)
    video_live.avi        처리한 프레임 원본(그림 없음), MJPG — 나중에 run_slam 으로 다시 돌릴 수 있음
    map.txt · live.json   마지막 지도 · 설정/요약
    READY                 카메라 열리고 첫 프레임 처리되면 생김 (drive.sh 가 기다림)
- epoch = 프레임을 받은 직후의 벽시계. 주행 세션의 t_s = epoch − session.json 의 t0_epoch (merge_live.py)
- 종료: SIGINT/SIGTERM, 2D 창에서 q, 또는 --seconds
"""
import argparse
import glob
import json
import os
import signal
import sys
import threading
import time

import numpy as np

FIXED = os.path.expanduser("~/demo/aruco-slam-fixed")
sys.path.insert(0, FIXED)
sys.dont_write_bytecode = True

import cv2  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

STOP = threading.Event()


def find_c920():
    for d in sorted(glob.glob("/sys/class/video4linux/video*"), key=lambda p: int(p.rsplit("video", 1)[1] or 0)):
        try:
            name = open(os.path.join(d, "name")).read().strip()
            idx = open(os.path.join(d, "index")).read().strip()
        except OSError:
            continue
        if "c920" in name.lower() and idx == "0":
            return int(os.path.basename(d)[5:]), name
    return None, None


class Grabber(threading.Thread):
    """카메라를 계속 읽어 '가장 최근 프레임'만 남긴다 → 처리가 느려도 지연이 쌓이지 않음 (프레임은 건너뜀)."""

    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap, self.lock, self.item, self.seq, self.grabbed = cap, threading.Lock(), None, 0, 0

    def run(self):
        while not STOP.is_set():
            ok, fr = self.cap.read()
            t = time.time()
            if not ok:
                time.sleep(0.01)
                continue
            with self.lock:
                self.seq += 1
                self.grabbed += 1
                self.item = (self.seq, t, fr)

    def latest(self, after):
        with self.lock:
            if self.item is None or self.item[0] <= after:
                return None
            return self.item


def make_ros():
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Int32
    except ImportError as e:
        print(f"[live] ROS 사용 불가 ({e}) — --ros 없이 계속", flush=True)
        return None
    rclpy.init()
    node = rclpy.create_node("live_slam")
    pub = node.create_publisher(PoseStamped, "/car/cam_pose", 10)
    pub_n = node.create_publisher(Int32, "/car/cam_markers", 10)

    def publish(epoch, cam, n):
        m = PoseStamped()
        m.header.frame_id = "slam_map"
        m.header.stamp.sec = int(epoch)
        m.header.stamp.nanosec = int((epoch % 1) * 1e9)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, cam[:3])
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = map(float, cam[3:7])
        pub.publish(m)
        pub_n.publish(Int32(data=int(n)))

    def close():
        node.destroy_node()
        rclpy.shutdown()
    print("[live] ROS 발행: /car/cam_pose (PoseStamped, frame slam_map) · /car/cam_markers", flush=True)
    return publish, close


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--camera", type=int, help="/dev/videoN 의 N (기본: 이름 C920 으로 찾기)")
    ap.add_argument("--no-2d", dest="show_2d", action="store_false")
    ap.add_argument("--3d", dest="show_3d", action="store_true")
    ap.add_argument("--no-video", dest="video", action="store_false")
    ap.add_argument("--ros", action="store_true")
    ap.add_argument("--seconds", type=float, default=0)
    a = ap.parse_args()

    out = os.path.abspath(os.path.expanduser(a.out))
    os.makedirs(out, exist_ok=True)
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: STOP.set())
    os.chdir(FIXED)                              # calibration/ 상대경로
    from filters.extended_kalman_filter import EKF

    W, H = map(int, a.size.split("x"))
    idx, name = (a.camera, f"/dev/video{a.camera}") if a.camera is not None else find_c920()
    if idx is None:
        sys.exit("[live] C920 을 못 찾음")
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H); cap.set(cv2.CAP_PROP_FPS, a.fps)
    if not cap.isOpened():
        sys.exit(f"[live] 카메라 열기 실패 /dev/video{idx} — 다른 프로그램(주행 노드 녹화?)이 쓰는 중인지 확인")
    rw, rh = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (rw, rh) != (W, H):
        print(f"[live] 요청 {W}x{H} → 실제 {rw}x{rh}", flush=True)
        W, H = rw, rh

    tr = EKF(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    scale = W / 1920.0
    if abs(scale - 1) > 1e-6:
        tr.calib_matrix = tr.calib_matrix.copy()
        tr.calib_matrix[:2] *= scale             # fx, fy, cx, cy 를 해상도에 맞게
    v2d = v3d = None
    if a.show_2d:
        import viewers.viewer_2d as m2d
        v2d = m2d.Viewer2D(export_video=False)
        if abs(scale - 1) > 1e-6:
            v2d.calib_matrix = tr.calib_matrix   # 뷰어도 같은 K 로 투영
    if a.show_3d:
        import viewers.viewer_3d as m3d
        v3d = m3d.Viewer3D(export_video=False)
    ros = make_ros() if a.ros else None
    vw = cv2.VideoWriter(os.path.join(out, "video_live.avi"), cv2.VideoWriter_fourcc(*"MJPG"), a.fps, (W, H)) if a.video else None

    csv = open(os.path.join(out, "trajectory_live.csv"), "w")
    csv.write("frame,epoch,x,y,z,qw,qx,qy,qz,yaw_deg,n_markers,proc_ms\n")
    g = Grabber(cap); g.start()
    meta = {"camera": name, "device_index": idx, "size": [W, H], "K_scale": scale, "start_epoch": time.time(),
            "filter": "EKF (aruco-slam-fixed)", "video": bool(vw)}
    t_start, last_seq, n, det_n, last_print, ready = time.time(), 0, 0, 0, time.time(), False
    proc_ms = []
    try:
        while not STOP.is_set():
            if a.seconds and time.time() - t_start > a.seconds:
                break
            item = g.latest(last_seq)
            if item is None:
                time.sleep(0.002)
                continue
            last_seq, epoch, fr = item
            if vw is not None:
                vw.write(fr)                      # 그림 그리기 전 원본
            t0 = time.perf_counter()
            fr, cam, marks, det = tr.process_frame(fr)
            dt = (time.perf_counter() - t0) * 1000
            nm = 0 if det.shape == (0,) else len(det)
            det_n += nm > 0
            fwd = Rotation.from_quat(cam[3:7], scalar_first=True).apply([0, 0, 1])
            yaw = np.degrees(np.arctan2(fwd[0], fwd[2]))
            csv.write(f"{n},{epoch:.4f},{cam[0]:.4f},{cam[1]:.4f},{cam[2]:.4f},{cam[3]:.6f},{cam[4]:.6f},{cam[5]:.6f},{cam[6]:.6f},"
                      f"{yaw:.2f},{nm},{dt:.1f}\n")
            proc_ms.append(dt)
            n += 1
            if ros:
                ros[0](epoch, cam, nm)
            if not ready:
                open(os.path.join(out, "READY"), "w").write(f"{epoch}\n")
                ready = True
                print(f"[live] 시작: {name} {W}x{H} · 출력 {out}", flush=True)
            if v3d:
                v3d.view(cam, marks, det)
            if v2d:
                cv2.putText(fr, f"x {cam[0]:+.2f}  z {cam[2]:+.2f} m  yaw {yaw:+.0f}  markers {nm}", (20, 40 * H // 720),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0 * H / 720, (0, 255, 255), 2)
                if v2d.view(fr, cam, marks, det):
                    break
            if time.time() - last_print > 2:
                el = time.time() - t_start
                print(f"[live] {n / el:4.1f} fps 처리 (카메라 {g.grabbed / el:4.1f}) · 마커 {nm} · "
                      f"x {cam[0]:+.2f} z {cam[2]:+.2f} m yaw {yaw:+.0f}°", flush=True)
                last_print = time.time()
            if n % 30 == 0:
                csv.flush()
    finally:
        STOP.set()
        el = time.time() - t_start
        csv.close()
        if vw is not None:
            vw.release()
        try:
            tr.save_map(os.path.join(out, "map.txt"))
        except Exception as e:  # 마커를 한 번도 못 봤을 때 등
            print(f"[live] map 저장 실패: {e}", flush=True)
        for v in (v2d, v3d):
            if v:
                v.close()
        cv2.destroyAllWindows()
        cap.release()
        if ros:
            ros[1]()
        meta.update({"end_epoch": time.time(), "frames_processed": n, "frames_grabbed": g.grabbed,
                     "frames_with_marker": int(det_n), "fps_processed": round(n / el, 1) if el else None,
                     "fps_camera": round(g.grabbed / el, 1) if el else None,
                     "proc_ms_median": round(float(np.median(proc_ms)), 1) if proc_ms else None,
                     "landmarks": sorted(int(k) for k in tr.landmarks)})
        json.dump(meta, open(os.path.join(out, "live.json"), "w"), ensure_ascii=False, indent=1)
        print(f"[live] 종료: {n}프레임 처리 / {g.grabbed} 수신 · {meta['fps_processed']} fps · 마커 프레임 {det_n} · "
              f"랜드마크 {meta['landmarks']} → {out}/", flush=True)


if __name__ == "__main__":
    main()
