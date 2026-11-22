#!/usr/bin/env python3
"""마커 배치 확인용 실시간 카메라 화면 — 주행·지도 만들기 전에 차를 옮겨 가며 마커가 잘 보이는지 본다.

  python3 -B ~/demo/track/tools/marker_preview.py              (C920 을 drive.sh 와 같은 설정으로 엶 — drive.sh 와 동시에 못 씀)
  python3 -B ~/demo/track/tools/marker_preview.py --video ~/demo/sessions/<세션>/video.mkv   (녹화로 시험)
  ~/demo/drive.sh --no-render --preview     ← 수동 주행하면서 이 창을 같이 띄움 (drive.sh 가 쓰는 실시간 JPEG 를 읽음: --jpeg)
  마커 세트는 ~/demo/track/track.yaml markers.use (한 번만 다르게: MARKERS=old python3 ...)

화면
  맨 위 줄: 마커 세트 · 지금 보이는 마커 수 — 초록 = 2개 이상 (출발 맞춤·위치 계산이 잘 됨) · 노랑 = 1개 · 빨강 = 0개
  마커마다: 테두리 + ID · 차에서 거리 [m] · 방향 (+ 왼쪽) · 한 변 픽셀
    빨간 테두리 = 너무 작음 (한 변 < 25 px, 멀어서 위치가 흔들림) · 주황 = 비스듬함 (가로/세로 비 < 0.45, 각도 모호)
  아래 줄: 지금까지 한 번이라도 본 ID (가장 가까웠던 거리) · 같은 ID 두 장이 동시에 보이면 경고 (마커 중복 인쇄)
키: q 끝 · s 사진 저장 (~/demo/track/camera_check/preview_<시각>.jpg) · r 본 ID 목록 지우기
"""
import argparse
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trackmap as tm  # noqa: E402  (cv2 를 먼저 불러야 함)
import cv2  # noqa: E402
import numpy as np  # noqa: E402

from check_camera import find_c920  # noqa: E402

OUT = os.path.expanduser("~/demo/track/camera_check")
MIN_SIDE_PX = 25.0
MIN_ASPECT = 0.45


def open_camera(dev):
    for c in ("focus_automatic_continuous=0", "auto_exposure=1", "focus_absolute=19", "exposure_time_absolute=156"):
        subprocess.run(["v4l2-ctl", "-d", dev, "-c", c], capture_output=True)    # drive.sh 와 같은 초점·노출
    cap = cv2.VideoCapture(int(dev.replace("/dev/video", "")), cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    return cap


class JpegSource:
    """계속 덮어써지는 JPEG (ffmpeg -atomic_writing). 새 파일일 때만 (True, frame), 아니면 (False, None)."""
    def __init__(self, path):
        self.path, self.last = os.path.expanduser(path), None

    def read(self):
        try:
            st = os.stat(self.path)
        except FileNotFoundError:
            return False, None
        key = (st.st_ino, st.st_mtime_ns)
        if key == self.last:
            return False, None
        fr = cv2.imread(self.path)
        if fr is None:
            return False, None
        self.last = key
        return True, fr

    def release(self):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", help="기본: C920 자동 찾기")
    ap.add_argument("--video", help="카메라 대신 녹화 영상")
    ap.add_argument("--jpeg", help="카메라 대신 계속 덮어써지는 JPEG (drive.sh --preview 가 줌, 카메라는 주행 노드가 씀)")
    ap.add_argument("--width", type=int, default=1280, help="화면 폭 [px]")
    a = ap.parse_args()
    ms = tm.marker_set()
    size = ms["size_m"]
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))   # drive.sh 가 끝날 때 → 본 마커 목록 출력하고 끝
    if a.jpeg:
        cap = JpegSource(a.jpeg)
    elif a.video:
        cap = cv2.VideoCapture(os.path.expanduser(a.video))
    else:
        dev = a.device or find_c920()
        if not dev:
            sys.exit("C920 을 못 찾음 (--device /dev/videoN). drive.sh 가 켜져 있으면 끄고 다시")
        cap = open_camera(dev)
    det = tm.make_detector(ms["dictionary"])
    mount = tm.load_mount()
    R_bo, t_bo = tm.T_base_opt(mount)
    K = D = None
    seen = {}                                            # id → 가장 가까웠던 거리
    win = "marker preview  (q quit, s save, r reset)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    while not stop["now"]:
        ok, fr = cap.read()
        if not ok:
            if a.video:
                break
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
            continue
        if K is None or fr.shape[1] != getattr(main, "_w", 0):
            K, D = tm.load_intrinsics(fr.shape[1]); main._w = fr.shape[1]
        corners, ids, _ = det.detectMarkers(fr)
        vis = fr.copy()
        fs = fr.shape[1] / 1920.0                        # 글자·선 크기 (실시간 JPEG 는 폭 960)
        n = 0
        dup = []
        if ids is not None:
            ids = ids.ravel()
            n = len(set(ids.tolist()))
            dup = sorted({int(i) for i in ids if (ids == i).sum() > 1})
            for c, k in zip(corners, ids):
                c = c[0]
                sides = np.linalg.norm(np.diff(np.r_[c, c[:1]], axis=0), axis=1)
                side = float(sides.mean())
                aspect = float(min(sides[0] + sides[2], sides[1] + sides[3]) / max(sides[0] + sides[2], sides[1] + sides[3]))
                try:
                    _, t_cm, _ = tm.marker_poses_ippe(c, size, K, D)[0]
                    p = R_bo @ t_cm + t_bo                  # 마커 중심, 차 (뒷축) 좌표: x 앞, y 왼쪽
                    dist, brg = float(np.hypot(p[0], p[1])), float(np.degrees(np.arctan2(p[1], p[0])))
                except Exception:
                    dist, brg = float("nan"), float("nan")
                col = (0, 200, 0)
                if side < MIN_SIDE_PX:
                    col = (0, 0, 255)
                elif aspect < MIN_ASPECT:
                    col = (0, 140, 255)
                cv2.polylines(vis, [c.astype(np.int32)], True, col, max(2, int(4 * fs)))
                txt = f"ID {int(k)}  {dist:.2f} m  {brg:+.0f} deg  {side:.0f} px"
                org = (int(c[:, 0].min()), max(int(80 * fs), int(c[:, 1].min()) - int(12 * fs)))
                cv2.putText(vis, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 1.1 * fs, (0, 0, 0), max(2, int(6 * fs)), cv2.LINE_AA)
                cv2.putText(vis, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 1.1 * fs, col, max(1, int(2 * fs)), cv2.LINE_AA)
                if np.isfinite(dist):
                    seen[int(k)] = min(seen.get(int(k), 1e9), dist)
        top_col = (0, 200, 0) if n >= 2 else (0, 200, 255) if n == 1 else (0, 0, 255)
        msg = {0: "no marker", 1: "1 marker (position ambiguous)"}.get(n, f"{n} markers OK")
        head = f"set {ms['name']} ({ms['dictionary']}, {size * 100:.1f} cm)   visible: {msg}"
        cv2.rectangle(vis, (0, 0), (vis.shape[1], int(64 * fs)), (30, 30, 30), -1)
        cv2.putText(vis, head, (int(20 * fs), int(45 * fs)), cv2.FONT_HERSHEY_SIMPLEX, 1.4 * fs, top_col, max(1, int(3 * fs)), cv2.LINE_AA)
        foot = "seen: " + "  ".join(f"{i}({d:.1f}m)" for i, d in sorted(seen.items())) if seen else "seen: -"
        if dup:
            foot = f"DUPLICATE ID {dup} in view!   " + foot
        cv2.rectangle(vis, (0, vis.shape[0] - int(56 * fs)), (vis.shape[1], vis.shape[0]), (30, 30, 30), -1)
        cv2.putText(vis, foot[:150], (int(20 * fs), vis.shape[0] - int(18 * fs)), cv2.FONT_HERSHEY_SIMPLEX, 1.0 * fs,
                    (0, 0, 255) if dup else (230, 230, 230), max(1, int(2 * fs)), cv2.LINE_AA)
        h = int(vis.shape[0] * a.width / vis.shape[1])
        cv2.imshow(win, cv2.resize(vis, (a.width, h), interpolation=cv2.INTER_AREA))
        key = cv2.waitKey(1 if not a.video else 30) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            os.makedirs(OUT, exist_ok=True)
            p = os.path.join(OUT, "preview_" + time.strftime("%Y%m%d_%H%M%S") + ".jpg")
            cv2.imwrite(p, vis); print("저장:", p)
        if key == ord("r"):
            seen.clear()
    cap.release()
    cv2.destroyAllWindows()
    print("본 마커 (가장 가까웠던 거리):", ", ".join(f"ID {i} {d:.2f} m" for i, d in sorted(seen.items())) or "없음")


if __name__ == "__main__":
    main()
