"""[demo] 깔끔한 결과 영상 — 원본 aruco-slam README 의 combined.mp4 형식 (desiered.png), 글자 없음.

  위   3D 뷰 (흰 배경): 초록 궤적 · 검은 카메라 프러스텀 · 파란 지도 마커 · 지금 보이는 마커는 빨간 원
  아래 카메라 영상: 검출 마커마다 좌표축(빨강 x · 초록 y · 파랑 z) + 지도 마커 투영 점(파랑)

  python3 -B render_clean.py --session <세션폴더> [--live <live폴더>] [--landmarks track.csv] [--width 1280] [--out x.mp4]

SLAM 은 render_localization.py 와 같은 함수(aruco-slam-fixed EKF)로 돌린다. 3D 뷰는 Pangolin 모양을 OpenCV 로
그린 것(화면 없이 렌더 가능). --landmarks 가 있으면 트랙 좌표(실측 정합)로, 없으면 출발 카메라 기준으로 그린다.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.dont_write_bytecode = True
import cv2  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from render_localization import kabsch, run_slam, session_times  # noqa: E402

# BGR — Pangolin 뷰어(viewers/viewer_3d.py) 색과 같게
TRAJ = (68, 135, 0)          # 초록 (0,135,68)
LM = (231, 87, 0)            # 파랑 (0,87,231)
DET = (32, 45, 214)          # 빨강 (214,45,32)
WHITE = (252, 253, 251)


def look_at(eye, target, up=(0, 0, 1)):
    f = target - eye; f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.vstack([r, -u, f])          # 행: 화면 x(오른쪽), y(아래), z(앞)


class View3D:
    def __init__(self, pts, W, H):
        self.W, self.H = W, H
        lo, hi = pts.min(0), pts.max(0)
        self.c = (lo + hi) / 2
        span = max(hi[0] - lo[0], hi[1] - lo[1], 0.8)
        # 뒤쪽 위에서 비스듬히 내려다봄 (Pangolin 기본 시점과 비슷)
        d = span * 1.05
        self.eye = self.c + np.array([0.0, -d * 0.95, d * 0.85])
        self.R = look_at(self.eye, self.c)
        self.f = W * 0.9

    def px(self, P):
        P = np.atleast_2d(P)
        q = (P - self.eye) @ self.R.T
        z = np.maximum(q[:, 2], 1e-3)
        return np.c_[self.W / 2 + self.f * q[:, 0] / z, self.H / 2 + self.f * q[:, 1] / z].astype(np.int32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session"); ap.add_argument("--live"); ap.add_argument("--landmarks"); ap.add_argument("--out")
    ap.add_argument("--width", type=int, default=1280)
    a = ap.parse_args()
    sess = os.path.abspath(os.path.expanduser(a.session)) if a.session else None
    if a.live:
        live = os.path.abspath(os.path.expanduser(a.live)); video = os.path.join(live, "video_live.avi")
        ep = np.loadtxt(os.path.join(live, "trajectory_live.csv"), delimiter=",", skiprows=1, usecols=1, ndmin=1)
        times = ep - (json.load(open(os.path.join(sess, "session.json")))["t0_epoch"] if sess else ep[0])
        name = os.path.basename(sess) if sess else os.path.basename(live)
    else:
        video = os.path.join(sess, "video.mkv"); _, times = session_times(sess); name = os.path.basename(sess)
    out = os.path.abspath(os.path.expanduser(a.out)) if a.out else os.path.join(sess or live, "render", "clean.mp4")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    print(f"[clean] 1/2 SLAM: {video}")
    frames, times, tr, (sw, sh), scale = run_slam(video, times)
    K = tr.calib_matrix.copy(); D = tr.dist_coeffs

    # 표시 좌표: 트랙(실측 정합) 또는 SLAM(광학 x右 y下 z前 → 오른쪽·앞·위)
    last = frames[-1]["lm"]; Rt = None
    if a.landmarks:
        gt = {int(r[0]): r[1:] for r in np.loadtxt(os.path.expanduser(a.landmarks), delimiter=",", skiprows=1, ndmin=2)}
        ids = [k for k in sorted(last) if k in gt]
        if len(ids) >= 3:
            Rt = kabsch(np.array([last[k] for k in ids]), np.array([gt[k] for k in ids]))
    A = Rt[0] if Rt else np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
    b = Rt[1] if Rt else np.zeros(3)

    def W(p):
        return np.atleast_2d(p) @ A.T + b

    cams = np.array([f["cam"] for f in frames])
    Pw = W(cams[:, :3])
    Rcw = [A @ Rotation.from_quat(c[3:7], scalar_first=True).as_matrix() for c in cams]   # 카메라→표시 좌표 회전
    allp = np.vstack([Pw] + [W(v) for v in last.values()])

    OW = a.width; OH = OW * 9 // 16
    view = View3D(allp, OW, OH)
    # 프러스텀 (카메라 좌표, 원본 pangolin.DrawCamera 비율)
    s = 0.05 * max(np.ptp(allp[:, 0]), np.ptp(allp[:, 1]), 1.0)
    fr_c = np.array([[0, 0, 0], [-s, -s * 0.56, s], [s, -s * 0.56, s], [s, s * 0.56, s], [-s, s * 0.56, s]])
    axis_len = 0.075

    enc = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                            "-s", f"{OW}x{OH * 2}", "-r", "30", "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "21",
                            "-pix_fmt", "yuv420p", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    cap = cv2.VideoCapture(video)
    n_out = int(np.ceil((times[-1] - times[0]) * 30)) + 1
    src_i, src = -1, None
    t0 = time.time()
    print(f"[clean] 2/2 영상: {n_out} 프레임 → {out}")
    for j in range(n_out):
        want = int(np.searchsorted(times, times[0] + j / 30, side="right") - 1)
        while src_i < want:
            ok, src = cap.read(); src_i += 1
            if not ok:
                break
        f = frames[want]

        # ── 위: 3D 뷰 ──
        top = np.full((OH, OW, 3), WHITE, np.uint8)
        tr_px = view.px(Pw[:want + 1])
        if len(tr_px) > 1:
            cv2.polylines(top, [tr_px], False, TRAJ, 2, cv2.LINE_AA)
        vis = set(f["ids"])
        for k, L in f["lm"].items():
            p = view.px(W(L))[0]
            if k in vis:
                ov = top.copy(); cv2.circle(ov, tuple(p), 11, DET, -1, cv2.LINE_AA)
                top = cv2.addWeighted(ov, 0.45, top, 0.55, 0)
            cv2.circle(top, tuple(p), 5, LM, -1, cv2.LINE_AA)
        corners = view.px(fr_c @ Rcw[want].T + Pw[want])
        for i in range(1, 5):
            cv2.line(top, tuple(corners[0]), tuple(corners[i]), (0, 0, 0), 1, cv2.LINE_AA)
            cv2.line(top, tuple(corners[i]), tuple(corners[1 + i % 4]), (0, 0, 0), 1, cv2.LINE_AA)

        # ── 아래: 카메라 + 축 + 지도 마커 투영 ──
        img = src.copy()
        if f["det"] is not None:
            for d in f["det"]:
                cv2.drawFrameAxes(img, K, D, d[3:6].astype(np.float64), d[:3].astype(np.float64), axis_len, max(2, sw // 480))
        cam = f["cam"]; Rm = Rotation.from_quat(cam[3:7], scalar_first=True).as_matrix()
        lm = np.array(list(f["lm"].values())) if f["lm"] else np.zeros((0, 3))
        if len(lm):
            pc = (lm - cam[:3]) @ Rm                       # 맵 → 카메라 (Rᵀ (L − t))
            ok = pc[:, 2] > 0.05
            if ok.any():
                uv, _ = cv2.projectPoints(pc[ok], np.zeros(3), np.zeros(3), K, D)
                for u in uv.reshape(-1, 2):
                    if 0 <= u[0] < sw and 0 <= u[1] < sh:
                        cv2.circle(img, (int(u[0]), int(u[1])), max(5, sw // 200), (139, 0, 0), -1, cv2.LINE_AA)
        bot = cv2.resize(img, (OW, OH), interpolation=cv2.INTER_AREA)
        enc.stdin.write(np.vstack([top, bot]).tobytes())
        if j and j % 300 == 0:
            print(f"  {j}/{n_out} ({j / (time.time() - t0):.1f} fps)", flush=True)
    enc.stdin.close(); enc.wait()
    print(f"[clean] 완료: {out}")


if __name__ == "__main__":
    main()
