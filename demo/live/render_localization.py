"""[demo] 수동 주행 + 로컬라이제이션 결과 영상 만들기 (MP4, 1920x1080, 30 fps).

  # 녹화 모드 세션 (주행 노드가 video.mkv 를 남긴 경우)
  python3 -B render_localization.py --session <세션폴더> [--landmarks track.csv] [--out out.mp4]
  # 실시간 모드 (live_slam.py 가 video_live.avi 를 남긴 경우) — 세션은 명령 타임라인용
  python3 -B render_localization.py --live <live폴더> --session <세션폴더> [...]

화면 구성
  왼쪽  카메라 영상 + 검출 마커(ID·거리·축)      오른쪽 위  위에서 본 지도: 궤적·마커·카메라 방향·화각
  아래  PWM 명령 · 조향 L/R · 보이는 마커 수 (현재 시각 커서)   오른쪽 아래  3D 사선 뷰
SLAM 은 aruco-slam-fixed 의 EKF(회전 수정본)로 영상 전체를 다시 돌린다 → 실시간 화면과 같은 알고리즘.
--landmarks (id,x_m,y_m,z_m, 오른손 좌표) 를 주면 마지막 지도를 Kabsch 로 정합해 트랙 좌표로 그린다.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time

import numpy as np

FIXED = os.path.expanduser("~/demo/aruco-slam-fixed")
FONT = os.path.expanduser("~/demo/live/fonts/NotoSansCJKkr-Regular.otf")
sys.path.insert(0, FIXED)
sys.dont_write_bytecode = True

import cv2  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

OUT_W, OUT_H, OUT_FPS = 1920, 1080, 30
CAM_W, CAM_H = 1280, 720
# BGR
BG, PANEL, INK, INK2, GRID = (251, 252, 252), (245, 246, 246), (11, 11, 11), (78, 81, 82), (226, 230, 231)
BLUE, ORANGE, AQUA, RED, GREEN = (214, 120, 42), (52, 104, 235), (122, 175, 27), (59, 59, 208), (12, 163, 12)
MARKER_LEN = None     # 마커 크기: track.yaml markers.size_m (base_filter.MARKER_SIZE 와 같음, 아래에서 읽음)


# ── 입력 ─────────────────────────────────────────────────────────
def session_times(sess):
    meta = json.load(open(os.path.join(sess, "session.json")))
    p = os.path.join(sess, "video_frames.csv")
    rows = list(csv.reader(open(p)))[1:] if os.path.exists(p) else []
    if rows:
        return meta, np.array([float(r[1]) for r in rows])
    v = meta.get("video") or {}
    if not v.get("first_frame_epoch"):
        return meta, None
    pts = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time",
                          "-of", "csv=p=0", os.path.join(sess, "video.mkv")], capture_output=True, text=True).stdout.split()
    pts = np.array(sorted(float(x) for x in pts if x not in ("", "N/A")))
    return meta, np.round(v["first_frame_epoch"] + pts - meta["t0_epoch"], 3)


def drive_events(sess):
    pwm, steer, keys = [], [], []
    p = os.path.join(sess, "drive_log.csv") if sess else None
    if not p or not os.path.exists(p):
        return pwm, steer, keys
    for r in csv.DictReader(open(p)):
        t = float(r["t_s"])
        if r["kind"] == "tx" and r["value"].startswith("V"):
            pwm.append((t, int(r["value"][1:])))
        elif r["kind"] == "tx" and r["value"] in ("L", "R"):
            steer.append((t, r["value"]))
        elif r["kind"] == "key":
            keys.append((t, r["value"]))
    return pwm, steer, keys


def kabsch(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cd - R @ cs


# ── 1단계: SLAM ──────────────────────────────────────────────────
def run_slam(video, times):
    os.chdir(FIXED)
    from filters.extended_kalman_filter import EKF
    tr = EKF(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    cap = cv2.VideoCapture(video)
    W, H = int(cap.get(3)), int(cap.get(4))
    s = W / 1920.0
    if abs(s - 1) > 1e-6:
        tr.calib_matrix = tr.calib_matrix.copy(); tr.calib_matrix[:2] *= s
    frames = []
    t0 = time.time()
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        corners, ids, _ = tr.detector.detectMarkers(fr)
        _, cam, marks, det = tr.process_frame(fr)
        frames.append({"cam": cam[:7].copy(), "ids": [] if ids is None else ids.flatten().tolist(),
                       "corners": [] if ids is None else [c[0] / s for c in corners],
                       "det": det.copy() if det.shape != (0,) else None,
                       "lm": {int(k): marks[i].copy() for k, i in tr.landmarks.items()}})
        if len(frames) % 300 == 0:
            print(f"  SLAM {len(frames)} 프레임 ({len(frames) / (time.time() - t0):.1f} fps)", flush=True)
    n = len(frames)
    if times is None or len(times) < n:
        fps = cap.get(cv2.CAP_PROP_FPS) or 30
        times = np.arange(n) / fps if times is None else np.concatenate([times, times[-1] + np.arange(1, n - len(times) + 1) / fps])
    return frames, times[:n], tr, (W, H), s


# ── 그리기 도우미 ─────────────────────────────────────────────────
def pil_text(img, items):
    """정적인 한글 라벨만 PIL 로 (프레임마다 쓰지 않음)."""
    from PIL import Image, ImageDraw, ImageFont
    im = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); d = ImageDraw.Draw(im)
    for (x, y), text, size, col, bold in items:
        f = ImageFont.truetype(FONT.replace("Regular", "Bold") if bold else FONT, size)
        d.text((x, y), text, font=f, fill=col[::-1])
    return cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)


def put(img, text, org, scale=0.6, col=INK, th=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, col, th, cv2.LINE_AA)


class MapView:
    """위에서 본 지도. 좌표: (u, v) — 트랙이면 (x, y), SLAM 이면 (x 오른쪽, z 앞)."""

    def __init__(self, box, pts):
        self.x0, self.y0, self.w, self.h = box
        lo, hi = pts.min(0), pts.max(0)
        span = max(hi - lo) * 1.15 + 0.5
        c = (lo + hi) / 2
        self.sc = min(self.w, self.h) / span
        self.c = c

    def px(self, uv):
        uv = np.atleast_2d(uv)
        x = self.x0 + self.w / 2 + (uv[:, 0] - self.c[0]) * self.sc
        y = self.y0 + self.h / 2 - (uv[:, 1] - self.c[1]) * self.sc
        return np.c_[x, y].astype(np.int32)

    def grid(self, img):
        step = 0.5 if self.w / self.sc < 6 else 1.0
        u0 = self.c[0] - self.w / 2 / self.sc; v0 = self.c[1] - self.h / 2 / self.sc
        for u in np.arange(np.floor(u0 / step) * step, u0 + self.w / self.sc, step):
            x = int(self.px([u, 0])[0, 0])
            if self.x0 <= x <= self.x0 + self.w:
                cv2.line(img, (x, self.y0), (x, self.y0 + self.h), GRID, 1)
        for v in np.arange(np.floor(v0 / step) * step, v0 + self.h / self.sc, step):
            y = int(self.px([0, v])[0, 1])
            if self.y0 <= y <= self.y0 + self.h:
                cv2.line(img, (self.x0, y), (self.x0 + self.w, y), GRID, 1)
        put(img, f"grid {step:g} m", (self.x0 + 8, self.y0 + self.h - 10), 0.45, INK2)


def oblique(P, az=np.radians(-35), el=np.radians(28)):
    """3D (u 오른쪽, v 앞, h 위) → 2D 사선 투영."""
    Rz = np.array([[np.cos(az), -np.sin(az), 0], [np.sin(az), np.cos(az), 0], [0, 0, 1]])
    Q = P @ Rz.T
    return np.c_[Q[:, 0], Q[:, 1] * np.sin(el) + Q[:, 2] * np.cos(el)]


# ── 2단계: 렌더 ──────────────────────────────────────────────────
def render(frames, times, tr, src_size, scale, video, out_mp4, sess, landmarks_csv, title):
    n = len(frames)
    K = tr.calib_matrix.copy(); D = tr.dist_coeffs
    if abs(scale - 1) > 1e-6:
        pass  # K 는 이미 소스 해상도용
    last_lm = frames[-1]["lm"]
    track = None
    if landmarks_csv:
        gt = {int(r[0]): r[1:] for r in np.loadtxt(landmarks_csv, delimiter=",", skiprows=1, ndmin=2)}
        ids = [k for k in sorted(last_lm) if k in gt]
        if len(ids) >= 3:
            R, t = kabsch(np.array([last_lm[k] for k in ids]), np.array([gt[k] for k in ids]))
            res = np.linalg.norm(np.array([last_lm[k] for k in ids]) @ R.T + t - np.array([gt[k] for k in ids]), axis=1)
            track = (R, t, gt, dict(zip(ids, res)))
            print(f"  트랙 정합: 마커 {ids} 잔차 cm {np.round(res * 100, 1).tolist()}")
        else:
            print(f"  [경고] 정합 불가 — 실측 CSV 와 겹치는 마커 {ids} (3개 이상 필요). SLAM 좌표로 그림")

    def to_world(p):           # SLAM 맵 → 표시 좌표 (u, v, h)
        p = np.atleast_2d(p)
        if track:
            q = p @ track[0].T + track[1]
            return np.c_[q[:, 0], q[:, 1], q[:, 2]]
        return np.c_[p[:, 0], p[:, 2], -p[:, 1]]     # 광학(x右,y下,z前) → (오른쪽, 앞, 위)

    cams = np.array([f["cam"] for f in frames])
    Pw = to_world(cams[:, :3])
    fwd = to_world(Rotation.from_quat(cams[:, 3:7], scalar_first=True).apply([0, 0, 1]) + cams[:, :3]) - Pw
    lm_w = {k: to_world(v)[0] for k, v in last_lm.items()}
    # heading: 트랙이면 +x 기준 반시계(수학 각), SLAM 이면 출발 정면 기준 오른쪽(+)
    head = np.degrees(np.arctan2(fwd[:, 1], fwd[:, 0])) if track else np.degrees(np.arctan2(fwd[:, 0], fwd[:, 1]))
    allpts = np.vstack([Pw[:, :2]] + [v[None, :2] for v in lm_w.values()] + ([np.array([g[:2] for g in track[2].values()])] if track else []))

    # 레이아웃
    map_box = (CAM_W + 30, 70, OUT_W - CAM_W - 60, 450)
    obl_box = (CAM_W + 30, 560, OUT_W - CAM_W - 60, 230)
    tl_box = (300, 830, OUT_W - 360, 210)
    mv = MapView(map_box, allpts)
    # 3D 사선 뷰 스케일
    all3 = np.vstack([Pw] + list(lm_w.values()))
    o = oblique(all3); olo, ohi = o.min(0), o.max(0)
    osc = min(obl_box[2] / max(ohi[0] - olo[0] + 0.6, 1e-3), obl_box[3] / max(ohi[1] - olo[1] + 0.6, 1e-3))
    oc = (olo + ohi) / 2

    def opx(P):
        q = oblique(np.atleast_2d(P))
        return np.c_[obl_box[2] / 2 + (q[:, 0] - oc[0]) * osc,
                     obl_box[3] / 2 - (q[:, 1] - oc[1]) * osc].astype(np.int32)   # 패널 안 좌표

    pwm, steer, keys = drive_events(sess)
    t_end = max(times[-1], pwm[-1][0] if pwm else 0)
    t_beg = min(0.0, times[0])

    def tx(t):
        return int(tl_box[0] + (t - t_beg) / max(t_end - t_beg, 1e-3) * tl_box[2])
    lanes = {"pwm": (tl_box[1], 110), "steer": (tl_box[1] + 125, 30), "mk": (tl_box[1] + 165, 40)}

    # 정적 배경
    bg = np.full((OUT_H, OUT_W, 3), BG, np.uint8)
    for (x, y, w, h) in (map_box, obl_box):
        cv2.rectangle(bg, (x, y), (x + w, y + h), PANEL, -1)
    cv2.rectangle(bg, (tl_box[0], tl_box[1]), (tl_box[0] + tl_box[2], tl_box[1] + tl_box[3]), PANEL, -1)
    mv.grid(bg)
    # 타임라인 정적: PWM 계단, 조향, 마커 수
    py0, ph = lanes["pwm"]
    pmax = max([abs(v) for _, v in pwm] + [20])
    cv2.line(bg, (tl_box[0], py0 + ph // 2), (tl_box[0] + tl_box[2], py0 + ph // 2), GRID, 1)
    if pwm:
        pts = []
        for i, (t, v) in enumerate(pwm):
            y = int(py0 + ph / 2 - v / pmax * (ph / 2 - 6))
            if pts:
                pts.append((tx(t), pts[-1][1]))
            pts.append((tx(t), y))
        pts.append((tx(t_end), pts[-1][1]))
        cv2.polylines(bg, [np.array(pts, np.int32)], False, BLUE, 2, cv2.LINE_AA)
        put(bg, f"+{pmax}", (tl_box[0] + 4, py0 + 14), 0.4, INK2); put(bg, f"-{pmax}", (tl_box[0] + 4, py0 + ph - 4), 0.4, INK2)
    sy0, sh = lanes["steer"]
    for t, c in steer:
        cv2.line(bg, (tx(t), sy0 + 2), (tx(t), sy0 + sh - 2), ORANGE if c == "L" else AQUA, 2)
    my0, mh = lanes["mk"]
    nm = np.array([len(f["ids"]) for f in frames]); mmax = max(nm.max(), 1)
    for i in range(1, n):
        x0, x1 = tx(times[i - 1]), tx(times[i])
        h = int(nm[i] / mmax * (mh - 4))
        col = GREEN if nm[i] >= 2 else (ORANGE if nm[i] == 1 else RED)
        cv2.rectangle(bg, (x0, my0 + mh - h), (max(x1, x0 + 1), my0 + mh), col, -1)
    for t in range(int(np.ceil(t_beg)), int(t_end) + 1, 5 if t_end < 120 else 10):
        cv2.line(bg, (tx(t), tl_box[1] + tl_box[3]), (tx(t), tl_box[1] + tl_box[3] + 6), INK2, 1)
        put(bg, f"{t}s", (tx(t) - 10, tl_box[1] + tl_box[3] + 22), 0.45, INK2)
    # 3D 바닥 격자 — 패널 캔버스에 그려서 밖으로 넘치지 않게
    obl_bg = np.full((obl_box[3], obl_box[2], 3), PANEL, np.uint8)
    gl, gh = all3[:, :2].min(0) - 0.3, all3[:, :2].max(0) + 0.3
    floor = all3[:, 2].min() if not track else 0.0
    for u in np.arange(np.floor(gl[0]), gh[0] + 0.01, 0.5):
        a, b = opx([u, gl[1], floor]), opx([u, gh[1], floor]); cv2.line(obl_bg, tuple(a[0]), tuple(b[0]), GRID, 1)
    for v in np.arange(np.floor(gl[1]), gh[1] + 0.01, 0.5):
        a, b = opx([gl[0], v, floor]), opx([gh[0], v, floor]); cv2.line(obl_bg, tuple(a[0]), tuple(b[0]), GRID, 1)
    # 정적 마커(실측)
    if track:
        for k, g in track[2].items():
            p = mv.px(g[:2])[0]
            cv2.rectangle(bg, tuple(p - 8), tuple(p + 8), INK, 2)
            put(bg, f"{k}", (p[0] + 10, p[1] - 8), 0.5, INK)
    frame_lab = "트랙 좌표 (실측 마커 정합)" if track else "SLAM 좌표 (출발 카메라 기준 · 오른쪽/앞)"
    bg = pil_text(bg, [((40, 18), title, 30, INK, True),
                       ((map_box[0], map_box[1] - 34), f"위에서 본 지도 — {frame_lab}", 20, INK, True),
                       ((obl_box[0], obl_box[1] - 32), "3D 사선 뷰", 20, INK, True),
                       ((tl_box[0], tl_box[1] - 36), "주행 명령 타임라인", 20, INK, True),
                       ((40, lanes["pwm"][0] + 40), "PWM 명령", 18, INK, False),
                       ((40, lanes["steer"][0] + 2), "조향 L 주황 · R 청록", 16, INK2, False),
                       ((40, lanes["mk"][0] + 4), "보이는 마커 수", 16, INK2, False),
                       ((40, lanes["mk"][0] + 24), "초록 ≥2 · 주황 1 · 빨강 0", 13, INK2, False)])
    if track:
        r = track[3]
        txt = "정합 잔차: " + "  ".join(f"M{k} {v * 100:.1f}cm" for k, v in r.items())
        bg = pil_text(bg, [((map_box[0] + 8, map_box[1] + 6), txt, 15, INK2, False)])

    # 인코더
    os.makedirs(os.path.dirname(out_mp4), exist_ok=True)
    enc = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                            "-s", f"{OUT_W}x{OUT_H}", "-r", str(OUT_FPS), "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
                            "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", out_mp4], stdin=subprocess.PIPE)
    cap = cv2.VideoCapture(video)
    src_i, src_frame = -1, None
    hfov = 2 * np.arctan(src_size[0] / 2 / K[0, 0])
    out_n = int(np.ceil((times[-1] - times[0]) * OUT_FPS)) + 1
    t_render = time.time()
    for j in range(out_n):
        t = times[0] + j / OUT_FPS
        want = int(np.searchsorted(times, t, side="right") - 1)
        while src_i < want:
            ok, src_frame = cap.read(); src_i += 1
            if not ok:
                break
        f = frames[want]
        img = bg.copy()
        # 카메라 영상
        cam_img = cv2.resize(src_frame, (CAM_W, CAM_H))
        sx = CAM_W / src_size[0] * scale
        if f["det"] is not None:
            for i, (mid, c) in enumerate(zip(f["ids"], f["corners"])):
                q = (c * sx).astype(np.int32)
                cv2.polylines(cam_img, [q], True, (168, 245, 61), 3, cv2.LINE_AA)
                d = f["det"][i] if i < len(f["det"]) else None
                lab = f"ID {mid}" + (f"  {np.linalg.norm(d[:3]):.2f} m" if d is not None else "")
                p = q.max(0)
                cv2.rectangle(cam_img, (p[0] + 4, p[1] - 26), (p[0] + 14 + 13 * len(lab), p[1] + 4), (20, 20, 20), -1)
                put(cam_img, lab, (p[0] + 9, p[1] - 5), 0.6, (255, 255, 255), 2)
        cam = f["cam"]
        tt = t
        pw = 0
        for et, v in pwm:
            if et <= tt:
                pw = v
        hud = f"t {tt:6.2f}s   PWM {pw:+4d}   markers {len(f['ids'])}   pos {Pw[want, 0]:+.2f}, {Pw[want, 1]:+.2f} m   heading {head[want]:+5.0f} deg"
        cv2.rectangle(cam_img, (0, 0), (CAM_W, 40), (20, 20, 20), -1)
        put(cam_img, hud, (14, 27), 0.7, (255, 255, 255), 2)
        img[70:70 + CAM_H, 30:30 + CAM_W] = cam_img
        cv2.rectangle(img, (30, 70), (30 + CAM_W, 70 + CAM_H), INK2, 1)
        # 지도: 추정 마커, 궤적, 카메라
        for k, L in f["lm"].items():
            p = mv.px(to_world(L)[0, :2])[0]
            vis = k in f["ids"]
            cv2.circle(img, tuple(p), 7, ORANGE, -1 if vis else 2, cv2.LINE_AA)
            if not track:
                put(img, f"{k}", (p[0] + 9, p[1] - 8), 0.5, INK)
        trail = mv.px(Pw[:want + 1, :2])
        if len(trail) > 1:
            cv2.polylines(img, [trail], False, BLUE, 3, cv2.LINE_AA)
        c = mv.px(Pw[want, :2])[0]
        h = np.arctan2(fwd[want, 1], fwd[want, 0])
        L = 1.2 * mv.sc
        wedge = [c, c + (L * np.array([np.cos(h + hfov / 2), -np.sin(h + hfov / 2)])).astype(int),
                 c + (L * np.array([np.cos(h - hfov / 2), -np.sin(h - hfov / 2)])).astype(int)]
        ov = img.copy(); cv2.fillPoly(ov, [np.array(wedge)], (240, 215, 180)); img = cv2.addWeighted(ov, 0.45, img, 0.55, 0)
        tip = (c + 18 * np.array([np.cos(h), -np.sin(h)])).astype(int)
        cv2.circle(img, tuple(c), 8, BLUE, -1, cv2.LINE_AA); cv2.line(img, tuple(c), tuple(tip), INK, 3, cv2.LINE_AA)
        # 3D 사선 뷰 (패널 캔버스)
        ob = obl_bg.copy()
        tr3 = opx(Pw[:want + 1])
        if len(tr3) > 1:
            cv2.polylines(ob, [tr3], False, BLUE, 2, cv2.LINE_AA)
        for k, Lm in f["lm"].items():
            p = opx(to_world(Lm)[0])[0]; base = opx(np.r_[to_world(Lm)[0][:2], floor])[0]
            cv2.line(ob, tuple(base), tuple(p), (180, 184, 186), 1); cv2.circle(ob, tuple(p), 5, ORANGE, -1, cv2.LINE_AA)
            put(ob, f"{k}", (p[0] + 6, p[1] - 6), 0.45, INK2)
        cp = opx(Pw[want])[0]; cb = opx(np.r_[Pw[want, :2], floor])[0]
        cv2.line(ob, tuple(cb), tuple(cp), INK2, 1); cv2.circle(ob, tuple(cp), 7, BLUE, -1, cv2.LINE_AA)
        put(ob, f"height {Pw[want, 2]:+.2f} m", (8, obl_box[3] - 10), 0.5, INK2)
        img[obl_box[1]:obl_box[1] + obl_box[3], obl_box[0]:obl_box[0] + obl_box[2]] = ob
        # 타임라인 커서
        x = tx(t); cv2.line(img, (x, tl_box[1] - 4), (x, tl_box[1] + tl_box[3] + 4), RED, 2)
        enc.stdin.write(img.tobytes())
        if j % 300 == 0 and j:
            print(f"  렌더 {j}/{out_n} ({j / (time.time() - t_render):.1f} fps)", flush=True)
    enc.stdin.close(); enc.wait()
    return out_n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", help="주행 세션 폴더 (drive_log.csv · session.json [· video.mkv])")
    ap.add_argument("--live", help="live_slam.py 출력 폴더 (video_live.avi · trajectory_live.csv)")
    ap.add_argument("--landmarks", help="실측 마커 CSV (id,x_m,y_m,z_m)")
    ap.add_argument("--out")
    a = ap.parse_args()
    if not a.session and not a.live:
        sys.exit("--session 또는 --live 필요")
    sess = os.path.abspath(os.path.expanduser(a.session)) if a.session else None
    if a.live:
        live = os.path.abspath(os.path.expanduser(a.live))
        video = os.path.join(live, "video_live.avi")
        ep = np.loadtxt(os.path.join(live, "trajectory_live.csv"), delimiter=",", skiprows=1, usecols=1, ndmin=1)
        t0 = json.load(open(os.path.join(sess, "session.json")))["t0_epoch"] if sess else ep[0]
        times = ep - t0
        name = os.path.basename(sess) if sess else os.path.basename(live)
    else:
        video = os.path.join(sess, "video.mkv")
        _, times = session_times(sess)
        name = os.path.basename(sess)
    if not os.path.exists(video):
        sys.exit(f"영상 없음: {video}")
    out = os.path.abspath(os.path.expanduser(a.out)) if a.out else os.path.join(sess or live, "render", "localization.mp4")
    lmk = os.path.abspath(os.path.expanduser(a.landmarks)) if a.landmarks else None
    print(f"[render] 1/2 SLAM: {video}")
    frames, times, tr, size, scale = run_slam(video, times)
    print(f"[render] 2/2 영상: {len(frames)} 프레임 → {out}")
    title = f"수동 주행 + ArUco 로컬라이제이션 · {name}"
    n = render(frames, times, tr, size, scale, video, out, sess, lmk, title)
    det = sum(1 for f in frames if f["ids"])
    summ = {"video_in": video, "out": out, "frames_in": len(frames), "frames_out": n, "duration_s": round(float(times[-1] - times[0]), 2),
            "frames_with_marker": det, "landmarks": sorted(frames[-1]["lm"]), "track_aligned": bool(lmk)}
    json.dump(summ, open(out.replace(".mp4", ".json"), "w"), ensure_ascii=False, indent=1)
    print(f"[render] 완료: {out}  ({n}프레임, 마커 검출 {det}/{len(frames)})")


if __name__ == "__main__":
    main()
