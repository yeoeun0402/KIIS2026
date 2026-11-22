#!/usr/bin/env python3
"""[demo] 매핑 주행 1회 → 마커 지도 (트랙 좌표) 자동 생성.

  python3 -B build_map.py --session ~/demo/sessions/<세션> [--out DIR] [--marker-size <m, 기본 track.yaml>]
         [--align auto|start|lane] [--fix-mount] [--stride 2]

주행 규칙: 출발선(10 cm 선 차 쪽 가장자리)에 앞범퍼, 2차선 중앙에 놓고 2~3초 정지 → 2차선을 따라 천천히 한 바퀴.

처리
  1. 영상 전체 ArUco 검출 (병렬, out/detections.pkl 에 캐시)
  2. 순차 초기화: 첫 프레임 = track.yaml 출발 자세, 이미 아는 마커로 PnP → 새 마커 등록
  3. 번들 조정 (scipy least_squares, Huber): 코너 재투영 오차 최소화
       차 자세 = 바닥 위 SE(2) (x, y, θ)  ← 평평한 바닥 · 장착 높이 고정 (mount.yaml)
       마커 = 6자유도 (위치 + 방향)        카메라 pitch·roll·yaw = 전체 공통 미지수 (--fix-mount 면 고정)
       뒷축은 옆으로 미끄러지지 않음 (bicycle) — 마커가 1개만 보일 때 방향을 잡아 주고, 장착 yaw 를 관측 가능하게 함
       첫 프레임 자세 = 출발 자세 (강한 prior) → 지도가 트랙 좌표에 붙는다
  4. 2차선 중심선과 궤적 비교 (ICP) → 출발 자세 기준과 차선 기준의 차이 보고 (대략 1°·15 cm 수준의 점검용)
       --align start (기본): 출발 자세 기준 · lane: 차선 기준으로 지도 전체 이동 · auto: 차선 70%+ 돌았으면 lane
  5. 이어보기 끊김 점검: 앞에서 보던 마커와 겹치지 않는 새 마커만 보이는 순간 (엔코더 없음 → 그 구간 거리 미관측)
  6. 출력  markers.csv · map.yaml · trajectory.csv · map_overview.png · report.json
"""
import argparse
import csv
import json
import os
import pickle
import sys
import time
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trackmap as tm  # noqa: E402
from trackmap import Rotation, cv2  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402
from scipy.optimize._numdiff import approx_derivative, group_columns  # noqa: E402
from scipy.sparse import diags  # noqa: E402
from scipy.sparse.linalg import spsolve  # noqa: E402
from scipy.sparse import lil_matrix  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

MAX_INIT_DIST = 6.0       # 이보다 먼 첫 관측으로는 마커를 등록하지 않음 (더 가까이 볼 때까지 대기)
REJECT_PX = 4.0           # 최종 BA 전 이상치 관측 기준 (코너 RMS)
MAX_SPEED = 1.5           # 초기화 예측용 상한 (m/s)
MAX_YAWRATE = 1.0         # (rad/s)
INIT_BA_EVERY = int(os.environ.get("INIT_BA_EVERY", 20))       # 초기화 중 BA 주기 (키프레임)
INIT_BA_WINDOW = 200      # 초기화 중 BA 는 최근 키프레임만 움직임 (그 이전은 고정)


# ── 1. 검출 ──────────────────────────────────────────────────────
_DET = None


def _det(gray):
    global _DET
    if _DET is None:
        _DET = tm.make_detector()
    corners, ids, _ = _DET.detectMarkers(gray)
    if ids is None:
        return np.zeros(0, int), np.zeros((0, 4, 2))
    return ids.ravel().astype(int), np.array([c[0] for c in corners], float)


def _frames(video):
    cap = cv2.VideoCapture(video)
    while True:
        ok, fr = cap.read()
        if not ok:
            return
        yield cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)


def detect_video(video, cache, workers=8):
    if os.path.exists(cache):
        return pickle.load(open(cache, "rb"))
    cap = cv2.VideoCapture(video); W = int(cap.get(3)); n = int(cap.get(7)); cap.release()
    t0 = time.time(); out = []
    with Pool(workers) as pool:
        for i, r in enumerate(pool.imap(_det, _frames(video), chunksize=4)):
            out.append(r)
            if i % 300 == 0:
                print(f"  검출 {i}/{n} ({(i + 1) / (time.time() - t0):.1f} fps)", flush=True)
    det = {"width": W, "frames": out}
    pickle.dump(det, open(cache, "wb"))
    return det


def session_info(sess, n):
    p = os.path.join(sess, "video_frames.csv")
    times = None
    if os.path.exists(p):
        rows = list(csv.reader(open(p)))[1:]
        times = np.array([float(r[1]) for r in rows]) if rows else None
    if times is None or len(times) < n:
        times = np.arange(n) / 30.0 if times is None else np.r_[times, times[-1] + np.arange(1, n - len(times) + 1) / 30]
    t_move = None
    p = os.path.join(sess, "drive_log.csv")
    if os.path.exists(p):
        ts = [float(r["t_s"]) for r in csv.DictReader(open(p))
              if r["kind"] == "tx" and r["value"].startswith("V") and r["value"] != "V0"]
        t_move = min(ts) if ts else None
    return times[:n], t_move


def huber_w(r, n_robust, k=1.5):
    """앞 n_robust 개 (픽셀 잔차) 만 Huber, 나머지 (prior·제약) 는 가중치 1."""
    w = np.ones(len(r))
    a = np.abs(r[:n_robust])
    w[:n_robust] = np.where(a <= k, 1.0, k / np.maximum(a, 1e-12))
    return w


def huber_cost(r, n_robust, k=1.5):
    a = np.abs(r[:n_robust])
    return float(np.sum(np.where(a <= k, 0.5 * a * a, k * a - 0.5 * k * k)) + 0.5 * np.sum(r[n_robust:] ** 2))


# ── 3. 번들 조정 ─────────────────────────────────────────────────
NP = 4                    # 미지수: 장착 pitch_down · roll · yaw (deg) + 초점거리 배율 f (fx·fy 에 곱함)
FOCAL_PRIOR = float(os.environ.get("FOCAL_PRIOR", 0.5))       # f 배율 prior σ (사실상 자유)
SLIP_SIGMA = float(os.environ.get("SLIP_SIGMA", 0.01))        # 키프레임 사이 옆미끄러짐 허용 (m) + 간격 1초당 SLIP_PER_S
SLIP_PER_S = 0.02
MOUNT_PRIOR_DEG = 3.0     # 장착 각을 mount.yaml 값 근처로 약하게 묶음 (초기 관측이 약할 때 안정용)
ACC_SIGMA = 1.0           # 가속도 변화 허용 (m/s²) — 관측이 약한 키프레임을 앞뒤 사이로 매끄럽게 묶음
YAWACC_SIGMA = 1.5        # 각가속도 (rad/s²)


def Kf(K, pr):
    """캘리브레이션 K 에 추정한 초점거리 배율 pr[3] 적용 (주점은 그대로)."""
    K2 = K.copy(); K2[0, 0] *= pr[3]; K2[1, 1] *= pr[3]
    return K2


def prior_sigma():
    return np.array([MOUNT_PRIOR_DEG] * 3 + [FOCAL_PRIOR])


class Problem:
    def __init__(self, K, D, size, mount, start, obs, n_cam, n_mk, kf_t):
        self.K, self.D, self.obj = K, D, tm.marker_obj(size)
        self.mount, self.start = mount, start
        self.pr0 = np.array([mount["pitch_down_deg"], mount["roll_deg"], mount["yaw_deg"], 1.0], float)
        self.ci = np.array([o[0] for o in obs], int)
        self.mj = np.array([o[1] for o in obs], int)
        self.uv = np.array([o[2] for o in obs], float)
        self.N, self.M = n_cam, n_mk
        dt = np.diff(np.asarray(kf_t[:n_cam], float))
        self.slip_w = 1.0 / (SLIP_SIGMA + SLIP_PER_S * np.abs(dt))
        self.dt = np.maximum(dt, 1e-3)

    def unpack(self, x):
        pr = x[:NP]
        cams = x[NP:NP + 3 * self.N].reshape(self.N, 3)
        mk = x[NP + 3 * self.N:].reshape(self.M, 6)
        return pr, cams, mk

    def predict(self, pr, cams, mk):
        R_bo, t_bo = tm.T_base_opt(self.mount, *pr[:3])
        R_wc, t_wc = tm.cam_from_base(cams, R_bo, t_bo)
        R_m = Rotation.from_rotvec(mk[:, :3]).as_matrix()
        Cw = mk[:, None, 3:] + np.einsum("mij,cj->mci", R_m, self.obj)
        d = Cw[self.mj] - t_wc[self.ci][:, None, :]
        Xc = np.einsum("kji,kcj->kci", R_wc[self.ci], d)
        Xc[..., 2] = np.maximum(Xc[..., 2], 0.05)
        return tm.project(Xc, Kf(self.K, pr), self.D)

    def residual(self, x):
        pr, cams, mk = self.unpack(x)
        r = (self.predict(pr, cams, mk) - self.uv).ravel()
        e = cams[0] - self.start
        e[2] = tm.wrap(e[2])
        slip = tm.lateral_slip(cams[:-1], cams[1:]) * self.slip_w
        return np.r_[r, e / np.array([0.002, 0.002, np.radians(0.1)]), slip, self.smooth(cams),
                     (pr - self.pr0) / prior_sigma()]

    def smooth(self, cams):
        if self.N < 3:
            return np.zeros(0)
        d = np.diff(cams, axis=0); d[:, 2] = tm.wrap(d[:, 2])
        v = d / self.dt[:, None]
        dv = v[1:] - v[:-1]
        tm_ = 0.5 * (self.dt[1:] + self.dt[:-1])
        sx = ACC_SIGMA * tm_ + 0.01; sth = YAWACC_SIGMA * tm_ + 0.01
        return np.c_[dv[:, 0] / sx, dv[:, 1] / sx, dv[:, 2] / sth].ravel()

    def sparsity(self, fix_mount):
        nobs = len(self.ci); n = NP + 3 * self.N + 6 * self.M
        ns = 3 * max(self.N - 2, 0)
        A = lil_matrix((nobs * 8 + 3 + self.N - 1 + ns + NP, n), dtype=int)
        for k in range(nobs):
            rows = slice(8 * k, 8 * k + 8)
            A[rows, 0:NP] = 1
            c = NP + 3 * self.ci[k]; A[rows, c:c + 3] = 1
            m = NP + 3 * self.N + 6 * self.mj[k]; A[rows, m:m + 6] = 1
        A[nobs * 8:nobs * 8 + 3, NP:NP + 3] = 1
        for i in range(self.N - 1):
            A[nobs * 8 + 3 + i, NP + 3 * i:NP + 3 * i + 6] = 1
        base = nobs * 8 + 3 + self.N - 1
        for i in range(self.N - 2):
            A[base + 3 * i:base + 3 * i + 3, NP + 3 * i:NP + 3 * i + 9] = 1
        for i in range(NP):
            A[base + ns + i, i] = 1
        return A

    def solve(self, x0, fix=(True, True), max_nfev=50, verbose=0, n_fixed_cams=0):
        """희소 Levenberg–Marquardt + Huber (IRLS). max_nfev = 반복 횟수 상한."""
        fixed = np.zeros(len(x0), bool)
        fixed[:3] = fix[0]                         # 장착 각
        fixed[3] = fix[1]                          # 초점거리 배율
        fixed[NP:NP + 3 * n_fixed_cams] = True
        free = np.where(~fixed)[0]
        A = self.sparsity(fix[0]).tocsc()[:, free]
        groups = group_columns(A)
        x = x0.copy()

        def f_free(z):
            xx = x.copy(); xx[free] = z
            return self.residual(xx)
        r = self.residual(x)
        nr = len(self.ci) * 8                      # 앞쪽 = 픽셀 잔차 (Huber), 뒤쪽 = 출발 prior·미끄럼 제약 (그대로)
        cost = huber_cost(r, nr)
        lam = 1e-3
        for it in range(max_nfev):
            J = approx_derivative(f_free, x[free], method="2-point", sparsity=(A, groups), f0=r).tocsr()
            w = huber_w(r, nr)
            JW = J.multiply(w[:, None]).tocsr()
            H = (J.T @ JW).tocsc(); g = J.T @ (w * r)
            dH = H.diagonal()
            while True:
                M = H + diags(lam * np.maximum(dH, 1e-9))
                try:
                    dz = -spsolve(M, g)
                except Exception:
                    dz = np.full(len(free), np.nan)
                xn = x.copy(); xn[free] += dz
                rn = self.residual(xn) if np.all(np.isfinite(dz)) else None
                cn = huber_cost(rn, nr) if rn is not None else np.inf
                if cn < cost:
                    lam = max(lam / 3, 1e-7); break
                lam *= 5
                if lam > 1e8:
                    break
            if lam > 1e8:
                break
            rel = (cost - cn) / max(cost, 1e-12)
            x, r, cost = xn, rn, cn
            if verbose:
                print(f"    LM {it}: cost {cost:.1f} λ {lam:.1e}")
            if rel < 1e-6:
                break
        return x, None

    def per_obs_rms(self, x):
        pr, cams, mk = self.unpack(x)
        e = self.predict(pr, cams, mk) - self.uv
        return np.sqrt((e ** 2).sum(-1).mean(-1))


def run_ba(K, D, size, mount, start, kf_obs, cams, markers, ids, pr, fix, max_nfev, kf_t, drop=None, window=None):
    """kf_obs: [(cam_idx, marker_id, uv4)], markers: {id: (rotvec, p)} → 갱신값."""
    idx = {k: i for i, k in enumerate(ids)}
    obs = [(c, idx[m], uv) for c, m, uv in kf_obs if m in idx]
    if drop is not None:
        obs = [o for o, d in zip(obs, drop) if not d]
    P = Problem(K, D, size, mount, start, obs, len(cams), len(ids), kf_t)
    x0 = np.r_[pr, np.asarray(cams).ravel(), np.concatenate([np.r_[markers[k][0], markers[k][1]] for k in ids])]
    x, res = P.solve(x0, fix=fix, max_nfev=max_nfev,
                     n_fixed_cams=0 if window is None else max(0, len(cams) - window))
    pr2, cams2, mk = P.unpack(x)
    return pr2.copy(), cams2.copy(), {k: (mk[i, :3].copy(), mk[i, 3:].copy()) for i, k in enumerate(ids)}, P, x


# ── 2. 순차 초기화 ────────────────────────────────────────────────
def pose_se2(pred, prev, dt, uv_list, world_list, K, D, R_bo, t_bo):
    """아는 마커 코너 + 운동 제약으로 차 자세 (x, y, θ) 만 푼다 → (pose, 재투영 rms_px).
    제약: 예측 자세 근처 (약하게) · 뒷축은 옆으로 미끄러지지 않음 (bicycle)."""
    W = np.concatenate(world_list); U = np.concatenate(uv_list)
    w_slip = 1.0 / (SLIP_SIGMA + SLIP_PER_S * abs(dt))

    def f(x):
        R_wc, t_wc = tm.cam_from_base(x, R_bo, t_bo)
        Xc = (W - t_wc[0]) @ R_wc[0]
        Xc[:, 2] = np.maximum(Xc[:, 2], 0.05)
        e = x - pred; e[2] = tm.wrap(e[2])
        return np.r_[(tm.project(Xc, K, D) - U).ravel(), e / np.array([0.3, 0.3, np.radians(10)]),
                     tm.lateral_slip(prev, x) * w_slip]
    r = least_squares(f, pred, loss="huber", f_scale=2.0, x_scale=[0.1, 0.1, 0.02], max_nfev=50)
    return r.x, float(np.sqrt(np.mean(r.fun[:len(U) * 2] ** 2)))


def marker_world(R_wc, t_wc, R_cm, t_cm):
    return Rotation.from_matrix(R_wc @ R_cm).as_rotvec(), R_wc @ t_cm + t_wc


def reinit_marker(k, kf_obs, cams, pr, mount, size, K, D, obj):
    """자세가 잡힌 키프레임들에서 본 모든 관측으로 마커 하나를 다시 추정: 위치 = 중앙값, 방향 = 가장 잘 맞는 IPPE 해."""
    R_bo, t_bo = tm.T_base_opt(mount, *pr[:3])
    K = Kf(K, pr)
    Rw, tw = tm.cam_from_base(np.asarray(cams), R_bo, t_bo)
    ob = [o for o in kf_obs if o[1] == k]
    cands = []
    for c, _, uv in ob:
        for R_cm, t_cm, _ in tm.marker_poses_ippe(uv, size, K, D)[:2]:
            cands.append((Rotation.from_matrix(Rw[c] @ R_cm).as_rotvec(), Rw[c] @ t_cm + tw[c]))
    p = np.median(np.array([cd[1] for cd in cands]), axis=0)
    best, best_e = None, np.inf
    sub = ob[:: max(1, len(ob) // 40)]
    for rv, _ in cands[:: max(1, len(cands) // 60)]:
        Cw = corners_world(rv, p, obj).T
        e = np.median([np.sqrt(((tm.project((Cw - tw[c]) @ Rw[c], K, D) - uv) ** 2).sum(-1).mean()) for c, _, uv in sub])
        if e < best_e:
            best, best_e = rv, e
    return best, p


def corners_world(rv, p, obj):
    return Rotation.from_rotvec(rv).as_matrix() @ obj.T + p[:, None]


# ── 4. 차선 정합 ─────────────────────────────────────────────────
def resample_path(xy, step=0.05):
    """키프레임 간격이 고르지 않으므로 경로 길이 기준으로 고르게 다시 뽑는다."""
    seg = np.hypot(*np.diff(xy, axis=0).T); u = np.r_[0, np.cumsum(seg)]
    if u[-1] < step:
        return xy
    q = np.arange(0, u[-1], step)
    return np.c_[np.interp(q, u, xy[:, 0]), np.interp(q, u, xy[:, 1])]


def lane_icp(xy, cl, iters=60, keep=0.9):
    xy = resample_path(xy)
    s = cl[:, 0]; L = s[-1] + (s[1] - s[0])
    ss = np.arange(0, L, 0.01)
    P = np.c_[np.interp(ss, s, cl[:, 1], period=L), np.interp(ss, s, cl[:, 2], period=L)]
    tree = cKDTree(P)
    R = np.eye(2); t = np.zeros(2)
    for _ in range(iters):
        q = xy @ R.T + t
        d, j = tree.query(q)
        m = d <= np.quantile(d, keep)
        a, b = q[m], P[j[m]]
        ca, cb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
        dR = Vt.T @ np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        R, t = dR @ R, dR @ (t - ca) + cb
    q = xy @ R.T + t
    d, j = tree.query(q)
    # 부호 있는 횡오차 (차선 진행 방향 왼쪽 +)
    tg = np.roll(P, -1, 0) - np.roll(P, 1, 0); tg /= np.linalg.norm(tg, axis=1, keepdims=True)
    nrm = np.c_[-tg[j, 1], tg[j, 0]]
    lat = ((q - P[j]) * nrm).sum(1)
    cov = len(np.unique((ss[j[d < 0.5]] // 1.0).astype(int))) / int(np.ceil(L))
    return R, t, lat, cov


# ── 5. 그림 ──────────────────────────────────────────────────────
DRAW = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drawing_ccw.png")
X0, YB, XR, YT = 338.5, 2205.0, 3234.0, 255.0       # drawing_ccw.png 의 매트 외곽 (픽셀)


def overview(path, cl, traj, markers, start, title):
    im = cv2.imread(DRAW) if os.path.exists(DRAW) else np.full((2484, 3509, 3), 255, np.uint8)
    im = (im * 0.55 + 255 * 0.45).astype(np.uint8)
    sx = (XR - X0) / 16.0; sy = (YB - YT) / 11.0

    def px(p):
        p = np.atleast_2d(p)
        return np.c_[X0 + p[:, 0] * sx, YB - p[:, 1] * sy].astype(np.int32)
    cv2.polylines(im, [px(cl[:, 1:3])], True, (0, 0, 220), 2, cv2.LINE_AA)
    cv2.polylines(im, [px(traj[:, :2])], False, (0, 150, 0), 3, cv2.LINE_AA)
    for k, (rv, p) in markers.items():
        R = Rotation.from_rotvec(rv).as_matrix(); n = R[:, 2]
        a = px(p[:2])[0]; b = px(p[:2] + 0.5 * n[:2] / max(np.linalg.norm(n[:2]), 1e-6))[0]
        cv2.arrowedLine(im, tuple(map(int, a)), tuple(map(int, b)), (200, 80, 0), 3, tipLength=0.3)
        cv2.circle(im, tuple(map(int, a)), 12, (200, 80, 0), -1)
        cv2.putText(im, str(k), (int(a[0]) + 14, int(a[1]) - 14), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (120, 40, 0), 3)
    s = px(start[:2])[0]; e = px(start[:2] + 0.6 * np.array([np.cos(start[2]), np.sin(start[2])]))[0]
    cv2.arrowedLine(im, tuple(map(int, s)), tuple(map(int, e)), (200, 0, 200), 5, tipLength=0.3)
    cv2.putText(im, title, (60, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 0), 3)
    cv2.putText(im, "red: lane-2 centerline  green: driven (base_link)  blue: markers (arrow = facing)  magenta: start",
                (60, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (60, 60, 60), 2)
    cv2.imwrite(path, cv2.resize(im, (im.shape[1] // 2, im.shape[0] // 2), interpolation=cv2.INTER_AREA))


# ── 본체 ─────────────────────────────────────────────────────────
def build(frames_ids, frames_uv, times, t_move, K, D, size, mount, start, cl, stride=2, fix_mount=False,
          align="auto", log=print, truth=None, fix_focal=False, focal0=1.0):
    # 키프레임: 마커 1개 이상 보이는 프레임. 출발 정지 구간은 첫 프레임 + 몇 장, 주행 중은 stride 간격
    has = [i for i in range(len(frames_ids)) if len(frames_ids[i])]
    if not has:
        raise SystemExit("마커가 한 번도 검출되지 않음")
    if has[0] > 0 and (t_move is None or times[has[0]] > t_move):
        log(f"  [경고] 출발 정지 중에 마커가 안 보임 (첫 검출 {times[has[0]]:.1f}s) — 출발 자세 기준이 약해짐")
    still = [i for i in has if t_move is not None and times[i] < t_move]
    moving = [i for i in has if i not in set(still)]
    kfs = still[:: max(1, len(still) // 10)] if still else []
    if not kfs:
        kfs = [has[0]]
    kfs += [i for i in moving[::stride] if i > kfs[-1]]
    log(f"  키프레임 {len(kfs)} (정지 {sum(1 for i in kfs if i in set(still))}) / 검출 프레임 {len(has)} / 전체 {len(frames_ids)}")

    pr = np.array([mount["pitch_down_deg"], mount["roll_deg"], mount["yaw_deg"], focal0], float)
    fix_i = (fix_mount, True)                # 순차 초기화 중엔 초점거리 고정 (관측이 적어 흔들림)
    fix = (fix_mount, fix_focal)
    K0 = K; K = Kf(K0, pr)
    R_bo, t_bo = tm.T_base_opt(mount, *pr[:3])
    obj = tm.marker_obj(size)
    markers, cams, kf_obs = {}, [], []
    pose = start.copy(); vel = np.zeros(3); last_t = times[kfs[0]]
    for n, fi in enumerate(kfs):
        ids, uvs = frames_ids[fi], frames_uv[fi]
        dt = times[fi] - last_t; last_t = times[fi]
        pred = pose + vel * dt
        known = [(k, uv) for k, uv in zip(ids, uvs) if k in markers]
        if n > 0 and known:
            cand, e = pose_se2(pred, pose, dt, [uv for _, uv in known], [corners_world(*markers[k], obj).T for k, _ in known],
                               K, D, R_bo, t_bo)
            good = np.all(np.isfinite(cand)) and e < 5.0 and np.hypot(*(cand[:2] - pred[:2])) < 0.5
            new_pose = cand if good else pred
        else:
            new_pose = pred if n > 0 else start.copy()
        if n > 0 and dt > 0:
            v_new = (new_pose - pose) / dt
            v_new[2] = ((new_pose[2] - pose[2] + np.pi) % (2 * np.pi) - np.pi) / dt
            vel = 0.7 * vel + 0.3 * v_new if known else vel
            sp = np.hypot(*vel[:2])
            if sp > MAX_SPEED:
                vel[:2] *= MAX_SPEED / sp
            vel[2] = np.clip(vel[2], -MAX_YAWRATE, MAX_YAWRATE)
        if t_move is not None and times[fi] < t_move:
            new_pose, vel = start.copy(), np.zeros(3)
        pose = new_pose
        R_wc, t_wc = tm.cam_from_base(pose, R_bo, t_bo); R_wc, t_wc = R_wc[0], t_wc[0]
        cams.append(pose.copy())
        if truth is not None and n % 30 == 0:
            e = truth[fi] - pose
            log(f"    dbg kf {n} t={times[fi]:.1f} known={len(known)} vis={len(ids)} err xy {np.hypot(*e[:2])*100:.0f}cm θ {np.degrees(e[2]):.1f}° vel {vel.round(2)}")
        for k, uv in zip(ids, uvs):
            kf_obs.append((n, int(k), uv))
            if k not in markers:
                sols = tm.marker_poses_ippe(uv, size, K, D)
                if np.linalg.norm(sols[0][1]) <= MAX_INIT_DIST:
                    markers[int(k)] = marker_world(R_wc, t_wc, sols[0][0], sols[0][1])
        if n and n % INIT_BA_EVERY == 0 and markers:
            ids_k = sorted(markers)
            pr, c2, markers, _, _ = run_ba(K0, D, size, mount, start, kf_obs, cams, markers, ids_k, pr, fix_i, 15,
                                           times[kfs], window=INIT_BA_WINDOW)
            cams = [c for c in c2]; pose = cams[-1].copy(); R_bo, t_bo = tm.T_base_opt(mount, *pr[:3])
            if n % 150 == 0:
                log(f"  초기화 {n}/{len(kfs)} 키프레임 · 마커 {len(markers)}")

    # 먼 곳에서만 본 마커: 가장 가까운 관측으로 등록
    for k in sorted({o[1] for o in kf_obs} - set(markers)):
        best = min((o for o in kf_obs if o[1] == k), key=lambda o: -np.ptp(o[2][:, 0]))
        R_wc, t_wc = tm.cam_from_base(cams[best[0]], R_bo, t_bo)
        s = tm.marker_poses_ippe(best[2], size, K, D)[0]
        markers[k] = marker_world(R_wc[0], t_wc[0], s[0], s[1])
    ids_k = sorted(markers)
    pr, cams, markers, _, _ = run_ba(K0, D, size, mount, start, kf_obs, cams, markers, ids_k, pr, fix, 60, times[kfs])

    # 마커 방향 다중 가설 (IPPE 두 해 × 여러 관측) — 뒤집힌 초기값 방지
    R_bo, t_bo = tm.T_base_opt(mount, *pr[:3]); K = Kf(K0, pr)
    Rw, tw = tm.cam_from_base(cams, R_bo, t_bo)
    for k in ids_k:
        ob = [o for o in kf_obs if o[1] == k]
        sample = sorted(ob, key=lambda o: -np.ptp(o[2][:, 0]))[:25]
        best, best_e = markers[k][0], None
        for o in sample:
            for R_cm, t_cm, _ in tm.marker_poses_ippe(o[2], size, K, D):
                rv = Rotation.from_matrix(Rw[o[0]] @ R_cm).as_rotvec()
                Cw = corners_world(rv, markers[k][1], obj).T
                e = [np.sqrt(((tm.project(((Cw - tw[c]) @ Rw[c]), K, D) - uv) ** 2).sum(-1).mean()) for c, _, uv in ob[::3]]
                e = np.median(e)
                if best_e is None or e < best_e:
                    best, best_e = rv, e
        markers[k] = (best, markers[k][1])

    # 최종: 장착 각 풀고 → 어긋난 마커 다시 잡기 → 이상치 제거 → 다시
    pr, cams, markers, P, x = run_ba(K0, D, size, mount, start, kf_obs, cams, markers, ids_k, pr, fix, 200, times[kfs])
    rms = P.per_obs_rms(x)
    bad = [k for j, k in enumerate(ids_k) if (P.mj == j).any() and np.median(rms[P.mj == j]) > REJECT_PX]
    if bad:
        log(f"  어긋난 마커 {bad} → 전체 관측으로 다시 초기화")
        for k in bad:
            m = reinit_marker(k, kf_obs, cams, pr, mount, size, K0, D, obj)
            if m[0] is None or not np.all(np.isfinite(m[1])):   # 다시 잡을 해가 없으면 (관측·IPPE 해 없음) 원래 값 유지
                log(f"  마커 {k}: 다시 초기화 실패 → 원래 추정 유지")
                continue
            markers[k] = m
        pr, cams, markers, P, x = run_ba(K0, D, size, mount, start, kf_obs, cams, markers, ids_k, pr, fix, 200, times[kfs])
        rms = P.per_obs_rms(x)
    drop = rms > max(REJECT_PX, 4 * np.median(rms))
    if drop.any():
        log(f"  이상치 관측 {drop.sum()}/{len(rms)} 제거 (> {max(REJECT_PX, 4 * np.median(rms)):.1f}px) 후 재조정")
        pr, cams, markers, P, x = run_ba(K0, D, size, mount, start, kf_obs, cams, markers, ids_k, pr, fix, 200, times[kfs], drop)
        rms = P.per_obs_rms(x)
    cams = np.asarray(cams)

    # 차선 정합
    mv = np.array([kfs[i] not in set(still) for i in range(len(kfs))])
    R2, t2, lat, cov = lane_icp(cams[mv, :2] if mv.sum() > 20 else cams[:, :2], cl)
    dth = np.arctan2(R2[1, 0], R2[0, 0])
    st_lane = np.r_[R2 @ cams[0, :2] + t2, cams[0, 2] + dth]
    use_lane = align == "lane" or (align == "auto" and cov >= 0.7)
    info = {"lane_coverage": round(float(cov), 3),
            "lane_fit_shift_m": [round(float(v), 4) for v in st_lane[:2] - cams[0, :2]],
            "lane_fit_rot_deg": round(float(np.degrees(dth)), 3),
            "lateral_dev_m": {"mean": round(float(lat.mean()), 3), "std": round(float(lat.std()), 3),
                              "p95_abs": round(float(np.percentile(np.abs(lat), 95)), 3)},
            "aligned_to": "lane" if use_lane else "start"}
    if use_lane:
        Rz3 = tm.Rz(dth); t3 = np.r_[t2, 0]
        cams = np.c_[cams[:, :2] @ R2.T + t2, cams[:, 2] + dth]
        markers = {k: (Rotation.from_matrix(Rz3 @ Rotation.from_rotvec(rv).as_matrix()).as_rotvec(), Rz3 @ p + t3)
                   for k, (rv, p) in markers.items()}
    # 마커별 통계
    stats = {}
    obs_k = P.mj
    for j, k in enumerate(ids_k):
        m = obs_k == j
        dists = []
        R_bo, t_bo = tm.T_base_opt(mount, *pr[:3])
        for c in P.ci[m][::5]:
            Rw, tw = tm.cam_from_base(cams[c], R_bo, t_bo)
            dists.append(np.linalg.norm(markers[k][1] - tw[0]))
        stats[k] = {"n_obs": int(m.sum()), "rms_px": float(np.sqrt((rms[m] ** 2).mean())) if m.any() else None,
                    "min_dist_m": float(min(dists)) if dists else None}
    gaps = covis_gaps(kfs, frames_ids, times, cams, cl, t_from=t_move)
    rms_all = float(np.sqrt((rms ** 2).mean()))
    warn = []
    if rms_all > 3.0:
        verdict = "FAIL"; warn.append(f"재투영 RMS {rms_all:.1f}px > 3 — 지도 신뢰 불가")
    else:
        verdict = "OK"
        if rms_all > 1.5:
            warn.append(f"재투영 RMS {rms_all:.1f}px 다소 큼")
        if gaps:
            warn.append(f"이어보기 끊김 {len(gaps)}곳 — 그 구간은 지도가 앞뒤로 밀렸을 수 있음 (마커 추가 권장)")
        if abs(info["lane_fit_rot_deg"]) > 3 or np.hypot(*info["lane_fit_shift_m"]) > 0.4:
            warn.append("차선 기준과 출발 자세 기준이 크게 다름 — 출발 위치·방향을 확인하거나 --align lane")
        if cov < 0.7:
            warn.append(f"차선을 {cov * 100:.0f}% 만 돎 — 한 바퀴 전체를 돌아야 지도가 완성됨")
        if warn:
            verdict = "WARN"
    info.update(verdict=verdict, warnings=warn, covis_gaps=gaps)
    for w in warn:
        log(f"  [{verdict}] {w}")
    return dict(kfs=kfs, cams=cams, markers=markers, pr=pr, rms=rms, stats=stats, info=info, still=still)


def covis_gaps(kfs, frames_ids, times, cams, cl, window=1.0, t_from=None):
    """순서대로 볼 때, 직전 window 초 안에 본 마커와 하나도 겹치지 않는 순간 = 이어보기 끊김.
    (바퀴 엔코더가 없으므로 그 구간의 앞뒤 거리는 관측되지 않는다.) → [{t, s_lane, x, y, gap_s}]"""
    s = cl[:, 0]; P = cl[:, 1:3]
    out, hist = [], []
    for n, fi in enumerate(kfs):
        cur = set(int(k) for k in frames_ids[fi])
        t = times[fi]
        hist = [(tt, ss) for tt, ss in hist if t - tt <= window]
        recent = set().union(*[ss for _, ss in hist]) if hist else set()
        last_t = times[kfs[n - 1]] if n else t
        if n and (t_from is None or t > t_from + window) and (not (cur & recent) or t - last_t > window):
            j = int(np.argmin(np.hypot(*(P - cams[n, :2]).T)))
            out.append({"t": round(float(t), 2), "s_lane_m": round(float(s[j]), 1),
                        "x": round(float(cams[n, 0]), 2), "y": round(float(cams[n, 1]), 2),
                        "gap_s": round(float(t - last_t), 2)})
        hist.append((t, cur))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--out")
    ap.add_argument("--marker-size", type=float, default=tm.marker_config()[1],
                    help="마커 검은 사각형 한 변 [m] (기본: track.yaml 에서 고른 마커 세트 markers.use)")
    ap.add_argument("--align", choices=["auto", "start", "lane"], default="start",
                    help="start: 출발 자세 기준 (기본) · lane: 2차선 중심선에 궤적 정합 · auto: 차선 70%%+ 돌았으면 lane")
    ap.add_argument("--fix-mount", action="store_true", help="카메라 pitch·roll·yaw 를 mount.yaml 값으로 고정")
    ap.add_argument("--fix-focal", action="store_true", help="초점거리를 캘리브레이션 값으로 고정 (기본: 배율 f 를 같이 추정)")
    ap.add_argument("--focal0", type=float, default=1.0, help="초점거리 배율 초기값 (fx·fy 에 곱함)")
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--t-move", type=float, default=None,
                    help="출발 시각 (영상 초). video_frames.csv 가 비었거나 첫 W 가 데드밴드라 안 움직였을 때")
    ap.add_argument("--track", default=None)
    ap.add_argument("--mount", default=None)
    a = ap.parse_args()
    sess = os.path.abspath(os.path.expanduser(a.session)); name = os.path.basename(sess)
    out = os.path.abspath(os.path.expanduser(a.out or os.path.join(sess, "map")))
    os.makedirs(out, exist_ok=True)
    track, start, cl = tm.load_track(a.track)
    mount = tm.load_mount(a.mount)

    print(f"[map] 1/3 검출: {sess}/video.mkv")
    det = detect_video(os.path.join(sess, "video.mkv"), os.path.join(out, "detections.pkl"))
    K, D = tm.load_intrinsics(det["width"])
    ids = [f[0] for f in det["frames"]]; uvs = [f[1] for f in det["frames"]]
    times, t_move = session_info(sess, len(ids))
    if a.t_move is not None:
        vf = os.path.join(sess, "video_frames.csv")
        if not os.path.exists(vf) or os.path.getsize(vf) < 20:
            times = np.arange(len(ids)) / 30.0          # 타임스탬프 없음 → 영상 시간 (--t-move 도 영상 초)
        t_move = a.t_move
    print(f"  프레임 {len(ids)} · 마커 검출 프레임 {sum(1 for i in ids if len(i))} · 출발 {t_move}s")

    print("[map] 2/3 초기화 + 번들 조정")
    t0 = time.time()
    r = build(ids, uvs, times, t_move, K, D, a.marker_size, mount, start, cl, a.stride, a.fix_mount, a.align,
              fix_focal=a.fix_focal, focal0=a.focal0)
    print(f"  {time.time() - t0:.0f}s")

    print("[map] 3/3 저장")
    with open(os.path.join(out, "markers.csv"), "w") as f:
        f.write("id,x_m,y_m,z_m,qw,qx,qy,qz,n_obs,rms_px,min_dist_m\n")
        for k in sorted(r["markers"]):
            rv, p = r["markers"][k]; q = Rotation.from_rotvec(rv).as_quat(scalar_first=True); s = r["stats"][k]
            f.write(f"{k},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f},"
                    f"{s['n_obs']},{s['rms_px'] or 0:.2f},{s['min_dist_m'] or 0:.2f}\n")
    with open(os.path.join(out, "trajectory.csv"), "w") as f:
        f.write("frame,t_s,x_m,y_m,theta_deg\n")
        for fi, c in zip(r["kfs"], r["cams"]):
            f.write(f"{fi},{times[fi]:.3f},{c[0]:.4f},{c[1]:.4f},{np.degrees(c[2]):.3f}\n")
    rep = {"session": sess, "marker_size_m": a.marker_size, "n_keyframes": len(r["kfs"]), "n_markers": len(r["markers"]),
           "reproj_rms_px": round(float(np.sqrt((r["rms"] ** 2).mean())), 3),
           "reproj_p95_px": round(float(np.percentile(r["rms"], 95)), 3),
           "mount_estimate": {"pitch_down_deg": round(float(r["pr"][0]), 3), "roll_deg": round(float(r["pr"][1]), 3),
                              "yaw_deg": round(float(r["pr"][2]), 3),
                              "fixed": a.fix_mount},
           "focal_estimate": {"scale": round(float(r["pr"][3]), 4), "fx": round(float(K[0, 0] * r["pr"][3]), 1),
                              "fy": round(float(K[1, 1] * r["pr"][3]), 1), "calib_fx": round(float(K[0, 0]), 1),
                              "fixed": a.fix_focal},
           "start_pose_used": [round(float(v), 4) for v in start],
           "start_pose_final": [round(float(v), 4) for v in r["cams"][0]],
           **r["info"]}
    json.dump(rep, open(os.path.join(out, "report.json"), "w"), indent=2, ensure_ascii=False)
    import yaml
    yaml.safe_dump({"frame": "track", "marker_size_m": a.marker_size, "marker_dictionary": tm.marker_config()[0], "source_session": name,
                    "created": time.strftime("%Y-%m-%d %H:%M"), "aligned_to": r["info"]["aligned_to"],
                    "mount_estimate": rep["mount_estimate"],
                    "focal_estimate": rep["focal_estimate"], "markers_csv": "markers.csv"},
                   open(os.path.join(out, "map.yaml"), "w"), allow_unicode=True, sort_keys=False)
    overview(os.path.join(out, "map_overview.png"), cl, r["cams"], r["markers"], r["cams"][0],
             f"{name}  markers {len(r['markers'])}  reproj {rep['reproj_rms_px']}px  aligned: {r['info']['aligned_to']}")
    print(json.dumps(rep, indent=2, ensure_ascii=False))
    print(f"[map] 완료 → {out}")


if __name__ == "__main__":
    main()
