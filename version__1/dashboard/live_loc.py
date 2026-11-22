import argparse
import glob
import json
import os
import sys
import time

import numpy as np

DEMO = os.path.expanduser("~/demo")
HERE = os.path.dirname(os.path.abspath(__file__))
HERE_V1 = os.path.dirname(HERE)                       # version__1
for p in (os.path.expanduser("~/aruco_slam_deps"), os.path.join(DEMO, "track", "tools"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)
sys.dont_write_bytecode = True

import cv2  # noqa: E402
import yaml  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

import trackmap as tm  # noqa: E402
import csv  # noqa: E402

from loc_kf import LocKF, CSV_COLS, csv_row, cov6, fmt  # noqa: E402

GAP_RESET_S = 3.0
MAX_RMS_PX = 8.0
# 마커 1개만 보이면 바닥 위 자세가 잘 안 정해짐 (녹화 시험: 2개 이상 중앙값 1 cm, 1개 0.65 m) →
# 직전 자세에서 이 이상 튀면 버리고 직전 자세 유지 (0.15 m + 최고 속도 1 m/s × 경과 시간)
ONE_MARKER_JUMP_M = 0.15
MAX_SPEED_MPS = 1.0
# 출발 자세 맞춤: 새 주행의 처음 (차가 출발선에 서 있을 때) 마커 2개 이상 본 프레임 ALIGN_N 개의 평균 자세를
# 출발 자세(track.yaml)에 맞추는 SE(2) 보정을 이후 모든 출력에 적용. build_map 이 출발 자세 prior 로 지도를 붙이는 것과 같은 기준
# → 차를 놓은 위치·지도마다 조금씩 다른 좌표가 시뮬(같은 출발 자세에서 시작)·주행 뒤 localization 과 맞춰짐
ALIGN_N = 10
MIN_MARKER_OBS = 20     # 지도에서 이보다 적게 관측된 마커는 쓰지 않음


def map_dictionary_ok(d):
    """지도를 만든 마커 사전이 지금 track.yaml 사전과 같은지 (다르면 ID 가 같아도 다른 마커). 예전 지도는 DICT_5X5_50."""
    try:
        cfg = yaml.safe_load(open(os.path.join(d, "map.yaml"))) or {}
    except OSError:
        return False
    return str(cfg.get("marker_dictionary", "DICT_5X5_50")) == tm.marker_config()[0]


def latest_map():
    """쓸 마커 지도: 지금 마커 세트의 고정 지도 (track.yaml markers.sets.<use>.live_map, 기본 ~/demo/track/live_map) 가
    있으면 그것, 없으면 같은 사전으로 만든 세션 지도 중 가장 좋은 것.
    좋은 것 = FAIL 아님 · 재투영 RMS 3 px 이하 중 마커별 관측 수의 최솟값이 가장 큰 (모든 마커를 가까이서 많이 본) 지도.
    (2026-11-21: 예전엔 '가장 최근' 지도를 썼는데, 8 m 만 달린 지도라 먼 마커 위치가 0.3~0.9 m 틀려 위치가 0.5 m 어긋났음)"""
    fixed = tm.marker_set()["live_map"]
    if os.path.isfile(os.path.join(fixed, "markers.csv")) and map_dictionary_ok(fixed):
        return fixed
    best, best_key = None, None
    for d in glob.glob(os.path.join(DEMO, "sessions", "*", "map")):
        try:
            r = json.load(open(os.path.join(d, "report.json")))
            m = np.genfromtxt(os.path.join(d, "markers.csv"), delimiter=",", names=True, ndmin=1)
        except (OSError, ValueError):
            continue
        if r.get("verdict") == "FAIL" or float(r.get("reproj_rms_px", 99)) > 3.0 or not map_dictionary_ok(d):
            continue
        key = (float(np.min(m["n_obs"])), -float(r.get("reproj_rms_px", 99)))
        if best_key is None or key > best_key:
            best, best_key = d, key
    return best


class MapLocalizer:
    def __init__(self, map_dir, width):
        cfg = yaml.safe_load(open(os.path.join(map_dir, "map.yaml")))
        size = float(cfg.get("marker_size_m", 0.2))
        me = cfg.get("mount_estimate", {}) or {}
        mount = tm.load_mount()
        self.R_bo, self.t_bo = tm.T_base_opt(mount, me.get("pitch_down_deg"), me.get("roll_deg"), me.get("yaw_deg"))
        K, self.D = tm.load_intrinsics(width)
        f = float((cfg.get("focal_estimate", {}) or {}).get("scale", 1.0))
        K[0, 0] *= f; K[1, 1] *= f
        self.K, self.width = K, width
        obj = tm.marker_obj(size)
        self.world = {}                                   # id → (4,3) 코너 트랙 좌표
        for r in np.genfromtxt(os.path.join(map_dir, "markers.csv"), delimiter=",", names=True, ndmin=1):
            if "n_obs" in r.dtype.names and r["n_obs"] < MIN_MARKER_OBS:   # 몇 번 안 보인 마커 = 오검출일 수 있음
                continue
            R = Rotation.from_quat([r["qw"], r["qx"], r["qy"], r["qz"]], scalar_first=True).as_matrix()
            self.world[int(r["id"])] = (R @ obj.T).T + np.array([r["x_m"], r["y_m"], r["z_m"]])
        self.det = tm.make_detector(str(cfg.get("marker_dictionary", "DICT_5X5_50")))   # 지도를 만든 마커 사전
        # 매트에 직접 맞춘 지도 (update_map.sh: 마커를 2차선 바깥 선에 맞춤 / --align lane) 는 지도 좌표를 그대로 믿음 →
        # 출발 자세 맞춤을 하지 않음 (하면 차를 놓은 방향 오차만큼 지도 회전 보정이 도로 풀림 — 2026-11-22 확인: -5.7° 되돌림)
        self.trust_map = str(cfg.get("aligned_to", "start")) in ("markers_on_outer_line", "lane")
        _, self.start, _ = tm.load_track()
        self.reset()

    def reset(self):
        self.pose = self.start.copy()
        self.align_buf, self.corr = [], None       # corr = (dθ, R, t): 지도 자세 → 출발 맞춘 자세

    def aligned(self, x):
        """지도 좌표 자세 x → 출발 자세에 맞춘 자세 (보정이 아직 없으면 None)."""
        if self.corr is None:
            return None
        dth, R, t = self.corr
        p = R @ x[:2] + t
        return np.array([p[0], p[1], tm.wrap(x[2] + dth)])

    def _try_align(self, x, n):
        if self.corr is not None or n < 2:
            return
        self.align_buf.append(x.copy())
        if len(self.align_buf) < ALIGN_N:
            return
        A = np.array(self.align_buf)
        m = np.r_[A[:, :2].mean(0), np.arctan2(np.sin(A[:, 2]).mean(), np.cos(A[:, 2]).mean())]
        dth = tm.wrap(self.start[2] - m[2])
        meas = (m[0] - self.start[0], m[1] - self.start[1], np.degrees(-dth))   # 출발선에서 잰 자세 − 출발 자세 (dx, dy, dθ°)
        if self.trust_map:                                 # 지도 그대로 (보정 없음), 잰 차이만 알림
            dth, R, t = 0.0, np.eye(2), np.zeros(2)
        else:
            R = np.array([[np.cos(dth), -np.sin(dth)], [np.sin(dth), np.cos(dth)]])
            t = self.start[:2] - R @ m[:2]
        self.corr = (dth, R, t)
        print(f"ALIGN,{meas[0]:.3f},{meas[1]:.3f},{meas[2]:.2f},{'map' if self.trust_map else 'start'}", flush=True)

    def _resid(self, xyth, Xw, uv):
        R_wc, t_wc = tm.cam_from_base(xyth, self.R_bo, self.t_bo)
        Xc = (Xw - t_wc[0]) @ R_wc[0]
        if np.any(Xc[:, 2] < 0.05):
            return np.full(uv.size, 1e3)
        return (tm.project(Xc, self.K, self.D) - uv).ravel()

    def step(self, frame, dt=0.07):
        if frame.shape[1] != self.width:
            s = frame.shape[1] / self.width
            self.K = self.K.copy(); self.K[:2] *= s; self.width = frame.shape[1]
        corners, ids, _ = self.det.detectMarkers(frame)
        if ids is None:
            return None
        Xw, uv, n = [], [], 0
        for c, k in zip(corners, ids.ravel()):
            if int(k) in self.world:
                Xw.append(self.world[int(k)]); uv.append(c[0]); n += 1
        if not n:
            return None
        Xw, uv = np.concatenate(Xw), np.concatenate(uv).astype(float)
        best = None
        for th0 in (0.0, 0.35, -0.35):                    # 직전 자세 + 방향 ±20° (빠른 회전 대비)
            x0 = self.pose + np.array([0, 0, th0])
            r = least_squares(self._resid, x0, args=(Xw, uv), loss="huber", f_scale=3.0, max_nfev=60)
            rms = float(np.sqrt(np.mean(r.fun ** 2)))
            if best is None or rms < best[1]:
                best = (r.x, rms, r.jac)
            if rms < 3.0:
                break
        x, rms = best[0], best[1]
        if rms > MAX_RMS_PX:
            return None
        if n == 1 and np.hypot(*(x[:2] - self.pose[:2])) > ONE_MARKER_JUMP_M + MAX_SPEED_MPS * dt:
            return None
        x[2] = tm.wrap(x[2])
        self.pose = x
        self._try_align(x, n)
        y = self.aligned(x)
        if y is None:                                     # 출발 자세 맞춤 전 (처음 1초 정도): 내보내지 않음
            return None
        C = self.covariance(best[2], rms, n)
        self.last_cov_white = self.covariance(best[2], rms, n, floor=False)   # 칼만(loc_kf)용: 지도 오차 바닥값 뺀 프레임 잡음
        return float(y[0]), float(y[1]), float(y[2]), n, rms, C

    def covariance(self, J, rms, n, floor=True):
        """측정 공분산 (x, y, θ) — 칼만 필터가 이 측정을 얼마나 믿을지.
        C = σ² (JᵀJ)⁻¹ (J = 픽셀 잔차의 자세 미분, σ = 픽셀 잡음 max(재투영 RMS, 1 px))
        마커가 멀리 한 줄로만 보이면 JᵀJ 가 한 방향으로 작아져 그 방향 분산이 커짐 (아래 직선에서 앞뒤·옆이 모호한 것).
        + 지도 오차 바닥값 (0.05 m, 1°)², 마커 1개면 4배. 출발 맞춤 회전도 적용.
        floor=False: 바닥값 없이 (ArUco 칼만용). 지도 오차는 같은 곳에서 늘 같은 쪽으로 틀린 '치우침'이라 프레임마다 새로 생기는
        잡음이 아님 → 칼만에 넣으면 측정을 너무 안 믿어 늦게 따라감 (녹화 재생: 바닥값 넣으면 NIS 평균 0.1, 이상적 3)."""
        sig = max(float(rms), 1.0)
        try:
            C = sig ** 2 * np.linalg.inv(J.T @ J)
        except np.linalg.LinAlgError:
            C = np.diag([1.0, 1.0, 0.1])
        if floor:
            C = C + np.diag([0.05 ** 2, 0.05 ** 2, np.radians(1.0) ** 2])
        if n < 2:
            C = C * 4.0
        if self.corr is not None:
            R = np.eye(3); R[:2, :2] = self.corr[1]
            C = R @ C @ R.T
        return C


def frames_from_jpeg(path):
    """파일이 바뀔 때마다 (epoch = 파일 수정 시각, frame). ffmpeg 원자적 교체라 반쯤 쓴 파일은 없음."""
    last = None
    while True:
        try:
            st = os.stat(path)
        except FileNotFoundError:
            st = None
        if st is not None and (st.st_ino, st.st_mtime_ns) != last:
            last = (st.st_ino, st.st_mtime_ns)
            fr = cv2.imread(path)
            if fr is not None:
                yield st.st_mtime, fr
                continue
        time.sleep(0.01)


def frames_from_video(path, width):
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    i, step = 0, max(1, int(round(fps / 15)))           # 실시간처럼 15 fps 로 솎음
    while True:
        ok, fr = cap.read()
        if not ok:
            return
        if i % step == 0:
            h = int(round(fr.shape[0] * width / fr.shape[1] / 2)) * 2
            yield i / fps, cv2.resize(fr, (width, h), interpolation=cv2.INTER_AREA)
        i += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jpeg", default="/dev/shm/twin_live/cam.jpg")
    ap.add_argument("--video", help="시험: 녹화 영상으로 (epoch 대신 영상 초)")
    ap.add_argument("--map", help="build_map 결과 폴더 (기본: 가장 최근 FAIL 아닌 것)")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--no-kf", action="store_true", help="칼만 필터 끄기 (KF 줄·kf 열 없음)")
    ap.add_argument("--log-dir", default=os.path.join(HERE_V1, "results", "loc_kf"),
                    help="CSV 기록 폴더 ('' 이면 기록 안 함)")
    ap.add_argument("--log-name", help="CSV 이름 앞부분 (기본 loc_<시각>)")
    a = ap.parse_args()

    map_dir = a.map or latest_map()
    if not map_dir:
        ms = tm.marker_set()
        print(f"ERR,0,마커 세트 '{ms['name']}' ({ms['dictionary']}) 지도 없음 — {ms['live_map']} 도, 같은 사전으로 만든 "
              f"~/demo/sessions/*/map 도 없음 (한 바퀴 주행 → build_map → 그 폴더에 복사)", flush=True)
        sys.exit(1)
    loc = MapLocalizer(map_dir, a.width)
    kf = None if a.no_kf else LocKF()
    src = frames_from_video(os.path.expanduser(a.video), a.width) if a.video else frames_from_jpeg(a.jpeg)
    ms = tm.marker_set()
    print(f"MARKERS,{ms['name']},{ms['dictionary']},{ms['size_m']}", flush=True)   # 지금 마커 세트 (track.yaml markers.use / MARKERS)
    print(f"READY,{map_dir}", flush=True)
    log = Logger(a.log_dir, a.log_name, map_dir)
    run = 0
    last_t, last_ok = None, None
    for t, fr in src:
        if last_t is not None and t - last_t > GAP_RESET_S:     # 새 주행: 출발 자세에서 다시
            loc.reset()
            if kf:
                kf.reset()
            last_ok = None
            run += 1
            log.close()
            print(f"RESET,{t:.3f}", flush=True)
        last_t = t
        try:
            res = loc.step(fr, dt=0.07 if last_ok is None else t - last_ok)
        except Exception as e:                                   # 한 프레임 실패로 멈추지 않게
            print(f"ERR,{t:.3f},{type(e).__name__}: {e}", flush=True)
            continue
        z = R = None; n, rms = 0, float("nan")
        if res:
            last_ok = t
            x, y, th, n, rms, C = res
            z, R = (x, y, th), loc.last_cov_white                 # 칼만에는 바닥값 뺀 R (LOC 줄 = 옵저버용은 그대로 C)
            print(f"LOC,{t:.3f},{x:.4f},{y:.4f},{th:.4f},{n},{rms:.2f},{C[0, 0]:.3e},{C[0, 1]:.3e},{C[1, 1]:.3e},"
                  f"{C[2, 2]:.3e},{C[0, 2]:.3e},{C[1, 2]:.3e}", flush=True)
        elif loc.corr is None:                                   # 출발 맞춤 전: 칼만도 아직 시작 안 함
            continue
        out = kf.step(t, z, R) if kf else None
        if out is not None:
            s_, P = out["x"], out["P"]
            print("KF,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3e,%.3e,%.3e,%.3e,%.3e,%.3e,%.2f,%d" % (
                t, *s_, *cov6(P), out["nis"] if np.isfinite(out["nis"]) else -1, out["flag"]), flush=True)
        log.write(csv_row(run, t, z, n, rms, R, out))
    log.close()


class Logger:
    """주행마다 CSV 한 파일 (처음 쓸 때 연다)."""
    def __init__(self, d, name, map_dir):
        self.d, self.name, self.map_dir = d, name, map_dir
        self.f = self.w = None
        self.k = 0

    def write(self, row):
        if not self.d:
            return
        if self.f is None:
            os.makedirs(self.d, exist_ok=True)
            base = self.name or "loc_" + time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.d, f"{base}{'' if self.k == 0 else f'_{self.k}'}.csv")
            self.f = open(path, "w", newline="")
            json.dump({"map": self.map_dir, "columns": CSV_COLS, "kf_flag": "1 반영, 0 이상치 버림, -1 예측만, 2 (다시) 시작, -2 칼만 꺼짐",
                       "frame": "track (원점 매트 왼쪽 아래)"}, open(path[:-4] + ".json", "w"), ensure_ascii=False, indent=1)
            self.w = csv.writer(self.f)
            self.w.writerow(CSV_COLS)
            print(f"LOG,{path}", flush=True)
        self.w.writerow([fmt(v) for v in row])
        self.f.flush()

    def close(self):
        if self.f is not None:
            self.f.close()
            self.f = None
            self.k += 1


if __name__ == "__main__":
    main()
