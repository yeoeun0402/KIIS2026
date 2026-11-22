"""[demo] 트랙 지도 공통 모듈 — 좌표계·장착·투영·마커 검출. (매핑 build_map.py · 위치추정 공용)

좌표계 (track.yaml)
  track       매트 왼쪽 아래 모서리, x 오른쪽 16 m · y 위 11 m · z 위
  base_link   뒷축 중심 바닥 투영 (bicycle 기준점), x 앞 · y 왼쪽 · z 위   — 바닥 위 SE(2) (x, y, θ)
  optical     카메라 OpenCV 축 (x 오른쪽 · y 아래 · z 앞)
  marker      마커 중심, 마커 평면 = xy, +y = 마커 그림의 위쪽, +z = 마커 앞면 법선(카메라 쪽)

T_a_b = b 좌표를 a 좌표로 (p_a = R p_b + t). 장착 각도 규약은 ros_ws/.../frames.py T_base_link 와 같다.
"""
import os
import sys

import numpy as np

DEPS = os.path.expanduser("~/aruco_slam_deps")      # cv2 4.10 (ArucoDetector) · scipy 1.14
if DEPS not in sys.path:
    sys.path.insert(0, DEPS)
import cv2  # noqa: E402
import yaml  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

TRACK_DIR = os.path.expanduser("~/demo/track")
MOUNT_YAML = os.path.expanduser("~/demo/ros_ws/src/car_bringup/config/mount.yaml")
CALIB_DIR = os.path.expanduser("~/demo/aruco-slam-fixed/calibration")
CALIB_W = 1920                                         # camera_matrix.npy 가 만들어진 해상도 폭

# camera_link(x 앞, y 왼, z 위) 에서 본 optical 축
R_LINK_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


# ── 설정 읽기 ────────────────────────────────────────────────────
def load_track(path=None):
    path = path or os.path.join(TRACK_DIR, "track.yaml")
    t = yaml.safe_load(open(path))
    cl = np.loadtxt(os.path.join(os.path.dirname(path), t["lane"]["centerline_csv"]), delimiter=",", skiprows=1)
    b = t["start"]["base_link"]
    return t, np.array([b["x"], b["y"], np.radians(b["theta_deg"])]), cl


def load_mount(path=None):
    m = yaml.safe_load(open(path or MOUNT_YAML)) or {}
    return {k: float(m.get(k, 0.0) or 0.0) for k in ("x", "y", "z", "pitch_down_deg", "yaw_deg", "roll_deg",
                                                    "wheelbase", "rear_axle_to_front_bumper")}


def load_intrinsics(width=1920):
    K = np.load(os.path.join(CALIB_DIR, "camera_matrix.npy")).astype(float)
    D = np.load(os.path.join(CALIB_DIR, "dist_coeffs.npy")).astype(float).ravel()
    s = width / CALIB_W
    if abs(s - 1) > 1e-9:
        K = K.copy(); K[:2] *= s
    return K, D


# ── 기하 ─────────────────────────────────────────────────────────
def T_base_opt(m, pitch_down_deg=None, roll_deg=None, yaw_deg=None):
    """장착값 → (R, t): optical → base_link."""
    p = m["pitch_down_deg"] if pitch_down_deg is None else pitch_down_deg
    r = m["roll_deg"] if roll_deg is None else roll_deg
    y = m["yaw_deg"] if yaw_deg is None else yaw_deg
    R_bl = Rotation.from_euler("ZYX", [y, p, r], degrees=True).as_matrix()
    return R_bl @ R_LINK_OPT, np.array([m["x"], m["y"], m["z"]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def lateral_slip(prev, cur):
    """두 자세 사이 이동의 옆 성분 (m). 뒷축 기준 bicycle 모델이면 0 (미끄러짐 없음)."""
    prev = np.atleast_2d(prev); cur = np.atleast_2d(cur)
    th = prev[:, 2] + wrap(cur[:, 2] - prev[:, 2]) / 2
    d = cur[:, :2] - prev[:, :2]
    return -np.sin(th) * d[:, 0] + np.cos(th) * d[:, 1]


def Rz(th):
    c, s = np.cos(th), np.sin(th)
    if np.ndim(th) == 0:
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    R = np.zeros((len(th), 3, 3)); R[:, 0, 0] = c; R[:, 0, 1] = -s; R[:, 1, 0] = s; R[:, 1, 1] = c; R[:, 2, 2] = 1
    return R


def cam_from_base(xyth, R_bo, t_bo):
    """base SE(2) [x y θ] (N,3) → optical 의 track 자세 (R_wc (N,3,3), t_wc (N,3))."""
    xyth = np.atleast_2d(xyth)
    Rw = Rz(xyth[:, 2])
    R_wc = Rw @ R_bo
    t_wc = np.c_[xyth[:, :2], np.zeros(len(xyth))] + Rw @ t_bo
    return R_wc, t_wc


def base_from_cam(R_wc, t_wc, R_bo, t_bo):
    """optical 자세 → base SE(2) (바닥에 투영)."""
    R_wb = R_wc @ R_bo.T
    t_wb = t_wc - R_wb @ t_bo
    return np.array([t_wb[0], t_wb[1], np.arctan2(R_wb[1, 0], R_wb[0, 0])])


def marker_obj(size):
    h = size / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)   # ArUco 코너 순서 (IPPE_SQUARE)


def project(Xc, K, D):
    """카메라 좌표 (...,3) → 픽셀 (...,2), OpenCV 5계수 왜곡 모델."""
    z = Xc[..., 2]
    x = Xc[..., 0] / z; y = Xc[..., 1] / z
    k1, k2, p1, p2, k3 = D[:5]
    r2 = x * x + y * y
    rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]], -1)


# ── 마커 검출 ─────────────────────────────────────────────────────
def marker_set(path=None):
    """지금 쓰는 마커 세트 (track.yaml markers.use, 환경변수 MARKERS 가 있으면 그것이 우선).
    → dict(name, dictionary, size_m, live_map = 실시간 고정 지도 폴더 절대 경로).
    예전 형식 (markers 바로 밑 dictionary·size_m) 도 읽음. 사전이 없으면 예전 마커 DICT_5X5_50."""
    path = path or os.path.join(TRACK_DIR, "track.yaml")
    m = load_track(path)[0].get("markers", {}) or {}
    sets = m.get("sets") or {}
    if sets:
        name = os.environ.get("MARKERS") or str(m.get("use", ""))
        if name not in sets:
            raise ValueError(f"마커 세트 '{name}' 없음 (track.yaml markers.sets: {', '.join(sets)})")
        c = sets[name]
    else:
        name, c = "default", m
    lm = str(c.get("live_map", "live_map"))
    return {"name": name, "dictionary": str(c.get("dictionary", "DICT_5X5_50")), "size_m": float(c.get("size_m", 0.20)),
            "live_map": lm if os.path.isabs(lm) else os.path.join(os.path.dirname(path), lm)}


def marker_config(path=None):
    """지금 마커 세트 → (사전 이름, 검은 사각형 한 변 m)."""
    c = marker_set(path)
    return c["dictionary"], c["size_m"]


def make_detector(dictionary=None):
    dictionary = dictionary or marker_config()[0]
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMaxIterations = 30
    p.cornerRefinementMinAccuracy = 0.01
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 30
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)), p)


def marker_poses_ippe(corners, size, K, D):
    """한 마커의 4 코너 → IPPE 두 해 [(R_cm, t_cm, err), ...] (오차 작은 순)."""
    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(marker_obj(size), corners.astype(np.float64), K, D,
                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    out = []
    for rv, tv, e in zip(rvecs, tvecs, np.ravel(errs)):
        out.append((cv2.Rodrigues(rv)[0], tv.ravel(), float(e)))
    return sorted(out, key=lambda a: a[2])
