"""T5. (a) 복도 궤적 떨림 정량화  (b) 실시간 처리량: 검출 1080p/720p, EKF 갱신 vs 랜드마크 수."""
import common  # noqa: F401
import json
import os
import time

import cv2
import numpy as np

from t1_ekf_convention import EKFFixedInit

out = {}

# (a) 떨림: 15프레임(0.5 s) 이동평균에서 벗어난 정도
T = np.loadtxt(os.path.join(common.OUT, "corridor_original", "trajectory.txt"))
p = T[:, 1:4]
k = 15
sm = np.array([p[max(0, i - k // 2): i + k // 2 + 1].mean(0) for i in range(len(p))])
hf = p - sm
step = np.linalg.norm(np.diff(p, axis=0), axis=1)
out["jitter"] = {"hf_std_cm_xyz(cam frame)": np.round(hf.std(0) * 100, 2).tolist(),
                 "frame_step_median_mm": round(float(np.median(step)) * 1000, 1),
                 "frame_step_p95_mm": round(float(np.percentile(step, 95)) * 1000, 1),
                 "smoothed_path_len_m": round(float(np.linalg.norm(np.diff(sm, axis=0), axis=1).sum()), 3),
                 "raw_path_len_m": round(float(step.sum()), 3)}

# (b1) 검출 시간
cap = cv2.VideoCapture(os.path.expanduser("~/car_calibration/corridor_map/videos/corridor_mapping_01.mkv"))
frames = []
for _ in range(60):
    ok, f = cap.read()
    frames.append(f)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
pr = cv2.aruco.DetectorParameters(); pr.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
pr.cornerRefinementWinSize = 3; pr.cornerRefinementMaxIterations = 3; pr.adaptiveThreshWinSizeMin = 3; pr.adaptiveThreshWinSizeMax = 30
det = cv2.aruco.ArucoDetector(d, pr)
for name, size in (("1920x1080", None), ("1280x720", (1280, 720))):
    ts, nd = [], 0
    for f in frames:
        g = f if size is None else cv2.resize(f, size)
        t0 = time.perf_counter(); c, ids, _ = det.detectMarkers(g); ts.append(time.perf_counter() - t0)
        nd += 0 if ids is None else len(ids)
    out[f"detect_ms_{name}"] = {"median": round(float(np.median(ts)) * 1000, 1), "markers_per_frame": round(nd / len(frames), 2)}

# (b2) EKF 1 스텝(predict+update) vs 지도에 있는 랜드마크 수 (보이는 건 3개)
res = {}
for n in (4, 10, 20, 40):
    f = EKFFixedInit(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    for i in range(n):
        f.add_marker(i, np.array([0.1 * i, 0.1, 2.0, 0, 0, 0]))
    ids = np.array([0, 1, 2]); meas = np.array([[0.0, 0.1, 2.0, 0, 0, 0], [0.1, 0.1, 2.0, 0, 0, 0], [0.2, 0.1, 2.0, 0, 0, 0]])
    ts = []
    for _ in range(30):
        t0 = time.perf_counter(); f.observe(ids, meas); ts.append(time.perf_counter() - t0)
    res[n] = round(float(np.median(ts)) * 1000, 2)
out["ekf_step_ms_vs_landmarks(3 visible)"] = res
print(json.dumps(out, ensure_ascii=False, indent=1))
with open(os.path.join(common.OUT, "t5_jitter_perf.json"), "w") as fh:
    json.dump(out, fh, ensure_ascii=False, indent=1)
