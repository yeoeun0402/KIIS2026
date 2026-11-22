"""T8. C920 720p 모드가 1080p 의 단순 축소(같은 화각)인지 — 고정된 카메라로 두 해상도 프레임을 찍어 마커 모서리 비교.

720p 모서리 × 1.5 ≈ 1080p 모서리 이면, 720p 녹화를 1080p 로 리사이즈하고 1080p 캘리브레이션을 써도 된다(run_slam 방식).
카메라를 움직이지 않는다. 결과만 demo/out 에.
"""
import common  # noqa: F401
import json
import os
import subprocess

import cv2
import numpy as np

T = os.path.join(common.OUT, "_tmp"); os.makedirs(T, exist_ok=True)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
pr = cv2.aruco.DetectorParameters(); pr.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
det = cv2.aruco.ArucoDetector(d, pr)
K = np.load("calibration/camera_matrix.npy"); D = np.load("calibration/dist_coeffs.npy")
M = 0.15
OBJ = np.array([[-M / 2, M / 2, 0], [M / 2, M / 2, 0], [M / 2, -M / 2, 0], [-M / 2, -M / 2, 0]], np.float32)


def grab(size):
    f = os.path.join(T, f"grab_{size}.mkv")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "v4l2", "-input_format", "mjpeg",
                    "-video_size", size, "-framerate", "30", "-i", "/dev/video2", "-t", "2", "-c", "copy", f], check=True)
    cap = cv2.VideoCapture(f); fr = []
    while True:
        ok, x = cap.read()
        if not ok:
            break
        fr.append(x)
    corners = {}
    for x in fr[-20:]:
        c, ids, _ = det.detectMarkers(x)
        for ci, i in zip(c, [] if ids is None else ids.flatten()):
            corners.setdefault(int(i), []).append(ci[0])
    return {k: np.median(np.array(v), axis=0) for k, v in corners.items()}


c1080 = grab("1920x1080"); c720 = grab("1280x720")
res = {"ids_1080": sorted(c1080), "ids_720": sorted(c720), "per_marker": {}}
for k in sorted(set(c1080) & set(c720)):
    diff = np.linalg.norm(c720[k] * 1.5 - c1080[k], axis=1)
    _, _, t1 = cv2.solvePnP(OBJ, c1080[k], K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    _, _, t2 = cv2.solvePnP(OBJ, (c720[k] * 1.5).astype(np.float32), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    res["per_marker"][k] = {"corner_diff_px(1080 scale)": round(float(diff.mean()), 2),
                            "pnp_1080_cm": np.round(t1.ravel() * 100, 1).tolist(),
                            "pnp_720x1.5_cm": np.round(t2.ravel() * 100, 1).tolist()}
print(json.dumps(res, ensure_ascii=False, indent=1))
json.dump(res, open(os.path.join(common.OUT, "t8_res_fov.json"), "w"), ensure_ascii=False, indent=1)
