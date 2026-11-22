"""T2. 캘리브레이션 후보별 PnP 거리 정확도 — 실측 100 cm / 150 cm 영상(car_calibration/aruco_tests/distance_*.mkv).

aruco-slam 과 같은 검출기 설정(DICT_5X5_50, SUBPIX)·같은 PnP(IPPE_SQUARE)·마커 0.15 m 로 측정.
후보: current(aruco-slam/calibration = 1080p/recal_20261119), old(1080p/results), upstream(레포 원래 값).
"""
import common  # noqa: F401
import json
import os
import subprocess

import cv2
import numpy as np

from filters.base_filter import BaseFilter  # noqa: F401  (검출기 설정 복사용 참조)

MARKER = 0.15
SC = os.path.join(common.OUT, "_tmp")
os.makedirs(SC, exist_ok=True)


def upstream_calib():
    for n in ("camera_matrix", "dist_coeffs"):
        with open(os.path.join(SC, f"up_{n}.npy"), "wb") as f:
            f.write(subprocess.run(["git", "-C", common.ARUCO, "show", f"HEAD:calibration/{n}.npy"],
                                   capture_output=True, check=True).stdout)
    return np.load(os.path.join(SC, "up_camera_matrix.npy")), np.load(os.path.join(SC, "up_dist_coeffs.npy"))


CANDS = {
    "current(recal_20261119)": (np.load(os.path.join(common.ARUCO, "calibration/camera_matrix.npy")),
                                np.load(os.path.join(common.ARUCO, "calibration/dist_coeffs.npy"))),
    "old(1080p/results)": (np.load(os.path.join(common.CALIB, "1080p/results/camera_matrix.npy")),
                           np.load(os.path.join(common.CALIB, "1080p/results/dist_coeffs.npy"))),
    "upstream(git HEAD)": upstream_calib(),
}


def detector():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 3
    p.cornerRefinementMaxIterations = 3
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 30
    return cv2.aruco.ArucoDetector(d, p)


OBJ = np.array([[-MARKER / 2, MARKER / 2, 0], [MARKER / 2, MARKER / 2, 0],
                [MARKER / 2, -MARKER / 2, 0], [-MARKER / 2, -MARKER / 2, 0]], np.float32)


def measure(video, step=3):
    det = detector()
    cap = cv2.VideoCapture(video)
    corners_all, ids_seen, side_px, n = [], set(), [], 0
    i = 0
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        i += 1
        if i % step:
            continue
        n += 1
        c, ids, _ = det.detectMarkers(fr)
        if ids is None:
            continue
        ids_seen.update(int(x) for x in ids.flatten())
        corners_all.append(c[0])
        side_px.append(float(np.mean(np.linalg.norm(np.diff(np.vstack([c[0][0], c[0][0][:1]]), axis=0), axis=1))))
    res = {"frames_sampled": n, "detected": len(corners_all), "ids": sorted(ids_seen),
           "marker_side_px": round(float(np.median(side_px)), 1) if side_px else None, "by_calib": {}}
    for name, (K, D) in CANDS.items():
        z, norm = [], []
        for c in corners_all:
            _, rv, tv = cv2.solvePnP(OBJ, c, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            z.append(float(tv[2])); norm.append(float(np.linalg.norm(tv)))
        if z:
            res["by_calib"][name] = {"z_median_cm": round(100 * np.median(z), 1), "range_median_cm": round(100 * np.median(norm), 1),
                                     "z_std_cm": round(100 * np.std(z), 2)}
    return res


if __name__ == "__main__":
    out = {"calibs": {k: {"fx": round(float(K[0, 0]), 1), "fy": round(float(K[1, 1]), 1), "cx": round(float(K[0, 2]), 1),
                          "cy": round(float(K[1, 2]), 1), "dist": np.round(D.ravel(), 4).tolist()} for k, (K, D) in CANDS.items()},
           "videos": {}}
    for name in ("distance_100cm", "distance_100cm_recheck", "distance_100cm_fixedfocus",
                 "distance_150cm", "distance_150cm_recheck", "distance_150cm_fixedfocus"):
        out["videos"][name] = measure(os.path.join(common.CALIB, "aruco_tests", name + ".mkv"))
        print(name, json.dumps(out["videos"][name]["by_calib"], ensure_ascii=False), out["videos"][name]["marker_side_px"], flush=True)
    # 핀홀 역산: 실측 거리 d, 픽셀 한 변 s → 필요한 fx ≈ s·d / 0.15 (마커가 광축 근처일 때)
    for name, v in out["videos"].items():
        d = 1.0 if "100" in name else 1.5
        if v["marker_side_px"]:
            v["implied_fx_if_marker_0.15m"] = round(v["marker_side_px"] * d / MARKER, 1)
    with open(os.path.join(common.OUT, "t2_calib_distance.json"), "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: v.get("implied_fx_if_marker_0.15m") for k, v in out["videos"].items()}))
