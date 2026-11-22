"""T3. 현재 1080p 캘리브레이션 재현·불확실도 — 원본 이미지를 읽기만 하고 결과는 demo/out 에만 쓴다.

보드: charuco_calibration.py 와 같은 DICT_5X5_250, 5x7, 0.03/0.015 m.
cv2.calibrateCameraExtended 로 fx·cx·cy 표준편차와 이미지별 오차까지 본다.
"""
import common  # noqa: F401
import glob
import json
import os

import cv2
import numpy as np

SETS = {
    "aruco-slam/images_1080p_selected": os.path.join(common.ARUCO, "calibration/images_1080p_selected"),
    "car_calibration/1080p/recal_20261119": os.path.join(common.CALIB, "1080p/recal_20261119/images_1080p_selected"),
}
board = cv2.aruco.CharucoBoard((5, 7), 0.03, 0.015, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250))
det = cv2.aruco.CharucoDetector(board)


def run(folder):
    objs, imgs, size, names = [], [], None, []
    for f in sorted(glob.glob(os.path.join(folder, "*.jpg"))):
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        size = im.shape[::-1]
        cc, ci, _, _ = det.detectBoard(im)
        if ci is None or len(ci) < 8:
            continue
        o, i = board.matchImagePoints(cc, ci)
        objs.append(o); imgs.append(i); names.append(os.path.basename(f))
    rms, K, D, _, _, sdi, _, per = cv2.calibrateCameraExtended(objs, imgs, size, None, None)
    sd = sdi.ravel()
    # 주점 고정(영상 중심) + 접선왜곡 0 으로 다시 — 주점이 데이터로 잘 안 잡히는지 비교
    K0 = np.array([[1450, 0, size[0] / 2], [0, 1450, size[1] / 2], [0, 0, 1]], float)
    rms2, K2, D2, *_ = cv2.calibrateCamera(objs, imgs, size, K0.copy(), None,
                                           flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_ZERO_TANGENT_DIST)
    per = per.ravel()
    worst = sorted(zip(per, names), reverse=True)[:5]
    return {
        "image_size": size, "images_used": len(objs), "rms_px": round(rms, 3),
        "fx": round(K[0, 0], 1), "fy": round(K[1, 1], 1), "cx": round(K[0, 2], 1), "cy": round(K[1, 2], 1),
        "std_fx": round(float(sd[0]), 1), "std_cx": round(float(sd[2]), 1), "std_cy": round(float(sd[3]), 1),
        "dist": np.round(D.ravel(), 4).tolist(),
        "worst_images_px": [[round(float(e), 2), n] for e, n in worst],
        "fixed_pp_zero_tangent": {"rms_px": round(rms2, 3), "fx": round(K2[0, 0], 1), "fy": round(K2[1, 1], 1),
                                  "dist": np.round(D2.ravel(), 4).tolist()},
    }


if __name__ == "__main__":
    out = {k: run(v) for k, v in SETS.items()}
    print(json.dumps(out, ensure_ascii=False, indent=1))
    with open(os.path.join(common.OUT, "t3_calib_recheck.json"), "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
