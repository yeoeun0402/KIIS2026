"""T5b. 처리 시간 실측(끝에서 끝까지): 복도 영상 300프레임, 1080p 와 720p.
720p 는 캘리브레이션 K 를 2/3 로 축소해서 올바르게 처리. 프레임당 디코드+리사이즈 / process_frame(검출+PnP+EKF) 분리."""
import common  # noqa: F401
import json
import os
import time

import cv2
import numpy as np

from t1_ekf_convention import EKFFixedInit

V = os.path.expanduser("~/car_calibration/corridor_map/videos/corridor_mapping_01.mkv")
res = {}
for name, size, s in (("1920x1080", (1920, 1080), 1.0), ("1280x720", (1280, 720), 2 / 3)):
    tr = EKFFixedInit(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    tr.calib_matrix = tr.calib_matrix.copy(); tr.calib_matrix[:2] *= s
    cap = cv2.VideoCapture(V); dec, proc = [], []
    for _ in range(300):
        t0 = time.perf_counter(); ok, fr = cap.read()
        if not ok:
            break
        if fr.shape[1] != size[0]:
            fr = cv2.resize(fr, size)
        t1 = time.perf_counter(); tr.process_frame(fr); t2 = time.perf_counter()
        dec.append(t1 - t0); proc.append(t2 - t1)
    d, p = np.median(dec) * 1000, np.median(proc) * 1000
    res[name] = {"decode_resize_ms": round(float(d), 1), "process_frame_ms": round(float(p), 1),
                 "total_ms": round(float(d + p), 1), "fps": round(1000 / float(d + p), 1), "landmarks": len(tr.landmarks)}
print(json.dumps(res, indent=1))
json.dump(res, open(os.path.join(common.OUT, "t5b_perf_e2e.json"), "w"), indent=1)
