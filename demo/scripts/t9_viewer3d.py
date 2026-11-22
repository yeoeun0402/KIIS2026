"""T9. Pangolin 3D 뷰어(원본 viewers/viewer_3d.py) 실제 창 띄우기 — 정지 세션 영상 120프레임, 내보내기 없음."""
import common  # noqa: F401
import json, os, time
import cv2, numpy as np
import viewers.viewer_3d as v3d
from t1_ekf_convention import EKFFixedInit
sess = os.path.join(common.OUT, "drive_logs", "20261120_002824", "video.mkv")
tr = EKFFixedInit(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
viewer = v3d.Viewer3D(export_video=False)
cap = cv2.VideoCapture(sess); n = 0; tv = []
while n < 120:
    ok, fr = cap.read()
    if not ok: break
    _, cam, mk, det = tr.process_frame(fr)
    t0 = time.perf_counter(); viewer.view(cam, mk, det); tv.append(time.perf_counter() - t0); n += 1
viewer.close()
r = {"frames": n, "viewer_ms_median": round(float(np.median(tv)) * 1000, 1), "landmarks": len(mk)}
print(json.dumps(r)); json.dump(r, open(os.path.join(common.OUT, "t9_viewer3d.json"), "w"))
