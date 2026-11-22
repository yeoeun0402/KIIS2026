"""카메라 초점거리(fx) 확인 — 줄자로 잰 거리에 마커를 놓고 사진 한 장으로.

  python3 -B check_camera.py --dist 2.00 [--dist 3.00 ...]   (실차 주행 중이 아닐 때 — 카메라를 이 스크립트가 씀)

준비
  1. 차를 평평한 바닥에 두고, 마커(검은 사각형 17.5 cm)를 차 정면에 세움. 마커 면이 카메라를 똑바로 보게.
  2. 렌즈 중심 바로 아래 바닥점 → 마커 받침 앞면(마커 면)까지 수평 거리를 줄자로 잼 = --dist (m)
     (렌즈가 뒷축 바로 위이므로 차 옆에서 렌즈 위치를 바닥에 표시하고 재면 됨)
  3. 실행하면 C920 을 drive.sh 와 같은 설정(1920x1080, 초점 고정 19, 노출 156)으로 열고 사진 몇 장을 찍어 마커를 찾음.

계산 (마커 크기 S 를 알면 거리 = fx · S / 픽셀 크기 → 초점거리는 거리에 그대로 비례)
  지금 보정값(fx 1481)으로 PnP 한 수평 거리 d_est 와 줄자 거리 d 로  fx_실제 ≈ fx_지금 · d / d_est
  여러 거리를 주면 평균. 마커 바닥 접점 픽셀로 렌즈 높이·아래 기울기 일관성도 같이 출력.
사진은 ~/demo/track/camera_check/ 에 저장.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trackmap as tm  # noqa: E402  (cv2 4.10 을 먼저 불러야 함)
import cv2  # noqa: E402
import numpy as np  # noqa: E402

SIZE = tm.marker_config()[1]                # track.yaml 에서 고른 마커 세트
OUT = os.path.expanduser("~/demo/track/camera_check")


def find_c920():
    for d in sorted(glob.glob("/sys/class/video4linux/video*"), key=lambda p: int(p.rsplit("video", 1)[1] or 0)):
        try:
            if "c920" in open(os.path.join(d, "name")).read().lower() and open(os.path.join(d, "index")).read().strip() == "0":
                return "/dev/" + os.path.basename(d)
        except OSError:
            pass
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, action="append", required=True, help="렌즈 아래 바닥점 → 마커 면 수평 거리 [m] (마커 하나씩 바꿔 가며 여러 번 실행해도 됨)")
    ap.add_argument("--device", default=None)
    ap.add_argument("--frames", type=int, default=15)
    a = ap.parse_args()
    dev = a.device or find_c920()
    if not dev:
        sys.exit("C920 을 못 찾음 (--device /dev/videoN)")
    for c in ("focus_automatic_continuous=0", "auto_exposure=1", "focus_absolute=19", "exposure_time_absolute=156"):
        subprocess.run(["v4l2-ctl", "-d", dev, "-c", c], capture_output=True)
    cap = cv2.VideoCapture(int(dev.replace("/dev/video", "")), cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    K, D = tm.load_intrinsics(1920)
    m = tm.load_mount()
    R_bo, t_bo = tm.T_base_opt(m)
    det = tm.make_detector()
    os.makedirs(OUT, exist_ok=True)
    for _ in range(10):
        cap.read()
    ests, sides = [], []
    for k in range(a.frames):
        ok, fr = cap.read()
        if not ok:
            continue
        corners, ids, _ = det.detectMarkers(fr)
        if ids is None:
            continue
        # 가장 크게 보이는 마커
        j = int(np.argmax([cv2.contourArea(c[0]) for c in corners]))
        c = corners[j][0]
        R_cm, t_cm, _ = tm.marker_poses_ippe(c, SIZE, K, D)[0]
        p_b = R_bo @ t_cm + t_bo                       # 마커 중심, 차(base) 좌표
        ests.append(np.hypot(p_b[0], p_b[1]) - t_bo[0]); sides.append(np.mean(np.linalg.norm(np.diff(np.r_[c, c[:1]], axis=0), axis=1)))
        if k == 0:
            cv2.imwrite(os.path.join(OUT, time.strftime("%Y%m%d_%H%M%S") + ".jpg"), fr)
    cap.release()
    if not ests:
        sys.exit("마커를 못 찾음 — 마커가 화면에 보이는지, 너무 가깝지 않은지 (0.9 m 보다 멀게) 확인")
    d_est = float(np.median(ests))
    d = a.dist[0]
    fx = K[0, 0]
    print(f"마커 크기 {SIZE} m · 지금 fx {fx:.1f} px · 마커 한 변 {np.median(sides):.1f} px (사진 {len(ests)}장)")
    print(f"PnP 수평 거리 {d_est:.3f} m  /  줄자 {d:.3f} m  →  비 {d / d_est:.4f}")
    print(f"→ 실제 fx ≈ {fx * d / d_est:.1f} px  (궤적이 차선에 맞으려면 약 1400~1415 로 예상)")


if __name__ == "__main__":
    main()
