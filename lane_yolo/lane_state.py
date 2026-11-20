"""차선 3개의 직선/곡선 판단 + 제어기 전환 판단 (사진마다)

1) YOLOv8-seg로 차선 마스크를 얻음 (--source gt면 Unity 정답 마스크를 대신 써서 YOLO 오차와 판단 오차를 나눠 볼 수 있음)
2) 마스크 픽셀을 바닥 평면으로 옮김 (camera.json: 높이 0.637 m, 수평, 세로 화각 60°)
     카메라 좌표 x_c = (u - cx) / f, y_c = (cy - v) / f,  f = (H/2) / tan(fov/2)
     바닥까지 거리 Z = h / (-y_c)  (수평선 아래 픽셀만),  옆 거리 X = x_c * Z
3) 차 앞 zmin~zmax [m] 안의 점으로 곡률 계산
     점들의 주축(PCA) 방향을 s, 수직 방향을 d로 두고 d = a s^2 + b s + c 를 맞춤
     곡률 kappa = 2a / (1 + b^2)^(3/2)  (주축 가운데에서), 단위 1/m
     주축 방향 길이(span)가 min-span [m]보다 짧으면 판단하지 않음 (none)
4) 차선 상태: |kappa| > kappa-th 이면 curve, 아니면 straight, 안 보이거나 너무 짧으면 none
5) 전환 규칙 (사용자 지정, 2026-11-15):
     보이는 차선(상태가 none이 아닌 차선)이 1개 이상이고 모두 curve  →  PP_PID
     그 외 (직선이 하나라도 있거나, 보이는 차선이 없음)              →  PID

--eval: MATLAB 레퍼런스 경로 곡률로 정답을 만들어 비교
     차 위치(capture_log의 x, y = 뒤 차축)에서 경로를 따라 zmin~zmax 앞까지의 평균 |kappa_ref| > ref-th 이면 정답 curve
     → 전환 판단 정확도와, 기준값(kappa-th)을 바꿔 가며 본 정확도 표를 출력

출력: analysis/<Play>_lane_state.csv  (--source gt면 _lane_state_gt.csv)

사용:
  python lane_state.py --play 20261115_205703 --eval
  python lane_state.py --play 20261115_205703 --eval --source gt
  python lane_state.py --play 20261115_205703 --kappa-th 0.12
"""
import argparse
import csv
import json
import re
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
NAMES = {0: "outer", 1: "dashed", 2: "inner"}
CLASS_BGR = {0: (0, 0, 255), 1: (0, 255, 0), 2: (255, 0, 0)}
NAME_RE = re.compile(r"^(?P<play>.+)_r(?P<run>\d+)_k(?P<k>\d+)")


class GroundProjector:
    def __init__(self, cam):
        self.w, self.h = cam["width"], cam["height"]
        self.f = (self.h / 2) / np.tan(np.radians(cam["vertical_fov_deg"]) / 2)
        self.cx, self.cy = self.w / 2, self.h / 2
        self.hgt = cam["height_above_ground_m"]
        self.pitch = np.radians(cam.get("pitch_down_deg", 0.0))
        self.fwd = cam.get("forward_offset_from_rear_axle_m", 0.0)

    def project(self, mask):
        """마스크 픽셀 → 바닥 점 (X 오른쪽, Z 앞) [m], 카메라 바로 아래 기준"""
        v, u = np.nonzero(mask)
        xc = (u + 0.5 - self.cx) / self.f
        yc = (self.cy - (v + 0.5)) / self.f
        # 아래로 pitch만큼 기운 카메라: 광선을 카메라 x축으로 회전
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        dy = yc * cp - sp          # 월드 위쪽 성분
        dz = yc * sp + cp          # 월드 앞쪽 성분
        ok = dy < -1e-6
        t = self.hgt / -dy[ok]
        return xc[ok] * t, dz[ok] * t + self.fwd


def line_curvature(X, Z, zmin, zmax, min_span):
    sel = (Z >= zmin) & (Z <= zmax)
    X, Z = X[sel], Z[sel]
    if len(X) < 20:
        return None, 0.0
    P = np.column_stack([X, Z])
    P = P - P.mean(0)
    _, _, vt = np.linalg.svd(P, full_matrices=False)
    s, d = P @ vt[0], P @ vt[1]
    span = float(s.max() - s.min())
    if span < min_span:
        return None, span
    a, b, _ = np.polyfit(s, d, 2)
    return float(2 * a / (1 + b * b) ** 1.5), span


def decide(states):
    vis = [st for st in states if st != "none"]
    return "PP_PID" if vis and all(st == "curve" for st in vis) else "PID"


def masks_from_result(res):
    """YOLO 결과 → 클래스별 마스크 (같은 클래스가 여럿이면 신뢰도가 가장 높은 것)"""
    masks = {c: None for c in NAMES}
    if res.masks is not None:
        for j in np.argsort(-res.boxes.conf.cpu().numpy()):
            c = int(res.boxes.cls[j])
            if masks[c] is None:
                masks[c] = res.masks.data[j].cpu().numpy() > 0.5
    return masks


def judge(masks, proj, kappa_th=0.05, zmin=1.0, zmax=5.0, min_span=1.0):
    """차선별 (곡률, 길이, 상태)와 전환 판단. lane_state.py와 lane_server.py가 같은 함수를 씀"""
    lines = {}
    for c, name in NAMES.items():
        kap, span = None, 0.0
        if masks.get(c) is not None:
            X, Z = proj.project(masks[c])
            kap, span = line_curvature(X, Z, zmin, zmax, min_span)
        state = "none" if kap is None else ("curve" if abs(kap) > kappa_th else "straight")
        lines[name] = (kap, span, state)
    return lines, decide([s for _, _, s in lines.values()])


def load_ref():
    rows = list(csv.DictReader(open(ROOT / "MATLAB" / "reference_path_outer_lane.csv")))
    x = np.array([float(r["x_m"]) for r in rows]); y = np.array([float(r["y_m"]) for r in rows])
    s = np.array([float(r["s_m"]) for r in rows]); k = np.array([float(r["kappa_1pm"]) for r in rows])
    return x, y, s, k


def ref_kappa_ahead(ref, px, py, zmin, zmax):
    x, y, s, k = ref
    i = int(np.argmin((x - px) ** 2 + (y - py) ** 2))
    L = s[-1] + (s[1] - s[0])
    ds = (s - s[i]) % L
    sel = (ds >= zmin) & (ds <= zmax)
    # 부호를 둔 채 평균한 뒤 절댓값: 경로 곡률의 약 0.35 m 주기 잔물결이 상쇄됨 (|k|를 평균하면 직선도 커짐)
    return float(abs(np.mean(k[sel])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--play", required=True)
    ap.add_argument("--weights", default=None)
    ap.add_argument("--source", choices=["pred", "gt"], default="pred")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--kappa-th", type=float, default=0.05,
                    help="이 곡률 [1/m]보다 크면 곡선. 0.05: 검증 주행에서 곡선 91%%·직선 86%% 맞힘 (2026-11-15)")
    ap.add_argument("--zmin", type=float, default=1.0)
    ap.add_argument("--zmax", type=float, default=5.0)
    ap.add_argument("--min-span", type=float, default=1.0)
    ap.add_argument("--eval", action="store_true")
    ap.add_argument("--ref-th", type=float, default=0.1, help="정답: 앞 구간 평균 |kappa_ref|가 이보다 크면 곡선")
    a = ap.parse_args()

    cam = json.loads((HERE / "camera.json").read_text(encoding="utf-8"))
    proj = GroundProjector(cam)
    play_dir = ROOT / "yolo" / a.play
    images = sorted((play_dir / "images").glob("*.png"))
    model = None
    if a.source == "pred":
        from ultralytics import YOLO
        w = Path(a.weights) if a.weights else sorted(HERE.glob("runs/*/weights/best.pt"), key=lambda p: p.stat().st_mtime)[-1]
        model = YOLO(str(w))

    cap = {}
    cap_path = ROOT / "analysis" / f"{a.play}_capture_log.csv"
    if cap_path.exists():
        for r in csv.DictReader(open(cap_path, encoding="utf-8")):
            cap[Path(r["image"]).name] = r
    ref = load_ref() if a.eval else None

    rows, kappas = [], []
    for img_path in images:
        masks = {c: None for c in NAMES}
        if model is not None:
            res = model.predict(str(img_path), conf=a.conf, retina_masks=True, verbose=False, device=0)[0]
            masks = masks_from_result(res)
        else:
            m = cv2.imread(str(play_dir / "masks" / (img_path.stem + ".png")))
            for c, bgr in CLASS_BGR.items():
                b = np.all(m == bgr, axis=2)
                masks[c] = b if b.sum() >= 12 else None

        mt = NAME_RE.match(img_path.stem)
        c_row = cap.get(img_path.name, {})
        row = dict(play=mt["play"], run=int(mt["run"]), k=int(mt["k"]), t_s=c_row.get("t_s", ""))
        lines, decision = judge(masks, proj, a.kappa_th, a.zmin, a.zmax, a.min_span)
        ks = {}
        for name, (kap, span, state) in lines.items():
            ks[name] = kap
            row[f"{name}_kappa"] = f"{kap:.4f}" if kap is not None else ""
            row[f"{name}_span_m"] = f"{span:.2f}"
            row[f"{name}_state"] = state
        row["n_visible"] = sum(row[f"{n}_state"] != "none" for n in NAMES.values())
        row["decision"] = decision
        if ref is not None and c_row:
            kr = ref_kappa_ahead(ref, float(c_row["x_m"]), float(c_row["y_m"]), a.zmin, a.zmax)
            row["ref_kappa_ahead"] = f"{kr:.4f}"
            row["ref_decision"] = "PP_PID" if kr > a.ref_th else "PID"
        rows.append(row)
        kappas.append((ks, row.get("ref_decision")))

    suffix = "" if a.source == "pred" else "_gt"
    out = ROOT / "analysis" / f"{a.play}_lane_state{suffix}.csv"
    with open(out, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    n = len(rows)
    print(f"{a.play} ({a.source}): {n} images, kappa-th {a.kappa_th}, z {a.zmin}-{a.zmax} m")
    for name in NAMES.values():
        cnt = {s: sum(r[f'{name}_state'] == s for r in rows) for s in ("curve", "straight", "none")}
        print(f"  {name:7s} curve {cnt['curve']:4d}  straight {cnt['straight']:4d}  none {cnt['none']:4d}")
    print(f"  decision: PP_PID {sum(r['decision'] == 'PP_PID' for r in rows)}, PID {sum(r['decision'] == 'PID' for r in rows)}")

    if ref is not None:
        ev = [(ks, rd) for ks, rd in kappas if rd]
        print(f"  정답 (앞 {a.zmin}-{a.zmax} m |평균 kappa_ref| > {a.ref_th}):PP_PID {sum(rd == 'PP_PID' for _, rd in ev)}, PID {sum(rd == 'PID' for _, rd in ev)}")
        print("  kappa-th | 정확도 | 곡선을 PP_PID로 맞힘 | 직선을 PID로 맞힘")
        for th in (0.03, 0.05, 0.07, 0.1, 0.12, 0.15, 0.2, 0.25):
            tp = tn = pc = pn = 0
            for ks, rd in ev:
                st = ["none" if k is None else ("curve" if abs(k) > th else "straight") for k in ks.values()]
                d = decide(st)
                if rd == "PP_PID":
                    pc += 1; tp += d == "PP_PID"
                else:
                    pn += 1; tn += d == "PID"
            mark = "  <- 지금" if abs(th - a.kappa_th) < 1e-9 else ""
            print(f"    {th:5.2f}  | {(tp + tn) / len(ev):.3f}  | {tp}/{pc} ({tp / max(pc, 1):.3f})       | {tn}/{pn} ({tn / max(pn, 1):.3f}){mark}")
    print(f"saved: {out}")


if __name__ == "__main__":
    main()
