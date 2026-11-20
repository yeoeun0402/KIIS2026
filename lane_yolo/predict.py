"""학습한 YOLOv8-seg 차선 모델을 한 Play(주행)의 사진 전체에 돌리고, 정답 마스크와 비교합니다.

출력
  analysis/<Play>_detections.csv         사진마다 차선 3개 검출 여부·신뢰도 (제어기 전환 분석용, k로 MATLAB과 연결)
  lane_yolo/runs/<학습>/pred_<Play>/      예측을 겹친 확인용 그림 몇 장 (왼쪽 사진, 오른쪽 예측, 아래 정답)
  화면                                     클래스별 검출률(재현율)·오검출·IoU 요약

비교 기준 (정답 = Unity가 만든 masks/*.png)
  - 검출: 정답에 그 차선이 있을 때 예측에도 있으면 맞음. 정답에 없는데 예측에 있으면 오검출
  - IoU: 얇은 선이라 1~2 px만 어긋나도 IoU가 크게 떨어지므로 두 마스크를 3 px 넓혀서 계산
  - 같은 클래스가 여러 개 나오면 신뢰도가 가장 높은 것 하나만 씀 (한 사진에 차선 종류마다 하나)

사용:
  python predict.py --play 20261115_205703
  python predict.py --play 20261115_205703 --weights runs/lane_seg/weights/best.pt --conf 0.25
"""
import argparse
import csv
import re
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
NAMES = {0: "outer_solid", 1: "center_dashed", 2: "inner_solid"}
CLASS_BGR = {0: (0, 0, 255), 1: (0, 255, 0), 2: (255, 0, 0)}
NAME_RE = re.compile(r"^(?P<play>.+)_r(?P<run>\d+)_k(?P<k>\d+)")


def latest_weights():
    cands = sorted(HERE.glob("runs/*/weights/best.pt"), key=lambda p: p.stat().st_mtime)
    if not cands:
        raise SystemExit("학습된 가중치가 없습니다 (lane_yolo/runs/*/weights/best.pt)")
    return cands[-1]


def gt_masks(mask_path, shape):
    m = cv2.imread(str(mask_path))
    if m is None:
        return {c: np.zeros(shape, bool) for c in NAMES}
    return {c: np.all(m == bgr, axis=2) for c, bgr in CLASS_BGR.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--play", required=True)
    ap.add_argument("--weights", default=None)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--samples", type=int, default=12, help="저장할 확인용 그림 수")
    a = ap.parse_args()

    weights = Path(a.weights) if a.weights else latest_weights()
    play_dir = ROOT / "yolo" / a.play
    images = sorted((play_dir / "images").glob("*.png")) + sorted((play_dir / "images").glob("*.jpg"))
    out_dir = weights.parent.parent / f"pred_{a.play}"
    out_dir.mkdir(exist_ok=True)
    sample_idx = set(np.linspace(0, len(images) - 1, min(a.samples, len(images))).astype(int).tolist())
    kernel = np.ones((7, 7), np.uint8)   # 3 px 넓히기

    model = YOLO(str(weights))
    stats = {c: dict(gt=0, hit=0, fp=0, iou=[]) for c in NAMES}
    rows = []

    for i, img_path in enumerate(images):
        res = model.predict(str(img_path), conf=a.conf, retina_masks=True, verbose=False, device=0)[0]
        h, w = res.orig_shape
        pred = {c: None for c in NAMES}
        if res.masks is not None:
            for j in np.argsort(-res.boxes.conf.cpu().numpy()):
                c = int(res.boxes.cls[j])
                if pred[c] is None:
                    pred[c] = (float(res.boxes.conf[j]), res.masks.data[j].cpu().numpy() > 0.5)

        gt = gt_masks(play_dir / "masks" / (img_path.stem + ".png"), (h, w))
        row = {"image": img_path.name}
        mt = NAME_RE.match(img_path.stem)
        row.update(play=mt["play"] if mt else a.play, run=int(mt["run"]) if mt else "", k=int(mt["k"]) if mt else "")
        for c, name in NAMES.items():
            has_gt = gt[c].sum() >= 12
            p = pred[c]
            row[f"{name}_found"] = int(p is not None)
            row[f"{name}_conf"] = f"{p[0]:.3f}" if p else ""
            if has_gt:
                stats[c]["gt"] += 1
                if p:
                    stats[c]["hit"] += 1
                    g = cv2.dilate(gt[c].astype(np.uint8), kernel) > 0
                    q = cv2.dilate(p[1].astype(np.uint8), kernel) > 0
                    stats[c]["iou"].append((g & q).sum() / max((g | q).sum(), 1))
            elif p:
                stats[c]["fp"] += 1
        rows.append(row)

        if i in sample_idx:
            img = cv2.imread(str(img_path))
            ov = img.copy()
            for c, p in pred.items():
                if p:
                    ov[p[1]] = (0.3 * ov[p[1]] + 0.7 * np.array(CLASS_BGR[c])).astype(np.uint8)
                    ys, xs = np.nonzero(p[1])
                    cv2.putText(ov, f"{NAMES[c]} {p[0]:.2f}", (int(xs.mean()), int(ys.min()) + 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, CLASS_BGR[c], 1, cv2.LINE_AA)
            gtv = img.copy()
            for c in NAMES:
                gtv[gt[c]] = CLASS_BGR[c]
            top = np.hstack([img, ov])
            bottom = np.hstack([gtv, np.zeros_like(img)])
            cv2.putText(bottom, "GT (Unity mask)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imwrite(str(out_dir / img_path.name), np.vstack([top, bottom]))

    csv_path = ROOT / "analysis" / f"{a.play}_detections.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    print(f"weights: {weights}")
    print(f"{a.play}: {len(images)} images, conf >= {a.conf}")
    for c, name in NAMES.items():
        s = stats[c]
        rec = s["hit"] / s["gt"] if s["gt"] else float("nan")
        miou = float(np.mean(s["iou"])) if s["iou"] else float("nan")
        print(f"  {name:14s} 정답에 있음 {s['gt']:4d} | 검출 {s['hit']:4d} (재현율 {rec:.3f}) | 오검출 {s['fp']:3d} | IoU(3px) {miou:.3f}")
    all3 = sum(1 for r in rows if all(r[f'{n}_found'] for n in NAMES.values()))
    print(f"  차선 3개 모두 검출된 사진: {all3} / {len(images)}")
    print(f"detections: {csv_path}")
    print(f"overlays:   {out_dir}")


if __name__ == "__main__":
    main()
