"""Unity가 저장한 차선 마스크(yolo/<Play>/masks/*.png)를 YOLOv8-seg 라벨(yolo/<Play>/labels/*.txt)로 바꿉니다.

마스크 색 (RGB) → 클래스
  (255, 0, 0) 빨강 → 0 outer_solid   바깥 실선
  (0, 255, 0) 초록 → 1 center_dashed 중앙 점선
  (0, 0, 255) 파랑 → 2 inner_solid   안쪽 실선
그 외 색(검정 배경, 차체·핸들)은 무시합니다. 색이 정확히 같아야 해당 클래스로 봅니다.

한 사진에서 같은 클래스는 차선 하나(인스턴스 하나)입니다. 점선 조각처럼 여러 덩어리로 나뉘면
Ultralytics의 merge_multi_segment로 한 다각형으로 잇습니다 (YOLO 라벨은 한 줄 = 다각형 하나).
라벨 한 줄: <클래스> x1 y1 x2 y2 ...  (0~1로 정규화한 다각형)

사용:
  python mask_to_yolo.py                       # yolo/ 아래 모든 Play 폴더
  python mask_to_yolo.py 20261115_204754 ...   # 지정한 Play 폴더만
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from ultralytics.data.converter import merge_multi_segment

HERE = Path(__file__).resolve().parent
YOLO_ROOT = HERE.parent / "yolo"

CLASS_BGR = {0: (0, 0, 255), 1: (0, 255, 0), 2: (255, 0, 0)}   # cv2는 BGR
MIN_AREA = 12        # 이보다 작은 덩어리 [px]는 버림 (멀리 있는 점 몇 개)
APPROX_EPS = 1.0     # 다각형 단순화 허용 오차 [px]


def mask_to_lines(mask_bgr):
    h, w = mask_bgr.shape[:2]
    lines = []
    for cls, bgr in CLASS_BGR.items():
        binary = np.all(mask_bgr == bgr, axis=2).astype(np.uint8)
        if binary.sum() < MIN_AREA:
            continue
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        segs = []
        for c in contours:
            if cv2.contourArea(c) < MIN_AREA and len(c) < 6:
                continue
            c = cv2.approxPolyDP(c, APPROX_EPS, True).reshape(-1, 2)
            if len(c) >= 3:
                segs.append(c.astype(np.float64))
        if not segs:
            continue
        poly = np.concatenate(merge_multi_segment(segs), axis=0) if len(segs) > 1 else segs[0]
        poly = poly / np.array([w, h])
        lines.append(f"{cls} " + " ".join(f"{v:.5f}" for v in np.clip(poly, 0, 1).reshape(-1)))
    return lines


def convert_play(play_dir: Path):
    masks = sorted((play_dir / "masks").glob("*.png"))
    labels_dir = play_dir / "labels"
    labels_dir.mkdir(exist_ok=True)
    counts = np.zeros(3, int)
    empty = 0
    for m in masks:
        lines = mask_to_lines(cv2.imread(str(m)))
        (labels_dir / (m.stem + ".txt")).write_text("\n".join(lines) + ("\n" if lines else ""), encoding="ascii")
        for ln in lines:
            counts[int(ln.split()[0])] += 1
        empty += not lines
    print(f"{play_dir.name}: {len(masks)} masks -> labels | outer_solid {counts[0]}, center_dashed {counts[1]}, "
          f"inner_solid {counts[2]}, no lane {empty}")


def main():
    names = sys.argv[1:]
    plays = [YOLO_ROOT / n for n in names] if names else sorted(p for p in YOLO_ROOT.iterdir() if (p / "masks").is_dir())
    for p in plays:
        convert_play(p)


if __name__ == "__main__":
    main()
