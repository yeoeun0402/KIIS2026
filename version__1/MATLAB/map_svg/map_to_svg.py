"""Unity 맵 텍스처(image.png) → 벡터 SVG (색 영역마다 채운 경로)
  1) 원본에서 Lab k-means로 색을 찾고, 면적이 아주 작은 색(안티앨리어싱 테두리의 중간색)은 버리고 비슷한 색은 합침
  2) 3배로 키운 그림의 각 픽셀을 가장 가까운 색에 배정 → 다수결 필터로 잔점 정리
     (3배에서 따면 곡선·글자 윤곽이 원본 픽셀보다 매끄럽게 나옴)
  3) 색마다 윤곽(구멍 포함)을 따서 단순화, 면적 큰 색부터 아래에 쌓음 (1 px 겹쳐 틈 방지)
사용: python -I map_to_svg.py <image.png> <out.svg> [rot180: 0|1]
"""
import sys
import cv2
import numpy as np
from PIL import Image

src, out = sys.argv[1], sys.argv[2]
ROT = len(sys.argv) > 3 and sys.argv[3] == '1'
S = 3                     # 확대 배율
K = 16
MIN_AREA = 0.0035         # 이보다 작은 색은 중간색으로 보고 버림 (전체 대비)
MERGE_DE = 8.0            # 이보다 가까운 색은 합침 (Lab 거리)
EPS = 1.0                 # 윤곽 단순화 [확대 픽셀]

rgb = np.array(Image.open(src).convert('RGB'))
if ROT:
    rgb = np.ascontiguousarray(rgb[::-1, ::-1])
H, W = rgb.shape[:2]

# 1) 색 찾기
lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).reshape(-1, 3).astype(np.float32)
cv2.setRNGSeed(1)
_, lb, cen = cv2.kmeans(lab, K, None, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.2), 4,
                         cv2.KMEANS_PP_CENTERS)
area = np.bincount(lb.ravel(), minlength=K) / lb.size
keep = [k for k in np.argsort(-area) if area[k] >= MIN_AREA]
pal, pw = [], []
for k in keep:                                    # 비슷한 색 합치기 (면적 가중 평균)
    for i, c in enumerate(pal):
        if np.linalg.norm(c - cen[k]) < MERGE_DE:
            pal[i] = (c * pw[i] + cen[k] * area[k]) / (pw[i] + area[k]); pw[i] += area[k]
            break
    else:
        pal.append(cen[k].copy()); pw.append(area[k])
pal = np.array(pal, np.float32)
P = len(pal)

# 2) 3배 그림을 가장 가까운 색에 배정 + 다수결
big = cv2.resize(rgb, (W * S, H * S), interpolation=cv2.INTER_CUBIC)
blab = cv2.cvtColor(big, cv2.COLOR_RGB2LAB).astype(np.float32)
best = np.full(blab.shape[:2], np.inf, np.float32); lbl = np.zeros(blab.shape[:2], np.uint8)
for i, c in enumerate(pal):
    d = ((blab - c) ** 2).sum(axis=2)
    m = d < best; best[m] = d[m]; lbl[m] = i
del blab, best
votes = np.stack([cv2.boxFilter((lbl == i).astype(np.float32), -1, (5, 5)) for i in range(P)])
lbl = votes.argmax(axis=0).astype(np.uint8); del votes

cols = [np.median(big[lbl == i], axis=0).astype(int) for i in range(P)]   # 실제 RGB 대표값
areas = np.array([(lbl == i).mean() for i in range(P)])
order = np.argsort(-areas)

def hexc(c):
    return f'#{c[0]:02x}{c[1]:02x}{c[2]:02x}'

parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">',
         '<title>KIIS2026 map (vectorized from Unity texture image.png)</title>',
         f'<rect width="{W}" height="{H}" fill="{hexc(cols[order[0]])}"/>']
kern = np.ones((3, 3), np.uint8)
for i in order[1:]:
    m = cv2.dilate((lbl == i).astype(np.uint8) * 255, kern)
    cnts, _ = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    d = []
    for c in cnts:
        if cv2.contourArea(c) < 4 * S * S:
            continue
        a = cv2.approxPolyDP(c, EPS, True).reshape(-1, 2) / S
        if len(a) >= 3:
            d.append('M' + ' L'.join(f'{x:.2f} {y:.2f}' for x, y in a) + 'Z')
    if d:
        parts.append(f'<path fill="{hexc(cols[i])}" fill-rule="evenodd" d="{" ".join(d)}"/>')
parts.append('</svg>')
open(out, 'w', encoding='utf-8').write('\n'.join(parts))

small = np.zeros((H * S, W * S, 3), np.uint8)
for i in range(P):
    small[lbl == i] = cols[i]
small = cv2.resize(small, (W, H), interpolation=cv2.INTER_AREA)
err = np.abs(small.astype(int) - rgb.astype(int)).mean()
print(f'colors {P}, svg {len(open(out, encoding="utf-8").read()) / 1e6:.2f} MB, mean abs color error {err:.1f}/255')
print('  ' + ', '.join(f'{hexc(cols[i])} {100 * areas[i]:.1f}%' for i in order))
