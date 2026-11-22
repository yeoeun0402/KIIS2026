"""Unity 바닥 그림(image.png)의 바깥 도로 경계 흰 선을 도면(drawing_ccw.png) 빨간 선 위치로 다시 그림.
실제 매트 실측(2026-11-22, 위 테두리~바깥 선 x≈5 m: 15 cm)이 도면과 일치 → 도면 기준.
나머지 그림(차선 점선, 주차칸 등)은 건드리지 않음. 다시 실행해도 결과는 거의 같음(이미 맞으면 변화 없음).
사용: python3 edge_repaint.py <입력 image.png> <출력 png>"""
import cv2, numpy as np, sys
from scipy.ndimage import map_coordinates, median_filter, gaussian_filter1d
DRAW = "/home/yeoeun/demo/track/tools/drawing_ccw.png"
CL = "/home/yeoeun/demo/track/lane2_centerline.csv"
X0, YT, YB, XR = 338.5, 255.0, 2205.0, 3234.0          # 도면 픽셀 ↔ 16×11 m 매트
PPM = 150
src = cv2.imread(sys.argv[1], cv2.IMREAD_UNCHANGED); H, W = src.shape[:2]
xs = (np.arange(16 * PPM) + 0.5) / PPM; ys = 11 - (np.arange(11 * PPM) + 0.5) / PPM; X, Y = np.meshgrid(xs, ys)
d = cv2.remap(cv2.imread(DRAW), (X / 16 * (XR - X0) + X0).astype(np.float32), (YB - Y / 11 * (YB - YT)).astype(np.float32), cv2.INTER_LINEAR)
h = cv2.cvtColor(d, cv2.COLOR_BGR2HSV)
red = (((h[..., 0] < 8) | (h[..., 0] > 165)) & (h[..., 1] > 90) & (h[..., 2] > 80)).astype(np.float32)
T = cv2.resize(src[..., :3], (16 * PPM, 11 * PPM), interpolation=cv2.INTER_AREA); hs = cv2.cvtColor(T, cv2.COLOR_BGR2HSV)
tw = ((hs[..., 2] > 190) & (hs[..., 1] < 50)).astype(np.float32)
cl = np.loadtxt(CL, delimiter=",", skiprows=1)
cx, cy, hd = cl[:, 1], cl[:, 2], np.radians(cl[:, 3]); nx, ny = np.sin(hd), -np.cos(hd)   # 바깥쪽 법선
ns = np.arange(-2.0, 1.0, 0.005)
def prof(img):
    PX = cx[:, None] + ns[None, :] * nx[:, None]; PY = cy[:, None] + ns[None, :] * ny[:, None]
    return map_coordinates(cv2.GaussianBlur(img, (0, 0), 1.0), [(11 - PY) * PPM - 0.5, PX * PPM - 0.5], order=1, mode="constant")
def centroid(P, lo, hi):
    m = (ns >= lo) & (ns <= hi); sub = P[:, m]; w = np.where(sub > 0.25 * sub.max(1, keepdims=True), sub, 0)
    return (w * ns[m]).sum(1) / np.maximum(w.sum(1), 1e-9)
def smooth(v): return gaussian_filter1d(median_filter(v, size=21, mode="wrap"), 3, mode="wrap")
nd_raw, nt_raw = centroid(prof(red), 0.15, 0.95), centroid(prof(tw), 0.15, 0.95)
nd = smooth(nd_raw)
e = nt_raw - nd_raw; bad = np.abs(e - median_filter(e, size=61, mode="wrap")) > 0.04     # 횡단보도·출발선 등 교차 표시 제외
i = np.arange(len(e)); nt = nd + smooth(np.interp(i, i[~bad], e[~bad], period=len(e)))
print("before: texture - drawing outer edge  median %.1f cm, worst %.1f cm" % (np.median(nt - nd) * 100, (nt - nd).min() * 100))
ns2 = np.arange(0.1, 1.0, 1 / PPM / 2)
P = map_coordinates(tw, [(11 - (cy[:, None] + ns2 * ny[:, None])) * PPM - 0.5, (cx[:, None] + ns2 * nx[:, None]) * PPM - 0.5], order=1)
hw = np.median((P > 0.5).sum(1)) * (ns2[1] - ns2[0]) / 2                     # 선 반폭 (~2.2 cm)
Q = 400
def sdist(off):                                                             # 부호 거리 (도로 안쪽 음수), m
    px = np.c_[(cx + off * nx) * Q, (11 - (cy + off * ny)) * Q]
    m = np.zeros((11 * Q, 16 * Q), np.uint8); cv2.fillPoly(m, [np.round(px * 16).astype(np.int32)], 1, cv2.LINE_8, shift=4)
    return (cv2.distanceTransform(1 - m, cv2.DIST_L2, 5) - cv2.distanceTransform(m, cv2.DIST_L2, 5)) / Q
XX, YY = np.meshgrid((np.arange(W) + 0.5) / W * 16, (np.arange(H) + 0.5) / H * 11)   # YY: 위에서부터 m
dn = map_coordinates(sdist(nd), [YY * Q - 0.5, XX * Q - 0.5], order=1)
do = map_coordinates(sdist(nt), [YY * Q - 0.5, XX * Q - 0.5], order=1)
asphalt = np.array([76, 74, 80], np.float32); white = np.array([253, 253, 253], np.float32)
cur = src[..., :3].astype(np.float32); dist = np.linalg.norm(cur - asphalt, axis=2)
prot = ((XX < 1.2) & (YY > 11 - 6.15) & (YY < 11 - 5.45)) | ((XX > 13.8) & (YY > 11 - 7.25) & (YY < 11 - 5.6))   # 출발선, 횡단보도
inside = (dn < -hw) & (dn > -0.6)
road_fill = inside & ((do > -hw - 0.015) | ((do > -hw - 0.05) & (dist > 4) & ~prot) | ((do > -0.18) & (dist < 25) & ~prot))
grass_fill = (dn > hw) & (do < hw + 0.015) & (dn < 0.6)
bgr = cur.copy(); bgr[road_fill] = asphalt
o8 = np.clip(bgr, 0, 255).astype(np.uint8)
if grass_fill.any(): o8 = cv2.inpaint(o8, cv2.dilate(grass_fill.astype(np.uint8), np.ones((3, 3), np.uint8)), 5, cv2.INPAINT_TELEA)
a = np.clip((hw + 8 / W - np.abs(dn)) / (16 / W), 0, 1)[..., None]           # 1 px 안티에일리어싱
bgr = np.where((np.abs(dn) < 0.6)[..., None], o8 * (1 - a) + white * a, o8)
out = src.copy(); out[..., :3] = np.clip(bgr, 0, 255).astype(np.uint8)
cv2.imwrite(sys.argv[2], out)
print("repainted: road %d px, grass %d px -> %s" % (road_fill.sum(), grass_fill.sum(), sys.argv[2]))
