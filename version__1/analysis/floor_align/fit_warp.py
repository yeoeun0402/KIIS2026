import cv2, numpy as np, scipy.sparse as sp
from scipy.sparse.linalg import lsqr
D = "/tmp/claude-1000/-home-yeoeun-unity-simulation/387b97e6-3fa1-4f98-bbe1-556dadf272de/scratchpad/floor"
G = 0.5; NX, NY = int(16 / G) + 1, int(11 / G) + 1; NN = NX * NY
def bil(x, y):
    fx, fy = np.clip(x / G, 0, NX - 1 - 1e-9), np.clip(y / G, 0, NY - 1 - 1e-9)
    i, j = np.floor(fx).astype(int), np.floor(fy).astype(int); ax, ay = fx - i, fy - j
    idx = [j * NX + i, j * NX + i + 1, (j + 1) * NX + i, (j + 1) * NX + i + 1]
    w = [(1 - ax) * (1 - ay), ax * (1 - ay), (1 - ax) * ay, ax * ay]
    return idx, w
rows, cols, vals, rhs, wts, kind = [], [], [], [], [], []
def add(x, y, cx, cy, target, w, k):
    """cx·u_x(P) + cy·u_y(P) = target"""
    idx, ww = bil(np.array([x]), np.array([y]))
    r = len(rhs)
    for ii, wi in zip(idx, ww):
        if cx: rows.append(r); cols.append(int(ii[0])); vals.append(cx * float(wi[0]))
        if cy: rows.append(r); cols.append(NN + int(ii[0])); vals.append(cy * float(wi[0]))
    rhs.append(target); wts.append(w); kind.append(k)
# 1) 특징 패치 (2 성분)
for x, y, dx, dy, ncc, mg in np.load(D + "/patch_matches.npy"):
    w = min(1.0, ncc) * min(1.0, mg / 0.15)
    add(x, y, 1, 0, dx, w, "patch"); add(x, y, 0, 1, dy, w, "patch")
# 2) 도로 경계선 옆 방향 (법선 성분만)
L = np.load(D + "/lat.npz")
cl = np.loadtxt("/home/yeoeun/demo/track/lane2_centerline.csv", delimiter=",", skiprows=1)
cx_, cy_, hd = cl[:, 1], cl[:, 2], np.radians(cl[:, 3]); nx, ny = np.sin(hd), -np.cos(hd)
for line in ("outer", "inner"):
    nt, nd, ok = L[line + "_t"], L[line + "_d"], L[line + "_ok"]
    for i in range(0, len(cl), 4):
        if not ok[i]: continue
        px, py = cx_[i] + nd[i] * nx[i], cy_[i] + nd[i] * ny[i]
        add(px, py, nx[i], ny[i], nt[i] - nd[i], 0.6, "lat")
# 3) 출발선 (도면 중심 5.825 / 텍스처 5.50, 앞뒤 = y)
for x in (0.65, 0.9, 1.15, 1.4, 1.65, 1.9):
    add(x, 5.825, 0, 1, 5.50 - 5.825, 3.0, "start")
# 4) 매트 테두리: 테두리에 수직인 성분 0 (텍스처 가장자리 = 매트 가장자리)
for i in range(NX):
    for y in (0.0, 11.0): add(i * G, y, 0, 1, 0.0, 2.0, "border")
for j in range(NY):
    for x in (0.0, 16.0): add(x, j * G, 1, 0, 0.0, 2.0, "border")
A = sp.csr_matrix((vals, (rows, cols)), shape=(len(rhs), 2 * NN)); b = np.array(rhs); w0 = np.array(wts); kind = np.array(kind)
# 매끄러움 (라플라시안)
lr, lc, lv = [], [], []; r = 0
for comp in (0, NN):
    for j in range(NY):
        for i in range(NX):
            for di, dj in ((1, 0), (0, 1)):
                if 0 < i < NX - 1 or dj == 1:
                    if di == 1 and not (0 < i < NX - 1): continue
                if dj == 1 and not (0 < j < NY - 1): continue
                if di == 1 and not (0 < i < NX - 1): continue
                c = comp + j * NX + i
                lr += [r, r, r]; lc += [c - di - dj * NX, c, c + di + dj * NX]; lv += [1, -2, 1]; r += 1
R = sp.csr_matrix((lv, (lr, lc)), shape=(r, 2 * NN))
import sys
lam = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
w = w0.copy()
for it in range(6):
    Aw = sp.diags(np.sqrt(w)) @ A
    M = sp.vstack([Aw, lam * R]); rr = np.r_[np.sqrt(w) * b, np.zeros(r)]
    u = lsqr(M, rr, atol=1e-10, btol=1e-10, iter_lim=20000)[0]
    res = A @ u - b
    delta = 0.04
    hub = np.where(np.abs(res) <= delta, 1.0, delta / np.maximum(np.abs(res), 1e-12))
    w = w0 * hub
for k in ("patch", "lat", "start", "border"):
    m = kind == k
    print(f"{k:6s} n {m.sum():4d}  |resid| median {np.median(np.abs(res[m]))*100:5.1f} cm  p90 {np.percentile(np.abs(res[m]),90)*100:5.1f} cm  outliers(>10cm) {(np.abs(res[m])>0.10).sum()}")
U = u[:NN].reshape(NY, NX); V = u[NN:].reshape(NY, NX)
print("field |u| max %.2f m, mean %.2f m" % (np.hypot(U, V).max(), np.hypot(U, V).mean()))
np.savez(D + (sys.argv[2] if len(sys.argv) > 2 else "/field.npz"), U=U, V=V, G=G)
