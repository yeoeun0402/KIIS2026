import cv2, numpy as np
from scipy.ndimage import map_coordinates, gaussian_filter1d, median_filter
D = "/tmp/claude-1000/-home-yeoeun-unity-simulation/387b97e6-3fa1-4f98-bbe1-556dadf272de/scratchpad/floor"
PPM = 150
T = np.load(D + "/tex150.npy"); Dr = np.load(D + "/draw150.npy")
hsv = cv2.cvtColor(T, cv2.COLOR_BGR2HSV)
tw = ((hsv[:, :, 2] > 190) & (hsv[:, :, 1] < 50)).astype(float)          # 텍스처 흰 선
dk = np.load(D + "/draw_lines.npy").astype(float)
tw = cv2.GaussianBlur(tw, (0, 0), 1.0); dk = cv2.GaussianBlur(dk, (0, 0), 1.0)
cl = np.loadtxt("/home/yeoeun/demo/track/lane2_centerline.csv", delimiter=",", skiprows=1)
s, cx, cy, hd = cl[:, 0], cl[:, 1], cl[:, 2], np.radians(cl[:, 3])
nx, ny = np.sin(hd), -np.cos(hd)          # 진행 방향 오른쪽 (바깥) 법선
ns = np.arange(-2.0, 1.0, 0.01)
def prof(img):
    X = cx[:, None] + ns[None, :] * nx[:, None]; Y = cy[:, None] + ns[None, :] * ny[:, None]
    return map_coordinates(img, [(11 - Y) * PPM - 0.5, X * PPM - 0.5], order=1, mode="constant")
Pt, Pd = prof(tw), prof(dk)
def peak(P, lo, hi):
    m = (ns >= lo) & (ns <= hi); sub = P[:, m]
    i = sub.argmax(1); return ns[m][i], sub.max(1)
res = {}
for name, (lo, hi) in {"outer": (0.15, 0.85), "inner": (-1.9, -0.95)}.items():
    nt, qt = peak(Pt, lo, hi); nd, qd = peak(Pd, lo, hi)
    ok = (qt > 0.35) & (qd > 0.25)
    res[name] = (nt, nd, ok)
    d = nt - nd
    print(f"{name}: valid {ok.mean():.0%}  drawing n median {np.median(nd[ok]):.3f}  texture n median {np.median(nt[ok]):.3f}  "
          f"diff(tex-draw) median {np.median(d[ok])*100:.1f} cm  p5 {np.percentile(d[ok],5)*100:.1f}  p95 {np.percentile(d[ok],95)*100:.1f} cm")
np.savez(D + "/lat.npz", s=s, **{k + "_t": v[0] for k, v in res.items()}, **{k + "_d": v[1] for k, v in res.items()}, **{k + "_ok": v[2] for k, v in res.items()})
# 구간별
for a in range(0, 40, 4):
    m = (s >= a) & (s < a + 4)
    o = res["outer"]; i_ = res["inner"]
    mo = m & o[2]; mi = m & i_[2]
    print(f"s {a:2d}-{a+4:2d} m: outer diff {np.median((o[0]-o[1])[mo])*100 if mo.any() else np.nan:6.1f} cm  inner diff {np.median((i_[0]-i_[1])[mi])*100 if mi.any() else np.nan:6.1f} cm  (draw outer n {np.median(o[1][mo]) if mo.any() else np.nan:.2f}, inner n {np.median(i_[1][mi]) if mi.any() else np.nan:.2f})")
