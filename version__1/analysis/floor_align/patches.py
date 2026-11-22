import cv2, numpy as np
D = "/tmp/claude-1000/-home-yeoeun-unity-simulation/387b97e6-3fa1-4f98-bbe1-556dadf272de/scratchpad/floor"
PPM = 150
T = np.load(D + "/tex150.npy"); Dr = np.load(D + "/draw150.npy")
hsv = cv2.cvtColor(T, cv2.COLOR_BGR2HSV)
tw = ((hsv[:, :, 2] > 190) & (hsv[:, :, 1] < 50)).astype(np.float32)
dk = np.load(D + "/draw_lines.npy")
tw = cv2.GaussianBlur(tw, (0, 0), 3); dk = cv2.GaussianBlur(dk, (0, 0), 3)
P, S = int(1.5 * PPM), int(0.45 * PPM)       # 패치 1.5 m, 탐색 ±0.45 m
H, W = tw.shape
out = []
for yc in np.arange(0.9, 10.2, 0.6):
    for xc in np.arange(0.9, 15.2, 0.6):
        u, v = int(xc * PPM), int((11 - yc) * PPM)
        a, b = v - P // 2, u - P // 2
        if a - S < 0 or b - S < 0 or a + P + S > H or b + P + S > W: continue
        tmpl = dk[a:a + P, b:b + P]
        if tmpl.std() < 0.04: continue
        win = tw[a - S:a + P + S, b - S:b + P + S]
        if tw[a:a + P, b:b + P].std() < 0.06: continue
        R = cv2.matchTemplate(win, tmpl, cv2.TM_CCOEFF_NORMED)
        _, mx, _, ml = cv2.minMaxLoc(R)
        R2 = R.copy(); r = int(0.12 * PPM)
        R2[max(0, ml[1] - r):ml[1] + r + 1, max(0, ml[0] - r):ml[0] + r + 1] = -1
        sec = R2.max()
        dx, dy = (ml[0] - S) / PPM, -(ml[1] - S) / PPM      # 도면 → 텍스처 (트랙 좌표 m)
        if mx > 0.35 and mx - sec > 0.07 and abs(dx) < 0.44 and abs(dy) < 0.44:
            out.append((xc, yc, dx, dy, mx, mx - sec))
out = np.array(out)
np.save(D + "/patch_matches.npy", out)
print("matches", len(out))
for r in out: print("(%.1f,%.1f) shift (%+.2f,%+.2f) ncc %.2f margin %.2f" % tuple(r))
