import cv2, numpy as np
from scipy.ndimage import map_coordinates
D = "/tmp/claude-1000/-home-yeoeun-unity-simulation/387b97e6-3fa1-4f98-bbe1-556dadf272de/scratchpad/floor"
F = np.load(D + "/field.npz"); U, V, G = F["U"], F["V"], float(F["G"])
src = cv2.imread("/home/yeoeun/version__1/KIIS2026_AutoCar/Assets/image.png", cv2.IMREAD_UNCHANGED)
H0, W0 = src.shape[:2]
def field(x, y):
    fx, fy = x / G, y / G
    return map_coordinates(U, [fy, fx], order=1, mode="nearest"), map_coordinates(V, [fy, fx], order=1, mode="nearest")
def warp(img, W, H):
    xs = (np.arange(W) + 0.5) / W * 16; ys = 11 - (np.arange(H) + 0.5) / H * 11
    X, Y = np.meshgrid(xs, ys)
    du, dv = field(X, Y)
    Xs, Ys = X + du, Y + dv                                   # 도면 위치 P → 원래 텍스처에서 같은 그림이 있는 곳
    mx = (Xs / 16 * img.shape[1] - 0.5).astype(np.float32); my = ((11 - Ys) / 11 * img.shape[0] - 0.5).astype(np.float32)
    return cv2.remap(img, mx, my, cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
out = warp(src, W0, H0)
cv2.imwrite(D + "/image_warped.png", out)
# 검사용 150 px/m
PPM = 150
tw150 = cv2.resize(out[:, :, :3], (16 * PPM, 11 * PPM), interpolation=cv2.INTER_CUBIC)
np.save(D + "/tex150_warped.npy", tw150)
Dr = np.load(D + "/draw150.npy"); nw = np.load(D + "/draw_lines.npy")
for nm, T in (("before", np.load(D + "/tex150.npy")), ("after", tw150)):
    ov = T.copy(); ov[nw > 0.35] = (0, 0, 255)
    cv2.imwrite(D + f"/overlay_{nm}.png", ov)
    cv2.imwrite(D + f"/overlay_{nm}_small.png", cv2.resize(ov, (1200, 825)))
print("saved", out.shape)
