import math

import numpy as np

CHI2_3_999 = 16.27
H = np.zeros((3, 6))
H[0, 0] = H[1, 1] = H[2, 2] = 1.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class LocKF:
    def __init__(self, sigma_a=0.5, sigma_alpha=1.0, gate=CHI2_3_999, reset_after=10, v0_sigma=0.5, w0_sigma=1.0):
        self.sigma_a, self.sigma_alpha = sigma_a, sigma_alpha
        self.gate, self.reset_after = gate, reset_after
        self.v0_sigma, self.w0_sigma = v0_sigma, w0_sigma
        self.reset()

    def reset(self):
        self.q = None          # 상태 (처음 측정 전 None)
        self.P = None
        self.t = None
        self.n_reject = 0

    @property
    def ready(self):
        return self.q is not None

    def predict(self, t):
        if self.q is None:
            return
        dt = max(0.0, t - self.t)
        F = np.eye(6)
        #F = np.eye(10)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        G = np.zeros((6, 3))
        G[0, 0] = G[1, 1] = G[2, 2] = 0.5 * dt * dt
        G[3, 0] = G[4, 1] = G[5, 2] = dt
        Q = G @ np.diag([self.sigma_a ** 2, self.sigma_a ** 2, self.sigma_alpha ** 2]) @ G.T
        self.q = F @ self.q
        self.q[2] = wrap(self.q[2])
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def _init(self, t, z, R):
        self.q = np.array([z[0], z[1], wrap(z[2]), 0.0, 0.0, 0.0])
        self.P = np.diag([0, 0, 0, self.v0_sigma ** 2, self.v0_sigma ** 2, self.w0_sigma ** 2]).astype(float)
        self.P[:3, :3] = R
        self.t = t
        self.n_reject = 0

    def step(self, t, z=None, R=None):
        """시각 t 까지 예측하고, 측정 z (x, y, θ) · R (3×3) 이 있으면 갱신. 결과 dict."""
        if self.q is None:
            if z is None:
                return None
            self._init(t, z, np.asarray(R, float))
            return self._out(math.nan, 2, np.zeros(3))
        self.predict(t)
        if z is None:
            return self._out(math.nan, -1, np.full(3, math.nan))
        R = np.asarray(R, float)
        nu = np.array([z[0] - self.q[0], z[1] - self.q[1], wrap(z[2] - self.q[2])])
        S = H @ self.P @ H.T + R
        d2 = float(nu @ np.linalg.solve(S, nu))
        if d2 > self.gate:
            self.n_reject += 1
            if self.n_reject >= self.reset_after:         # 계속 안 맞음 = 필터가 길을 잃음 → 측정으로 다시 시작
                self._init(t, z, R)
                return self._out(d2, 2, nu)
            return self._out(d2, 0, nu)
        self.n_reject = 0
        K = self.P @ H.T @ np.linalg.inv(S)
        self.q = self.q + K @ nu
        self.q[2] = wrap(self.q[2])
        IKH = np.eye(6) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T          # Joseph 형식 (수치적으로 대칭·양정치 유지)
        self.P = (self.P + self.P.T) / 2
        return self._out(d2, 1, nu)

    def _out(self, d2, flag, nu):
        x, y, th, vx, vy, w = self.q
        return {"x": np.array([x, y, th, math.hypot(vx, vy), w]), "P": self.P.copy(), "nis": d2, "flag": flag,
                "nu": np.asarray(nu, float)}


def cov6(P):
    """앞 3×3 (x, y, θ) → Pxx,Pxy,Pyy,Ptt,Pxt,Pyt (LOC 줄과 같은 순서)."""
    return P[0, 0], P[0, 1], P[1, 1], P[2, 2], P[0, 2], P[1, 2]


# CSV (live_loc --log-dir, 잡음 시험이 같은 열을 씀)
CSV_COLS = ["run", "epoch", "meas", "x_raw", "y_raw", "th_raw", "n_markers", "rms_px",
            "Rxx", "Rxy", "Ryy", "Rtt", "Rxt", "Ryt",
            "x_kf", "y_kf", "th_kf", "v_kf", "w_kf", "Pxx", "Pxy", "Pyy", "Ptt", "Pxt", "Pyt",
            "nis", "kf_flag", "inn_x", "inn_y", "inn_th"]


def fmt(v):
    """CSV 칸: 실수는 유효숫자 6개, 단 시각(epoch, 1e5 이상)은 소수점 3자리 (6자리면 초가 잘림 — 2026-11-22 버그)."""
    if isinstance(v, float):
        return f"{v:.3f}" if abs(v) >= 1e5 else f"{v:.6g}"
    return v


def csv_row(run, t, z, n, rms, R, out):
    nan3, nan6 = [math.nan] * 3, [math.nan] * 6
    zz = list(z) if z is not None else nan3
    rr = list(cov6(R)) if R is not None else nan6
    if out is None:                                        # 칼만 꺼짐 (--no-kf)
        return [run, t, int(z is not None), *zz, n, rms, *rr, *([math.nan] * 5), *nan6, math.nan, -2, *nan3]
    return [run, t, int(z is not None), *zz, n, rms, *rr, *out["x"], *cov6(out["P"]), out["nis"], out["flag"], *out["nu"]]
