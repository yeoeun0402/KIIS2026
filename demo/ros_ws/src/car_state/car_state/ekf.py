"""CTRV(등속·등회전) EKF — SLAM 자세 [x, y, θ] 만으로 v, yaw_rate 추정. 순수 numpy (ROS 없이 시험).

상태 s = [x, y, θ, v, ω]  (기준점 base_link = 뒷바퀴 축 중심 → 자전거 모델과 같은 점, 옆 미끄러짐 0 가정)
  예측  x += v/ω (sin(θ+ωdt) − sin θ),  y += v/ω (cos θ − cos(θ+ωdt)),  θ += ω dt   (ω≈0 이면 직선)
  잡음  가속도 σ_a (m/s²) · 각가속도 σ_α (rad/s²) 를 백색잡음으로
  측정  z = [x, y, θ] (SLAM), R = diag(σ_p², σ_p², σ_θ²)
왜 그냥 미분이 아닌가: 30 Hz 에서 위치 잡음 1 cm 를 차분하면 속도 잡음 ±0.4 m/s — RC카 저속보다 크다.
이상치: 마할라노비스 d² > gate 면 버림, 연속 reset_after 번 버려지면 (재위치 추정 점프) 새로 시작.
"""
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ctrv(s, dt):
    x, y, th, v, w = s
    if abs(w) > 1e-4:
        x2 = x + v / w * (math.sin(th + w * dt) - math.sin(th))
        y2 = y + v / w * (math.cos(th) - math.cos(th + w * dt))
    else:
        x2 = x + v * dt * math.cos(th)
        y2 = y + v * dt * math.sin(th)
    return np.array([x2, y2, wrap(th + w * dt), v, w])


def ctrv_jacobian(s, dt, eps=1e-6):
    J = np.zeros((5, 5))
    f0 = ctrv(s, dt)
    for i in range(5):
        d = np.zeros(5); d[i] = eps
        df = ctrv(s + d, dt) - f0
        df[2] = wrap(df[2])
        J[:, i] = df / eps
    return J


class CtrvEkf:
    H = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 1, 0, 0]], float)

    def __init__(self, sigma_pos=0.02, sigma_theta=math.radians(1.5), sigma_acc=2.0, sigma_yaw_acc=4.0,
                 gate=16.3, reset_after=5, max_dt=0.5):
        self.R = np.diag([sigma_pos ** 2, sigma_pos ** 2, sigma_theta ** 2])
        self.sa, self.sw = sigma_acc, sigma_yaw_acc
        self.gate, self.reset_after, self.max_dt = gate, reset_after, max_dt
        self.s = None
        self.P = None
        self.t = None
        self.n_reject = 0
        self.stats = {"updates": 0, "rejected": 0, "resets": 0}

    @property
    def ready(self):
        return self.s is not None

    def reset(self, t, z):
        self.s = np.array([z[0], z[1], wrap(z[2]), 0.0, 0.0])
        self.P = np.diag([self.R[0, 0], self.R[1, 1], self.R[2, 2], 1.0 ** 2, 1.0 ** 2])
        self.t = t
        self.n_reject = 0

    def predict(self, t):
        """t 까지 예측한 상태 사본 (내부 상태는 안 바꿈) — 제어기가 지연 보상할 때."""
        if self.s is None:
            return None
        return ctrv(self.s, max(0.0, t - self.t))

    def _Q(self, dt):
        th = self.s[2]
        G = np.array([[0.5 * dt * dt * math.cos(th), 0], [0.5 * dt * dt * math.sin(th), 0], [0, 0.5 * dt * dt],
                      [dt, 0], [0, dt]])
        return G @ np.diag([self.sa ** 2, self.sw ** 2]) @ G.T

    def update(self, t, z):
        """측정 z=[x, y, θ] (시각 t, 초). 반환: 'init' | 'ok' | 'rejected' | 'reset' | 'skip'."""
        z = np.asarray(z, float)
        if self.s is None:
            self.reset(t, z)
            return "init"
        dt = t - self.t
        if dt <= 0:
            return "skip"
        if dt > self.max_dt:                          # 너무 오래 끊겼으면 속도를 믿지 않고 새로
            self.reset(t, z)
            self.stats["resets"] += 1
            return "reset"
        F = ctrv_jacobian(self.s, dt)
        s_pred = ctrv(self.s, dt)
        P_pred = F @ self.P @ F.T + self._Q(dt)
        y = z - s_pred[:3]
        y[2] = wrap(y[2])
        S = self.H @ P_pred @ self.H.T + self.R
        d2 = float(y @ np.linalg.solve(S, y))
        if d2 > self.gate:
            self.n_reject += 1
            self.stats["rejected"] += 1
            if self.n_reject >= self.reset_after:
                self.reset(t, z)
                self.stats["resets"] += 1
                return "reset"
            return "rejected"
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.s = s_pred + K @ y
        self.s[2] = wrap(self.s[2])
        self.P = (np.eye(5) - K @ self.H) @ P_pred
        self.t = t
        self.n_reject = 0
        self.stats["updates"] += 1
        return "ok"

    def std(self):
        return np.sqrt(np.clip(np.diag(self.P), 0, None))
