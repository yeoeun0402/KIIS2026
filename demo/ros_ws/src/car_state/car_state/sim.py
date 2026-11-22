"""가짜 차량 (시험용): DC 모터 조향 + POT + 자전거 모델 + SLAM 잡음. 순수 numpy.

조향 모델 (펌웨어와 같은 명령)
  L/R 펄스 → 120 ms 동안 모터 회전 (지연 dead_s 뒤 시작), POT 이 adc_per_pulse 만큼 이동, 끝(pot_min/max)에서 멈춤
  바퀴 각 = true_delta(pot ± 유격/2)  — 모터 쪽 POT 와 바퀴 사이 유격 → 같은 POT 라도 다가간 방향에 따라 δ 다름
  true_delta: 비선형 (링키지) — 보정이 이걸 되찾아야 함
"""
import math

import numpy as np


class SimCar:
    def __init__(self, wheelbase=0.26, pot_center=505.0, pot_min=330, pot_max=690, adc_per_pulse=22.0,
                 left_increases_pot=False, dead_s=0.04, pulse_s=0.12, backlash_adc=6.0, max_delta_deg=24.0,
                 nonlin=0.15, pot_noise=0.8, seed=0):
        self.L = wheelbase
        self.c, self.lo, self.hi = pot_center, pot_min, pot_max
        self.rate = adc_per_pulse / pulse_s            # ADC/s 모터 돌 때
        self.left_sign = 1 if left_increases_pot else -1
        self.dead, self.pulse = dead_s, pulse_s
        self.backlash = backlash_adc
        self.max_d = math.radians(max_delta_deg)
        self.nonlin = nonlin
        self.pot_noise = pot_noise
        self.rng = np.random.default_rng(seed)
        self.pot = pot_center
        self.last_dir = 0          # +1 pot 증가 방향으로 마지막 이동
        self.motor = []            # (t_start, t_end, dir)
        self.x = self.y = self.th = 0.0
        self.v = 0.0

    def true_delta(self, pot_eff):
        """POT → 바퀴 δ (rad, 좌 +). 비선형: 끝으로 갈수록 덜 꺾임."""
        u = (pot_eff - self.c) / ((self.hi - self.lo) / 2)          # -1..1
        u = float(np.clip(u, -1.2, 1.2))
        d = self.max_d * (u - self.nonlin * u ** 3) / (1 - self.nonlin)
        return self.left_sign * d          # POT 이 '좌' 방향으로 움직이면 δ +

    def delta(self):
        return self.true_delta(self.pot - self.last_dir * self.backlash / 2)

    def command(self, t, c):
        if c in ("L", "R"):
            d = self.left_sign if c == "L" else -self.left_sign
            self.motor.append((t + self.dead, t + self.dead + self.pulse, d))
        elif c == "F":
            self.motor = [(a, min(b, t), d) for a, b, d in self.motor if a < t]

    def step(self, t, dt):
        mv = 0.0
        for a, b, d in self.motor:
            ov = max(0.0, min(b, t + dt) - max(a, t))
            mv += d * ov * self.rate
        self.motor = [m for m in self.motor if m[1] > t]
        if mv:
            self.pot = float(np.clip(self.pot + mv, self.lo, self.hi))
            self.last_dir = 1 if mv > 0 else -1
        # 자전거 모델 (기준 = 뒷축)
        d = self.delta()
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.th += self.v / self.L * math.tan(d) * dt

    def read_pot(self):
        return int(round(self.pot + self.rng.normal(0, self.pot_noise)))


def run_script(car, script, dt=0.005, cam_hz=30.0, pot_hz=20.0, sigma_pos=0.01, sigma_th_deg=0.7, t_end=None):
    """script: [(t, 'L'|'R'|'F'|('v', m/s))] → dict(pose_t, x, y, th, pot_t, pot, cmd_t, cmd, truth)."""
    script = sorted(script, key=lambda e: e[0])
    t_end = t_end or (script[-1][0] + 2.0)
    out = {k: [] for k in ("pose_t", "x", "y", "th", "pot_t", "pot", "cmd_t", "cmd", "v_true", "w_true", "delta_true")}
    k, t, next_cam, next_pot = 0, 0.0, 0.0, 0.0
    rng = car.rng
    while t < t_end:
        while k < len(script) and script[k][0] <= t:
            _, c = script[k]
            if isinstance(c, tuple):
                car.v = c[1]
            else:
                car.command(t, c)
                out["cmd_t"].append(t); out["cmd"].append(c)
            k += 1
        car.step(t, dt)
        t += dt
        if t >= next_cam:
            out["pose_t"].append(t)
            out["x"].append(car.x + rng.normal(0, sigma_pos))
            out["y"].append(car.y + rng.normal(0, sigma_pos))
            out["th"].append(math.atan2(math.sin(car.th), math.cos(car.th)) + rng.normal(0, math.radians(sigma_th_deg)))
            out["v_true"].append(car.v)
            out["w_true"].append(car.v / car.L * math.tan(car.delta()))
            out["delta_true"].append(car.delta())
            next_cam += 1.0 / cam_hz
        if t >= next_pot:
            out["pot_t"].append(t); out["pot"].append(car.read_pot())
            next_pot += 1.0 / pot_hz
    return {k: np.array(v) if k != "cmd" else v for k, v in out.items()}


def calib_script(levels=(-2, 2, -4, 4, -6, 6, -8, 8), arc_s=4.0, v=0.35, both_sides_at=(2, 4), center_pulses=8):
    """현장 순서 그대로: 바퀴 띄우고 스윕 → 각 조향 단계로 맞추고 원호 → 일부 단계는 반대쪽에서 다가가 한 번 더.
    levels: 펄스 수(+ = L). 반환 script, 끝 시각."""
    sc, t = [], 0.5
    # 스윕 (차는 서 있음): 좌 끝까지, 우 끝까지, 중앙으로 — 펄스 사이 0.6 s (스윕 분석이 펄스 하나씩 재도록)
    # 끝까지 좌 → 끝까지 우 → 중앙으로 (현장에선 눈으로 바퀴가 곧을 때까지; 시뮬은 펄스 수로)
    for c, n in (("L", 20), ("R", 40), ("L", center_pulses)):
        for _ in range(n):
            sc.append((t, c)); t += 0.6
    t += 1.0
    pos = 0

    def go(target, via=None):
        nonlocal t, pos
        if via is not None:
            go(via)
        while pos != target:
            c = "L" if target > pos else "R"
            sc.append((t, c)); t += 0.3
            pos += 1 if c == "L" else -1
        t += 0.5

    for lv in (0,) + tuple(levels):
        go(lv)
        sc.append((t, ("v", v))); t += arc_s
        sc.append((t, ("v", 0.0))); t += 1.0
        if abs(lv) in both_sides_at:
            go(lv, via=lv + (2 if lv > 0 else -2))        # 반대쪽에서 다가가기
            sc.append((t, ("v", v))); t += arc_s
            sc.append((t, ("v", 0.0))); t += 1.0
    go(0)
    return sc, t + 1.0
