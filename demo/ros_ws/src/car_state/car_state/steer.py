"""조향 POT ↔ 유효 조향각 δ. 순수 numpy (ROS 없이 시험).

차량 조향 = DC 모터 + POT (펌웨어 L/R = 120 ms 펄스 한 번, P <adc> 50 ms 마다) → 위치 명령이 없고 POT 가 유일한 측정치.

1. 스윕 분석 analyze_sweep  (바퀴 띄우고 A/D 반복)
   POT 범위(기계적 끝) · 펄스당 ADC (좌/우) · L 이 POT 를 올리는지 내리는지 · 반응 지연 · 정지 잡음
2. 원호 분석 find_segments → fit_table  (저속으로 조향 고정 원호 주행, SLAM 자세 필요)
   POT 가 일정하고 움직이는 구간마다 곡률 κ = dθ/ds (θ 를 진행 거리 s 에 직선 맞춤) → δ = atan(L·κ)
   각도기로 바퀴를 재는 대신 '자전거 모델의 유효 조향각'을 직접 잼 (애커만·링키지·유격 포함)
   같은 POT 를 왼쪽/오른쪽에서 다가간 구간이 둘 다 있으면 히스테리시스(유격)도 나옴
3. SteerCalib: steer_calib.yaml 읽어 pot → δ 보간
"""
import math

import numpy as np

SIGN_LEFT = +1   # δ 좌 + (ROS 관례, x 앞 · y 왼쪽)


# ---------------------------------------------------------------- 보정 파일
class SteerCalib:
    def __init__(self, pot, delta_rad, backlash_adc=0.0, meta=None):
        self.backlash = float(backlash_adc or 0.0)
        pot = np.asarray(pot, float)
        d = np.asarray(delta_rad, float)
        o = np.argsort(pot)
        self.pot, self.delta = pot[o], d[o]
        if len(self.pot) < 2:
            raise ValueError("보정 점이 2개 이상 필요")
        dd = np.diff(self.delta)
        if not (np.all(dd > 0) or np.all(dd < 0)):
            raise ValueError(f"pot→δ 가 단조가 아님: {list(zip(self.pot, np.degrees(self.delta).round(2)))}")
        self.meta = meta or {}

    @classmethod
    def load(cls, path):
        import yaml
        y = yaml.safe_load(open(path))
        return cls(y["table"]["pot"], np.radians(y["table"]["delta_deg"]), y.get("backlash_adc", 0.0), y)

    def __call__(self, pot, direction=0):
        """(δ rad, 범위 안인지). direction = POT 이 마지막으로 움직인 방향(+1 증가 / −1 감소 / 0 모름) → 유격 보정."""
        p = float(pot) - direction * self.backlash / 2
        ok = self.pot[0] - 5 <= p <= self.pot[-1] + 5
        return float(np.interp(p, self.pot, self.delta)), ok

    def pot_for(self, delta_rad):
        """역변환 δ → pot (폐루프 조향 목표용)."""
        o = np.argsort(self.delta)
        return float(np.interp(delta_rad, self.delta[o], self.pot[o]))


class PotDirection:
    """POT 이 마지막으로 의미 있게 움직인 방향 (잡음 thr 이상). 유격 보정용."""

    def __init__(self, thr=3.0):
        self.thr, self.anchor, self.dir = thr, None, 0

    def update(self, pot):
        p = float(pot)
        if self.anchor is None:
            self.anchor = p
        elif p - self.anchor > self.thr or (self.dir > 0 and p > self.anchor):
            self.dir, self.anchor = 1, p
        elif self.anchor - p > self.thr or (self.dir < 0 and p < self.anchor):
            self.dir, self.anchor = -1, p
        return self.dir


# ---------------------------------------------------------------- 1. 스윕
def analyze_sweep(pot_t, pot, cmd_t, cmd, settle=0.4, isolate=0.4):
    """pot_t/pot: POT 시계열 (s, adc). cmd_t/cmd: 'L'|'R'|'F' 명령 시각.
    펄스 하나만 있는(앞뒤 isolate 초 안에 다른 L/R 없는) 펄스에서 펄스당 변화량·지연을 잰다."""
    pot_t, pot = np.asarray(pot_t, float), np.asarray(pot, float)
    lr = [(t, c) for t, c in zip(cmd_t, cmd) if c in ("L", "R")]
    out = {"pot_samples": int(len(pot)), "pulses": {"L": sum(c == "L" for _, c in lr), "R": sum(c == "R" for _, c in lr)}}
    if len(pot) < 10:
        return out
    out["pot_min"], out["pot_max"] = int(np.percentile(pot, 0.5)), int(np.percentile(pot, 99.5))
    # 정지 잡음: 명령 없는 1 s+ 구간
    quiet = np.ones(len(pot), bool)
    for t, _ in lr:
        quiet &= ~((pot_t > t - 0.2) & (pot_t < t + 1.0))
    q = pot[quiet]
    noise = float(np.std(q - np.round(np.median(q)))) if len(q) > 20 else float("nan")
    # 구간별로 잡음을 다시: 정지 구간 각각의 표준편차 중앙값
    runs, cur = [], []
    for i, f in enumerate(quiet):
        if f:
            cur.append(pot[i])
        elif cur:
            runs.append(cur); cur = []
    if cur:
        runs.append(cur)
    stds = [np.std(r) for r in runs if len(r) >= 10]
    noise = float(np.median(stds)) if stds else noise
    out["noise_adc"] = round(noise, 2)
    thr = max(2.0, 3 * noise)
    per, lat = {"L": [], "R": []}, {"L": [], "R": []}
    times = np.array([t for t, _ in lr])
    for t, c in lr:
        others = times[(np.abs(times - t) < isolate) & (times != t)]
        if len(others):
            continue
        before = pot[(pot_t >= t - 0.25) & (pot_t < t)]
        after = pot[(pot_t >= t + settle) & (pot_t < t + settle + 0.25)]
        if len(before) < 2 or len(after) < 2:
            continue
        b, a = float(np.median(before)), float(np.median(after))
        if min(b, a) <= out["pot_min"] + 2 or max(b, a) >= out["pot_max"] - 2:
            continue                                  # 끝에 닿아 있으면 펄스 크기 아님
        per[c].append(a - b)
        moved = np.where((pot_t >= t) & (np.abs(pot - b) > thr))[0]
        if len(moved) and abs(a - b) > thr:
            lat[c].append(pot_t[moved[0]] - t)
    for c in ("L", "R"):
        if per[c]:
            out[f"adc_per_pulse_{c}"] = round(float(np.median(per[c])), 1)
            out[f"adc_per_pulse_{c}_n"] = len(per[c])
        if lat[c]:
            out[f"response_ms_{c}"] = round(1000 * float(np.median(lat[c])), 0)
    if per["L"]:
        out["left_increases_pot"] = bool(np.median(per["L"]) > 0)
    out["note"] = "POT 는 50 ms 마다 → 지연 해상도 50 ms. 범위 = 기계적 끝(끝까지 돌렸을 때만 의미)"
    return out


# ---------------------------------------------------------------- 2. 원호
def heading_unwrap(theta):
    return np.unwrap(np.asarray(theta, float))


def signed_arclength(x, y, theta):
    """진행 방향으로 투영한 누적 거리 (후진이면 감소)."""
    dx, dy = np.diff(x), np.diff(y)
    thm = theta[:-1] + 0.5 * np.diff(heading_unwrap(theta))
    ds = dx * np.cos(thm) + dy * np.sin(thm)
    return np.concatenate([[0.0], np.cumsum(ds)])


def fit_circle(x, y):
    """Kasa 원 맞춤 → (cx, cy, r, rms)."""
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(max(c + cx ** 2 + cy ** 2, 0.0))
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return float(cx), float(cy), r, rms


def find_segments(t, x, y, theta, pot_t, pot, cmd_t=(), cmd=(), wheelbase=None,
                  pot_tol=4.0, v_min=0.08, min_dur=1.0, min_len=0.4, trim=0.3):
    """조향 고정 + 움직이는 구간 → [{pot, kappa, delta_deg, ...}].
    t, x, y, theta: 자세 (base_link, 평면). pot_t/pot: POT. cmd: L/R (다가간 방향 표시용)."""
    t, x, y, theta = (np.asarray(a, float) for a in (t, x, y, theta))
    if len(t) < 10 or len(pot) < 2:
        return []
    th = heading_unwrap(theta)
    p = np.interp(t, np.asarray(pot_t, float), np.asarray(pot, float))
    s = signed_arclength(x, y, th)
    # 속도 (0.3 s 창 중앙차분, 구간 판정용)
    k = max(1, int(round(0.15 / max(np.median(np.diff(t)), 1e-3))))
    v = np.zeros(len(t))
    v[k:-k] = (s[2 * k:] - s[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    moving = np.abs(v) >= v_min
    pot_t = np.asarray(pot_t, float)
    pd = PotDirection()
    pot_dir = np.array([pd.update(v) for v in pot])
    lr_t = np.array([a for a, c in zip(cmd_t, cmd) if c in ("L", "R")])
    lr_c = [c for c in cmd if c in ("L", "R")]
    segs, i, n = [], 0, len(t)
    while i < n:
        if not moving[i]:
            i += 1
            continue
        j = i
        ref = p[i]
        while j + 1 < n and moving[j + 1] and abs(p[j + 1] - ref) <= pot_tol:
            j += 1
            if j - i < 5:
                ref = float(np.median(p[i:j + 1]))
        # 앞뒤 trim 초 잘라 가감속·조향 변화 직후 제외
        a = np.searchsorted(t, t[i] + trim)
        b = np.searchsorted(t, t[j] - trim, side="right") - 1
        if b - a >= 8 and t[b] - t[a] >= min_dur and abs(s[b] - s[a]) >= min_len:
            ss, tt = s[a:b + 1], th[a:b + 1]
            A = np.c_[ss, np.ones(len(ss))]
            (kappa, c0), *_ = np.linalg.lstsq(A, tt, rcond=None)
            res = tt - (kappa * ss + c0)
            seg = {"t0": round(float(t[a]), 2), "t1": round(float(t[b]), 2), "pot": round(float(np.median(p[a:b + 1])), 1),
                   "pot_spread": round(float(np.ptp(p[a:b + 1])), 1), "length_m": round(float(abs(ss[-1] - ss[0])), 3),
                   "v_mean": round(float((ss[-1] - ss[0]) / (t[b] - t[a])), 3), "kappa": float(kappa),
                   "heading_change_deg": round(float(np.degrees(tt[-1] - tt[0])), 1),
                   "theta_rms_deg": round(float(np.degrees(np.sqrt(np.mean(res ** 2)))), 2), "n": int(b - a + 1)}
            if abs(seg["heading_change_deg"]) > 15:
                cx, cy, r, rms = fit_circle(x[a:b + 1], y[a:b + 1])
                seg["circle_r"], seg["circle_rms_cm"] = round(r, 3), round(rms * 100, 1)
            # 다가간 방향: 구간 시작 전 마지막 L/R · POT 이 마지막으로 움직인 방향
            prev = np.where(lr_t < t[i])[0]
            seg["approach"] = lr_c[prev[-1]] if len(prev) else "?"
            seg["pot_dir"] = int(pot_dir[min(np.searchsorted(pot_t, t[i]), len(pot_dir) - 1)])
            if wheelbase:
                seg["delta_deg"] = round(math.degrees(math.atan(wheelbase * kappa)), 2)
            segs.append(seg)
        i = j + 1
    return segs


def _group(P, tol):
    o = np.argsort(P)
    groups, cur = [], [o[0]]
    for a, b in zip(o[:-1], o[1:]):
        if P[b] - P[cur[0]] <= tol:
            cur.append(b)
        else:
            groups.append(cur); cur = [b]
    groups.append(cur)
    return groups


def fit_table(segs, wheelbase, pot_tol=6.0, sweep=None, backlash_adc=None):
    """구간들 → 보정 표 {pot, delta_deg} (유격 보정된 POT 기준) + 다항 맞춤 · 유격 · 직진 POT.
    유격 b: 같은 POT 를 양쪽에서 다가간 묶음에서 δ(+방향) − δ(−방향) = −기울기·b → b. 그다음 각 구간 POT 를
    pot_eff = pot − dir·b/2 로 옮겨 표를 만든다 (실행 중에도 SteerCalib 가 같은 보정)."""
    if not segs:
        raise ValueError("쓸 만한 원호 구간 없음")
    P0 = np.array([s["pot"] for s in segs], float)
    D = np.array([math.degrees(math.atan(wheelbase * s["kappa"])) for s in segs])
    W = np.array([s["length_m"] for s in segs])                 # 긴 구간일수록 믿음
    DIR = np.array([s.get("pot_dir", 0) for s in segs])
    out = {"wheelbase": wheelbase, "n_segments": len(segs)}
    # 유격 추정
    hy = []
    if len(segs) >= 3:
        c0 = np.polyfit(P0, D, min(2, len(set(P0.round())) - 1), w=W)
        for g in _group(P0, pot_tol):
            dp = [i for i in g if DIR[i] > 0]
            dm = [i for i in g if DIR[i] < 0]
            if dp and dm:
                slope = np.polyval(np.polyder(c0), np.mean(P0[g]))
                diff = np.average(D[dp], weights=W[dp]) - np.average(D[dm], weights=W[dm])
                if abs(slope) > 1e-6:
                    hy.append({"pot": round(float(np.mean(P0[g])), 1), "delta_diff_deg": round(float(diff), 2),
                               "adc": round(float(-diff / slope), 1)})
    out["hysteresis"] = hy
    if backlash_adc is None:
        backlash_adc = float(np.median([h["adc"] for h in hy])) if hy else 0.0
    out["backlash_adc"] = round(max(backlash_adc, 0.0), 1)
    P = P0 - DIR * out["backlash_adc"] / 2
    groups = _group(P, pot_tol)
    tp = [float(np.average(P[g], weights=W[g])) for g in groups]
    td = [float(np.average(D[g], weights=W[g])) for g in groups]
    out["points"] = len(tp)
    out["group_spread_deg"] = [round(float(np.ptp(D[g])), 2) if len(g) > 1 else 0.0 for g in groups]
    c = np.polyfit(P, D, min(2, len(tp) - 1), w=W) if len(tp) >= 2 else None
    if c is not None:
        out["poly_deg"] = [float(v) for v in c]
        out["poly_rms_deg"] = round(float(np.sqrt(np.average((np.polyval(c, P) - D) ** 2, weights=W))), 2)
        roots = [r.real for r in np.roots(c) if abs(r.imag) < 1e-9 and min(P) - 100 <= r.real <= max(P) + 100]
        if roots:
            out["pot_center"] = round(float(min(roots, key=lambda r: abs(r - np.median(P)))), 1)
        out["deg_per_adc_center"] = round(float(np.polyval(np.polyder(c), out.get("pot_center", np.median(P)))), 4)
    table_p, table_d, extrap = list(tp), list(td), []
    if sweep and len(tp) >= 2 and "pot_min" in sweep:
        # 측정 범위 밖(스윕 끝까지)은 끝 두 점의 기울기로 직선 연장 — 다항 연장은 끝에서 크게 틀림
        o1 = np.argsort(tp); sp, sd = np.array(tp)[o1], np.array(td)[o1]
        for pe in (sweep["pot_min"], sweep["pot_max"]):
            if pe < sp[0] - 3:
                table_d.append(float(sd[0] + (pe - sp[0]) * (sd[1] - sd[0]) / (sp[1] - sp[0])))
            elif pe > sp[-1] + 3:
                table_d.append(float(sd[-1] + (pe - sp[-1]) * (sd[-1] - sd[-2]) / (sp[-1] - sp[-2])))
            else:
                continue
            table_p.append(float(pe)); extrap.append(float(pe))
    o2 = np.argsort(table_p)
    out["table"] = {"pot": [round(table_p[i], 1) for i in o2], "delta_deg": [round(table_d[i], 3) for i in o2]}
    out["extrapolated_pot"] = extrap
    if min(td) < 0 < max(td):
        out["max_left_deg"], out["max_right_deg"] = round(max(td), 2), round(min(td), 2)
    return out
