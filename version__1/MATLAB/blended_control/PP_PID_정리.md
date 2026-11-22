# PP_PID 정리 (blended_control)

- 기준 논문: Y. Chen et al., "Optimization of Pure Pursuit Controller based on PID Controller and Low-pass Filter", ITSC 2018 (PP_PI)
- 이 문서의 PP_PID는 논문의 PP_PI에 D 항(K_d (e[k] − e[k−1]) / T_s)을 더한 것입니다.
- 줄 번호는 2026-11-09 코드 기준입니다(파일: `MATLAB/blended_control/`).

---

## 1. 사용한 이산 모델

### 1.1 상태와 입력
| 구분 | 기호 | 뜻 | 단위 | 코드 |
|---|---|---|---|---|
| 상태 (선형 부분) | s | 차가 실제로 달린 거리 (주행 거리계) | m | `st.s` |
| | v | 속도 | m/s | `st.v` |
| | δ | 조향각 (왼쪽 +) | rad | `st.delta` |
| 상태 (위치·방향) | x, y | 뒤 차축 중앙 위치 | m | `st.x`, `st.y` |
| | θ | 차 방향 (+x축에서 반시계 +) | rad | `st.yaw` |
| 입력 | a | 가속도 | m/s² | `uk(1)` |
| | ω | 조향 속도 | rad/s | `uk(2)` |
| 상수 | T_s | 한 스텝 시간 = 0.1 s | s | `Ts`, `T` |
| | L | 축간거리 = 0.59 m | m | `p.L` |

### 1.2 이산 상태공간 모델: 조향 = 인테그레이터, 전진 = 더블 인테그레이터
입력이 한 스텝 동안 일정하다고 보고(영차 유지, ZOH) 연속 시간 식 ṡ = v, v̇ = a, δ̇ = ω를 정확히 적분한 식입니다.

$$\begin{bmatrix} s \\ v \\ \delta \end{bmatrix}[k+1] =
\underbrace{\begin{bmatrix} 1 & T_s & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}}_{A}
\begin{bmatrix} s \\ v \\ \delta \end{bmatrix}[k] +
\underbrace{\begin{bmatrix} T_s^2/2 & 0 \\ T_s & 0 \\ 0 & T_s \end{bmatrix}}_{B}
\begin{bmatrix} a \\ \omega \end{bmatrix}[k]$$

| 부분 | 식 | 이름 |
|---|---|---|
| 전진 | s[k+1] = s[k] + T_s v[k] + T_s²/2 · a[k],  v[k+1] = v[k] + T_s a[k] | 더블 인테그레이터 (a → v → s) |
| 조향 | δ[k+1] = δ[k] + T_s ω[k] | 인테그레이터 (ω → δ) |

```matlab
% path_model_matrices.m 6–11행
A = [1 T 0;
     0 1 0;
     0 0 1];
B = [T^2 / 2  0;
     T        0;
     0        T];
```

### 1.3 입력 한계 (실제 차와 같은 제한)
선형식에 넣기 전에 입력을 먼저 자릅니다. 그래서 s, v, δ는 항상 위 식을 정확히 따릅니다.

```matlab
% path_model_step.m 15–20행
a     = min(max(uk(1), -p.brake), p.accel);                               % -2.0 ~ +1.0 m/s²
a     = min(max(a, -v / T), (p.vMax - v) / T);                            % v[k+1]이 0 ~ 1.5 m/s 안
omega = min(max(uk(2), -p.steerRate), p.steerRate);                       % ±90°/s
omega = min(max(omega, (-p.deltaMax - delta) / T), (p.deltaMax - delta) / T);   % δ[k+1]이 ±30° 안
used  = [a; omega];
xk1   = A * xk(:) + B * used;                                             % x[k+1] = A x[k] + B u[k]
```

### 1.4 위치·방향: 기구학 자전거 모델 (차가 경로에서 벗어날 수 있음)
1.2절에서 나온 이번 스텝 이동 거리 Δs = s[k+1] − s[k] = T_s v[k] + T_s²/2 a[k]만큼 차 방향으로 움직입니다.

$$x[k+1] = x[k] + \Delta s\cos\theta[k],\qquad
y[k+1] = y[k] + \Delta s\sin\theta[k],\qquad
\theta[k+1] = \theta[k] + \frac{\Delta s\,\tan\delta[k+1]}{L}$$

```matlab
% bicycle_discrete_step.m 23–33행
[xk1, used] = path_model_step([st.s; st.v; st.delta], uk, p, T);   % 1.2·1.3절
ds = xk1(1) - st.s;                                                 % Δs = T v + T^2/2 a
st.s = xk1(1); st.v = xk1(2); st.delta = xk1(3);
st.yawRate = st.v * tan(st.delta) / p.L;
st.x   = st.x + ds * cos(st.yaw);
st.y   = st.y + ds * sin(st.yaw);
yaw    = st.yaw + ds * tan(st.delta) / p.L;
st.yaw = atan2(sin(yaw), cos(yaw));                                 % -π ~ π
st.t   = st.t + T;
```
- `bicycle_step.m`(기존 이산화 기구학 자전거 모델)과 같은 순서입니다. 액추에이터를 먼저 갱신하고, 바뀐 δ로 방향을 갱신합니다.
- a = 0이면 두 함수의 결과가 같습니다(`test_blended` 3번).

### 1.5 제어기가 보는 출력 (경로와 비교)
`path_error(ref, x, y, θ)`가 차를 레퍼런스 경로에 수직 투영합니다.

| 출력 | 뜻 |
|---|---|
| s_proj | 투영점까지의 경로 거리 |
| e_lat | 횡오차 (경로 왼쪽 +) |
| e_head | 방향 오차 θ − ψ_path |
| κ | 투영점의 경로 곡률 |

---

## 2. 이산 시간에 맞게 update 되는 값

### 2.1 한 스텝(k → k+1)에서 바뀌는 값
| 값 | 갱신 식 | 위치 |
|---|---|---|
| a[k] (자동 버전) | a[k] = (v_tar − v[k]) / T_s (한계는 1.3절에서 자름) | `blended_discrete.m` 235행 |
| δ_cmd[k] | 3장의 제어 식 (PP / PID / PP_PID) | 241, 243행 |
| ω[k] | ω[k] = (δ_cmd[k] − δ[k]) / T_s | 245행 |
| s, v, δ [k+1] | x[k+1] = A x[k] + B u[k] (한계 포함) | `path_model_step.m` 20행 |
| x, y, θ [k+1] | 기구학 자전거 (Δs) | `bicycle_discrete_step.m` 29–32행 |
| t | t[k+1] = t[k] + T_s | 33행 |
| Σe (PP_PID의 I 기억) | Σe[k] = Σe[k−1] + e[k] | `blended_steer.m` 54행 |
| e[k−1] (D 기억) | 이번 e[k]를 다음 스텝의 e[k−1]로 저장 | 63행 |
| 필터 기억 (이전 출력) | 이번 δ_cmd[k]를 맨 앞에 넣고 가장 오래된 값을 버림 | `lpf_window.m` 21행 |
| 경로 탐색 힌트 | 이번 최근접 경로점 번호 (다음 스텝은 그 근처만 찾음) | `blended_steer.m` 35행 |
| 비교 PID의 I 기억 | I[k] = I[k−1] + e[k] T_s | `pid_steer.m` 18행 |
| 슬라이더 값 | 바뀌었으면 이번 스텝 시작에 opt·게인에 반영 | `blended_discrete.m` 220–226행 |

### 2.2 조향 인테그레이터가 한 스텝 늦게 명령을 따라가는 방식
- ω[k] = (δ_cmd[k] − δ[k]) / T_s를 넣으면 δ[k+1] = δ[k] + T_s ω[k] = **δ_cmd[k]**가 됩니다.
- 즉 제어기가 낸 조향각은 **다음 스텝의 실제 조향각**이 됩니다.
- 단, 한 스텝 변화가 90°/s × 0.1 s = 9°를 넘거나 δ가 ±30°를 넘으면 그 한계까지만 움직입니다.

```matlab
% blended_discrete.m 238–246행 (세 제어기 모두 같은 순서)
for c = 1:nC
    if c == 2
        [dCmd, mem{c}, d] = pid_steer(car(c), ref, p, G{c}, mem{c}, Ts);       % PID
    else
        [dCmd, mem{c}, d] = blended_steer(car(c), ref, p, G{c}, mem{c}, Ts);   % PP 또는 PP_PID
    end
    omega = (dCmd - car(c).delta) / Ts;                                       % ω[k]
    [car(c), used] = bicycle_discrete_step(car(c), [aReq; omega], p, Ts);     % x[k] → x[k+1]
end
```

### 2.3 기록되는 시점
- `timeseries.csv`의 k행은 **시각 kT_s의 상태**와 **그 스텝에서 계산한 값**입니다.
  - 상태: x, y, θ, v, δ
  - 계산값: δ_ref, e, e_la, δ_pp, δ_p, δ_i, δ_d, δ_raw, δ_cmd, ω
- k → k+1로 바뀐 상태는 다음 행에 들어갑니다.

---

## 3. PP, PID, PP_PID 구현 핵심 식

### 3.1 공통: 오차 정의 (`blended_steer.m` 34–39행)
| 기호 | 식 | 코드 |
|---|---|---|
| e (횡오차) | e = 0 − e_lat (목표 − 현재). 차가 경로 오른쪽이면 e > 0 → 왼쪽으로 꺾음 | `eLat = 0 - e.e_lat;` |
| Δψ (방향 오차) | Δψ = ψ_path − θ | `dPsi = 0 - e.e_head;` |
| δ_ref (목표 조향각, 그래프 ①) | δ_ref = atan(L κ) | `dRef = atan(L * e.kappa);` |

### 3.2 PP (Pure Pursuit, 논문 Eq.3)
$$\delta_{pp} = \tan^{-1}\!\left(\frac{2L\sin\alpha}{L_{ad}}\right)$$
- 선행점: 차를 경로에 투영한 점에서 경로를 따라 L_f 앞의 점
- α: 차 방향과 (차 → 선행점) 방향 사이의 각
- L_ad: 차와 선행점 사이의 거리

```matlab
% blended_steer.m 42–47행
ql = path_pose(ref, e.s + g.Lf);                 % 선행점 (경로를 따라 s_proj + Lf)
dx = ql.x - car.x; dy = ql.y - car.y;
alpha = atan2(dy, dx) - car.yaw;
alpha = atan2(sin(alpha), cos(alpha));
Lad   = hypot(dx, dy);
dPP   = atan(2 * L * sin(alpha) / Lad);          % δ_pp
```
- **PP 제어기:** K_pp = 1, K_p = K_i = K_d = 0, 필터 없음. 그래서 δ_cmd = δ_pp입니다.

### 3.3 PID (비교용, `pid_steer.m`, `PID/PID_control.m`과 같은 식)
$$\delta = K_p\,e[k] + K_i \sum_{j\le k} e[j]\,T_s + K_d\,\frac{e[k]-e[k-1]}{T_s}$$

```matlab
% pid_steer.m 18–22행
mem.I = mem.I + eLat * T;                                       % Σ e T_s
if isnan(mem.ePrev), dE = 0; else, dE = (eLat - mem.ePrev) / T; end
mem.ePrev = eLat;
dP = g.Kp * eLat; dI = g.Ki * mem.I; dD = g.Kd * dE;
dRaw = dP + dI + dD;                                            % 경로 정보(곡률·선행점) 없이 오차만 사용
```
- 게인: K_p 3.0, K_i 0.8, K_d 2.6 (`pidKp`, `pidKi`, `pidKd`). 필터 없음.

### 3.4 PP_PID (blended 방식) — 핵심
$$\boxed{\;\delta_{cmd}[k] = \mathrm{Lowpass}\Big(K_{pp}\,\delta_{pp}[k] + \underbrace{K_p\,e_{la}[k]}_{\delta_p} + \underbrace{K_i\sum_{j\le k} e[j]}_{\delta_i} + \underbrace{K_d\,\frac{e[k]-e[k-1]}{T_s}}_{\delta_d}\Big),\qquad K_{pp}+K_p+K_i+K_d = 1\;}$$

$$e_{la} = e + (a + L_{ad})\sin\Delta\psi$$
- e_la: 차 앞쪽 (a + L_ad) 지점의 횡오차입니다. 횡오차에 방향 오차를 더한 값입니다(논문 Eq.5).

| 항 | 식 | 출처 | 코드 (`blended_steer.m`) |
|---|---|---|---|
| δ_pp | atan(2L sin α / L_ad) | 논문 Eq.3 | 47행 `dPP = atan(2 * L * sin(alpha) / Lad);` |
| e_la | e + (a + L_ad) sin Δψ | 논문 Eq.5 | 50행 `ela = eLat + (g.a + Lad) * sin(dPsi);` |
| δ_p | K_p e_la | 논문 Eq.4, 6 | 51행 `dP = g.Kp * ela;` |
| δ_i | K_i Σe (스텝마다 더함, T_s 안 곱함) | 논문 Eq.7 | 54–55행 `mem.Se = mem.Se + eLat; dI = g.Ki * mem.Se;` |
| δ_d | K_d (e[k] − e[k−1]) / T_s | 추가한 D 항 | 58–64행 `dE = (eLat - mem.ePrev) / T; ... dD = g.Kd * dE;` |
| **blending (합)** | **K_pp δ_pp + δ_p + δ_i + δ_d** | 논문 Eq.10 + D | **67행 `dRaw = g.Kpp * dPP + dP + dI + dD;`** |
| Low pass | 5장 | 논문 Eq.8–9 | 68행 `[dCmd, mem.hist] = lpf_window(mem.hist, dRaw, g.ell, g.wcur);` |
| 게인 합 = 1 | K_pp = 1 − K_p − K_i − K_d | 논문 (D까지 넓힘) | `blended_params.m` 62행, 실행 중 `blended_discrete.m` 223행, 570행 |

```matlab
% blended_steer.m 50–68행 (PP_PID의 핵심)
ela  = eLat + (g.a + Lad) * sin(dPsi);       % e_la (Eq.5)
dP   = g.Kp * ela;                            % δ_p  (Eq.4)
mem.Se = mem.Se + eLat;                       % Σ e
dI   = g.Ki * mem.Se;                         % δ_i  (Eq.7)
if isnan(mem.ePrev), dE = 0; else, dE = (eLat - mem.ePrev) / T; end
mem.ePrev = eLat;
dD   = g.Kd * dE;                             % δ_d  (D)
dRaw = g.Kpp * dPP + dP + dI + dD;            % ★ blending: 매 스텝 네 항을 모두 계산해 가중합
[dCmd, mem.hist] = lpf_window(mem.hist, dRaw, g.ell, g.wcur);   % ★ Low pass (Eq.8–9)
```

**blended 방식인 이유:**
- 매 스텝 PP 항과 P·I·D 항을 **모두 계산해서 더합니다**. 상황에 따라 PP와 PID 중 하나를 고르는 조건문(switching)은 없습니다.
- 각 항이 얼마나 들어갈지는 게인(가중치) K_pp, K_p, K_i, K_d가 정합니다.
  - 논문 규칙에 따라 가중치의 합은 1입니다.
  - 그래서 K_p·K_i·K_d를 키우면 PP의 몫 K_pp가 줄어듭니다.
- 기록으로 확인한 결과(기본값 한 바퀴):
  - δ_raw = K_pp δ_pp + (δ_p + δ_i + δ_d)가 매 스텝 성립합니다(차이 최대 5.6e−17 rad).
  - 두 항이 동시에 쓰인 스텝은 99.6%입니다.
- `blended_discrete.m` 434–442행 `makeGains`는 같은 함수로 세 제어기를 만듭니다.

| 제어기 | K_pp | K_p, K_i, K_d | 필터 |
|---|---|---|---|
| PP | 1 | 0 | 없음 |
| PP_PID | 1 − K_p − K_i − K_d | 슬라이더 값 | ℓ, w_current |
| PID | – | `pid_steer` 사용 | 없음 |

---

## 4. 코드 흐름 (입력 → 출력 → 다시 입력)

### 4.1 전체 흐름도 (한 스텝 k)
```
 [시작 전] blended_params ──▶ opt (v, Lf, a, Kp, Ki, Kd, Kpp, ℓ, w_cur, ...) ──▶ makeGains ──▶ G{1..3}
           출발 상태 x[0] = 경로 위 투영점, θ = 경로 방향, v = 0, δ = atan(Lκ)

 ┌──────────────────────────── 스텝 k (반복) ─────────────────────────────────────────────┐
 │ (0) 슬라이더가 바뀌었으면 opt 갱신 → Kpp = 1 − Kp − Ki − Kd → G 다시 만들기              │
 │                                                                                         │
 │ (1) 가속도:  v[k], v_tar ─▶ a[k] = (v_tar − v[k]) / T_s                                 │
 │                                                                                         │
 │ (2) 제어기 (blended_steer):                                                             │
 │     차 상태 x[k], y[k], θ[k], δ[k] ─▶ path_error ─▶ e_lat, e_head, s_proj, κ            │
 │        │                                   │                                           │
 │        │      s_proj + Lf ─▶ path_pose ─▶ 선행점 (x_l, y_l) ─▶ α, L_ad ─▶ δ_pp          │
 │        │                                                                               │
 │        ├─ e = −e_lat, Δψ = −e_head ─▶ e_la = e + (a + L_ad) sin Δψ ─▶ δ_p = K_p e_la   │
 │        ├─ e, [기억 Σe]      ─▶ Σe ← Σe + e        ─▶ δ_i = K_i Σe     ─▶ [기억 Σe]     │
 │        ├─ e, [기억 e[k−1]]  ─▶ (e − e[k−1]) / T_s ─▶ δ_d = K_d (…)    ─▶ [기억 e[k−1]=e]│
 │        │                                                                               │
 │        └─▶ δ_raw = K_pp δ_pp + δ_p + δ_i + δ_d                     (blending)          │
 │              [기억: 이전 출력 ℓ−1개] ─▶ lpf_window ─▶ δ_cmd[k] ─▶ [기억에 δ_cmd 추가]   │
 │                                                                                         │
 │ (3) 조향 인테그레이터 입력:  δ_cmd[k], δ[k] ─▶ ω[k] = (δ_cmd[k] − δ[k]) / T_s            │
 │                                                                                         │
 │ (4) 이산 모델 (bicycle_discrete_step):                                                  │
 │     [s, v, δ][k], [a, ω][k] ─▶ 한계로 자르기 ─▶ A x + B u ─▶ [s, v, δ][k+1], Δs         │
 │     x, y, θ [k], Δs, δ[k+1] ─▶ 기구학 자전거 ─▶ x, y, θ [k+1]                          │
 │                                                                                         │
 │ (5) 출력: Unity로 PP_PID 차 상태 전송 / 그림 갱신 / 기록                                 │
 └──────────────── x[k+1]이 다음 스텝 (1), (2)의 입력이 됨 ────────────────────────────────┘
```

### 4.2 함수별 입출력과 출력이 다시 들어가는 곳
| 함수 (파일) | 입력 | 출력 | 출력이 들어가는 곳 |
|---|---|---|---|
| `blended_params` | 기본값, 덮어쓸 이름-값 | opt (v, Lf, a, Kp, Ki, Kd, Kpp, ℓ, w_cur, …) | `makeGains` → 게인 G |
| `path_error` | 차 x, y, θ, 경로 | e_lat, e_head, s_proj, κ | e, Δψ, δ_ref 계산 / PP 선행점 위치 |
| `path_pose` | s_proj + L_f | 선행점 x_l, y_l | α, L_ad → δ_pp. L_ad는 e_la에도 쓰임 |
| P (`blended_steer` 50–51행) | e, Δψ, L_ad, a, K_p | δ_p | δ_raw |
| I (54–55행) | e, 기억 Σe, K_i | δ_i, 새 Σe | δ_raw / 다음 스텝 I |
| D (58–64행) | e, 기억 e[k−1], K_d, T_s | δ_d, 새 e[k−1] | δ_raw / 다음 스텝 D |
| blending (67행) | δ_pp, δ_p, δ_i, δ_d, K_pp | δ_raw | Low pass |
| `lpf_window` | δ_raw, 이전 출력 ℓ−1개, ℓ, w_cur | δ_cmd, 새 이전 출력 | ω 계산 / 다음 스텝 필터 |
| ω 계산 (245행) | δ_cmd, 현재 δ | ω | 이산 모델 입력 |
| `path_model_step` | [s, v, δ], [a, ω] | [s, v, δ][k+1], 실제로 쓴 [a, ω] | Δs, 다음 스텝 상태 |
| 기구학 (`bicycle_discrete_step`) | x, y, θ, Δs, δ[k+1] | x, y, θ [k+1] | 다음 스텝 `path_error` 입력 |
| `unity_link('state')` | PP_PID 차 상태, 오차, 선행점 | TCP 한 줄 | Unity 차 이동 |

### 4.3 되먹임 고리 (무엇이 무엇으로 돌아가는가)
- **위치 되먹임:** x, y, θ [k+1] → 다음 스텝 `path_error` → e, Δψ → δ_pp, δ_p, δ_i, δ_d
- **조향 되먹임:** δ[k+1] → 다음 스텝 ω = (δ_cmd − δ) / T_s
- **기억 (스텝 사이에 남는 값):**
  - Σe (I)
  - e[k−1] (D)
  - 필터의 이전 출력 ℓ−1개
  - 경로 탐색 힌트
- **세 차는 같은 가속도 a[k]를 씁니다.** 조향만 각자 계산하므로 같은 시각끼리 비교할 수 있습니다.

---

## 5. Low-pass Filter 식과 적용 순간

### 5.1 식 (논문 Eq.8–9)
$$\delta_{cmd}[k] = w_{current}\,\delta_{raw}[k] + w_{previous}\sum_{j=1}^{\ell-1}\delta_{cmd}[k-j],\qquad w_{previous} = \frac{1-w_{current}}{\ell-1}$$
- δ_raw[k]: 이번 스텝에 계산한 조향각, K_pp δ_pp + δ_p + δ_i + δ_d (논문의 δ_current)
- δ_cmd[k − j]: 직전 스텝들의 **최종 출력**(필터를 거친 조향각, 논문의 δ_previous)
- 가중치 합 = w_current + (ℓ − 1) w_previous = 1 → 일정한 입력은 그대로 통과합니다(DC 이득 1).

```matlab
% lpf_window.m 11–21행
ell = round(ell);
if ell <= 1
    y = u;                                            % ℓ = 1: 필터 없음
else
    wprev = (1 - wcur) / (ell - 1);                   % Eq.9
    y = wcur * u + wprev * sum(hist(1:ell - 1));      % Eq.8
end
hist = [y; hist(1:end - 1)];                          % 이번 출력을 기억 맨 앞에 넣음
```

### 5.2 기본값으로 풀어 쓰면 (ℓ = 3, w_current = 0.5 → w_previous = 0.25)
$$\delta_{cmd}[k] = 0.5\,\delta_{raw}[k] + 0.25\,\delta_{cmd}[k-1] + 0.25\,\delta_{cmd}[k-2]$$
- 0에서 1로 바뀐 입력에 대한 출력:

  | k | 1 | 2 | 3 | 4 | 5 |
  |---|---|---|---|---|---|
  | 출력 | 0.50 | 0.625 | 0.78 | 0.85 | 0.91 |

- 즉 약 0.3 s 동안 서서히 새 값으로 갑니다.
- ℓ = 2이면 δ_cmd[k] = w δ_raw[k] + (1 − w) δ_cmd[k−1]로, 1차 저역통과 필터와 같습니다.

### 5.3 적용 순간
```
 δ_pp, δ_p, δ_i, δ_d ──▶ 합 δ_raw ──▶ [Low-pass Filter] ──▶ δ_cmd ──▶ ω = (δ_cmd − δ)/T_s ──▶ 조향 인테그레이터 ──▶ δ[k+1]
                                        ▲ 여기 (매 스텝 한 번)
```
- **언제:** 매 스텝 한 번, 네 항을 더한 **직후**, 조향 속도 ω로 바꾸기 **직전**입니다(`blended_steer.m` 68행).
- **무엇에:** 최종 조향 명령에만 적용합니다. 각 항(δ_pp, δ_p, δ_i, δ_d)이나 오차에는 적용하지 않습니다.
- **어느 제어기에:** 기본으로 PP_PID에만 적용합니다. PP, PID는 필터 없이 비교합니다(`lpfAll = false`, 논문과 같음).
- **처음 값:** 이전 출력 기억은 출발 조향각 δ[0]으로 채웁니다. 그래서 출발할 때 조향이 튀지 않습니다.
- **기록:** `bl_delta_raw_rad`가 필터 전, `bl_delta_cmd_rad`가 필터 후입니다.
- **효과 (측정, 필터만 넣은 PP):**
  - 궤적 오차 RMS: 필터 없음 0.0118 m → ℓ 3·w 0.5에서 0.0099 m
  - w를 너무 작게 하면 지연 때문에 다시 나빠집니다(w 0.3 → 0.0124 m).

---

## 6. 파라미터 조절 슬라이더 (그림 왼쪽 아래)
- 슬라이더를 움직이거나 입력칸에 값을 쓰고 Enter를 누르면 **다음 스텝부터** 적용됩니다.
- 스텝마다 쓴 값은 `timeseries.csv`의 `p_*` 열에 남습니다.
- 아래 측정값은 한 바퀴(0.8 m/s, T_s 0.1 s) 궤적 오차 RMS입니다.

| 슬라이더 | 기호·단위 | 기본 (범위) | 들어가는 식 | 영향을 받는 제어기 | 키우면 / 줄이면 |
|---|---|---|---|---|---|
| **v 목표 속도** | v_tar [m/s] | 0.8 (0.1 ~ 1.5) | a[k] = (v_tar − v[k]) / T_s (가속 1.0, 감속 2.0 m/s² 한계) | 세 차 모두 (자동 버전만. 키보드 버전은 ↑/↓ 키) | 빨라지면 같은 선행점에서 오차가 커지고 한 바퀴가 빨리 끝남 |
| **Lf 선행점 거리** | L_f [m] | 0.8 (0.3 ~ 2.5) | 선행점 = 경로(s_proj + L_f) → α, L_ad → δ_pp. L_ad는 e_la에도 쓰임 | PP, PP_PID | 키우면 조향이 부드럽지만 곡선에서 더 질러감(PP 0.8 m: 0.0118 → 1.2 m: 0.0284 m). 줄이면 경로에 붙지만 흔들림 |
| **a (e_la 식)** | a [m] | 0.59 (0 ~ 1.5) | e_la = e + (a + L_ad) sin Δψ | PP_PID의 P 항 | 키우면 방향 오차를 더 크게 봄. 기본 K_p(0.01)에서는 영향이 거의 없음. K_p가 클 때는 작을수록 좋았음(K_pp 1·K_p 2: a 0 → 0.0078, 0.59 → 0.0093 m) |
| **w_current (LPF)** | w_current | 0.5 (0.05 ~ 1) | 5.1절. 이번 값 가중치. 이전 출력 하나는 (1 − w)/(ℓ − 1) | PP_PID (lpfAll이면 셋 다) | 1이면 필터 없음. 줄이면 더 부드럽지만 늦게 반응(w 0.7: 0.0108, 0.5: 0.0099, 0.3: 0.0124 m, 필터만 넣은 PP) |
| **ell LPF 창 길이** | ℓ (정수) | 3 (1 ~ 10) | 5.1절. 섞는 이전 출력 수 = ℓ − 1 | PP_PID (lpfAll이면 셋 다) | 1이면 필터 없음. 키우면 더 오래된 값까지 섞어 부드럽지만 늦어짐(ℓ 2: 0.0102, 3: 0.0099, 5: 0.0107 m) |
| **Kp P 게인** | K_p [rad/m] | 0.01 (0 ~ 1) | δ_p = K_p e_la | PP_PID | 앞쪽 지점의 횡오차에 바로 반응. 합 = 1 규칙 때문에 키우면 K_pp가 줄어 이 차에서는 나빠짐(0.05: 0.0128, 0.2: 약 0.024 m) |
| **Ki I 게인** | K_i [rad/m] | 0.006 (0 ~ 0.05) | δ_i = K_i Σe (스텝마다 더함) | PP_PID | 오래 남는 치우침(곡선에서 질러가기)을 줄임. 너무 크면 출렁임. 키우면 K_pp도 줄어듦 |
| **Kd D 게인** | K_d [rad·s/m] | 0.001 (0 ~ 0.05) | δ_d = K_d (e[k] − e[k−1]) / T_s | PP_PID | 오차가 커지는 중이면 미리 더 꺾음. 이 차는 오차 변화율이 작아(0.013 m/s) 합 = 1 규칙 아래에서는 효과가 거의 없음(0.05: 0.0133 m로 나빠짐) |
| **Kpp PP 게인** | K_pp | 0.983 (잠김) | K_pp = 1 − K_p − K_i − K_d (`sumToOne`) | PP_PID의 δ_pp 가중치 | 직접 못 바꿈. K_p, K_i, K_d를 바꾸면 자동으로 바뀌어 표시됨. 음수가 되는 값은 받지 않음 |

**슬라이더가 없는 파라미터** (`blended_params.m`이나 실행할 때 이름-값으로 바꿈)
| 이름 | 기본 | 뜻 |
|---|---|---|
| `Ts` | 0.1 s | 한 스텝 시간 (모델의 A, B와 D 항의 T_s) |
| `sumToOne` | true | 게인 합 = 1 규칙. false면 K_pp를 따로 정함 |
| `pidKp`, `pidKi`, `pidKd` | 3.0, 0.8, 2.6 | 비교용 PID 게인 |
| `lpfAll` | false | true면 PP, PID에도 같은 Low-pass Filter |

**맵 아래 체크박스 (보기만 바꿈, 계산에는 영향 없음)**
- 경로 (R), PP (1), PID (2), PP_PID (3), 선행점 (L), 목표 조향각 (Y), 현재 조향각 (D), 조향각 오차 (E)
