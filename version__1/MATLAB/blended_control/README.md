# blended_control — PP_PID (Pure Pursuit + PID + Low-pass Filter)

이 폴더는 교수님이 주신 논문의 PP_PI에 D 항을 더한 경로 추종 제어기입니다.
> Y. Chen, Y. Shan, L. Chen, K. Huang, D. Cao, **"Optimization of Pure Pursuit Controller based on PID Controller and Low-pass Filter"**, ITSC 2018 (이하 "논문")

- **논문의 PP_PI:** Pure Pursuit + PI + Low-pass Filter
- **이 코드의 PP_PID:** 여기에 **D 항 δ_d = K_d (e[k] − e[k−1]) / T_s**를 더했습니다. K_d = 0이면 논문의 PP_PI와 같습니다.
- **차량 모델:** 이산 기구학 자전거 모델입니다. 조향은 인테그레이터, 전진은 더블 인테그레이터입니다.
- **비교:** 논문 4장처럼 PP, PID와 같은 차·같은 경로에서 나란히 비교합니다.

| 파일 | 역할 | 출처 |
|---|---|---|
| `blended_steer.m` | **PP_PID 한 스텝** (논문 Eq.3–10 + D) | 새 파일 |
| `lpf_window.m` | 논문의 Low-pass Filter (Eq.8–9: 창 길이 ℓ, 가중치 w_current) | 새 파일 |
| `pid_steer.m` | 비교용 PID (`PID/PID_control.m`과 같은 횡오차 PID) | 새 파일 |
| `blended_params.m` | 튜닝 파라미터 기본값과 K_pp + K_p + K_i + K_d = 1 규칙 | 새 파일 |
| `blended_discrete.m` | **실행 파일**: 세 제어기를 나란히 계산 → Unity 전송 → 그림 → 기록. 표시 체크박스, 파라미터 슬라이더 포함 | 새 파일 |
| `bicycle_discrete_step.m` | 차량 모델: 액추에이터 = `path_model_step`, 움직임 = `bicycle_step`의 기구학 식 | 새 파일 |
| `bicycle_step.m`, `path_model_matrices.m`, `path_model_step.m`, `path_pose.m` | 이산 기구학 자전거 모델과 인테그레이터 / 더블 인테그레이터 모델 | `MATLAB/`, `MATLAB/test1/` **복제** (원본과 같음) |
| `check_live_tuning.m` | 실행 중 파라미터 변경이 그대로 반영되는지 기록으로 확인 | 새 파일 |
| `plot_blend_components.m` | K_pp δ_pp와 δ_p + δ_i + δ_d를 그려 **합(blending)인지 확인** | 새 파일 |
| `test_blended.m` | 테스트 16개 (1번 = Unity 연동 한 바퀴) | 새 파일 |
| `PP_PID_정리.md` | 이산 모델, 갱신 값, 핵심 식, 코드 흐름, Low pass, 슬라이더 의미를 한 문서로 정리 | 새 문서 |

- 공용 함수는 상위 `MATLAB/` 폴더의 것을 그대로 씁니다: `vehicle_params`, `load_reference_path`, `path_error`, `unity_link`, `draw_map`.
- 결과는 `MATLAB/results/<날짜_시간>_blended_discrete_<keyboard|auto>/`에 저장됩니다.

**실행** (`cd MATLAB; setup_paths` 뒤, Unity Play 먼저)
```matlab
blended_discrete                                        % 키보드 버전: Unity Game 창에서 ↑를 누른 채 Space로 출발
blended_discrete('input', 'auto')                       % 자동 버전: 0.8 m/s로 한 바퀴 (약 52 s)
blended_discrete('input', 'auto', 'useUnity', false)    % Unity 없이: MATLAB 그림만 실제 속도로 움직임
blended_discrete('input', 'auto', 'useUnity', false, 'realtime', false)   % 기다리지 않고 빨리 (약 5 s, 분석용)
blended_discrete('input', 'auto', 'Kp', 0.02, 'Ki', 0.004, 'Kd', 0.002, 'Lf', 1.0, 'ell', 4)   % 튜닝 값으로 실행 (7장)
blended_discrete('input', 'auto', 'Kd', 0)              % D 없이 = 논문의 PP_PI
blended_discrete('input', 'auto', 'sumToOne', false, 'Kpp', 1, 'Kp', 2, 'Ki', 0.02, 'Kd', 1, 'a', 0)   % 합 = 1 규칙 없이 (4.4절)
test_blended                                            % 테스트 16개: 1번에서 Unity 차가 한 바퀴 끝까지 감
test_blended('unity', false)                            % Unity 없이 (1번 건너뜀, 15 / 15)
check_live_tuning                                       % 실행 중 파라미터 변경 확인 (7장)
plot_blend_components                                   % 가장 최근 결과로 항별 기여 그림 (8장)
```
- `'useUnity', false`면 Unity 차는 움직이지 않습니다. 자동 버전은 기본으로 실제 시간에 맞춰 진행합니다.

---

## 1. 차량 모델 (`bicycle_discrete_step.m`)
- **움직임:** 기구학 자전거 모델을 씁니다. 그래서 차가 경로에서 벗어날 수 있고, 논문처럼 "레퍼런스 경로 − 주행 경로"(그래프 ③)를 잴 수 있습니다.
- **조향·전진:** test1의 이산 상태공간 모델 그대로입니다.

$$\begin{bmatrix} s \\ v \\ \delta \end{bmatrix}(k+1) =
\begin{bmatrix} 1 & T & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} s \\ v \\ \delta \end{bmatrix}(k) +
\begin{bmatrix} T^2/2 & 0 \\ T & 0 \\ 0 & T \end{bmatrix}
\begin{bmatrix} a \\ \omega \end{bmatrix}(k)$$

$$x(k+1) = x(k) + \Delta s\cos\theta(k),\quad y(k+1) = y(k) + \Delta s\sin\theta(k),\quad \theta(k+1) = \theta(k) + \frac{\Delta s\tan\delta(k+1)}{L}$$

- Δs = s(k+1) − s(k) = T v + T²/2 a는 더블 인테그레이터로 구한 이번 스텝 이동 거리입니다.
- a = 0이면 `bicycle_step`과 결과가 같습니다(테스트 3).
- 제어기가 낸 조향각 δ는 조향 인테그레이터 입력 ω = (δ − δ_now)/T_s로 바뀝니다. 90°/s, ±30° 제한이 있습니다.

---

## 2. PP_PID 제어기 (`blended_steer.m`)

$$\boxed{\;\delta(t) = \mathrm{Lowpass}\big(K_{pp}\,\delta_{pp} + \delta_p + \delta_i + \delta_d\big),\qquad K_{pp} + K_p + K_i + K_d = 1\;}$$

| 항 | 식 | 출처 | 이 코드 |
|---|---|---|---|
| Pure Pursuit | δ_pp = atan(2L sin α / L_ad) | 논문 Eq.3 | 선행점 = 차를 경로에 투영한 점에서 경로를 따라 L_f 앞. L_ad = 차와 선행점 사이 거리 |
| P | δ_p = K_p · e_la | 논문 Eq.4 | |
| 선행 오차 | e_la = e + (a + L_ad) sin Δψ | 논문 Eq.5 | e: 횡오차, Δψ: 방향 오차 (아래 부호) |
| I | δ_i = K_i · Σ e | 논문 Eq.7 | 스텝마다 e를 더함. **T_s를 곱하지 않음** (논문 식 그대로) |
| **D** | **δ_d = K_d · (e[k] − e[k−1]) / T_s** | **추가 (사용자 식)** | e는 I 항과 같은 횡오차. 첫 스텝은 e[−1] = e[0]으로 봐서 0 |
| Low-pass Filter | δ = Σ δ_previous · w_previous + δ_current · w_current, w_previous = (1 − w_current)/(ℓ − 1) | 논문 Eq.8–9 | `lpf_window.m`. δ_previous = 직전 ℓ − 1개 스텝의 최종 조향각 출력 |
| 게인 합 | K_pp + K_p + K_i = 1 | 논문 | D를 더해 **K_pp + K_p + K_i + K_d = 1**로 넓힘. `sumToOne = true`(기본)이면 K_pp = 1 − K_p − K_i − K_d로 자동 계산 |

**부호와 기준점 (논문에 없어서 정한 것):**
- 조향각은 왼쪽이 +입니다. 그래서 오차는 "목표 − 현재"로 씁니다.
  - e = 0 − e_lat: 차가 경로 오른쪽에 있으면 e > 0이 되어 왼쪽으로 꺾습니다.
  - Δψ = ψ_path − θ
- e와 Δψ는 차의 기준점(뒤 차축 중앙)에서 잽니다.
- 논문 그림 3의 a는 "e를 재는 점에서 앞바퀴 쪽으로의 거리"입니다. 이 차는 기준점이 뒤 차축이므로 기본값을 **a = L = 0.56 m**(앞바퀴 축까지, 2026-11-21 실차값. 이전 0.59 m)로 두었습니다.
  - 논문은 e를 무게중심에서 재고 a를 무게중심 → 앞바퀴 축 거리로 씁니다.
  - 그 해석에 맞추려면 a를 바꾸면 됩니다(슬라이더 있음).

**Low-pass Filter의 성질 (`lpf_window.m`):**
- 가중치 합이 1이라 일정한 입력은 그대로 통과합니다(DC 이득 1).
- ℓ = 1이면 필터가 없습니다.
- ℓ = 2이면 y[k] = w·u[k] + (1 − w)·y[k−1]로, 1차 저역통과 필터와 같습니다(테스트 7).

```matlab
% blended_steer.m (요약)
eLat = 0 - e.e_lat;   dPsi = 0 - e.e_head;                  % e, Δψ (목표 - 현재)
dPP  = atan(2 * L * sin(alpha) / Lad);                      % Eq.3
ela  = eLat + (g.a + Lad) * sin(dPsi);                      % Eq.5
dP   = g.Kp * ela;                                          % Eq.4
mem.Se = mem.Se + eLat;   dI = g.Ki * mem.Se;               % Eq.7
dE = (eLat - mem.ePrev) / T;   dD = g.Kd * dE;              % D (첫 스텝 dE = 0)
dRaw = g.Kpp * dPP + dP + dI + dD;                          % Eq.10 괄호 안 + D
[dCmd, mem.hist] = lpf_window(mem.hist, dRaw, g.ell, g.wcur);   % Eq.8–9
```

### 2.1 비교 제어기 (논문 4장의 PP, PID)
| 제어기 | 식 | 기본값 |
|---|---|---|
| PP | δ = δ_pp (Eq.3만, K_pp = 1, 필터 없음) | L_f = 0.8 m |
| PID | δ = K_p e + K_i Σ e T_s + K_d Δe/T_s, e = −e_lat (`pid_steer.m`, `PID/PID_control.m`과 같은 식) | 3.0, 0.8, 2.6 (`PID_control` 기본 게인) |
| PP_PID | 위 식 | 3장 |

- 논문은 Low-pass Filter를 PP_PI의 구성 요소로 둡니다. 그래서 PP와 PID에는 필터를 넣지 않았습니다(`'lpfAll', true`면 같은 필터를 넣음).
- 세 차는 매 스텝 **같은 가속도**로 나란히 계산하고, Unity에는 PP_PID 차를 보냅니다.

---

## 3. 그림 (`blended_discrete.m`)
![기본 실행 결과](img/result_auto_default.png)

| 위치 | 내용 |
|---|---|
| 왼쪽 | Unity 맵, 레퍼런스 경로(노랑), PP(초록)·PID(파랑)·PP_PID(빨강) 궤적, PP_PID 선행점(점선) |
| 오른쪽 ① | Tracking error (PP_PID): 목표 조향각 δ_ref = atan(Lκ)(노랑), 현재 조향각 δ(빨강), δ_ref − δ(주황) |
| 오른쪽 ② | (δ_ref − δ)의 누적 MSE: PP(초록), PID(파랑), PP_PID(빨강) |
| 오른쪽 ③ | Trajectory tracking error: 레퍼런스 경로 − 주행 경로 = 0 − e_lat |
| 맵 아래 | **표시 체크박스** 8개 (3.1절), 그 아래 **파라미터 슬라이더** 9개 (7장) |

### 3.1 표시 켜기·끄기 (체크박스 또는 그림 창에서 키)
| 체크박스 | 키 | 켜고 끄는 것 |
|---|---|---|
| 레퍼런스 경로 | R | 맵의 노란 경로 |
| PP | 1 | 맵의 PP 궤적 + 그래프 ②, ③의 PP 선 |
| PID | 2 | 맵의 PID 궤적 + 그래프 ②, ③의 PID 선 |
| PP_PID | 3 | 맵의 PP_PID 궤적과 차 위치 + 그래프 ②, ③의 PP_PID 선 |
| 선행점 | L | PP_PID 차 → 선행점 점선 |
| 목표 조향각 | Y | 그래프 ①의 δ_ref |
| 현재 조향각 | D | 그래프 ①의 δ |
| 조향각 오차 | E | 그래프 ①의 δ_ref − δ |

- 체크박스 글자색은 해당 선의 색과 같습니다. 키는 MATLAB 그림 창을 클릭한 뒤 누릅니다.
- 꺼 둔 선은 실행이 끝날 때 저장되는 `result.png`에도 그려지지 않습니다.

---

## 4. 결과 (자동 한 바퀴, 0.8 m/s, T_s = 0.1 s)
### 4.1 기본값 (합 = 1 규칙)
기본값: L_f 0.8 m, K_p 0.01, K_i 0.006, **K_d 0.001**, K_pp 0.983, a 0.56 m(이전 0.59), ℓ 3, w_current 0.5

| 제어기 | 조향각 오차 MSE [rad²] (RMS, 최대) | 궤적 오차 MSE [m²] (RMS, 최대) | 조향 속도 RMS [deg/s] |
|---|---|---|---|
| PP | 1.17e−3 (1.96°, 8.01°) | 1.40e−4 (0.0118 m, 0.033 m) | 3.9 |
| PID | 3.29e−3 (3.29°, 13.57°) | 1.16e−3 (0.0341 m, 0.086 m) | 12.5 |
| **PP_PID** | **1.10e−3 (1.90°, 7.96°)** | **1.05e−4 (0.0102 m, 0.027 m)** | 4.3 |

- PP_PID가 조향각 오차와 궤적 오차 모두 가장 작습니다(궤적 오차 MSE가 PP보다 25% 작음).
- 조향 속도는 PP보다 약간 큽니다.

### 4.2 합 = 1 규칙에서 K_d를 바꿔 본 결과 (K_p 0.01, K_i 0.006)
| K_d | K_pp | 궤적 오차 RMS [m] (최대) | 조향각 오차 RMS | δ_d RMS |
|---|---|---|---|---|
| 0 (= PP_PI) | 0.984 | 0.0102 (0.0272) | 1.90° | 0 |
| **0.001 (기본)** | **0.983** | **0.0102 (0.0273)** | **1.90°** | **0.001°** |
| 0.005 | 0.979 | 0.0105 (0.0276) | 1.91° | 0.003° |
| 0.01 | 0.974 | 0.0108 (0.0281) | 1.91° | 0.007° |
| 0.02 | 0.964 | 0.0114 (0.0291) | 1.92° | 0.014° |
| 0.05 | 0.934 | 0.0133 (0.0322) | 1.95° | 0.038° |

- **합 = 1 규칙에서는 D 항도 P 항처럼 효과가 거의 없습니다.**
  - 이 차에서 횡오차 변화율 (e[k] − e[k−1])/T_s는 RMS 0.013 m/s에 불과합니다. K_d = 0.05여도 δ_d는 0.04°입니다.
  - 반면 K_d만큼 K_pp가 줄어 PP가 덜 꺾습니다. 그래서 K_d를 키울수록 오히려 나빠집니다.
- 기본값 K_d = 0.001은 D 항이 살아 있으면서(K_d > 0) PP_PI와 같은 성능을 내는 값입니다.

### 4.3 합 = 1 규칙에서 K_p, K_i를 바꿔 본 결과 (K_d 0)
| K_p | K_i | K_pp | 궤적 오차 RMS [m] |
|---|---|---|---|
| 0 | 0 | 1 (Low-pass Filter만) | 0.0099 |
| 0 | 0.003 ~ 0.006 | 0.997 ~ 0.994 | 0.0096 |
| **0.01** | **0.006** | **0.984** | **0.0102** |
| 0.05 | 0.003 | 0.947 | 0.0128 |
| 0.1 | 0.003 ~ 0.01 | 0.90 ~ 0.89 | 0.016 ~ 0.018 |
| 0.2 | 0.003 ~ 0.01 | 0.80 ~ 0.79 | 0.024 ~ 0.026 |
| 0.5 | 0 ~ 0.03 | 0.50 ~ 0.47 | 0.051 ~ 0.059 |

**주의할 점: 이 차에서는 합 = 1 규칙 때문에 P, D 항이 거의 도움이 되지 않습니다.**
- K_p, K_d를 키우면 K_pp(단위 없음)가 그만큼 줄어 PP가 덜 꺾습니다. 이 손해가 P, D 항이 버는 것보다 큽니다.
  - 이 차의 오차는 수 cm입니다. K_p = 0.1이어도 e_la가 2 cm면 δ_p = 0.002 rad(0.1°)밖에 안 됩니다.
  - 반면 K_pp가 1 → 0.9가 되면 곡선의 10°짜리 조향이 1° 줄어듭니다.
- 논문의 차(CARLA, 오차 0.5 ~ 2 m)에서는 같은 게인으로도 P 항이 충분히 컸을 것입니다. **"합 = 1"은 오차의 크기(단위)에 따라 효과가 달라지는 규칙입니다.**
- 이 규칙 아래에서 PP 대비 개선은 대부분 Low-pass Filter에서 옵니다(필터만 0.0099 m).

### 4.4 합 = 1 규칙을 빼면 (`'sumToOne', false`, K_pp = 1, K_p 2, K_i 0.02, a 0)
| K_d | 궤적 오차 RMS [m] (최대) | 조향각 오차 RMS | 조향 속도 RMS [deg/s] |
|---|---|---|---|
| 0 (= PP_PI) | 0.0073 (0.0201) | 2.15° | 6.5 |
| 0.3 | 0.0071 (0.0200) | 2.18° | 6.9 |
| 1.0 | 0.0067 (0.0197) | 2.25° | 7.9 |
| 2.0 | **0.0063** (0.0189) | 2.39° | 9.7 |

- 규칙이 없으면 D 항이 궤적 오차를 줄입니다(K_d 2: PP_PI 대비 14% 작음, PP 대비 47% 작음).
- 대신 조향이 거칠어집니다(조향 속도 RMS 6.5 → 9.7 deg/s).
- **a의 영향** (K_pp 1, K_p 2, K_i 0.005, K_d 0): a = 0 → 0.0078 m, 0.3 → 0.0085, 0.59 → 0.0093, 1.0 → 0.0104

### 4.5 Low-pass Filter만 (K_pp 1, PID 0)
| ℓ | w_current | 궤적 오차 RMS [m] | 조향 속도 RMS [deg/s] |
|---|---|---|---|
| 1 (필터 없음 = PP) | – | 0.0118 | 3.9 |
| 2 | 0.5 | 0.0102 | 4.2 |
| **3** | **0.5** | **0.0099** | 4.3 |
| 5 | 0.5 | 0.0107 | 4.4 |
| 3 | 0.3 | 0.0124 | 4.8 |
| 3 | 0.7 | 0.0108 | 4.1 |

---

## 5. 해석
- **PP_PID가 PP보다 나은 이유:**
  - PP는 곡선에서 안쪽으로 질러갑니다(corner cutting).
  - I 항(Σe)은 그 일정한 치우침을 줄입니다.
  - Low-pass Filter는 직전 조향각들을 섞어 조향 변화를 늦춥니다. 이 경로에서는 그것이 질러가기를 조금 줄였습니다(4.5절).
- **D 항의 역할:**
  - 횡오차가 커지는 중이면 미리 더 꺾고, 줄어드는 중이면 덜 꺾습니다.
  - 이 차에서는 오차 변화가 작아(0.013 m/s) 합 = 1 규칙 아래에서는 효과가 거의 없습니다(4.2절). 규칙이 없을 때는 궤적 오차를 줄입니다(4.4절).
  - 논문이 D를 뺀 이유는 "PP의 선행점이 미분보다 미래를 더 잘 예측하기 때문"입니다. 합 = 1 규칙 아래 결과는 이 설명과 맞습니다.
- **PID가 가장 나쁜 이유:** 경로 정보(곡률, 선행점) 없이 오차만 보고 따라가므로, 곡선에 들어갈 때 늦습니다. 논문 4.A절에서도 PID가 오차와 부드러움 모두 가장 나빴습니다.
- **논문과 다른 점:** 논문은 PP_PI의 조향이 PP보다 부드럽다고 했습니다. 여기서는 조향 속도 RMS가 PP 3.9 → PP_PID 4.3 deg/s로 약간 컸습니다.
  - 목표 조향각(경로 곡률)에 약 0.35 m 주기의 잔물결이 있습니다. PI가 그 잔물결을 일부 따라가기 때문으로 보입니다.

---

## 6. 확인한 것
| 확인 | 결과 |
|---|---|
| `test_blended` | 16 / 16 통과 (약 90 s) |
| 1번 Unity 연동 | 실시간 한 바퀴 518스텝, 상태 519줄. Unity 수신 519줄, t = 51.8 s에 출발점으로 돌아옴 |
| 7 ~ 11번 식 | LPF(ℓ = 1, 2, 3, DC 이득), PP(Eq.3), PID(Eq.4–7 + D) 3스텝 손 계산(첫 스텝 δ_d = 0), PP_PID(K_pp = 1 − K_p − K_i − K_d), 비교 PID 손 계산 |
| 14번 blending | 한 바퀴 내내 δ_raw = K_pp δ_pp + (δ_p + δ_i + δ_d) |
| 16번 실행 중 변경 | `check_live_tuning`, 7장 |
| 표시 체크박스 | 체크박스 콜백과 키로 각 그룹의 선이 맵·그래프에서 함께 꺼지고 켜짐 |

> **Unity가 아무것도 안 받을 때:** Unity(`CarStateReceiver`)는 한 번에 한 연결만 받습니다.
> - 지금 버전은 실행이 끝나면(정상 종료, 오류, Ctrl+C 모두) 연결을 닫습니다.
> - 예전 버전으로 연 그림이 남아 있으면 `close all`로 닫으세요. 그래도 안 되면 Unity Play를 한 번 껐다 켜세요.

---

## 7. 파라미터 조절 (튜닝)
| 방법 | 예 |
|---|---|
| ① `blended_params.m`의 기본값 고치기 | `prm.Kd = 0.002;` |
| ② 실행할 때 덮어쓰기 | `blended_discrete('input', 'auto', 'Kp', 0.02, 'Kd', 0.002, 'Lf', 1.0)` |
| ③ 실행 중 슬라이더·입력칸 (그림 왼쪽 아래) | 값을 바꾸면 **다음 스텝부터** 적용 |

| 이름 | 기본값 | 뜻 | 슬라이더 |
|---|---|---|---|
| `v` | 0.8 m/s | 자동 버전 목표 속도 | 0.1 ~ 1.5 |
| `Lf` | 0.8 m | PP 선행점 거리 (경로를 따라) | 0.3 ~ 2.5 |
| `a` | 0.56 m (이전 0.59) | e_la 식의 a | 0 ~ 1.5 |
| `wcur` | 0.5 | Low-pass Filter 이번 값 가중치 | 0.05 ~ 1 |
| `ell` | 3 | Low-pass Filter 창 길이 ℓ (정수) | 1 ~ 10 |
| `Kp` | 0.01 rad/m | P 게인 (e_la에 곱함) | 0 ~ 1 |
| `Ki` | 0.006 rad/m | I 게인 (Σe에 곱함) | 0 ~ 0.05 |
| `Kd` | 0.001 rad·s/m | D 게인 ((e[k] − e[k−1])/T_s에 곱함). 0이면 PP_PI | 0 ~ 0.05 |
| `Kpp` | 0.983 | PP 게인. `sumToOne`이면 1 − Kp − Ki − Kd로 자동(슬라이더 잠김) | 0 ~ 1 |
| `sumToOne` | true | K_pp + K_p + K_i + K_d = 1 규칙 | 실행 중 변경 불가 |
| `pidKp`, `pidKi`, `pidKd` | 3.0, 0.8, 2.6 | 비교용 PID 게인 | 실행 중 변경 불가 |
| `lpfAll` | false | PP, PID에도 같은 필터 | 실행 중 변경 불가 |
| `Ts` | 0.1 s | 한 스텝 시간 | 실행 중 변경 불가 |

- `sumToOne`일 때 Kp, Ki, Kd 중 하나를 바꾸면 Kpp 표시도 함께 바뀝니다. Kpp가 음수가 되는 값은 받지 않습니다.
- 입력칸에는 슬라이더 범위 밖의 값도 넣을 수 있습니다.
- 맵 위 제목 둘째 줄에 지금 쓰는 값이 표시됩니다.
- 스텝마다 쓴 값은 `timeseries.csv`의 `p_v`, `p_Lf`, `p_a`, `p_wcur`, `p_ell`, `p_Kp`, `p_Ki`, `p_Kd`, `p_Kpp` 열에 남습니다. `summary.txt`에는 바꾼 내역이 적힙니다.
- **반응 속도:** 실시간 실행에서는 0.2 s 이상 간격으로 그림 전체를 다시 그립니다. 이때 슬라이더·체크박스 이벤트가 처리되므로, 바꾼 값은 약 0.3 s 안에 반영됩니다.

**확인 (`check_live_tuning`)**
- 한 바퀴 동안 정해진 스텝에 패널을 조작합니다. 슬라이더 Value나 입력칸 String을 바꾸고 콜백을 부르는 방식으로, 마우스로 조작할 때와 같은 경로입니다.
- 그 뒤 **기록만으로 실제로 쓰인 값을 거꾸로 잽니다.**

| 파라미터 | 조작 (스텝 k) | 기록에서 잰 방법 | 결과 |
|---|---|---|---|
| v 0.8 → 1.0 | 슬라이더 40 | 차 속도 | k = 40부터 |
| Lf 0.8 → 1.3 | 입력칸 90 | 선행점이 투영점에서 경로를 따라 몇 m 앞인지 | k = 90부터 |
| a 0.59 → 0.3 | 슬라이더 140 | (e_la − e) / sin Δψ − L_ad | k = 140부터 |
| w_current 0.5 → 0.7 | 입력칸 190 | (δ − m) / (δ_raw − m), m = 이전 ℓ − 1개 출력 평균 | k = 190부터 |
| Kp 0.01 → 0.3 | 슬라이더 240 | δ_p / e_la | k = 240부터 |
| Ki 0.006 → 0.004 | 입력칸 290 | δ_i / Σe | k = 290부터 |
| ℓ 3 → 5 | 슬라이더 340 | ℓ = 1 ~ 10 중 Eq.8을 만족하는 값 | k = 340부터 |
| **Kd 0.001 → 0.003** | **입력칸 380** | **δ_d / ((e[k] − e[k−1]) / T_s)** | **k = 380부터** |
| Kpp (자동) | Kp·Ki·Kd와 함께 | (δ_raw − δ_p − δ_i − δ_d) / δ_pp | 0.983 → 0.693, 매 스텝 K_pp + K_p + K_i + K_d = 1 |

- 모든 값이 조작한 바로 다음 스텝부터 새 값으로 쓰였습니다.
- 기록된 차 위치와 `p_*` 값만으로 세 제어기의 δ를 처음부터 다시 계산했더니, 기록과의 차이는 **0**이었습니다.

![실행 중 파라미터 변경 확인](img/live_tune_check.png)

---

## 8. Blending인가 switching인가
- 매 스텝 K_pp δ_pp와 δ_p + δ_i + δ_d를 **둘 다 계산해서 더합니다**. PP와 PID를 번갈아 고르는 조건문은 없습니다.

![PP_PID 항별 기여](img/blend_components.png)

| 확인 (기본값 한 바퀴) | 결과 | switching이라면 |
|---|---|---|
| δ_raw − (K_pp δ_pp + δ_p + δ_i + δ_d) | 최대 5.6e−17 rad | 크게 어긋나는 스텝이 생김 |
| 두 항이 동시에 0이 아닌 스텝 | 99.6% | 0% |
| PP 몫 \|K_pp δ_pp\| / (\|K_pp δ_pp\| + \|δ_p + δ_i + δ_d\|) | 평균 0.87 | 0 또는 1만 |

- 기본값은 PID 게인이 작아서 PP 몫이 큽니다(평균 0.87).

---

## 9. 핵심 코드와 수식 위치
**① PP_PID: [`blended_steer.m`](blended_steer.m)**

| 줄 | 코드 | 식 |
|---|---|---|
| 6–14 | 주석 | 전체 식 정리 |
| 34–38 | `e = path_error(...)`, `eLat = 0 - e.e_lat;`, `dPsi = 0 - e.e_head;`, `dRef = atan(L * e.kappa);` | e, Δψ, δ_ref |
| 42–47 | `ql = path_pose(ref, e.s + g.Lf);` … `dPP = atan(2 * L * sin(alpha) / Lad);` | **논문 Eq.3** |
| 50 | `ela = eLat + (g.a + Lad) * sin(dPsi);` | **논문 Eq.5** |
| 51 | `dP = g.Kp * ela;` | **논문 Eq.4, 6** |
| 54–55 | `mem.Se = mem.Se + eLat;  dI = g.Ki * mem.Se;` | **논문 Eq.7** |
| 58–64 | `dE = (eLat - mem.ePrev) / T;` … `dD = g.Kd * dE;` | **D: K_d (e[k] − e[k−1]) / T_s** |
| 67 | `dRaw = g.Kpp * dPP + dP + dI + dD;` | **논문 Eq.10 괄호 안 + D** |
| 68 | `[dCmd, mem.hist] = lpf_window(mem.hist, dRaw, g.ell, g.wcur);` | **논문 Eq.10 Lowpass** |

**② Low-pass Filter: [`lpf_window.m`](lpf_window.m)** — 3–4행 수식, 18행 `wprev = (1 - wcur) / (ell - 1);` (**Eq.9**), 19행 `y = wcur * u + wprev * sum(hist(1:ell - 1));` (**Eq.8**)

**③ 게인 합 = 1: [`blended_params.m`](blended_params.m)** — 24–30행 Kp, Ki, Kd, Kpp, sumToOne, a / 33–34행 ell, wcur / 62행 `prm.Kpp = 1 - prm.Kp - prm.Ki - prm.Kd;`. 실행 중에는 [`blended_discrete.m`](blended_discrete.m) 223행, 570행

**④ 비교용 PID: [`pid_steer.m`](pid_steer.m)** — 18행 적분, 21–22행 P·I·D 합

**⑤ 실행 루프: [`blended_discrete.m`](blended_discrete.m)**

| 줄 | 하는 일 |
|---|---|
| 434–442 | `makeGains`: PP (K_pp 1, PID 0, 필터 없음), PID, PP_PID 게인 |
| 220–223 | 슬라이더로 바꾼 값을 이번 스텝부터 적용 (sumToOne이면 Kpp 다시 계산) |
| 241, 243 | 제어기 호출 (`pid_steer` / `blended_steer`) |
| 245–246 | ω = (δ − δ_now)/T_s → `bicycle_discrete_step` (차량 모델) |
| 180–189, 490–520 | 표시 체크박스와 키 |
| 522–589 | 파라미터 패널과 콜백 |

**⑥ 차량 모델:** [`bicycle_discrete_step.m`](bicycle_discrete_step.m) 23–31행, [`path_model_step.m`](path_model_step.m) 20행

> 줄 번호는 2026-11-09 기준입니다. 코드를 고치면 바뀔 수 있으니 위 코드 문자열로 찾으세요.
