# test1 — 정해진 경로 위 이산 상태공간 모델과 Unity 한 스텝 주행

이 폴더에는 아래 요구사항을 구현한 코드가 모여 있습니다.
> 이산, k+1 하는 거 유니티에서, 정해진 패스 가정 / 이산 공간 모델링 / 인테그레이터(조향), 전진(더블 인테그레이터) / 키보드를 누르든 뭘 하든 해서 유니티에서 이산 공간에서 하나씩 패스에 따라 갈 수 있게 만들기

| 파일 | 역할 |
|---|---|
| `MATLAB/test1/path_model_matrices.m` | 이산 상태공간 행렬 A, B |
| `MATLAB/test1/path_model_step.m` | 한 스텝 x(k) → x(k+1), 입력 한계 포함 |
| `MATLAB/test1/path_pose.m` | 경로를 따라 간 거리 s → 경로 위 위치·방향·곡률 |
| `MATLAB/test1/discrete_path_drive.m` | 실행 파일: Unity 키 받기 → 입력 계산 → k+1 → Unity 전송 → 기록 |
| `MATLAB/test1/test_path_model.m` | 테스트 13개 |
| `KIIS2026_AutoCar/Assets/scripts/test1/StepInput.cs` | Unity 키 → MATLAB으로 보내기 |
| `KIIS2026_AutoCar/Assets/scripts/CarStateReceiver.cs` 의 `SendLine()` | 공용 파일이라 제자리에 둠. 같은 TCP 연결로 MATLAB에 한 줄 보내기 |

공용 함수는 상위 `MATLAB/` 폴더의 것을 그대로 씁니다: `vehicle_params`, `load_reference_path`, `path_error`, `unity_link`, `draw_map`. 결과는 `MATLAB/results/`에 저장됩니다.

**실행 (Unity Play 먼저)**
```matlab
cd MATLAB
addpath('test1')
discrete_path_drive                      % 키보드 버전
discrete_path_drive('input', 'auto')     % 자동 버전
test_path_model                          % 테스트 (13 / 13 passed)
```

---

## 1. 정해진 path를 어떻게 가정했는가
**가정: 차는 정해진 경로 위에 항상 있고, 경로를 따라서만 움직인다 (옆으로 벗어나지 않는다).**

- **경로:** pure pursuit·PID와 같은 레퍼런스 경로 `MATLAB/reference_path_outer_lane.mat`입니다.
  - Unity 맵 이미지에서 뽑은 바깥 차선 중심선입니다.
  - 1 cm 간격 점 4107개, 한 바퀴 41.07 m, 닫힌 경로입니다.
  - 각 점마다 위치 (x, y), 방향 ψ, 곡률 κ, 출발점부터 거리 s를 가지고 있습니다.
- **차 위치 = 경로 위 한 점:**
  - 차가 경로 위에 있으면 위치와 방향은 **"경로를 따라 얼마나 갔는가(s)" 하나로 정해집니다.**
  - 차 위치 (x, y) = 경로에서 거리 s인 점
  - 차 방향 θ = 그 점에서 경로의 접선 방향 ψ
  - 그 점의 곡률 κ(s) = 그 위치에서 필요한 회전 정도
- **얻는 것:**
  - 원래 차 모델은 상태 5개(x, y, θ, v, δ)에 cos·sin·tan이 들어간 비선형입니다.
  - 경로 가정을 하면 상태가 **3개(s, v, δ)로 줄고 식이 선형**이 됩니다.
- **잃는 것:**
  - 옆으로 벗어난 거리(횡오차)를 표현할 수 없습니다(항상 0).
  - 조향각 δ는 상태로 계산하지만, 차의 위치에는 영향을 주지 않습니다. 경로 곡률에 맞춰 따라가는 값입니다.
- **출발점:** Unity 차 위치 (7.20, −0.89)를 경로에 투영한 지점(s₀)에서 정지 상태(v = 0)로 시작합니다. 조향각은 그 지점 곡률에 맞는 값으로 시작합니다.

```matlab
% discrete_path_drive.m — 출발 상태
e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
x0 = [e0.s; 0; atan(p.L * path_pose(ref, e0.s).kappa)];     % [s; v; delta]
```

**경로에서 위치·방향을 읽는 함수 (`path_pose.m`)**
```matlab
function q = path_pose(ref, s)
sP = [ref.s; ref.L];                         % 끝에 첫 점을 한 번 더 붙여 한 바퀴를 닫음
sm = mod(s(:), ref.L);                       % s가 한 바퀴(41.07 m)를 넘어도 됨
q.x     = interp1(sP, [ref.x; ref.x(1)], sm);
q.y     = interp1(sP, [ref.y; ref.y(1)], sm);
psi     = interp1(sP, unwrap([ref.psi; ref.psi(1)]), sm);   % 각도는 펼친 뒤 보간
q.psi   = atan2(sin(psi), cos(psi));
q.kappa = interp1(sP, [ref.kappa; ref.kappa(1)], sm);
end
```
- **`mod(s, L)`:** 한 바퀴를 넘으면 다시 처음 점이 됩니다. s 자체는 계속 커지게 두어 진행 거리로 씁니다.
- **`unwrap`:** 방향이 +180° ↔ −180°로 넘어가는 곳에서 보간이 엉뚱한 값을 내지 않게 각도를 이어 붙입니다.

---

## 2. 이산 공간 모델링을 어떻게 진행했고 어떻게 나타냈는가
네 단계로 진행했습니다.

**① 상태와 입력 정하기**

| | 기호 | 뜻 | 단위 |
|---|---|---|---|
| 상태 x | s | 경로를 따라 간 거리 | m |
| | v | 속도 | m/s |
| | δ | 조향각 | rad |
| 입력 u | a | 가속도 | m/s² |
| | ω | 조향 속도 (조향각이 바뀌는 빠르기) | rad/s |

**② 연속 시간 상태공간 식**

$$\dot x = A_c\,x + B_c\,u,\qquad
A_c = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix},\quad
B_c = \begin{bmatrix} 0 & 0 \\ 1 & 0 \\ 0 & 1 \end{bmatrix}$$

**③ 이산화 (영차 유지, ZOH)**
- **가정:** 입력은 T초마다 한 번 정해지고, 다음 스텝까지 같은 값을 유지합니다. 디지털 제어기가 실제로 동작하는 방식과 같습니다.
- 이 가정에서 한 주기 [kT, (k+1)T] 동안 연속 시간 식을 **정확히 적분**하면 아래 식이 됩니다.

$$A = e^{A_c T},\qquad B = \int_0^{T} e^{A_c \tau}\,d\tau\;B_c$$

**④ 결과: 이산 상태공간 모델**

$$\boxed{\,x(k+1) = A\,x(k) + B\,u(k)\,},\qquad
x(k) = \begin{bmatrix} s(k) \\ v(k) \\ \delta(k) \end{bmatrix},\quad
u(k) = \begin{bmatrix} a(k) \\ \omega(k) \end{bmatrix}$$

$$A = \begin{bmatrix} 1 & T & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix},\qquad
B = \begin{bmatrix} T^2/2 & 0 \\ T & 0 \\ 0 & T \end{bmatrix},\qquad T = 0.1\ \text{s (기본값)}$$

- **k의 뜻:** k는 스텝 번호이고, 시간은 t = k·T입니다.
- **T = 0.1 s:** 0.8 m/s일 때 한 스텝에 8 cm씩 움직여, 한 스텝씩 가는 것이 눈에 보입니다. 실행 때 `'T', 0.02`처럼 바꿀 수 있습니다.

**출력 (화면에 보낼 차 위치)** — 비선형이지만 상태 식에는 들어가지 않습니다.

$$\big(x,\ y,\ \psi\big)(k) = h\big(s(k)\big) \quad \text{(경로를 } s(k)\text{에서 보간, 1절의 path\_pose)}$$

즉 **"선형 상태 식 + 경로에서 읽는 출력 식"** 구조입니다.

**입력 한계:** 실제 차의 한계는 선형식에 넣을 수 없습니다. 그래서 입력을 먼저 자른 뒤 선형식에 넣습니다(3절 코드).

| 입력 | 한계 |
|---|---|
| a | −2.0 ~ +1.0 m/s², 그리고 다음 속도가 0 ~ 1.5 m/s 안 (후진 없음) |
| ω | ±90°/s, 그리고 다음 조향각이 ±30° 안 |

---

## 3. 인테그레이터(조향), 더블 인테그레이터(전진)의 수식과 코드 반영

### 3.1 전진 = 더블 인테그레이터 (가속도 → 속도 → 거리)
**연속 시간:**

$$\dot s = v,\qquad \dot v = a \qquad\Longleftrightarrow\qquad \ddot s = a \quad\text{(가속도를 두 번 적분하면 거리)}$$

$$\frac{d}{dt}\begin{bmatrix} s \\ v \end{bmatrix} = \underbrace{\begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}}_{N}\begin{bmatrix} s \\ v \end{bmatrix} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} a$$

**이산화:** N² = 0이라 지수함수가 두 항에서 끝납니다.

$$e^{NT} = I + NT = \begin{bmatrix} 1 & T \\ 0 & 1 \end{bmatrix},\qquad
\int_0^T e^{N\tau}d\tau\begin{bmatrix} 0 \\ 1 \end{bmatrix} = \begin{bmatrix} T & T^2/2 \\ 0 & T \end{bmatrix}\begin{bmatrix} 0 \\ 1 \end{bmatrix} = \begin{bmatrix} T^2/2 \\ T \end{bmatrix}$$

$$\begin{bmatrix} s \\ v \end{bmatrix}(k+1) = \begin{bmatrix} 1 & T \\ 0 & 1 \end{bmatrix}\begin{bmatrix} s \\ v \end{bmatrix}(k) + \begin{bmatrix} T^2/2 \\ T \end{bmatrix} a(k)$$

풀어 쓰면 등가속도 운동 공식과 같습니다.

$$s(k+1) = s(k) + T\,v(k) + \tfrac{T^2}{2}\,a(k),\qquad v(k+1) = v(k) + T\,a(k)$$

**T²/2 항을 넣은 이유:** 오일러 근사(s(k+1) = s(k) + T·v(k))를 쓰면 가속할 때마다 거리가 덜 나와 오차가 쌓입니다. ZOH 식은 입력이 한 주기 동안 일정하면 연속 시간과 **정확히 같습니다**.
- 예: T = 0.1 s, a = 0.5 m/s²로 10스텝(1초) 가속
  - ZOH: s = 0.25 m (정답 ½·0.5·1²)
  - 오일러: s = 0.225 m

### 3.2 조향 = 인테그레이터 (조향 속도 → 조향각)
$$\dot\delta = \omega \quad\Longrightarrow\quad \delta(k+1) = \delta(k) + T\,\omega(k)\qquad (A = 1,\ B = T)$$

### 3.3 합친 모델
$$\begin{bmatrix} s \\ v \\ \delta \end{bmatrix}(k+1) =
\underbrace{\begin{bmatrix} 1 & T & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}}_{A}
\begin{bmatrix} s \\ v \\ \delta \end{bmatrix}(k) +
\underbrace{\begin{bmatrix} T^2/2 & 0 \\ T & 0 \\ 0 & T \end{bmatrix}}_{B}
\begin{bmatrix} a \\ \omega \end{bmatrix}(k)$$

전진(위 2×2 블록)과 조향(마지막 1×1)은 서로 영향을 주지 않는 독립된 두 시스템입니다.

### 3.4 코드 반영
**행렬 — `path_model_matrices.m`**
```matlab
function [A, B] = path_model_matrices(T)
A = [1 T 0;          % s(k+1) = s(k) + T v(k)      (+ T^2/2 a(k), B의 첫 행)
     0 1 0;          % v(k+1) = v(k)               (+ T a(k))
     0 0 1];         % δ(k+1) = δ(k)               (+ T ω(k))
B = [T^2 / 2  0;     % 더블 인테그레이터: 가속도가 거리에 들어가는 몫
     T        0;     % 더블 인테그레이터: 가속도가 속도에 들어가는 몫
     0        T];    % 인테그레이터: 조향 속도가 조향각에 들어가는 몫
end
```

**한 스텝 (입력 한계 포함) — `path_model_step.m`**
```matlab
function [xk1, used] = path_model_step(xk, uk, p, T)
[A, B] = path_model_matrices(T);
v = xk(2); delta = xk(3);
a     = min(max(uk(1), -p.brake), p.accel);                    % ① -2.0 ~ +1.0 m/s²
a     = min(max(a, -v / T), (p.vMax - v) / T);                 % ② v(k+1)이 0 ~ 1.5 m/s 안에
omega = min(max(uk(2), -p.steerRate), p.steerRate);            % ③ ±90°/s
omega = min(max(omega, (-p.deltaMax - delta) / T), (p.deltaMax - delta) / T);   % ④ δ(k+1)이 ±30° 안에
used  = [a; omega];                                            % 실제로 쓴 입력 (기록용)
xk1   = A * xk(:) + B * used;                                  % x(k+1) = A x(k) + B u(k)
end
```

| 줄 | 하는 일 | 이유 |
|---|---|---|
| ① | 가속도를 차의 가감속 능력으로 자름 | `vehicle_params`의 accel 1.0, brake 2.0 |
| ② | 다음 속도가 0 아래(후진)나 최고 속도 위로 가지 않게 a를 더 자름 | 계산 뒤 v만 자르면 같은 스텝의 s(T²/2·a)와 어긋남. a를 잘라야 s와 v가 항상 더블 인테그레이터 식을 함께 따름 |
| ③④ | 조향 속도·조향각 한계 | 같은 이유로 ω를 잘라 δ가 ±30°를 넘지 않게 함 |
| 마지막 | 선형 식 그대로 계산 | A, B는 위 수식과 같음 |

**두 입력을 정하는 곳 — `discrete_path_drive.m`의 `doStep`**
```matlab
% 가속도 a(k): 더블 인테그레이터의 입력
if opt.input == "keyboard"
    aReq = p.accel * (up && ~down) - p.brake * (down && ~up);   % ↑ +1.0 / ↓ -2.0 / 없음 0
else
    aReq = (opt.v_tar - x(2)) / T;          % 자동: 한 스텝에 목표 속도로 (한계는 모델이 자름)
end
% 조향 속도 ω(k): 인테그레이터의 입력. 다음 위치 s(k+1)의 곡률에 맞는 조향각으로
xs = path_model_step(x, [aReq; 0], p, T);  % s(k+1)을 먼저 구함 (s, v는 조향과 무관)
deltaRef = atan(p.L * path_pose(ref, xs(1)).kappa);            % tanδ = L/R = L·κ
[xn, used] = path_model_step(x, [aReq; (deltaRef - x(3)) / T], p, T);
x = xn;  k = k + 1;                         % k → k+1
```
- **ω를 정하는 법:** 경로 위를 달리므로, 조향각은 그 위치 곡률에 맞는 δ_ref = atan(L·κ)이어야 합니다.
  - ω = (δ_ref − δ)/T는 "한 스텝에 맞춰라"는 뜻입니다.
  - 조향 속도 한계(90°/s)에 걸리면 몇 스텝에 걸쳐 따라갑니다.
- **모델을 두 번 부르는 이유:** ω를 정하려면 다음 위치 s(k+1)의 곡률이 필요합니다.
  - s(k+1)은 a에만 달려 있으므로, ω = 0으로 한 번 계산해 s(k+1)을 얻습니다.
  - 그 뒤 ω를 정하고, 한 번 더 진짜로 계산합니다.

**확인 (`test_path_model.m`):**
- 행렬 A, B
- 1초 가속 결과 s = 0.25 m, v = 0.5 m/s가 정확히 맞음
- 가속·속도·조향 한계
- 반경 3 m 원 경로에서 조향각이 atan(L/R) = 11.13°로 수렴
- 0 → 0.8 m/s가 정확히 8스텝

---

## 4. Unity에서 키를 눌러 한 스텝씩 경로를 따라가게 만드는 방법

### 4.1 구조
계산은 MATLAB이 하고, Unity는 **키를 보내고 결과를 보여줍니다.** 이미 있던 MATLAB → Unity TCP 연결(127.0.0.1:55001) 하나를 **양방향**으로 씁니다.
```
Unity (StepInput.cs)                          MATLAB (discrete_path_drive.m)
 Space/→ 누름 ── "STEP,↑,↓" ─────────────▶   a(k), ω(k) 정하기
                                                x(k+1) = A x(k) + B u(k)   (k → k+1)
                                                위치 = 경로(s(k+1)), 기록
 차를 그 위치로 옮김 ◀── 상태 한 줄 ───────    "t,x,y,ψ,v,δ,..."  (t = kT)
 (CarStateReceiver.cs)
```

### 4.2 키와 메시지 약속
| Unity 키 (Game 창 클릭 후) | MATLAB으로 가는 줄 | MATLAB이 하는 일 |
|---|---|---|
| Space 또는 → | `STEP,<↑ 눌림 0/1>,<↓ 눌림 0/1>` | 한 스텝 (k → k+1) |
| ↑ / ↓ (스텝 키와 함께) | 위 줄의 0/1 | 그 스텝의 가속도: ↑ +1.0, ↓ −2.0, 없으면 0 |
| P | `PLAY` | 자동 버전: 진행 ↔ 일시정지 |
| Backspace | `RESET` | k = 0, 출발점으로 (새 에피소드로 기록) |
| Q | `QUIT` | 종료, 기록 저장 |

> **주의:** 키보드 버전에서 Space만 누르면 "가속도 0으로 한 스텝"입니다.
> - 멈춘 상태(v = 0)에서는 t는 0.1씩 늘어도 차는 제자리입니다.
> - **↑를 누른 채 Space**를 눌러야 출발합니다. 꾹 누르면 연속으로 진행합니다.

### 4.3 두 버전
| | 키보드 버전 (`discrete_path_drive`) | 자동 버전 (`discrete_path_drive('input', 'auto')`) |
|---|---|---|
| k → k+1 | Unity 스텝 키를 누를 때마다 | MATLAB이 T초마다 스스로 |
| 가속도 a | 키 (↑ / ↓ / 없음) | 목표 속도(0.8 m/s)에 맞춰 자동 |
| 조향 속도 ω | 경로 곡률을 따라감 (공통) | 〃 |
| Unity 없이 | 실행 불가 | `'useUnity', false`로 가능 |

### 4.4 Unity 쪽 구현
**(1) 같은 연결로 MATLAB에 보내기 — `CarStateReceiver.cs`에 추가한 `SendLine`**
```csharp
volatile NetworkStream stream;        // 받는 스레드가 연결될 때 저장 (stream = client.GetStream()), 끊기면 null

public bool SendLine(string line)
{
    NetworkStream s = stream;
    if (!clientConnected || s == null) return false;           // MATLAB이 연결 안 됐으면 아무것도 안 함
    try
    {
        byte[] bytes = Encoding.ASCII.GetBytes(line + "\n");    // MATLAB readline은 줄바꿈(LF)까지 읽음
        lock (sendLock) { s.Write(bytes, 0, bytes.Length); }
        return true;
    }
    catch (Exception) { return false; }
}
```
- **받기와 보내기를 동시에:**
  - 원래 Unity는 별도 스레드에서 **받기만** 했습니다.
  - TCP 스트림은 한쪽 스레드가 읽고 다른 스레드가 쓰는 것을 동시에 해도 되므로, 받기 스레드는 그대로 두고 메인 스레드에서 같은 스트림에 씁니다.
  - 쓰기끼리 겹치지 않게만 `lock`을 겁니다.

**(2) 키 읽기 — `test1/StepInput.cs`**
```csharp
void Update()
{
    bool stepHeld = Held(K.Space) || Held(K.Right);
    if (Pressed(K.Space) || Pressed(K.Right))                 // 누른 순간: 바로 한 스텝
    {
        SendStep();
        nextRepeat = Time.unscaledTime + repeatDelay;          // 0.3초 뒤부터
    }
    else if (stepHeld && Time.unscaledTime >= nextRepeat)     // 누르고 있으면: 0.1초마다 계속
    {
        SendStep();
        nextRepeat = Time.unscaledTime + repeatInterval;
    }
    if (Pressed(K.P)) Send("PLAY");
    if (Pressed(K.Backspace)) Send("RESET");
    if (Pressed(K.Q)) Send("QUIT");
}
void SendStep() => Send($"STEP,{(Held(K.Up) ? 1 : 0)},{(Held(K.Down) ? 1 : 0)}");
```
- **키 반복:** 한 번 누르면 정확히 한 스텝, 꾹 누르면 키보드 자동 반복처럼 연속 진행합니다.
- **입력 시스템:** 이 프로젝트는 구 Input Manager를 씁니다. 다른 스크립트처럼 `#if ENABLE_LEGACY_INPUT_MANAGER`(Input.GetKey)와 `#elif ENABLE_INPUT_SYSTEM`(Keyboard.current) 둘 다 지원합니다.
- **자동으로 붙기:** Play하면 장면에 `CarStateReceiver`가 있을 때 `StepInput` 오브젝트를 스스로 만듭니다. 그래서 장면 파일을 고칠 필요가 없습니다.
  ```csharp
  [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
  static void Install()
  {
      if (FindAnyObjectByType<StepInput>() != null) return;
      var r = FindAnyObjectByType<CarStateReceiver>();
      if (r == null) return;
      new GameObject("StepInput").AddComponent<StepInput>().receiver = r;
  }
  ```
- **화면 안내:** 왼쪽 아래에 키 안내와 `connected, last: STEP,1,0`(마지막으로 보낸 줄)을 표시합니다. 키가 MATLAB으로 갔는지 여기서 확인할 수 있습니다.
- **스텝 확인:** 왼쪽 위 HUD의 `t`가 k·T라서, 몇 번째 스텝인지 알 수 있습니다.

**(3) 받은 상태로 차 옮기기:** 기존 `CarStateReceiver.Update()` 그대로입니다. MATLAB x → Unity X, MATLAB y → Unity Z, 회전 Y = 90° − ψ, 앞바퀴 = −δ로 옮깁니다.

### 4.5 MATLAB 쪽 구현 — `discrete_path_drive.m`
**키 받기 (기다리지 않고, 와 있는 것만 읽기):**
```matlab
function cmds = pollCommands()
    cmds = strings(0, 1);
    try
        while client.NumBytesAvailable > 0
            ln = strtrim(string(readline(client)));
            if strlength(ln) > 0, cmds(end + 1, 1) = ln; end
        end
    catch
        lost = true;                                % 읽기 실패 = 연결 끊김
    end
end
```

**메인 루프 (요약, 실제 코드는 `discrete_path_drive.m`):**
```matlab
while running
    for c = pollCommands()'                          % ① Unity 키 처리
        parts = split(c, ',');
        switch parts(1)
            case "STEP"                              % 키보드 버전, 또는 자동 버전 일시정지 중
                if opt.input == "keyboard" || ~playing
                    doStep(numel(parts) >= 2 && parts(2) == "1", numel(parts) >= 3 && parts(3) == "1");
                end
            case "PLAY",  if opt.input == "auto", playing = ~playing; end
            case "RESET", resetEpisode();            % k = 0, 출발점, Unity 궤적 지우기
            case "QUIT",  running = false;
        end
    end
    if running && playing && (~opt.realtime || toc(t0) >= tNext)   % ② 자동 버전: T초마다
        doStep(false, false);
        tNext = tNext + T;
    end
    % ③ 자동 버전: 한 바퀴 돌면 완주로 끝
    % ④ 할 일이 없으면 2 ms 쉬고, 1초마다 같은 상태를 다시 보내 연결 확인 (실패하면 저장 후 종료)
end
```
- **키보드 버전:** MATLAB은 스스로 진행하지 않고 `STEP`이 올 때만 `doStep`을 합니다. 그래서 **k → k+1을 Unity 키가 정합니다.**
- **자동 버전:** `tNext`로 T초마다 진행합니다.
- **이전 키 버리기:** 연결 직후 이미 와 있던 데이터는 버립니다. 이전 실행에서 누른 키가 섞이지 않게 하려는 것입니다.

**한 스텝 (`doStep`) 순서:**
1. a(k), ω(k)를 정합니다(3.4절 코드).
2. `path_model_step`으로 x(k+1)을 구합니다.
3. 기록합니다.
4. k = k+1로 넘어갑니다.
5. `path_pose`로 위치를 구해 Unity에 상태를 보냅니다.

```matlab
function sendState()
    q = path_pose(ref, x(1));
    st = struct('t', k * T, 'x', q.x, 'y', q.y, 'yaw', q.psi, 'v', x(2), 'delta', x(3), ...
        'yawRate', x(2) * q.kappa);
    if ~unity_link('state', client, st), lost = true; end
end
```

### 4.6 기록 — `MATLAB/results/<시간>_discrete_<keyboard|auto>/`
| 파일 | 내용 |
|---|---|
| `timeseries.csv` | 한 행 = 스텝 k의 상태와 그때 넣은 입력. `k`, `episode`, `t_s`, `s_m`, `v_mps`, `delta_rad`, `x_m`, `y_m`, `yaw_rad`, `kappa_1pm`, `a_req_mps2`(요청), `a_mps2`(실제), `omega_radps`, `delta_ref_rad`, `key_up`, `key_down`, `progress_m` |
| `summary.txt` | A, B 행렬, 끝난 이유, 스텝·에피소드 수, 진행 거리, 최고 속도, 한계에 걸린 스텝 수 |
| `trajectory.png` | 맵 위 스텝 위치(점 하나 = 스텝 하나, 색 = 속도), v(t), δ(t) |

### 4.7 확인한 것
| 확인 | 결과 |
|---|---|
| `test_path_model` | 13 / 13 통과 |
| 자동 한 바퀴 (Unity 없이) | 518스텝(51.8 s)에 완주, 경로를 벗어나지 않음 |
| 통신 (가짜 Unity) | 가속 10스텝 → 1.0 m/s, 그냥 5스텝 → 1.0 유지, 감속 3스텝 → 0.4 m/s, 거리 1.21 m (손 계산과 같음), RESET 동작 |
| 실제 Unity | Unity에서 STEP 30개 → 차가 경로를 따라 2.5 m 이동 (t = 3.0 s, v = 1.0 m/s). 사용자의 키 입력도 MATLAB에 전달되어 처리됨 |
