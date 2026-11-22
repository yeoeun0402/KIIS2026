# Plant ↔ Unity 인터페이스 규약 (초안 v0.1)

작성 2026-11-21 · 상태: **초안, Unity 개발자 검토 전** · 짝 문서 `PLAN_SIM2REAL.md`

## 0. 경계

기준 그림: `~/demo/system_block.png`.

```
          ┌──────────────── Unity (노란 부분) ────────────────┐
  r ──▶ (+) ── u ─────────────────────────────────────────────┼──▶ /plant/u
         ▲−                                                   │
         K ◀── x̂ ── observer: x̂' = A x̂ + B u + L (y − ŷ)  ◀───┼──── /plant/y (+ /plant/y_aux)
          └───────────────────────────────────────────────────┘
  ┌──────────────────── Plant (이 저장소, 실차) ─────────────────────┐
  │ /plant/u → plant_io → 구동 계층(조향 위치 루프·속도 루프) → 차량   │
  │ 차량 → C920 → ArUco 검출 → 고정 지도 로컬라이제이션(측정 KF) → y   │
  └──────────────────────────────────────────────────────────────────┘
```

- **이 저장소가 맡는 것:** u를 정의대로 실행하는 것, 그리고 y를 정의대로 측정해서 내보내는 것. 그림의 파란 점선과 C 블록에 해당합니다.
- **Unity가 맡는 것:** 기준 r, 제어기 K, observer(Luenberger에서 KF까지), 차량 모델 A·B.

### 0.1 아르코 측정 KF ≠ observer
| | 측정 KF (plant 쪽, C 블록 안) | observer (Unity) |
|---|---|---|
| 하는 일 | 마커 검출 잡음·튐 제거, 마커 코너 → 차량 자세 | 모델과 y로 전체 상태 x̂ 추정 |
| u 사용 | **안 함** | 함 (B u) |
| 운동 모델 | 없음 (zero-motion 랜덤워크) | 차량 동역학 A, B |
| 출력 | y + 측정 공분산 R_k | x̂ |

규칙:
1. `/plant/y`에는 **u나 차량 운동 모델로 예측·평활한 값을 넣지 않습니다.** 넣으면 observer가 모델을 두 번 적용하고 지연이 생기며, 잡음이 시간적으로 상관되어 L(또는 KF의 R) 설계가 틀어집니다.
2. 기존 `/car/state`(car_state 노드, CTRV EKF로 v·yaw_rate 추정)는 **모니터링·진단 전용**입니다. observer 입력으로 쓰지 않습니다.
3. y가 없는 프레임(마커 미검출)은 값을 지어내지 않고 `valid=false`로 보냅니다. observer는 그 주기에 예측만 합니다.

## 1. 좌표계·단위·부호

| 항목 | 규약 |
|---|---|
| 좌표계 `track` | 오른손. 원점 = 매트(16 × 11 m) 왼쪽 아래 모서리 바닥, x 오른쪽(16 m 변), y 위쪽(11 m 변), z 위. 정의 원본 `track/track.yaml` |
| 차량 기준점 | `base_link` = **뒷바퀴 축 중심의 바닥 투영점** (kinematic bicycle 기준점) |
| θ | +x 기준 반시계 +, rad, (−π, π] |
| v | m/s, 전진 + |
| δ | rad, 자전거 모델 유효 조향각(가상 앞바퀴), **좌 +** |
| 시간 | ROS 시각(벽시계, ns). plant와 Unity가 같은 PC에 있거나 chrony로 동기 |
| 단위 | SI (m, rad, s) |

**Unity 좌표 변환** (Unity는 왼손, y 위). 원점을 Unity 트랙과 같은 점(매트 왼쪽 아래 모서리)에 둔다는 가정이며 **확인 필요**합니다.
```
track ← unity :  x = z_u,   y = −x_u,   z = y_u
unity ← track :  x_u = −y,  y_u = z,    z_u = x
heading       :  yaw_u(deg, +y 축, 위에서 볼 때 시계방향) = −θ(deg)
```
검증: track에서 θ=0(+x 방향) → Unity +z 방향(yaw_u=0). θ=+90°(+y 방향) → Unity −x 방향(yaw_u=−90°).

**차량 상수** (원본은 `ros_ws/src/car_bringup/config/mount.yaml`):
- 축간거리 L = 0.56 m
- 뒷축 → 앞범퍼 0.78 m
- 카메라: 뒷축 바로 위(앞 0 m), 높이 0.75 m (2026-11-21 수정, 이전 문서값 뒷축 앞 0.56 m)
- 기준 경로: `track/lane2_centerline.csv` (2차선 중심선, 반시계, 39.78 m, 5 cm 간격, 열 `s_m,x_m,y_m,heading_deg,curvature_1pm`, s=0 = 출발 시 앞범퍼 위치)
- 출발 자세(base_link): (1.265, 6.659, −90°)
  - ⚠ 11/21 calib_check 분석에서 실제와 약 0.9 m 차이 가능성이 나왔습니다. PLAN P2에서 확정합니다.

## 2. 상태·입력·출력 정의

### 2.1 상태 x (plant의 실제 상태, Unity 차량 모델과 같은 정의)
```
x = [X, Y, θ, v, δ]ᵀ
Ẋ = v cos θ
Ẏ = v sin θ
θ̇ = v tan δ / L
v̇ = (v_cmd − v) / τ_v                       ← 속도 루프 포함 구동계 1차 근사
δ̇ = sat_{±δ̇max}((δ_cmd − δ) / τ_δ)          ← 조향 위치 루프 1차 근사 + 속도 제한
+ 입력 지연 T_u, 측정 지연 T_y (PLAN P4에서 측정)
```
- τ_v, τ_δ, δ̇max, δ 한계, 데드밴드, T_u, T_y는 PLAN P4 시스템 식별로 `plant_params.yaml`에 넣어 넘깁니다.
- 그림의 `ẋ = Ax + Bu`는 이 비선형 모델을 기준 경로 주변에서 선형화한 것입니다. 선형화 좌표로는 아래 Frenet 오차 좌표를 권장합니다.
  ```
  e = [e_y, e_θ, e_v, e_δ]
  e_y = 경로 법선 방향 횡오차(좌 +), e_θ = θ − θ_path(s), e_v = v − v_ref, e_δ = δ − δ_ff(s)
  ```

### 2.2 입력 u (Unity → plant)
기본 모드 `KINEMATIC`: **u = [v_cmd (m/s), δ_cmd (rad)]**
- plant의 구동 계층이 v_cmd를 속도 루프(엔코더)로, δ_cmd를 조향 위치 루프(POT)로 실행합니다. 이 내부 루프는 plant의 일부이며, 그 동특성이 위 τ_v, τ_δ입니다.
- 포화: |δ_cmd| ≤ δmax (P3에서 실측), 0 ≤ v_cmd ≤ v_max(시험 단계별 제한). 포화된 실제 적용값은 `/plant/u_applied`로 되돌려 줍니다. observer의 Bu에는 **u_applied**를 쓰는 것을 권장합니다.

시험·식별용 모드 `RAW`: **u = [pwm (−255..255), pot_target (ADC 0..1023)]**
- 내부 루프를 우회하거나 일부만 씁니다. P4 식별과 고장 진단에만 씁니다.

주기와 안전:
- 권장 30 Hz(y와 같음), 최소 20 Hz.
- 마지막 u를 받은 뒤 **0.3 s** 동안 새 u가 없으면 plant가 정지합니다(v=0, δ 유지).
- 정지 조건: e-stop(Space), u 워치독, y 끊김 0.5 s(설정 가능), 트랙 범위 이탈.

### 2.3 출력 y (plant → Unity)
**주 측정** `y = [X, Y, θ]` (ArUco, 30 Hz, 마커 없으면 valid=false) → `C = [I₃ 0]`

**보조 측정** (별도 토픽, 각자 stamp, 20 Hz):
- `v_enc`: 뒷바퀴 엔코더로 구한 속도 → C에 v 행 추가
- `δ_pot`: 조향 POT를 `steer_calib`로 바꾼 값 → C에 δ 행 추가
- 보조 측정을 쓰면 `C = I₅`

관측성 메모:
- 주 측정만으로도 주행 중에는 v, δ가 관측 가능합니다(θ̇ = v tanδ / L).
- **정지 상태나 저속에서는 δ가 관측 불가능합니다.** 출발 직후 안정성을 위해 δ_pot 채널 사용을 권장합니다.

측정 잡음:
- `/plant/y`에 매 프레임 3×3 공분산(X, Y, θ)을 넣습니다. KF observer의 R_k로 바로 쓸 수 있습니다.
- Luenberger로 갈 경우 정지 잡음 통계(P2에서 측정)를 참고값으로 줍니다.

시각:
- y의 stamp = **카메라 노출 시각**(V4L2 버퍼 시각)이며, 발행 시각이 아닙니다.
- 처리 지연(현재 중앙값 약 33 ms)은 `latency` 필드에 넣습니다.
- observer는 stamp 기준으로 지연 보상을 할 수 있습니다.

## 3. ROS 2 토픽 (Humble)

| 토픽 | 방향 | 메시지 | 주기 |
|---|---|---|---|
| `/plant/u` | Unity → plant | `car_msgs/PlantInput` | 20~30 Hz |
| `/plant/u_applied` | plant → Unity | `car_msgs/PlantInput` (포화·안전 적용 후 실제로 보낸 값, stamp = 시리얼 전송 시각) | u마다 |
| `/plant/y` | plant → Unity | `car_msgs/PlantOutput` | 30 Hz (카메라 프레임마다) |
| `/plant/y_aux` | plant → Unity | `car_msgs/PlantAux` | 20 Hz |
| `/plant/status` | plant → Unity | `car_msgs/PlantStatus` | 10 Hz + 변화 시 |
| `/car/state` | (모니터링) | `car_msgs/CarState` | **observer 입력 금지** (§0.1) |

전송 방식(제안, Unity 개발자와 결정): Unity Robotics `ROS-TCP-Connector` + `ros_tcp_endpoint`, 또는 `rosbridge`. 이 규약은 토픽과 메시지만 고정하고 전송 방식은 바꿔도 됩니다.

### 3.1 메시지 초안
```
# PlantInput.msg
std_msgs/Header header     # stamp = Unity가 u를 계산한 시각 (u_applied 에선 시리얼 전송 시각)
uint32 seq
uint8  MODE_KINEMATIC=0
uint8  MODE_RAW=1
uint8  mode
float64 v_cmd              # m/s     (KINEMATIC)
float64 delta_cmd          # rad 좌+ (KINEMATIC)
int16   pwm                # RAW
int32   pot_target         # RAW
bool    saturated          # u_applied 에서만: 포화/안전으로 값이 바뀌었으면 true

# PlantOutput.msg
std_msgs/Header header     # stamp = 카메라 노출 시각, frame_id = "track"
uint32 frame_seq
bool   valid               # 이 프레임에서 자세가 측정됐는가 (false 면 아래 값 무시)
float64 x                  # m
float64 y                  # m
float64 theta              # rad
float64[9] cov             # 3x3 row-major (x, y, theta)
uint8  n_markers           # 이 프레임에서 쓴 마커 수
float64 reproj_rms_px      # 이 프레임 재투영 RMS
float64 latency            # s, stamp → 발행
# 편의 채널 (측정 자세의 기하 변환일 뿐, 필터 아님)
float64 s                  # m, lane2 중심선 위 투영 호길이
float64 e_y                # m, 횡오차 좌+
float64 e_theta            # rad

# PlantAux.msg
std_msgs/Header header     # stamp = 보드 millis 를 ROS 시각으로 맞춘 값
float64 v_enc              # m/s (엔코더 차분, 창 평균 — 운동모델 없음)
int32   enc_count
bool    enc_valid
float64 delta_pot          # rad 좌+
int32   pot_adc
bool    pot_valid

# PlantStatus.msg
std_msgs/Header header
bool armed                 # u 를 받아 실행 중
bool estop
string stop_reason         # "", "estop", "u_timeout", "y_lost", "out_of_track"
float64 u_age              # s
float64 y_age              # s
```

## 4. 오프라인 기록 형식 (인계 데이터)
주행마다 `sessions/<세션>/plant/` 아래에 둡니다. ROS bag 원본도 함께 넘깁니다.

| 파일 | 열 |
|---|---|
| `y.csv` | `t,frame_seq,valid,x,y,theta,cxx,cxy,cxt,cyy,cyt,ctt,n_markers,reproj_rms_px,latency,s,e_y,e_theta` |
| `y_aux.csv` | `t,v_enc,enc_count,enc_valid,delta_pot,pot_adc,pot_valid` |
| `u.csv` | `t,seq,mode,v_cmd,delta_cmd,pwm,pot_target` (받은 값) |
| `u_applied.csv` | 같은 열 + `saturated` (실제로 보낸 값) |
| `status.csv` | `t,armed,estop,stop_reason` |
| `meta.yaml` | 지도 버전, mount, steer_calib, plant_params 버전, git 해시 대신 파일 해시 |

`t` = ROS 시각(s, float). 모든 파일이 같은 시계를 씁니다.

## 5. 미정 · Unity 개발자와 정할 것
- [ ] Unity 트랙 원점·축이 §1 가정(매트 왼쪽 아래 모서리)과 같은지
- [ ] 전송 방식 (ROS-TCP-Connector / rosbridge)과 Unity·ROS PC 구성(같은 PC인지)
- [ ] u 주기, Unity 쪽 제어 주기
- [ ] observer에 보조 측정(v_enc, δ_pot)을 쓸지
- [ ] Frenet 오차 좌표 사용 여부, v_ref 프로파일 정의
- [ ] Unity 차량 모델이 §2.1 형태(1차 지연 + 속도 제한 + 지연)를 받아들일 수 있는지
