# version__1 — Unity 시뮬레이션 + 실차 동시 수동 주행 (twin)

Unity Game 창에서 누른 키(W/A/S/D/Space/Q)로 **시뮬레이션 차와 실차가 같이 움직이고**,
실차는 C920 영상으로 ArUco localization을 한 뒤 **두 궤적의 횡방향 오차를 비교**합니다.

- 원본 복사: `~/unity_simulation/KIIS2026-main` (2026-11-21). 원본은 그대로 두고 여기서만 작업합니다.
- 실차 쪽 코드는 `~/demo`에 있습니다 (따로 git 저장소).
- 백업: 이 폴더 자체가 git 저장소입니다 (`git log`로 기록 확인, `git diff`로 바뀐 내용 확인).

## 1. 전체 흐름

```
 Unity Game 창 키 (TwinInput.cs)
        │ TCP 55001 (같은 연결로 양방향)
        ▼
 MATLAB twin_drive.m ──── 시뮬 차 계산 (twin_model_step: PWM·조향 펄스 → 자전거 모델, 입력은 실차 APPLY)
        │   │                └─ 상태를 Unity로 → 차 위치 + 지나온 궤적(빨강) + 기준선(노랑) 표시
        │   └─ 끝날 때: timeseries.csv · summary.json · trajectory_track.png · Unity Top 이미지(unity_top.png)
        │ TCP 55004: 키(KEY,w …) ▶            ◀ APPLY,<키>,<PWM> (실차가 실제로 적용한 키·속도)
        ▼
 실차 노드 (~/demo/drive.sh --twin → wasd_drive + twin_link.py) ──시리얼──▶ 아두이노 (모터·조향)
        │   └─ C920 녹화 (video.mkv)
        ▼ 주행이 끝나면 (drive.sh가 자동으로)
 결과 영상 (render/localization.mp4) → localization 숫자·이미지 (map/trajectory.csv, map_overview.png)
        ▼
 analysis/compare_twin.py → 두 궤적 비교 (results/<run>/compare/)
```

### 속도 조절은 실차 값 하나를 따름 (APPLY)

- Unity 키는 MATLAB을 거쳐 실차에만 보냅니다. 실차 노드(`Drive.key`)가 적용할 때마다 `APPLY,<키>,<PWM>`을 돌려보내고,
  MATLAB 시뮬은 **이 줄만 보고** 움직입니다 (`twin_apply.m`). 그래서 W/S 한 번 = ±10, Space = 0, 최고 PWM 한계가 실차와 항상 같습니다.
- 실차 터미널에서 누른 W/S/Space, 연결 끊김·0.5초 무신호 안전 정지도 시뮬에 똑같이 들어갑니다 (예전에는 시뮬에 안 들어가 두 차가 갈라짐).
- Unity 화면 아래 노란 글씨 `PWM +30 / 40 (real car)` = 실차 터미널의 `속도 +30`과 같은 값.
- `keys.csv`의 `src`: `unity` = Unity에서 받은 키, `car` = 실차가 적용해서 시뮬에 들어간 키.

### 웹 대시보드 (twin.sh가 자동으로 켬, http://127.0.0.1:8765)

| 위치 | 내용 | 출처 |
|---|---|---|
| 위 왼쪽 | 실차 카메라 (15 fps) | 주행 노드 ffmpeg가 녹화와 함께 `/dev/shm/twin_live/cam.jpg`를 덮어씀 (`live_jpeg`) |
| 위 오른쪽 | Unity Game 화면 (10 fps) | `TwinStream.cs`가 `/dev/shm/twin_live/sim.jpg`를 덮어씀 (폴더가 있을 때만) |
| 아래 왼쪽 | 무게 조절 (기본 15 kg) + 상태 | 바꾸면 MATLAB에 `MASS,<kg>` → `twin_mass.m` (주행 중에도 바로 반영) |
| 아래 오른쪽 | 지도: 시뮬 경로 **빨강**, localization 경로 **노랑** | MATLAB `SIM` 줄 · `dashboard/live_loc.py` |

- 실시간 localization(`live_loc.py`): 카메라 JPEG에서 마커를 찾아, 이전 build_map 지도(가장 최근 FAIL 아닌 `sessions/*/map`)에
  맞는 바닥 위 자세 (x, y, θ)를 매 프레임 구함. 녹화 영상 시험: build_map 결과와 차이 중앙값 1 cm, RMS 9 cm.
  마커 1개만 보이면 모호해서 흐린 점으로만 표시. 주행 뒤 build_map 결과(`map/trajectory.csv`)가 더 정확함.
- 키보드: 웹 페이지에서도 W/A/S/D/Space/Q가 Unity Game 창과 똑같이 동작.
- 대시보드는 twin.sh가 끝나도 켜 둠 (경로를 계속 볼 수 있음). 끄기: 터미널에서 `pkill -f twin_dash.py`.

### 칼만 필터 (EKF, 기본) — 시뮬을 실차 쪽으로 보정

루엔버그 옵저버(아래)는 측정을 늘 같은 무게로 믿어서, 마커가 멀리 한 줄로만 보이는 아래 직선에서 1~2 m 틀린 localization 을
그대로 따라가 시뮬을 매트 밖으로 끌고 갔습니다 (20261121_224504). 그래서 기본 추정기를 칼만 필터로 바꿨습니다.

    상태 X = [x y yaw δ vs] · 예측: 차량 모델 + P ← F P Fᵀ + Q dt (조향 키마다 δ 분산 추가)
    갱신: z = localization [x y yaw], 측정 공분산 Rz = σ²(JᵀJ)⁻¹ (live_loc.py, 픽셀 잔차 자코비안 → 마커 배치가 나쁜 방향은 분산이 큼)
          K = P Hᵀ (H P Hᵀ + Rz)⁻¹, Joseph 형태 공분산, 마할라노비스² > 16.3 (99.9%) 이면 버림, 3초 넘게 다 버리면 P 키워 다시 잡기

- 파일: `MATLAB/twin/twin_ekf_predict.m`, `twin_ekf_update.m`, 값 `twin_params.m`의 `tp.ekf`, 고르기 `tp.estimator` ("ekf" | "luenberger" | "none").
- 대시보드 체크박스 = 추정기 켜고 끄기 (다시 켜면 위치 불확실성을 크게 해서 다시 잡음).
- 검증: 합성 (8~12 s 동안 측정이 0.8 m 틀리고 그 방향 분산이 큼): 최대 오차 칼만 0.05 m / 루엔버그 0.82 m.
  실제 주행 다시 돌리기 20261121_214854 (localization 이 좋은 주행): 칼만 0.13 m (최대 0.24) · 루엔버그 0.13 m (최대 0.28) · 없음 1.38 m.

### ArUco 측정 칼만 필터 (`dashboard/loc_kf.py`) — 마커 측정 잡음 줄이기 (위 MATLAB EKF 와 다른 필터)

    live_loc.py 프레임 측정 (x, y, θ, R) ─▶ loc_kf.py (선형 칼만, 상태 [x y θ vx vy ω], 차량 모델·PWM 안 씀) ─▶ KF 줄
                                      └─▶ LOC 줄 (원 측정, 그대로) ─▶ MATLAB EKF (옵저버, 위)
    KF·LOC ─▶ twin_dash.py ─▶ twin_drive.m ─▶ Unity (LocKfOverlay.cs: 주황 = 측정 + R 95% 타원, 하늘색 = 칼만 + P 95% 타원, 빨강 = 버린 측정)

- 칼만에 주는 R = live_loc 측정 공분산에서 지도 오차 바닥값(0.05 m, 1°)을 뺀 것 (바닥값은 매 프레임 새 잡음이 아니라 치우침).
- 기록: `results/loc_kf/loc_<시각>.csv` (주행마다, 프레임마다 원 측정·R 과 칼만·P·NIS 같은 줄) + `results/<run>_twin/timeseries.csv` 의 `kf_*` 열.
- 그림: `./twin.sh` 가 끝날 때 자동으로 만듦 (이번 실행의 `loc_<시각>.csv` 마다 → 같은 이름 .png · _summary.json, `results/<run>_twin/` 에도 복사).
  직접: `python3 analysis/loc_kf/plot_loc_kf.py results/loc_kf/loc_<시각>.csv`
- 잡음 시험 (참값 = build_map 궤적에 무작위 잡음 + 이상치 + 측정 빠짐): `python3 analysis/loc_kf/noise_test.py [--r-from <loc csv>]`
- 녹화 영상으로: `python3 dashboard/live_loc.py --video ~/demo/sessions/<세션>/video.mkv --map <지도> --log-dir results/loc_kf`
- 칼만 끄기: `live_loc.py --no-kf`. Unity 키: K 칼만 · M 측정 · E 타원 · J 전체.
- 결과 (2026-11-22, 세션 20261122_002434):
  잡음 시험 σ 5 cm·2°, 이상치 2 %, 빠짐 5 % (10회): 위치 RMSE 7.1 → 3.4 cm (이상치 제외), 방향 2.6 → 1.2°, 이상치 100 % 버림, 95% 타원 안 참값 97 %.
  실제 측정 R 로 같은 시험: 위치 중앙값 0.86 → 0.58 cm, 95% 타원 안 참값 96 %.
  녹화 재생: 흔들림(연속 위치 2차 차분) 4.0 → 1.0 cm, NIS 평균 1.7 (95% 선 넘는 비율 4 %, 이상적 5 %), 버림 12 / 953.

### 루엔버그 옵저버 (`tp.estimator = "luenberger"`)

같은 키를 받아도 두 차는 서로 위치를 몰라 시간이 갈수록 벌어집니다 (조향 오프셋·속도 차이·조향 센서 없음).
그래서 실시간 localization (x, y, θ)을 측정값으로 쓰는 루엔버그 옵저버를 넣었습니다 (`MATLAB/twin/twin_observer.m`).

    x̂' = f(x̂, u) + K(v) (y − ŷ)       f = 차량 모델 (twin_model_step), u = 실차가 적용한 명령, y = localization
    추정 상태: 위치 · 방향 · 조향각 δ · 속도 배율 vs (v = v_모델 · (1 + vs))
    오차 모델 (직진 근처, 차체 좌표):  ė_lat = v e_θ, ė_θ = (v/L) e_δ, ė_δ = 0  |  ė_lon = v e_s, ė_s = 0
    게인 (극배치, 속도에 따라): 옆 위치 1/tauPos · 방향-조향 s² + 2ζh ωh s + ωh² · 앞뒤-속도 s² + 2ζl ωl s + ωl²
      kDel = ωh² L / v,  kS = ωl² / v   (|v| < 0.08 m/s 이면 조향·속도 배율은 추정 안 함)

- 값: `twin_params.m`의 `tp.obs` (tauPos 1 s, ωh 1.2, ζh 0.9, ωl 0.8, ζl 0.9, 1 m·35° 넘게 튀는 측정 버림, 마커 2개 이상만).
- 측정 지연 (카메라 → MATLAB 약 0.1 s)은 추정 자세를 그만큼 거슬러 올라가 비교해서 보정.
- 켜고 끄기: 대시보드 체크박스, 또는 `twin_drive('obsEnabled', false)`. 대시보드에 추정 조향각·속도 배율 표시.
- 기록: `timeseries.csv`에 `speed_scale, obs_on, loc_*_track, inn_lon_m, inn_lat_m, inn_theta_rad` (측정·혁신), `summary.json`에 혁신 RMS.
- 검증:
  - 합성 (조향 오프셋 −6°, 속도 +25%를 모델은 모름): 20 s 뒤 위치 오차 0.006 m (옵저버 없이 2.84 m), 속도 배율·조향 오프셋 정확히 추정.
  - 실제 주행 다시 돌리기 (`twin_replay.m`, 20261121_214854, 실제 보낸 명령 + 실시간 localization): build_map 과 차이
    옵저버 없이 중앙값 1.38 m (최대 4.05 m) → 옵저버 0.13 m (최대 0.28 m). 이 주행의 속도 배율 추정 −0.22 (모델보다 22% 느림).
- ⚠ 옵저버를 켜면 시뮬 경로는 "실차를 따라가는 추정값"이 됩니다. 모델만의 예측을 보려면 끄세요 (비교 분석 `compare_twin.py`도 마찬가지).

### 무게는 어디에 들어가나

Unity 차는 물리 계산을 하지 않습니다 (`isKinematic`, MATLAB이 계산한 위치에 놓기만 함) → Unity에서 질량을 바꿔도 아무 변화가 없습니다.
노트북·카메라 무게의 영향은 MATLAB 모델 파라미터 `kv`(PWM당 속도), `tauV`(가속 지연), `delay`, `pwmDead`(출발 문턱)에 들어갑니다.
이 값들은 **장비를 다 실은 실차의 주행 기록**으로 맞춘 값이므로, 그때 무게를 `massRef`(15 kg 가정)로 둡니다.
무게를 바꾸면 (`twin_params('mass', 18)`, `./twin.sh --mass 18`, 또는 대시보드 슬라이더) `twin_mass.m`이 다시 계산합니다:

    m v' = a·PWM − b·v − crr·g·(m − massRef)      (a, b는 massRef에서 맞춘 kv·tauV로 정함)
    tauV = 0.42 s × m / 15,   pwmDead = crr·g·(m − 15) / a  ≈ +1.0 PWM / kg  (crr 0.03)

예: 20 kg → tauV 0.56 s, PWM 40에서 0.279 m/s (15 kg 0.32 m/s) · 10 kg → 0.28 s, 0.361 m/s.
`massRef`(맞출 때 실제 무게)와 `crr`(굴림 저항)은 측정하지 않은 가정값입니다 → 차 무게를 재서 `twin_params.m`에 넣으세요.

## 2. 폴더 구조

```
~/version__1/
├── README.md                      이 문서
├── twin.sh                        ★ 명령 하나로 웹 대시보드 + Unity(자동 Play) + 실차 drive.sh + MATLAB twin_drive 실행
├── dashboard/
│   ├── twin_dash.py               웹 대시보드 서버 (카메라·Unity 화면 MJPEG, 무게·키 → MATLAB, 경로 모으기)
│   ├── index.html                 대시보드 화면 (2 x 2: 카메라 | Unity / 무게 | 지도)
│   ├── live_loc.py                실시간 localization (마커 지도에 맞춘 바닥 위 자세 → 노란 경로) + 칼만 KF 줄 + CSV 기록
│   └── loc_kf.py                  ArUco 측정 칼만 필터 (live_loc · noise_test 가 같이 씀)
├── run_unity.sh                   이 폴더의 Unity 프로젝트 열기 (한글 입력기 끄고 실행)
├── .gitignore                     Library·results·영상 등 백업에서 뺄 것
├── KIIS2026_AutoCar/              Unity 프로젝트 (원본 복사 + twin 스크립트)
│   ├── Assets/
│   │   ├── Scenes/SampleScene.unity        장면 (차 시작 위치를 실차 출발 자세로 바꿈)
│   │   └── scripts/
│   │       ├── CarStateReceiver.cs         MATLAB ↔ Unity TCP (55001). "CMD,..." 줄 받기 추가
│   │       ├── PathOverlay.cs              기준선(노랑)·지나온 궤적(빨강) 그리기 (원본 그대로)
│   │       ├── CameraViews.cs              숫자키 1~5 화면 전환 (원본 그대로)
│   │       ├── CarCameraCapture.cs         차 카메라 사진 (원본 그대로, twin에서는 안 씀)
│   │       ├── Editor/TwinAutoPlay.cs      ★ twin.sh 요청 파일(twin_play.request)을 보면 SampleScene 열고 Play
│   │       ├── test1/StepInput.cs          test1용 키 (원본 그대로, twin에서는 무시됨)
│   │       └── twin/                       ★ 새로 만든 것
│   │           ├── TwinInput.cs            W/A/S/D/Space/Q → MATLAB ("KEY,w" …)
│   │           ├── TwinSnapshot.cs         Top 화면(맵 + 궤적)을 PNG로 저장 (1 px = 1 cm, 트랙 방향)
│   │           └── LocKfOverlay.cs         ArUco 측정·칼만 경로와 R·P 95% 타원 (자동으로 붙음)
│   ├── Packages/ · ProjectSettings/        Unity 설정 (원본 그대로)
│   └── Library/                            Unity가 처음 열 때 자동으로 만듦 (백업 안 함)
├── MATLAB/                        MATLAB 코드 (원본 복사 + twin 폴더)
│   ├── setup_paths.m                       경로 추가 (twin 폴더 추가함)
│   ├── vehicle_params.m · unity_link.m · path_error.m …   공용 함수 (원본 그대로)
│   ├── test1/ · Pure-Pursuit/ · PID/ · blended_control/ · switching*/   기존 제어기 (원본 그대로)
│   └── twin/                               ★ 새로 만든 것
│       ├── twin_drive.m                    실행 파일: 키 받기 → 시뮬 계산 → 실차 전달 → Unity 표시 → 저장
│       ├── twin_params.m                   파라미터 (PWM→속도, 반응 지연, 조향 속도·최대각·중립 오프셋, 주기, 출발 자세, 포트)
│       ├── twin_model_step.m               차량 모델 한 스텝 (명령 지연 + 속도 1차 지연 + 조향 인테그레이터 + 자전거 모델)
│       ├── twin_key.m                      키 하나 → 모델 입력 (실차 drive.py와 같은 규칙)
│       ├── twin_apply.m                    실차 APPLY(적용한 키·PWM) → 모델 입력 (속도는 실차 값 그대로)
│       ├── twin_mass.m                     무게 [kg] → tauV · pwmDead (massRef에서는 맞춘 값 그대로)
│       ├── twin_ekf_predict.m              칼만 필터 예측 (공분산 전파, 조향 키마다 불확실성 추가)
│       ├── twin_ekf_update.m               칼만 필터 갱신 (localization + 측정 공분산, 마할라노비스 거르기)
│       ├── twin_observer.m                 루엔버그 옵저버 한 번 (localization 측정 → 위치·방향·조향각·속도 배율 보정)
│       ├── twin_replay.m                   실차 기록 다시 돌리기 (옵저버 켬/끔) → build_map 과 비교 (오프라인 검증)
│       ├── twin_frames.m                   시뮬 좌표 ↔ 실차 트랙 좌표 변환
│       ├── twin_reference.m                기준선 불러오기 (도면 2차선 중심선)
│       ├── lane2_centerline.csv            기준선 데이터 (~/demo/track 에서 복사, 5 cm 간격, 39.78 m)
│       ├── twin_demo_keys.m                시험용 키 순서 (Unity 없이 시험할 때)
│       └── test_twin.m                     테스트 18개 (Unity·실차 없이, Python 맞추기 모델과 같은지 포함)
├── analysis/
│   ├── compare_twin.py            ★ 시뮬 궤적 vs 실차 localization 궤적 → 횡방향 오차 분석
│   ├── fit_twin_params.py         ★ 실차 주행 기록으로 시뮬 차량 모델 파라미터 맞추기 (속도·지연·조향)
│   └── loc_kf/
│       ├── plot_loc_kf.py         ArUco 칼만 기록 그림 (측정 vs 칼만, R·P 타원, NIS) + 요약 json
│       └── noise_test.py          잡음 시험: 참값 궤적 + 무작위 잡음 → 칼만 전·후 오차
└── results/                       주행 결과 (twin_drive가 만듦, 백업 안 함)
    ├── loc_kf/                     live_loc 칼만 기록 loc_<시각>.csv · 잡음 시험 noise_test_*/
    └── <날짜_시간>_twin/
        ├── timeseries.csv          주기(0.05 s)마다: 시각, 시뮬 좌표, 트랙 좌표, 속도, 조향각, PWM, 횡오차
        ├── keys.csv                받은 키와 시각
        ├── summary.json            요약 숫자 + 연결된 실차 세션 폴더
        ├── trajectory_track.png    MATLAB 그림: 트랙 위 시뮬 궤적 + 기준선
        ├── unity_top.png / .json   Unity Top 화면 (맵 + 지나온 궤적)
        └── compare/                compare_twin.py 결과 (아래 4장)
```

### 실차 쪽 (`~/demo`)에서 바꾸거나 만든 파일

| 파일 | 내용 |
|---|---|
| `~/demo/drive.sh` | `--twin` 옵션: MATLAB 키 받기 켬, 결과 영상 기본 full, 주행 뒤 localization 숫자(build_map) + 비교 자동 실행 |
| `~/demo/car_drive_exp_copy/src/car_drive/car_drive/twin_link.py` | ★ 새 파일. TCP 55004 서버: MATLAB 키 → 터미널 키처럼 처리, 0.5 s 동안 신호 없으면 정지, 끊기면 정지 |
| `~/demo/car_drive_exp_copy/src/car_drive/car_drive/wasd_drive.py` | `key_tcp_port` 파라미터 추가: 터미널 키 + MATLAB 키를 같이 받음 |
| `~/demo/car_drive_exp_copy/src/car_drive/test/test_twin_link.py` | ★ 새 파일. twin_link 테스트 |

실차 세션 폴더 `~/demo/sessions/<시각>/`에 남는 것:
`video.mkv`(원본 영상) · `drive_log.csv`(키·보낸 명령) · `session.json` · `render/localization.mp4`(마커·지도 영상)
· `map/trajectory.csv`(localization 궤적 숫자: 트랙 좌표 x, y, θ) · `map/map_overview.png`(지도 이미지) · `map/report.json`

## 3. 실행 순서

### 마커를 옮겼을 때: 마커 확인 → 지도 → 반영 → twin (단계마다 명령 한 줄)
| 단계 | 명령 | 하는 일 |
|---|---|---|
| 1. 마커 배치 확인 | `~/demo/drive.sh --no-render --preview` | 수동 주행 + 카메라 창 (보이는 마커 ID·거리·개수, 2개 이상이면 초록). 끝나면 본 마커 목록 출력 |
| 2. 지도 주행 | `~/demo/drive.sh --no-render --preview` | 출발선 2~3초 정지 → 2차선 따라 천천히 한 바퀴 → 출발선에서 Q |
| 3. 지도 만들고 반영 | `~/demo/update_map.sh` | 방금 세션으로 build_map → 점검 (마커가 2차선 바깥 선 위에 있는지) → 마커에 맞춰 회전 보정 → 그림 보고 `y` → `track.yaml` live_map 폴더에 반영 (이전 것 백업) → 같은 영상 칼만 재생 검증. 6~8분 |
| 4. twin 주행 | `cd ~/version__1 && ./twin.sh` | 바뀐 지도를 보고 대시보드 자동 재시작. Unity·MATLAB 쪽은 바꿀 것 없음 |

- 3단계 옵션: `--skip 14`(바깥 선이 아닌 곳에 세운 마커), `--align lane`(마커 대신 주행 궤적을 차선에 맞춤), `--yes`(묻지 않고 반영),
  `<세션 폴더>`(최근 세션 말고 다른 것). 점검만: `python3 -B ~/demo/track/tools/map_check.py <지도 폴더>`
- 마커 크기: `~/demo/track/track.yaml` `markers.sets.new.size_m` = 0.265 (자로 잰 27.5 cm로 만들면 지도가 약 4% 커짐, 매트 실측 전 잠정)

### 명령 하나로 (권장)

차를 출발선에 놓고 (처음에는 바퀴를 띄운 상태로):
```bash
~/version__1/twin.sh
```
- Unity가 안 열려 있으면 열고, **자동으로 Play** 합니다 (처음 열 때는 몇 분 걸림).
  - 새 스크립트를 처음 받을 때는 Unity 창을 한 번 클릭해야 컴파일되어 Play가 눌립니다.
- 이어서 실차 `drive.sh --twin --max-speed 40`이 이 터미널에서 실행됩니다. 안전 확인 `[y/N]`에 `y`.
- 실차 포트가 열리면 MATLAB `twin_drive`가 뒤에서 자동으로 켜집니다 (10~30초). 출력은 `[matlab]`으로 이 터미널에 보이고,
  `results/launch/<시각>_matlab.log`에도 남습니다.
- **Unity Game 창을 클릭**하고 W/S/A/D/Space/Q로 운전. 이 터미널의 Space(정지)·Q(종료)도 언제나 동작.
- Q 뒤에는 drive.sh가 영상·localization·비교 결과를 만듭니다. 끝나면 MATLAB 로그와 결과 폴더를 알려 줍니다.
- 옵션: drive.sh 옵션은 그대로 넘깁니다 (`--max-speed 60`, `--no-render` 등). `--no-car`는 실차 없이 Unity + MATLAB만.
- Unity는 끝난 뒤에도 Play 상태로 켜 둡니다. 다음 실행 때 그대로 다시 씁니다.

### 하나씩 (직접)

1. **실차 터미널** (차를 출발선에 놓고, 처음에는 바퀴를 띄운 상태로):
   ```bash
   cd ~/demo && ./drive.sh --twin --max-speed 40
   ```
   `● [twin] MATLAB 키 대기: 127.0.0.1:55004`가 보이면 다음으로.
2. **Unity**:
   ```bash
   bash ~/version__1/run_unity.sh
   ```
   SampleScene을 열고 Play.
3. **MATLAB**:
   ```bash
   cd ~/version__1/MATLAB && ~/MATLAB/bin/matlab
   ```
   ```matlab
   setup_paths
   twin_drive
   ```
4. **Unity Game 창 클릭** → W/S 속도 · A/D 조향 · Space 정지 · **Q 종료** (화면 5번 = 큰 Top 화면).
   실차 터미널의 Space(정지)와 Q(종료)도 언제나 동작합니다.
5. Q를 누르면: MATLAB이 결과를 저장하고 Unity Top 이미지를 받음 → 실차 노드 종료
   → drive.sh가 영상·localization 숫자를 만들고 → 비교 결과를 `results/<run>/compare/`에 저장.

실차 없이 시험: `twin_drive('useCar', false)` · Unity·실차 둘 다 없이: `test_twin`

## 4. 비교 결과 (`results/<run>/compare/`)

| 파일 | 내용 |
|---|---|
| `compare_trajectories.png` | 트랙 위 시뮬 궤적(빨강) · 실차 궤적(파랑) · 기준선(노랑) |
| `compare_lateral.png` | 위: 진행 거리 s 기준 횡오차 (시뮬, 실차, 차이) · 아래: 같은 시각 옆/앞뒤 차이 |
| `compare_on_unity.png` | Unity Top 이미지 위에 실차 궤적을 겹친 그림 |
| `compare.csv` | 실차 샘플마다 숫자 |
| `report.json` | RMS · 최대 · 평균 · 속도 보정 추정값 |

횡방향 오차 (모두 왼쪽 +, m):
- **같은 s에서 차이 (주 지표)**: 같은 진행 거리에서 `실차 횡오차 − 시뮬 횡오차` (기준선 = 도면 2차선 중심선)
- 같은 시각에서 차이: 실차 위치를 시뮬 차 진행 방향 기준으로 본 옆 거리 (속도가 다르면 앞뒤 차이가 커짐)

## 5. 모델 파라미터 맞추기

시뮬레이션 차량 모델 값(`MATLAB/twin/twin_params.m`)은 실차 주행 기록으로 맞춥니다.

```bash
python3 ~/version__1/analysis/fit_twin_params.py --real ~/demo/sessions/<세션> --sim ~/version__1/results/<run>
```

- 실차에 실제로 보낸 명령(`drive_log.csv`의 V·L·R·F)을 시뮬레이션 모델에 그대로 다시 넣고, 실차 localization 궤적과 가장 가까워지는 값을 찾습니다.
- 결과: `<세션>/twin_fit/fit.json` · `fit_replay.png` (실차 · 맞추기 전 모델 · 맞춘 모델) · `replay.csv`
- 나온 값을 `twin_params.m`에 넣습니다 (MATLAB 모델과 Python 모델은 같은 식, `test_twin` 4b에서 확인).

2026-11-21 세션 `20261121_180841`(Unity 키로 약 8 m)로 맞춘 현재 값:

| 파라미터 | 맞추기 전 (가정) | 맞춘 값 | 뜻 |
|---|---|---|---|
| kv | 0.015 | 0.0080 m/s per PWM | PWM 40 → 0.32 m/s |
| pwmDead | 0 | 0 | 바퀴가 도는 최소 PWM |
| tauV | 0.3 s | 0.42 s | 속도 1차 지연 |
| delay | 0 | 0.30 s | PWM 명령 뒤 바퀴 반응까지 |
| steerRate | 60 °/s | 30.5 °/s | 조향 모터 속도 |
| deltaMax | 30° | 20.5° | 최대 조향각 |
| delta0 | 0 | −1.6° | 조향 중립 오프셋 (오른쪽으로 조금 휨) |

같은 명령 재생 시 실차 궤적과의 위치 오차 (움직인 구간): 맞추기 전 RMS 2.90 m → 맞춘 뒤 0.14 m.

## 6. 주의

- 파라미터는 **한 번의 주행(약 8 m, PWM 최대 40)**으로 맞춘 값입니다. 더 빠른 속도, 다른 바닥, 배터리 상태에 따라 달라질 수 있으니 주행을 더 하면 다시 맞추세요.
- 실차 조향 센서(POT)가 계속 0이었습니다 → 실제 조향각은 측정하지 못하고, 궤적에서 거꾸로 추정한 값입니다.
- Unity 차 모델(BenzGTR_RideOn)은 장면에서 **0.9396배**로 줄여 축간거리를 실차 0.56 m에 맞췄습니다 (원래 0.596 m).
  줄인 뒤 크기: 길이 0.928 · 폭 0.522 · 높이 0.424 m, 뒷축 → 앞범퍼 0.748 m (실차 0.78 m: 모델 비율 차이로 3 cm 짧음).
  기준점(car 위치) = 뒷축 중앙 그대로, Car_Camera는 뒷축 바로 위 0.75 m 그대로 (부모 축소를 상쇄하도록 위치·크기 보정).
  실차 길이·폭을 재면 그에 맞춰 다시 조정할 수 있습니다.
- Unity 바닥 그림(`Assets/image.png`)을 도면에 맞게 휘어 다시 만들었습니다 (2026-11-21, `analysis/floor_align/`).
  전: 출발선 33 cm, 바깥 경계선 구간별 최대 28 cm 어긋남 → 후: 출발선 1~3 cm, 경계선 대부분 0~5 cm (원본은 git 기록에 있음).
  그림 속 주차칸 선은 약간 기울거나 휠 수 있음 (도면과 그림의 주차칸 배치가 달라 매끄럽게 맞춘 결과).
  ⚠ 예전 MATLAB 모듈(test1·PID·switching)의 `reference_path_outer_lane.csv`는 예전 그림에서 뽑은 경로라 이제 그림과 조금 어긋남 → twin처럼 `lane2_centerline.csv`(도면) 사용 권장.
- **끝낼 때는 Q** (Unity 또는 실차 터미널). 실차 터미널 Ctrl-C도 차는 멈추지만, 결과 영상 만들기 중에 Ctrl-C를 누르면 localization·비교가 만들어지지 않습니다.
- 후진(S로 PWM 음수)이 섞이면 localization(build_map)이 실패할 수 있습니다 (20261121_203331: 재투영 173 px, FAIL). 비교용 주행은 전진만 권장.
- 같은 입력을 받아도 두 차는 위치 피드백이 없어서 시간이 지날수록 벌어집니다. 운전자는 Unity를 보므로 **실차 옆에 사람이 꼭 있어야** 합니다.
- 실차 localization은 출발 자세(출발선, 2차선 가운데)를 기준으로 트랙 좌표에 맞춥니다 → 차를 정확히 출발선에 놓아야 비교가 맞습니다.
# version_1
# version_1
# version_1
# version_1
