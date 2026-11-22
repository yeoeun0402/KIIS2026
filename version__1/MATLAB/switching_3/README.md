# switching_3 — PP / PID / BC 블랜디드(스위칭) 조향 + Low-pass Filter

switching_2를 그대로 복사하고, **블랜디드 차의 조향각에만 Low-pass Filter(LPF)를 넣은 버전**입니다(2026-11-16).
switching_2는 바꾸지 않았습니다(LPF 없는 버전으로 그대로 남아 있음).
이 폴더에서는 "switching"과 "블랜디드"를 같은 뜻으로 씁니다.

```
δ[k]       = λ0·δ_PP[k] + λ1·δ_PID[k] + λ2·δ_BC[k]      (스위칭)
δ_blend[k] = Lowpass(δ[k])                              (블랜디드 차의 조향각 = 차량 모델 입력)
PP  → λ = [1 0 0]     PID → λ = [0 1 0]     BC → λ = [0 0 1]
```
- **LPF는 블랜디드 차에만 씁니다.** 순수 PP 차, 순수 PID 차, BC 차(레퍼런스)에는 필터가 없습니다.
- 필터는 PP_PID와 같은 논문 식(Chen et al. ITSC 2018, Eq.8–9, `lpf_window.m`)입니다.
  - y[k] = w_current·δ[k] + (1 − w_current)/(ℓ − 1) · (직전 ℓ − 1개 출력의 합)
  - 기본값은 창 길이 ℓ = 3, w_current = 0.5입니다(blended_params와 같음).
- ℓ = 1이면 필터가 없습니다. 이때는 switching_2와 결과가 완전히 같습니다.

| 폴더/파일 | 내용 |
|---|---|
| `PP/pp_steer.m` | 순수 Pure Pursuit: δ = atan(2L sin α / L_ad), 선행점 = 투영점 + Lf |
| `PID/pid_steer.m` | 순수 횡오차 PID: δ = Kp e + Ki Σe·Ts + Kd Δe/Ts, e = 0 − e_lat (필터 없음) |
| `BC/` | `test1/discrete_path_drive.m`과 모델 함수 복사본 + `bc_steer.m` (δ_BC = atan(L·κ(s(k+1))), 레퍼런스 경로를 그대로 따라가는 조향각) |
| `decide_lambda.m` | 카메라 feature(차선 3개 곡률) + BC 구간 → λ. **판단 기준은 이 파일 하나에 있음** |
| `pp_error_map.m` | PP 오차 지도: PP만으로 한 바퀴 미리 계산해서 BC 구간을 정함 (필터 없는 순수 PP) |
| `lpf_window.m` | 논문 Eq.8–9 Low-pass Filter (blended_control에서 복사). 블랜디드 차의 δ에만 씀 |
| `switch3_discrete.m` | 메인: 네 차 계산, Unity·lane_server 연결, Figure 1 + 실시간 파라미터 패널, 저장 |
| `plot_switch3.m` | 저장된 `timeseries.csv`를 읽어 Figure 1, 2를 그리고 `graph/`에 저장 (다시 그릴 때도 사용) |
| `switch3_fig1.m`, `switch3_fig2.m`, `switch3_style.m` | 그림 배치와 색 |
| `check_switch3_tuning.m` | 실행 중 패널 조작(ℓ, w_current 포함)이 다음 스텝부터 적용되는지 확인하는 테스트 |
| `plot_error_compare.m` | 궤적 오차 비교 그림: (a) PP / PID / BC, (b) 블랜디드를 같은 크기·같은 시간축·같은 세로축으로 위아래에 놓음. `plot_error_compare('<실행 시각>')` → `graph/<실행 시각>/error_compare.png`, `.svg` |
| `plot_lpf_spikes.m` | 분석 그림: LPF 때문에 오차가 갑자기 커진 곳을 맵에 빨간 점으로 표시 (아래 "오차가 튀는 곳") |
| `bicycle_discrete_step.m` | 차량 모델 (blended_control에서 복사) |

## λ 규칙 (decide_lambda.m, 2026-11-16 변경)
기본 주행은 **PP**입니다. 직선은 **PID**, PP로 달려도 오차가 커지는 구간은 **BC**를 씁니다.

| 상황 | 판단 |
|---|---|
| 보이는 차선이 모두 직선 (\|κ\| ≤ κ_th) | **PID** (패널에서 PP로 바꿀 수 있음) |
| 그 밖 (곡선, 섞임, 안 보임) + 차가 BC 구간 안 | **BC** |
| 그 밖 | **PP** |

- κ는 lane_server가 보낸 값입니다. 곡선/직선 판단은 MATLAB이 κ_th로 다시 합니다.
- **BC 구간 (pp_error_map.m)**
  1. 실행을 시작할 때 PP 차 하나를 같은 출발점, 같은 v, 같은 Lf로 한 바퀴 미리 계산합니다(0.3 s 정도).
  2. 궤적 오차 |0 − e_lat| ≥ e_BC인 곳과, 그곳보다 lead [m] 앞(차가 다가오는 쪽)까지를 BC 구간으로 정합니다.
  3. 패널에서 v, Lf, e_BC, lead를 바꾸면 BC 구간을 다시 계산합니다.
- **미리 바꾸는 이유:** BC는 되먹임이 없어서 이미 생긴 오차를 되돌리지 못합니다. 그래서 오차가 커진 **뒤에** BC로 바꾸면 오차가 그대로 남거나 더 커집니다. 2026-11-16 비교에서 이렇게 하면 RMS가 0.1–1.5 m까지 나빠졌습니다.
- 처음 기준값: κ_th 0.05, e_BC 0.02 m, lead 0.5 m
- 첫 판단이 오기 전에는 PID를 씁니다.
- 카메라 판단이 처음 2스텝(10 s) 동안 하나도 오지 않으면, 이유를 알려 주고 멈춥니다. 2026-11-16에 lane_server를 켜지 않고 실행했을 때 한 바퀴 내내 PID로만 달린 적이 있어서 추가했습니다.

## 실행
1. **Unity Play**
   - Car_Camera → Car Camera Capture에서 Period를 0.1로 두고, Send To Lane Server를 켭니다.
2. **판단 서버** (터미널):
   ```
   C:\Users\haepi\venvs\kiis_yolo\Scripts\python.exe C:\Users\haepi\OneDrive\Desktop\KIIS2026_Unity\lane_yolo\lane_server.py
   ```
   - "Unity 연결됨"이 나오면 됩니다.
   - 포트 55003은 feature 사진 저장용으로 함께 열립니다.
3. **MATLAB**
   ```matlab
   cd MATLAB; setup_paths; addpath('switching_3')
   switch3_discrete                                     % 카메라 판단, 자동 0.8 m/s 한 바퀴
   switch3_discrete('input', 'keyboard')                % Unity에서 Space 한 번 = 한 스텝
   switch3_discrete('decision', 'ref', 'useUnity', false)   % 카메라 없이 경로 곡률로 판단 (확인용)
   switch3_discrete('eBC', 0.015, 'lead', 1.0)          % 처음 값 바꾸기
   switch3_discrete('ell', 3, 'wcur', 0.7)              % LPF 값 바꾸기
   switch3_discrete('ell', 1)                           % LPF 끄기 (switching_2와 같음)
   plot_switch3('20261116_092549')                      % 저장된 실행을 다시 그리기
   ```
   - switching_2와 같은 MATLAB 세션에서 번갈아 실행해도 됩니다. 두 폴더의 보조 함수(`decide_lambda` 등) 이름이 같지만, 실행할 때마다 자기 폴더를 경로 맨 앞에 다시 넣습니다.

## 실행 중 조절 (Figure 1 왼쪽 아래 패널, 다음 스텝부터 적용)
- **슬라이더/입력칸:** v, Lf, Kp, Ki, Kd, κ_th, e_BC, lead, ℓ (블랜디드 LPF 창 길이, 정수 1–21), w_current (블랜디드 LPF, 0 초과 1 이하)
  - v는 자동 버전만 해당합니다. 키보드 버전에서는 ↑/↓로 속도를 바꿉니다.
- **선택 상자**
  - 모두 직선일 때 쓸 제어기: PID / PP
  - 제어기 고정: 자동 / PP / PID / BC. 기준을 찾을 때 비교용입니다.
- 이 슬라이더는 **실제로 쓰는 PP와 PID**에 연결됩니다.
  - blended_discrete에서 Kp/Ki/Kd 슬라이더는 PP_PID 안의 PID 항 게인이었습니다. 그래서 파란 PID 차는 바뀌지 않았습니다.
- 바꾼 값은 `timeseries.csv`의 `p_*` 열에 스텝마다 남습니다.

## 그림
**Figure 1**
- **색:** PP 초록, PID 파랑, BC 노랑, 블랜디드 보라
- **왼쪽:** 맵
- **오른쪽:**
  1. 입력 조향각 δ_cmd (블랜디드는 LPF 후 값. LPF 전 δ는 그림에 없고 `timeseries.csv`의 `bl_delta_blend_rad`에 있음)
  2. 출력 조향각 δ
  3. 경로: 궤적 오차 0 − e_lat
- **배경 띠:** 블랜디드 차가 그 시각에 쓴 제어기의 색입니다(PP 초록, PID 파랑, BC 노랑).
  - 한 바퀴 내내 파랑이면 계속 PID였다는 뜻입니다. 카메라 판단이 안 왔을 때 그랬습니다.
- **맵의 옅은 노란 띠:** BC 구간(PP 오차 지도)입니다. ③의 점선은 ±e_BC입니다.

**Figure 2:** 블랜디드(보라)와 레퍼런스(갈색)를 비교하고, 오차는 빨강으로 그립니다.
1. **주행 경로 차이:** 맵과 0 − e_lat
2. **입력 조향각 차이:** tracking error
3. **출력 조향각 차이**
- 레퍼런스는 경로를 그대로 따라간 BC 차이고, 같은 시각 k끼리 비교합니다.
- 오차는 레퍼런스 − 블랜디드입니다.
- **배경 띠:** Figure 1처럼 블랜디드 차가 그 시각에 쓴 제어기의 색입니다(PP 초록, PID 파랑, BC 노랑). 맵 아래 그래프와 ②, ③에 모두 있습니다.
- **맵의 띠** (2026-11-17): 트랙 바깥 색 띠 = 그 위치에서 블랜디드가 쓴 제어기(배경 띠와 같은 색), 검은 눈금 = 전환 지점, 트랙 안쪽 글자 = 그 구간의 제어기(0.8 m 이상인 구간). `switch3_mode_ribbon.m`

두 그림 모두 체크박스로 선을 켜고 끕니다(Figure 2의 "구간 배경" = 배경 띠). 범례는 그래프 밖 맨 위에 한 줄로 둡니다.

**축 (2026-11-17):** 맵에는 x, y축(눈금·이름)을 표시하지 않습니다. 그래프 세로축은 같은 양이면 같은 범위·같은 눈금 간격을 씁니다: 조향각 5° 간격(Figure 1 ①②, Figure 2 ②③ 각각 같은 범위), 궤적 오차 0.02 m 간격. 눈금이 12칸을 넘으면 다음 간격(10°, 0.05 m …)으로 넘어갑니다(`switch3_yaxis.m`).

## 저장
`data/<실행 시각>/`
- `switching3.xlsx`: 데이터 저장 형식.xlsx 구성을 따릅니다. 행 = 시간 주기 k(0부터, 카메라 사진 k와 같음)입니다.

| 시트 | 내용 |
|---|---|
| 블랜디드 | 주행 경로 차이 [m], tracking error (입력 조향각 차이) [deg], 조향각 오차 (출력 조향각 차이) [deg], λ0 λ1 λ2 |
| 주행 경로 | PP, PID, BC, 블랜디드의 x, y [m] |
| 입력 조향각 | PP, PID, BC, 블랜디드 δ_cmd[k] [deg] (블랜디드는 LPF 후), 마지막 열 = 블랜디드 LPF 전 δ |
| 출력 조향각 | PP, PID, BC, 블랜디드 δ[k+1] [deg] (시각 t + Ts) |
| 횡오차 | PP, PID, BC, 블랜디드의 횡오차 0 − e_lat [m]: 뒤 차축 중앙과 레퍼런스 경로 사이, 레퍼런스 − 주행 (진행 방향 오른쪽 +), 시각 t + Ts (2026-11-17 추가) |
| 카메라 | 차선별 κ, 상태, BC 구간 여부, 규칙, 판단 대기 시간, 시간 초과, 사진 저장 여부 |

- `timeseries.csv`: 모든 값(rad 단위). 블랜디드 차가 계산한 세 후보 조향각, 받은 LANE 줄, 스텝별 파라미터를 함께 넣습니다.
  - 블랜디드: `bl_delta_blend_rad` = λ로 합친 δ (LPF 전), `bl_delta_in_rad` = LPF 후 (차량 모델 입력)
  - `p_ell`, `p_wcur` = 그 스텝의 LPF 값
- `summary.txt`: 궤적 오차 RMS/최대, 블랜디드 vs 레퍼런스 차이, 전환 횟수, 구간 비율
- `pp_error_map.csv`: 경로점마다 BC 구간인지 표시하고, 그때의 e_BC와 lead를 함께 저장합니다.
- `features/k000162_PID_to_PP.jpg`: λ가 바뀐 스텝의 Car_Camera 사진에 detection을 그린 것
  - 표시: 차선 마스크(outer 빨강, dashed 초록, inner 파랑), 차선별 곡선/직선과 κ, 판단
  - lane_server가 최근 60장을 메모리에 들고 있다가 MATLAB 요청이 오면 저장합니다.

`graph/<실행 시각>/figure1.png`, `figure2.png`: `plot_switch3`가 저장된 데이터로 그린 그림입니다.
- 같은 이름의 `.svg`(벡터 그림)도 함께 저장합니다. 선과 글자는 벡터이고, 맵 사진만 그림으로 들어갑니다. 체크박스는 빠집니다.

## 결과: 블랜디드에 LPF를 넣었을 때 (2026-11-16, 경로 곡률 판단, 0.8 m/s 한 바퀴)
확인한 것 (`matlab -batch`, Unity 없이):
- ℓ = 1로 돌리면 switching_2와 모든 열(59개)이 완전히 같습니다.
- 기본값(ℓ 3, w 0.5)에서도 순수 PP, 순수 PID, BC 차는 switching_2와 완전히 같습니다(차이 0).
- 블랜디드 입력 조향각을 LPF 전 δ와 `p_ell`, `p_wcur`로 따로 다시 계산해도 완전히 같습니다.
- `check_switch3_tuning` 통과: 실행 중 ℓ을 1로 바꾸면 다음 스텝부터 필터가 꺼집니다.

| 블랜디드 | LPF 없음 (ℓ 1 = switching_2) | LPF (ℓ 3, w 0.5) |
|---|---|---|
| 한 바퀴 RMS (직선 = PID, 기본) | 0.0160 m | 0.0200 m |
| 최대 | 0.067 m | 0.077 m |
| 한 바퀴 RMS (직선 = PP) | 0.0088 m | 0.0141 m |
| 입력 조향각 변화율 RMS | 12.6 °/s | 7.7 °/s |
| 전환 순간 가장 큰 조향각 점프 | 11.5° | 3.6° |

- **조향은 부드러워졌지만, 궤적 오차는 커졌습니다.**
- 제어기를 하나로 고정하고 LPF만 넣어서 비교한 결과입니다(블랜디드 차 = 그 제어기 + LPF).

| 고정 | LPF 없음 | LPF |
|---|---|---|
| PP | 0.0119 m | 0.0100 m (좋아짐, PP_PID와 같은 경향) |
| PID | 0.0341 m | 0.0354 m |
| BC | 0.0006 m | 0.113 m |

- 오차가 커진 주된 이유는 **BC 구간**입니다.
  - BC는 되먹임 없이 정확한 조향각을 제때 내는 방식입니다. 그래서 LPF의 지연(ℓ 3, w 0.5에서 약 1.5스텝 = 0.15 s, 0.8 m/s에서 약 12 cm)이 그대로 경로 이탈로 남습니다.
  - PP는 선행점 때문에 원래 조금 일찍 꺾습니다. LPF 지연이 이것을 상쇄해서 PP만 쓸 때는 오히려 좋아집니다.
- w_current를 키우면 지연이 줄어 LPF 없음에 가까워집니다: w 0.7 → 0.0165 m, w 0.9 → 0.0161 m.

**오차가 튀는 곳 (LPF 때문에 새로 생긴 피크, LPF 없음 대비)**

그림: `graph/분석_LPF오차급증_20261116.png` (벡터: 같은 이름 `.svg`, 구간 표: 같은 이름 `.csv`)
- 왼쪽 맵: 오차가 갑자기 커진 위치 = 빨간 점, LPF 없어도 큰 곳 = 회색 점(참고)
- 오른쪽: ① 궤적 오차 (LPF / LPF 없음), ② 입력 조향각 (LPF 후 / LPF 전), 배경 = 쓴 제어기
- 다시 만들기: `plot_lpf_spikes` (경로 곡률 판단으로 LPF 없음·LPF를 한 번씩 계산해 같은 시각끼리 비교. 약 1분)
  - `plot_lpf_spikes('wcur', 0.7, 'eTh', 0.025)`처럼 LPF 값과 기준(기본 |오차| > 3 cm)을 바꿀 수 있습니다.
  - 카메라 판단 실행은 두 번 돌리면 판단이 달라져서 같은 시각끼리 비교할 수 없으므로, 경로 곡률 판단으로 계산합니다.

| 시각 | 맵 위치 (x, y) | 최대 오차 LPF / 없음 | 바로 앞 전환 |
|---|---|---|---|
| t 27.0 s | (−7.17, −0.06) 왼쪽 직선, 횡단보도 바로 위 | 41 / 11 mm | BC → PP (t 26.6 s) |
| t 30.1 s | (−6.98, −2.50) 왼쪽 아래 코너 입구 | 49 / 16 mm | BC → PP (t 29.5 s) |
| t 42.9 s | (2.37, −4.58) 아래 S자 오른쪽 | 33 / 7 mm | BC → PP (t 42.4 s) |

- 세 곳 모두 BC 구간이 끝나고 PP로 넘어간 뒤 0.4–0.6 s에 생깁니다.
- BC 구간 안에서 늦게 꺾어 생긴 오차를 BC가 되돌리지 못하고, PP로 넘어간 뒤에는 PP의 수정도 LPF 때문에 늦어서입니다.
- BC 구간 안에서도 오차가 이미 약 두 배입니다(예: 11 → 29 mm, 15 → 30 mm).

## LPF 없는 결과와 BC 구간 분석
switching_2의 README(`../switching_2/README.md`)에 있습니다. switching_3에서 `'ell', 1`로 돌리면 같은 결과가 나옵니다.
