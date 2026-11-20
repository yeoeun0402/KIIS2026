# switching — PID ↔ PP_PID 전환 제어 (카메라 차선 인식)

- 기본 주행은 **PID**입니다.
- Car_Camera 사진을 YOLOv8-seg로 보고, 보이는 차선(바깥 실선, 중앙 점선, 안쪽 실선)이 **모두 곡선**이면 **PP_PID**로 바꿉니다.
- 하나라도 직선이거나 하나도 안 보이면 **PID**로 바꿉니다.
- 비교를 위해 **PID만**, **PP_PID만**, **Switching** 세 차를 같은 출발점·같은 가속도로 나란히 계산합니다.
  - 제어기와 차량 모델은 `blended_control/`의 것을 그대로 씁니다.
  - Unity에는 Switching 차를 보냅니다.

## 실행 순서
1. **Unity Play**
   - Car_Camera → Car Camera Capture에서 Period 0.1(= Ts)로 두고, Send To Lane Server를 켭니다.
2. **판단 서버** (터미널, `KIIS2026_Unity/lane_yolo` 폴더에서):
   ```
   C:\Users\haepi\venvs\kiis_yolo\Scripts\python.exe lane_server.py
   ```
   "Unity 연결됨"이 나오면 됩니다.
3. **MATLAB**
   ```matlab
   cd MATLAB; setup_paths; addpath('switching')
   switch_discrete                      % 카메라 판단, 0.8 m/s 한 바퀴
   switch_discrete('decision', 'ref', 'useUnity', false, 'realtime', false)   % 카메라 없이 경로 곡률로 판단 (확인용)
   ```

## 한 스텝
MATLAB이 상태 x[k]를 보내면 다음 순서로 진행됩니다.
1. Unity가 사진 k를 찍어 lane_server로 보냅니다.
2. lane_server가 YOLO로 차선 3개를 찾고, 바닥 좌표에서 곡률을 계산해 판단합니다(|κ| > 0.05이면 곡선).
3. `LANE,k,...` 줄이 Unity를 거쳐 MATLAB으로 옵니다.
4. MATLAB은 이 줄이 올 때까지 **기다렸다가** 그 판단으로 u[k]를 계산합니다. 그래서 같은 조건이면 결과가 같습니다.
5. 0.5 s 안에 오지 않으면 직전 판단을 유지하고 `sw_timeout`에 기록합니다.

Switching 차는 매 스텝 PID와 PP_PID를 둘 다 계산합니다. 안 쓰는 쪽의 적분이나 필터도 계속 갱신되므로, 전환하는 순간 조향이 처음부터 다시 시작하지 않습니다.

## 결과
`MATLAB/results/<날짜_시간>_switching_<camera|ref>/`
- `timeseries.csv`: 세 차의 위치, 조향각, 조향각 명령, 궤적 오차, 그리고 `sw_mode`, `sw_timeout`, `sw_wait_ms`, `sw_lane`(받은 판단 줄)
- `summary.txt`: 궤적 오차 MSE, RMS, 최대값, 조향각 명령 RMS, 전환 횟수, PP_PID 비율
- `result.png`: 맵(세 궤적), 궤적 오차, 조향각 명령, Switching 차가 쓴 제어기

## 첫 결과 (2026-11-15, 0.8 m/s 한 바퀴, Ts 0.1 s)
| 제어기 | 궤적 오차 RMS | 최대 |
|---|---|---|
| PID | 0.0341 m | 0.086 m |
| PP_PID | 0.0102 m | 0.027 m |
| Switching (카메라 판단) | 0.0227 m | 0.083 m |
| Switching (경로 곡률 판단) | 0.0206 m | 0.081 m |

- **전환 횟수:** 카메라 판단은 16번, 경로 곡률 판단은 6번입니다. 직선과 곡선이 바뀌는 구간에서 판단이 왔다 갔다 합니다.
- **해석:** 지금 게인으로는 직선에서도 PID 오차가 PP_PID보다 큽니다. 그래서 Switching의 오차는 두 제어기의 중간에 있습니다.
