# [demo] 차량 상태 (자전거 모델, 기준점 base_link = 뒷바퀴 축 중심)
# stamp = 이 상태가 나온 카메라 프레임 시각 (V4L2 버퍼 시각). 제어기는 now 까지 앞으로 예측해서 쓸 것.
std_msgs/Header header      # frame_id = track (실측 정합 있을 때) | map

float64 x                   # m
float64 y                   # m
float64 theta               # rad, x 축 기준 반시계 +, (-pi, pi]
float64 v                   # m/s, 진행 방향 + (후진 −) — EKF 추정
float64 delta               # rad, 유효 조향각(자전거 모델), 좌 + — POT + steer_calib.yaml. 보정 없으면 NaN

float64 yaw_rate            # rad/s — EKF 추정
float64 v_lat               # m/s, 옆 방향 속도 (미끄러짐 없으면 ~0 → 장착 yaw·정합 점검용)
float64 delta_from_motion   # rad, atan(L·yaw_rate/v) — 검증용, |v| 작으면 NaN
int32 steer_pot             # 조향 POT 원시값 (없으면 -1)

float64 x_std               # m   (EKF 1σ)
float64 y_std
float64 theta_std           # rad
float64 v_std               # m/s

bool pose_valid             # 최근 pose_timeout 안에 자세 들어옴
bool delta_valid            # POT 최근 값 + 보정 범위 안
float64 pose_age            # s, 마지막 자세 stamp 부터 발행 시각까지
