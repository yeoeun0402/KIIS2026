# 명령어 모음 (demo ROS)

모든 명령은 터미널을 새로 열었다면 먼저:
```bash
source ~/demo/ros_ws/env.sh
```

## 1. 주행 + 실시간 위치 추정 (보통 이것만 쓰면 됨)
```bash
~/demo/ros_ws/drive_ros.sh --max-speed 40 --landmarks ~/track_landmarks.csv --rviz
```
- 카메라, SLAM, 트랙 정합, 차량 상태(`/car/state`), bag 녹화가 다 같이 켜짐
- 키: **W/S** 속도 · **A/D** 조향 · **Space** 정지 · **Q** 종료 (Ctrl+C 도 안전하게 정리됨)
- 끝나면 요약 + 결과 영상 자동: `~/demo/sessions/<시각>/render/clean.mp4`
- 기록: `~/demo/sessions/<시각>/` (bag · slam · 주행 로그)
- 바퀴 엔코더: 주행 세션 폴더의 `encoder.csv` (t_s,board_ms,count) + bag 의 `/car/wheel_encoder`. 펌웨어를 새로 올려야 나옴 (아래 6번)

| 옵션 | 뜻 |
|---|---|
| `--landmarks <csv>` | 실측 마커 → 트랙 좌표. 빼면 출발점 기준(map) 좌표 |
| `--rviz` | rviz 창 (키 입력은 계속 터미널에) |
| `--max-speed 40 --step 10` | 최고 PWM · W 한 번당 증가량 |
| `--render clean\|full\|none` | 주행 후 영상 종류 (none = 안 만듦) |
| `--size 1280x720` | 해상도 낮추기 (기본 1920x1080) |
| `--no-bag` | 녹화 안 함 |
| `--source video --video <mkv>` | 카메라 대신 녹화 영상 (하드웨어 없는 시험) |

## 2. 수동 주행만 (위치 추정 없이, 바퀴 띄우고 모터 확인 등)
```bash
CAR_DRIVE_ROOT=~/demo/car_drive_exp_copy CAR_DRIVE_LOG=~/demo/sessions ros2 run car_drive wasd_drive --ros-args \
  -p camera:=none -p max_speed:=40 -p step:=10
```
키는 위와 같음. 기록은 `~/demo/sessions/<시각>/`.

## 3. 위치 추정만 (주행 없이, 차를 손으로 밀거나 카메라만 들고)
```bash
ros2 launch car_bringup slam.launch.py landmarks:=$HOME/track_landmarks.csv rviz:=true
```
- 종료: Ctrl+C
- 녹화된 영상으로: `source:=video video:=<mkv>`
- 녹화된 bag 다시 틀기: `source:=none` 으로 켜고 다른 터미널에서
  `ros2 bag play ~/demo/sessions/<시각>/bag --topics /camera/image_raw/compressed /camera/camera_info`

## 4. 상태 보기 (다른 터미널에서, 켜져 있는 동안)
```bash
ros2 topic echo /car/state           # x y θ v δ + 검증값 (제어기가 쓸 것)
ros2 topic echo /car/pose_track      # 트랙 좌표 차량 자세
ros2 topic echo /track/align         # 정합 잔차 (마커별 cm, RMS)
ros2 topic hz /car/state             # 약 30 Hz 이어야 함
ros2 topic echo /car/wheel_encoder   # 뒷바퀴 엔코더 누적 카운트 (1회전 = 80, 약 20 Hz)
ros2 run tf2_ros tf2_echo track base_link
```

## 5. 주행 끝난 뒤
```bash
ros2 run car_state state_report ~/demo/sessions/<시각> --plot          # v·δ·옆미끄럼 점검
ros2 run aruco_slam_ros bag_export ~/demo/sessions/<시각>/bag \
  --render clean --landmarks ~/track_landmarks.csv                         # 영상 다시 만들기 (full = 대시보드)
ls -t ~/demo/sessions | head -1                                         # 가장 최근 주행
```

## 6. 현장에서 한 번씩
```bash
~/demo/car_drive_exp_copy/flash.sh                                # 펌웨어 업로드 (엔코더 E 줄 추가판, 2026-11-21) — 주행 노드 끈 채로
~/demo/ros_ws/build.sh                                            # 파이썬 노드 바뀐 뒤 (엔코더 기록·토픽)
~/demo/ros_ws/field_check.sh --landmarks ~/track_landmarks.csv   # 준비 상태 (출발 전·도착 후)
~/demo/ros_ws/p1_camera_check.sh                                  # C920 처음 꽂았을 때
nano ~/demo/ros_ws/src/car_bringup/config/mount.yaml && ~/demo/ros_ws/build.sh   # 장착 실측값 넣기
~/demo/ros_ws/calib_steer.sh --landmarks ~/track_landmarks.csv   # 조향 보정 (순서는 실행하면 나옴)
~/demo/ros_ws/calib_steer.sh --analyze ~/demo/sessions/<시각>  # 보정 주행 다시 분석
```

## 7. 문제 생기면
```bash
pgrep -xa 'rviz2|aruco_slam_node|c920_camera|car_state_node|wasd_drive'   # 남은 노드
tail -30 ~/demo/sessions/<시각>/bringup.log                            # SLAM·카메라 쪽 로그
~/demo/ros_ws/build.sh --test                                             # 코드 고친 뒤 빌드 + 테스트
```
