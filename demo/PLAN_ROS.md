# ROS 2 기반 실시간 로컬라이제이션 구축 계획 (B안)

작성 2026-11-20 · 대상 ROS 2 Humble · 작업 위치 `~/demo/ros_ws` (원본 `~/car_drive_exp`, `~/aruco-slam` 은 손대지 않음)

## 0. 목표와 원칙
- 카메라 영상을 **ROS 토픽 하나로** 내보내고, 주행·SLAM·녹화·시각화가 모두 그 토픽을 구독한다 → "카메라는 한 프로세스만" 문제 해소.
- 주행 안전은 SLAM 과 분리: SLAM 노드가 죽거나 느려져도 `wasd_drive` + 펌웨어 워치독은 그대로.
- 녹화 = `ros2 bag` 하나 (영상·명령·자세 같은 시계). 나중에 bag 을 다시 틀면 **SLAM 결과가 같은 입력으로 재현**된다.
- 결과 영상(`render_clean.py` 형식)은 bag 에서 뽑는다.

## 1. 현재 환경 (확인함)
| 항목 | 상태 | 계획에 미치는 영향 |
|---|---|---|
| `usb_cam` 0.8.1 | 설치됨, `raw_mjpeg` 지원 | C920 MJPG 를 **재인코딩 없이** `CompressedImage` 로 발행 가능 (지금 mkv 녹화 기준 1080p 약 11 MB/s, 720p 약 4 MB/s) |
| `cv_bridge` | 설치됨, 그러나 시스템 OpenCV 4.5.4 / numpy 1.21 로 빌드 | SLAM 은 OpenCV 4.10(`ArucoDetector`) + numpy 1.26 필요 → **cv_bridge 쓰지 않고** `cv2.imdecode` 로 직접 디코드 |
| `compressed_image_transport`, `rosbag2`(sqlite3), `rosbag2_compression`, `tf2_ros`, `rviz2`, `rqt_image_view`, `camera_info_manager` | 설치됨 | 추가 설치 없이 시작 가능 |
| `rosbag2_storage_mcap`, `robot_localization`, `v4l2_camera` | 없음 | 필수 아님 (mcap 은 있으면 좋음: `sudo apt install ros-humble-rosbag2-storage-mcap`) |
| rclpy + aruco_slam_deps (numpy 1.26) | `live_slam.py --ros` 로 발행 동작 확인 | SLAM 노드도 같은 `PYTHONPATH` 로 실행 |
| CPU i7-9750H 12 스레드 | 1080p 프레임당: JPEG 디코드 컬러 11 ms / 흑백 5 ms, 검출+PnP+EKF 23 ms (마커 4개). 마커 20~40개면 EKF +~11 ms | 기본 **1080p** (카메라는 MJPG 1080p 30 fps 그대로). 720p 는 마커가 많아 처리율이 모자랄 때의 대비책 (§4 P2) |

## 2. 목표 구조
```
                     /camera/image_raw/compressed  (MJPG 그대로, stamp = 수신 시각)
 [C920] ─ usb_cam ─┬─ /camera/camera_info           (K: 1080p 캘리브레이션 × W/1920)
                   │
                   ├──▶ aruco_slam_node ──▶ /car/cam_pose   geometry_msgs/PoseStamped (frame map)
                   │      (최신 프레임만,     /car/path       nav_msgs/Path
                   │       QoS depth 1)      /aruco/markers  visualization_msgs/MarkerArray
                   │                         /aruco/detections (검출 ID·거리, 커스텀 없이 MarkerArray 또는 Int32MultiArray)
                   │                         /aruco/debug_image/compressed (선택, 축 그린 영상)
                   │                         TF  map → base_link (카메라 자세 × 장착 정적 TF 역변환)
                   ├──▶ rviz2 (궤적·마커·카메라 프러스텀·디버그 영상)
                   └──▶ ros2 bag record
 [키보드] ─ wasd_drive (camera:=none) ─▶ 시리얼 ─▶ 아두이노
                   └─ (추가) /car/cmd  (PWM·조향 명령 발행 → bag 에 함께 기록)
```
### 2.1 좌표계와 TF 트리 (REP-103/105)
```
track ─(정적, P4: 실측 마커 Kabsch)─▶ map ─(SLAM 노드, 동적)─▶ base_link ─(정적: 장착 실측)─▶ camera_link ─(정적: 축 회전)─▶ camera_optical
```
| 프레임 | 정의 |
|---|---|
| `camera_optical` | OpenCV/ArUco 측정 기준. 원점 = 렌즈 **광학 중심**, x 오른쪽 · y 아래 · z 앞 |
| `camera_link` | 같은 원점, ROS 차체 관례 x 앞 · y 왼쪽 · z 위. `camera_link → camera_optical` = 고정 회전 (rpy −90°, 0, −90°) |
| `base_link` | 차량 기준점 = **뒷바퀴 축 중심의 바닥 투영**(자전거 모델 기준). x 앞 · y 왼쪽 · z 위 |
| `map` | SLAM 지도 원점. 지금은 첫 프레임 카메라 자세에서 정해짐 → `base_link` 기준으로 다시 정의(첫 프레임의 base_link 를 원점, 바닥을 xy 평면) |
| `track` | 실측 마커 CSV 좌표 (Unity 트랙과 같은 원점, 오른손 좌표) |

- SLAM 이 추정하는 건 `map` 에서 본 **카메라 광학 중심**의 자세 T_map_cam. TF 트리에서는 프레임마다 부모가 하나뿐이므로 노드는
  **T_map_base = T_map_cam · (T_base_cam)⁻¹** 를 계산해 `map → base_link` 로 발행한다 (`/car/cam_pose` 는 카메라 자세 그대로 따로 발행).
- `base_link → camera_link` 정적 TF 는 장착 실측값(체크리스트 C): 앞뒤 x, 좌우 y, 높이 z, 아래로 숙인 각 pitch (필요하면 yaw·roll).
  실측 전에는 0 으로 두고(= base_link 가 카메라와 같은 위치), 값이 생기면 launch 인자만 바꾼다.
- tf2 는 좌표 변환·시각 보간·rviz 표시를 맡고, **차량 kinematics(자전거 모델: v, δ, 축간거리 L)는 `base_link` 기준으로 별도 계산**한다 (이후 단계: EKF 예측 모델).

## 3. 패키지 구성 (`~/demo/ros_ws/src`)
| 패키지 | 내용 | 출처 |
|---|---|---|
| `car_drive` | 기존 `car_drive_exp_copy` 그대로 + `/car/cmd` 발행 몇 줄 | 복사본 수정 |
| `aruco_slam_ros` | `aruco_slam_node.py` (EKF = `aruco-slam-fixed`), `video_publisher.py` (mkv → 토픽, 하드웨어 없는 시험용), `bag_to_render.py` | 새로 작성 |
| `car_bringup` | `bringup.launch.py` (카메라·SLAM·rviz·bag 선택), `usb_cam_c920.yaml`, `camera_info_c920_720p.yaml`/`_1080p.yaml`, `car.rviz`, `drive_ros.sh` (사전점검 + 실행) | 새로 작성 |

## 4. 단계별 계획 (각 단계 끝에 "통과 기준")

### P0. 작업공간·실행 환경 (0.5일)
- `~/demo/ros_ws` 만들고 `car_drive` 복사본 이동/링크, colcon 빌드.
- SLAM 노드 실행 시 `PYTHONPATH=aruco_slam_deps` 가 앞에 오도록 launch 에서 `SetEnvironmentVariable` (cv_bridge 미사용).
- **통과**: `ros2 run aruco_slam_ros aruco_slam_node --help` 가 cv2 4.10 으로 뜸, 기존 13개 pytest 통과.

### P1. 카메라 노드 (0.5일, C920 만 있으면 됨)
- `usb_cam` `pixel_format: raw_mjpeg`, 720p/1080p 30 fps 파라미터 파일.
- 초점 19·노출 156 은 usb_cam 파라미터로 안 맞을 수 있음 → launch 앞단에서 `v4l2-ctl` 실행 후 값 확인 (drive.sh 사전점검과 동일).
- `camera_info` 는 1080p 캘리브레이션을 해상도에 맞게 축소해 yaml 로.
- 대안: usb_cam 이 MJPG 그대로 발행을 못 하거나 타임스탬프가 부정확하면, `live_slam.py` 의 Grabber 를 떼어 만든 자체 카메라 노드(OpenCV `CONVERT_RGB=0` 로 MJPG 바이트 그대로 발행).
- **통과**: `ros2 topic hz` 30±1 Hz (720p·1080p), 대역폭 1080p ≈ 11 MB/s · 720p ≈ 4 MB/s (현 mkv 와 같은 수준), `rqt_image_view` 로 확인, v4l2 값 유지.

### P2. SLAM 노드 (1일, 하드웨어 없이)
- 구독: `/camera/image_raw/compressed` (QoS best-effort, depth 1 → 처리 느리면 오래된 프레임 버림, 지연 누적 없음), `/camera/camera_info` 로 K 설정.
- 처리: `cv2.imdecode` → `EKF.process_frame` (live_slam.py 와 같은 코드).
- 발행: `/car/cam_pose`(카메라), `/car/pose`·`/car/path`(base_link), `/aruco/markers`, TF `map → base_link`, 선택 `/aruco/debug_image/compressed`, 처리 시간 진단.
- 정적 TF: launch 에서 `base_link → camera_link`(장착 인자), `camera_link → camera_optical`(고정).
- 시험: `video_publisher.py` 로 **복도 영상(corridor_mapping_01.mkv)** 을 실시간 속도로 흘려서 SLAM 노드 결과를 오프라인 t4 결과와 비교.
- 1080p 유지 방법: 디코드를 별도 스레드로 처리와 겹치기, 흑백 디코드(`IMREAD_GRAYSCALE`, 11→5 ms), 최신 프레임만 처리. 그래도 모자라면 (1) 축소 영상에서 검출 → 원본에서 코너 정밀화, (2) 720p 순으로 내림.
- **통과**: 실시간 재생에서 처리율 ≥ 25 fps(1080p, 마커 20개 이상 지도 포함), 지연(stamp→pose 발행) 중앙값 < 50 ms, 마커 지도 Kabsch 잔차가 오프라인과 1 cm 이내.

### P3. 주행 연동 + 녹화 (0.5일, 가짜 보드)
- `wasd_drive`: `camera:=none` 기본, `/car/cmd` (예: `std_msgs/Int32MultiArray` [pwm, steer] 또는 `geometry_msgs/Twist`) 발행 추가. CSV 로그는 유지.
- `drive_ros.sh`: 사전점검(현 drive.sh 재사용) → `bringup.launch.py` 백그라운드 → `wasd_drive` 전경(키 입력은 터미널) → Q 종료 시 bag 정리.
- bag 토픽: compressed image, camera_info, `/car/cmd`, `/car/cam_pose`, `/aruco/markers`, `/tf`.
- **통과**: t10 과 같은 자동 키 입력 시험에서 세션 CSV + bag 생성, Ctrl+C·Q 둘 다 bag 정상 종료, 가짜 보드 워치독 정상.

### P4. 시각화·결과 영상 (0.5일)
- `car.rviz`: Path(초록), MarkerArray(파랑, 보이는 것 빨강), 카메라 프러스텀, 디버그 영상 패널.
- 실측 CSV → `track → map` 정적 TF (마커 3개 이상 보이면 계산).
- rviz 에 차체 상자(base_link)와 카메라 프러스텀을 함께 표시해 장착 TF 가 맞는지 눈으로 확인.
- `bag_to_render.py`: bag 에서 영상·자세를 읽어 `render_clean.py` 형식 MP4 (SLAM 재실행 또는 기록된 자세 사용 선택).
- **통과**: 복도 bag 으로 rviz 화면과 MP4 가 desiered.png 형식으로 나옴.

### P5. 실제 주행 (하드웨어)
- 바퀴 띄우고 1회 → 저속(`max_speed 40`) 마커 2개 이상 보이는 경로 → 트랙.
- **통과**: 주행 중 rviz 에서 궤적 실시간 표시, 종료 후 MP4 생성, bag 재생 시 같은 궤적.

### 이후 (선택)
- 운동 모델: `/car/cmd` PWM→속도 보정 후 EKF 예측 단계에 넣기 (마커 안 보일 때 위치 유지).
- 지도 불러오기(`load_map` 수정) → 트랙 지도를 미리 넣고 위치만 추정.
- `robot_localization` 으로 다른 센서와 융합, Unity 시뮬레이터와 같은 토픽 이름으로 맞추기.

## 5. 위험과 대응
| 위험 | 대응 |
|---|---|
| OpenCV/numpy 버전 충돌 (cv_bridge) | cv_bridge 미사용, CompressedImage 를 직접 디코드 |
| usb_cam 이 C920 제어값을 덮어씀 | launch 에서 usb_cam 시작 **후** v4l2-ctl 재설정 + 값 확인, 안 되면 자체 카메라 노드 |
| 지도 마커가 늘수록 EKF 비용 증가 (해상도와 무관) | 마커 수별 처리율 측정, 디코드 스레드 분리·흑백 디코드, QoS depth 1. 최후에 720p |
| 키보드 포커스 (rviz 창이 키 입력을 가져감) | wasd_drive 는 별도 터미널 전경, 안내문에 명시 |
| bag 디스크 사용 | MJPG 그대로라 현 mkv 와 같은 1080p ≈ 0.69 GB/min, 720p ≈ 0.24 GB/min (여유 8.8 GB → 1080p 약 12분) → 사전점검에 반영 |

## 6. 내일 테스트와의 관계
B안은 하드웨어 없이 P0–P4 를 끝낸 뒤 P5 로 간다 (약 3일). **내일 트랙 테스트는 검증된 `drive.sh record`** 로 하고, 그때 녹화한 video.mkv 를 `video_publisher.py` 로 흘려 P2 시험 데이터로 재사용한다.

## 7. 진행 기록

### P0 완료 (2026-11-20)
- 작업공간 `~/demo/ros_ws` (원본·`car_drive_exp_copy` 는 그대로, `car_drive` 는 복사본에서 다시 복사 + `tests_require` 만 추가)
  ```
  ros_ws/env.sh                 source 하면 ROS Humble + 이 작업공간 (conda 제거, PYTHONNOUSERSITE)
  ros_ws/build.sh [--test]      colcon build --symlink-install (+ colcon test)
  src/car_drive/                주행 노드 (기존 그대로)
  src/aruco_slam_ros/
    __init__.py                 import 시 ~/aruco_slam_deps · aruco-slam-fixed 를 sys.path 앞에 (cv_bridge 미사용)
    frames.py                   좌표 변환: R_LINK_OPT, T_base_link(장착), FrameChain(slam→map/base_link)
    env_check.py                ros2 run aruco_slam_ros env_check — 환경 점검
    test/test_frames.py         변환 시험 6개 (뒷축 제자리 회전·숙인 카메라 바닥점 등)
  src/car_bringup/
    config/mount.yaml           장착 실측값 (지금 0, 체크리스트 C 로 채움)
    launch/frames.launch.py     정적 TF base_link → camera_link → camera_optical
  ```
- 통과 기준 확인
  - `env_check`: numpy 1.26.4 · OpenCV 4.10.0(ArucoDetector) · scipy 1.14.1 · EKF 로드(fx 1481.4) · rclpy/메시지/tf2 · frames — **모두 OK**
  - `colcon test`: **19개 통과** (aruco_slam_ros 6 + car_drive 기존 13)
  - `frames.launch.py x:=0.28 z:=0.14 pitch_down_deg:=12` → `tf2_echo base_link camera_optical` 가 `frames.py` 계산과 일치 (t [0.28 0 0.14], q xyzw [0.550 −0.550 0.445 −0.445])
- 알게 된 것: colcon 은 setup.py 에 `tests_require=["pytest"]` 가 없으면 unittest 로 돌려 0개로 센다.

### P1 카메라 노드 — 소프트웨어 완료, C920 측정 대기 (2026-11-20)
**결정: usb_cam 대신 자체 노드 `car_bringup/c920_camera`.** usb_cam 0.8.1 `raw_mjpeg` 을 내장 웹캠으로 시험하니 stamp 가 틀림:
첫 프레임 stamp 가 usb_cam 노드 시작 로그보다 0.3 s **앞섬**, 수신−stamp 가 실행마다 964 / 669 ms 로 달라 보정 불가.
같은 카메라를 OpenCV 로 직접 읽으면 드라이버 지연 35 ms. 또 format 문자열이 `yuv422; jpeg compressed mono8` 로 잘못 붙음.

| 항목 | usb_cam 0.8.1 | c920_camera (자체) |
|---|---|---|
| stamp | 실행마다 0.7~1 s 어긋남 | V4L2 버퍼 시각(monotonic) → 벽시계 변환 |
| 수신 − stamp (내장 웹캠 720p) | 669~964 ms | **44 ms** (p95 47) |
| format | `yuv422; jpeg compressed mono8` | `jpeg` |
| MJPG 재인코딩 | 없음 | 없음 (`CAP_PROP_CONVERT_RGB=0`, 시스템 OpenCV 4.5.4 로 충분) |
| 초점·노출 | 옛 컨트롤 이름이라 C920 에서 불확실 | 시작 전·후 v4l2-ctl 로 설정, 5 s 마다 다시 읽어 바뀌면 경고·재설정 |
| DHT 없는 MJPG | 그대로 | 표준 허프만 테이블 삽입 (시험으로 확인) |

추가 파일
```
src/car_bringup/car_bringup/c920_camera.py   카메라 노드 (stamp·DHT·컨트롤 점검·5 s 통계 로그·Ctrl+C 이중 SIGINT 처리)
src/car_bringup/launch/camera.launch.py       driver:=c920(기본)|usb_cam(비교용) · size · device · controls · focus · exposure
src/car_bringup/config/camera_info_c920_{1080p,720p}.yaml   1080p 캘리브레이션 (720p 는 ×2/3)
src/car_bringup/config/usb_cam_c920.yaml      비교용
src/car_bringup/test/test_camera.py           DHT 삽입 시험 2개
src/aruco_slam_ros/aruco_slam_ros/cam_probe.py   토픽 측정 (Hz·간격·끊김·KB·MB/s·디코드·지연·camera_info)
ros_ws/p1_camera_check.sh [--builtin]         P1 자동 확인 (launch 자동 종료, 결과 ~/demo/out/p1/<시각>/)
```
- 내장 웹캠(MJPG 720p) 시험: 디코드 OK, camera_info K 720p 축소값 확인, Ctrl+C 종료 깨끗, 웹캠 컨트롤 변화 없음.
  수신율 7.5 Hz 는 **웹캠 자체 한계**(어두운 곳 자동노출 + dynamic framerate, `v4l2-ctl --stream-mmap` 도 7.49 fps).
- colcon test 21개 통과.
- **남은 것 (C920 필요, 01:14 경 USB 분리됨)**: `~/demo/ros_ws/p1_camera_check.sh` → 1080p·720p 각각 30±1 Hz,
  1080p ≈ 11 MB/s, 지연 < 50 ms, 노드 실행 중 초점 19·노출 156 유지.

### P2 SLAM 노드 — 완료 (2026-11-20, 복도 영상 1080p 실시간 재생)
| 기준 | 결과 |
|---|---|
| 처리율 ≥ 25 fps (1080p) | 수신 30 / 처리 29~30 fps, 처리 중앙값 20.6 ms |
| 지연 중앙값 < 50 ms (stamp → 자세 발행) | **32.8 ms** (p95 68.8) |
| 마커 잔차가 오프라인과 1 cm 이내 | 8.2 / 6.0 / 0.6 / 13.3 cm vs 오프라인 8.2 / 6.1 / 0.7 / 13.3 → 차이 ≤ 0.1 cm, 지도끼리 0.05~0.13 cm |
| 궤적 | 오프라인과 위치 차 중앙값 0.51 cm (p95 1.46), yaw 0.03° |
| 마커 20개 이상 | EKF observe: 40개 **5.3 ms**, 60개 5.9 ms (BLAS 1스레드). 1080p 합계 ≈ 디코드 5 + 검출 15 + EKF 5 ≈ 25 ms |
| 토픽·TF | /car/pose · /car/cam_pose · /car/path · /aruco/markers(보이는 것 빨강) · /aruco/visible · /aruco_slam/stats · TF map→base_link. TF 연쇄로 구한 카메라 = /car/cam_pose (0.000 mm) |

고친 것 / 알게 된 것
1. **rclpy 에서 `msg.data = bytes` 는 300 KB 에 33 ms** (원소마다 검사) → 30 fps 불가. `array.array("B", data)` 로 0 ms. c920_camera·video_publisher 둘 다 수정 (P1 은 내장 웹캠 7.5 fps 라 안 드러났음).
2. **OpenBLAS 멀티스레드가 EKF 를 3배 느리게 함**: 마커 20개부터 상태 행렬이 커지면 스레드 전환 → 16.9 ms. `aruco_slam_ros/__init__.py` 에서 OPENBLAS/OMP/MKL_NUM_THREADS=1 (numpy import 전) → 5.3 ms.
   REPORT §2.2 의 "마커 20~40개면 +11 ms" 도 이 때문 (live_slam.py · render_*.py 도 `OPENBLAS_NUM_THREADS=1` 로 돌리면 빨라짐).
3. QoS: 발행 reliable(depth 5) + SLAM 구독 reliable(depth 1) → 897/897 수신 (best-effort 구독은 300 KB 메시지 일부 유실 843~888/897). 처리 중 건너뛴 프레임(최신만 처리)은 별개로 22장.
4. 정적 TF(frames.launch)와 SLAM 노드가 서로 다른 장착값을 읽던 문제 → slam.launch 가 같은 `mount_file` 을 둘 다에 전달.
5. video_publisher 는 구독자가 붙을 때까지 기다린 뒤 시작 (첫 20여 장 유실 방지).

추가 파일
```
src/aruco_slam_ros/aruco_slam_ros/aruco_slam_node.py   SLAM 노드 (최신 프레임만, 흑백 디코드, FrameChain 으로 base_link 자세, out_dir 기록)
src/aruco_slam_ros/aruco_slam_ros/video_publisher.py   녹화 mkv → 카메라 토픽 (MJPEG 바이트 그대로, PTS 대로)
src/car_bringup/launch/slam.launch.py                  source:=video|camera · mount_file · out_dir · reliable · decode_gray
~/demo/scripts/p2_compare.py                           ROS 결과 vs 오프라인 비교 → compare_offline.json
~/demo/out/p2/corridor_rt_rel_true/                     기준 실행 결과 (trajectory_ros.csv · map.txt · summary.json · compare_offline.json)
```
재현: `source ~/demo/ros_ws/env.sh && ros2 launch car_bringup slam.launch.py source:=video video:=~/car_calibration/corridor_map/videos/corridor_mapping_01.mkv out_dir:=<dir>`
→ `python3 -B ~/demo/scripts/p2_compare.py <dir>`

남은 것: rviz 화면 확인은 P4 (car.rviz 와 함께).

### P3 주행 연동 + bag 녹화 — 완료 (2026-11-20, 가짜 보드 + 녹화 영상 소스)
- **주행 노드는 수정하지 않음**: `wasd_drive` 가 이미 `/car/pwm`(Int16) · `/car/steer_cmd`(String L/R/F) · `/car/steering_pot`(Int32) · `/car/state`(Int8) 를 발행 (P5 준비에서 `/car/board_state` 로 이름 바꿈 — `/car/state` 는 차량 상태 CarState) → 계획의 `/car/cmd` 대신 그대로 bag 에 기록.
- `car_bringup/launch/bringup.launch.py`: 영상 소스 + SLAM + 정적 TF + `ros2 bag record` (영상·camera_info·자세·마커·주행 명령·TF)
- `~/demo/ros_ws/drive_ros.sh`: 사전 점검(카메라/영상·포트·장착값·디스크·안전) → bringup(별도 세션) → SLAM 첫 프레임 확인 → `wasd_drive`(camera:=none) 전경 → 종료 시 bringup 에 SIGINT 1번 → bag 닫힘·요약.
  결과는 실행마다 `~/demo/ros_ws/runs/<시각>/{bag, slam, <주행 세션>, bringup.log, mount.yaml}`
- `slam.launch.py source:=none`: 영상 소스 없이 SLAM 만 (bag 재생용)
- 시험 `~/demo/ros_ws/p3_drive_check.sh q|ctrlc` (socat pty 가짜 보드 + 복도 영상 0.5배속 + 자동 키)

| 기준 | 결과 |
|---|---|
| 세션 CSV + bag 생성 | 둘 다 같은 run 폴더에. bag 20 s ≈ 99 MB, 영상 514/514 (1배속 첫 시험) |
| Q 종료 · Ctrl+C 종료 둘 다 bag 정상 | 둘 다 모든 노드 `finished cleanly`, metadata.yaml 생성, `ros2 bag info` 정상 |
| 가짜 보드 워치독 | 둘 다 마지막 명령 V0 → F → S, can_go False, 최고 PWM 20 |
| SLAM (주행 중) | 처리 20.5 ms · 지연 29 ms · 마커 프레임 100 % |
| bag 재생 → 같은 지도 | 시작 프레임 차이로 원점 1 cm·0.07° 다름, 그것을 빼면 마커 차이 **0.000 cm** |

고친 것 / 알게 된 것
1. **비대화형 bash 에서 `&` 로 띄운 프로세스는 SIGINT 가 '무시'로 시작** → `ros2 launch` 가 SIGINT 를 못 받아 15 s 뒤 강제 종료되고 bag 이 위험. bash 로는 되돌릴 수 없어 python 래퍼로 SIG_DFL 복원 후 exec. (P1 에서 `kill -INT` 로 launch 가 안 꺼지던 것도 같은 원인)
2. Ctrl+C 는 스크립트에도 가므로 주행 중에는 `trap ':' INT` (빈 핸들러: 자식은 기본 동작 유지, 스크립트는 정리까지 진행).
3. 지도 원점 = SLAM 이 받은 첫 프레임 → 출발 위치에서 정지한 채 시작해야 bag 재생·실시간 결과가 같은 기준이 됨 (체크리스트 E-3 그대로).

남은 것: C920 실물로 `drive_ros.sh` (기본 source:=camera) — P1 측정과 함께. 바퀴 띄운 실제 보드 시험은 P5.

### P4 시각화 — 완료 (2026-11-20)
| 항목 | 결과 |
|---|---|
| 트랙 정합 `track_aligner` | 실측 CSV ↔ SLAM 지도(/aruco/markers) Kabsch → TF **track → map** (/tf_static, 2 s 마다 갱신) · `/car/pose_track` · `/track/align`(잔차 JSON) · `/track/landmarks`(회색). 복도 최종 잔차 **8.3 / 6.0 / 0.6 / 13.3 cm (RMS 8.4)** = 오프라인과 같음 |
| 제어기용 조회 | `tf2 lookup_transform('track', 'base_link')` = `/car/pose_track` (1 mm 이내) |
| rviz (`car.rviz`, `rviz:=true`) | 고정 좌표 track(실측 있을 때)/map · 그리드 0.5 m · TF · 궤적(초록) · 차량 자세(파랑 화살표) · SLAM 마커(보이면 빨강) · 실측 마커(회색 gt) · 카메라 패널(/aruco/debug_image, 640 폭 5 Hz, 검출 테두리·ID). 캡처 `~/demo/out/p4/rviz/rviz.png`, Global Status Ok, 31 fps |
| bag → 영상 `bag_export` | bag → 세션 형식(video.mkv: JPEG 그대로 · video_frames.csv: stamp · drive_log.csv: /car/pwm·steer_cmd) → `render_clean.py`(기본) / `render_localization.py`(`--render full`) |
| drive_ros.sh | `--landmarks` · `--rviz` · `--render clean|full|none` 추가 → 주행 끝나면 결과 영상 자동 |
| 전체 시험 (1배속, 가짜 보드) | bag 204 MB · SLAM 처리 20.6 ms · 지연 31 ms · 영상 572 프레임 29.9 fps → `~/demo/out/videos/20261120_042353_clean.mp4` |

알게 된 것
1. **정합은 기본 3D**. yaw 만 맞추면(바닥 평면 가정) 장착 pitch/roll 이 0 인 상태에서 카메라 기울기를 흡수 못 해 잔차가 커짐 (복도 RMS 10.4 vs 8.4). 장착값을 실측해 mount.yaml 에 넣은 뒤에 `align_yaw_only:=true` 권장.
2. 주행 중 정합은 지도가 수렴하면서 좋아진다 (복도: 처음 RMS 12~13 cm → 끝 8.4 cm). 먼 마커(4.5 m+)는 가까이 갈 때까지 거칠다 → 경고 기준 10 cm 는 초반에 한두 번 뜰 수 있음.
3. rviz2(Humble) Image 디스플레이는 압축 영상을 바로 못 받음 → SLAM 노드가 작은 원본 영상(/aruco/debug_image)을 5 Hz 로 따로 발행.
4. rosbag2 sqlite3 bag 은 `rosbag2_py` 로 바로 읽힘. Wayland 화면 캡처는 `QT_QPA_PLATFORM=xcb` + `xwd -id` + ffmpeg 변환.

추가 파일
```
src/aruco_slam_ros/aruco_slam_ros/track_aligner.py   트랙 정합 노드
src/aruco_slam_ros/aruco_slam_ros/bag_export.py      ros2 run aruco_slam_ros bag_export <bag> [--render clean|full|none] [--landmarks csv]
src/car_bringup/rviz/car.rviz · launch/rviz.launch.py
(수정) aruco_slam_node.py (/aruco/debug_image) · slam.launch.py (landmarks · rviz · align_yaw_only) · bringup.launch.py · drive_ros.sh
```

### 다음: P5 실제 주행 (하드웨어: C920 + 아두이노)
1. C920 연결 → `~/demo/ros_ws/p1_camera_check.sh` (P1 마무리: 1080p 30 Hz · 초점·노출 유지)
2. 장착 실측 → `src/car_bringup/config/mount.yaml` (체크리스트 C) → `./build.sh`
3. 바퀴 띄우고: `~/demo/ros_ws/drive_ros.sh --max-speed 40 --rviz` (W 한 번 → Space → Q)
4. 트랙: `~/demo/ros_ws/drive_ros.sh --max-speed 40 --landmarks ~/track_landmarks.csv --rviz` → 결과 영상 자동
