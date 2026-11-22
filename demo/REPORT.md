# ArUco 로컬라이제이션 점검 보고서 + 내일 트랙 시험 체크리스트

- 작성: 2026-11-20 00:35 (KST) · 대상: `~/aruco-slam`(yishaiSilver/aruco-slam `d1409c7` + 로컬 수정), `~/car_drive_exp`(`e2beda0`), `~/car_calibration`
- **원본은 건드리지 않았다.** 모든 작업은 `~/demo/` 안에서 했고, 원본 파일 412개의 크기·수정시각이 처음 스냅샷과 같고 두 레포의 git status도 같다 (`out/originals_before.txt` ↔ `out/originals_after.txt`). git이 `git status` 때 스스로 갱신하는 `.git/index`의 시각만 바뀌었다.
- 하드웨어: C920은 연결돼 있음(카메라는 고정된 채로 찍기만 함). 아두이노는 연결 안 됨 → 가짜 보드(pty)로 대체. **바퀴·조향 모터는 한 번도 구동하지 않았다.**

- **그림**: `figures/F1~F9.png` (이 문서의 각 절에 들어 있음). 다시 그리기: `cd ~/demo/scripts && python3 -B make_figures.py`

---

## 0. 결론 먼저

| 질문 | 답 |
|---|---|
| 1. 레포가 제대로 적용됐나? | **대체로 그렇다. 그러나 원본 레포 자체에 회전 버그가 있다.** 로컬 수정 5건은 모두 맞다. 복도 결과(정합 잔차 8.2 / 6.1 / 0.7 / 13.3 cm)는 내가 똑같이 재현했다. 다만 원본 `EKF.add_marker()`가 카메라가 **회전한 뒤 처음 보인 마커**를 엉뚱한 위치에 넣는다. 복도 실험은 yaw가 2° 미만인 직진이라 이 버그가 드러나지 않았다. 회전이 있는 트랙에서는 위치 오차가 0.13 m에서 1.16 m로 커진다(합성 시뮬레이션). |
| 2. WASD 주행 중 3D 추적이 될까? | **실시간으로는 현재 구조상 안 된다.** 주행 노드의 ffmpeg가 C920을 독점하기 때문에 SLAM이 카메라를 열지 못한다(실측 확인). **주행 후 오프라인 처리는 된다.** 녹화 영상과 `video_frames.csv`를 넣으면 `drive_log`와 같은 시간축의 3D 궤적이 나온다. 이를 위한 파이프라인을 demo에 만들고 검증했다. 속도는 1080p 기준 프레임당 31 ms(마커 4개)~42 ms(마커 20~40개)라서, 나중에 카메라를 SLAM 노드 하나가 맡는 구조로 바꾸면 24~32 Hz의 실시간 처리도 가능하다. |
| 3. 내일 프로토콜 | §3 체크리스트. 가장 중요한 다섯 가지: ① 디스크 여유가 **8.8 GB**뿐이라 1080p 녹화는 약 12분이면 가득 찬다. ② 마커 실측 좌표는 **오른손 좌표계로** 적는다(Unity는 왼손). ③ 출발 전 3초 정지해서 마커 3개 이상을 보여 준다. ④ 종료는 반드시 **Q**로 한다(Ctrl+C로 끝내면 영상 시각이 날아감). ⑤ 카메라를 USB에 다시 꽂으면 초점·노출 설정이 초기화된다. |
| 4. 스모크 테스트 | 하드웨어를 움직이지 않는 항목 **14개 중 13개 통과, 1개는 이미 알려진 버그 재현**(Ctrl+C 때 영상 시각 손실). 이 버그는 demo 사본에서 한 줄 수정으로 고쳐지는 것까지 확인했다. 원본에는 적용하지 않았다(§6 패치). |

---

## 1. 레포 적용 검증

### 1.1 실행 환경
- 실행 방식: `PYTHONPATH=~/aruco_slam_deps:~/aruco-slam/thirdparty/pangolin python3 -m main.run_slam --video … --filter ekf` (`.bash_history` 기준). `aruco-slam/.venv`는 비어 있어 쓰이지 않는다.
- 확인한 버전: cv2 4.10.0(contrib, `ArucoDetector` 있음), numpy 1.26.4, scipy 1.14.1, sympy 1.9, gtsam, pangolin 모두 import 정상. 레포 `requirements.txt`의 numpy 2.1.3과 다르지만 동작에는 문제없다.

### 1.2 로컬 수정 5건 판정 (원본 레포 대비 `git diff`)

| 수정 | 판정 |
|---|---|
| `base_filter.py:197` 마커 크기 0.20 → **0.15 m** | 실제 인쇄 크기가 15 cm라면 맞다. **자로 검은 테두리 한 변을 재서 확인할 것**(T2에서 +4.5% 스케일 편향이 있었다. 원인 후보 중 하나). |
| EKF `rot_mc.inv()` → `rot_mc.T` | **맞다.** sympy `to_rotation_matrix()`는 정규화된 직교행렬이라 수학적으로 같고, 기호 역행렬보다 훨씬 빠르다. |
| `run_slam.py` DISPLAY_3D=False, SAVE_2D=False | 디버그용 설정. 3D 뷰어 자체는 정상 동작한다(T9: 120프레임, 프레임당 0.9 ms). |
| `charuco_calibration.py` 경로·None 가드·**이미지 크기 (w,h)**·RMS 출력 | **맞다.** 특히 원본은 `image.shape[:2]`(h,w)를 넘기는 버그가 있었는데 그걸 고친 것이다. |
| `calibration/*.npy` 교체 | 현재 값 = `car_calibration/1080p/recal_20261119`(60장). 같은 폴더의 `images_1080p_selected`(34장)로 만든 값이 **아니다**. 34장으로 다시 돌리면 fx가 1302로 크게 달라진다. 출처를 기억해 둘 것. |

### 1.3 원본 레포 자체의 문제 (yishaiSilver 코드 그대로인 부분)

1. **`EKF.add_marker()` 회전 규약이 반대다 (중요).** 측정 모델 `h()`, 2D/3D 뷰어는 모두 `p_cam = Rᵀ (L − t)`, 즉 R = 카메라→맵 규약을 쓴다. 그런데 `add_marker()`만 `L = R⁻¹ p + t`로 반대 규약을 쓴다. 원본 코드 주석에도 "this may be wrong"이라고 적혀 있다.
   - T1 단일 검사: 카메라 yaw 10° / 30° / 60° / 90°에서 새 마커 초기 위치 오차가 0.74 / 2.12 / 3.67 / 4.24 m다. 수정본은 모두 0이다.
   - T1 합성 주행(직진 2 m → 60° 선회 → 직진 2 m, 선회 뒤 새 마커 3개, 5회 평균):

     | | 선회 전 위치오차 | 선회 후 | 최종 | 최종 yaw (참값 60°) |
     |---|---|---|---|---|
     | 원본 | 0.055 m | **1.164 m** | 1.505 m | 30.6° |
     | add_marker 수정 | 0.059 m | **0.134 m** | 0.194 m | 51.8° |
     | + 반각 수정 | 0.060 m | 0.134 m | 0.215 m | 52.5° |

     ![F1 EKF 회전 버그](figures/F1_ekf_rotation_bug.png)
   - 복도 실험은 모든 마커가 첫 프레임부터 보였고 yaw가 2° 미만이었다. 그래서 원본과 수정본 결과가 같았다(T4). **내일 곡선 트랙에서는 이 버그가 결과를 망친다.** demo에서는 `EKFFixedInit` 서브클래스로 고쳐 쓴다(원본은 그대로 둠).
2. 오차 쿼터니언 스케일 불일치: `h()`는 dq=(1,e)(회전각 ≈ 2|e|)인데, 업데이트는 dq=(1,δ/2)를 쓴다. 보정이 절반만 들어간다. 영향은 작다(위 표).
3. **움직임 모델이 없다.** README에는 "moving average"라고 써 있지만 `predict()`는 공분산만 키우고 상태를 움직이지 않는다. 그래서 마커가 안 보이는 프레임에서는 위치가 멈춰 있다. 위치만 측정하고 마커 방향은 안 쓰기 때문에, 마커가 1개만 보이면 yaw가 잘 잡히지 않는다. 시뮬레이션에서 yaw를 60°가 아니라 52°로 추정한 것도 같은 이유다.
4. 맵 불러오기가 동작하지 않는다(코드 읽기로 확인). `EKF.__init__`이 map_file=None을 고정으로 넘기고, `BaseFilter.load_map()`은 존재하지 않는 `self.filter`를 호출한다. 그래서 실측 마커 지도를 사전정보로 넣을 수 없다. 매번 새 지도를 만든 뒤 **사후에 실측 좌표로 정합해야 한다**(복도에서 한 방식, T4·T7에 포함).
5. `trajectory.txt`의 쿼터니언 순서가 TUM 표준(qx qy qz qw)이 아니라 **qw qx qy qz**다. 복도 정합(`camera_pose_relative.csv`)은 올바른 순서로 해석돼 있었다(yaw 차이 0.08°; 반대 순서로 읽으면 3.5°). `evo` 같은 TUM 도구에 넣을 때는 순서를 바꿔야 한다.
6. 원본 `run_slam.py`는 실행할 때마다 `outputs/trajectory.txt`·`map.txt`(SAVE_3D면 `outputs/images/…`도)를 **덮어쓴다**. 결과를 보존하려면 매번 복사하거나 demo의 headless 실행기를 쓸 것.

### 1.4 캘리브레이션 판정

- T2. 실측 거리 영상에서 PnP로 잰 거리 z(중앙값):

  | 영상 | current (fx 1481) | old 1080p (fx 1244) | upstream 값 (fx 1415) |
  |---|---|---|---|
  | 100 cm | 101.6 | 84.7 | 97.7 |
  | 100 cm 재확인 | 104.0 | 86.6 | 98.8 |
  | 100 cm 고정초점 | **104.8** | 87.2 | 99.6 |
  | 150 cm | 150.4 | 125.7 | 144.6 |
  | 150 cm 재확인 | 156.7 | 130.8 | 149.2 |
  | 150 cm 고정초점 | **156.6** | 130.7 | 149.0 |

  ![F2 캘리브레이션 거리](figures/F2_calibration_distance.png)

  → **현재 값이 쓸 만하다**(예전 1080p 값은 −13~16%라 쓰면 안 된다). 다만 고정초점에서 일정하게 **+4.5%** 크게 나온다. 핀홀로 역산하면 fx는 약 1435~1460이다. 원인 후보는 셋이다: (a) fx가 약간 큼, (b) 실제 마커가 15 cm보다 작음, (c) 거리를 잴 때 기준점이 다름. 내일 마커 한 변을 자로 재면 (b)를 바로 가릴 수 있다.
- T3. 현재 값은 60장, RMS 0.729 px로 정확히 재현된다. fx 표준편차는 9.9이다. 주점 cy=429는 영상 중심 540에서 111 px 위에 있다(표준편차 11). 데이터로는 유의하지만 흔한 값은 아니다. 보드가 화면 위·아래 끝까지 오는 사진을 보강해 다시 찍는 걸 권장한다(필수는 아님).
- C920 설정: 현재 `focus_automatic_continuous=0`, `focus_absolute=19`, `auto_exposure=1(수동)`, `exposure_time_absolute=156`이다. 이 값들은 USB를 다시 꽂거나 재부팅하면 초기화된다. 캘리브레이션은 초점에 민감하다(34장 세트는 fx 1302).
- `car_calibration/results`(720p 리포트)의 왜곡계수(k1 0.57, k3 1.84, p1 0.064)는 비정상적으로 크다. 쓰지 말 것. 720p 영상은 1080p로 키워서 현재 1080p 값을 쓰면 된다(T8 참고).

### 1.5 복도 결과 재현 (T4)
- `corridor_mapping_01.mkv`를 원본 EKF로 GUI 없이 다시 돌리면 `map.txt`가 `slam_result_01/map.txt`와 **완전히 같다**. 같은 Kabsch 방식으로 정합하면 잔차 **8.2 / 6.1 / 0.7 / 13.3 cm**(RMS 8.4 cm, det=+1)로 기존 결과와 일치한다.

  ![F3 복도 정합](figures/F3_corridor_alignment.png)

- 궤적의 프레임 단위 떨림: 고주파 표준편차가 축마다 0.4 / 0.2 / 0.6 cm다. 원시 경로 길이는 5.5 m지만 0.5초 평활 후 1.4 m이고, 실제 이동은 약 1.1 m다. 차량 궤적으로 쓸 때는 평활하거나 움직임 모델을 넣을 것.

---

## 2. ROS WASD 주행 중 3D 카메라 추적 평가

### 2.1 지금 구조로 되는 것 / 안 되는 것

| 항목 | 결과 | 근거 |
|---|---|---|
| 주행 중 **실시간** SLAM | ✗ | 녹화 중 두 번째 프로세스로 C920을 열 수 없다(`isOpened False`, T6). 혼자 열 때는 정상. 주행 노드의 ffmpeg가 카메라를 독점한다. |
| 주행 후 **오프라인** 3D 궤적 | ✓ | `t7_session_slam.py`: 세션 폴더를 넣으면 `t_s,x,y,z,yaw,pwm,n_markers`가 나온다. 정지 상태 세션에서 마커 0~3을 매 프레임 검출했다. |
| 영상과 주행 명령의 시간 맞춤 | ✓ | `video_frames.csv`가 `drive_log.csv`와 같은 시간축이다. Ctrl+C로 깨진 세션도 mkv PTS와 `first_frame_epoch`로 복구할 수 있다. 정상 세션에서 비교하면 오차 **0.0 ms**. |
| 720p 녹화(주행 노드 기본값)로 SLAM | ✓ | 720p는 1080p와 같은 화각을 축소한 것이다. 모서리 차이 ≤ 1.2 px, PnP 차이 2~3 m에서 ≤ 1.2 cm, 4.5 m에서 3 cm(T8). 먼 마커는 1080p가 더 정밀하다. |
| 3D 뷰어(Pangolin) | ✓ | 원본 `Viewer3D`로 120프레임, 프레임당 0.9 ms(T9). |
| 정지 상태 안정성 | ✓ | 마커 3개가 보일 때 위치 표준편차 < 0.8 cm, 최대 편차 3 cm, yaw 표준편차 0.1°. |

![F7 현재 카메라 시야](figures/F7_camera_view.png)

![F4 정지 안정성](figures/F4_static_stability.png)

![F6 주행 노드 스모크](figures/F6_drive_smoke.png)

### 2.2 처리량 (이 노트북, 단일 스레드, 다른 작업 없음 — T5b)
- 1080p, 마커 4개: 디코드+리사이즈 9.1 ms + 검출·PnP·EKF 22.2 ms = **31 ms (≈ 32 fps)**
- 720p(1080p 영상을 축소), 마커 4개: 10.0 + 17.1 = **27 ms (≈ 37 fps)**. 실제 720p 녹화면 디코드가 더 빠르다.
- 지도에 마커가 20~40개면 EKF가 약 11 ms 더 걸린다(T5) → 1080p 약 42 ms(24 fps), 720p 약 38 ms(26 fps).
  - **정정 (P2, 2026-11-20):** 이 +11 ms 는 계산량이 아니라 OpenBLAS 멀티스레드 오버헤드였다. `OPENBLAS_NUM_THREADS=1` 이면 마커 40개 EKF 가 16.9 → 5.3 ms. ROS SLAM 노드는 자동으로 1스레드로 돈다(`PLAN_ROS.md` §7 P2). 1080p 합계 약 25 ms(≈ 40 fps).
- 정정: T4 로그의 28.8 ms+12 ms는 원본·수정본 두 실행을 **동시에** 돌려 CPU를 나눠 쓴 상태에서 잰 값이라 부풀려져 있었다. 위 수치는 단독 실행으로 다시 잰 것이다.

![F5 처리량](figures/F5_throughput.png)

### 2.3 움직이면서 추적할 때의 정확도 위험 (중요한 순서)
1. **원본 EKF 회전 버그**(§1.3-1): 곡선 구간에서 처음 보이는 마커가 틀린 위치에 들어간다. → demo 파이프라인은 기본으로 `--fixed`를 쓴다.
2. **움직임 모델이 없다**: 마커가 안 보이는 순간 위치가 멈춘다. → 어느 위치에서든 마커가 **2개 이상** 보이게 배치한다.
3. 모션 블러: 노출이 15.6 ms(수동 156)라 빠르게 달리면 검출이 끊길 수 있다. → 낮은 PWM으로 먼저 시험하고, 검출률(`n_markers` > 0 비율)을 확인한다.
4. 맵 좌표 = 첫 프레임의 카메라 광학 좌표계(x 오른쪽, y 아래, z 앞)다. 트랙 좌표로 바꾸려면 실측 마커가 3개 이상 필요하다(공선이면 안 되고, 높이가 섞여 있으면 좋음).
5. 결과는 카메라 위치다. 차량 기준점으로 바꾸려면 카메라 장착 오프셋(높이·전후·피치)이 필요하다.

### 2.4 권장

![F8 데이터 흐름](figures/F8_pipeline.png)

- **내일은 "녹화 → 주행 후 오프라인 3D 궤적"으로 간다.** 이미 검증된 경로다.
- 실시간이 필요하면 다음 단계에서 카메라를 한 프로세스가 맡게 바꾼다. 예: SLAM 노드가 OpenCV로 캡처해 `/car/pose`를 발행하고 영상도 직접 저장하는 방식, 또는 ffmpeg tee + v4l2loopback. 처리량상 가능하다.

---

## 3. 내일 실제 트랙 시험 체크리스트

### A. 출발 전 (노트북)
- [ ] **디스크 확보**: 지금 `/home` 여유 공간이 **8.8 GB**다. 1080p는 약 0.69 GB/분이라 12분 정도, 720p는 약 0.24 GB/분이라 36분 정도 녹화할 수 있다. `~/car_calibration`의 지난 영상(2.8 GB)을 외장 디스크로 옮기거나 720p로 녹화할 것.
- [ ] 노트북 충전기, USB 허브, C920, Mega 케이블, 줄자·자, 마스킹테이프, 여분 마커
- [ ] 마커 인쇄물: **DICT_5X5_50, ID 0~49, ID 중복 금지**. 검은 테두리 한 변이 **정확히 15.0 cm**인지 자로 확인하고 값을 기록한다(다르면 `base_filter.py`의 0.15와 맞춰야 함). 무광 용지, 평평한 판에 붙여서 휘지 않게.
- [ ] `cd ~/car_drive_exp && git status`가 깨끗한지 확인. `./flash.sh --compile-only` 통과(오늘 확인함: 4648 B).
- [ ] demo 스크립트 동작 확인: `cd ~/demo/scripts && python3 -B t7_session_slam.py ../out/drive_logs/20261120_002824 --fixed`

### B. 현장 설치 (마커)
- [ ] 트랙 좌표 기준(원점, x·y 방향)을 Unity 트랙과 같게 정한다. **Unity는 왼손 좌표계(y가 위)다.** CSV에는 오른손 좌표계로 적는다. 예: `x = z_unity, y = −x_unity, z = y_unity`. 왼손 좌표를 그대로 넣으면 Kabsch가 반사를 거부해서 잔차가 커진다.
- [ ] 마커마다 **중심 좌표 id,x_m,y_m,z_m**를 줄자로 실측해 `track_landmarks.csv`에 적는다(복도의 `aruco_landmarks.csv`와 같은 형식). 최소 3개, 공선이 아니게, 높이를 섞어서.
- [ ] 배치 규칙: 주행선 위 어느 위치에서든 **카메라 화각(가로 약 66°) 안에, 0.5~4 m 거리에서, 마커 2개 이상**이 보이게. 곡선 구간 안쪽과 바깥쪽에 추가로 둔다. 4.5 m보다 먼 마커는 오차가 커진다(복도 마커 3: 13 cm).
- [ ] 마커 면은 주행 방향 쪽을 보게(±60° 이내), 조명 반사가 없게.
- [ ] 출발 위치에서 마커 3개 이상이 보이는지 화면으로 확인한다. 맵 원점은 첫 프레임 카메라이고, 모든 마커가 처음부터 보이면 회전 버그를 피할 수 있다.

![F9 마커 배치 가이드](figures/F9_marker_layout.png)

### C. 카메라·차량 장착
- [ ] C920을 차에 단단히 고정한다(주행 중 흔들리면 그만큼 자세가 틀어진다).
- [ ] **장착 위치 실측** → ROS `base_link → camera_link` 정적 TF 값이 된다 (`PLAN_ROS.md` §2.1). 기준점 `base_link` = **뒷바퀴 축 중심을 바닥에 내린 점**.

  | 항목 | 재는 법 | 값 |
  |---|---|---|
  | x 앞뒤 (m) | 뒷축 중심 → 렌즈 중심, 앞이 + | |
  | y 좌우 (m) | 차 중심선 → 렌즈 중심, 왼쪽이 + | |
  | z 높이 (m) | 바닥 → 렌즈 중심 | |
  | pitch (°) | 카메라가 아래로 숙인 각, 아래가 + (수평계 앱/각도기) | |
  | yaw (°) | 차 중심선과 카메라 정면이 틀어진 각, 왼쪽이 + (보통 0) | |
  | 축간거리 L (m) | 앞축 중심 ↔ 뒷축 중심 (나중에 자전거 모델용) | |
  | 바퀴 지름 (m) · 최대 조향각 (°) | 나중에 운동 모델용 | |

  광학 중심은 C920 앞 유리보다 몇 mm 안쪽이다. 렌즈 앞면까지 재고 그대로 적어도 된다(오차 수 mm). 차 사진도 옆·위에서 한 장씩 찍어 둔다.
- [ ] USB를 꽂은 뒤 **매번** 카메라 설정을 다시 넣는다(재연결하면 초기화됨):
  ```bash
  v4l2-ctl -d /dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0 \
    -c focus_automatic_continuous=0 -c focus_absolute=19 -c auto_exposure=1 -c exposure_time_absolute=156
  v4l2-ctl -d /dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0 -l | grep -E 'focus|exposure'
  ```
  (초점 19는 캘리브레이션 때 값과 같다고 가정한 것이다. 노출은 현장 밝기에 맞게 바꿔도 되지만, 짧을수록 블러가 적다.)
- [ ] `/dev/video*` 번호는 바뀔 수 있다. 주행 노드는 이름(C920)으로 카메라를 찾는다.

### D. 안전 (매 시험)
- [ ] 첫 실행은 **바퀴를 띄운 채로**. W 한 번 → 바퀴가 돌고 Space → 정지하는지 확인.
- [ ] 보드 응답: 시작 3초 안에 "보드 응답 확인"이 떠야 한다(빨간 경고가 뜨면 펌웨어·baud 확인).
- [ ] 워치독 확인(바퀴 띄운 상태): 주행 중 USB를 뽑으면 0.5초 안에 멈춰야 한다.
- [ ] 조향 POT 값이 A/D 키에 따라 변하는지 확인(오늘 세션에서는 461~462로 고정이었다. 미장착이거나 정지 상태였을 것).
- [ ] 트랙 위에서는 한 사람이 차 옆에서 잡을 준비, 낮은 속도부터(`-p max_speed:=60 -p step:=10` 권장).

### E. 주행 한 번의 순서
1. `~/demo/drive.sh --max-speed 40 --landmarks ~/track_landmarks.csv` — 카메라 설정·포트·디스크 사전점검과 안전 확인 질문 후 주행 시작(복사본 노드, Ctrl+C 영상 수정 포함). 원본으로 하려면 `cd ~/car_drive_exp && ./run.sh -p video_size:=1920x1080 -p max_speed:=60 -p step:=10` (이때는 C의 v4l2 설정을 먼저).
2. "● 영상 녹화 … C920" 문구와 보드 응답 확인
3. **출발 위치에서 3초 정지**(마커 3개 이상이 보이게)
4. 천천히 주행. 곡선은 특히 천천히.
5. **종료는 반드시 `Q`.** Ctrl+C로 끝내면 `video_frames.csv`가 비고 mkv 트레일러가 깨진다(T6에서 재현). 그렇게 됐더라도 t7이 시각을 복구하므로 영상은 버리지 말 것.
6. 터미널 요약 확인: 영상 프레임 수가 0보다 크고 fps가 약 30인지.
7. 세션 폴더 이름과 그 주행에서 한 일을 메모한다(직진·곡선·속도).

### F. 주행 직후 확인 (현장)
`drive.sh`로 주행했다면 Q 종료 후 결과 영상이 자동으로 생긴다: `~/demo/out/videos/<세션>_clean.mp4` (위 3D 궤적 · 아래 카메라+마커 축). 원본 `run.sh`로 했다면:
```bash
PYTHONPATH=~/aruco_slam_deps:~/aruco-slam/thirdparty/pangolin python3 -B ~/demo/live/render_clean.py --session ~/car_drive_exp/log/<세션> --landmarks ~/track_landmarks.csv
```
수치 확인:
```bash
cd ~/demo/scripts
python3 -B t7_session_slam.py ~/car_drive_exp/log/<세션> --fixed --landmarks ~/track_landmarks.csv
#   → ~/demo/out/session_<세션>/trajectory_session.csv · summary.json
```
- [ ] `frames_with_marker / frames` 비율: 90% 미만이면 마커를 보강하거나 속도를 낮춘다.
- [ ] 정합 잔차가 10 cm보다 큰 마커는 좌표를 다시 실측한다.
- [ ] 원본 `run_slam.py`를 쓸 경우 `outputs/`가 덮어써지므로, 결과를 세션별 폴더로 바로 복사한다.

---

## 4. 스모크 테스트 결과 (하드웨어를 움직이지 않는 항목만)

| # | 항목 | 결과 |
|---|---|---|
| S1 | aruco-slam 의존성 import (cv2·aruco·gtsam·pangolin·sympy) | ✓ |
| S2 | EKF 회전 규약 단일 검사 / 합성 주행 (T1) | ✓ 수행 → 원본 버그 확인, 수정본 정상 |
| S3 | 캘리브레이션 거리 정확도 (T2) | ✓ 현재 값이 최선, +4.5% 편향 |
| S4 | 캘리브레이션 재현·불확실도 (T3) | ✓ RMS 0.729 px 재현 |
| S5 | 복도 SLAM 재현·정합 (T4) | ✓ map.txt 동일, 잔차 동일 |
| S6 | 처리량·떨림 (T5, T5b) | ✓ 1080p 24~32 fps, 720p 26~37 fps |
| S7 | car_drive 사본 빌드(colcon) + 단위 테스트 13개 | ✓ |
| S8 | 펌웨어 컴파일 (`flash.sh --compile-only`, 업로드 없음) | ✓ 4648 B |
| S9 | 주행 노드 + 가짜 보드: 시작 순서가 **V0 → G**(펌웨어 기본 속도 100으로 바로 출발하지 않음), keepalive 약 8회/초, W/A/D/Space 명령, 종료 시 V0·F·S | ✓ 가짜 보드의 마지막 명령이 F, S이고 can_go=false |
| S10 | 실제 C920 1080p30 녹화 + `video_frames.csv` (Q 종료) | ✓ 324프레임, 29.35 fps |
| S11 | Ctrl+C 종료 | ✗ (알려진 버그 재현): frames 0, mkv 트레일러 없음. 정지 명령은 정상 전송 |
| S11' | 위 버그 수정본 (demo 사본, `start_new_session=True`) | ✓ Ctrl+C로 끝내도 331프레임, 29.9 fps, ffmpeg 남지 않음 |
| S12 | 녹화 중 카메라 동시 사용 | ✓ 확인 수행 → **불가**(구조상 제약) |
| S13 | 세션 → 3D 궤적 파이프라인 (T7), 시각 복구 | ✓ 복구 오차 0.0 ms, 정지 표준편차 < 0.8 cm |
| S14 | 720p/1080p 화각 동일성 (T8), 3D 뷰어 창 (T9) | ✓ |

하드웨어가 있어야 해서 **이번에 못 한 것**: 실제 Mega와의 통신, 모터와 조향 방향, 워치독 실동작, POT 값 변화, 주행 중 블러와 검출률, 실제 트랙에서의 정합. 이 항목들은 §3 D~F에서 확인한다.

---

## 5. 작업 기록 (한 일, 순서대로)

1. 다섯 폴더(aruco-slam, aruco_slam_deps, aruco_slam_deps_cv2_backup, car_calibration, car_drive_exp)를 훑어보고 요약했다(앞 대화).
2. `~/demo` 생성. 원본 파일 412개의 크기·수정시각과 두 레포의 git status를 스냅샷으로 저장했다(`out/*_before.txt`).
3. aruco-slam 코드를 읽었다(EKF, base_filter, 2D/3D 뷰어, trajectory_writer, run_slam). `git diff`와 `.bash_history`로 로컬 수정 5건과 실행 방식을 파악했다.
4. 회전 규약을 손으로 추적해서 `add_marker()`만 다른 규약을 쓴다는 걸 찾았다. → **T1** 합성 검사·시뮬레이션으로 확인했다.
5. 캘리브레이션 후보 3개(현재·예전·upstream)의 fx가 19% 다른 걸 발견했다. → **T2** 실측 거리 영상 6개로 비교, **T3** 이미지 세트 2개로 재계산.
6. 복도 결과를 확인했다. 쿼터니언 순서 두 가지 가설을 `camera_pose_relative.csv`와 대조해서 올바른 순서였음을 확인했다. → **T4** 복도 영상을 GUI 없이 다시 돌려(원본·수정본) 결과가 같음을 확인. JSON 직렬화 오류(int32)가 한 번 나서 고쳤다.
7. **T5** 떨림과 처리량 측정.
8. 하드웨어 확인: C920 연결, Mega 미연결. 아두이노가 없어서 펌웨어 로직을 흉내 내는 `fake_board.py`를 작성했다.
9. car_drive_exp를 `demo/car_drive_exp_copy`로 복사(log·build·.git 제외) → colcon 빌드, pytest 13개 통과.
10. **T6** 주행 노드 스모크. 첫 시도는 스크립트의 `set -u` 때문에 ROS setup.bash가 실패해서 무효였다. 남은 socat·fake_board 프로세스를 정리하고 고친 뒤 다시 돌렸다. 카메라 동시 사용 검사도 처음엔 경로 문자열 때문에 무효여서 인덱스로 다시 했다.
11. Ctrl+C 버그를 재현했다. 원인은 터미널 SIGINT와 stop()의 SIGINT가 겹쳐 ffmpeg가 두 번 신호를 받는 것이다. demo 사본의 `video_recorder.py`에만 `start_new_session=True`를 넣고 다시 시험해서 해결을 확인했다. 패치 파일만 남겼다.
12. **T7** 세션 → 3D 궤적 파이프라인 작성. 시각 복구가 정확한지 정상 세션으로 확인했다.
13. **T8** 720p/1080p 화각 비교(카메라 고정, 2초씩 촬영). **T9** Pangolin 창 실행. 펌웨어 컴파일만 확인.
14. 원본 스냅샷을 다시 떠서 비교: 412개 파일 모두 같고 git status도 같다.
15. 이 문서를 작성했다.
16. (추가 요청) 보고서용 그림 9장을 `make_figures.py`로 만들었다. 모든 그림은 위 시험 결과 파일에서 직접 그린 것이다.
    - 시스템 Noto CJK 폰트가 TTC 형식이라 matplotlib 3.5가 한글을 □로 그렸다. fontTools로 KR 면만 OTF로 뽑아 `work/fonts/`에 두고 등록해서 해결했다. 이 과정에서 앞서 sed로 넣은 주석이 글꼴 크기 설정까지 주석 처리하던 실수도 함께 고쳤다.
    - 처리량 그림을 그리다가, 구성 요소별 시간 합(33 ms)과 T4 로그의 전체 시간(41 ms)이 맞지 않는 걸 발견했다. → **T5b**로 단독 실행을 다시 재서 31 ms / 27 ms를 얻었다. T4 값은 두 실행을 동시에 돌린 탓이었다. §0·§2.2·§4 수치를 고쳤다.
    - 그림을 하나씩 열어 보며 라벨 겹침(F2·F5·F6·F8)과 여백(F7)을 고쳤다. `t1_ekf_convention.py`에는 그림용으로 궤적을 꺼내는 `TRACE`만 추가했다(결과 수치는 같다).

---

## 6. 원본에 적용을 권하는 패치 (적용하지 않음 — 결정은 사용자)

1. `car_drive_exp/src/car_drive/car_drive/video_recorder.py`: ffmpeg `Popen(..., start_new_session=True)`. 파일은 `out/video_recorder_ctrlc_fix.patch`. 단점: 노드가 SIGKILL로 죽으면 ffmpeg가 남아 카메라를 잡고 있을 수 있다(`pkill -INT ffmpeg`로 정리).
2. `aruco-slam/filters/extended_kalman_filter.py` `add_marker()`: `t_ml = rot_cm @ p + t`를 `t_ml = rot_mc @ p + t`로(= `inv` 제거). demo의 `EKFFixedInit`과 같은 효과다. `ekf_with_rotations.py`는 이번에 검증하지 않았다.
3. (선택) `run_slam.py`의 출력 경로를 인자로 받게 해서, 결과를 덮어쓰지 않게.

---

## 7. demo 폴더 구조와 재현 명령

```
~/demo/
  REPORT.md                    이 문서
  scripts/
    common.py                  원본을 import만, cwd=work, 출력=out
    t1_ekf_convention.py       회전 규약 검사 + 합성 주행 (원본/수정본)
    t2_calib_distance.py       실측 거리 영상으로 캘리브레이션 비교
    t3_calib_recheck.py        캘리브레이션 재계산·불확실도
    t4_slam_headless.py        영상 → SLAM(GUI 없음) [+ --fixed, --landmarks 정합]
    t5_jitter_perf.py          떨림·처리량 (구성 요소별)
    t5b_perf_e2e.py            처리 시간 실측 (1080p·720p, 단독 실행)
    make_figures.py            figures/F1~F9 생성
    t6_drive_smoke.sh          주행 노드 + 가짜 보드 + C920 (q | ctrlc)
    fake_board.py              펌웨어 로직을 흉내 내는 가짜 Mega (pty)
    t7_session_slam.py         주행 세션 → t_s 축 3D 궤적 (내일 사용)
    t8_res_fov.py              720p/1080p 화각 비교
    t9_viewer3d.py             Pangolin 창 시험
  work/calibration/            현재 캘리브레이션 사본
  work/fonts/                  한글 그림용 Noto Sans CJK KR (TTC에서 추출한 OTF)
  figures/                     F1~F9 PNG (아래 표)
  car_drive_exp_copy/          주행 패키지 사본 (Ctrl+C 수정 적용본, 빌드 포함)
  out/                         모든 결과(JSON·로그·세션 3개·궤적·패치·원본 스냅샷)
```
모든 파이썬 스크립트는 `cd ~/demo/scripts && python3 -B <스크립트>`로 실행한다. `-B`는 원본 레포에 `__pycache__`를 만들지 않게 하는 옵션이다.

## 8. 그림 목록

| 그림 | 내용 | 데이터 |
|---|---|---|
| F1 `F1_ekf_rotation_bug.png` | 원본 `add_marker` 회전 버그: yaw별 초기화 오차, 합성 선회 주행 궤적 | T1 |
| F2 `F2_calibration_distance.png` | 캘리브레이션 3종의 실측 100/150 cm PnP 거리 | T2 |
| F3 `F3_corridor_alignment.png` | 복도 실험 재현: 정합된 카메라 궤적, 실측 대비 SLAM 마커, 잔차 | T4 |
| F4 `F4_static_stability.png` | 카메라 고정 상태의 x/y/z 흔들림 (실제 C920) | T6+T7 |
| F5 `F5_throughput.png` | 프레임당 처리 시간, 30/15 fps 기준선 | T5b, T5 |
| F6 `F6_drive_smoke.png` | 주행 노드 → 가짜 보드 명령 타임라인, 종료 방식별 영상 프레임 기록 | T6 |
| F7 `F7_camera_view.png` | 현재 C920 화면의 마커 검출과 PnP 거리 | T6 세션 영상 |
| F8 `F8_pipeline.png` | 내일 쓰는 데이터 흐름, 실시간이 막히는 이유, 나중 구조(제안) | 구조도 |
| F9 `F9_marker_layout.png` | 마커 배치 가이드(화각 66°, 0.5~4 m, 규칙 6개) | 체크리스트 §3-B |

## 9. 수동 주행 → 로컬라이제이션 영상 (추가)

| 파일 | 역할 |
|---|---|
| `drive.sh [record\|live]` | 사전 점검(C920 by-id·초점 19/노출 156 재설정·아두이노·디스크·안전 확인) → 주행(복사본 run.sh) → SLAM → 결과 영상 |
| `live/render_clean.py` | **깔끔한 영상 (drive.sh 기본, `desiered.png` 형식)**: 위는 3D(초록 궤적·검은 프러스텀·파란 마커, 보이는 마커는 빨간 원), 아래는 카메라+마커 축. 글자 없음 |
| `live/render_localization.py` | 대시보드 영상 (`--style full`): 카메라+ID·거리, 위에서 본 지도, 3D 뷰, PWM/조향/마커 수 타임라인 |
| `live/live_slam.py` | live 모드: 주행 중 C920을 맡아 실시간 SLAM(2D/3D 창, `--ros` 시 /car/cam_pose) |
| `aruco-slam-fixed/` | EKF add_marker 회전 수정본 (모든 렌더러·live가 사용) |
| `scripts/t10_drive_sh_smoke.sh` | drive.sh를 가짜 보드(pty)와 자동 키 입력으로 끝까지 시험 |

검증(바퀴 안 움직임): T10 record 모드 → 세션 20261120_010050(422프레임, 마커 검출 422/422) → 영상 생성 OK.
live_slam 실제 C920 시험: 720p 27 fps, 1080p 22 fps 처리, ROS 토픽 수신 확인. 복도 영상 + 실측 마커 정합으로 두 형식 모두 렌더 확인.

실행: `~/demo/drive.sh --landmarks <트랙 실측 csv>` → `~/demo/out/videos/<세션>_clean.mp4`
