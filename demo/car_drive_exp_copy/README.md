# car_drive_exp — 실차 WASD 주행 (실험용)

실차 노트북(Ubuntu + ROS 2)에서 **주행 노드 하나만** 띄운다. Unity·실시간 모니터는 기본으로 뜨지 않는다.

## 처음 한 번

```bash
git clone https://github.com/Yulee3542/car_drive_exp.git ~/car_drive_exp
cd ~/car_drive_exp
sudo apt install -y python3-serial python3-colcon-common-extensions
sudo usermod -aG dialout $USER     # 그 뒤 로그아웃·로그인
./flash.sh                         # 실차 메가에 firmware/car_controller 업로드 (메가를 USB로 꽂은 채)
```

## 실행

```bash
cd ~/car_drive_exp && git pull && ./run.sh
```

| 키 | 동작 |
|---|---|
| W | 전진 가속 (+20, 최고 150) |
| S | 감속 → 0 아래로 내려가면 후진 |
| A / D | 좌 / 우 조향 (펄스 120 ms, 누르고 있으면 계속) |
| Space | 정지 (속도 0 + 조향 정지) |
| Q | 종료 (정지 명령 보내고 끝) |

- 주행 허용(G)·정지(S)는 노드가 알아서 보낸다. 시작할 때 속도 0으로 시작한다.
- 노트북이나 노드가 죽으면 펌웨어 워치독이 **0.5초 안에** 차를 세운다(노드가 0.1초마다 속도를 다시 보냄).
- 옵션: `./run.sh -p port:=/dev/ttyACM0 -p max_speed:=120 -p step:=10`
- 처음 시험은 **바퀴를 띄운 채로**.

## 세션 기록 (실행할 때마다 자동)

시작할 때 `● 기록 시작: …`, 끝날 때 `● 기록 저장: …`과 요약(주행 시간·키 횟수·최고 |PWM|·조향 POT 범위·보드 응답·바퀴 엔코더 순 카운트)을 터미널에 띄운다.

레포의 `log/` 아래에 **세션마다 폴더 하나**.

```
log/<YYYYmmdd_HHMMSS>/
  drive_log.csv     t_s,kind,value   kind = key(누른 키) · tx(보낸 명령 V<pwm>/L/R/F/G/S) · pot(조향 ADC) · enc(바퀴 엔코더 카운트) · state(0/1/2) · note
  encoder.csv       t_s,board_ms,count   뒷바퀴 엔코더 (약 20 Hz). board_ms = 카운트를 읽은 보드 millis()
  serial_raw.log    보드에서 받은 원문 줄 (t_s<TAB>줄)
  session.json      시작·끝 시각, 종료 이유(q/ctrl+c/error), 포트, 설정, git 커밋, 펌웨어 해시, 요약, 영상 정보
  video.mkv         (C920 있을 때) 카메라 영상
  video_frames.csv  frame,t_s
  video_ffmpeg.log  ffmpeg 오류 출력
log/last_build.txt  마지막 colcon 빌드 출력
```

- `t_s` = 세션 시작부터의 초. `log/`는 git에 올라가지 않는다(노트북에만 남음). 같은 초에 두 번 실행하면 `<시각>-2/`.
- **영상**: C920이 꽂혀 있으면 `video.mkv`(MJPEG 그대로 저장, 재인코딩 없음)와
  `video_frames.csv`(frame,t_s — drive_log.csv와 같은 시간축)를 남긴다. t_s에는 카메라·USB 지연(수십 ms)이 들어 있다.
  - 카메라는 이름(C920)으로만 고른다. 없으면 노트북 내장 캠으로 바꾸지 않고 영상 없이 기록한다.
  - 다른 카메라: `-p camera:=/dev/video0` · 끄기: `-p camera:=none` · 해상도: `-p video_size:=640x480`
  - 용량: 1280×720 MJPEG ≈ **분당 240 MB** — 오래 돌릴 땐 해상도를 낮추거나 오래된 세션 폴더를 옮길 것
  - 재생(노트북): 파일 탐색기에서 더블클릭(mpv) 또는 `mpv log/<세션>/video.mkv` (스페이스 일시정지 · ←/→ 5초 이동 · q 닫기). `ffplay`도 된다.
- 연결 후 3초 동안 보드 응답(펌웨어 줄)이 없으면 **빨간 경고**를 띄우고 `note,no_board_reply`를 남긴다 — 펌웨어·baud 문제를 바로 알 수 있게.
- 기록 끄기: `./run.sh -p record:=false` · 다른 위치: `CAR_DRIVE_LOG=/path ./run.sh`

## 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/car/pwm` | Int16 | 보내는 속도 명령 −255..255 (음수 = 후진) |
| `/car/steer_cmd` | String | 보낸 조향 명령 L / R / F |
| `/car/steering_pot` | Int32 | 조향 POT ADC (약 20 Hz) |
| `/car/wheel_encoder` | Int32 | 뒷바퀴 엔코더 누적 카운트 (약 20 Hz, 1회전 = 80) |
| `/car/state` | Int8 | 펌웨어 상태 0 정지 / 1 전진 / 2 후진 |

## 펌웨어

`firmware/car_controller/car_controller.ino` = autodrive_skku `main`(a300036)의 `arduino/car_controller/car_controller.ino` 사본.
9600 bps · `G` 주행 허용 · `S` 정지 · `V<-255..255>` 속도 · `L`/`R` 조향 펄스 · `F` 조향 정지 · 출력 `0/1/2` 상태 · `P <adc>` · `E <count> <ms>`.
초음파 출력은 없다(필요하면 이 사본에 추가).

**바퀴 엔코더 (KY-040, 뒷바퀴)** — 2026-11-21 추가
- 배선: `CLK → D33` · `DT → D32` · `+ → 5V` · `GND → GND` (`SW`는 안 씀). 32·33은 인터럽트 핀이 아니라 펌웨어가 Timer0 1 kHz + loop 폴링으로 읽는다.
- 4체배 디코딩 → KY-040(EC11 20클릭) **1회전 = 80 카운트**. 50 ms마다 `E <누적 카운트> <보드 millis>`.
- 카운트는 포트를 열 때(보드 리셋) 0부터. **전진 = +** (2026-11-21 실차 확인, 펌웨어 `ENC_DIR = -1` 로 맞춤 — 엔코더를 반대로 다시 달면 이 값을 뒤집을 것).
- 엔코더가 안 꽂혀 있어도 `E 0 …`이 계속 나온다(내부 풀업). 주행했는데 요약에 "카운트 변화 없음"이면 배선 확인.
- 속도 = Δcount / Δboard_ms (encoder.csv). 바퀴 둘레 C [m]면 v = Δcount / 80 · C / Δt. 50 ms 창 하나로는 1카운트 차이가 크니 여러 줄을 묶어서 계산할 것.
- 9600 bps 대역: P·상태·E 합쳐 약 550 B/s (한도 960 B/s).

## 더미 (기본 꺼짐)

자리만 남겨 둔 것. 주행 노드와 같이 뜨지 않는다.

- `./run.sh --twin` — `twin_stub`: Unity 연결 없이 가짜 `/twin/car/steering_pot`만 낸다
- `./run.sh --monitor` — `monitor_stub`: 2초마다 `/car/*` 한 줄 요약 (웹 모니터 없음)

## 구조

```
run.sh  flash.sh  firmware/car_controller/
src/car_drive/car_drive/
  wasd_drive.py   ROS 2 노드 (키보드 → 시리얼, 토픽 발행, 세션 기록)
  session_log.py  세션 폴더(log/<시각>/)·CSV·원문·session.json
  video_recorder.py  ffmpeg 영상 기록 (C920 이름으로 찾기, 프레임 시각)
  drive.py        키 → 펌웨어 명령 (V0 → G 순서, 워치독 keepalive)
  serial_link.py  포트 찾기·연결·줄 해석
  extras/         twin_stub.py · monitor_stub.py (더미)
src/car_drive/test/test_drive.py   python3 -m pytest src/car_drive/test
```
