#!/usr/bin/env bash
# [demo] ROS 실시간 로컬라이제이션 + WASD 수동 주행 + bag 녹화 (PLAN_ROS.md P3).
#
#   ~/demo/ros_ws/drive_ros.sh [옵션]
#     --source camera|video   camera(기본) = C920 · video = 녹화 영상을 카메라 대신 (하드웨어 없는 시험)
#     --video <mkv>  --rate 1.0   (source=video)
#     --size 1920x1080|1280x720   --mount <yaml> (기본 car_bringup/config/mount.yaml)
#     --port /dev/ttyACM0 (기본 자동)  --max-speed 60  --step 10
#     --landmarks <csv>  실측 마커(id,x_m,y_m,z_m) → TF track→map · /car/pose_track · 영상도 트랙 좌표
#     --rviz     rviz 창 (키 입력은 계속 이 터미널)   --render clean|full|none  주행 후 bag → 결과 영상 (기본 clean)
#     --no-bag   bag 녹화 안 함       --yes   안전 확인 질문 생략 (시험용)
#
# 순서: 사전 점검 → [백그라운드] 카메라·SLAM·TF·bag (bringup.launch.py, 별도 세션) → SLAM 첫 프레임 확인
#       → [전경] wasd_drive (camera:=none, 키 입력은 이 터미널) → Q/Ctrl+C → bringup 에 SIGINT 1번 → bag 닫힘 확인 → 요약
# 결과: ~/demo/sessions/<시각>/  { bag/ · slam/ · <주행 세션>/ · bringup.log }
# 주행 안전은 SLAM 과 무관: bringup 이 죽어도 wasd_drive + 펌웨어 워치독(500 ms)은 그대로.
# 2026-11-21: dialout 에 추가됐지만 아직 재로그인 전이면(현재 셸에 그룹이 안 붙음) sg 로 그룹을 붙여 자기 자신을 다시 실행
#   → 재로그인 없이도 /dev/ttyACM0 권한 오류(Errno 13) 안 남. 재로그인 후엔 이 줄은 아무것도 안 함
if ! id -nG | grep -qw dialout && getent group dialout | cut -d: -f4 | tr , '\n' | grep -qx "$USER" && [ -z "${_DIALOUT_REEXEC:-}" ]; then
  export _DIALOUT_REEXEC=1
  exec sg dialout -c "$(printf '%q ' "$(readlink -f "$0")" "$@")"
fi
WS="$HOME/demo/ros_ws"
SOURCE=camera; VIDEO=""; RATE=1.0; SIZE=1920x1080; MOUNT=""; PORT=""; MAXSPD=60; STEP=10; BAG=true; YES=0; LMK=""; RVIZ=false; RENDER=clean
while [ $# -gt 0 ]; do
  case "$1" in
    --source) SOURCE="$2"; shift ;; --video) VIDEO="$(readlink -f "$2")"; shift ;; --rate) RATE="$2"; shift ;;
    --size) SIZE="$2"; shift ;; --mount) MOUNT="$(readlink -f "$2")"; shift ;; --port) PORT="$2"; shift ;;
    --max-speed) MAXSPD="$2"; shift ;; --step) STEP="$2"; shift ;; --no-bag) BAG=false ;; --yes) YES=1 ;;
    --landmarks) LMK="$(readlink -f "$2")"; shift ;; --rviz) RVIZ=true ;; --render) RENDER="$2"; shift ;;
    -h|--help) sed -n 2,19p "$0"; exit 0 ;;
    *) echo "모르는 옵션: $1"; exit 2 ;;
  esac; shift
done
red() { printf '\033[31m%s\033[0m\n' "$*"; }; grn() { printf '\033[32m%s\033[0m\n' "$*"; }; ylw() { printf '\033[33m%s\033[0m\n' "$*"; }
# shellcheck disable=SC1091
source "$WS/env.sh"
[ -f "$WS/install/setup.bash" ] || { red "작업공간 빌드 안 됨: $WS/build.sh"; exit 1; }
SHARE="$(ros2 pkg prefix car_bringup)/share/car_bringup"
[ -n "$MOUNT" ] || MOUNT="$SHARE/config/mount.yaml"
FAIL=0
echo "━━ 사전 점검 (source $SOURCE · $SIZE · 최고 PWM $MAXSPD · step $STEP · bag $BAG) ━━"
C920=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0
if [ "$SOURCE" = camera ]; then
  if [ -e "$C920" ]; then
    grn "✓ C920: $(readlink -f $C920)"
    fuser "$(readlink -f $C920)" >/dev/null 2>&1 && { red "✗ 카메라를 다른 프로그램이 쓰는 중: $(fuser "$(readlink -f $C920)" 2>&1)"; FAIL=1; }
  else red "✗ C920 없음 — USB 확인"; FAIL=1; fi
else
  [ -f "$VIDEO" ] && grn "✓ 영상 소스: $VIDEO (${RATE}배속, 끝나면 SLAM·bag 종료)" || { red "✗ --video 파일 없음: $VIDEO"; FAIL=1; }
fi
if [ -n "$PORT" ]; then [ -e "$PORT" ] && grn "✓ 포트(지정): $PORT" || { red "✗ 포트 없음: $PORT"; FAIL=1; }
else
  P="$(python3 -c "from car_drive.serial_link import find_port; print(find_port() or '')" 2>/dev/null)"
  [ -n "$P" ] && grn "✓ 아두이노: $P" || { red "✗ 아두이노 포트 없음 (또는 --port)"; FAIL=1; }
fi
id -nG | grep -qw dialout || ylw "! dialout 그룹 아님 — 포트 권한 오류 시 sudo usermod -aG dialout $USER"
python3 - "$MOUNT" <<'PY' || FAIL=1
import sys, yaml
m = yaml.safe_load(open(sys.argv[1])) or {}
k = ["x", "y", "z", "pitch_down_deg"]
v = {x: float(m.get(x, 0)) for x in k}
print(("\033[32m✓" if any(v.values()) else "\033[33m!") + f" 장착값 {sys.argv[1]}: " + " ".join(f"{x}={v[x]:g}" for x in k)
      + ("" if any(v.values()) else "  (모두 0 → base_link = 카메라 위치. 실측하면 이 파일에)") + "\033[0m")
PY
SCAL="$SHARE/config/steer_calib.yaml"
python3 - "$MOUNT" "$SCAL" <<'PY'
import os, sys, yaml
L = float((yaml.safe_load(open(sys.argv[1])) or {}).get("wheelbase", 0) or 0)
print(("\033[32m✓" if L > 0 else "\033[33m!") + f" 축간거리 {L:g} m" + ("" if L > 0 else "  (0 → delta_from_motion 없음, 조향 보정 불가 — mount.yaml wheelbase 실측)") + "\033[0m")
if os.path.exists(sys.argv[2]):
    c = yaml.safe_load(open(sys.argv[2]))
    print(f"\033[32m✓ 조향 보정 {c.get('generated')}: 직진 POT {c.get('pot_center')} · 좌 {c.get('max_left_deg')}° / 우 {c.get('max_right_deg')}° · 유격 {c.get('backlash_adc')} ADC\033[0m")
else:
    print("\033[33m! 조향 보정 없음 → /car/state 의 delta = NaN (x, y, θ, v 는 나옴). 만들기: ~/demo/ros_ws/calib_steer.sh\033[0m")
PY
if [ -n "$LMK" ]; then
  [ -f "$LMK" ] && grn "✓ 실측 마커: $LMK ($(($(wc -l < "$LMK") - 1))개) → track 좌표" || { red "✗ 실측 마커 파일 없음: $LMK"; FAIL=1; }
fi
AVAIL=$(df --output=avail -BM "$HOME" | tail -1 | tr -dc 0-9)
case "$SIZE" in 1920x1080) MBPM=690 ;; *) MBPM=240 ;; esac
[ "$BAG" = true ] || MBPM=5
MIN=$((AVAIL / MBPM))
if [ "$AVAIL" -lt 1024 ]; then red "✗ 디스크 여유 ${AVAIL} MB"; FAIL=1
elif [ "$MIN" -lt 10 ]; then ylw "! 디스크 여유 $((AVAIL / 1024)) GB → bag 약 ${MIN}분"
else grn "✓ 디스크 여유 $((AVAIL / 1024)) GB → bag 약 ${MIN}분"; fi
[ "$FAIL" = 1 ] && { red "사전 점검 실패 — 중단"; exit 1; }
if [ "$YES" != 1 ]; then
  ylw "안전 확인: 첫 시험은 바퀴를 띄웠나요? 주행 공간이 비었고 차 옆에 잡을 사람이 있나요? 종료는 Q."
  read -r -p "진행할까요? [y/N] " ans; [ "$ans" = y ] || [ "$ans" = Y ] || { echo "취소"; exit 0; }
fi

RUN="$HOME/demo/sessions/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$RUN"; cp "$MOUNT" "$RUN/mount.yaml"
L_ARGS=(run_dir:="$RUN" source:="$SOURCE" size:="$SIZE" mount_file:="$MOUNT" record:="$BAG" rate:="$RATE" rviz:="$RVIZ")
[ -n "$LMK" ] && { L_ARGS+=(landmarks:="$LMK"); cp "$LMK" "$RUN/landmarks.csv"; }
[ "$SOURCE" = video ] && L_ARGS+=(video:="$VIDEO")
[ -f "$SCAL" ] && cp "$SCAL" "$RUN/steer_calib.yaml"
# 별도 세션: 터미널 Ctrl+C 가 bringup 으로 가지 않게 (끝낼 때 아래에서 SIGINT 를 launch 에만 한 번).
# 비대화형 bash 의 백그라운드(&)는 SIGINT 가 '무시'로 시작 → 그대로 띄우면 launch 가 SIGINT 를 못 받아 bag 이 안 닫힌다.
# bash 로는 되돌릴 수 없어서 python 으로 기본값으로 되돌린 뒤 exec.
setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 launch car_bringup bringup.launch.py "${L_ARGS[@]}" > "$RUN/bringup.log" 2>&1 &
LP=$!
stop_bringup() {
  kill -0 $LP 2>/dev/null || return 0
  kill -INT $LP 2>/dev/null
  for _ in $(seq 1 150); do kill -0 $LP 2>/dev/null || break; sleep 0.1; done
  kill -0 $LP 2>/dev/null && { ylw "! bringup 이 15 s 안에 안 끝남 → 강제 종료"; kill -TERM -$LP 2>/dev/null; sleep 2; kill -KILL -$LP 2>/dev/null; }
}
trap 'stop_bringup' EXIT
echo -n "SLAM 준비 대기"
for _ in $(seq 1 200); do
  grep -q "aruco_slam.*camera_info" "$RUN/bringup.log" 2>/dev/null && break
  kill -0 $LP 2>/dev/null || break; echo -n "."; sleep 0.1
done; echo
if ! grep -q "aruco_slam.*camera_info" "$RUN/bringup.log"; then
  red "✗ SLAM 이 영상을 못 받음 (20 s) — $RUN/bringup.log:"; grep -E 'ERROR|error|실패|없음' "$RUN/bringup.log" | tail -5; exit 1
fi
grn "✓ 카메라 → SLAM 동작 · 기록 $RUN  (rviz: ros2 run rviz2 rviz2 — 키 입력은 이 터미널에)"

echo "━━ 주행 (W/S 속도 · A/D 조향 · Space 정지 · Q 종료) ━━"
D_ARGS=(-p camera:=none -p log_dir:="$RUN" -p max_speed:="$MAXSPD" -p step:="$STEP")
[ -n "$PORT" ] && D_ARGS+=(-p port:="$PORT")
# Ctrl+C: wasd_drive 는 기본 처리(정지 명령 후 종료), 이 스크립트는 빈 핸들러로 살아남아 아래 정리를 끝까지 한다
trap ':' INT
CAR_DRIVE_ROOT="$HOME/demo/car_drive_exp_copy" ros2 run car_drive wasd_drive --ros-args "${D_ARGS[@]}"
RC=$?
trap - INT

echo "━━ 정리: SLAM·bag 종료 ━━"
stop_bringup; trap - EXIT
SESS="$(ls -1d "$RUN"/2*/ 2>/dev/null | tail -1)"
if [ "$BAG" = true ]; then
  if [ -f "$RUN/bag/metadata.yaml" ]; then
    grn "✓ bag 닫힘: $RUN/bag ($(du -sh "$RUN/bag" | cut -f1))"
    ros2 bag info "$RUN/bag" 2>/dev/null | grep -E 'Duration|Messages:|compressed|/car/pose |/car/pwm |/camera/camera_info' | sed 's/^/   /'
  else red "✗ bag metadata.yaml 없음 — 정상 종료 안 됨 ($RUN/bringup.log 확인)"; fi
fi
[ -f "$RUN/slam/summary.json" ] && python3 -c "
import json; d=json.load(open('$RUN/slam/summary.json'))
print(f\"   SLAM: 수신 {d['frames_received']} · 처리 {d['frames_processed']} · 마커 보인 프레임 {d['frames_with_marker']} · 처리 {d['proc_ms_median']} ms · 지연 {d['latency_ms_median']} ms · 지도 마커 {d['landmarks']}\")"
[ -n "$SESS" ] && echo "   주행 세션: $SESS"
grep -o '최종 정합.*' "$RUN/bringup.log" | sed 's/^/   /'
grep -o '\[car_state\] 발행.*' "$RUN/bringup.log" | sed 's/^\[car_state\] /   차량 상태: /'
if [ "$BAG" = true ] && [ "$RENDER" != none ] && [ -f "$RUN/bag/metadata.yaml" ]; then
  echo "━━ 결과 영상 ($RENDER, 주행 시간의 약 2배) ━━"
  E_ARGS=(--render "$RENDER"); [ -n "$LMK" ] && E_ARGS+=(--landmarks "$LMK")
  ros2 run aruco_slam_ros bag_export "$RUN/bag" "${E_ARGS[@]}" 2>&1 | grep -E 'bag_export\] (/|영상)|정합|Error' 
fi
grep -c Traceback "$RUN/bringup.log" | grep -qv '^0$' && ylw "! bringup.log 에 Traceback 있음"
echo "결과: $RUN   (주행 노드 rc=$RC)"
