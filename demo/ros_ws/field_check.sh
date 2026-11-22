#!/usr/bin/env bash
# [demo] 현장 준비 점검 — 출발 전(소프트웨어)과 도착 후(하드웨어) 둘 다.  REPORT.md 체크리스트 A·B
#
#   ~/demo/ros_ws/field_check.sh [--landmarks <csv>] [--test]
#     --test  colcon 테스트까지 (약 30 s)
#
# ✓ 됨 · ! 현장에서 할 일(막지는 않음) · ✗ 이대로는 주행 불가
WS="$HOME/demo/ros_ws"
LMK=""; TEST=0
while [ $# -gt 0 ]; do
  case "$1" in --landmarks) LMK="$(readlink -f "$2")"; shift ;; --test) TEST=1 ;; -h|--help) sed -n 2,7p "$0"; exit 0 ;; esac; shift
done
red() { printf '\033[31m✗ %s\033[0m\n' "$*"; BAD=$((BAD + 1)); }
grn() { printf '\033[32m✓ %s\033[0m\n' "$*"; }
ylw() { printf '\033[33m! %s\033[0m\n' "$*"; TODO=$((TODO + 1)); }
BAD=0; TODO=0
source "$WS/env.sh"
SHARE="$(ros2 pkg prefix car_bringup 2>/dev/null)/share/car_bringup"
CFG="$WS/src/car_bringup/config"

echo "━━ 1. 소프트웨어 (출발 전) ━━"
for p in aruco_slam_ros car_bringup car_drive car_msgs car_state; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || red "패키지 $p 빌드 안 됨 → $WS/build.sh"
done
[ $BAD = 0 ] && grn "패키지 5개 빌드됨"
newer="$(find "$WS/src" -newer "$WS/install/setup.bash" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.msg' -o -name '*.rviz' \) 2>/dev/null | grep -v __pycache__ | head -3)"
[ -z "$newer" ] && grn "빌드가 소스보다 최신" || ylw "빌드 후 바뀐 파일 있음 → $WS/build.sh  ($(echo "$newer" | head -1 | sed "s#$WS/##") …)"
if [ $TEST = 1 ]; then
  r="$("$WS/build.sh" --test 2>&1 | grep -E '^Summary' | tail -1)"
  echo "$r" | grep -q ' 0 errors, 0 failures' && grn "테스트: $r" || red "테스트: $r"
fi
ros2 run aruco_slam_ros env_check >/dev/null 2>&1 && grn "SLAM 환경 (cv2·numpy·scipy·EKF)" || red "env_check 실패 → ros2 run aruco_slam_ros env_check"
ros2 interface show car_msgs/msg/CarState >/dev/null 2>&1 && grn "/car/state 메시지 (car_msgs/CarState)" || red "car_msgs 없음"
for f in camera_info_c920_1080p.yaml camera_info_c920_720p.yaml; do
  [ -f "$SHARE/config/$f" ] || red "카메라 보정 없음: $f"
done
python3 -I - "$CFG/mount.yaml" <<'PY'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1])) or {}
need = ["z", "pitch_down_deg", "wheelbase"]  # x 는 뺌: 카메라가 뒷축 바로 위라 x = 0 이 실측값 (2026-11-21)
zero = [k for k in need if not float(m.get(k, 0) or 0)]
if zero:
    print(f"\033[33m! 장착값 미측정: {', '.join(zero)} = 0  ({sys.argv[1]}) → 현장 체크리스트 C, 측정 후 build.sh\033[0m")
    sys.exit(1)
print(f"\033[32m✓ 장착값: x {m['x']} · z {m['z']} · pitch {m['pitch_down_deg']}° · yaw {m.get('yaw_deg', 0)}° · 축간거리 {m['wheelbase']} m\033[0m")
PY
[ $? = 0 ] || TODO=$((TODO + 1))
if [ -f "$CFG/steer_calib.yaml" ]; then
  grn "조향 보정: $(grep -E '^generated' "$CFG/steer_calib.yaml") · 직진 POT $(grep -E '^pot_center' "$CFG/steer_calib.yaml" | cut -d' ' -f2)"
else
  ylw "조향 보정 없음 → 현장에서 ~/demo/ros_ws/calib_steer.sh (장착값 wheelbase 먼저). 그 전엔 δ = NaN"
fi
if [ -n "$LMK" ]; then
  python3 -I - "$LMK" <<'PY'
import sys
import numpy as np
try:
    a = np.loadtxt(sys.argv[1], delimiter=",", skiprows=1, ndmin=2)
except Exception as e:
    print(f"\033[31m✗ 실측 마커 CSV 읽기 실패 ({e}) — 형식 id,x_m,y_m,z_m (첫 줄 머리글)\033[0m"); sys.exit(2)
ids = a[:, 0].astype(int)
bad = []
if a.shape[1] < 4: bad.append("열 4개(id,x,y,z) 아님")
if len(set(ids)) != len(ids): bad.append(f"ID 중복 {sorted(i for i in set(ids) if list(ids).count(i) > 1)}")
if len(ids) < 3: bad.append("마커 3개 미만")
if len(ids) >= 3:
    sv = np.linalg.svd(a[:, 1:3] - a[:, 1:3].mean(0), compute_uv=False)
    if sv[-1] < 0.05 * sv[0]: bad.append("거의 일직선 → 정합 불가")
if np.abs(a[:, 1:4]).max() > 100: bad.append("좌표가 100 m 넘음 — cm 로 적었나?")
if bad:
    print(f"\033[31m✗ 실측 마커 {sys.argv[1]}: {' · '.join(bad)}\033[0m"); sys.exit(2)
span = np.ptp(a[:, 1:3], axis=0)
print(f"\033[32m✓ 실측 마커 {len(ids)}개 (ID {ids.min()}~{ids.max()}) · 범위 {span[0]:.1f} × {span[1]:.1f} m\033[0m")
PY
  [ $? = 2 ] && BAD=$((BAD + 1))
else
  ylw "실측 마커 CSV 미지정 (--landmarks) — 없으면 map 좌표(출발점 기준)로만"
fi
AV=$(df --output=avail -BG "$HOME" | tail -1 | tr -dc 0-9)
RS=$(du -sm "$WS/runs" 2>/dev/null | cut -f1)
[ "$AV" -ge 10 ] && grn "디스크 여유 ${AV} GB (1080p bag 약 $((AV * 1024 / 690))분) · runs ${RS} MB" \
  || ylw "디스크 여유 ${AV} GB (1080p bag 약 $((AV * 1024 / 690))분) — runs ${RS} MB 정리 고려"
for b in /sys/class/power_supply/BAT*; do
  [ -e "$b/capacity" ] || continue
  c=$(cat "$b/capacity"); s=$(cat "$b/status")
  [ "$c" -ge 80 ] || [ "$s" = Charging ] || [ "$s" = Full ] && grn "노트북 배터리 ${c}% ($s)" || ylw "노트북 배터리 ${c}% ($s) — 충전"
done
[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && grn "화면 있음 (rviz 가능)" || ylw "DISPLAY 없음 — rviz 안 됨 (주행은 됨)"

echo "━━ 2. 하드웨어 (도착 후) ━━"
C920=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0
if [ -e "$C920" ]; then
  grn "C920: $(readlink -f $C920)"
  fuser "$(readlink -f $C920)" >/dev/null 2>&1 && red "카메라를 다른 프로그램이 쓰는 중"
  ls /sys/bus/usb/devices/*/product 2>/dev/null | while read -r f; do
    grep -q "C920" "$f" && { sp=$(cat "$(dirname "$f")/speed"); [ "$sp" -ge 480 ] && grn "C920 USB ${sp} Mb/s" || ylw "C920 USB ${sp} Mb/s — USB 2.0 이상 포트로"; }
  done
else
  ylw "C920 없음 (연결 후 다시) → 첫 연결 땐 ~/demo/ros_ws/p1_camera_check.sh"
fi
P="$(python3 -c "from car_drive.serial_link import find_port; print(find_port() or '')" 2>/dev/null)"
[ -n "$P" ] && grn "아두이노: $P" || ylw "아두이노 없음 (연결 후 다시)"
id -nG | grep -qw dialout && grn "dialout 그룹" || red "dialout 그룹 아님 → sudo usermod -aG dialout $USER 후 재로그인"
pgrep -xa 'rviz2|aruco_slam_node|c920_camera|car_state_node|wasd_drive' >/dev/null && ylw "이전 노드가 남아 있음: $(pgrep -xa 'rviz2|aruco_slam_node|c920_camera|car_state_node|wasd_drive' | tr '\n' ' ')"

echo "━━ 결과: ✗ ${BAD}개 · 현장에서 할 일 ${TODO}개 ━━"
cat <<'TXT'
챙길 것: 차 + 배터리(완충, 예비) · 아두이노 USB 케이블 · C920 + 마운트 · 노트북 충전기 · USB 허브(필요시)
        줄자(5 m+) · 각도기/스마트폰 기울기 앱 · 마스킹 테이프 · 바퀴 띄울 받침대 · 인쇄 마커(15 cm, 여분) · 실측 마커 CSV
현장 순서: field_check → p1_camera_check.sh → 장착 실측(mount.yaml) → build.sh → 바퀴 띄우고 drive_ros.sh
          → calib_steer.sh → drive_ros.sh --landmarks <csv> --rviz   (자세히: ~/demo/REPORT.md §3)
TXT
[ $BAD = 0 ]
