#!/usr/bin/env bash
# T6. 주행 노드 스모크 테스트 — 실제 아두이노 대신 가짜 보드(pty), 카메라는 실제 C920.
#   원본 car_drive_exp 는 건드리지 않는다: demo/car_drive_exp_copy 에서 빌드·실행, 기록은 demo/out/drive_logs
#   $1 = 종료 방식: q (정상) | ctrlc (터미널 Ctrl+C 재현)   $2 = 해상도 (기본 1920x1080)
# (set -u 금지: ROS setup.bash 가 미정의 변수를 쓴다)
END="${1:-q}"; SIZE="${2:-1920x1080}"
DEMO="$HOME/demo"; COPY="$DEMO/car_drive_exp_copy"; OUT="$DEMO/out/drive_logs"; mkdir -p "$OUT"
TMP="$(mktemp -d -p "${SCRATCH:-/tmp}")"; A="$TMP/ttyBOARD"; B="$TMP/ttyLAPTOP"
socat pty,raw,echo=0,link="$A" pty,raw,echo=0,link="$B" & SOCAT=$!
trap 'kill $BOARD $SOCAT 2>/dev/null' EXIT; sleep 1
python3 -B "$DEMO/scripts/fake_board.py" "$A" "$OUT/fake_board_${END}_${SIZE}.json" & BOARD=$!

for _d in jazzy humble; do [ -f "/opt/ros/$_d/setup.bash" ] && { source "/opt/ros/$_d/setup.bash"; break; }; done; source "$COPY/install/setup.bash"
export PYTHONNOUSERSITE=1 CAR_DRIVE_LOG="$OUT" CAR_DRIVE_ROOT="$COPY"

# 7초 시점: 녹화 중 카메라를 다른 프로세스가 열 수 있는지(실시간 SLAM 과 동시 사용 가능성) 확인
( sleep 7; python3 -B - <<'PY' > "$OUT/camera_share_${END}.txt" 2>&1
import sys; sys.path.insert(0, __import__("os").path.expanduser("~/aruco_slam_deps"))
import cv2
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)  # 인덱스로 (경로 문자열은 V4L2 백엔드가 거부)
ok, fr = cap.read() if cap.isOpened() else (False, None)
print("second_open_isOpened", cap.isOpened(), "read_ok", ok)
PY
) &

if [ "$END" = q ]; then LAST='q'; else LAST='\003'; fi
# 키: 6 s 대기 → W W (V20,V40) → A D → Space(V0,F) → 4 s → 종료
( sleep 6; printf w; sleep 0.5; printf w; sleep 0.5; printf a; sleep 0.3; printf d; sleep 0.5; printf ' '; sleep 4; printf "$LAST"; sleep 8 ) |
  script -qfec "ros2 run car_drive wasd_drive --ros-args -p port:=$B -p video_size:=$SIZE" "$OUT/console_${END}_${SIZE}.txt" > /dev/null
echo "node exit=$?"
kill $BOARD; wait $BOARD 2>/dev/null; kill $SOCAT 2>/dev/null; rm -rf "$TMP"
ls -td "$OUT"/2* | head -1
