#!/usr/bin/env bash
# [demo] P3 확인: drive_ros.sh 를 가짜 보드(pty) + 녹화 영상 소스 + 자동 키 입력으로 끝까지.
#   ~/demo/ros_ws/p3_drive_check.sh q|ctrlc     (모터 없음, 카메라 없음)
MODE="${1:-q}"
OUT="$HOME/demo/out/p3"; mkdir -p "$OUT"
TMP="$(mktemp -d -p "$OUT")"; A="$TMP/ttyBOARD"; B="$TMP/ttyLAPTOP"
socat pty,raw,echo=0,link="$A" pty,raw,echo=0,link="$B" & SOCAT=$!
trap 'kill $BOARD $SOCAT 2>/dev/null; rm -rf "$TMP"' EXIT
sleep 1
python3 -B "$HOME/demo/scripts/fake_board.py" "$A" "$OUT/fake_board_$MODE.json" & BOARD=$!
END=q; [ "$MODE" = ctrlc ] && END=$'\x03'
# 준비(빌드·SLAM·보드 리셋) 15 s → W W → A → D → Space → 3 s → 종료 키 → 정리 끝날 때까지 입력 유지
( sleep 15; printf w; sleep 1; printf w; sleep 1; printf a; sleep 0.5; printf d; sleep 1; printf ' '; sleep 3; printf '%s' "$END"; sleep 240 ) |
  script -qfec "$HOME/demo/ros_ws/drive_ros.sh --source video --video $HOME/car_calibration/corridor_map/videos/corridor_mapping_01.mkv --rate ${RATE:-0.5} --port $B --landmarks $HOME/car_calibration/corridor_map/aruco_landmarks.csv --yes" "$OUT/console_$MODE.txt" > /dev/null
echo "drive_ros.sh exit=$?"
kill $BOARD; wait $BOARD 2>/dev/null
sed 's/\x1b\[[0-9;]*m//g' "$OUT/console_$MODE.txt" | tr '\r' '\n' | grep -v '^\s*$' | tail -32
