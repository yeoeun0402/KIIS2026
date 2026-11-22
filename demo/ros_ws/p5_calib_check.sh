#!/usr/bin/env bash
# [demo] P5 준비 확인: calib_steer.sh 를 가짜 보드(조향 POT 시뮬) + 녹화 영상 + 자동 키로 끝까지 (모터·카메라 없음)
#   ~/demo/ros_ws/p5_calib_check.sh
# 확인: 스윕(POT 범위·펄스당 ADC·L 방향)이 실제 시리얼→ROS→bag→분석 경로로 재지는지, /car/state·/car/board_state 기록
# (원호는 영상 자세가 가짜 조향과 무관하므로 보정은 '구간 부족'으로 끝나는 게 정상)
OUT="$HOME/demo/out/p5prep/rehearsal"; mkdir -p "$OUT"
TMP="$(mktemp -d -p "$OUT")"; A="$TMP/ttyBOARD"; B="$TMP/ttyLAPTOP"
socat pty,raw,echo=0,link="$A" pty,raw,echo=0,link="$B" & SOCAT=$!
trap 'kill $BOARD $SOCAT 2>/dev/null; rm -rf "$TMP"' EXIT
printf 'x: 0.0\ny: 0.0\nz: 0.0\npitch_down_deg: 0.0\nyaw_deg: 0.0\nroll_deg: 0.0\nwheelbase: 0.26   # 시험용\n' > "$OUT/mount_test.yaml"
sleep 1
python3 -B "$HOME/demo/scripts/fake_board.py" "$A" "$OUT/fake_board.json" & BOARD=$!
keys() {
  sleep 15
  for _ in $(seq 10); do printf a; sleep 0.6; done      # 좌 끝까지
  for _ in $(seq 20); do printf d; sleep 0.6; done      # 우 끝까지
  for _ in $(seq 8); do printf a; sleep 0.6; done       # 가운데로
  printf w; sleep 0.3; printf w; sleep 3; printf ' '; sleep 1
  printf q; sleep 200
}
keys | script -qfec "$HOME/demo/ros_ws/calib_steer.sh --source video --video $HOME/car_calibration/corridor_map/videos/corridor_mapping_01.mkv --rate 0.5 --port $B --yes --mount $OUT/mount_test.yaml" "$OUT/console.txt" > /dev/null
echo "calib_steer exit=$?"
kill $BOARD; wait $BOARD 2>/dev/null
sed 's/\x1b\[[0-9;]*m//g' "$OUT/console.txt" | tr '\r' '\n' | grep -v '^\s*$' | grep -vE '^\s*(속도|W 전진|\(주행)' | tail -30
