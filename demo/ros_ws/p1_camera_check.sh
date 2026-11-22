#!/usr/bin/env bash
# [demo] P1 통과 확인 — 카메라 노드를 띄우고 cam_probe 로 잰 뒤 자동 종료.
#   ~/demo/ros_ws/p1_camera_check.sh                 C920 1920x1080 (기본) + 1280x720
#   ~/demo/ros_ws/p1_camera_check.sh --builtin       노트북 내장 웹캠(/dev/video0)으로 파이프라인만 (컨트롤 안 건드림)
# 결과: ~/demo/out/p1/<시각>/ (probe_*.json · launch_*.log · ctrl_*.txt · summary.txt)
source "$HOME/demo/ros_ws/env.sh"
O="$HOME/demo/out/p1/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$O"
C920=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0
if [ "${1:-}" = --builtin ]; then DEV=/dev/video0; SIZES="1280x720"; CTRL=false
else DEV=$C920; SIZES="1920x1080 1280x720"; CTRL=true
  [ -e "$C920" ] || { echo "C920 없음 ($C920) — USB 연결 확인"; exit 1; }
fi
SEC=10
for S in $SIZES; do
  echo "━━ $DEV $S (컨트롤 고정 $CTRL) ━━"
  v4l2-ctl -d "$DEV" -l > "$O/ctrl_before_$S.txt"
  timeout -s INT $((SEC + 12)) ros2 launch car_bringup camera.launch.py device:="$DEV" size:="$S" controls:="$CTRL" > "$O/launch_$S.log" 2>&1 &
  LP=$!
  ros2 run aruco_slam_ros cam_probe --seconds $SEC --json "$O/probe_$S.json" | grep -E 'rate|dt_ms|gaps|kb_per|mb_per|format|decoded|latency|info_size|  K '
  # 노드가 살아 있는 동안 컨트롤 값 (C920 이면 고정값이어야 함)
  v4l2-ctl -d "$DEV" -l > "$O/ctrl_during_$S.txt"
  wait $LP 2>/dev/null
  grep -E 'focus_automatic_continuous|focus_absolute|auto_exposure|exposure_time_absolute' "$O/ctrl_during_$S.txt" | awk '{for(i=1;i<=NF;i++) if($i~/^value=/) printf "  v4l2 %-28s %s\n", $1, $i}'
  if grep -q Traceback "$O/launch_$S.log"; then echo "  [!] 노드 로그에 Traceback"; else echo "  노드 종료 깨끗함"; fi
  [ "$CTRL" = false ] && { diff -q "$O/ctrl_before_$S.txt" <(v4l2-ctl -d "$DEV" -l) >/dev/null && echo "  카메라 컨트롤 변화 없음 (건드리지 않음)"; }
  sleep 1
done
pgrep -xa c920_camera && echo "[!] 노드가 남아 있음" || echo "남은 노드 없음"
echo "결과: $O"
