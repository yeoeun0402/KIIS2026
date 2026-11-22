#!/usr/bin/env bash
# [demo] 수동 주행 + 로컬라이제이션 → 결과 영상까지 한 번에.
#
#   ~/demo/drive.sh [record|live] [옵션]
#f
#   record (기본, 검증된 경로)  주행 노드가 C920 을 녹화 → 주행 끝나면 SLAM + 결과 영상
#   live                        live_slam.py 가 C920 을 맡아 주행 중 실시간 2D 화면(+--3d) 표시·기록,
#                               주행 노드는 카메라 끔(-p camera:=none) → 끝나면 결과 영상
#
#   옵션  --size 1920x1080|1280x720   (record 기본 1920x1080, live 기본 1280x720)
#         --max-speed N (기본 60)  --step N (기본 10)   --port /dev/ttyACM0 (기본 자동)v
#         --landmarks track.csv      실측 마커(id,x_m,y_m,z_m, 오른손 좌표) → 트랙 좌표로 그림
#         --3d  (live: Pangolin 3D 창)   --ros (live: /car/cam_pose 발행)   --no-render (영상 생략)
#         --style clean|full  clean(기본)=3D 궤적+카메라 축만, 글자 없음 · full=타임라인·지도·수치 대시보드
#         --yes (안전 확인 질문 생략 — 시험용)
#         --preview  마커 배치 확인 창(track/tools/marker_preview.py)을 같이 띄움: 수동 주행·녹화는 그대로,
#                    창에 보이는 마커 ID·거리·개수 (2개 이상 초록). 키 입력은 이 터미널에 (창이 뜨면 터미널을 다시 클릭)
#         --twin [--twin-port 55004]   [twin] Unity 시뮬레이션과 동시 주행: MATLAB(~/version__1 twin_drive.m)이 보내는 키로도 주행
#                                      (Unity Game 창 W/A/S/D/Space/Q = 이 터미널 키와 같음, 터미널 Space/Q 도 계속 동작)
#                                      주행 뒤: 결과 영상(기본 full) → localization 숫자·이미지(map/, build_map.py)
#                                      → ~/version__1/analysis/compare_twin.py 로 Unity 궤적과 횡오차 비교 (TWIN_ROOT 로 위치 변경)
#
# 원본 ~/car_drive_exp · ~/aruco-slam 은 쓰지 않는다:
#   주행 = ~/demo/car_drive_exp_copy (Ctrl+C 영상 수정본) · SLAM = ~/demo/aruco-slam-fixed (EKF 회전 수정본)
#   세션마다 한 폴더: ~/demo/sessions/<세션>/
#     video.mkv · drive_log.csv · encoder.csv · serial_raw.log · session.json   (주행 원본)
#     live/   (live 모드 실시간 기록)   render/clean.mp4 · localization.mp4   map/  (track/tools/build_map.py)
# 2026-11-21: dialout 에 추가됐지만 아직 재로그인 전이면(현재 셸에 그룹이 안 붙음) sg 로 그룹을 붙여 자기 자신을 다시 실행
#   → 재로그인 없이도 /dev/ttyACM0 권한 오류(Errno 13) 안 남. 재로그인 후엔 이 줄은 아무것도 안 함
if ! id -nG | grep -qw dialout && getent group dialout | cut -d: -f4 | tr , '\n' | grep -qx "$USER" && [ -z "${_DIALOUT_REEXEC:-}" ]; then
  export _DIALOUT_REEXEC=1
  exec sg dialout -c "$(printf '%q ' "$(readlink -f "$0")" "$@")"
fi
DEMO="$HOME/demo"; COPY="$DEMO/car_drive_exp_copy"; SESSIONS="$DEMO/sessions"; LIVE_PY="$DEMO/live/live_slam.py"; RENDER_PY="$DEMO/live/render_localization.py"
CAM_ID="/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"
MODE=record; SIZE=""; MAXSPD=60; STEP=10; PORT=""; LMK=""; YES=0; RENDER=1; STYLE=""; LIVE_ARGS=(); TWIN=0; TWIN_PORT=55004; PREVIEW=0
PREVIEW_JPEG="/dev/shm/drive_preview/cam.jpg"
case "${1:-}" in record|live) MODE="$1"; shift ;; esac
while [ $# -gt 0 ]; do
  case "$1" in
    --size) SIZE="$2"; shift ;;
    --max-speed) MAXSPD="$2"; shift ;;
    --step) STEP="$2"; shift ;;
    --port) PORT="$2"; shift ;;
    --landmarks) LMK="$(readlink -f "$2")"; shift ;;
    --3d) LIVE_ARGS+=(--3d) ;;
    --ros) LIVE_ARGS+=(--ros) ;;
    --no-render) RENDER=0 ;;
    --style) STYLE="$2"; shift ;;
    --yes) YES=1 ;;
    --twin) TWIN=1 ;;
    --preview) PREVIEW=1 ;;
    --twin-port) TWIN_PORT="$2"; shift ;;
    -h|--help) sed -n 2,25p "$0"; exit 0 ;;
    *) echo "모르는 옵션: $1"; exit 2 ;;
  esac; shift
done
[ -z "$SIZE" ] && { [ "$MODE" = live ] && SIZE=1280x720 || SIZE=1920x1080; }
[ -z "$STYLE" ] && { [ "$TWIN" = 1 ] && STYLE=full || STYLE=clean; }   # twin: 마커·지도·명령 타임라인이 보이는 영상
red() { printf '\033[31m%s\033[0m\n' "$*"; }; grn() { printf '\033[32m%s\033[0m\n' "$*"; }; ylw() { printf '\033[33m%s\033[0m\n' "$*"; }
FAIL=0

echo "━━ 사전 점검 ($MODE · $SIZE · 최고 PWM $MAXSPD · step $STEP) ━━"
# 1) 카메라
if [ -e "$CAM_ID" ]; then grn "✓ C920: $(readlink -f "$CAM_ID")"; else red "✗ C920 없음 ($CAM_ID)"; FAIL=1; fi
if [ -e "$CAM_ID" ] && command -v fuser >/dev/null && fuser "$(readlink -f "$CAM_ID")" >/dev/null 2>&1; then
  red "✗ 카메라를 다른 프로그램이 쓰는 중: $(fuser "$(readlink -f "$CAM_ID")" 2>&1)"; FAIL=1; fi
# 2) 카메라 설정 (USB 재연결하면 초기화됨 → 매번 다시 넣는다)
if [ -e "$CAM_ID" ]; then
  # ----수정한 부분 --------
  v4l2-ctl -d "$CAM_ID" -c focus_automatic_continuous=0 >/dev/null 2>&1
  v4l2-ctl -d "$CAM_ID" -c auto_exposure=1 >/dev/null 2>&1
  sleep 0.2
  v4l2-ctl -d "$CAM_ID" -c focus_absolute=19 >/dev/null 2>&1
  v4l2-ctl -d "$CAM_ID" -c exposure_time_absolute=156 >/dev/null 2>&1
  # ---------------------------
  
  CTL="$(v4l2-ctl -d "$CAM_ID" -l 2>/dev/null)"
  F=$(echo "$CTL" | awk '/focus_absolute/{for(i=1;i<=NF;i++) if($i~/^value=/){sub("value=","",$i);print $i}}')
  AF=$(echo "$CTL" | awk '/focus_automatic_continuous/{for(i=1;i<=NF;i++) if($i~/^value=/){sub("value=","",$i);print $i}}')
  E=$(echo "$CTL" | awk '/exposure_time_absolute/{for(i=1;i<=NF;i++) if($i~/^value=/){sub("value=","",$i);print $i}}')
  if [ "$F" = 19 ] && [ "$AF" = 0 ]; then grn "✓ 초점 고정 19 (캘리브레이션과 같게) · 노출 수동 $E"; else red "✗ 초점 설정 실패 (focus=$F auto=$AF)"; FAIL=1; fi
fi
# 3) 아두이노
if [ -n "$PORT" ]; then
  [ -e "$PORT" ] && grn "✓ 포트(지정): $PORT" || { red "✗ 포트 없음: $PORT"; FAIL=1; }
else
  P="$(python3 -c "import sys; sys.path.insert(0, '$COPY/src/car_drive'); from car_drive.serial_link import find_port; print(find_port() or '')")"
  [ -n "$P" ] && grn "✓ 아두이노: $P" || { red "✗ 아두이노 포트 없음 — USB 확인 (또는 --port)"; FAIL=1; }
fi
id -nG | grep -qw dialout || ylw "! $USER 가 dialout 그룹이 아님 — 포트 권한 오류 시 sudo usermod -aG dialout $USER 후 재로그인"
# 4) 디스크
AVAIL=$(df --output=avail -BM "$HOME" | tail -1 | tr -dc 0-9)
case "$SIZE" in 1920x1080) MBPM=$([ "$MODE" = live ] && echo 420 || echo 690) ;; *) MBPM=$([ "$MODE" = live ] && echo 200 || echo 240) ;; esac
MIN=$((AVAIL / MBPM))
if [ "$AVAIL" -lt 1024 ]; then red "✗ 디스크 여유 ${AVAIL} MB — 1 GB 미만"; FAIL=1
elif [ "$MIN" -lt 10 ]; then ylw "! 디스크 여유 $((AVAIL / 1024)) GB → 이 설정으로 약 ${MIN}분 녹화 가능"
else grn "✓ 디스크 여유 $((AVAIL / 1024)) GB → 약 ${MIN}분 녹화 가능"; fi
# 5) 실측 마커
[ -n "$LMK" ] && { [ -f "$LMK" ] && grn "✓ 실측 마커: $LMK ($(($(wc -l < "$LMK") - 1))개)" || { red "✗ 실측 마커 파일 없음: $LMK"; FAIL=1; }; }
[ "$FAIL" = 1 ] && { red "사전 점검 실패 — 중단"; exit 1; }

if [ "$YES" != 1 ]; then
  echo
  ylw "안전 확인: 첫 시험은 바퀴를 띄웠나요? 주행 공간이 비었고 차 옆에 잡을 사람이 있나요?"
  ylw "          출발 위치에서 마커 3개 이상 보이게 두고, 시작하면 3초 정지 후 출발. 종료는 반드시 Q."
  [ "$TWIN" = 1 ] && ylw "  [twin] 차는 Unity 키(MATLAB)를 따라 움직임 — 운전자는 Unity 를 보므로 차 옆 사람이 반드시 지켜볼 것. 이 터미널 Space = 정지"
  read -r -p "진행할까요? [y/N] " ans; [ "$ans" = y ] || [ "$ans" = Y ] || { echo "취소"; exit 0; }
fi

export PYTHONPATH="$HOME/aruco_slam_deps:$HOME/aruco-slam/thirdparty/pangolin${PYTHONPATH:+:$PYTHONPATH}"
ROS_P=(-p max_speed:="$MAXSPD" -p step:="$STEP")
[ "$TWIN" = 1 ] && ROS_P+=(-p key_tcp_port:="$TWIN_PORT" -p live_jpeg:="${TWIN_LIVE_JPEG:-/dev/shm/twin_live/cam.jpg}")   # 대시보드 카메라 화면
[ -n "$PORT" ] && ROS_P+=(-p port:="$PORT")
if [ "$PREVIEW" = 1 ]; then
  [ "$MODE" = live ] && { red "--preview 는 record 모드에서만 (live 는 이미 화면이 있음)"; exit 2; }
  if [ "$TWIN" = 1 ]; then PREVIEW_JPEG="${TWIN_LIVE_JPEG:-/dev/shm/twin_live/cam.jpg}"   # twin 은 이미 대시보드용 JPEG 를 씀 → 같이 읽음
  else ROS_P+=(-p live_jpeg:="$PREVIEW_JPEG"); fi
fi
export CAR_DRIVE_LOG="$SESSIONS"; mkdir -p "$SESSIONS"
BEFORE="$(ls -1d "$SESSIONS"/2* 2>/dev/null | tail -1)"
LIVE_PID=""; LIVE_DIR=""

if [ "$MODE" = live ]; then
  LIVE_DIR="$SESSIONS/_live_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LIVE_DIR"
  # 주행 터미널의 Ctrl+C 가 SLAM 을 먼저 죽이지 않게 새 세션으로 (끝낼 땐 아래에서 SIGINT)
  setsid python3 -B "$LIVE_PY" --out "$LIVE_DIR" --size "$SIZE" "${LIVE_ARGS[@]}" > "$LIVE_DIR/live.log" 2>&1 &
  LIVE_PID=$!
  for _ in $(seq 1 100); do [ -f "$LIVE_DIR/READY" ] && break; kill -0 $LIVE_PID 2>/dev/null || break; sleep 0.1; done
  if [ ! -f "$LIVE_DIR/READY" ]; then red "✗ 실시간 SLAM 시작 실패:"; tail -5 "$LIVE_DIR/live.log"; kill -INT $LIVE_PID 2>/dev/null; exit 1; fi
  grn "✓ 실시간 SLAM 동작 중 (로그 $LIVE_DIR/live.log) — 2D 창은 보기만, 키 입력은 이 터미널에"
  ROS_P+=(-p camera:=none)
else
  ROS_P+=(-p video_size:="$SIZE")
fi

PREVIEW_PID=""
if [ "$PREVIEW" = 1 ]; then
  setsid python3 -B "$DEMO/track/tools/marker_preview.py" --jpeg "$PREVIEW_JPEG" > "$SESSIONS/.preview.log" 2>&1 &
  PREVIEW_PID=$!
  grn "✓ 마커 확인 창 (카메라가 켜지면 나타남 · 창이 뜨면 이 터미널을 다시 클릭해야 W/A/S/D 가 들어감)"
fi
echo "━━ 주행 시작 (W/S 속도 · A/D 조향 · Space 정지 · Q 종료) ━━"
[ "$TWIN" = 1 ] && echo "   [twin] 다음: Unity(~/version__1) Play → MATLAB: cd ~/version__1/MATLAB; setup_paths; twin_drive → Unity Game 창에서 조작"
"$COPY/run.sh" "${ROS_P[@]}"
RC=$?

if [ -n "$PREVIEW_PID" ]; then
  kill -TERM $PREVIEW_PID 2>/dev/null; wait $PREVIEW_PID 2>/dev/null
  grep -a "본 마커" "$SESSIONS/.preview.log" | tail -1
fi
if [ -n "$LIVE_PID" ]; then
  kill -INT $LIVE_PID 2>/dev/null; wait $LIVE_PID 2>/dev/null
  tail -1 "$LIVE_DIR/live.log"
fi
SESS="$(ls -1d "$SESSIONS"/2* 2>/dev/null | tail -1)"
if [ -z "$SESS" ] || [ "$SESS" = "$BEFORE" ]; then red "새 세션 폴더가 없음 (주행 노드 rc=$RC)${LIVE_DIR:+ · 실시간 기록: $LIVE_DIR}"; exit 1; fi
# 실시간 기록을 세션 폴더 안으로
[ -n "$LIVE_DIR" ] && mv "$LIVE_DIR" "$SESS/live" && LIVE_DIR="$SESS/live"
echo "━━ 세션: $SESS ━━"
twin_post() {   # [twin] localization 숫자·이미지(map/) + Unity 궤적과 비교
  [ "$TWIN" = 1 ] || return 0
  echo "━━ [twin] localization 숫자·이미지 (build_map.py → $SESS/map/) ━━"
  if python3 -B "$DEMO/track/tools/build_map.py" --session "$SESS" --out "$SESS/map" > "$SESS/map_build.log" 2>&1; then
    grn "✓ 궤적 숫자: $SESS/map/trajectory.csv · 이미지: map/map_overview.png · 요약: map/report.json"
  else
    red "✗ build_map 실패 — $SESS/map_build.log"; tail -3 "$SESS/map_build.log"
  fi
  local TR="${TWIN_ROOT:-$HOME/version__1}"
  if [ -f "$TR/analysis/compare_twin.py" ]; then
    echo "━━ [twin] Unity 궤적과 비교 ($TR/analysis/compare_twin.py) ━━"
    python3 -B "$TR/analysis/compare_twin.py" --real "$SESS"
  else
    ylw "! 비교 스크립트 없음: $TR/analysis/compare_twin.py (TWIN_ROOT 확인)"
  fi
}
[ "$RENDER" = 1 ] || { echo "영상 생략 (--no-render). 나중에: python3 -B $RENDER_PY --session $SESS ${LIVE_DIR:+--live $LIVE_DIR}"; twin_post; exit 0; }

R_ARGS=(--session "$SESS"); [ -n "$LIVE_DIR" ] && R_ARGS+=(--live "$LIVE_DIR"); [ -n "$LMK" ] && R_ARGS+=(--landmarks "$LMK")
echo "━━ 결과 영상 만들기 ($STYLE · 주행 시간의 약 2배 걸림) ━━"
if [ "$STYLE" = full ]; then PY="$RENDER_PY"; SUF=localization; else PY="$DEMO/live/render_clean.py"; SUF=clean; fi
python3 -B "$PY" "${R_ARGS[@]}" && grn "▶ 재생: mpv $SESS/render/$SUF.mp4"
twin_post
