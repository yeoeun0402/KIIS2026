#!/usr/bin/env bash
# [demo] 지도 주행 → 실시간 고정 지도 반영까지 한 번에 (지도 만들기 → 점검 → 회전 맞춤 → 확인 → 반영 → 칼만 재생 검증)
#
#   ~/demo/update_map.sh                 가장 최근 주행 세션으로 (drive.sh --no-render 로 한 바퀴 돈 직후)
#   ~/demo/update_map.sh <세션 폴더>      예: ~/demo/sessions/20261122_160555
#   옵션  --align lane|markers|start   지도를 매트에 붙이는 기준 (기본 lane = 한 바퀴 주행 궤적을 2차선 중심에 맞춤,
#                                       markers = 마커가 2차선 바깥 선 위에 오도록 돌림, start = 출발선 자세만 믿음 — 예전 방식)
#         (2026-11-22 22시: 출발 순간 카메라 화면의 흰 차선과 비교하니 lane 이 가장 잘 맞음 — markers 는 3° 더 돌아감.
#          markers 는 점검 숫자로만 보고, 차선과 마커가 둘 다 크게 어긋나면 지도 주행을 다시)
#         --skip 14 [..]   바깥 선이 아닌 곳에 세운 마커 ID (markers 맞춤에서 뺌)
#         --size 0.265     마커 크기 (기본: track.yaml markers.sets.<use>.size_m)
#         --yes            반영할지 묻지 않음      --no-replay   칼만 재생 검증 생략 (1~2분 절약)
#         --no-open        그림 창 안 띄움
# 결과: <세션>/map_live/ (지도·그림) → 반영하면 track.yaml 의 live_map 폴더 (예: ~/demo/track/live_map_7x7, 이전 것은 *_backup_<시각>)
# Unity·MATLAB 쪽은 바꿀 것 없음: 실시간 위치는 대시보드(live_loc)가 이 고정 지도로 계산해서 보냄 → 다음 ./twin.sh 가 대시보드를 자동 재시작
# 회전 맞춘 지도(markers·lane)는 live_loc 이 출발 자세로 다시 돌리지 않고 그대로 씀 (차를 놓은 자세 오차가 그대로 보임 = 실제 위치)
set -u
DEMO="$HOME/demo"; TOOLS="$DEMO/track/tools"; V1="${TWIN_ROOT:-$HOME/version__1}"
red() { printf '\033[31m%s\033[0m\n' "$*"; }; grn() { printf '\033[32m%s\033[0m\n' "$*"; }; ylw() { printf '\033[33m%s\033[0m\n' "$*"; }
step() { printf '\n\033[1m━━ %s ━━\033[0m\n' "$*"; }
SESS=""; ALIGN=lane; SKIP=(); SIZE=""; YES=0; REPLAY=1; OPEN=1
while [ $# -gt 0 ]; do
  case "$1" in
    --align) ALIGN="$2"; shift ;;
    --skip) shift; while [ $# -gt 0 ] && [[ "$1" != --* ]]; do SKIP+=("$1"); shift; done; continue ;;
    --size) SIZE="$2"; shift ;;
    --yes) YES=1 ;;
    --no-replay) REPLAY=0 ;;
    --no-open) OPEN=0 ;;
    -h|--help) sed -n 2,15p "$0"; exit 0 ;;
    --*) red "모르는 옵션: $1"; exit 2 ;;
    *) SESS="$(readlink -f "$1")" ;;
  esac; shift
done
case "$ALIGN" in markers|lane|start) ;; *) red "--align 은 markers|lane|start"; exit 2 ;; esac
if [ -z "$SESS" ]; then   # 영상이 있는 가장 최근 세션
  for d in $(ls -1dr "$DEMO"/sessions/2* 2>/dev/null); do [ -s "$d/video.mkv" ] && { SESS="$d"; break; }; done
fi
[ -n "$SESS" ] && [ -s "$SESS/video.mkv" ] || { red "세션 영상이 없음: ${SESS:-(세션 없음)}/video.mkv"; exit 1; }
read -r MS_NAME MS_DICT MS_SIZE MS_MAP < <(cd "$TOOLS" && python3 -B -c 'import trackmap as tm; c=tm.marker_set(); print(c["name"], c["dictionary"], c["size_m"], c["live_map"])')
SIZE="${SIZE:-$MS_SIZE}"
OUT="$SESS/map_live"
echo "세션: $SESS"
echo "마커 세트: $MS_NAME ($MS_DICT, ${SIZE} m) · 맞춤: $ALIGN${SKIP[*]:+ (뺄 마커 ${SKIP[*]})} · 반영할 곳: $MS_MAP"

step "1/4 지도 만들기 (build_map, 3~5분)"
mkdir -p "$OUT"
if [ ! -f "$OUT/detections.pkl" ]; then   # 같은 영상의 마커 검출 결과가 이미 있으면 재사용 (검출이 제일 오래 걸림)
  for c in "$SESS"/map*/detections.pkl; do [ -f "$c" ] && { cp "$c" "$OUT/"; echo "마커 검출 재사용: $c"; break; }; done
fi
BA=start; [ "$ALIGN" = lane ] && BA=lane
if ! python3 -B "$TOOLS/build_map.py" --session "$SESS" --out "$OUT" --marker-size "$SIZE" --align "$BA" > "$OUT/build.log" 2>&1; then
  red "✗ build_map 실패 — $OUT/build.log"; tail -5 "$OUT/build.log"; exit 1
fi
grep -E "^\s*\[(OK|WARN|FAIL)\]" "$OUT/build.log" | sort -u

step "2/4 점검 (마커가 2차선 바깥 선 위에 있는지)"
rm -f "$OUT"/*.orig                                   # 지난번 회전 맞춤의 원본 사본 (지도를 새로 만들었으니 버림)
CK=(python3 -B "$TOOLS/map_check.py" "$OUT"); [ "$ALIGN" = markers ] && CK+=(--realign); [ ${#SKIP[@]} -gt 0 ] && CK+=(--skip "${SKIP[@]}")
"${CK[@]}" || { red "✗ 점검 실패"; exit 1; }
RMS="$(python3 -c "import json;print(json.load(open('$OUT/report.json'))['reproj_rms_px'])")"
NMK="$(python3 -c "import json;print(json.load(open('$OUT/report.json'))['n_markers'])")"
python3 -c "import sys; sys.exit(0 if float('$RMS') <= 4.0 else 1)" || ylw "! 재투영 ${RMS} px > 4 — 마커가 휘었거나 흔들렸을 수 있음. 그림을 보고 판단"
[ "$OPEN" = 1 ] && command -v xdg-open > /dev/null && { xdg-open "$OUT/markers_vs_line.png" > /dev/null 2>&1 & }
echo "그림: $OUT/markers_vs_line.png (마커·선) · $OUT/map_overview.png (도면 위, 회전 맞춤 전)"

step "3/4 고정 지도에 반영"
if [ "$YES" != 1 ]; then
  printf '반영할까요? 초록·주황 마커가 대부분이고 빨강이 없거나 이유를 알면 y [y/N] '; read -r ans
  [[ "$ans" =~ ^[yY] ]] || { ylw "반영 안 함. 지도는 $OUT 에 그대로 있음 (나중에: ~/demo/update_map.sh $SESS --yes)"; exit 0; }
fi
if [ -d "$MS_MAP" ]; then BK="${MS_MAP}_backup_$(date +%Y%m%d_%H%M%S)"; cp -a "$MS_MAP" "$BK"; echo "이전 지도 백업: $BK"; fi
mkdir -p "$MS_MAP"
cp "$OUT"/markers.csv "$OUT"/map.yaml "$OUT"/report.json "$MS_MAP"/
cp "$OUT"/markers_vs_line.png "$OUT"/map_overview.png "$MS_MAP"/ 2>/dev/null
{
  echo "실시간 localization 고정 지도 — 마커 세트 $MS_NAME ($MS_DICT). ~/demo/update_map.sh 가 $(date '+%Y-%m-%d %H:%M') 에 반영."
  echo "원본: $OUT (마커 ${NMK}개, 크기 ${SIZE} m, 재투영 ${RMS} px, 맞춤 $ALIGN${SKIP[*]:+, 뺀 마커 ${SKIP[*]}})"
  echo "이전 지도: ${BK:-없음}"
  echo "되돌리기: cp ${BK:-<백업>}/* $MS_MAP/"
} > "$MS_MAP/SOURCE.txt"
grn "✓ 반영: $MS_MAP"

if [ "$REPLAY" = 1 ]; then
  step "4/4 같은 영상으로 칼만 재생 검증 (1~2분)"
  NAME="replay_$(basename "$SESS")"
  ( cd "$V1" && MARKERS="$MS_NAME" python3 -B dashboard/live_loc.py --video "$SESS/video.mkv" --map "$MS_MAP" --log-dir results/loc_kf --log-name "$NAME" > /dev/null 2>&1 \
      && python3 -B analysis/loc_kf/plot_loc_kf.py "results/loc_kf/$NAME.csv" > /dev/null 2>&1 ) \
    && python3 - "$V1/results/loc_kf/${NAME}_summary.json" <<'EOF' || ylw "! 재생 검증 실패 (지도 반영은 됨)"
import json, sys
s = json.load(open(sys.argv[1]))
print(f"측정 {s['measurements']}/{s['rows']} 프레임 · 버림 {s['kf_rejected']} · NIS 평균 {s['nis_mean']:.2f} (3 근처, 1~2 도 괜찮음) · "
      f"흔들림 {s['jitter_meas_m'] * 100:.1f} → {s['jitter_kf_m'] * 100:.1f} cm")
print("그래프:", sys.argv[1].replace("_summary.json", ".png"))
EOF
fi
echo
grn "다음: cd ~/version__1 && ./twin.sh   (지도가 바뀌었으니 대시보드는 자동으로 다시 켜짐 · Unity 쪽은 바꿀 것 없음)"
