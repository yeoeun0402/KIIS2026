#!/usr/bin/env bash
# 쌍둥이 주행을 명령 하나로: 웹 대시보드 + Unity(Play) + 실차(drive.sh --twin) + MATLAB(twin_drive)
#
#   ./twin.sh                     실차 연동 (기본 --max-speed 40, 무게 15 kg)
#   ./twin.sh --mass 18           대시보드 시작 무게 [kg] (주행 중에도 웹 페이지에서 바꿈)
#   ./twin.sh --no-browser        브라우저를 자동으로 열지 않음 (주소: http://127.0.0.1:8765)
#   ./twin.sh --max-speed 60      drive.sh 옵션은 그대로 넘김 (--step, --port, --no-render ...)
#   ./twin.sh --no-car            실차 없이 Unity + MATLAB 만 (연결 시험)
#   ./twin.sh --markers old       이번만 마커 세트 바꾸기 (old = 5x5 17.5 cm · new = 7x7 27.5 cm). 늘 쓰는 값은
#                                 ~/demo/track/track.yaml 의 markers.use 한 줄. 대시보드가 다른 세트로 켜져 있으면 다시 켬
#
# 순서
#   0. 웹 대시보드 (dashboard/twin_dash.py, http://127.0.0.1:8765): 안 켜져 있으면 켜고 브라우저로 엶
#      위: 실차 카메라 | Unity 화면 · 아래: 무게 조절 | 지도 (시뮬 경로 빨강 · localization 경로 노랑)
#      주행 키는 Unity Game 창 또는 이 웹 페이지에서 W/A/S/D/Space/Q (같은 동작)
#   1. Unity: 이 프로젝트가 안 열려 있으면 연다 → Play 요청 파일을 만든다 (TwinAutoPlay.cs가 Play 누름)
#      → MATLAB 포트 55001 이 열릴 때까지 기다림 (처음 열 때 수 분 걸릴 수 있음)
#   2. 뒤에서 MATLAB 대기: 실차 키 포트 55004 가 열리면 twin_drive 실행 (출력은 이 터미널과 로그 파일)
#   3. 앞에서 drive.sh --twin: 안전 확인 [y/N] → 주행 (이 터미널 Space = 정지, Q = 종료)
#      Unity Game 창을 클릭하고 W/A/S/D/Space/Q 로 운전. Unity Q → MATLAB 저장 → 실차 종료
#      → drive.sh가 영상 · localization · 비교 결과(results/<run>/compare/)를 만듦
#   drive.sh가 끝나면 (Ctrl-C 포함) 남은 MATLAB을 정리한다. Unity와 대시보드는 켜 둔다 (다음 실행 때 그대로 씀).
#   4. 끝: ArUco 칼만 그래프 (results/loc_kf/loc_<시각>.png, 시뮬 결과 폴더에도 복사)
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$ROOT/KIIS2026_AutoCar"
DEMO="${DEMO:-$HOME/demo}"
MATLAB_BIN="${MATLAB_BIN:-$HOME/MATLAB/bin/matlab}"
REQUEST="$PROJECT/twin_play.request"
UNITY_WAIT=600           # Unity Play 기다리는 최대 시간 [s] (처음 열 때 셰이더·에셋 가져오기)

USE_CAR=1
MASS=""
BROWSER=1
DASH_PORT=8765
CAR_ARGS=()
HAS_SPEED=0
while [ $# -gt 0 ]; do
    case "$1" in
        --no-car) USE_CAR=0 ;;
        --mass) MASS="$2"; shift ;;
        --no-browser) BROWSER=0 ;;
        --markers) export MARKERS="$2"; shift ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        --max-speed) HAS_SPEED=1; CAR_ARGS+=("$1" "$2"); shift ;;
        *) CAR_ARGS+=("$1") ;;
    esac
    shift
done
[ "$HAS_SPEED" = 1 ] || CAR_ARGS+=(--max-speed 40)   # 모델 파라미터를 PWM 40 이하 주행으로 맞췄음

STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$ROOT/results/launch"
mkdir -p "$LOGDIR"
MLOG="$LOGDIR/${STAMP}_matlab.log"
ULOG="$LOGDIR/${STAMP}_unity.log"
DLOG="$LOGDIR/${STAMP}_dashboard.log"

say()  { printf '\033[1;36m[twin.sh]\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31m[twin.sh]\033[0m %s\n' "$*"; exit 1; }
listening() { ss -ltnH "sport = :$1" 2>/dev/null | grep -q .; }

# ── 0. 사전 점검 ───────────────────────────────────────────────────────────────
[ -x "$MATLAB_BIN" ] || fail "MATLAB이 없습니다: $MATLAB_BIN (MATLAB_BIN 으로 경로 지정)"
if [ "$USE_CAR" = 1 ]; then
    [ -x "$DEMO/drive.sh" ] || fail "drive.sh가 없습니다: $DEMO/drive.sh (DEMO 로 경로 지정)"
    listening 55004 && fail "포트 55004 가 이미 쓰이는 중 — 다른 drive.sh --twin 이 켜져 있습니다. 먼저 끄세요."
fi
for p in $(pgrep -x MATLAB); do   # 프로세스 이름이 MATLAB 인 것만 (셸 명령줄에 글자가 들어간 경우 제외)
    grep -qa twin_drive "/proc/$p/cmdline" 2>/dev/null && fail "twin_drive 를 실행 중인 MATLAB 이 이미 있습니다 (pid $p). 먼저 끄세요."
done

# ── 0. 마커 세트 (track.yaml markers.use, --markers / MARKERS 가 우선) ───────────────
MSET="$(cd "$DEMO/track/tools" && python3 -B -c 'import trackmap as tm; c = tm.marker_set(); print(c["name"], c["dictionary"], c["size_m"], c["live_map"])')" \
    || fail "마커 세트를 못 읽었습니다 (~/demo/track/track.yaml markers.use 또는 --markers 값 확인)"
read -r MSET_NAME MSET_DICT MSET_SIZE MSET_MAP <<< "$MSET"
export MARKERS="$MSET_NAME"          # 대시보드(live_loc)·drive.sh(영상 localization·build_map)가 같은 세트를 쓰게
say "마커 세트: $MSET_NAME ($MSET_DICT, ${MSET_SIZE} m) · 고정 지도 $([ -f "$MSET_MAP/markers.csv" ] && echo "$MSET_MAP" || echo "없음 → 같은 사전의 세션 지도 중 가장 좋은 것")"

# ── 0. 웹 대시보드 ─────────────────────────────────────────────────────────────
DASH_URL="http://127.0.0.1:$DASH_PORT"
if listening "$DASH_PORT"; then
    dash_set="$(curl -s "$DASH_URL/state" | python3 -c 'import json, sys; print(json.load(sys.stdin).get("markers") or "")' 2>/dev/null)"
    dash_pid="$(pgrep -f "dashboard/twin_dash.py" | head -1)"
    dash_t="$(stat -c %Y "/proc/$dash_pid" 2>/dev/null || echo 0)"                       # 대시보드가 켜진 시각
    code_t="$(stat -c %Y "$ROOT"/dashboard/*.py "$MSET_MAP"/markers.csv "$MSET_MAP"/map.yaml 2>/dev/null | sort -n | tail -1)"   # 코드·고정 지도 바뀐 시각 (live_loc 은 시작할 때만 지도를 읽음)
    if [ "$dash_set" != "$MSET_NAME" ] || [ "${code_t:-0}" -gt "$dash_t" ]; then   # 세트가 다르거나 (비어 있음 = 예전 버전) 코드가 바뀜 → 다시 켬
        say "대시보드: 마커 세트가 다르거나 코드·지도가 바뀜 (세트 ${dash_set:-?} → $MSET_NAME) → 다시 켬"
        pkill -TERM -f "dashboard/twin_dash.py" 2>/dev/null; pkill -TERM -f "dashboard/live_loc.py" 2>/dev/null
        for _ in $(seq 50); do listening "$DASH_PORT" || break; sleep 0.1; done
    fi
fi
if listening "$DASH_PORT"; then
    say "대시보드: 이미 켜져 있음 ($DASH_URL)"
    [ -n "$MASS" ] && curl -s -X POST -d "{\"mass\": $MASS}" "$DASH_URL/mass" >/dev/null
else
    say "대시보드: 켜는 중 ($DASH_URL · 로그 $DLOG)"
    setsid python3 -B "$ROOT/dashboard/twin_dash.py" --port "$DASH_PORT" --mass "${MASS:-15}" >"$DLOG" 2>&1 < /dev/null &
    for _ in $(seq 50); do listening "$DASH_PORT" && break; sleep 0.1; done
    listening "$DASH_PORT" || say "  대시보드가 안 켜졌습니다 (로그 $DLOG) — 주행은 대시보드 없이 계속"
fi
if [ "$BROWSER" = 1 ] && listening "$DASH_PORT"; then
    (xdg-open "$DASH_URL" >/dev/null 2>&1 &)
fi

# ── 1. Unity 열기 + Play ───────────────────────────────────────────────────────
if pgrep -if "Editor/Unity .*-projectpath $PROJECT" >/dev/null; then
    say "Unity: 이미 열려 있음"
else
    say "Unity: 여는 중 (로그 $ULOG)"
    setsid bash "$ROOT/run_unity.sh" >"$ULOG" 2>&1 < /dev/null &
fi
if listening 55001; then
    say "Unity: 이미 Play 중"
else
    : > "$REQUEST"
    say "Unity: Play 요청 → 포트 55001 기다리는 중 (최대 ${UNITY_WAIT}s)"
    say "  처음 한 번은 Unity 창을 클릭해야 새 스크립트가 컴파일됩니다. 오래 걸리면 Unity에서 직접 Play 를 눌러도 됩니다."
    t0=$SECONDS
    until listening 55001; do
        (( SECONDS - t0 > UNITY_WAIT )) && { rm -f "$REQUEST"; fail "Unity가 ${UNITY_WAIT}s 안에 Play 되지 않았습니다. Unity 창을 확인하세요."; }
        pgrep -if "Editor/Unity .*-projectpath $PROJECT" >/dev/null || { rm -f "$REQUEST"; fail "Unity가 꺼졌습니다 (로그 $ULOG)"; }
        sleep 1
    done
    say "Unity: Play 됨 ($((SECONDS - t0))s)"
fi

# ── 2. MATLAB (뒤에서) ─────────────────────────────────────────────────────────
# drive.sh가 터미널을 raw 모드로 쓰는 동안에도 줄이 깨지지 않게 줄 앞뒤에 \r 을 붙여 출력
TAG="twinsh_${STAMP}_$$"   # 이 실행의 MATLAB만 골라 끄기 위한 표시 (pkill -f)
MATLAB_CMD="cd('$ROOT/MATLAB'); setup_paths; tag = '$TAG'; res = twin_drive('useCar', $([ "$USE_CAR" = 1 ] && echo true || echo false)); disp(res.folder)"
run_matlab() {
    "$MATLAB_BIN" -batch "$MATLAB_CMD" 2>&1 | tee "$MLOG" | sed -u $'s/^/\r\033[K\033[1;35m[matlab]\033[0m /; s/$/\r/'
}

if [ "$USE_CAR" = 0 ]; then
    say "MATLAB: twin_drive (실차 없음) — Unity Game 창을 클릭하고 W/A/S/D/Space/Q (로그 $MLOG)"
    run_matlab
    exit "${PIPESTATUS[0]}"
fi

matlab_waiter() {
    until listening 55004; do sleep 0.5; done
    printf '\r\033[K\033[1;36m[twin.sh]\033[0m MATLAB 시작 (10~30초) — 연결되면 Unity Game 창 클릭 후 운전\r\n'
    run_matlab
}
# setsid: 터미널 Ctrl-C가 MATLAB에 가지 않게 (따로 세션). 실차를 Ctrl-C로 멈춰도 MATLAB은 연결 끊김을 보고 결과를 저장한다.
export MATLAB_BIN MATLAB_CMD MLOG
export -f listening run_matlab matlab_waiter
setsid bash -c matlab_waiter &
MPID=$!

wait_matlab() {   # Q 직후면 MATLAB이 결과를 저장하는 중일 수 있어 최대 $1 초 기다림
    [ -e "$MLOG" ] || return 0      # MATLAB이 아직 시작 안 함 (실차 포트가 안 열림: 안전 확인에서 N 등)
    for _ in $(seq "$1"); do kill -0 "$MPID" 2>/dev/null || return 0; sleep 1; done
}
cleanup() {
    trap - EXIT INT TERM
    wait_matlab 30
    if kill -0 "$MPID" 2>/dev/null; then
        pkill -TERM -f "$TAG" 2>/dev/null
        pkill -TERM -P "$MPID" 2>/dev/null
        kill -TERM "$MPID" 2>/dev/null
    fi
    rm -f "$REQUEST"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# ── 3. 실차 (앞에서) ───────────────────────────────────────────────────────────
say "실차: drive.sh --twin ${CAR_ARGS[*]}"
"$DEMO/drive.sh" --twin "${CAR_ARGS[@]}"
code=$?
wait_matlab 30

echo
[ -e "$MLOG" ] && say "MATLAB 로그: $MLOG"
res_dir="$(grep -o "$ROOT/results/[0-9_]*_twin" "$MLOG" 2>/dev/null | tail -1)"
[ -n "$res_dir" ] && say "시뮬 결과: $res_dir  (비교: $res_dir/compare/)"

# ── 4. ArUco 칼만 그래프 (이번 실행에서 live_loc 이 쓴 results/loc_kf/loc_<시각>.csv 마다) ─────────────
#   측정 vs 칼만 경로 · R/P 95% 타원 · σ_pos · NIS → 같은 이름 .png + _summary.json. 시뮬 결과 폴더에도 복사
n_kf=0
for csv in "$ROOT"/results/loc_kf/loc_*.csv; do
    [ -f "$csv" ] || continue
    b="$(basename "$csv" .csv)"; t="${b#loc_}"; t="${t:0:15}"
    [[ "$t" < "$STAMP" ]] && continue                       # 이번 twin.sh 보다 앞서 만든 기록은 건너뜀
    [ "$(wc -l < "$csv")" -gt 10 ] || continue             # 출발 맞춤 전에 끝난 빈 기록
    if python3 -B "$ROOT/analysis/loc_kf/plot_loc_kf.py" "$csv" > "${csv%.csv}_plot.log" 2>&1; then
        say "칼만 그래프: ${csv%.csv}.png  (요약 ${csv%.csv}_summary.json)"
        [ -n "$res_dir" ] && cp "${csv%.csv}.png" "${csv%.csv}_summary.json" "$res_dir/" 2>/dev/null
        n_kf=$((n_kf + 1))
    else
        say "  칼만 그래프 실패: $csv (로그 ${csv%.csv}_plot.log)"
    fi
done
[ "$n_kf" = 0 ] && say "칼만 그래프: 이번 실행의 기록 없음 (출발 위치에서 마커 2개 이상이 1초 넘게 보여야 시작)"
exit "$code"
