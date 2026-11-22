#!/usr/bin/env bash
# T10. ~/demo/drive.sh 를 가짜 보드(pty)로 끝까지 — 사전점검 → 주행(키 자동 입력) → 세션 → 결과 영상.
#   $1 = record | live    (모터 없음, 카메라는 실제 C920)
MODE="${1:-record}"
DEMO="$HOME/demo"; OUT="$DEMO/out/t10"; mkdir -p "$OUT"
TMP="$(mktemp -d -p "$OUT")"; A="$TMP/ttyBOARD"; B="$TMP/ttyLAPTOP"
socat pty,raw,echo=0,link="$A" pty,raw,echo=0,link="$B" & SOCAT=$!
trap 'kill $BOARD $SOCAT 2>/dev/null; rm -rf "$TMP"' EXIT
sleep 1
python3 -B "$DEMO/scripts/fake_board.py" "$A" "$OUT/fake_board_$MODE.json" & BOARD=$!
# 키: 빌드·연결 대기 12 s → W W → A → D → Space → 3 s → Q → 렌더 끝날 때까지 입력 유지
( sleep 12; printf w; sleep 0.6; printf w; sleep 0.6; printf a; sleep 0.4; printf d; sleep 0.6; printf ' '; sleep 3; printf q; sleep 240 ) |
  script -qfec "$DEMO/drive.sh $MODE --port $B --yes" "$OUT/console_$MODE.txt" > /dev/null
echo "drive.sh exit=$?"
kill $BOARD; wait $BOARD 2>/dev/null
sed 's/\x1b\[[0-9;]*m//g' "$OUT/console_$MODE.txt" | tr '\r' '\n' | grep -v '^\s*$' | grep -v '렌더 [0-9]' | tail -30
