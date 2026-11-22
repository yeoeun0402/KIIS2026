#!/usr/bin/env bash
# 실차 메가에 firmware/car_controller 업로드 (arduino-cli, 처음이면 ~/.local/bin에 설치).
#   ./flash.sh                  포트 자동
#   ./flash.sh /dev/ttyACM0
#   ./flash.sh --compile-only   컴파일만 (보드 없이 확인)
# 주행 노드(./run.sh)가 포트를 잡고 있으면 업로드가 실패한다 → 먼저 Q로 끌 것.
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
SKETCH="$ROOT/firmware/car_controller"
FQBN="arduino:avr:mega"          # Arduino Mega 2560
export PATH="$HOME/.local/bin:$PATH"

if ! command -v arduino-cli >/dev/null; then
  echo "[flash] arduino-cli 설치 → ~/.local/bin"
  mkdir -p "$HOME/.local/bin"
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$HOME/.local/bin" sh
fi
if ! arduino-cli core list 2>/dev/null | grep -q '^arduino:avr'; then
  echo "[flash] AVR 보드 패키지 설치 (처음 한 번)"
  arduino-cli core update-index
  arduino-cli core install arduino:avr
fi

echo "[flash] 컴파일: $SKETCH"
arduino-cli compile --fqbn "$FQBN" "$SKETCH"
[ "${1:-}" = "--compile-only" ] && { echo "[flash] 컴파일만 완료"; exit 0; }

PORT="${1:-$(python3 -c "import sys; sys.path.insert(0, '$ROOT/src/car_drive'); from car_drive.serial_link import find_port; print(find_port() or '')")}"
[ -n "$PORT" ] || { echo "[flash] 아두이노 포트를 못 찾음 — USB 확인 또는 ./flash.sh /dev/ttyACM0"; exit 1; }
if command -v fuser >/dev/null && fuser "$PORT" >/dev/null 2>&1; then
  echo "[flash] $PORT 를 다른 프로그램이 쓰는 중 (주행 노드?) — 끄고 다시"; exit 1
fi

echo "[flash] 업로드: $PORT"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"
echo "[flash] 완료 — 이제 ./run.sh"
