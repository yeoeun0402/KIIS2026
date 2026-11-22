#!/usr/bin/env bash
# KIIS2026_AutoCar를 Unity 에디터로 엽니다 (Linux).
#   bash run_unity.sh
#
# 한글 입력기(IBus)를 끄고 실행합니다. IBus가 켜진 채(XMODIFIERS=@im=ibus)로 Unity를 열면
# Game 창에서 숫자키(1~5 화면 전환), Space/방향키(test1 한 스텝), P/Q 같은 키가 Unity에 전달되지 않을 수 있습니다.
# Unity 안에서는 한글을 입력할 일이 없으므로 이 Unity 창에만 입력기를 끕니다 (다른 프로그램은 그대로).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION="$(sed -n 's/^m_EditorVersion: //p' "$ROOT/KIIS2026_AutoCar/ProjectSettings/ProjectVersion.txt")"
EDITOR="${UNITY_EDITOR:-$HOME/Unity/Hub/Editor/$VERSION/Editor/Unity}"

if [ ! -x "$EDITOR" ]; then
    echo "Unity $VERSION 에디터가 없습니다: $EDITOR (Unity Hub에서 설치하거나 UNITY_EDITOR로 경로 지정)"
    exit 1
fi
if pgrep -if "Editor/Unity .*-projectpath $ROOT/KIIS2026_AutoCar" > /dev/null; then
    echo "이 프로젝트가 이미 Unity에서 열려 있습니다. 먼저 Unity를 닫으세요."
    exit 1
fi

exec env XMODIFIERS=@im=none GTK_IM_MODULE=xim QT_IM_MODULE=xim \
    "$EDITOR" -projectPath "$ROOT/KIIS2026_AutoCar" "$@"
