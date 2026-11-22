#!/usr/bin/env bash
# [demo] 조향 보정 주행 → steer_calib.yaml (POT → 유효 조향각 δ).  REPORT.md 체크리스트 D2
#
#   ~/demo/ros_ws/calib_steer.sh [drive_ros.sh 옵션...]      주행 + 분석 (기본 --max-speed 40 --render none)
#   ~/demo/ros_ws/calib_steer.sh --analyze <run 폴더>         이미 한 주행만 다시 분석
#
# 필요: mount.yaml 의 wheelbase 실측 · 마커가 보이는 3×3 m 이상 빈 바닥 (원호 하나 ~1.5 m)
WS="$HOME/demo/ros_ws"
source "$WS/env.sh"
analyze() {
  local RUN="$1"
  echo "━━ 분석: $RUN ━━"
  ros2 run car_state steer_calib "$RUN" 2>&1 | grep -v rosbag2_storage
  local rc=${PIPESTATUS[0]}
  [ "$rc" = 0 ] || { echo "보정 실패 (rc=$rc) — 위 메시지 확인. 같은 run 을 다시 분석: $0 --analyze $RUN"; return $rc; }
  echo "그림: $RUN/steer_calib/steer_calib.png"
  read -r -p "이 보정을 설치할까요 (src/car_bringup/config/steer_calib.yaml + 빌드)? [y/N] " ans
  if [ "$ans" = y ] || [ "$ans" = Y ]; then
    ros2 run car_state steer_calib "$RUN" --install 2>&1 | grep '설치'
    "$WS/build.sh" | tail -1
  fi
}
if [ "${1:-}" = --analyze ]; then analyze "$(readlink -f "$2")"; exit $?; fi
cat <<'TXT'
━━ 조향 보정 순서 (약 5분) ━━
 0. 마커가 잘 보이는 곳, 차 주위 1.5 m 이상 비움. 시작 직후 1~2초 정지 (SLAM 자리 잡기)
 1. 스윕 (차 정지): A 를 0.5초 간격으로 끝까지(POT 멈출 때까지) → D 로 반대 끝까지 → A 로 바퀴 똑바로
    · 펄스를 하나씩 띄엄띄엄 눌러야 펄스당 ADC·지연이 재짐 (꾹 누르지 말 것)
 2. 원호 (조향 단계마다 반복):  조향 맞추기(정지 상태에서 A/D n번) → W 를 차가 천천히 굴러갈 만큼 → 3~4초 → Space
    · 단계: 직진, 좌 2·4·6·8, 우 2·4·6·8 펄스 (끝까지 안 가도 됨, 넓게 퍼지게)
    · 좌 4 / 우 4 는 한 번 더: 2 펄스 더 꺾었다가 2 펄스 되돌려서 (반대쪽에서 다가가기 → 유격 측정)
    · 주행 중엔 조향 키 누르지 말 것 · 가장 느린 속도로 (빠르면 미끄러져 δ 가 작게 나옴)
 3. Q → 자동 분석 → 그림 확인 후 설치 여부 선택
TXT
ARGS=("$@")
printf '%s\n' "${ARGS[@]}" | grep -q -- '--max-speed' || ARGS+=(--max-speed 40)
printf '%s\n' "${ARGS[@]}" | grep -q -- '--render' || ARGS+=(--render none)
before="$(ls -1d "$HOME"/demo/sessions/2*/ 2>/dev/null | tail -1)"
"$WS/drive_ros.sh" "${ARGS[@]}"
RUN="$(ls -1d "$HOME"/demo/sessions/2*/ 2>/dev/null | tail -1)"
[ -n "$RUN" ] && [ "$RUN" != "$before" ] || { echo "새 주행 기록 없음"; exit 1; }
analyze "${RUN%/}"
