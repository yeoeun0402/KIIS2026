#!/usr/bin/env bash
# [demo] 작업공간 빌드 + 테스트:  ~/demo/ros_ws/build.sh [--test]
cd "$(dirname "$0")" || exit 1
source ./env.sh
colcon build --symlink-install --event-handlers console_cohesion+ 2>&1 | tail -15 || exit 1
if [ "${1:-}" = --test ]; then
  colcon test --event-handlers console_cohesion+ --pytest-args -q 2>&1 | grep -E 'passed|failed|error|Summary' | tail -10
  colcon test-result --verbose | tail -5
fi
