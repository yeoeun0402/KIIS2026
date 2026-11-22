# 주행 세션 — 세션마다 폴더 하나

```
sessions/<YYYYmmdd_HHMMSS>/
  video.mkv · video_frames.csv · drive_log.csv · encoder.csv · serial_raw.log · session.json   주행 원본 (drive.sh / run.sh)
  live/        drive.sh live 모드의 실시간 SLAM 기록
  render/      clean.mp4 · localization.mp4 (+ .json)                                        결과 영상
  map/         detections.pkl · markers.csv · map.yaml · trajectory.csv · report.json · map_overview.png   (track/tools/build_map.py)
  bag/ slam/ export/ bringup.log                                                             ROS 경로(ros_ws/drive_ros.sh)로 주행했을 때
```

- 기록 위치는 `CAR_DRIVE_LOG=~/demo/sessions` (drive.sh · car_drive_exp_copy/run.sh · ros_ws/env.sh 기본값).
- 지도 만들기: `python3 -B ~/demo/track/tools/build_map.py --session ~/demo/sessions/<세션>` → `<세션>/map/`
- 결과 영상: `python3 -B ~/demo/live/render_clean.py --session ~/demo/sessions/<세션>` → `<세션>/render/clean.mp4`

| 세션 | 내용 |
|---|---|
| 20261120_094812 | ROS 경로 조향 보정 주행 (steer_calib) |
| 20261120_224608 ~ 233235 | 필드 로컬라이제이션 주행 8회 (마커 크기 0.20 반영 전 설정으로 처리됨) |
| 20261121_051209 | 지도 만들기 주행 (영상 끝부분 디스크 부족으로 잘림, video_frames.csv 비어 있음 → `--t-move 11.4`) |
