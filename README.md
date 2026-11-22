# KIIS2026 — twin.sh 실행 묶음 (branch `twin-setup`)

`./twin.sh`(웹 대시보드 + Unity + 실차 + MATLAB 쌍둥이 주행)에 필요한 코드를 두 폴더로 담았습니다.

| 폴더 | 원래 위치 | 내용 |
|---|---|---|
| `version__1/` | `~/version__1` | `twin.sh`, Unity 프로젝트(`KIIS2026_AutoCar`), MATLAB(`twin_drive`), 대시보드, 분석 스크립트 |
| `demo/` | `~/demo` | `drive.sh`(실차 주행), ROS 2 작업공간(`ros_ws`), ArUco SLAM, 트랙 지도(`track/`), `setup_new_pc.sh` |

## 설치

스크립트 안에 `~/demo`, `~/version__1` 경로가 들어 있으므로 두 폴더를 홈 아래에 두어야 합니다 (복사 또는 심볼릭 링크).

```bash
git clone -b twin-setup https://github.com/yeoeun0402/KIIS2026.git ~/KIIS2026_twin
ln -s ~/KIIS2026_twin/version__1 ~/version__1
ln -s ~/KIIS2026_twin/demo ~/demo
~/demo/setup_new_pc.sh        # ROS 2 · 파이썬 패키지 설치, ros_ws 빌드
```

그 밖에 필요한 것:

- Unity 에디터 `6000.6.0f1` (`~/Unity/Hub/Editor/6000.6.0f1/`, 다른 곳이면 `UNITY_EDITOR`로 지정). `Library/`는 처음 열 때 다시 만들어집니다.
- MATLAB (`~/MATLAB/bin/matlab`, 다른 곳이면 `MATLAB_BIN`으로 지정)

## 실행

```bash
cd ~/version__1 && ./twin.sh            # 실차 연동
cd ~/version__1 && ./twin.sh --no-car   # 실차 없이 Unity + MATLAB만
```

옵션은 `./twin.sh --help`를 보세요.

## 올리지 않은 것

- 주행 결과(`results/`, `MATLAB/results/`), 영상(`*.mkv`, `*.mp4`), YOLO 캡처(`yolo/2026*/`)
- `demo/sessions/`의 주행 원본 중 git에 아직 없던 것 (고정 지도 `demo/track/live_map*`가 있으므로 `twin.sh` 실행에는 필요 없음)
- Unity가 자동으로 만드는 `Library/`, `Temp/`, `Logs/`, 그리고 복구 장면 `Assets/_Recovery/`
- ROS 빌드 결과(`build/`, `install/`, `log/`): `setup_new_pc.sh`가 다시 빌드합니다.
