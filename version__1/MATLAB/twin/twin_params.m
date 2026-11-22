function tp = twin_params(varargin)
% [twin] Unity 시뮬레이션 + 실차 동시 수동 주행(twin_drive.m) 파라미터
%   tp = twin_params()                       % 기본값
%   tp = twin_params('kv', 0.012)            % 이름-값으로 덮어쓰기
%
% 실차 명령 체계(~/demo/car_drive_exp_copy/src/car_drive/car_drive/drive.py, 펌웨어 car_controller.ino)를 그대로 흉내 냄:
%   W/S : PWM 속도 명령 ±step (한계 ±maxSpeed)   → 시뮬: delay 뒤 목표 속도 = kv × (|PWM| - pwmDead), 1차 지연 tauV
%   A/D : 조향 모터 펄스 120 ms (같은 키가 또 오면 120 ms 연장) → 시뮬: 모터가 도는 동안 조향각이 steerRate로 변함
%         손을 떼도 조향각은 그대로 (실차 조향은 자동으로 가운데로 돌아오지 않음)
%   Space : 속도 0 + 조향 모터 정지                Q : 종료
% step, maxSpeed는 실차 노드가 연결 직후 HELLO로 알려 주면 그 값으로 바꿈 (drive.sh --max-speed --step과 항상 같게)
%
% 속도·조향 값 (2026-11-22 맞춤, analysis/fit_twin_multi.py): 마커 크기를 자로 잰 0.175 m 로 고친 지도로 다시 만든
%   실차 궤적 20261121_212500 + 214854 를 6 s 구간마다 실차 상태에서 다시 시작해 예측 오차 최소화.
%   6 s 예측 위치 오차 RMS: 212500 0.104 → 0.081 m, 214854 0.171 → 0.079 m (맞추기 전 = 아래 첫 맞춤 값)
%   첫 맞춤 값 (180841 한 주행, 마커 0.20 m 지도 — 궤적이 약 20% 크게 나온 지도라 속도가 과대): kv 0.0080, pwmDead 0,
%   tauV 0.42, delay 0.30, steerRate 30.5°/s, deltaMax 20.5°, delta0 −1.6°
% 첫 맞춤 기록 (참고):
%   세션 ~/demo/sessions/20261121_180841 (Unity 키로 약 8 m, 출발선 → 아래 왼쪽 커브, PWM 최대 40)
%   같은 명령을 모델에 다시 넣었을 때 실차 궤적과의 위치 오차: 맞추기 전 RMS 2.90 m → 후 0.14 m (움직인 구간)
%   맞추기 전 가정값: kv 0.015, pwmDead 0, tauV 0.3, delay 0, steerRate 60°/s, deltaMax 30°, delta0 0
%   한 번의 주행으로 맞춘 값이라 더 빠른 속도·다른 바닥·배터리 상태에서는 다를 수 있음 → 주행을 더 하면 다시 맞출 것:
%     python3 ~/version__1/analysis/fit_twin_params.py --real ~/demo/sessions/<세션>

% 무게 (2026-11-21): 위 값은 장비(노트북·카메라)를 다 실은 실차로 맞춘 값 → 그때 무게 = massRef (15 kg 가정).
%   mass를 바꾸면 twin_mass.m이 tauV·pwmDead를 다시 계산 (kv·조향은 그대로). 웹 대시보드 슬라이더로 주행 중에도 바꿈.

%% 실차 명령 (HELLO가 오면 덮어씀)
tp.step     = 10;                 % W/S 한 번에 바뀌는 PWM
tp.maxSpeed = 60;                 % |PWM| 한계 (drive.sh 기본 60)

%% 속도 모델 (실차 기록으로 맞춘 값)
tp.kv      = 0.01054;             % 정상 속도 / PWM [m/s per PWM]  (PWM 40 → 0.279 m/s, 30 → 0.173, 20 → 0.068)
tp.pwmDead = 13.55;                % 이 PWM 이하에서는 바퀴가 안 돎 (PWM 10 → 정지, 20 → 아주 느림)
tp.tauV    = 0.589;               % 속도 1차 지연 시정수 [s]
tp.delay   = 0.0;                 % PWM 명령이 바뀐 뒤 바퀴가 반응하기까지 [s] (출발이 늦는 것은 pwmDead·tauV 로 설명됨)

%% 무게 → 속도 모델 (twin_mass.m):  m v' = a·PWM − b·v − crr·g·(m − massRef)
%   a, b는 massRef에서 맞춘 kv·tauV로 정함 (b = massRef/tauVRef, a = kv·b) → massRef에서는 맞춘 모델과 똑같음
%   무게가 늘면: 가속이 느려짐 (tauV ∝ m) + 굴림 저항만큼 같은 PWM에서 느려짐 (pwmDead ↑, kg당 약 crr·g/a PWM)
tp.mass       = 15;               % 지금 차 무게 [kg] (대시보드에서 바꿈)
tp.massRef    = 15;               % 위 kv·tauV·pwmDead를 맞출 때의 실차 무게 [kg] (재서 넣을 것: 지금은 가정값)
tp.crr        = 0.03;             % 굴림 저항 계수 (매트 위 고무 바퀴 대략값, 측정 안 함)
tp.g          = 9.81;
tp.tauVRef    = tp.tauV;          % massRef에서 맞춘 값 (twin_mass가 여기서 다시 계산)
tp.pwmDeadRef = tp.pwmDead;

%% 조향 모델 (실차 기록으로 맞춘 값)
tp.pulse     = 0.12;              % 조향 펄스 길이 [s] (펌웨어 STEER_PULSE_MS = 120, 측정값 아님·코드값)
tp.steerRate = deg2rad(26.9);     % 조향 모터가 돌 때 조향각 변화 속도 [rad/s] (키 한 번 0.12 s → 3.2°)
tp.deltaMax  = deg2rad(13.0);     % 최대 조향각 [rad] (delta0 과 합쳐 실제 −9.5° ~ +16.5°)
tp.delta0    = deg2rad(3.5);      % 조향 모터 위치와 실제 조향각의 차이 [rad] (실제 = delta + delta0). 출발은 바퀴가 곧다고 보고 delta = −delta0 에서 시작

%% 루엔버그 옵저버 (twin_observer.m): 실시간 localization (x, y, θ) 으로 시뮬 상태를 실차 쪽으로 당김
%   추정 상태: 위치 x, y · 방향 yaw · 조향각 delta · 속도 배율 vs (v_실제 = v_모델 · (1 + vs))
%   직진 근처 선형 오차 모델 (차체 좌표: 앞뒤 e_lon, 옆 e_lat, 방향 e_θ, 조향 e_δ, 속도 배율 e_s):
%     옆:    ė_lat = v e_θ,   ė_θ = (v/L) e_δ,   ė_δ = 0        측정 e_lat, e_θ
%     앞뒤:  ė_lon = v e_s,   ė_s = 0                          측정 e_lon
%   게인 (극배치): e_lat → s + kLat · e_θ, e_δ → s² + kTh s + kDel·v/L (ωn, ζ) · e_lon, e_s → s² + kLon s + kS·v
%   v 가 작으면 (|v| < vMin) 조향·속도 배율은 보정하지 않음 (관측 불가). 측정이 없을 때는 모델만으로 진행.
tp.obs.enabled   = true;
tp.obs.tauPos    = 1.0;           % 옆 위치 오차 시정수 [s]          → kLat = 1/tauPos
tp.obs.wnHead    = 1.2;           % 방향·조향 오차 고유진동수 [rad/s]  → kTh = 2ζωn, kDel = ωn² L / v
tp.obs.zetaHead  = 0.9;
tp.obs.wnLon     = 0.8;           % 앞뒤·속도 배율 오차 [rad/s]        → kLon = 2ζωn, kS = ωn² / v
tp.obs.zetaLon   = 0.9;
tp.obs.vMin      = 0.08;          % 이보다 느리면 조향·속도 배율 추정 멈춤 [m/s]
tp.obs.gate      = 1.0;           % 위치 차이가 이보다 크면 그 측정은 버림 [m] (잘못된 localization)
tp.obs.gateDeg   = 35;            % 방향 차이 한계 [deg]
tp.obs.minMarkers = 2;            % 마커 이만큼 이상 본 측정만 씀 (1개면 위치가 모호)
tp.obs.maxAge    = 0.5;           % 이보다 오래된 측정은 버림 [s] (지연은 v·age 로 보정)
tp.obs.vsMax     = 0.8;           % 속도 배율 한계 |vs|
tp.obs.deltaMaxObs = deg2rad(35); % 옵저버가 조향각을 이 범위까지 움직일 수 있음 (모델 deltaMax 보다 넓게)

%% 칼만 필터 (EKF, twin_ekf_predict.m · twin_ekf_update.m) — 기본 추정기 (tp.estimator = "ekf")
%   상태 [x y yaw delta vs], 측정 = localization [x y yaw] + 측정마다 공분산 (live_loc.py: 마커 배치에 따라 방향별로 다름)
%   루엔버그 옵저버와 달리 측정을 얼마나 믿을지 매번 계산 → 마커가 멀리 한 줄로만 보이는 곳의 틀린 측정에 끌려가지 않음
tp.estimator   = "ekf";           % "ekf" | "luenberger" | "none"
tp.ekf.P0      = [0.03^2, 0.03^2, deg2rad(2)^2, deg2rad(8)^2, 0.15^2];   % 처음 불확실성 (출발 자세는 잘 알고, 조향·속도 배율은 모름)
tp.ekf.qPos    = 0.01^2;          % 위치 과정 잡음 [m²/s] (서 있을 때)
tp.ekf.qPosPerSpeed = 0.03^2;     % + 속도 1 m/s 당 [m²/s per m/s] (미끄러짐·모델 오차)
tp.ekf.qYaw    = deg2rad(1.5)^2;  % 방향 [rad²/s]
tp.ekf.qDelta  = deg2rad(1.0)^2;  % 조향각 [rad²/s] (조향이 저절로 조금씩 바뀜)
tp.ekf.qKey    = deg2rad(2.0)^2;  % 조향 키 하나마다 조향각 분산 추가 [rad²] (키 한 번에 꺾이는 양 오차)
tp.ekf.qVs     = 0.02^2;          % 속도 배율 [1/s] (배터리·바닥에 따라 천천히 바뀜)
tp.ekf.rFloor  = [0.02^2, 0.02^2, deg2rad(0.5)^2];   % 측정 공분산에 더하는 바닥값
tp.ekf.gateChi2 = 16.27;          % 마할라노비스² 한계 (자유도 3, 99.9%) — 넘으면 그 측정은 버림
tp.ekf.vMin    = 0.05;            % 이보다 느리면 조향각·속도 배율 갱신 안 함 [m/s]
tp.ekf.maxAge  = 0.5;             % 측정 지연 한계 [s]
tp.ekf.vsMax   = 0.8;
tp.ekf.deltaMaxEst = deg2rad(35);

%% 시간
tp.T     = 0.05;                  % 메인 루프 주기 [s]: 키 처리 · Unity 전송 · 기록 · 실차 HB
tp.dtSub = 0.01;                  % 모델 적분 간격 [s]
tp.tMax  = 900;                   % 최대 주행 시간 [s]

%% 출발 자세 = 실차 출발 자세 (~/demo/track/track.yaml start.base_link: 트랙 좌표 1.265, 6.659, -90°)
tp.startTrack = [1.265, 6.659, deg2rad(-90)];

%% 통신
tp.carIP   = "127.0.0.1";
tp.carPort = 55004;               % drive.sh --twin 이 여는 포트 (wasd_drive key_tcp_port)
tp.dashPort = 55005;              % 웹 대시보드 (dashboard/twin_dash.py): 시뮬 위치 보내기 · 무게·키 받기 (없으면 건너뜀)

for k = 1:2:numel(varargin)
    name = varargin{k};
    if name == "obsEnabled", tp.obs.enabled = logical(varargin{k + 1}); continue; end
    if name == "estimator", tp.estimator = string(varargin{k + 1}); continue; end
    if ~isfield(tp, name)
        error("twin_params: 알 수 없는 파라미터 '%s'", name);
    end
    tp.(name) = varargin{k + 1};
    if name == "tauV", tp.tauVRef = tp.tauV; end          % 덮어쓴 값 = massRef에서의 값
    if name == "pwmDead", tp.pwmDeadRef = tp.pwmDead; end
end
tp = twin_mass(tp, tp.mass);
end
