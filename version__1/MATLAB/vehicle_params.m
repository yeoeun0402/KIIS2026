function p = vehicle_params()
% 차량·시뮬레이션 공용 파라미터 (pure_pursuit_control 등에서 사용)
% 값은 manual_drive.m과 같고, 축간거리만 Unity에서 측정한 값으로 바꿈

%% 차량
p.L         = 0.56;          % 축간거리 [m]  2026-11-21 실차값(demo mount.yaml wheelbase)으로 변경. 이전 0.59 = Unity 차 모델 Wheel_FL z - Wheel_RL z (모델 원래 0.596 m → 장면에서 0.9396배로 줄여 0.56 m에 맞춤)
p.vMax      = 1.5;           % 최고 속도 [m/s]
p.vRevMax   = 0.5;           % 최고 후진 속도 [m/s]
p.accel     = 1.0;           % 가속도 [m/s^2]
p.brake     = 2.0;           % 감속도 [m/s^2]
p.deltaMax  = deg2rad(30);   % 최대 조향각 [rad]
p.steerRate = deg2rad(90);   % 조향 속도 [rad/s]
p.Rmin      = p.L / tan(p.deltaMax);   % 최소 회전반경 [m] (= 0.97 m)

%% 시뮬레이션
p.dt = 0.02;                 % 계산·전송 주기 [s] (50 Hz)

%% 맵 (Unity Plane: 10 m x Scale 1.6, 10 m x Scale 1.1 = 16 m x 11 m, 중심 = 원점)
%   2026-11-21: 17 m x 11 m(Scale 1.7)에서 16:11로 변경. 예전 경로는 reference_path_17x11/ 에 보관
p.mapHalfX = 10 * 1.6 / 2;
p.mapHalfY = 10 * 1.1 / 2;

%% 출발 위치: Unity에서 car의 Inspector 값 (Position X, Position Z, Rotation Y)
p.startUnity = [6.78, -0.89, 0];   % 16 x 11 맵 (17 x 11에서는 7.20 → x를 16/17배)

%% Unity 연결
p.unityIP   = "127.0.0.1";
p.unityPort = 55001;
end
