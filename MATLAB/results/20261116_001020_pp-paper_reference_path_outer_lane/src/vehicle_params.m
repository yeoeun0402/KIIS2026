function p = vehicle_params()
% 차량·시뮬레이션 공용 파라미터 (pure_pursuit_control 등에서 사용)
% 값은 manual_drive.m과 같고, 축간거리만 Unity에서 측정한 값으로 바꿈

%% 차량
p.L         = 0.59;          % 축간거리 [m]  (Unity: Wheel_FL z - Wheel_RL z = -0.30 - (-0.89))
p.vMax      = 1.5;           % 최고 속도 [m/s]
p.vRevMax   = 0.5;           % 최고 후진 속도 [m/s]
p.accel     = 1.0;           % 가속도 [m/s^2]
p.brake     = 2.0;           % 감속도 [m/s^2]
p.deltaMax  = deg2rad(30);   % 최대 조향각 [rad]
p.steerRate = deg2rad(90);   % 조향 속도 [rad/s]
p.Rmin      = p.L / tan(p.deltaMax);   % 최소 회전반경 [m] (= 1.02 m)

%% 시뮬레이션
p.dt = 0.02;                 % 계산·전송 주기 [s] (50 Hz)

%% 맵 (Unity Plane: 10 m x Scale 1.7, 10 m x Scale 1.1, 중심 = 원점)
p.mapHalfX = 10 * 1.7 / 2;
p.mapHalfY = 10 * 1.1 / 2;

%% 출발 위치: Unity에서 car의 Inspector 값 (Position X, Position Z, Rotation Y)
p.startUnity = [7.20, -0.89, 0];

%% Unity 연결
p.unityIP   = "127.0.0.1";
p.unityPort = 55001;
end
