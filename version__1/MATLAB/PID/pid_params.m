function pid = pid_params(varargin)
% PID 궤적 추종 제어기의 파라미터 (조향 PID + 속도 PID, 순수 PID: 곡률 피드포워드 없음)
%   pid = pid_params()                         % 기본값
%   pid = pid_params('Kp', 2.5, 'v_tar', 1.0)  % 이름-값으로 덮어쓰기
%
% 기본 게인: 경로 근처에서 선형화한 모델의 극배치로 정한 값 (README 10장)
%   조향: d²e_lat/dt² ≈ (v²/L) delta  →  s³ + b Kd s² + b Kp s + b Ki = 0,  b = v²/L (v = 0.8, L = 0.59 으로 설계. 2026-11-21 L을 0.56으로 바꿨지만 게인은 다시 계산하지 않음)
%         극: wn = 1.5 rad/s, zeta = 0.8, 실근 0.4  →  Kp 2.96, Ki 0.83, Kd 2.58
%   속도: d²e_s/dt² = a_r - a          →  s³ + Kd_s s² + Kp_s s + Ki_s = 0
%         극: wn = 1.5 rad/s, zeta = 0.9, 실근 0.3  →  Kp_s 3.06, Ki_s 0.68, Kd_s 3.0
%   조향 게인의 효과는 v²에 따라 달라지므로 v_tar를 바꾸면 다시 맞춰야 함

pid.mode = "pid";

%% 레퍼런스 궤적 (make_reference_trajectory.m)
pid.v_tar  = 0.8;            % 목표(최대) 속도 [m/s]
pid.v_mode = "curve";        % "curve": 곡선에서 sqrt(9.79(e_max+f_max)/|kappa|)로 감속 (pure pursuit와 같은 규칙)
                             % "const": 항상 v_tar
pid.e_max  = 0;              % 도로 경사 (논문 식 4)
pid.f_max  = 0.05;           % 마찰 계수 (논문 식 4)
pid.r_up   = 2.5;            % 앞쪽 곡률을 읽는 거리 r_min(v_tar)의 상한 [m] (pure pursuit와 같음)
pid.a_ref  = 0.5;            % 레퍼런스 가속도 [m/s^2] (차 한계 1.0의 절반: 뒤처지면 따라잡을 여유)
pid.b_ref  = 1.0;            % 레퍼런스 감속도 [m/s^2] (차 한계 2.0의 절반)

%% 조향 PID (이산 시간): delta[t] = Kp e[t] + Ki Σ e[j] Ts + Kd (e[t] - e[t-1]) / Ts,  e[t] = -e_lat[t]
pid.Kp      = 3.0;           % [rad/m]
pid.Ki      = 0.8;           % [rad/(m s)]
pid.Kd      = 2.6;           % [rad s/m]
pid.anti_windup = false;     % false: 위 식 그대로 / true: 조향 포화 동안 적분 멈춤 + |Σ e Ts| <= int_max
pid.int_max = 0.5;           % anti_windup = true일 때 |Σ e Ts| 상한 [m s]

%% 속도 PID: a_cmd = Kp_s e_s + Ki_s ∫e_s dt + Kd_s e_v
pid.Kp_s      = 3.0;         % [1/s^2]
pid.Ki_s      = 0.7;         % [1/s^3]
pid.Kd_s      = 3.0;         % [1/s]
pid.int_s_max = 1.0;         % |∫e_s dt| 상한 [m s]
pid.allow_reverse = false;   % false면 v_cmd >= 0 (후진 안 함)

for k = 1:2:numel(varargin)
    name = varargin{k};
    if ~isfield(pid, name)
        error("pid_params: 알 수 없는 파라미터 '%s'", name);
    end
    pid.(name) = varargin{k + 1};
end
pid.v_mode = string(pid.v_mode);
if ~ismember(pid.v_mode, ["curve", "const"])
    error("v_mode는 'curve', 'const' 중 하나입니다.");
end
end
