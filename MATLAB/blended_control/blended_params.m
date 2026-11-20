function prm = blended_params(varargin)
% PP_PID 튜닝 파라미터 (논문 PP_PI: Chen et al., ITSC 2018 + D 항. 기본값은 여기서 바꾸면 됨)
%   prm = blended_params()                                % 기본값
%   prm = blended_params('Kp', 0.3, 'Ki', 0.01, 'v', 1.0)  % 이름-값으로 덮어쓰기
% 바꾸는 방법 세 가지
%   ① 이 파일의 기본값을 고침
%   ② 실행할 때 덮어씀:  blended_discrete('input', 'auto', 'Kp', 0.3, 'Lf', 1.2)
%   ③ 실행 중에 바꿈:    blended_discrete 그림 왼쪽 아래 슬라이더·입력칸 (범위는 아래 prm.range)
%                        바꾼 값은 다음 스텝부터 적용되고, timeseries.csv의 p_* 열에 스텝마다 기록됨
%
% PP_PID: δ = Lowpass( K_pp δ_pp + δ_p + δ_i + δ_d ),  δ_p = K_p e_la,  δ_i = K_i Σ e,  δ_d = K_d (e[k] - e[k-1]) / T_s
%         K_pp + K_p + K_i + K_d = 1  (논문의 합 = 1 규칙을 D까지 넓힘).  K_d = 0이면 논문의 PP_PI

%% 주기 (실행 중 변경 불가)
prm.Ts = 0.1;          % 한 스텝 [s]

%% 속도
prm.v = 0.8;           % 자동 버전 목표 속도 [m/s] (키보드 버전은 ↑/↓ 키로 가속·감속, 최고 1.5 m/s)

%% Pure Pursuit (Eq.3)
prm.Lf  = 0.8;         % 선행점: 차를 경로에 투영한 점에서 경로를 따라 Lf [m] 앞 (L_ad = 차와 선행점 사이 거리)

%% PID (P: Eq.4–6, I: Eq.7, D: K_d (e[k] - e[k-1]) / T_s)
prm.Kp  = 0.01;        % P 게인 [rad/m]: δ_p = K_p e_la
prm.Ki  = 0.006;       % I 게인 [rad/m]: δ_i = K_i Σ e  (스텝마다 더함, T_s를 곱하지 않음 = 논문 Eq.7)
prm.Kd  = 0.001;       % D 게인 [rad s/m]: δ_d = K_d (e[k] - e[k-1]) / T_s  (0이면 PP_PI)
                       % 기본값: 합 = 1 규칙을 지키면서 한 바퀴 궤적 오차가 작은 값 (README 4장)
prm.Kpp = NaN;         % PP 게인: sumToOne이면 1 - Kp - Ki - Kd로 자동 계산 (직접 주면 sumToOne = false일 때만 씀)
prm.sumToOne = true;   % 논문: "Kpp, Kp, Ki's sum is equal to 1"을 D까지: K_pp + K_p + K_i + K_d = 1 (false면 Kpp를 따로 정함)
prm.a   = 0.59;        % e_la = e + (a + L_ad) sin Δψ 의 a [m]. e를 재는 뒤 차축에서 앞바퀴 축까지 = 축간거리 L

%% Low-pass Filter (Eq.8–9)
prm.ell  = 3;          % 필터 창 길이 ℓ (현재 1개 + 이전 출력 ℓ-1개). 1이면 필터 없음
prm.wcur = 0.5;        % 이번 값 가중치 w_current. 이전 출력 하나의 가중치 = (1 - w_current) / (ℓ - 1)

%% 비교용 PID (PID/PID_control.m과 같은 조향 PID: e = -e_lat, 이산 시간)
prm.pidKp = 3.0;       % [rad/m]
prm.pidKi = 0.8;       % [rad/(m s)]
prm.pidKd = 2.6;       % [rad s/m]
prm.lpfAll = false;    % true: PP, PID 비교 제어기에도 같은 Low-pass Filter (논문은 PP_PI에만)

%% 실행 중 슬라이더 범위 [최소 최대]
prm.range = struct('v', [0.1 1.5], 'Lf', [0.3 2.5], 'a', [0 1.5], 'wcur', [0.05 1], ...
                   'Kp', [0 1], 'Ki', [0 0.05], 'Kd', [0 0.05], 'Kpp', [0 1], 'ell', [1 10]);

for k = 1:2:numel(varargin)
    name = char(varargin{k});
    if ~isfield(prm, name)
        error("blended_params: 알 수 없는 파라미터 '%s' (있는 이름: %s)", name, ...
            strjoin(setdiff(fieldnames(prm), {'range'}), ', '));
    end
    prm.(name) = varargin{k + 1};
end
prm.sumToOne = logical(prm.sumToOne);
prm.lpfAll   = logical(prm.lpfAll);
prm.ell      = round(prm.ell);
if prm.sumToOne
    if ~isnan(prm.Kpp) && abs(prm.Kpp - (1 - prm.Kp - prm.Ki - prm.Kd)) > 1e-12
        warning("blended_params: sumToOne이 true라 Kpp = 1 - Kp - Ki - Kd = %.4g로 바꿉니다 (준 값 %.4g).", ...
            1 - prm.Kp - prm.Ki - prm.Kd, prm.Kpp);
    end
    prm.Kpp = 1 - prm.Kp - prm.Ki - prm.Kd;
elseif isnan(prm.Kpp)
    prm.Kpp = 1;
end
if prm.Kpp < 0, error("blended_params: Kpp = 1 - Kp - Ki - Kd가 음수입니다 (Kp %.4g, Ki %.4g, Kd %.4g).", prm.Kp, prm.Ki, prm.Kd); end
if prm.ell < 1 || prm.ell > 21, error("blended_params: ell은 1~21이어야 합니다."); end
if prm.wcur <= 0 || prm.wcur > 1, error("blended_params: wcur는 0 초과 1 이하여야 합니다."); end
end
