function pp = pp_params(varargin)
% Pure pursuit 선행점 모드와 파라미터 (docs/pure_pursuit_design.md 2.6, 2.8절)
%   pp = pp_params()                          % 기본값
%   pp = pp_params('mode', 'fixed', 'Lf', 1.0) % 이름-값으로 덮어쓰기
%
% 모드: 선행점을 고르는 방법만 다르고, 원 -> 조향각 계산은 모두 같음 (pursuit_steer.m)
%   "fixed" : 경로를 따라 Lf 앞
%   "speed" : Lf = clip(t_la * v, Lmin, Lmax)              (논문의 Proportional to Speed)
%   "paper" : Step 1 (Dubins로 p_d) + Step 2 (p_l로 옮기기) (논문의 제안 방법)

pp.mode   = "paper";
pp.v_tar  = 0.8;             % 목표(최대) 속도 [m/s]
pp.v_mode = "curve";         % "curve": 곡선에서 sqrt(9.79(e_max+f_max)/|kappa|)로 감속 (논문 4.1절)
                             % "const": 항상 v_tar. f_max=0.05면 우리 경로에서 감속은 v_tar > 0.93일 때만 걸림

%% fixed
pp.Lf = 1.8;                 % 선행 거리 [m]

%% speed
pp.t_la = 1.2;               % Lf = t_la * v [s]
pp.Lmin = 0.6;               % [m]
pp.Lmax = 2.5;               % [m]

%% paper (논문 값을 차량 크기 비율 0.21로 줄인 시작값)
pp.e_max     = 0;            % 도로 경사 (식 4)
pp.f_max     = 0.05;         % 마찰 계수 (식 4). 작을수록 r_min이 커져 선행점이 멀어짐
pp.r_up      = 2.5;          % r_min 상한 [m] (논문 12 m)
pp.d_search  = 2.5;          % p_w에서 후보까지 최대 거리 [m] (논문 12 m)
pp.cand_step = 0.03;         % 후보 경로점 간격 [m] (Dubins 계산량 조절)
pp.alpha_max = 1.0;          % 식 9 [m] (논문 5 m)
pp.beta_max  = 0.95;         % 식 10 [1/m] (논문 0.2)
pp.step1_on  = true;         % false면 p_d = 경로를 따라 r_min 앞
pp.step2_on  = true;         % false면 p_l = p_d
pp.phi_max   = deg2rad(80);  % Step 2의 tan|phi|가 너무 커지지 않게 (논문: 90도 미만 가정)

for k = 1:2:numel(varargin)
    name = varargin{k};
    if ~isfield(pp, name)
        error("pp_params: 알 수 없는 파라미터 '%s'", name);
    end
    pp.(name) = varargin{k + 1};
end
pp.mode = string(pp.mode);
if ~ismember(pp.mode, ["fixed", "speed", "paper"])
    error("mode는 'fixed', 'speed', 'paper' 중 하나입니다.");
end
pp.v_mode = string(pp.v_mode);
if ~ismember(pp.v_mode, ["curve", "const"])
    error("v_mode는 'curve', 'const' 중 하나입니다.");
end
end
