function [dCmd, mem, d] = blended_steer(car, ref, p, g, mem, T)
% PP_PID 조향 한 스텝
%   기준 논문: Chen et al., "Optimization of Pure Pursuit Controller based on PID Controller and
%   Low-pass Filter", ITSC 2018 (PP_PI). 여기에 D 항(K_d (e[k] - e[k-1]) / T_s)을 더함
%
%   δ(t) = Lowpass( K_pp · δ_pp + δ_p + δ_i + δ_d )                        (논문 Eq.10 + D)
%     δ_pp = atan( 2 L sin α / L_ad )                                      (Eq.3, Pure Pursuit)
%     δ_p  = K_p · e_la,   e_la = e + (a + L_ad) sin(Δψ)                   (Eq.4–6, P: 선행 오차)
%     δ_i  = K_i · Σ e                                                      (Eq.7,   I: 횡오차의 합)
%     δ_d  = K_d · (e[k] - e[k-1]) / T_s                                    (D: 횡오차의 변화율, 첫 스텝은 0)
%     Lowpass: lpf_window.m (Eq.8–9, 창 길이 ℓ, 이번 값 가중치 w_current)
%     K_pp + K_p + K_i + K_d = 1          (논문의 "합 = 1" 규칙을 D까지 넓힘, blended_params에서 맞춤)
%     K_d = 0이면 논문의 PP_PI와 같음
%
%   오차의 부호 (목표 - 현재, 조향각 왼쪽 + 와 맞춤):
%     e   = 0 - e_lat          횡오차 [m]  (e_lat: 경로 왼쪽 +. 차가 오른쪽에 있으면 e > 0 → 왼쪽으로 꺾음)
%     Δψ  = ψ_path - θ          방향 오차 [rad]
%     e, Δψ는 차의 기준점(뒤 차축 중앙)에서 잼. a = 그 점에서 앞쪽으로 더하는 거리 (기본 L: 앞바퀴 축)
%     L_ad = 차와 PP 선행점 사이 거리 (Eq.3과 같은 값)
%
%   car : 차 상태 (x, y, yaw, delta) ← bicycle_discrete_step.m
%   g   : Kpp, Kp, Ki, Kd, Lf, a, ell, wcur
%           PP만 (논문의 PP): Kpp = 1, Kp = Ki = Kd = 0, ell = 1 (필터 없음)
%   mem : 내부 상태 (Se = Σ e, ePrev = e[k-1], hist = 이전 출력, hint). 처음에는 []
%   반환 d (기록용): e (path_error 결과), dRef = atan(Lκ), eSteer = δ_ref - δ, eLat, eHead, ela, Lad,
%                    Se, dE, dPP, dP, dI, dD, dRaw, dCmd, alpha, xl, yl

if isempty(mem)
    mem = struct('Se', 0, 'ePrev', NaN, 'hist', repmat(car.delta, 20, 1), 'hint', NaN);   % 필터 창은 최대 21까지
end
L = p.L;

%% 1. 경로에 투영: 횡오차 e, 방향 오차 Δψ, 목표 조향각 δ_ref (그래프 ①용)
e = path_error(ref, car.x, car.y, car.yaw, mem.hint);
mem.hint = e.idx;
eLat = 0 - e.e_lat;                     % e  (목표 - 현재)
dPsi = 0 - e.e_head;                    % Δψ (목표 - 현재)
dRef = atan(L * e.kappa);
eSteer = dRef - car.delta;

%% 2. Pure Pursuit (Eq.3): 선행점 = 경로를 따라 투영점 + Lf
ql = path_pose(ref, e.s + g.Lf);
dx = ql.x - car.x; dy = ql.y - car.y;
alpha = atan2(dy, dx) - car.yaw;
alpha = atan2(sin(alpha), cos(alpha));
Lad   = hypot(dx, dy);
dPP   = atan(2 * L * sin(alpha) / Lad);

%% 3. P (Eq.4–6): 선행 오차 e_la = e + (a + L_ad) sin Δψ
ela = eLat + (g.a + Lad) * sin(dPsi);
dP  = g.Kp * ela;

%% 4. I (Eq.7): 횡오차의 합 Σ e
mem.Se = mem.Se + eLat;
dI = g.Ki * mem.Se;

%% 5. D: 횡오차의 변화율 (e[k] - e[k-1]) / T_s
if isnan(mem.ePrev)
    dE = 0;                             % 첫 스텝: e[k-1] = e[k]로 봄
else
    dE = (eLat - mem.ePrev) / T;
end
mem.ePrev = eLat;
dD = g.Kd * dE;

%% 6. PP_PID → Low-pass Filter (Eq.8–9)
dRaw = g.Kpp * dPP + dP + dI + dD;
[dCmd, mem.hist] = lpf_window(mem.hist, dRaw, g.ell, g.wcur);

d = struct('e', e, 'dRef', dRef, 'eSteer', eSteer, 'eLat', eLat, 'eHead', dPsi, 'ela', ela, 'Lad', Lad, ...
    'Se', mem.Se, 'dE', dE, 'dPP', dPP, 'dP', dP, 'dI', dI, 'dD', dD, 'dRaw', dRaw, 'dCmd', dCmd, ...
    'alpha', alpha, 'xl', ql.x, 'yl', ql.y);
end
