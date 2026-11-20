function [dCmd, mem, d] = pid_steer(car, ref, p, g, mem, T)
% 비교용 PID 조향 한 스텝 (논문의 비교 대상 PID: 되먹임만 쓰는 제어기)
%   PID/PID_control.m의 조향 PID와 같은 식·같은 기본 게인 (이산 시간):
%     δ[k] = K_p e[k] + K_i Σ_{j≤k} e[j] T_s + K_d (e[k] - e[k-1]) / T_s,   e = 0 - e_lat (횡오차, 목표 - 현재)
%   g : Kp, Ki, Kd, ell, wcur  (ell = 1이면 Low pass 없음. 'lpfAll'이 true일 때만 PP_PI와 같은 필터)
%   mem: 내부 상태 (I, ePrev, hist, hint). 처음에는 []
%   반환 d: blended_steer와 같은 이름의 기록용 값 (PP 관련 값은 NaN)

if isempty(mem)
    mem = struct('I', 0, 'ePrev', NaN, 'hist', repmat(car.delta, 20, 1), 'hint', NaN);
end
L = p.L;
e = path_error(ref, car.x, car.y, car.yaw, mem.hint);
mem.hint = e.idx;
eLat = 0 - e.e_lat;
dRef = atan(L * e.kappa);

mem.I = mem.I + eLat * T;
if isnan(mem.ePrev), dE = 0; else, dE = (eLat - mem.ePrev) / T; end
mem.ePrev = eLat;
dP = g.Kp * eLat; dI = g.Ki * mem.I; dD = g.Kd * dE;
dRaw = dP + dI + dD;
[dCmd, mem.hist] = lpf_window(mem.hist, dRaw, g.ell, g.wcur);

d = struct('e', e, 'dRef', dRef, 'eSteer', dRef - car.delta, 'eLat', eLat, 'eHead', -e.e_head, 'ela', NaN, ...
    'Lad', NaN, 'Se', mem.I, 'dPP', NaN, 'dP', dP, 'dI', dI, 'dD', dD, 'dRaw', dRaw, 'dCmd', dCmd, ...
    'alpha', NaN, 'xl', NaN, 'yl', NaN);
end
