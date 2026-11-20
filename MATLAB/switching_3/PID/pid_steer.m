function [dCmd, mem, d] = pid_steer(car, ref, p, g, mem, T)
% 순수 PID 조향 한 스텝 (switching_3용, 필터 없음)
%   PID/PID_control.m · pid_step.m의 조향 PID와 같은 식, blended_control/pid_steer.m에서 Low-pass 부분만 뺀 것
%   (blended_discrete.m의 파란 "PID" 차와 같은 값)
%
%   δ_PID[k] = K_p e[k] + K_i Σ_{j≤k} e[j] T_s + K_d (e[k] - e[k-1]) / T_s
%   e = 0 - e_lat   (횡오차, 목표 - 현재. 차가 경로 오른쪽이면 e > 0 → 왼쪽으로 꺾음)
%
%   car : 차 상태 (x, y, yaw, delta) ← bicycle_discrete_step.m
%   g   : g.Kp [rad/m], g.Ki [rad/(m s)], g.Kd [rad s/m]
%   mem : 내부 상태 (I = Σ e T_s, ePrev, hint). 처음에는 []
%   반환 d: e (path_error 결과), eLat, dP, dI, dD

if isempty(mem), mem = struct('I', 0, 'ePrev', NaN, 'hint', NaN); end
e = path_error(ref, car.x, car.y, car.yaw, mem.hint);
mem.hint = e.idx;
eLat = 0 - e.e_lat;

mem.I = mem.I + eLat * T;
if isnan(mem.ePrev), dE = 0; else, dE = (eLat - mem.ePrev) / T; end   % 첫 스텝: e[k-1] = e[k]로 봄
mem.ePrev = eLat;
dP = g.Kp * eLat; dI = g.Ki * mem.I; dD = g.Kd * dE;
dCmd = dP + dI + dD;

d = struct('e', e, 'eLat', eLat, 'dP', dP, 'dI', dI, 'dD', dD);
end
