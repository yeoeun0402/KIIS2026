function [u, dbg, mem] = pid_step(st, e, rk, p, pid, mem, sCar)
% PID 한 스텝 (조향 PID + 속도 PID, 순수 PID: 곡률 피드포워드 없음)
%   입력: st   차량 상태 (x, y, yaw, v)
%         e    path_error() 결과 (e_lat: 경로에서 옆으로 벗어난 거리, 왼쪽 +)
%         rk   이번 시각의 레퍼런스 한 행 (s, x, y, psi, v)  ← make_reference_trajectory.m
%         p    vehicle_params(),  pid  pid_params()
%         mem  내부 상태 (두 적분값, 직전 오차 e[t-1]). 처음에는 []
%         sCar 차가 경로를 따라 간 거리 [m] (rk.s와 같은 기준)
%   출력: u.delta_cmd [rad], u.v_cmd [m/s] (= v + a_cmd*dt → 차량 모델 입력)
%         dbg  기록·Unity 표시용 내부값
%
%   조향 PID (이산 시간 모델, Ts = dt):
%     delta[t] = Kp e[t] + Ki Σ_{j=0..t} e[j] Ts + Kd (e[t] - e[t-1]) / Ts
%     e[t] = 0 - e_lat[t]  (레퍼런스 횡위치 = 경로 위 0 에서 지금 횡위치를 뺀 값)
%     → 차가 경로 오른쪽(e_lat < 0)이면 e > 0 → delta > 0 (왼쪽으로 꺾음)
%   속도 PID: a_cmd     =   Kp_s e_s + Ki_s ∫e_s dt + Kd_s e_v
%             e_s = s_r - s_car (기준점보다 뒤처지면 +),  e_v = v_r - v (= e_s의 시간 미분)

tStart = tic;
dt = p.dt;
if isempty(mem)
    mem = struct('iE', 0, 'ePrev', NaN, 'iS', 0);
end

%% 조향 PID (이산 시간 모델 그대로)
eK = -e.e_lat;                                      % e[t]
if isnan(mem.ePrev)
    eDiff = 0;                                      % 첫 스텝: e[t-1] = e[t]로 봄
else
    eDiff = (eK - mem.ePrev) / dt;                  % (e[t] - e[t-1]) / Ts
end
[delta_cmd, mem.iE, satD, tD] = pidLimited(pid.Kp, pid.Ki, pid.Kd, eK, eDiff, ...
    mem.iE, dt, pid.int_max, -p.deltaMax, p.deltaMax, pid.anti_windup);   % ±30°로 자름
mem.ePrev = eK;

%% 속도 PID (가속도 한계 = bicycle_step의 가감속 한계, 후진 금지면 v_cmd >= 0까지만)
eS = rk.s - sCar;
eV = rk.v - st.v;
aLo = -p.brake;
aHi = min(p.accel, (p.vMax - st.v) / dt);
if ~pid.allow_reverse
    aLo = max(aLo, -st.v / dt);
end
[a_cmd, mem.iS, satA, tA] = pidLimited(pid.Kp_s, pid.Ki_s, pid.Kd_s, eS, eV, ...
    mem.iS, dt, pid.int_s_max, aLo, aHi, true);

%% 차량 모델 입력: 가속도 → 다음 스텝 목표 속도 (bicycle_step의 가감속 한계가 그대로 가속도 포화로 작동)
u.delta_cmd = delta_cmd;
u.v_cmd     = st.v + a_cmd * dt;
if ~pid.allow_reverse
    u.v_cmd = max(u.v_cmd, 0);
end

%% 기록·표시용
gap = hypot(rk.x - st.x, rk.y - st.y);
dbg = struct('e', eK, 'e_int', mem.iE, 'e_diff', eDiff, ...
    'p_term', tD(1), 'i_term', tD(2), 'd_term', tD(3), 'sat', satD, 'delta_cmd', delta_cmd, ...
    'e_s', eS, 'e_v', eV, 'e_s_int', mem.iS, ...
    'a_p', tA(1), 'a_i', tA(2), 'a_d', tA(3), 'a_cmd', a_cmd, 'a_sat', satA, ...
    'x_w', e.x_w, 'y_w', e.y_w, ...                             % Unity: p_w 자리 = 최근접 경로점
    'x_d', NaN, 'y_d', NaN, 'x_l', rk.x, 'y_l', rk.y, ...      % Unity: p_l 자리 = 기준점 p_r
    'x_c', NaN, 'y_c', NaN, 'R', NaN, 'Lf', gap, 'tau', NaN);   % Unity HUD의 Lf = 차-기준점 거리
dbg.comp_ms = toc(tStart) * 1000;
end

function [out, I, sat, terms] = pidLimited(Kp, Ki, Kd, err, dErr, I, dt, iMax, lo, hi, antiWindup)
% 출력 = Kp err + Ki Σ err dt + Kd dErr 를 lo ~ hi로 자름
% antiWindup = true면 적분 와인드업 방지: ① 상한 iMax ② 출력이 포화됐고 적분이 포화를 더 키우는 방향이면 적분 멈춤
Inew = I + err * dt;                                 % Σ err[j] Ts
if antiWindup, Inew = min(max(Inew, -iMax), iMax); end
raw  = Kp * err + Ki * Inew + Kd * dErr;
if antiWindup && ((raw > hi && Ki * err > 0) || (raw < lo && Ki * err < 0))
    Inew = I;
    raw  = Kp * err + Ki * Inew + Kd * dErr;
end
out   = min(max(raw, lo), hi);
sat   = raw > hi || raw < lo;
terms = [Kp * err, Ki * Inew, Kd * dErr];
I     = Inew;
end
