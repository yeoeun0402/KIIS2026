function ok = test_pid()
% PID 궤적 추종 확인용 테스트. 실행: test_pid
%   1) 레퍼런스 궤적: 정지에서 출발, 가속 <= a_ref, 감속 <= b_ref, 위치가 경로 위,
%      경로 점마다 속도 상한이 pure pursuit의 v_cmd와 같음 (v_tar 0.8, 1.2)
%   2) 부호: 경로 왼쪽이면 delta < 0, 뒤처지면 가속, 앞서면 감속, 오차 0이면 출력 0
%   3) 적분: Σ e Ts 그대로 쌓임 (anti_windup = true면 상한과 포화 시 멈춤)
%   4) 미분: (e[t] - e[t-1]) / Ts 그대로
%   5) 후진 방지: 기준점보다 앞서 있어도 v, v_cmd >= 0 (브레이크로 멈춤)
%   6) 폐루프: 직선에서 0.3 m 옆, 정지 상태로 출발 → 10~15 s 동안 e_lat, e_s 작음
%   7) 이산 시간 모델 식: delta[t] = Kp e[t] + Ki Σ e[j] Ts + Kd (e[t] - e[t-1]) / Ts 를 따로 계산한 값과 같음

here = fileparts(mfilename('fullpath'));            % MATLAB/PID
addpath(fileparts(here), fullfile(fileparts(here), 'Pure-Pursuit'));   % 공용 함수, 속도 규칙 비교용 pure pursuit
p = vehicle_params();
dt = p.dt;
ds = 0.01;
results = strings(0, 1);
pass = true(0, 1);

%% 1) 레퍼런스 궤적 (실제 맵 경로)
ref = load_reference_path();
for vt = [0.8 1.2]
    pid = pid_params('v_tar', vt);
    traj = make_reference_trajectory(ref, p, pid, 60, 0);
    acc = diff(traj.v) / dt;
    check(traj.v(1) == 0 && traj.s(1) == 0, sprintf('traj v_tar %.1f: starts at rest at s0', vt));
    check(max(acc) <= pid.a_ref + 1e-9 && min(acc) >= -1.05 * pid.b_ref, ...
        sprintf('traj v_tar %.1f: accel %.3f ~ %.3f m/s^2 (limits +%.1f / -%.1f)', vt, min(acc), max(acc), pid.a_ref, pid.b_ref));
    k = 1:25:numel(traj.t);
    eOn = arrayfun(@(i) path_error(ref, traj.x(i), traj.y(i), traj.psi(i)).e_lat, k);
    check(max(abs(eOn)) < 1e-3, sprintf('traj v_tar %.1f: points on path (max |e_lat| %.1e m)', vt, max(abs(eOn))));
    ppf = pp_params('mode', 'fixed', 'v_tar', vt);
    idx = 1:7:ref.N;
    dv = zeros(size(idx));
    for j = 1:numel(idx)
        i = idx(j);
        st = state(ref.x(i), ref.y(i), ref.psi(i), vt);
        e = path_error(ref, st.x, st.y, st.yaw, i);
        u = pure_pursuit_step(st, ref, p, ppf, e);
        dv(j) = u.v_cmd - traj.vprof_raw(e.idx);
    end
    check(max(abs(dv)) < 1e-12, sprintf('traj v_tar %.1f: speed limit = pure pursuit v_cmd (max diff %.1e)', vt, max(abs(dv))));
end

%% 2) 부호 (직선 경로, +x 방향)
ref = makeRef(stadium(-20, 40, 10, ds), ds);
pid = pid_params();
[uL, ~] = stepAt(5, 0.1, 0);
[uR, ~] = stepAt(5, -0.1, 0);
[u0, ~] = stepAt(5, 0, 0);
check(uL.delta_cmd < 0 && uR.delta_cmd > 0, sprintf('sign: left -> delta %.3f, right -> delta %.3f rad', uL.delta_cmd, uR.delta_cmd));
check(u0.delta_cmd == 0 && abs(u0.v_cmd - 0.8) < 1e-12, sprintf('sign: on reference -> delta %.3g, v_cmd %.3f', u0.delta_cmd, u0.v_cmd));
[uB, ~] = stepAt(5, 0, 0.2);
[uA, ~] = stepAt(5, 0, -0.2);
check(uB.v_cmd > 0.8 && uA.v_cmd < 0.8, sprintf('sign: behind -> v_cmd %.4f, ahead -> v_cmd %.4f (v = 0.8)', uB.v_cmd, uA.v_cmd));

%% 3) 적분
pid = pid_params('Kp', 0, 'Kd', 0, 'Ki', 0.01);
mem = runSteps(5, 0.05, 50);                     % e_lat = 0.05 → e = -0.05
check(abs(mem.iE + 0.05 * 50 * dt) < 1e-12, sprintf('integral: %.4f (expected %.4f)', mem.iE, -0.05 * 50 * dt));
mem = runSteps(5, 0.05, 600);
check(abs(mem.iE + 0.05 * 600 * dt) < 1e-12, sprintf('integral (model, no clamp): %.3f (expected %.3f)', mem.iE, -0.05 * 600 * dt));
pid = pid_params('Kp', 0, 'Kd', 0, 'Ki', 0.01, 'anti_windup', true);
mem = runSteps(5, 0.05, 600);
check(abs(mem.iE + pid.int_max) < 1e-12, sprintf('anti_windup clamp: %.3f (int_max %.1f)', mem.iE, pid.int_max));
pid = pid_params('Kp', 20, 'anti_windup', true);
mem = runSteps(5, 0.1, 50);
check(mem.iE == 0, sprintf('anti_windup: steering saturated -> integral %.3g', mem.iE));

%% 4) 미분: e_lat = 0.1 t → e = -0.1 t → (e[t] - e[t-1]) / Ts = -0.1
pid = pid_params();
mem = [];
st = state(0, 0, 0, 0.8);
rk = struct('s', 0, 'x', 0, 'y', 0, 'psi', 0, 'v', 0.8);
for k = 1:5
    [~, d, mem] = pid_step(st, struct('e_lat', 0.1 * k * dt, 'x_w', 0, 'y_w', 0), rk, p, pid, mem, 0);
end
check(abs(d.e_diff + 0.1) < 1e-12, sprintf('derivative: %.5f m/s (expected -0.1)', d.e_diff));

%% 7) 이산 시간 모델 식과 같은지 (임의의 오차 열 200개)
rng(1);
eLat = 0.05 * cumsum(randn(200, 1)) * 0.1;
pid = pid_params();
mem = []; maxDiff = 0; S = 0; ePrev = NaN;
for k = 1:numel(eLat)
    [u, ~, mem] = pid_step(st, struct('e_lat', eLat(k), 'x_w', 0, 'y_w', 0), rk, p, pid, mem, 0);
    ek = -eLat(k);
    S = S + ek * dt;
    if isnan(ePrev), dk = 0; else, dk = (ek - ePrev) / dt; end
    ePrev = ek;
    dModel = min(max(pid.Kp * ek + pid.Ki * S + pid.Kd * dk, -p.deltaMax), p.deltaMax);
    maxDiff = max(maxDiff, abs(u.delta_cmd - dModel));
end
check(maxDiff < 1e-12, sprintf('model: delta = Kp e + Ki Σ e Ts + Kd (e[t]-e[t-1])/Ts (max diff %.1e)', maxDiff));

%% 5) 후진 방지: 기준점보다 1 m 앞서 있음 → 브레이크로 멈춘 뒤 0에서 더 내려가지 않음
st = state(5, 0, 0, 0.3);
e = path_error(ref, st.x, st.y, st.yaw);
rk = struct('s', e.s - 1, 'x', 4, 'y', 0, 'psi', 0, 'v', 0);
mem = []; vMin = inf;
for k = 1:100
    [u, ~, mem] = pid_step(st, e, rk, p, pid, mem, e.s);
    st = bicycle_step(st, u, p, dt);
    st.x = 5; st.y = 0; st.yaw = 0;              % 위치는 고정 (기준점보다 계속 앞선 상태)
    vMin = min([vMin, st.v, u.v_cmd]);
end
check(vMin >= 0 && st.v == 0, sprintf('no reverse: min v / v_cmd %.3f, final v %.3f', vMin, st.v));

%% 6) 폐루프: 직선에서 0.3 m 오른쪽, 정지 상태로 출발
ref = makeRef(stadium(-5, 15, 2, ds), ds);
st = state(-4.5, -0.3, 0, 0);
e = path_error(ref, st.x, st.y, st.yaw);
traj = make_reference_trajectory(ref, p, pid, 16, e.s);
mem = []; progress = 0; sPrev = e.s;
E = nan(750, 3);
for k = 1:750
    if k > 1
        e = path_error(ref, st.x, st.y, st.yaw, e.idx);
        dS = e.s - sPrev; progress = progress + dS - ref.L * round(dS / ref.L); sPrev = e.s;
    end
    rk = struct('s', traj.s(k), 'x', traj.x(k), 'y', traj.y(k), 'psi', traj.psi(k), 'v', traj.v(k));
    [u, dbg, mem] = pid_step(st, e, rk, p, pid, mem, traj.s(1) + progress);
    E(k, :) = [st.t, e.e_lat, dbg.e_s];
    st = bicycle_step(st, u, p, dt);
end
late = E(:, 1) >= 10;
check(max(abs(E(late, 2))) < 0.02 && max(abs(E(late, 3))) < 0.05, ...
    sprintf('closed loop: t 10~15 s max|e_lat| %.4f m, max|e_s| %.4f m (start 0.3 m off, at rest)', ...
    max(abs(E(late, 2))), max(abs(E(late, 3)))));

%% 결과
for k = 1:numel(results)
    fprintf('%s  %s\n', ternary(pass(k), 'PASS', 'FAIL'), results(k));
end
ok = all(pass);
fprintf('%d / %d passed\n', sum(pass), numel(pass));

    function check(cond, msg)
        results(end + 1, 1) = string(msg);
        pass(end + 1, 1) = cond;
    end

    function [u, dbg] = stepAt(x, y, sAhead)
        % 직선 위 (x, y)에 v = 0.8로 있는 차, 기준점은 경로 위 sAhead만큼 앞에서 같은 속도
        s = state(x, y, 0, 0.8);
        ee = path_error(ref, s.x, s.y, s.yaw);
        r = struct('s', ee.s + sAhead, 'x', x + sAhead, 'y', 0, 'psi', 0, 'v', 0.8);
        [u, dbg] = pid_step(s, ee, r, p, pid, [], ee.s);
    end

    function m = runSteps(x, y, n)
        % 같은 자리에서 같은 오차로 n 스텝 (기준점은 차와 같은 앞뒤 위치·속도)
        s = state(x, y, 0, 0.8);
        ee = path_error(ref, s.x, s.y, s.yaw);
        r = struct('s', ee.s, 'x', x, 'y', 0, 'psi', 0, 'v', 0.8);
        m = [];
        for n1 = 1:n
            [~, ~, m] = pid_step(s, ee, r, p, pid, m, ee.s);
        end
    end
end

%% ---- 보조 함수 (test_pure_pursuit.m과 같음) ----
function st = state(x, y, yaw, v)
st = struct('t', 0, 'x', x, 'y', y, 'yaw', yaw, 'v', v, 'delta', 0, 'yawRate', 0);
end

function xy = stadium(x0, len, R, ds)
% 반시계 경기장: 아래 직선 (x0 -> x0+len, y=0) → 오른쪽 반원 → 위 직선 → 왼쪽 반원
s = (0:ds:len - ds)';
a1 = (-pi / 2:ds / R:pi / 2 - ds / R)';
a2 = (pi / 2:ds / R:3 * pi / 2 - ds / R)';
xy = [x0 + s, 0 * s;
      x0 + len + R * cos(a1), R + R * sin(a1);
      x0 + len - s, 2 * R + 0 * s;
      x0 + R * cos(a2), R + R * sin(a2)];
end

function ref = makeRef(xy, ds)
% extract_lane_path와 같은 방식으로 s, psi, kappa 계산
N = size(xy, 1); x = xy(:, 1); y = xy(:, 2); h = 10;
ip = mod((0:N - 1)' + h, N) + 1; im = mod((0:N - 1)' - h, N) + 1;
dx = (x(ip) - x(im)) / (2 * h * ds); dy = (y(ip) - y(im)) / (2 * h * ds);
ddx = (x(ip) - 2 * x + x(im)) / (h * ds)^2; ddy = (y(ip) - 2 * y + y(im)) / (h * ds)^2;
seg = hypot(diff([x; x(1)]), diff([y; y(1)]));
ref = struct('x', x, 'y', y, 's', [0; cumsum(seg(1:end - 1))], 'psi', atan2(dy, dx), ...
    'kappa', (dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^1.5, 'N', N, 'ds', ds, ...
    'L', sum(seg), 'closed', true, 'name', "test");
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
