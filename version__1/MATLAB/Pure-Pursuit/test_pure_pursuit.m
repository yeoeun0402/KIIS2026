function ok = test_pure_pursuit()
% Pure pursuit 기하·모델 확인용 테스트 (설계 문서 8절). 실행: test_pure_pursuit
%   1) 직선 경로에 정렬 → delta ≈ 0 (모든 모드)
%   2) 반경 R 원 경로 위 → delta ≈ atan(L/R)
%   3) 직선에서 왼쪽 코너 진입 (paper, Step 2) → p_l이 p_d보다 코너 바깥(p_w 접선 쪽)
%   4) path_error: 경로 왼쪽이면 e_lat > 0
%   5) bicycle_step: 일정 조향각 → yawRate = v tan(delta) / L
%   6) 속도 명령: 직선 v_tar, 곡선 sqrt(9.79 (e_max+f_max) R), 코너 직전(선행점이 곡선 위)에도 감속
%   7) 이산 시간 모델 식: delta[t] = atan(2 L sin(alpha[t]) / L_ad) 를 따로 계산한 값과 같음

base = fileparts(fileparts(mfilename('fullpath')));   % MATLAB (공용 함수)
if ~exist('vehicle_params', 'file'), addpath(base); end
p = vehicle_params();
ds = 0.01;
results = strings(0, 1);
pass = true(0, 1);

%% 7) 이산 시간 모델 식 (임의의 차 자세·선행점 500개)
rng(1);
maxDiff = 0;
for k = 1:500
    st = state(randn, randn, pi * (2 * rand - 1), 0.8, 0);
    pl = [st.x, st.y] + (0.5 + 2 * rand) * [cos(st.yaw + 1.2 * randn), sin(st.yaw + 1.2 * randn)];
    Lad   = norm(pl - [st.x, st.y]);                                       % 선행 거리
    alpha = angle(exp(1i * (atan2(pl(2) - st.y, pl(1) - st.x) - st.yaw))); % 차 방향과 선행점 방향 사이 각
    dModel = min(max(atan(2 * p.L * sin(alpha) / Lad), -p.deltaMax), p.deltaMax);
    maxDiff = max(maxDiff, abs(pursuit_steer(st, pl, p) - dModel));
end
check(maxDiff < 1e-12, sprintf('model: delta = atan(2 L sin(alpha) / L_ad) (max diff %.1e)', maxDiff));

%% 1) 직선
ref = makeRef(stadium(-20, 40, 10, ds), ds);
st = state(0, 0, 0, 0.8, 0);
for m = ["fixed", "speed", "paper"]
    pp = pp_params('mode', m);
    u = pure_pursuit_step(st, ref, p, pp, path_error(ref, st.x, st.y, st.yaw));
    check(abs(u.delta_cmd) < 1e-3, sprintf('straight (%s): delta_cmd = %.2e rad', m, u.delta_cmd));
end

%% 2) 원 (반경 3 m, 반시계)
R = 3;
a = (0:ds / R:2 * pi - ds / R)';
ref = makeRef(R * [cos(a), sin(a)], ds);
target = atan(p.L / R);
st = state(R, 0, pi / 2, 0.8, target);
for m = ["fixed", "speed", "paper"]
    pp = pp_params('mode', m);
    u = pure_pursuit_step(st, ref, p, pp, path_error(ref, st.x, st.y, st.yaw));
    check(abs(u.delta_cmd - target) < deg2rad(0.5), sprintf('circle R=%g (%s): delta_cmd = %.2f deg, atan(L/R) = %.2f deg', ...
        R, m, rad2deg(u.delta_cmd), rad2deg(target)));
end

%% 3) 왼쪽 코너 진입: 직선 (y = 0, +x 방향) 끝 x = 10에서 반경 2 m 왼쪽 반원
ref = makeRef(stadium(-20, 30, 2, ds), ds);
st = state(9.2, 0, 0, 0.8, 0);
pp = pp_params('mode', 'paper');
[~, dbg] = pure_pursuit_step(st, ref, p, pp, path_error(ref, st.x, st.y, st.yaw));
check(dbg.x_d > 10 && dbg.y_d > 0, sprintf('corner: p_d on the arc (x_d = %.2f, y_d = %.2f)', dbg.x_d, dbg.y_d));
check(dbg.tau > 0.1, sprintf('corner: tau = %.2f (alpha %.2f, beta %.2f)', dbg.tau, dbg.alpha, dbg.beta));
check(dbg.y_l < dbg.y_d && dbg.y_l > -0.01, sprintf('corner: p_l moved outward (y_l = %.3f < y_d = %.3f, stays above tangent y=0)', dbg.y_l, dbg.y_d));
pp2 = pp_params('mode', 'paper', 'step2_on', false);
[u1, ~] = pure_pursuit_step(st, ref, p, pp, path_error(ref, st.x, st.y, st.yaw));
[u2, ~] = pure_pursuit_step(st, ref, p, pp2, path_error(ref, st.x, st.y, st.yaw));
check(u1.delta_cmd < u2.delta_cmd, sprintf('corner: Step 2 reduces early left steer (%.2f < %.2f deg)', ...
    rad2deg(u1.delta_cmd), rad2deg(u2.delta_cmd)));

%% 4) path_error 부호
ref = makeRef(stadium(-20, 40, 10, ds), ds);
eL = path_error(ref, 5, 0.1, 0);
eR = path_error(ref, 5, -0.1, 0);
check(abs(eL.e_lat - 0.1) < 1e-6 && abs(eR.e_lat + 0.1) < 1e-6, ...
    sprintf('path_error: left %.4f, right %.4f', eL.e_lat, eR.e_lat));

%% 5) bicycle_step 정상상태
st = state(0, 0, 0, 0.5, 0);
u = struct('delta_cmd', 0.3, 'v_cmd', 0.5);
for k = 1:200, st = bicycle_step(st, u, p, p.dt); end
check(abs(st.yawRate - 0.5 * tan(0.3) / p.L) < 1e-9 && abs(st.delta - 0.3) < 1e-12, ...
    sprintf('bicycle_step: yawRate %.4f rad/s (expected %.4f)', st.yawRate, 0.5 * tan(0.3) / p.L));

%% 6) 속도 명령 (논문 4.1절): 곡선에서 v = sqrt(9.79 (e_max + f_max) R), 직선에서 v_tar
gf = 9.79 * 0.05;
ref = makeRef(stadium(-20, 40, 10, ds), ds);
st = state(0, 0, 0, 1.2, 0);
u = pure_pursuit_step(st, ref, p, pp_params('v_tar', 1.2), path_error(ref, st.x, st.y, st.yaw));
check(abs(u.v_cmd - 1.2) < 1e-12, sprintf('speed straight: v_cmd = %.3f (v_tar 1.2)', u.v_cmd));

R = 1.78;
a = (0:ds / R:2 * pi - ds / R)';
ref = makeRef(R * [cos(a), sin(a)], ds);
st = state(R, 0, pi / 2, 1.2, atan(p.L / R));
e = path_error(ref, st.x, st.y, st.yaw);
u = pure_pursuit_step(st, ref, p, pp_params('v_tar', 1.2), e);
check(abs(u.v_cmd - sqrt(gf * R)) < 5e-3, sprintf('speed circle R=%g: v_cmd = %.4f (expected %.4f)', R, u.v_cmd, sqrt(gf * R)));
u = pure_pursuit_step(st, ref, p, pp_params('v_tar', 0.8), e);
check(abs(u.v_cmd - 0.8) < 1e-12, sprintf('speed circle R=%g: v_tar 0.8 below limit -> v_cmd = %.3f', R, u.v_cmd));
u = pure_pursuit_step(st, ref, p, pp_params('v_tar', 1.2, 'v_mode', 'const'), e);
check(abs(u.v_cmd - 1.2) < 1e-12, sprintf('speed circle R=%g (const): v_cmd = %.3f', R, u.v_cmd));

ref = makeRef(stadium(-20, 30, 2, ds), ds);      % 직선 끝 x = 10에서 반경 2 m 코너
st = state(8.5, 0, 0, 1.2, 0);                   % 차는 직선 위, 2.5 m 앞은 코너 위
u = pure_pursuit_step(st, ref, p, pp_params('v_tar', 1.2), path_error(ref, st.x, st.y, st.yaw));
check(abs(u.v_cmd - sqrt(gf * 2)) < 5e-3, sprintf('speed before corner: v_cmd = %.4f (expected %.4f)', u.v_cmd, sqrt(gf * 2)));

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
end

%% ---- 보조 함수 ----
function st = state(x, y, yaw, v, delta)
st = struct('t', 0, 'x', x, 'y', y, 'yaw', yaw, 'v', v, 'delta', delta, 'yawRate', 0);
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
