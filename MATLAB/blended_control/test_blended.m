function ok = test_blended(varargin)
% blended_control 폴더 테스트. 실행: cd MATLAB/blended_control; test_blended
%   test_blended                        % Unity Play 먼저: 1번에서 Unity 차가 한 바퀴(약 52 s) 끝까지 감
%   test_blended('unityTime', 20)       % 1번을 20초만 (짧게 확인)
%   test_blended('unity', false)        % Unity 없이 (1번 건너뜀)
%   1) Unity 연동: 자동 버전을 실시간으로 한 바퀴 돌려 PP_PID 차 상태를 Unity로 매 스텝 보냄 (Unity 차가 한 바퀴 돎)
%   2) 복제본이 원본(MATLAB/bicycle_step.m, test1/*)과 같은 파일
%   3) bicycle_discrete_step: a = 0이면 bicycle_step과 같은 결과
%   4) bicycle_discrete_step: 직진 가속 거리가 더블 인테그레이터 식과 같음 (s = a(kT)^2/2)
%   5) bicycle_discrete_step: 조향 한계 (±90°/s, ±30°), 후진 없음
%   6) bicycle_discrete_step: 일정 조향각이면 반지름 L / tan(delta)인 원
%   7) lpf_window (논문 Eq.8–9): ℓ = 1 통과, ℓ = 2 = 1차 필터, 일정 입력 유지 (DC 이득 1), ℓ = 3 손 계산
%   8) blended_steer: 경로 위 + 경로 방향이면 e = 0, e_la = 0, δ_ref = atan(Lκ)
%   9) blended_steer: K_pp = 1, PID = 0, 필터 없음 → δ = δ_pp (Eq.3)
%  10) blended_steer: PID만 → δ = K_p e_la + K_i Σe + K_d (e[k] - e[k-1]) / T_s (Eq.4–7 + D, 3 스텝 손 계산)
%  11) PP_PID: δ_raw = K_pp δ_pp + δ_p + δ_i + δ_d → LPF, K_pp = 1 - K_p - K_i - K_d, 비교용 PID(pid_steer) 손 계산
%  12) 자동 버전 오프라인 한 바퀴: 세 제어기 완주, 결과 파일, PP_PID 오차가 PP·PID보다 작음
%  13) 키보드 버전은 Unity 없이 실행하면 오류
%  14) PP_PID = 매 스텝 합 (switching 아님): 한 바퀴 내내 δ_raw = K_pp δ_pp + (δ_p + δ_i + δ_d), 두 항이 동시에 쓰임
%  15) 튜닝 파라미터: blended_params 덮어쓰기, 모르는 이름 오류, 합 = 1 규칙, v·Lf·게인·필터가 한 바퀴 내내 반영됨
%  16) 실행 중 튜닝 (check_live_tuning): 한 바퀴 동안 v, Lf, a, wcur, Kp, Ki, ell, Kd를 슬라이더·입력칸으로 바꾸고,
%      기록에서 잰 값이 바꾼 스텝부터 새 값과 같은지 (Kpp = 1 - Kp - Ki - Kd 포함) + 세 제어기 δ 다시 계산 = 기록

topt = struct('unity', true, 'unityTime', Inf);
for i = 1:2:numel(varargin), topt.(char(varargin{i})) = varargin{i + 1}; end
here = fileparts(mfilename('fullpath'));
base = fileparts(here);
if ~exist('vehicle_params', 'file'), addpath(base, '-end'); end
addpath(here);
p = vehicle_params();
ref = load_reference_path();
L = p.L;
results = strings(0, 1);
pass = true(0, 1);

%% 1) Unity 연동 (Unity Play 먼저)
if topt.unity
    fullLap = isinf(topt.unityTime);
    if fullLap
        fprintf('[1] Unity 연동: Unity Game 창에서 PP_PID 차가 경로를 따라 한 바퀴(약 52 s) 돌아야 합니다.\n');
        r = [];
        try, r = blended_discrete('input', 'auto', 'save', false); catch ME, end
    else
        fprintf('[1] Unity 연동: Unity Game 창에서 PP_PID 차가 %g초 동안 경로를 따라 움직여야 합니다.\n', topt.unityTime);
        r = [];
        try, r = blended_discrete('input', 'auto', 'tMax', topt.unityTime, 'save', false); catch ME, end
    end
    if ~isempty(r)
        nExp = round(min(topt.unityTime, 51.8) / 0.1) - 1;
        check(r.unity.used && ~r.unity.lost && r.steps >= nExp && r.unity.sent == r.steps + 1 && (r.lapDone || ~fullLap), ...
            sprintf('Unity link: %d steps in real time (%.2f m, lap %s), %d states sent to %s:%d', ...
            r.steps, r.distance, string(ternary(r.lapDone, 'done', 'not done')), r.unity.sent, p.unityIP, p.unityPort));
    else
        check(false, "Unity link: 연결 실패 - Unity에서 Play를 먼저 누르세요 (" + ME.message + ")");
    end
else
    fprintf('[1] Unity 연동 건너뜀 (''unity'', false)\n');
end

%% 2) 복제본 = 원본
same = @(a, b) strcmp(fileread(a), fileread(b));
check(same(fullfile(here, 'bicycle_step.m'), fullfile(base, 'bicycle_step.m')) && ...
      same(fullfile(here, 'path_model_matrices.m'), fullfile(base, 'test1', 'path_model_matrices.m')) && ...
      same(fullfile(here, 'path_model_step.m'), fullfile(base, 'test1', 'path_model_step.m')) && ...
      same(fullfile(here, 'path_pose.m'), fullfile(base, 'test1', 'path_pose.m')), ...
    'copies: bicycle_step, path_model_matrices, path_model_step, path_pose = originals');

%% 3) a = 0이면 bicycle_step과 같음 (여러 상태, 여러 omega)
rng(1);
worst = 0;
for i = 1:50
    st = struct('t', 0, 'x', randn, 'y', randn, 'yaw', pi * (2 * rand - 1), 's', 0, ...
        'v', 1.4 * rand, 'delta', deg2rad(25) * (2 * rand - 1), 'yawRate', 0);
    om = deg2rad(80) * (2 * rand - 1);
    s1 = bicycle_discrete_step(st, [0; om], p, 0.1);
    s2 = bicycle_step(st, struct('delta_cmd', st.delta + 0.1 * om, 'v_cmd', st.v), p, 0.1);
    worst = max([worst, abs(s1.x - s2.x), abs(s1.y - s2.y), abs(s1.yaw - s2.yaw), ...
        abs(s1.v - s2.v), abs(s1.delta - s2.delta), abs(s1.yawRate - s2.yawRate)]);
end
check(worst < 1e-12, sprintf('a = 0: bicycle_discrete_step = bicycle_step (max diff %.1e)', worst));

%% 4) 직진 가속 (a = 0.5, T = 0.1, 10 스텝 = 1 s)
st = struct('t', 0, 'x', 0, 'y', 0, 'yaw', 0, 's', 0, 'v', 0, 'delta', 0, 'yawRate', 0);
for k = 1:10, st = bicycle_discrete_step(st, [0.5; 0], p, 0.1); end
check(abs(st.x - 0.25) < 1e-12 && abs(st.s - 0.25) < 1e-12 && abs(st.v - 0.5) < 1e-12 && abs(st.y) < 1e-15, ...
    sprintf('double integrator: x %.6f, s %.6f (0.25), v %.6f (0.5)', st.x, st.s, st.v));

%% 5) 한계
st = struct('t', 0, 'x', 0, 'y', 0, 'yaw', 0, 's', 0, 'v', 0.5, 'delta', 0, 'yawRate', 0);
[s1, u1] = bicycle_discrete_step(st, [0; 10], p, 0.1);
st.delta = deg2rad(29);
s2 = bicycle_discrete_step(st, [0; p.steerRate], p, 0.1);
st.v = 0.1;
[s3, u3] = bicycle_discrete_step(st, [-p.brake; 0], p, 0.1);
check(abs(u1(2) - p.steerRate) < 1e-12 && abs(s1.delta - 0.1 * p.steerRate) < 1e-12 && ...
      abs(s2.delta - p.deltaMax) < 1e-12 && abs(s3.v) < 1e-12 && abs(u3(1) + 1) < 1e-12, ...
    sprintf('limits: omega %.3f rad/s, delta max %.1f deg, no reverse v %.2g', u1(2), rad2deg(s2.delta), s3.v));

%% 6) 일정 조향각 → 반지름 L / tan(delta) 원 (작은 T로 적분 오차 확인)
dl = 0.3; R = L / tan(dl);
st = struct('t', 0, 'x', 0, 'y', 0, 'yaw', 0, 's', 0, 'v', 0.5, 'delta', dl, 'yawRate', 0);
xs = zeros(2000, 1); ys = xs;
for k = 1:2000
    st = bicycle_discrete_step(st, [0; 0], p, 0.01);
    xs(k) = st.x; ys(k) = st.y;
end
r = hypot(xs - 0, ys - R);                    % 원의 중심 (0, R)
check(abs(st.yawRate - 0.5 * tan(dl) / L) < 1e-12 && max(abs(r - R)) < 0.01, ...
    sprintf('circle: R %.3f m, radius error max %.4f m, yawRate %.4f rad/s', R, max(abs(r - R)), st.yawRate));

%% 7) Low-pass Filter (논문 Eq.8–9): ℓ=1 통과, ℓ=2 = 1차 필터, 일정 입력 유지, ℓ=3 손 계산
hist = repmat(0.2, 20, 1);
[y1, ~] = lpf_window(hist, 1.0, 1, 0.5);
yA = 0; hA = zeros(20, 1); yB = 0;
for k = 1:5
    [yA, hA] = lpf_window(hA, 1, 2, 0.3);              % ℓ = 2
    yB = 0.3 * 1 + 0.7 * yB;                           % 1차 저역통과 y = w u + (1 - w) y[k-1]
end
hC = repmat(0.4, 20, 1); yC = 0;
for k = 1:10, [yC, hC] = lpf_window(hC, 0.4, 4, 0.2); end   % 일정 입력 → 그대로 (DC 이득 1)
hD = [0.3; 0.1; 5; 5]; [yD, hD2] = lpf_window(hD, 1.0, 3, 0.5);  % ℓ = 3: 0.5*1 + 0.25*(0.3 + 0.1)
check(y1 == 1.0 && abs(yA - yB) < 1e-15 && abs(yC - 0.4) < 1e-15 && abs(yD - 0.6) < 1e-15 && hD2(1) == yD && hD2(2) == 0.3, ...
    sprintf('LPF window: ell 1 pass, ell 2 = 1st order (%.6f), DC gain 1 (%.4f), ell 3 hand calc %.3f (0.6)', yA, yC, yD));

%% 8) 경로 위 + 경로 방향: e = 0, e_la ≈ 0 (경로 방향 보간과 선분 방향의 차이만큼), δ_ref = atan(Lκ)
s0 = 12.3;
q = path_pose(ref, s0);
car = struct('x', q.x, 'y', q.y, 'yaw', q.psi, 'delta', 0);
g = struct('Kpp', 1, 'Kp', 0, 'Ki', 0, 'Kd', 0, 'Lf', 0.8, 'a', 0.59, 'ell', 1, 'wcur', 1);
[~, ~, d] = blended_steer(car, ref, p, g, [], 0.1);
check(abs(d.e.e_lat) < 1e-9 && abs(d.ela) < 2e-3 && abs(d.dRef - atan(L * q.kappa)) < 1e-3 && abs(d.eSteer - d.dRef) < 1e-15, ...
    sprintf('on path: e_lat %.1e m, e_la %.1e m, delta_ref %.3f deg (atan(L kappa) %.3f deg)', d.e.e_lat, d.ela, ...
    rad2deg(d.dRef), rad2deg(atan(L * q.kappa))));

%% 9) PP만 (K_pp 1, PID 0, 필터 없음): δ = atan(2 L sin α / L_ad) (Eq.3)
car = struct('x', q.x + 0.1, 'y', q.y - 0.05, 'yaw', q.psi + 0.1, 'delta', 0.05);
[dCmd, ~, d] = blended_steer(car, ref, p, g, [], 0.1);
e = path_error(ref, car.x, car.y, car.yaw);
ql = path_pose(ref, e.s + 0.8);
al = atan2(ql.y - car.y, ql.x - car.x) - car.yaw; al = atan2(sin(al), cos(al));
LadH = hypot(ql.x - car.x, ql.y - car.y);
dPPhand = atan(2 * L * sin(al) / LadH);
check(abs(dCmd - dPPhand) < 1e-12 && abs(d.dPP - dPPhand) < 1e-12, ...
    sprintf('PP only: delta %.4f rad = atan(2L sin(alpha)/L_ad) %.4f rad', dCmd, dPPhand));

%% 10) PID (Eq.4–7 + D): δ_p = K_p e_la, δ_i = K_i Σ e, δ_d = K_d (e[k] - e[k-1]) / T_s (3 스텝 손 계산)
g = struct('Kpp', 0, 'Kp', 0.3, 'Ki', 0.02, 'Kd', 0.01, 'Lf', 0.8, 'a', 0.59, 'ell', 1, 'wcur', 1);
memP = []; dCmds = zeros(3, 1); dDs = zeros(3, 1);
cars = [car, car, car]; cars(2).x = car.x - 0.03; cars(3).yaw = car.yaw - 0.05; cars(3).y = car.y + 0.02;
hand = zeros(3, 1); Sh = 0; ePrevH = NaN;
for k = 1:3
    [dCmds(k), memP, d] = blended_steer(cars(k), ref, p, g, memP, 0.1);
    dDs(k) = d.dD;
    ek = path_error(ref, cars(k).x, cars(k).y, cars(k).yaw);
    qk = path_pose(ref, ek.s + 0.8);
    Ladk = hypot(qk.x - cars(k).x, qk.y - cars(k).y);
    elaH = -ek.e_lat + (0.59 + Ladk) * sin(-ek.e_head);
    Sh = Sh + (-ek.e_lat);
    if isnan(ePrevH), dEH = 0; else, dEH = (-ek.e_lat - ePrevH) / 0.1; end
    ePrevH = -ek.e_lat;
    hand(k) = 0.3 * elaH + 0.02 * Sh + 0.01 * dEH;
end
check(max(abs(dCmds - hand)) < 1e-12 && dDs(1) == 0 && all(dDs(2:3) ~= 0), ...
    sprintf('PID part: 3 steps match hand calc e_la, sum(e), (e[k]-e[k-1])/Ts (max diff %.1e, delta_d %.2e %.2e)', ...
    max(abs(dCmds - hand)), dDs(2), dDs(3)));

%% 11) PP_PID: δ_raw = K_pp δ_pp + δ_p + δ_i + δ_d → LPF,  합 = 1 규칙 (K_d 포함),  비교용 PID 손 계산
prm1 = blended_params('Kp', 0.1, 'Ki', 0.02, 'Kd', 0.01);
g = struct('Kpp', prm1.Kpp, 'Kp', 0.1, 'Ki', 0.02, 'Kd', 0.01, 'Lf', 0.8, 'a', 0.59, 'ell', 3, 'wcur', 0.4);
[dCmd, memB, d] = blended_steer(car, ref, p, g, [], 0.1);
raw = prm1.Kpp * d.dPP + d.dP + d.dI + d.dD;
lpfH = 0.4 * raw + 0.3 * (car.delta + car.delta);       % 이전 출력 2개 = 출발 조향각
gP = struct('Kp', 3, 'Ki', 0.8, 'Kd', 2.6, 'ell', 1, 'wcur', 1);
[dPid1, mP] = pid_steer(car, ref, p, gP, [], 0.1);
car2 = car; car2.y = car.y + 0.02;
dPid2 = pid_steer(car2, ref, p, gP, mP, 0.1);
e1 = -path_error(ref, car.x, car.y, car.yaw).e_lat; e2 = -path_error(ref, car2.x, car2.y, car2.yaw).e_lat;
pidH = 3 * e2 + 0.8 * (e1 + e2) * 0.1 + 2.6 * (e2 - e1) / 0.1;
threwK = false; try, blended_params('Kp', 0.7, 'Ki', 0.2, 'Kd', 0.2); catch, threwK = true; end
check(abs(prm1.Kpp - 0.87) < 1e-15 && abs(d.dRaw - raw) < 1e-15 && abs(dCmd - lpfH) < 1e-15 && memB.hist(1) == dCmd && ...
      abs(dPid2 - pidH) < 1e-12 && abs(dPid1 - (3 * e1 + 0.8 * e1 * 0.1)) < 1e-12 && threwK, ...
    sprintf('PP_PID: Kpp = 1 - Kp - Ki - Kd = %.2f, raw = Kpp dPP + dP + dI + dD, LPF %.4f; PID baseline hand calc; Kpp < 0 -> error', prm1.Kpp, dCmd));

%% 12) 자동 버전 오프라인 한 바퀴
r = blended_discrete('input', 'auto', 'useUnity', false, 'showFigure', false, 'realtime', false); r12 = r;
M = r.metrics;
done = r.steps * 0.1 * 0.8 >= 0.95 * ref.L;
files = isfile(fullfile(r.folder, 'timeseries.csv')) && isfile(fullfile(r.folder, 'summary.txt')) && ...
        isfile(fullfile(r.folder, 'result.png'));
better = M(3).mse_steer < min(M(1).mse_steer, M(2).mse_steer) && M(3).mse_traj < min(M(1).mse_traj, M(2).mse_traj);
check(done && files && better && all([M.max_traj] < 0.5), ...
    sprintf('auto lap: %d steps, traj RMS PP %.4f / PID %.4f / PP_PID %.4f m, steer RMS %.2f / %.2f / %.2f deg', ...
    r.steps, M(1).rms_traj, M(2).rms_traj, M(3).rms_traj, M(1).rms_steer_deg, M(2).rms_steer_deg, M(3).rms_steer_deg));

%% 13) 키보드 + Unity 없음 → 오류
threw = false;
try
    blended_discrete('input', 'keyboard', 'useUnity', false, 'showFigure', false);
catch
    threw = true;
end
check(threw, 'keyboard without Unity -> error');

%% 14) PP_PID = 매 스텝 합 (switching 아님). 12)의 한 바퀴 기록으로 확인
Tt = r12.timeseries;
ppPart  = Tt.p_Kpp .* Tt.bl_delta_pp_rad;
pidPart = Tt.bl_delta_p_rad + Tt.bl_delta_i_rad + Tt.bl_delta_d_rad;   % P + I + D
sumErr  = max(abs(Tt.bl_delta_raw_rad - (ppPart + pidPart)));
both    = mean(abs(ppPart) > 1e-6 & abs(pidPart) > 1e-6);
% switching이라면 어떤 스텝에서 출력 = 한 항만 (다른 항이 0이 아닌데도) 이 됨
onlyOne = sum((abs(pidPart) > 1e-6 & abs(Tt.bl_delta_raw_rad - ppPart) < 1e-12) | ...
              (abs(ppPart) > 1e-6 & abs(Tt.bl_delta_raw_rad - pidPart) < 1e-12));
share = abs(ppPart) ./ (abs(ppPart) + abs(pidPart));
check(sumErr < 1e-12 && onlyOne == 0 && both > 0.99, ...
    sprintf('blend, not switch: raw = K_pp dPP + (dP + dI + dD) every step (max err %.1e), both active %.1f%% of steps, PP share mean %.2f', ...
    sumErr, 100 * both, mean(share, 'omitnan')));

%% 15) 튜닝 파라미터: 덮어쓰기, 모르는 이름 오류, 실행에 반영
pr = blended_params('Kp', 0.3, 'Ki', 0.006, 'Kd', 0.001, 'v', 1.0, 'Lf', 1.2);   % Kpp = 1 - 0.3 - 0.006 - 0.001 = 0.693
threw = false;
try, blended_params('Kq', 1); catch, threw = true; end
r15 = blended_discrete('input', 'auto', 'useUnity', false, 'showFigure', false, 'save', false, 'realtime', false, ...
    'v', 1.0, 'Lf', 1.2, 'Kp', 0.3, 'Ki', 0.004, 'Kd', 0.002, 'a', 0.3, 'ell', 4, 'wcur', 0.6);
T15 = r15.timeseries;
applied = all(T15.p_v == 1.0) && all(T15.p_Lf == 1.2) && all(T15.p_Kp == 0.3) && all(T15.p_Ki == 0.004) && ...
          all(T15.p_a == 0.3) && all(T15.p_ell == 4) && all(T15.p_wcur == 0.6) && all(T15.p_Kd == 0.002) && ...
          all(abs(T15.p_Kpp - 0.694) < 1e-15) && abs(max(T15.v_mps) - 1.0) < 1e-12;
check(pr.Kp == 0.3 && pr.v == 1.0 && pr.Lf == 1.2 && pr.Kd == 0.001 && abs(pr.Kpp - 0.693) < 1e-15 && threw && applied && r15.lapDone, ...
    sprintf('params: override ok, unknown name -> error, full lap (%d steps) used v %.2f m/s, Lf %.2f, Kp %.2f', ...
    r15.steps, T15.p_v(1), T15.p_Lf(1), T15.p_Kp(1)));

%% 16) 실행 중 튜닝: check_live_tuning (빠른 실행, 한 바퀴, 7개 파라미터를 차례로 바꿈)
figsBefore = findobj(groot, 'Type', 'figure');         % 1)의 Unity 그림 등 이미 열린 그림은 건드리지 않음
chk = check_live_tuning('realtime', false, 'save', false);
figsNew = findobj(groot, 'Type', 'figure');
close(figsNew(~ismember(figsNew, figsBefore)));
check(chk.pass, sprintf('live tune: 8 params (+ Kpp) changed during a full lap (%d steps), all used from the next step; replay err PP %.1e / PID %.1e / PP_PID %.1e', ...
    chk.result.steps, chk.replay_err));

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

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
