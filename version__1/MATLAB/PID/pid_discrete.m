function result = pid_discrete(varargin)
% PID + 정해진 경로 위 이산 상태공간 모델 (test1과 같은 구조). Unity 키로 한 스텝씩 진행
%   모델 (test1의 path_model_step, path_pose):
%     상태 x[t] = [s; v; delta],  입력 u[t] = [a; omega]
%     x[t+1] = A x[t] + B u[t]      전진(s, v): 더블 인테그레이터 / 조향(delta): 인테그레이터
%     차 위치·방향 = 경로(s[t]) 위의 점과 접선 (경로에서 벗어나지 않음)
%   기준과 오차:
%     delta_ref[t] = atan( L kappa(s[t]) )           (그 위치에서 경로가 요구하는 조향각)
%     e[t]         = delta_ref[t] - delta[t]         (reference 조향각 - 현재 조향각)
%     e_cmd[t]     = delta_cmd[t] - delta[t]         (제어기가 낸 조향각 - 실제 조향각, 그래프·기록용)
%   제어 (PID, 필터 없음):
%     delta_cmd[t] = Kp e[t] + Ki Σ_{j=1..t} e[j] Ts + Kd (e[t] - e[t-1]) / Ts
%     omega[t]     = (delta_cmd[t] - delta[t]) / Ts   (조향 인테그레이터가 명령을 따라감, 90°/s 제한)
%   가속도 a[t]:
%     키보드 버전: 스텝 키와 함께 ↑ +1.0, ↓ -2.0, 없으면 0 (속도 유지)
%     자동 버전  : 목표 속도 v로 a = (v - v[t]) / Ts (한계는 모델이 자름)
%
% 두 버전 ('input'):
%   "keyboard" (기본): Unity Game 창에서 Space 또는 → 한 번 = 한 스텝 (t → t+1), Q = 종료
%   "auto"           : 키 없이 스스로 한 바퀴
%
% 사용 예 (MATLAB 폴더에서 setup_paths 후, Unity Play 먼저):
%   pid_discrete                                      % 키보드: ↑를 누른 채 Space로 출발
%   pid_discrete('input', 'auto')
%   pid_discrete('input', 'auto', 'useUnity', false, 'Kp', 0.3)
% 결과: results/<날짜_시간>_pid_discrete_<input>/ 에 timeseries.csv, error.png

%% 0. 설정
opt.input = "keyboard"; opt.Kp = 0.2; opt.Ki = 5.0; opt.Kd = 0.01;
opt.v = 0.8; opt.Ts = 0.1; opt.tMax = 600; opt.useUnity = true;
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i + 1}; end
opt.input = string(opt.input);
here = fileparts(mfilename('fullpath'));
base = fileparts(here);
if ~exist('vehicle_params', 'file'), addpath(base); end
if ~exist('path_model_step', 'file'), addpath(fullfile(base, 'test1')); end

p = vehicle_params();
if isfield(opt, 'port'), p.unityPort = opt.port; end   % 통신 테스트용
ref = load_reference_path();
L   = p.L;                         % 축간거리 (vehicle_params, 0.56 m)
Ts  = opt.Ts;                      % 한 스텝 = 0.1 s
N   = round(opt.tMax / Ts);
if opt.input == "keyboard" && ~opt.useUnity
    error("키보드 버전은 Unity에서 키를 받으므로 'useUnity', true 가 필요합니다.");
end

%% 1. 배열 준비 (t = 1 이 시각 0). 출발: Unity 차 위치를 경로에 투영한 점, 정지
s = zeros(N + 1, 1); v = zeros(N + 1, 1); delta = zeros(N + 1, 1);
a = nan(N, 1); omega = nan(N, 1); dCmd = nan(N, 1); dRef = nan(N, 1); e = nan(N, 1); eCmd = nan(N, 1);
pTerm = nan(N, 1); iTerm = nan(N, 1); dTerm = nan(N, 1);
e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
s(1) = e0.s;
delta(1) = atan(L * path_pose(ref, s(1)).kappa);

client = [];
if opt.useUnity
    client = unity_link('connect', p);
    if isempty(client), error("Unity에 연결할 수 없습니다. Unity에서 Play를 먼저 누르세요."); end
    if client.NumBytesAvailable > 0, read(client, client.NumBytesAvailable); end   % 이전 키 버림
    unity_link('reset', client); unity_link('path', client, ref, 0.05);
    q = path_pose(ref, s(1));
    unity_link('state', client, struct('t', 0, 'x', q.x, 'y', q.y, 'yaw', q.psi, 'v', 0, 'delta', delta(1), 'yawRate', 0));
end
if opt.input == "keyboard"
    fprintf('Unity Game 창 클릭 → ↑를 누른 채 Space로 출발, Space 한 번 = 한 스텝, Q 종료\n');
end

%% 화면: 왼쪽 맵·경로·지나온 점, 오른쪽 위 오차 e = delta_ref - delta, 오른쪽 아래 조향각
fig = figure('Name', "PID (" + opt.input + ") - 오차 = delta_ref - delta, delta_cmd - delta", 'NumberTitle', 'off', 'Position', [60 60 1450 780]);
ax = axes(fig, 'Position', [0.04 0.08 0.58 0.84]);
draw_map(ax, p);
plot(ax, ref.x, ref.y, 'y-', 'LineWidth', 1.5);
q = path_pose(ref, s(1));
hTraj  = plot(ax, q.x, q.y, 'r.', 'MarkerSize', 8);
hCar   = plot(ax, q.x, q.y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 7);
hTitle = title(ax, '');
axE = axes(fig, 'Position', [0.68 0.56 0.30 0.36]); hold(axE, 'on'); grid(axE, 'on');
yline(axE, 0, 'k-');
hErr  = plot(axE, NaN, NaN, 'b.-');
hErr2 = plot(axE, NaN, NaN, 'r.-');
hLegE = legend(axE, [hErr, hErr2], {'추종 오차 \delta_{ref} - \delta', '명령 오차 \delta_{cmd} - \delta'}, 'Location', 'best');
xlabel(axE, 't = kT_s [s]'); ylabel(axE, '오차 [deg]'); title(axE, '조향각 오차 (같은 시각 t)');
axD = axes(fig, 'Position', [0.68 0.08 0.30 0.36]); hold(axD, 'on'); grid(axD, 'on');
hRef = plot(axD, NaN, NaN, 'k-', 'LineWidth', 1.2);
hCmd = plot(axD, NaN, NaN, 'r--');
hDel = plot(axD, NaN, NaN, 'b.-');
legend(axD, {'\delta_{ref} = atan(L\kappa)', '\delta_{cmd} = K_p e + K_I \Sigma e T_s + K_d \Delta e / T_s', '\delta (현재)'}, 'Location', 'best');
xlabel(axD, 't = kT_s [s]'); ylabel(axD, '[deg]'); title(axD, '조향각');
px = q.x; py = q.y;

%% 2. 메인 루프: t → t+1
n = 0;
S = 0;                                                        % Σ e[j] Ts
for t = 1:N
    % (0) 가속도 a[t]: 키보드면 Unity에서 STEP이 올 때까지 기다림
    if opt.input == "keyboard"
        cmd = "";
        while ~startsWith(cmd, "STEP") && ~startsWith(cmd, "QUIT")
            if client.NumBytesAvailable > 0
                cmd = strtrim(string(readline(client)));
            else
                pause(0.005);
            end
        end
        if startsWith(cmd, "QUIT"), break; end
        k = split(cmd, ',');                                  % "STEP,↑,↓"
        up = k(2) == "1"; down = k(3) == "1";
        a(t) = p.accel * (up && ~down) - p.brake * (down && ~up);
    else
        a(t) = (opt.v - v(t)) / Ts;
    end

    % (1) 지금 차 위치·방향 (경로 위)
    q = path_pose(ref, s(t));

    % (2) 기준과 오차: delta_ref[t] = atan(L kappa(s[t])), e[t] = delta_ref[t] - delta[t]
    dRef(t) = atan(L * q.kappa);
    e(t)    = dRef(t) - delta(t);

    % (3) PID: delta_cmd[t] = Kp e[t] + Ki Σ e Ts + Kd (e[t] - e[t-1]) / Ts
    S = S + e(t) * Ts;
    if t == 1
        dE = 0;                                               % 첫 스텝: e[0] = e[1]로 봄
    else
        dE = (e(t) - e(t - 1)) / Ts;
    end
    pTerm(t) = opt.Kp * e(t);
    iTerm(t) = opt.Ki * S;
    dTerm(t) = opt.Kd * dE;
    dCmd(t)  = pTerm(t) + iTerm(t) + dTerm(t);

    % (3b) 명령 오차: e_cmd[t] = delta_cmd[t] - delta[t] (제어기가 낸 조향각 - 실제 조향각)
    eCmd(t) = dCmd(t) - delta(t);

    % (4) 조향 인테그레이터 입력: omega[t] = (delta_cmd - delta) / Ts
    omega(t) = (dCmd(t) - delta(t)) / Ts;

    % (5) 이산 모델: x[t+1] = A x[t] + B u[t] (입력 한계 포함)
    [xn, used] = path_model_step([s(t); v(t); delta(t)], [a(t); omega(t)], p, Ts);
    a(t) = used(1); omega(t) = used(2);
    s(t + 1) = xn(1); v(t + 1) = xn(2); delta(t + 1) = xn(3);
    n = t;

    % (6) Unity로 전송
    qn = path_pose(ref, s(t + 1));
    if ~isempty(client)
        st = struct('t', t * Ts, 'x', qn.x, 'y', qn.y, 'yaw', qn.psi, 'v', v(t + 1), ...
            'delta', delta(t + 1), 'yawRate', v(t + 1) * qn.kappa);
        unity_link('state', client, st);
        if opt.input == "auto", pause(Ts); end
    end

    % (7) 화면 갱신
    px(end + 1) = qn.x; py(end + 1) = qn.y; %#ok<AGROW>
    if isvalid(fig) && (opt.input == "keyboard" || mod(t, 5) == 0)
        tk = (0:t - 1)' * Ts;
        set(hTraj, 'XData', px, 'YData', py);
        set(hCar,  'XData', qn.x, 'YData', qn.y);
        set(hErr,  'XData', tk, 'YData', rad2deg(e(1:t)));
        set(hErr2, 'XData', tk, 'YData', rad2deg(eCmd(1:t)));
        set(hRef,  'XData', tk, 'YData', rad2deg(dRef(1:t)));
        set(hCmd,  'XData', tk, 'YData', rad2deg(dCmd(1:t)));
        set(hDel,  'XData', tk, 'YData', rad2deg(delta(1:t)));
        hTitle.String = sprintf('k = %d   t = %.1f s   v = %.2f m/s   \\delta = %.1f deg   e = %.2f deg', ...
            t, t * Ts, v(t + 1), rad2deg(delta(t + 1)), rad2deg(e(t)));
        drawnow limitrate;
    end

    % 자동 버전: 한 바퀴 돌면 끝
    if opt.input == "auto" && s(t + 1) - s(1) >= ref.L, break; end
end

%% 3. 결과 저장, MSE
tt = (0:n - 1)' * Ts;
q = path_pose(ref, s(1:n));
T = table((0:n - 1)', tt, s(1:n), v(1:n), a(1:n), q.x, q.y, q.psi, q.kappa, ...
    dRef(1:n), dCmd(1:n), delta(1:n), omega(1:n), e(1:n), eCmd(1:n), pTerm(1:n), iTerm(1:n), dTerm(1:n), ...
    'VariableNames', {'k', 't_s', 's_m', 'v_mps', 'a_mps2', 'x_m', 'y_m', 'psi_rad', 'kappa_1pm', ...
    'delta_ref_rad', 'delta_cmd_rad', 'delta_rad', 'omega_radps', 'e_ref_rad', 'e_cmd_rad', 'p_term_rad', 'i_term_rad', 'd_term_rad'});
folder = fullfile(base, 'results', string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_pid_discrete_" + opt.input);
mkdir(folder);
writetable(T, fullfile(folder, 'timeseries.csv'));
result.folder = folder;
result.steps  = n;
result.mse    = mean(e(1:n).^2);                % [rad^2]
result.rmse_deg = rad2deg(sqrt(result.mse));
result.max_deg  = rad2deg(max(abs(e(1:n))));
result.mse_cmd      = mean(eCmd(1:n).^2);      % 명령 오차 [rad^2]
result.rmse_cmd_deg = rad2deg(sqrt(result.mse_cmd));
result.max_cmd_deg  = rad2deg(max(abs(eCmd(1:n))));
if isvalid(fig) && n > 0
    hLegE.String = {sprintf('추종 오차 \\delta_{ref} - \\delta:  MSE %.2e rad^2, RMS %.2f deg, 최대 %.2f deg', result.mse, result.rmse_deg, result.max_deg), ...
                     sprintf('명령 오차 \\delta_{cmd} - \\delta:  MSE %.2e rad^2, RMS %.2f deg, 최대 %.2f deg', result.mse_cmd, result.rmse_cmd_deg, result.max_cmd_deg)};
    exportgraphics(fig, fullfile(folder, 'error.png'), 'Resolution', 110);
end
fprintf('PID (%s, Kp %.2f, Ki %.2f, Kd %.2f, Ts %.2f s): 조향각 오차 MSE %.2e rad^2, RMS %.2f deg, 최대 %.2f deg, %d 스텝\n', ...
    opt.input, opt.Kp, opt.Ki, opt.Kd, Ts, result.mse, result.rmse_deg, result.max_deg, n);
end
