function result = switch_discrete(varargin)
% PID ↔ PP_PID 전환 제어 (카메라 차선 인식으로 전환) + PID만 / PP_PID만과 비교
%   기본 주행은 PID. Car_Camera 사진을 YOLOv8-seg로 보고 (lane_yolo/lane_server.py)
%     보이는 차선이 모두 곡선이면  → PP_PID
%     보이는 차선이 하나라도 직선이면 (또는 하나도 안 보이면) → PID
%   세 차를 같은 출발점·같은 가속도로 나란히 계산 (blended_discrete.m과 같은 이산 모델, Ts = 0.1 s):
%     PID      : pid_steer.m       (blended_params의 pidKp, pidKi, pidKd)
%     PP_PID   : blended_steer.m   (blended_params의 Kpp, Kp, Ki, Kd, Lf, a, ell, wcur)
%     Switching: 매 스텝 두 제어기를 모두 이 차의 상태로 계산하고, 판단에 따라 한쪽 출력만 씀
%                (안 쓰는 쪽의 적분·필터도 계속 갱신 → 전환하는 순간 조향이 튀지 않음)
%   Unity에는 Switching 차를 보냄 (카메라가 이 차의 시점을 봐야 하므로)
%
% 한 스텝 (opt.decision = "camera", 기본):
%   MATLAB이 상태 x[k]를 Unity로 보냄 → Unity가 Car_Camera 사진 k를 찍어 lane_server로 보냄
%   → lane_server가 "LANE,k,PP_PID|PID,..."를 Unity로 답장 → Unity가 MATLAB으로 전달
%   → MATLAB은 LANE,k가 올 때까지 기다린 뒤 (최대 opt.waitTimeout s) 그 판단으로 u[k]를 계산
%   시간 안에 오지 않으면 직전 판단을 유지하고 timeseries의 sw_timeout = 1로 기록
%
% 실행 순서 (camera):
%   1) Unity Play  (Car_Camera의 Car Camera Capture: Period 0.1 = Ts, Send To Lane Server 켜짐)
%   2) lane_server: 꺼져 있으면 MATLAB이 새 창으로 자동으로 켬 (ensure_lane_server.m, 첫 준비 약 20 s)
%      직접 켜도 됨: C:\Users\haepi\venvs\kiis_yolo\Scripts\python.exe lane_yolo\lane_server.py
%   3) MATLAB:  cd MATLAB; setup_paths; addpath('switching'); switch_discrete
%
% 사용 예:
%   switch_discrete                                         % 카메라 판단, 0.8 m/s 한 바퀴
%   switch_discrete('v', 1.0)                               % 튜닝 파라미터는 blended_params 이름 그대로
%   switch_discrete('decision', 'ref', 'useUnity', false, 'realtime', false)
%       % 카메라 없이: 레퍼런스 경로 곡률로 판단 (앞 1~5 m의 |평균 κ| > 0.1이면 곡선) — 로직 확인·비교용
% 결과: MATLAB/results/<날짜_시간>_switching_<decision>/  timeseries.csv, summary.txt, result.png
%   그림: 맵(세 궤적), ① 궤적 오차 (0 - e_lat), ② 조향각 명령 (제어 입력), ③ Switching 차가 쓴 제어기

%% 0. 설정
opt.decision = "camera";        % "camera": lane_server 판단 / "ref": 레퍼런스 경로 곡률로 판단
opt.useUnity = true;
opt.realtime = true;            % 스텝 간격을 실제 Ts에 맞춤 (camera는 판단을 기다리므로 Ts보다 길어질 수 있음)
opt.showFigure = true;
opt.save = true;
opt.tMax = 120;
opt.laps = 1;
opt.waitTimeout = 0.5;          % LANE,k를 기다리는 최대 시간 [s]
opt.firstWaitTimeout = 5;       % 처음 5스텝은 더 기다림 (MATLAB 첫 실행 지연으로 2026-11-15에 k = 1, 2, 4가 시간 초과)
opt.refKappaTh = 0.1;           % "ref" 판단: 앞 구간 |평균 κ_ref| > 이 값이면 곡선
opt.refAhead = [1 5];           % "ref" 판단: 차 앞 구간 [m] (lane_server의 zmin~zmax와 같게)
opt.autoLaneServer = true;      % camera: lane_server가 꺼져 있으면 새 창으로 켜고 준비될 때까지 기다림 (ensure_lane_server.m)
here = fileparts(mfilename('fullpath'));
base = fileparts(here);
if ~exist('vehicle_params', 'file'), addpath(base, '-end'); end
addpath(fullfile(base, 'blended_control'));     % blended_steer, pid_steer, bicycle_discrete_step, blended_params ...
prmArgs = {};
for i = 1:2:numel(varargin)
    name = char(varargin{i});
    if isfield(opt, name), opt.(name) = varargin{i + 1}; else, prmArgs(end + 1:end + 2) = {name, varargin{i + 1}}; end %#ok<AGROW>
end
opt.decision = string(opt.decision);
if opt.decision == "camera" && ~opt.useUnity
    error("카메라 판단은 Unity가 필요합니다. Unity 없이 하려면 'decision', 'ref'를 쓰세요.");
end
if opt.decision == "camera" && opt.autoLaneServer
    % Unity에 첫 상태를 보내기 전에 lane_server와 Unity 연결까지 준비 (사진 k = 0부터 판단이 오게)
    if ~exist('ensure_lane_server', 'file'), addpath(base, '-end'); end   % MATLAB 폴더
    ensure_lane_server();
end
prm = blended_params(prmArgs{:});

p = vehicle_params();
ref = load_reference_path();
L = p.L; Ts = prm.Ts;
N = round(opt.tMax / Ts);

%% 1. 제어기 게인 (blended_discrete.m의 makeGains와 같은 값)
gPID = struct('Kp', prm.pidKp, 'Ki', prm.pidKi, 'Kd', prm.pidKd, 'ell', 1, 'wcur', 1);
gBL  = struct('Kpp', prm.Kpp, 'Kp', prm.Kp, 'Ki', prm.Ki, 'Kd', prm.Kd, 'Lf', prm.Lf, 'a', prm.a, ...
              'ell', prm.ell, 'wcur', prm.wcur);
names = ["PID", "PP_PID", "Switching"]; tex = ["PID", "PP\_PID", "Switching"]; tags = ["pid", "pppid", "sw"];
nC = 3; iSw = 3;
col = [0.00 0.40 0.95; 0.85 0.10 0.10; 0.10 0.10 0.10];

%% 2. 출발 (blended_discrete.m과 같음)
e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
q0 = path_pose(ref, e0.s);
st0 = struct('t', 0, 'x', q0.x, 'y', q0.y, 'yaw', q0.psi, 's', 0, 'v', 0, 'delta', atan(L * q0.kappa), 'yawRate', 0);
car = repmat(st0, 1, nC);
memPID = cell(1, nC); memBL = cell(1, nC);       % 차마다 제어기 내부 상태 (Switching 차는 둘 다)

X = nan(N + 1, nC); Y = X; YAW = X; DEL = X;
X(1, :) = st0.x; Y(1, :) = st0.y; YAW(1, :) = st0.yaw; DEL(1, :) = st0.delta;
DCMD = nan(N, nC); ETRAJ = DCMD; DREF = DCMD; V = nan(N, 1); A = V;
MODE = strings(N, 1); DEC = MODE; TIMEOUT = zeros(N, 1); WAITMS = nan(N, 1);
LANE = strings(N, 1);                            % 받은 LANE 줄 그대로 (차선별 상태·곡률)
REFK = nan(N, 1);
mode = "PID";                                    % 기본 주행은 PID

%% 3. Unity 연결
client = []; lost = false; nSent = 0;
if opt.useUnity
    client = unity_link('connect', p);
    if isempty(client), error("Unity에 연결할 수 없습니다. Unity에서 Play를 먼저 누르세요."); end
    guard = onCleanup(@() releaseUnity(client));
    if client.NumBytesAvailable > 0, read(client, client.NumBytesAvailable); end
    unity_link('reset', client); unity_link('path', client, ref, 0.05);
    nSent = nSent + sendCar(st0);
    fprintf('Unity 연결됨: Switching 차를 보냅니다 (판단: %s)\n', opt.decision);
end

%% 4. 그림
if opt.showFigure, vis = 'on'; else, vis = 'off'; end
fig = figure('Name', 'PID / PP_PID / Switching', 'Color', 'w', 'Position', [60 60 1500 820], 'Visible', vis);
axM = subplot(3, 2, [1 3 5]); draw_map(axM, p); hold(axM, 'on'); axis(axM, 'equal');
hRef = plot(axM, ref.x, ref.y, '-', 'Color', [0.95 0.78 0], 'LineWidth', 1.5);
hTr = gobjects(1, nC);
for c = 1:nC, hTr(c) = plot(axM, st0.x, st0.y, '-', 'Color', col(c, :), 'LineWidth', 1.2); end
hPP = plot(axM, NaN, NaN, '.', 'Color', [0.6 0 0.8], 'MarkerSize', 6);   % Switching 차가 PP_PID를 쓴 지점
hCar = plot(axM, st0.x, st0.y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 7);
legend(axM, [hRef, hTr, hPP], ["레퍼런스 경로", tex, "Switching이 PP\_PID를 쓴 구간"], 'Location', 'southoutside', 'NumColumns', 3);
xlabel(axM, 'x [m]'); ylabel(axM, 'y [m]');
xlim(axM, p.mapHalfX * [-1 1]); ylim(axM, p.mapHalfY * [-1 1]);
hTitle = title(axM, '', 'Interpreter', 'none');
axE = subplot(3, 2, 2); hold(axE, 'on'); grid(axE, 'on');
axD = subplot(3, 2, 4); hold(axD, 'on'); grid(axD, 'on');
axS = subplot(3, 2, 6); hold(axS, 'on'); grid(axS, 'on');
hE = gobjects(1, nC); hD = gobjects(1, nC);
for c = 1:nC
    hE(c) = plot(axE, NaN, NaN, '-', 'Color', col(c, :), 'LineWidth', 1.1);
    hD(c) = plot(axD, NaN, NaN, '-', 'Color', col(c, :), 'LineWidth', 1.1);
end
hDref = plot(axD, NaN, NaN, '--', 'Color', [0.95 0.78 0], 'LineWidth', 1.0);
hS = stairs(axS, NaN, NaN, '-', 'Color', [0.6 0 0.8], 'LineWidth', 1.5);
title(axE, 'Trajectory tracking error: 레퍼런스 경로 - 주행 경로 (0 - e_{lat})'); ylabel(axE, '[m]');
title(axD, '제어 입력: 조향각 명령 \delta_{cmd}'); ylabel(axD, '[deg]');
title(axS, 'Switching 차가 쓴 제어기'); ylabel(axS, ''); xlabel(axS, 't = kT_s [s]');
yticks(axS, [0 1]); yticklabels(axS, ["PID", "PP\_PID"]); ylim(axS, [-0.3 1.3]);
hLegE = legend(axE, tex, 'Location', 'best'); hLegD = legend(axD, [tex, "목표 \delta_{ref} = atan(L\kappa)"], 'Location', 'best');

%% 5. 메인 루프
% 예열: 루프 안에서 처음 부르면 느린 함수들을 미리 한 번 부름 (첫 스텝 판단 대기가 길어지지 않게)
pause(0.01); path_error(ref, st0.x, st0.y, st0.yaw, NaN); path_pose(ref, 0);
n = 0; tLoop = tic; tNext = 0; tDraw = -inf; nTimeout = 0;
for k = 1:N
    % (1) 판단: 지금 상태 x[k-1] (= Unity로 마지막에 보낸 상태)의 사진 판단
    if opt.decision == "camera"
        [dec, line, waited, ok] = waitLane(k - 1);
        WAITMS(k) = waited * 1000; LANE(k) = line;
        if ok, mode = dec; else, TIMEOUT(k) = 1; nTimeout = nTimeout + 1; end
        if lost, break; end
    else
        eS = path_error(ref, car(iSw).x, car(iSw).y, car(iSw).yaw, NaN);
        REFK(k) = abs(mean(path_pose(ref, ref.s(eS.idx) + linspace(opt.refAhead(1), opt.refAhead(2), 41)).kappa));
        if REFK(k) > opt.refKappaTh, mode = "PP_PID"; else, mode = "PID"; end
    end
    DEC(k) = mode; MODE(k) = mode;

    % (2) 가속도 (세 차 공통, blended_discrete 자동 버전과 같음)
    aReq = (prm.v - car(iSw).v) / Ts;

    % (3) 세 차: 조향각 → omega → 이산 모델
    for c = 1:nC
        if c ~= 2, [dP, memPID{c}, d1] = pid_steer(car(c), ref, p, gPID, memPID{c}, Ts); end
        if c ~= 1, [dB, memBL{c}, d2] = blended_steer(car(c), ref, p, gBL, memBL{c}, Ts); end
        switch c
            case 1, dCmd = dP; d = d1;
            case 2, dCmd = dB; d = d2;
            otherwise
                if mode == "PP_PID", dCmd = dB; d = d2; else, dCmd = dP; d = d1; end
        end
        omega = (dCmd - car(c).delta) / Ts;
        car(c) = bicycle_discrete_step(car(c), [aReq; omega], p, Ts);
        DCMD(k, c) = dCmd; ETRAJ(k, c) = d.eLat; DREF(k, c) = d.dRef;
        X(k + 1, c) = car(c).x; Y(k + 1, c) = car(c).y; YAW(k + 1, c) = car(c).yaw; DEL(k + 1, c) = car(c).delta;
    end
    V(k) = car(iSw).v; A(k) = aReq; n = k;

    % (4) Unity로 Switching 차 전송
    if ~isempty(client)
        if sendCar(car(iSw)), nSent = nSent + 1; else
            lost = true; fprintf('Unity로 보내지 못했습니다 (k = %d). 지금까지 결과를 저장합니다.\n', k); break;
        end
    end

    % (5) 그림 (0.3 s마다)
    if opt.showFigure && isvalid(fig) && toc(tLoop) - tDraw >= 0.3
        updatePlots(k); drawnow; tDraw = toc(tLoop);
    end

    % (6) 실시간 간격
    if opt.realtime
        tNext = tNext + Ts;
        w = tNext - toc(tLoop);
        if w > 0, pause(w); else, tNext = toc(tLoop); end
    end
    if min([car.s]) >= opt.laps * ref.L, break; end
end
loopTime = toc(tLoop);
if ~isempty(client), delete(guard); end

%% 6. 지표
M = struct('name', cellstr(names));
for c = 1:nC
    et = ETRAJ(1:n, c);
    M(c).mse_traj = mean(et .^ 2); M(c).rms_traj = sqrt(M(c).mse_traj); M(c).max_traj = max(abs(et));
    M(c).rms_dcmd_deg = rad2deg(sqrt(mean(DCMD(1:n, c) .^ 2)));
    M(c).rms_ddelta_degps = rad2deg(sqrt(mean((diff([st0.delta; DEL(2:n + 1, c)]) / Ts) .^ 2)));   % 조향 속도 RMS
end
sw = MODE(1:n);
nSwitch = sum(sw(2:end) ~= sw(1:end - 1));
ppShare = mean(sw == "PP_PID");

%% 7. 데이터 저장 (그림보다 먼저)
folder = "";
if opt.save
    folder = fullfile(base, 'results', string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_switching_" + opt.decision);
    mkdir(folder);
    T = table((0:n - 1)', (0:n - 1)' * Ts, V(1:n), A(1:n), 'VariableNames', {'k', 't_s', 'v_mps', 'a_mps2'});
    for c = 1:nC
        T.(tags(c) + "_x_m") = X(1:n, c); T.(tags(c) + "_y_m") = Y(1:n, c); T.(tags(c) + "_yaw_rad") = YAW(1:n, c);
        T.(tags(c) + "_delta_rad") = DEL(1:n, c); T.(tags(c) + "_delta_cmd_rad") = DCMD(1:n, c);
        T.(tags(c) + "_delta_ref_rad") = DREF(1:n, c); T.(tags(c) + "_e_traj_m") = ETRAJ(1:n, c);
    end
    T.sw_mode = sw; T.sw_timeout = TIMEOUT(1:n); T.sw_wait_ms = WAITMS(1:n);
    T.sw_ref_kappa_ahead = REFK(1:n); T.sw_lane = LANE(1:n);
    writetable(T, fullfile(folder, 'timeseries.csv'));
    writeSummary(fullfile(folder, 'summary.txt'));
end

%% 8. 그림 마무리 + result.png (실패해도 데이터는 위에서 저장됨)
if isvalid(fig) && n > 0
    try
        updatePlots(n);
        hLegE.String = arrayfun(@(c) sprintf('%s: RMS %.4f m, 최대 %.3f m', tex(c), M(c).rms_traj, M(c).max_traj), 1:nC, 'UniformOutput', false);
        hLegD.String = [arrayfun(@(c) sprintf('%s: RMS %.1f deg', tex(c), M(c).rms_dcmd_deg), 1:nC, 'UniformOutput', false), {'목표 \delta_{ref}'}];
        if opt.save
            exportgraphics(fig, fullfile(folder, 'result.png'), 'Resolution', 110);
            exportgraphics(fig, fullfile(folder, 'result.svg'), 'ContentType', 'vector');   % 벡터 그림
        end
    catch ME
        warning('switch_discrete:figure', '그림(result.png)을 저장하지 못했습니다: %s (데이터는 저장됨)', ME.message);
    end
end
if ~opt.showFigure && isvalid(fig), close(fig); end

result = struct('folder', folder, 'steps', n, 'opt', opt, 'prm', prm, 'metrics', M, 'switches', nSwitch, ...
    'ppShare', ppShare, 'timeouts', nTimeout, 'lapDone', min([car.s]) >= ref.L, 'wallTime', loopTime, ...
    'unity', struct('used', ~isempty(client), 'sent', nSent, 'lost', lost));
fprintf('Switching (%s, v %.2f m/s, Ts %.2f s): %d 스텝, 전환 %d번, PP_PID %.0f%%, 판단 시간 초과 %d번\n', ...
    opt.decision, prm.v, Ts, n, nSwitch, 100 * ppShare, nTimeout);
for c = 1:nC
    fprintf('  %-9s 궤적 오차 MSE %.2e m^2 (RMS %.4f m, 최대 %.3f m)   조향각 명령 RMS %.1f deg\n', ...
        names(c), M(c).mse_traj, M(c).rms_traj, M(c).max_traj, M(c).rms_dcmd_deg);
end
if opt.save, fprintf('결과: %s\n', folder); end

    % ---------- 내부 함수 ----------
    function ok = sendCar(st)
        e = path_error(ref, st.x, st.y, st.yaw, NaN);
        dbg = struct('x_w', e.x_proj, 'y_w', e.y_proj, 'x_d', NaN, 'y_d', NaN, 'x_l', NaN, 'y_l', NaN, ...
            'x_c', NaN, 'y_c', NaN, 'R', NaN, 'Lf', prm.Lf, 'tau', NaN, 'delta_cmd', NaN);
        ok = unity_link('state', client, st, e, dbg);
    end

    function [dec, line, waited, ok] = waitLane(kWant)
        % Unity가 전달하는 "LANE,k,..." 중 k = kWant인 줄을 기다림 (더 오래된 줄은 버림)
        dec = mode; line = ""; ok = false; t0 = tic;
        limit = opt.waitTimeout;
        if kWant < 5, limit = max(limit, opt.firstWaitTimeout); end
        while toc(t0) < limit
            if client.NumBytesAvailable > 0
                s = strtrim(string(readline(client)));
                if startsWith(s, "LANE,")
                    f = split(s, ',');
                    if str2double(f(2)) == kWant
                        dec = f(3); line = s; ok = true; break;
                    end
                end
            else
                pause(0.001);
            end
        end
        waited = toc(t0);
    end

    function updatePlots(m)
        tk = (0:m - 1)' * Ts;
        for cc = 1:nC
            set(hTr(cc), 'XData', X(1:m + 1, cc), 'YData', Y(1:m + 1, cc));
            set(hE(cc), 'XData', tk, 'YData', ETRAJ(1:m, cc));
            set(hD(cc), 'XData', tk, 'YData', rad2deg(DCMD(1:m, cc)));
        end
        set(hDref, 'XData', tk, 'YData', rad2deg(DREF(1:m, iSw)));
        pp = MODE(1:m) == "PP_PID";
        set(hPP, 'XData', X([false; pp], iSw), 'YData', Y([false; pp], iSw));
        set(hS, 'XData', tk, 'YData', double(pp));
        set(hCar, 'XData', X(m + 1, iSw), 'YData', Y(m + 1, iSw));
        hTitle.String = sprintf('k = %d, t = %.1f s, Switching: %s', m, (m - 1) * Ts, MODE(m));
    end

    function writeSummary(file)
        fid = fopen(file, 'w', 'n', 'UTF-8');
        fprintf(fid, 'PID / PP_PID / Switching (switch_discrete) - 판단 %s, %s\n', opt.decision, string(datetime('now')));
        fprintf(fid, '규칙: 보이는 차선이 모두 곡선이면 PP_PID, 하나라도 직선이거나 안 보이면 PID (기본 PID)\n');
        fprintf(fid, '모델: bicycle_discrete_step, Ts = %.3f s, L = %.2f m, v = %.2f m/s\n', Ts, L, prm.v);
        fprintf(fid, 'PID: Kp %.3g, Ki %.3g, Kd %.3g / PP_PID: Kpp %.4g, Kp %.4g, Ki %.4g, Kd %.4g, Lf %.2f, a %.2f, ell %d, w_cur %.2f\n', ...
            prm.pidKp, prm.pidKi, prm.pidKd, prm.Kpp, prm.Kp, prm.Ki, prm.Kd, prm.Lf, prm.a, prm.ell, prm.wcur);
        if opt.decision == "ref", fprintf(fid, 'ref 판단: 앞 %.1f~%.1f m |평균 kappa_ref| > %.3f\n', opt.refAhead, opt.refKappaTh); end
        fprintf(fid, '스텝 %d (%.1f s), 전환 %d번, PP_PID 구간 %.1f%%, 판단 시간 초과 %d번, 판단 대기 평균 %.1f ms\n', ...
            n, n * Ts, nSwitch, 100 * ppShare, nTimeout, mean(WAITMS(1:n), 'omitnan'));
        fprintf(fid, '\n%-10s | %-12s %-10s %-10s | %s\n', '제어기', 'MSE[m^2]', 'RMS[m]', '최대[m]', '조향각 명령 RMS [deg] / 조향 속도 RMS [deg/s]');
        for cc = 1:nC
            fprintf(fid, '%-10s | %-12.3e %-10.4f %-10.4f | %.2f / %.2f\n', names(cc), M(cc).mse_traj, M(cc).rms_traj, ...
                M(cc).max_traj, M(cc).rms_dcmd_deg, M(cc).rms_ddelta_degps);
        end
        fclose(fid);
    end
end

function releaseUnity(client)
try
    if ~isempty(client) && isvalid(client), delete(client); end
catch
end
end
