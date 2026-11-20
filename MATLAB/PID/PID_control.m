function result = PID_control(varargin)
% PID로 레퍼런스 궤적(시간별 위치·속도)을 따라 주행하고, 그 상태를 Unity로 보내며 기록하는 코드
% 차량 모델: 기구학 자전거 모델 (vehicle_params.m, bicycle_step.m — 기준점 = 뒤 차축 중앙)
%
% 입출력:
%   입력  레퍼런스 궤적: 시각 t마다 위치 (x_r, y_r), 방향 psi_r, 속도 v_r  ← make_reference_trajectory.m
%         차량 상태: x, y, yaw, v  ← bicycle_step.m
%   출력  delta_cmd (조향 PID), a_cmd (속도 PID)
%         → 차량 모델 입력 (delta_cmd, v_cmd = v + a_cmd*dt)  (pure pursuit와 같은 모델 입력)
%
% 사용 예:
%   PID_control                                      % Unity 연결, 실시간, 그림
%   PID_control('Kp', 2.5, 'Kd', 2.0)                % 게인 바꾸기 (pid_params.m 참고)
%   PID_control('useUnity', false, 'realtime', false, 'showFigure', false)   % 빠른 오프라인
%   PID_control('liveTune', true)                    % 그림의 슬라이더로 Kp 조절 (탐색용)
%
% 그림 창 조작 (창을 클릭한 상태에서)
%   Q 또는 Esc : 종료 (창을 닫아도 종료)
%   R / T / L : 경로 / 궤적 / 기준점 표시 켜기·끄기 (아래 체크박스와 같음)
%
% 실행 순서 (Unity 사용 시): Unity Play 먼저 → 이 함수 실행
%   Unity 화면: 빨간 점과 선 = 기준점 p_r과 차-기준점 선, 파란 점 = 최근접 경로점,
%              HUD의 "Lf" = 차-기준점 거리 (pure pursuit 원은 그리지 않음)
% 결과: results/<날짜_시간>_pid_<경로>/ 에 timeseries.csv, reference_trajectory.csv,
%       run.mat, summary.txt, trajectory.png
% 반환: 요약 구조체 (횡오차 평균·RMS·MSE·최대, 완주 여부, 결과 폴더 등 + e_s·e_v의 RMS·최대)

%% 0. 옵션 (여기 없는 이름은 pid_params로 넘어감)
here = fileparts(mfilename('fullpath'));     % MATLAB/PID
base = fileparts(here);                      % MATLAB (공용 함수: vehicle_params, path_error, unity_link, ...)
if ~exist('vehicle_params', 'file'), addpath(base); end
opt.useUnity   = true;      % Unity로 상태 전송
opt.realtime   = true;      % 실제 시간에 맞춰 진행 (false면 최대한 빠르게)
opt.showFigure = true;      % MATLAB 실시간 그림
opt.liveTune   = false;     % 그림에 Kp 슬라이더 표시
opt.laps       = 1;         % 몇 바퀴 돌고 끝낼지
opt.tMax       = 180;       % 최대 시간 [s]
opt.pathFile   = '';        % 비우면 reference_path_outer_lane.mat
opt.tag        = '';        % 결과 폴더 이름 뒤에 붙일 메모
opt.resultsDir = fullfile(base, 'results');
opt.plotEvery  = 5;         % 그림 갱신 간격 [스텝]
pidArgs = {};
for k = 1:2:numel(varargin)
    if isfield(opt, varargin{k})
        opt.(varargin{k}) = varargin{k + 1};
    else
        pidArgs(end + 1:end + 2) = varargin(k:k + 1);
    end
end

p   = vehicle_params();
pid = pid_params(pidArgs{:});
ref = load_reference_path(opt.pathFile);
dt  = p.dt;
tune = struct('name', 'Kp', 'min', 0.5, 'max', 8);   % 주행 중 슬라이더로 바꿀 파라미터

%% 1. 출발 상태: Unity car 위치 그대로 (정지)
st = struct('t', 0, 'x', p.startUnity(1), 'y', p.startUnity(2), ...
    'yaw', deg2rad(90 - p.startUnity(3)), 'v', 0, 'delta', 0, 'yawRate', 0);
e = path_error(ref, st.x, st.y, st.yaw);        % 처음에는 경로 전체에서 찾기

%% 2. 레퍼런스 궤적: 차 출발점에서 정지 상태로 시작해 경로를 따라 움직이는 기준점
traj = make_reference_trajectory(ref, p, pid, opt.tMax, e.s);

%% 3. Unity 연결 (기록 폴더를 만들기 전에 확인)
client = [];
if opt.useUnity
    client = unity_link('connect', p);
    if isempty(client)
        error("Unity에 연결할 수 없습니다. Unity Play를 먼저 누르거나, 'useUnity', false 로 실행하세요.");
    end
    unity_link('reset', client);
    unity_link('path', client, ref, 0.05);
end

%% 4. 기록 준비 (앞 18열은 pure_pursuit_control과 같은 이름·순서·의미)
cols = {'t_s', 'step', 'x_m', 'y_m', 'yaw_rad', 'v_mps', 'delta_rad', 'yawRate_radps', ...
        'delta_cmd_rad', 'v_cmd_mps', 'delta_sat', ...
        's_ref_m', 'x_proj_m', 'y_proj_m', 'psi_ref_rad', 'kappa_ref_1pm', 'e_lat_m', 'e_head_rad', ...
        's_r_m', 'x_r_m', 'y_r_m', 'psi_r_rad', 'v_r_mps', ...
        'e_s_m', 'e_v_mps', 'x_w_m', 'y_w_m', ...
        'e_int_ms', 'e_diff_mps', 'p_term_rad', 'i_term_rad', 'd_term_rad', ...
        'e_s_int_ms', 'a_p_mps2', 'a_i_mps2', 'a_d_mps2', 'a_cmd_mps2', 'a_sat', ...
        'comp_ms', 'progress_m', 'lap', 'tune_val'};
runName = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_pid_" + ref.name;
if strlength(string(opt.tag)) > 0, runName = runName + "_" + opt.tag; end
meta = struct('runName', runName, 'started', string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    'matlab', string(version), 'options', opt, 'tuneName', tune.name, 'traj', traj);
logger = RunLogger(fullfile(opt.resultsDir, runName), cols, ceil(opt.tMax / dt) + 10, ref, ...
    struct('vehicle', p, 'pid', pid), meta);
logger.sourceFiles = [fullfile(here, ["PID_control.m", "pid_step.m", "pid_params.m", "make_reference_trajectory.m"]), ...
    fullfile(base, ["vehicle_params.m", "bicycle_step.m", "path_error.m", "unity_link.m"])];
writetable(table(traj.t, traj.s, traj.x, traj.y, traj.psi, traj.kappa, traj.v, ...
    'VariableNames', {'t_s', 's_m', 'x_m', 'y_m', 'psi_rad', 'kappa_1pm', 'v_mps'}), ...
    fullfile(logger.folder, 'reference_trajectory.csv'));
cleanupObj = onCleanup(@() finishRun(logger));

%% 5. 그림
running = true;
G = [];
if opt.showFigure, setupFigure(); end

%% 6. 메인 루프
progress = 0; sPrev = NaN; step = 0;
tuneStart = pid.(tune.name);
mem = [];
t0 = tic;

while running
    step = step + 1;
    if step > 1, e = path_error(ref, st.x, st.y, st.yaw, e.idx); end

    % 경로를 따라 진행한 거리 (한 바퀴 지날 때 s가 0으로 돌아가는 것 보정)
    if ~isnan(sPrev)
        dS = e.s - sPrev;
        progress = progress + dS - ref.L * round(dS / ref.L);
    end
    sPrev = e.s;

    % --- 이번 시각의 레퍼런스 (시각 (step-1)*dt = st.t) ---
    kr = min(step, numel(traj.t));
    rk = struct('s', traj.s(kr), 'x', traj.x(kr), 'y', traj.y(kr), 'psi', traj.psi(kr), 'v', traj.v(kr));

    % --- 제어기: 레퍼런스 + 상태 -> (delta_cmd, v_cmd) ---
    [u, dbg, mem] = pid_step(st, e, rk, p, pid, mem, traj.s(1) + progress);

    % --- 기록 (이 스텝의 상태·오차·명령) ---
    logger.add([st.t, step, st.x, st.y, st.yaw, st.v, st.delta, st.yawRate, ...
        u.delta_cmd, u.v_cmd, dbg.sat, ...
        e.s, e.x_proj, e.y_proj, e.psi, e.kappa, e.e_lat, e.e_head, ...
        rk.s, rk.x, rk.y, rk.psi, rk.v, ...
        dbg.e_s, dbg.e_v, e.x_w, e.y_w, ...
        dbg.e_int, dbg.e_diff, dbg.p_term, dbg.i_term, dbg.d_term, ...
        dbg.e_s_int, dbg.a_p, dbg.a_i, dbg.a_d, dbg.a_cmd, dbg.a_sat, ...
        dbg.comp_ms, progress, floor(progress / ref.L), pid.(tune.name)]);

    % --- Unity로 전송 ---
    if ~isempty(client) && ~unity_link('state', client, st, e, dbg)
        warning("Unity 연결이 끊겼습니다. 종료합니다.");
        logger.reason = "Unity 연결 끊김";
        break;
    end

    % --- 그림 갱신 ---
    if ~isempty(G) && mod(step, opt.plotEvery) == 0 && isvalid(G.fig)
        updateFigure(dbg, e, rk);
    end

    % --- 끝낼 조건 ---
    if progress >= opt.laps * ref.L
        logger.completed = true;
        logger.reason = "완주";
        break;
    end
    if st.t >= opt.tMax
        logger.reason = "최대 시간";
        break;
    end

    % --- 차량 모델: 명령 -> 다음 상태 ---
    st = bicycle_step(st, u, p, dt);

    % --- 실제 시간에 맞추기 (그림 창 키 입력도 여기서 처리됨) ---
    if opt.realtime
        waitTime = step * dt - toc(t0);
        if waitTime > 0, pause(waitTime); else, drawnow limitrate; end
    elseif ~isempty(G) && mod(step, opt.plotEvery) == 0
        drawnow limitrate;
    end
    if ~isempty(G) && ~isvalid(G.fig), running = false; end
end

if ~running && logger.reason == "중단", logger.reason = "사용자 종료"; end
if pid.(tune.name) ~= tuneStart
    logger.note(sprintf("주행 중 %s를 %g에서 %g로 바꿈 (매 스텝 값은 tune_val 열)", ...
        tune.name, tuneStart, pid.(tune.name)));
end
result = finishRun(logger);
fprintf('[pid] %s, %.1f s, 횡오차 RMS %.4f m / 최대 %.4f m, 앞뒤 위치오차 RMS %.4f m, 속도오차 RMS %.4f m/s\n', ...
    logger.reason, result.lap_time, result.e_lat_rms, result.e_lat_max_abs, result.e_s_rms, result.e_v_rms);
if ~isempty(G) && isvalid(G.fig), G.fig.Name = G.fig.Name + "  [종료]"; end

%% ---- 그림 (중첩 함수: 위의 변수를 함께 씀) ----
    function setupFigure()
        G.fig = figure('Name', 'PID 궤적 추종 - Q/Esc 종료', ...
            'NumberTitle', 'off', 'Position', [60 60 1450 780], ...
            'KeyPressFcn', @onKey, 'CloseRequestFcn', @onClose);
        G.ax = axes(G.fig, 'Position', [0.04 0.12 0.58 0.82]);
        draw_map(G.ax, p);
        G.ref  = plot(G.ax, ref.x, ref.y, 'y-', 'LineWidth', 1.5);
        G.traj = plot(G.ax, NaN, NaN, 'r-', 'LineWidth', 1.5);
        G.gap  = plot(G.ax, NaN, NaN, 'r--', 'LineWidth', 1);
        G.pw   = plot(G.ax, NaN, NaN, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0 0.45 1], 'MarkerEdgeColor', 'w');
        G.pr   = plot(G.ax, NaN, NaN, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', 'w');
        G.car  = plot(G.ax, st.x, st.y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 7);
        G.head = plot(G.ax, [st.x st.x], [st.y st.y], 'k-', 'LineWidth', 2);
        G.title = title(G.ax, '');

        G.axE = axes(G.fig, 'Position', [0.68 0.71 0.30 0.24]); hold(G.axE, 'on');
        G.eLat = plot(G.axE, NaN, NaN, 'b-');
        G.eS   = plot(G.axE, NaN, NaN, 'm-'); grid(G.axE, 'on'); G.eS.Visible = 'off';
        ylabel(G.axE, '\delta_{cmd} - \delta [deg] (목표 - 현재)'); yline(G.axE, 0, 'k-');
        title(G.axE, '조향각 오차: PID 목표 조향각 - 현재 조향각');
        G.axD = axes(G.fig, 'Position', [0.68 0.40 0.30 0.24]); hold(G.axD, 'on');
        G.dAct = plot(G.axD, NaN, NaN, 'b-');
        G.dCmd = plot(G.axD, NaN, NaN, 'r--'); grid(G.axD, 'on');
        ylabel(G.axD, '조향각 [deg]'); legend(G.axD, {'\delta', '\delta_{cmd}'}, 'Location', 'northeast');
        G.axV = axes(G.fig, 'Position', [0.68 0.09 0.30 0.24]); hold(G.axV, 'on');
        G.vAct = plot(G.axV, NaN, NaN, 'b-');
        G.vRef = plot(G.axV, NaN, NaN, 'r--'); grid(G.axV, 'on');
        ylabel(G.axV, '속도 [m/s]'); xlabel(G.axV, 't [s]'); legend(G.axV, {'v', 'v_r'}, 'Location', 'southeast');

        % 표시 켜기·끄기 (그룹: 이름, 핸들, 체크박스 글자)
        G.groups = {'ref', G.ref, '경로 (R)'; 'traj', G.traj, '궤적 (T)'; ...
                    'point', [G.pr, G.gap, G.pw], '기준점 (L)'};
        for gi = 1:size(G.groups, 1)
            G.check(gi) = uicontrol(G.fig, 'Style', 'checkbox', 'String', G.groups{gi, 3}, ...
                'Units', 'normalized', 'Position', [0.04 + 0.11 * (gi - 1), 0.02, 0.10, 0.04], ...
                'Value', 1, 'Callback', @(src, ~) setGroup(gi, src.Value));
        end

        if opt.liveTune
            G.tuneText = uicontrol(G.fig, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.40 0.02 0.12 0.035], 'String', '');
            uicontrol(G.fig, 'Style', 'slider', 'Units', 'normalized', ...
                'Position', [0.52 0.02 0.25 0.035], 'Min', tune.min, 'Max', tune.max, ...
                'Value', min(max(pid.(tune.name), tune.min), tune.max), ...
                'Callback', @(src, ~) setTune(src.Value));
            setTune(pid.(tune.name));
        end
    end

    function updateFigure(dbg, e, rk)
        D = logger.data(1:logger.n, :);
        set(G.traj, 'XData', D(:, 3), 'YData', D(:, 4));
        set(G.car,  'XData', st.x, 'YData', st.y);
        set(G.head, 'XData', st.x + [0, 0.5 * cos(st.yaw)], 'YData', st.y + [0, 0.5 * sin(st.yaw)]);
        set(G.pw, 'XData', e.x_w, 'YData', e.y_w);
        set(G.pr, 'XData', rk.x, 'YData', rk.y);
        set(G.gap, 'XData', [st.x, rk.x], 'YData', [st.y, rk.y]);
        set(G.eLat, 'XData', D(:, 1), 'YData', rad2deg(D(:, 9) - D(:, 7)));   % 목표 조향각 delta_cmd - 현재 조향각 delta
        set(G.eS,   'XData', D(:, 1), 'YData', D(:, 24));
        set(G.dAct, 'XData', D(:, 1), 'YData', rad2deg(D(:, 7)));
        set(G.dCmd, 'XData', D(:, 1), 'YData', rad2deg(D(:, 9)));
        set(G.vAct, 'XData', D(:, 1), 'YData', D(:, 6));
        set(G.vRef, 'XData', D(:, 1), 'YData', D(:, 23));
        G.title.String = sprintf(['t = %.1f s   v = %.2f (v_r %.2f) m/s   \\delta = %.1f deg   ' ...
            'e_{lat} = %.3f m   e_s = %.3f m'], st.t, st.v, rk.v, rad2deg(st.delta), e.e_lat, dbg.e_s);
    end

    function setGroup(gi, on)
        set(G.groups{gi, 2}, 'Visible', matlab.lang.OnOffSwitchState(logical(on)));
        G.check(gi).Value = on;
    end

    function setTune(val)
        pid.(tune.name) = val;
        G.tuneText.String = sprintf('%s = %.3g', tune.name, val);
    end

    function onKey(~, evt)
        switch evt.Key
            case {'q', 'escape'}, running = false;
            case 'r', setGroup(1, ~G.check(1).Value);
            case 't', setGroup(2, ~G.check(2).Value);
            case 'l', setGroup(3, ~G.check(3).Value);
        end
    end

    function onClose(src, ~)
        running = false;
        delete(src);
    end
end

function S = finishRun(logger)
% 앞뒤 위치오차 e_s와 속도오차 e_v의 요약을 메모로 남긴 뒤 저장 (Q/Esc, 오류, Ctrl+C에도 한 번만)
if ~logger.finished && logger.n > 0
    D = logger.data(1:logger.n, :);
    eS = D(:, strcmp(logger.cols, 'e_s_m'));
    eV = D(:, strcmp(logger.cols, 'e_v_mps'));
    logger.note(sprintf("앞뒤 위치오차 e_s [m]: RMS %.4f, 최대|e| %.4f / 속도오차 e_v [m/s]: RMS %.4f, 최대|e| %.4f", ...
        sqrt(mean(eS.^2)), max(abs(eS)), sqrt(mean(eV.^2)), max(abs(eV))));
end
S = logger.finish();
if logger.n > 0
    D = logger.data(1:logger.n, :);
    eS = D(:, strcmp(logger.cols, 'e_s_m'));
    eV = D(:, strcmp(logger.cols, 'e_v_mps'));
    S.e_s_rms = sqrt(mean(eS.^2));  S.e_s_max_abs = max(abs(eS));
    S.e_v_rms = sqrt(mean(eV.^2));  S.e_v_max_abs = max(abs(eV));
else
    S.e_s_rms = NaN; S.e_s_max_abs = NaN; S.e_v_rms = NaN; S.e_v_max_abs = NaN;
end
end
