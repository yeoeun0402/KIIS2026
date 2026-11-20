function result = pure_pursuit_control(varargin)
% Pure pursuit로 레퍼런스 경로(차선)를 따라 주행하고, 그 상태를 Unity로 보내며 기록하는 코드
% 차량 모델: 기구학 자전거 모델 (vehicle_params.m, bicycle_step.m — 기준점 = 뒤 차축 중앙)
%
% 사용 예:
%   pure_pursuit_control                                % paper 모드, Unity 연결, 실시간, 그림
%   pure_pursuit_control('mode', 'fixed', 'Lf', 1.0)     % 모드·파라미터 바꾸기 (pp_params.m 참고)
%   pure_pursuit_control('useUnity', false, 'realtime', false, 'showFigure', false)   % 빠른 오프라인
%   pure_pursuit_control('liveTune', true)              % 그림의 슬라이더로 주행 중 조절 (탐색용)
%
% 그림 창 조작 (창을 클릭한 상태에서)
%   Q 또는 Esc : 종료 (창을 닫아도 종료)
%   R / T / L / C : 레퍼런스 / 궤적 / 선행점 / 원 표시 켜기·끄기 (아래 체크박스와 같음)
%
% 실행 순서 (Unity 사용 시): Unity Play 먼저 → 이 함수 실행
% 결과: results/<날짜_시간>_pp-<모드>_<경로>/ 에 timeseries.csv, run.mat, summary.txt, trajectory.png
% 반환: 요약 구조체 (횡오차 평균·RMS·MSE·최대, 완주 여부, 결과 폴더 등)

%% 0. 옵션 (여기 없는 이름은 pp_params로 넘어감)
here = fileparts(mfilename('fullpath'));     % MATLAB/Pure-Pursuit
base = fileparts(here);                      % MATLAB (공용 함수: vehicle_params, path_error, unity_link, ...)
if ~exist('vehicle_params', 'file'), addpath(base); end
opt.useUnity   = true;      % Unity로 상태 전송
opt.realtime   = true;      % 실제 시간에 맞춰 진행 (false면 최대한 빠르게)
opt.showFigure = true;      % MATLAB 실시간 그림
opt.liveTune   = false;     % 그림에 파라미터 슬라이더 표시
opt.laps       = 1;         % 몇 바퀴 돌고 끝낼지
opt.tMax       = 180;       % 최대 시간 [s]
opt.pathFile   = '';        % 비우면 reference_path_outer_lane.mat
opt.tag        = '';        % 결과 폴더 이름 뒤에 붙일 메모
opt.resultsDir = fullfile(base, 'results');
opt.plotEvery  = 5;         % 그림 갱신 간격 [스텝]
ppArgs = {};
for k = 1:2:numel(varargin)
    if isfield(opt, varargin{k})
        opt.(varargin{k}) = varargin{k + 1};
    else
        ppArgs(end + 1:end + 2) = varargin(k:k + 1);
    end
end

p   = vehicle_params();
pp  = pp_params(ppArgs{:});
ref = load_reference_path(opt.pathFile);
dt  = p.dt;

% 주행 중 슬라이더로 바꿀 대표 파라미터 (모드별)
switch pp.mode
    case "fixed", tune = struct('name', 'Lf',    'min', 0.3,  'max', 3.0);
    case "speed", tune = struct('name', 't_la',  'min', 0.3,  'max', 3.0);
    case "paper", tune = struct('name', 'f_max', 'min', 0.02, 'max', 0.2);
end

%% 1. 출발 상태: Unity car 위치 그대로
st = struct('t', 0, 'x', p.startUnity(1), 'y', p.startUnity(2), ...
    'yaw', deg2rad(90 - p.startUnity(3)), 'v', 0, 'delta', 0, 'yawRate', 0);

%% 2. Unity 연결 (기록 폴더를 만들기 전에 확인)
client = [];
if opt.useUnity
    client = unity_link('connect', p);
    if isempty(client)
        error("Unity에 연결할 수 없습니다. Unity Play를 먼저 누르거나, 'useUnity', false 로 실행하세요.");
    end
    unity_link('reset', client);
    unity_link('path', client, ref, 0.05);
end

%% 3. 기록 준비
cols = {'t_s', 'step', 'x_m', 'y_m', 'yaw_rad', 'v_mps', 'delta_rad', 'yawRate_radps', ...
        'delta_cmd_rad', 'v_cmd_mps', 'delta_sat', ...
        's_ref_m', 'x_proj_m', 'y_proj_m', 'psi_ref_rad', 'kappa_ref_1pm', 'e_lat_m', 'e_head_rad', ...
        'i_w', 'x_w_m', 'y_w_m', 'i_d', 'x_d_m', 'y_d_m', 'x_l_m', 'y_l_m', ...
        'Lf_m', 'eta_rad', 'kappa_cmd_1pm', 'R_m', 'x_c_m', 'y_c_m', ...
        'rmin_m', 'alpha', 'beta', 'tau', 'phi_rad', 'dubins_len_m', 'cur_pw_1pm', 'cur_pd_1pm', ...
        'comp_ms', 'progress_m', 'lap', 'tune_val'};
runName = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_pp-" + pp.mode + "_" + ref.name;
if strlength(string(opt.tag)) > 0, runName = runName + "_" + opt.tag; end
meta = struct('runName', runName, 'started', string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    'matlab', string(version), 'options', opt, 'tuneName', tune.name);
logger = RunLogger(fullfile(opt.resultsDir, runName), cols, ceil(opt.tMax / dt) + 10, ref, ...
    struct('vehicle', p, 'pp', pp), meta);
logger.sourceFiles = [fullfile(here, ["pure_pursuit_control.m", "pure_pursuit_step.m", ...
    "select_lookahead.m", "pursuit_steer.m", "pp_params.m"]), ...
    fullfile(base, ["vehicle_params.m", "bicycle_step.m", "path_error.m", "unity_link.m"])];
cleanupObj = onCleanup(@() logger.finish());

%% 4. 그림
running = true;
G = [];
if opt.showFigure, setupFigure(); end

%% 5. 메인 루프
progress = 0; sPrev = NaN; step = 0;
tuneStart = pp.(tune.name);
e = path_error(ref, st.x, st.y, st.yaw);       % 처음에는 경로 전체에서 찾기
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

    % --- 제어기: 상태 -> (delta_cmd, v_cmd) ---
    [u, dbg] = pure_pursuit_step(st, ref, p, pp, e);

    % --- 기록 (이 스텝의 상태·오차·명령) ---
    logger.add([st.t, step, st.x, st.y, st.yaw, st.v, st.delta, st.yawRate, ...
        u.delta_cmd, u.v_cmd, dbg.sat, ...
        e.s, e.x_proj, e.y_proj, e.psi, e.kappa, e.e_lat, e.e_head, ...
        dbg.i_w, dbg.x_w, dbg.y_w, dbg.i_d, dbg.x_d, dbg.y_d, dbg.x_l, dbg.y_l, ...
        dbg.Lf, dbg.eta, dbg.kappa_cmd, dbg.R, dbg.x_c, dbg.y_c, ...
        dbg.rmin, dbg.alpha, dbg.beta, dbg.tau, dbg.phi, dbg.dubins_len, dbg.cur_pw, dbg.cur_pd, ...
        dbg.comp_ms, progress, floor(progress / ref.L), pp.(tune.name)]);

    % --- Unity로 전송 ---
    if ~isempty(client) && ~unity_link('state', client, st, e, dbg)
        warning("Unity 연결이 끊겼습니다. 종료합니다.");
        logger.reason = "Unity 연결 끊김";
        break;
    end

    % --- 그림 갱신 ---
    if ~isempty(G) && mod(step, opt.plotEvery) == 0 && isvalid(G.fig)
        updateFigure(dbg, e);
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
if pp.(tune.name) ~= tuneStart
    logger.note(sprintf("주행 중 %s를 %g에서 %g로 바꿈 (매 스텝 값은 tune_val 열)", ...
        tune.name, tuneStart, pp.(tune.name)));
end
result = logger.finish();
fprintf('[%s] %s, %.1f s, 횡오차 RMS %.4f m / 최대 %.4f m\n', result.mode, logger.reason, ...
    result.lap_time, result.e_lat_rms, result.e_lat_max_abs);
if ~isempty(G) && isvalid(G.fig), G.fig.Name = G.fig.Name + "  [종료]"; end

%% ---- 그림 (중첩 함수: 위의 변수를 함께 씀) ----
    function setupFigure()
        G.fig = figure('Name', sprintf('Pure pursuit (%s) - Q/Esc 종료', pp.mode), ...
            'NumberTitle', 'off', 'Position', [60 60 1450 780], ...
            'KeyPressFcn', @onKey, 'CloseRequestFcn', @onClose);
        G.ax = axes(G.fig, 'Position', [0.04 0.12 0.58 0.82]);
        draw_map(G.ax, p);
        G.ref    = plot(G.ax, ref.x, ref.y, 'y-', 'LineWidth', 1.5);
        G.traj   = plot(G.ax, NaN, NaN, 'r-', 'LineWidth', 1.5);
        G.circle = plot(G.ax, NaN, NaN, '-', 'Color', [1 0 1 0.6], 'LineWidth', 0.8);
        G.arc    = plot(G.ax, NaN, NaN, 'm-', 'LineWidth', 2.5);
        G.lf     = plot(G.ax, NaN, NaN, 'r--', 'LineWidth', 1);
        G.pw     = plot(G.ax, NaN, NaN, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0 0.45 1], 'MarkerEdgeColor', 'w');
        G.pd     = plot(G.ax, NaN, NaN, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'w');
        G.pl     = plot(G.ax, NaN, NaN, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', 'w');
        G.car    = plot(G.ax, st.x, st.y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 7);
        G.head   = plot(G.ax, [st.x st.x], [st.y st.y], 'k-', 'LineWidth', 2);
        G.title  = title(G.ax, '');

        G.axE = axes(G.fig, 'Position', [0.68 0.58 0.30 0.36]);
        G.eLat = plot(G.axE, NaN, NaN, 'b-'); grid(G.axE, 'on');
        ylabel(G.axE, '\delta_{cmd} - \delta [deg] (목표 - 현재)'); xlabel(G.axE, 't [s]'); yline(G.axE, 0, 'k-');
        title(G.axE, '조향각 오차: pure pursuit 목표 조향각 - 현재 조향각');
        G.axD = axes(G.fig, 'Position', [0.68 0.12 0.30 0.36]); hold(G.axD, 'on');
        G.dAct = plot(G.axD, NaN, NaN, 'b-');
        G.dCmd = plot(G.axD, NaN, NaN, 'r--'); grid(G.axD, 'on');
        ylabel(G.axD, '조향각 [deg]'); xlabel(G.axD, 't [s]');
        legend(G.axD, {'\delta', '\delta_{cmd}'}, 'Location', 'northeast');

        % 표시 켜기·끄기 (그룹: 이름, 핸들, 체크박스 글자)
        G.groups = {'ref', G.ref, '레퍼런스 (R)'; 'traj', G.traj, '궤적 (T)'; ...
                    'look', [G.pw, G.pd, G.pl, G.lf], '선행점 (L)'; 'circle', [G.circle, G.arc], '원 (C)'};
        for gi = 1:size(G.groups, 1)
            G.check(gi) = uicontrol(G.fig, 'Style', 'checkbox', 'String', G.groups{gi, 3}, ...
                'Units', 'normalized', 'Position', [0.04 + 0.11 * (gi - 1), 0.02, 0.10, 0.04], ...
                'Value', 1, 'Callback', @(src, ~) setGroup(gi, src.Value));
        end

        if opt.liveTune
            G.tuneText = uicontrol(G.fig, 'Style', 'text', 'Units', 'normalized', ...
                'Position', [0.50 0.02 0.12 0.035], 'String', '');
            uicontrol(G.fig, 'Style', 'slider', 'Units', 'normalized', ...
                'Position', [0.62 0.02 0.25 0.035], 'Min', tune.min, 'Max', tune.max, ...
                'Value', min(max(pp.(tune.name), tune.min), tune.max), ...
                'Callback', @(src, ~) setTune(src.Value));
            setTune(pp.(tune.name));
        end
    end

    function updateFigure(dbg, e)
        D = logger.data(1:logger.n, :);
        set(G.traj, 'XData', D(:, 3), 'YData', D(:, 4));
        set(G.car,  'XData', st.x, 'YData', st.y);
        set(G.head, 'XData', st.x + [0, 0.5 * cos(st.yaw)], 'YData', st.y + [0, 0.5 * sin(st.yaw)]);
        set(G.pw, 'XData', dbg.x_w, 'YData', dbg.y_w);
        set(G.pd, 'XData', dbg.x_d, 'YData', dbg.y_d);
        set(G.pl, 'XData', dbg.x_l, 'YData', dbg.y_l);
        set(G.lf, 'XData', [st.x, dbg.x_l], 'YData', [st.y, dbg.y_l]);
        [cx, cy, ax_, ay_] = circleAndArc(st, dbg);
        set(G.circle, 'XData', cx, 'YData', cy);
        set(G.arc, 'XData', ax_, 'YData', ay_);
        set(G.eLat, 'XData', D(:, 1), 'YData', rad2deg(D(:, 9) - D(:, 7)));   % 목표 조향각 delta_cmd - 현재 조향각 delta
        set(G.dAct, 'XData', D(:, 1), 'YData', rad2deg(D(:, 7)));
        set(G.dCmd, 'XData', D(:, 1), 'YData', rad2deg(D(:, 9)));
        G.title.String = sprintf(['t = %.1f s   v = %.2f m/s   \\delta = %.1f deg   ' ...
            'e_{lat} = %.3f m   L_f = %.2f m   \\tau = %.2f   R = %.2f m'], ...
            st.t, st.v, rad2deg(st.delta), e.e_lat, dbg.Lf, dbg.tau, dbg.R);
    end

    function setGroup(gi, on)
        set(G.groups{gi, 2}, 'Visible', matlab.lang.OnOffSwitchState(logical(on)));
        G.check(gi).Value = on;
    end

    function setTune(val)
        pp.(tune.name) = val;
        G.tuneText.String = sprintf('%s = %.3g', tune.name, val);
    end

    function onKey(~, evt)
        switch evt.Key
            case {'q', 'escape'}, running = false;
            case 'r', setGroup(1, ~G.check(1).Value);
            case 't', setGroup(2, ~G.check(2).Value);
            case 'l', setGroup(3, ~G.check(3).Value);
            case 'c', setGroup(4, ~G.check(4).Value);
        end
    end

    function onClose(src, ~)
        running = false;
        delete(src);
    end
end

function [cx, cy, ax, ay] = circleAndArc(st, dbg)
% pure pursuit 원(전체)과 차 -> 선행점 호. 거의 직선이면 원 없이 선분만
if isnan(dbg.R) || abs(dbg.R) > 50
    cx = NaN; cy = NaN;
    ax = [st.x, dbg.x_l]; ay = [st.y, dbg.y_l];
    return;
end
th = linspace(0, 2 * pi, 121);
r = abs(dbg.R);
cx = dbg.x_c + r * cos(th);
cy = dbg.y_c + r * sin(th);
a0 = atan2(st.y - dbg.y_c, st.x - dbg.x_c);
a1 = atan2(dbg.y_l - dbg.y_c, dbg.x_l - dbg.x_c);
if dbg.R > 0
    sweep = mod(a1 - a0, 2 * pi);       % 왼쪽 회전: 반시계
else
    sweep = -mod(a0 - a1, 2 * pi);      % 오른쪽 회전: 시계
end
a = a0 + linspace(0, sweep, 40);
ax = dbg.x_c + r * cos(a);
ay = dbg.y_c + r * sin(a);
end
