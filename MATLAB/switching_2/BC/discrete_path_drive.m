function result = discrete_path_drive(varargin)
% 정해진 경로 위를 따라 움직이는 이산 상태공간 모델을 한 스텝씩 (k → k+1) 진행하고 Unity로 보여주는 코드
% 모델 (path_model_step.m):  x(k+1) = A x(k) + B u(k),   x = [s; v; delta],  u = [a; omega]
%   전진 (s, v): 더블 인테그레이터,  조향 (delta): 인테그레이터
%   차 위치·방향 = 경로(s) 위의 점과 접선 (path_pose.m) → 경로에서 벗어나지 않음
%
% 두 버전 ('input'):
%   "keyboard": Unity에서 스텝 키를 누를 때마다 한 스텝.
%               가속도 a = 그 스텝에 누른 ↑ (+accel) / ↓ (-brake), 아무것도 안 누르면 0 (속도 유지)
%   "auto"    : MATLAB이 매 T마다 스스로 한 스텝. 가속도 a = 목표 속도 v_tar에 맞추도록 자동
%   조향 속도 omega는 두 버전 모두: 다음 위치 곡률에 맞는 조향각 atan(L*kappa)를 따라가도록
%
% Unity 키 (Game 창을 클릭한 상태에서, StepInput.cs → TCP로 MATLAB에 전달)
%   Space / → : 한 스텝 (누르고 있으면 계속)    ↑ / ↓ : 그 스텝의 가속 / 감속 (키보드 버전)
%   P : 자동 진행 / 일시정지 (자동 버전)       Backspace : 처음으로 (k = 0)       Q : 종료·저장
%
% 사용 예 (Unity Play 먼저, MATLAB 폴더에서 addpath('test1') 한 번):
%   discrete_path_drive                                     % 키보드 버전
%   discrete_path_drive('input', 'auto')                    % 자동 버전
%   discrete_path_drive('input', 'auto', 'T', 0.02, 'v_tar', 1.0)
%   discrete_path_drive('input', 'auto', 'useUnity', false, 'realtime', false)   % Unity 없이 빠르게
% 결과: results/<날짜_시간>_discrete_<input>/ 에 timeseries.csv, summary.txt, trajectory.png

%% 0. 옵션
% [switching_2/BC 복사본] test1/discrete_path_drive.m을 그대로 복사함. 바뀐 곳은 아래 base 경로 한 줄뿐
%   (이 폴더는 MATLAB/switching_2/BC라 두 단계 위가 MATLAB 폴더). switching_2에서는 이 파일의 조향 규칙
%   δ_ref = atan(L κ(s(k+1)))만 bc_steer.m으로 꺼내 BC 조향각으로 씀
here = fileparts(mfilename('fullpath'));     % MATLAB/switching_2/BC
base = fileparts(fileparts(here));           % MATLAB (공용 함수: vehicle_params, path_error, unity_link, ...)
if ~exist('vehicle_params', 'file'), addpath(base); end
opt.input      = "keyboard";   % "keyboard" 또는 "auto"
opt.T          = 0.1;          % 이산 주기 [s] (한 스텝 = T초)
opt.v_tar      = 0.8;          % 자동 버전의 목표 속도 [m/s]
opt.useUnity   = true;         % Unity 연결 (키보드 버전은 반드시 필요)
opt.realtime   = true;         % 자동 버전에서 T초마다 한 스텝 (false면 최대한 빠르게)
opt.laps       = 1;            % 자동 버전: 몇 바퀴 돌고 끝낼지
opt.tMax       = 600;          % 자동 버전: 최대 시간 [s]
opt.pathFile   = '';           % 비우면 reference_path_outer_lane.mat
opt.tag        = '';           % 결과 폴더 이름 뒤에 붙일 메모
opt.resultsDir = fullfile(base, 'results');
opt.port       = [];           % 비우면 vehicle_params의 unityPort (통신 테스트용)
for i = 1:2:numel(varargin)
    if ~isfield(opt, varargin{i})
        error("discrete_path_drive: 알 수 없는 옵션 '%s'", varargin{i});
    end
    opt.(varargin{i}) = varargin{i + 1};
end
opt.input = string(opt.input);
if ~ismember(opt.input, ["keyboard", "auto"])
    error("input은 'keyboard' 또는 'auto' 입니다.");
end
if opt.input == "keyboard" && ~opt.useUnity
    error("키보드 버전은 Unity에서 키 입력을 받으므로 'useUnity', true 가 필요합니다.");
end

p = vehicle_params();
if ~isempty(opt.port), p.unityPort = opt.port; end
ref = load_reference_path(opt.pathFile);
T = opt.T;
[A, B] = path_model_matrices(T);

%% 1. 출발 상태: Unity car 위치를 경로에 투영한 지점에서 정지
e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
x0 = [e0.s; 0; atan(p.L * path_pose(ref, e0.s).kappa)];
x = x0; k = 0; episode = 1;

%% 2. Unity 연결
client = [];
lost = false;
if opt.useUnity
    client = unity_link('connect', p);
    if isempty(client)
        error("Unity에 연결할 수 없습니다. Unity에서 Play를 먼저 누르세요.");
    end
    if client.NumBytesAvailable > 0
        read(client, client.NumBytesAvailable);     % 연결 전에 쌓인 키 입력은 버림
    end
    unity_link('reset', client);
    unity_link('path', client, ref, 0.05);
end

%% 3. 기록 준비 (한 행 = 스텝 k의 상태 x(k)와 그때 넣은 입력 u(k))
cols = {'k', 'episode', 't_s', 's_m', 'v_mps', 'delta_rad', 'x_m', 'y_m', 'yaw_rad', 'kappa_1pm', ...
        'a_req_mps2', 'a_mps2', 'omega_radps', 'delta_ref_rad', 'key_up', 'key_down', 'progress_m'};
data = nan(4096, numel(cols));
n = 0;
runName = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_discrete_" + opt.input;
if strlength(string(opt.tag)) > 0, runName = runName + "_" + opt.tag; end
folder = fullfile(opt.resultsDir, runName);
if ~isfolder(folder), mkdir(folder); end
reason = "중단"; completed = false; saved = false; result = struct();
cleanupObj = onCleanup(@() saveRun());

sendState();                                        % k = 0 상태 표시
if opt.input == "keyboard"
    fprintf('[키보드 버전] Unity Game 창을 클릭하고 Space/→로 한 스텝, ↑/↓로 가속/감속, Backspace 처음으로, Q 종료\n');
else
    fprintf('[자동 버전] T = %.3g s마다 한 스텝. Unity에서 P 일시정지/재개, 멈춘 상태에서 Space 한 스텝, Q 종료\n', T);
end

%% 4. 메인 루프
running = true;
playing = (opt.input == "auto");
t0 = tic; tNext = T; tBeat = tic;
while running
    worked = false;

    % --- Unity에서 온 키 입력 처리 ---
    for c = pollCommands()'
        parts = split(c, ',');
        switch parts(1)
            case "STEP"
                if opt.input == "keyboard" || ~playing   % 자동 버전은 멈춘 상태에서만 한 스텝
                    doStep(numel(parts) >= 2 && parts(2) == "1", numel(parts) >= 3 && parts(3) == "1");
                    worked = true;
                end
            case "PLAY"
                if opt.input == "auto"
                    playing = ~playing;
                    tNext = toc(t0) + T;
                end
            case "RESET"
                resetEpisode();
                worked = true;
            case "QUIT"
                running = false;
                reason = "사용자 종료";
        end
    end

    % --- 자동 버전: T초마다 한 스텝 ---
    if running && playing && (~opt.realtime || toc(t0) >= tNext)
        doStep(false, false);
        worked = true;
        tNext = tNext + T;
        if tNext < toc(t0), tNext = toc(t0) + T; end   % 많이 밀렸으면 따라잡지 않고 다시 맞춤
    end

    % --- 끝낼 조건 ---
    if lost
        reason = "Unity 연결 끊김";
        break;
    end
    if opt.input == "auto" && running
        if x(1) - x0(1) >= opt.laps * ref.L
            completed = true; reason = "완주"; running = false;
        elseif k * T >= opt.tMax
            reason = "최대 시간"; running = false;
        end
    end

    % --- 할 일이 없으면 잠깐 쉼 (1초마다 같은 상태를 다시 보내 연결 확인) ---
    if ~worked && running
        if ~isempty(client) && toc(tBeat) > 1
            sendState();
            tBeat = tic;
        end
        pause(0.002);
    end
end
result = saveRun();

%% ---- 보조 함수 (중첩 함수: 위의 변수를 함께 씀) ----
    function doStep(up, down)
        % 가속도 입력 a(k)
        if opt.input == "keyboard"
            aReq = p.accel * (up && ~down) - p.brake * (down && ~up);
        else
            aReq = (opt.v_tar - x(2)) / T;          % 한 스텝에 목표 속도로 (한계는 모델이 자름)
        end
        % 조향 속도 입력 omega(k): 다음 위치 s(k+1)의 곡률에 맞는 조향각으로
        % (s, v는 조향과 무관하므로 조향 입력 0으로 먼저 s(k+1)을 구함)
        xs = path_model_step(x, [aReq; 0], p, T);
        deltaRef = atan(p.L * path_pose(ref, xs(1)).kappa);
        [xn, used] = path_model_step(x, [aReq; (deltaRef - x(3)) / T], p, T);

        logRow([aReq, used(1), used(2), deltaRef, up, down]);
        x = xn;
        k = k + 1;
        sendState();
    end

    function resetEpisode()
        logRow(nan(1, 6));                          % 이번 에피소드의 마지막 상태
        episode = episode + 1;
        x = x0; k = 0;
        if ~isempty(client), unity_link('reset', client); end
        sendState();
    end

    function logRow(u)
        q = path_pose(ref, x(1));
        if n == size(data, 1), data = [data; nan(size(data))]; end   % 모자라면 두 배로
        n = n + 1;
        data(n, :) = [k, episode, k * T, x(1), x(2), x(3), q.x, q.y, q.psi, q.kappa, u, x(1) - x0(1)];
    end

    function sendState()
        if isempty(client), return; end
        q = path_pose(ref, x(1));
        st = struct('t', k * T, 'x', q.x, 'y', q.y, 'yaw', q.psi, 'v', x(2), 'delta', x(3), ...
            'yawRate', x(2) * q.kappa);
        if ~unity_link('state', client, st), lost = true; end
    end

    function cmds = pollCommands()
        cmds = strings(0, 1);
        if isempty(client), return; end
        try
            while client.NumBytesAvailable > 0
                ln = strtrim(string(readline(client)));
                if strlength(ln) > 0, cmds(end + 1, 1) = ln; end %#ok<AGROW>
            end
        catch
            lost = true;
        end
    end

    function S = saveRun()
        % 기록 저장 (Q, 완주, 오류, Ctrl+C 어느 경우든 한 번만)
        if saved, S = result; return; end
        saved = true;
        logRow(nan(1, 6));                          % 마지막 상태
        Tb = array2table(data(1:n, :), 'VariableNames', cols);
        last = Tb.episode == episode;
        S = struct('folder', string(folder), 'input', opt.input, 'T', T, 'completed', completed, ...
            'reason', reason, 'steps', sum(~isnan(Tb.a_mps2)), 'episodes', episode, ...
            'distance', max(Tb.progress_m(last)), 'time', max(Tb.t_s(last)), 'v_max', max(Tb.v_mps), ...
            'a_limited', sum(abs(Tb.a_req_mps2 - Tb.a_mps2) > 1e-9), ...
            'omega_limited', sum(abs(Tb.delta_ref_rad - (Tb.delta_rad + T * Tb.omega_radps)) > 1e-9));
        result = S;
        tryRun(@() writetable(Tb, fullfile(folder, 'timeseries.csv')), 'timeseries.csv');
        tryRun(@() writeSummary(S), 'summary.txt');
        tryRun(@() plotRun(Tb), 'trajectory.png');
        fprintf('기록 저장: %s (%s, %d 스텝)\n', folder, reason, S.steps);
    end

    function writeSummary(S)
        lines = [
            "실행: " + runName
            "버전: " + opt.input + sprintf(" (T = %.3g s", T) + ...
                ternary(opt.input == "auto", sprintf(", 목표 속도 %.2f m/s)", opt.v_tar), ")")
            "모델: x(k+1) = A x(k) + B u(k),  x = [s; v; delta],  u = [a; omega]"
            "  A = " + mat2str(A, 4)
            "  B = " + mat2str(B, 4)
            "경로: " + string(ref.name) + sprintf(" (한 바퀴 %.2f m)", ref.L)
            "끝난 이유: " + S.reason + sprintf(" (에피소드 %d개, 총 %d 스텝)", S.episodes, S.steps)
            sprintf("마지막 에피소드: 진행 거리 %.2f m, 시간 %.2f s", S.distance, S.time)
            sprintf("최고 속도 %.3f m/s", S.v_max)
            sprintf("한계에 걸린 스텝: 가속도 %d, 조향 속도 %d", S.a_limited, S.omega_limited)
            ];
        writelines(lines, fullfile(folder, 'summary.txt'));
    end

    function plotRun(Tb)
        fig = figure('Visible', 'off', 'Position', [100 100 1500 650]);
        ax1 = axes(fig, 'Position', [0.04 0.1 0.55 0.82]);
        draw_map(ax1, p);
        plot(ax1, ref.x, ref.y, 'y-', 'LineWidth', 1.5);
        scatter(ax1, Tb.x_m, Tb.y_m, 12, Tb.v_mps, 'filled');
        cb = colorbar(ax1); cb.Label.String = 'v [m/s]';
        title(ax1, strrep(runName, '_', '\_') + sprintf("  (점 하나 = 스텝 하나, T = %.3g s)", T));
        ax2 = axes(fig, 'Position', [0.68 0.58 0.30 0.34]);
        ax3 = axes(fig, 'Position', [0.68 0.10 0.30 0.34]);
        plot(ax2, Tb.t_s, Tb.v_mps, '.-'); grid(ax2, 'on');
        ylabel(ax2, 'v [m/s]'); xlabel(ax2, 't = kT [s]');
        plot(ax3, Tb.t_s, rad2deg(Tb.delta_rad), '.-', Tb.t_s, rad2deg(Tb.delta_ref_rad), 'r--');
        grid(ax3, 'on'); legend(ax3, {'\delta', '\delta_{ref} = atan(L\kappa)'});
        ylabel(ax3, '조향각 [deg]'); xlabel(ax3, 't = kT [s]');
        exportgraphics(fig, fullfile(folder, 'trajectory.png'), 'Resolution', 110);
        close(fig);
    end
end

function tryRun(fn, what)
try
    fn();
catch err
    warning("%s 저장 실패: %s", what, err.message);
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
