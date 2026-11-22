function result = switch2_discrete(varargin)
% switching_2: PP / PID / BC 블랜디드(스위칭) 조향 제어 (카메라 차선 feature로 λ를 정함)
%   δ_blend[k] = λ0 δ_PP[k] + λ1 δ_PID[k] + λ2 δ_BC[k]
%     PP  (PP/pp_steer.m)   : 순수 Pure Pursuit  δ = atan(2 L sin α / L_ad)
%     PID (PID/pid_steer.m) : 순수 횡오차 PID    δ = Kp e + Ki Σ e Ts + Kd Δe / Ts,  e = 0 - e_lat
%     BC  (BC/bc_steer.m)   : 레퍼런스 경로 그대로 따라가는 조향각 δ = atan(L κ(s(k+1)))  (test1/discrete_path_drive)
%     λ   (decide_lambda.m) : 보이는 차선이 모두 직선 → PID (직선은 PID)
%                             그 밖에는 기본 PP, 단 "PP로 달려도 오차가 커지는 구간" → BC
%     BC 구간 (pp_error_map.m): 시작 전에 PP만으로 한 바퀴 미리 계산 → |오차| >= e_BC인 곳과 그 lead [m] 앞
%                             (오차가 생기기 전에 BC로 바꿈. 오차가 커진 뒤 바꾸면 BC가 되돌리지 못함)
%
%   네 차를 같은 출발점·같은 가속도로 나란히 계산 (Ts = 0.1 s):
%     PP 차, PID 차 : 기구학 자전거 모델 (bicycle_discrete_step.m), 각 제어기만
%     BC 차         : discrete_path_drive 모델 (경로 위를 그대로 감) = 레퍼런스
%     블랜디드 차   : 자전거 모델. 매 스텝 세 조향각을 이 차의 상태로 모두 계산하고 λ로 합침
%                     (안 쓰는 제어기의 적분도 계속 갱신 → 전환하는 순간 PID가 처음부터 시작하지 않음)
%                     Unity에는 이 차를 보냄 (카메라가 이 차의 시점을 봐야 하므로)
%   입력 조향각 = 제어기가 낸 명령 δ_cmd[k],  출력 조향각 = 차량 모델을 거친 실제 조향각 δ[k+1]
%
% 한 스텝 (decision = "camera"):
%   MATLAB이 상태 x[k]를 Unity로 보냄 → Unity가 Car_Camera 사진 k를 lane_server로 보냄
%   → lane_server가 "LANE,k,...,차선별 상태·곡률,..."을 Unity를 거쳐 MATLAB으로 보냄
%   → MATLAB은 이 줄을 기다렸다가 (최대 waitTimeout s) decide_lambda로 λ를 정하고 u[k] 계산
%   λ가 바뀐 스텝은 lane_server에 "SAVE,k"를 보내 detection을 그린 사진을 data/<실행시각>/features/에 저장
%
% 실행 순서 (camera):
%   1) Unity Play  (Car_Camera → Car Camera Capture: Period 0.1, Send To Lane Server 켜짐)
%   2) lane_server: 꺼져 있으면 MATLAB이 새 창으로 자동으로 켬 (ensure_lane_server.m, 첫 준비 약 20 s)
%      직접 켜도 됨: C:\Users\haepi\venvs\kiis_yolo\Scripts\python.exe lane_yolo\lane_server.py
%   3) MATLAB:  cd MATLAB; setup_paths; addpath('switching_2'); switch2_discrete
%
% 사용 예:
%   switch2_discrete                                          % 카메라, 자동 0.8 m/s 한 바퀴
%   switch2_discrete('input', 'keyboard')                     % Unity에서 Space 한 번 = 한 스텝
%   switch2_discrete('decision', 'ref', 'useUnity', false)    % 카메라 없이 경로 곡률을 feature 대신 씀 (확인용)
%   switch2_discrete('eBC', 0.015, 'lead', 0.8, 'Kp', 2.5)   % 튜닝 값으로 시작 (아래 tune 이름)
% 실행 중에는 그림 왼쪽 아래 슬라이더·입력칸·선택 상자로 v, Lf, Kp, Ki, Kd, κ_th, e_BC, lead, 직선 제어기, 고정을 바꿈
%   (다음 스텝부터 적용, timeseries.csv의 p_* 열에 스텝마다 기록)
% 결과:
%   switching_2/data/<실행시각>/   switching2.xlsx (데이터 저장 형식.xlsx 구성), timeseries.csv (모든 값),
%                                  summary.txt, pp_error_map.csv (BC 구간), features/ (λ가 바뀐 스텝의 detection 사진)
%   switching_2/graph/<실행시각>/  figure1.png (PP·PID·BC), figure2.png (블랜디드 vs 레퍼런스)
%   그래프는 저장된 timeseries.csv를 plot_switch2.m이 읽어서 그림 (나중에 plot_switch2(폴더)로 다시 그릴 수 있음)

%% 0. 설정
opt.decision = "camera";        % "camera": lane_server 판단 / "ref": 레퍼런스 경로 곡률을 feature 대신 씀
opt.input = "auto";             % "auto": 목표 속도 v로 스스로 / "keyboard": Unity에서 Space = 한 스텝, ↑/↓ 가감속
opt.useUnity = true;
opt.realtime = true;            % 스텝 간격을 실제 Ts에 맞춤 (camera는 판단을 기다리므로 더 길어질 수 있음)
opt.showFigure = true;
opt.save = true;
opt.tMax = 120;                 % 최대 시간 [s]
opt.laps = 1;
opt.waitTimeout = 0.5;          % LANE,k를 기다리는 최대 시간 [s]
opt.firstWaitTimeout = 5;       % 처음 5스텝은 더 기다림 (MATLAB 첫 실행 지연)
opt.refAhead = [1 5];           % "ref" 판단: 차 앞 구간 [m] (lane_server의 zmin~zmax와 같게)
opt.features = true;            % λ가 바뀐 스텝의 detection 사진 저장 (lane_server에 직접 연결)
opt.featurePort = 55003;        % lane_server의 사진 저장 명령 포트
opt.autoLaneServer = true;      % camera: lane_server가 꺼져 있으면 새 창으로 켜고 준비될 때까지 기다림 (ensure_lane_server.m)
opt.startMode = "PID";          % 첫 판단 전에 쓸 제어기
opt.stepHook = [];             % 테스트용: 매 스텝 끝에 hook(fig, k) 호출 (패널 조작 확인)
opt.Ts = 0.1;                   % 한 스텝 [s] (실행 중 변경 불가)
% 실행 중에도 바꿀 수 있는 값 (그림 패널, 초기값은 여기 또는 이름-값으로)
tune.v = 0.8;                   % 목표 속도 [m/s] (자동 버전)
tune.Lf = 0.8;                  % PP 선행점 거리 [m]
tune.Kp = 3.0;                  % PID [rad/m]          (blended_params의 pidKp와 같은 값)
tune.Ki = 0.8;                  % PID [rad/(m s)]      (pidKi)
tune.Kd = 2.6;                  % PID [rad s/m]        (pidKd)
tune.kappaTh = 0.05;            % 차선 곡선 기준 |κ| [1/m] (lane_server 기본과 같음)
tune.eBC = 0.02;                % PP 오차 지도에서 |0 - e_lat|가 이 값 이상인 곳 = BC 구간 [m]
tune.lead = 0.5;                % 그곳보다 이만큼 앞에서부터 BC [m] (오차가 생기기 전에 바꿈)
tune.straightMode = "PID";      % 모두 직선일 때: "PID" | "PP"
tune.force = "auto";            % "auto" | "PP" | "PID" | "BC" (고정해서 비교할 때)
range = struct('v', [0.1 1.5], 'Lf', [0.3 2.5], 'Kp', [0 8], 'Ki', [0 3], 'Kd', [0 6], ...
    'kappaTh', [0 0.3], 'eBC', [0.005 0.08], 'lead', [0 2]);
for i = 1:2:numel(varargin)
    name = char(varargin{i});
    if isfield(opt, name), opt.(name) = varargin{i + 1};
    elseif isfield(tune, name), tune.(name) = varargin{i + 1};
    else, error("switch2_discrete: 알 수 없는 옵션 '%s'", name);
    end
end
opt.decision = string(opt.decision); opt.input = string(opt.input);
tune.straightMode = string(tune.straightMode); tune.force = string(tune.force);
if opt.decision == "camera" && ~opt.useUnity
    error("카메라 판단은 Unity가 필요합니다. Unity 없이 하려면 'decision', 'ref'를 쓰세요.");
end
if opt.input == "keyboard" && ~opt.useUnity
    error("키보드 버전은 Unity에서 키를 받으므로 'useUnity', true 가 필요합니다.");
end

here = fileparts(mfilename('fullpath'));          % MATLAB/switching_2
base = fileparts(here);                           % MATLAB (vehicle_params, path_error, unity_link, draw_map ...)
addpath(base, '-end');
addpath(here, fullfile(here, 'PP'), fullfile(here, 'PID'), fullfile(here, 'BC'));   % 이 폴더 함수를 먼저 씀
if opt.decision == "camera" && opt.autoLaneServer
    % Unity에 첫 상태를 보내기 전에 lane_server와 Unity 연결까지 준비 (사진 k = 0부터 판단이 오게)
    ensure_lane_server('ctrlPort', opt.featurePort);
end
p = vehicle_params();
ref = load_reference_path();
L = p.L; Ts = opt.Ts; N = round(opt.tMax / Ts);
S = switch2_style();
tuneNames = ["v", "Lf", "Kp", "Ki", "Kd", "kappaTh", "eBC", "lead"];

runName = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
dataDir = fullfile(here, 'data', runName);
graphDir = fullfile(here, 'graph', runName);
if opt.save, mkdir(dataDir); end

%% 1. 출발: Unity 차 위치를 경로에 투영한 점, 경로 방향, 정지
e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
q0 = path_pose(ref, e0.s);
st0 = struct('t', 0, 'x', q0.x, 'y', q0.y, 'yaw', q0.psi, 's', 0, 'v', 0, 'delta', atan(L * q0.kappa), 'yawRate', 0);
car = repmat(st0, 1, 4);                          % 1 PP, 2 PID, 3 BC (경로 위), 4 블랜디드
bc = [e0.s; 0; st0.delta];                        % BC 차 상태 [s; v; delta] (discrete_path_drive 모델)
memPP = []; memPID = []; memBlPP = []; memBlPID = []; hint = NaN(1, 4);
mode = string(opt.startMode); rule = "시작";

% 기록 (행 = 스텝 k = 0..n-1: 입력 u[k]와 그 결과 x[k+1])
D.Ts = Ts; D.x0 = repmat(st0.x, 1, 4); D.y0 = repmat(st0.y, 1, 4);
D.t = (0:N - 1)' * Ts;
D.x = nan(N, 4); D.y = D.x; D.yaw = D.x; D.din = D.x; D.dout = D.x; D.etraj = D.x;
D.v = nan(N, 1); D.a = D.v; D.xp = D.v; D.yp = D.v;
D.mode = strings(N, 1); D.rule = D.mode; D.lam = nan(N, 3); D.cand = nan(N, 3);
D.kap = nan(N, 3); D.states = strings(N, 3); D.lane = strings(N, 1);
D.waitms = nan(N, 1); D.timeout = zeros(N, 1); D.featureSaved = zeros(N, 1); D.zone = false(N, 1);
PRM = nan(N, numel(tuneNames)); OTH = strings(N, 1); FRC = strings(N, 1);
Z = pp_error_map(p, ref, tune.v, tune.Lf, Ts, tune.eBC, tune.lead);   % BC 구간 (PP 오차 지도)
fprintf('PP 오차 지도: |오차| >= %.3f m인 곳과 그 %.2f m 앞 = BC 구간 (경로의 %.0f%%)\n', tune.eBC, tune.lead, 100 * mean(Z.grid));

%% 2. Unity, lane_server 연결
client = []; lost = false; nSent = 0; ctrl = []; nFeat = 0;
cmdQueue = strings(0, 1); laneBuf = strings(0, 1);
if opt.useUnity
    client = unity_link('connect', p);
    if isempty(client), error("Unity에 연결할 수 없습니다. Unity에서 Play를 먼저 누르세요."); end
    guard = onCleanup(@() releaseTcp(client));
    if client.NumBytesAvailable > 0, read(client, client.NumBytesAvailable); end
    unity_link('reset', client); unity_link('path', client, ref, 0.05);
    nSent = nSent + sendCar(st0, NaN, NaN);
    fprintf('Unity 연결됨: 블랜디드 차를 보냅니다 (판단: %s, %s)\n', opt.decision, opt.input);
end
if opt.decision == "camera" && opt.features && opt.save
    try
        ctrl = tcpclient('127.0.0.1', opt.featurePort, 'ConnectTimeout', 2);
        ctrlGuard = onCleanup(@() releaseTcp(ctrl));
        mkdir(fullfile(dataDir, 'features'));
        writeline(ctrl, "RUN," + string(fullfile(dataDir, 'features')));
        fprintf('lane_server 사진 저장 연결됨 (포트 %d): λ가 바뀐 스텝의 사진을 features/에 저장\n', opt.featurePort);
    catch
        ctrl = [];
        warning('switch2_discrete:features', ['lane_server의 사진 저장 포트(%d)에 연결하지 못했습니다. ' ...
            'feature 사진 없이 진행합니다 (lane_server.py를 최신으로 다시 실행하세요).'], opt.featurePort);
    end
end

%% 3. 그림 (Figure 1 + 파라미터 패널)
H = []; fig = [];
if opt.showFigure
    H = switch2_fig1(p, ref, struct('withPanel', true, 'visible', true, ...
        'name', "switching_2 (" + opt.decision + ", " + opt.input + ") - delta = l0 PP + l1 PID + l2 BC"));
    fig = H.fig;
    makeTunePanel(fig, tune, range, tuneNames, Ts, opt.input);
    H.setZone(Z);
end

%% 4. 메인 루프
pause(0.01); path_error(ref, st0.x, st0.y, st0.yaw, NaN); path_pose(ref, 0);   % 예열 (첫 판단 대기가 길어지지 않게)
n = 0; tLoop = tic; tNext = 0; tDraw = -inf; nTimeout = 0; nLaneOk = 0;
if opt.input == "keyboard", fprintf('Unity Game 창 클릭 → Space 한 번 = 한 스텝 (↑ 가속, ↓ 감속), Q 종료\n'); end
for k = 1:N
    kk = k - 1;                                   % 스텝 번호 (0부터, 카메라 사진 k와 같음)
    % (0) 키보드: STEP이 올 때까지 기다림 (기다리는 동안에도 패널 사용 가능)
    keyUp = false; keyDown = false;
    if opt.input == "keyboard"
        cmd = waitCommand();
        if cmd == "QUIT" || lost, break; end
        f = split(cmd, ',');
        keyUp = numel(f) >= 2 && f(2) == "1"; keyDown = numel(f) >= 3 && f(3) == "1";
    end

    % (1) 패널에서 바꾼 값 → 이번 스텝부터
    if ~isempty(fig) && isvalid(fig) && fig.UserData.dirty
        old = tune; tune = fig.UserData.tune; fig.UserData.dirty = false;
        if any([old.v old.Lf old.eBC old.lead] ~= [tune.v tune.Lf tune.eBC tune.lead])   % BC 구간 다시 계산
            Z = pp_error_map(p, ref, tune.v, tune.Lf, Ts, tune.eBC, tune.lead);
            H.setZone(Z);
        end
    end
    PRM(k, :) = arrayfun(@(nm) tune.(nm), tuneNames); OTH(k) = tune.straightMode; FRC(k) = tune.force;
    cfg = struct('kappaTh', tune.kappaTh, 'straightMode', tune.straightMode, 'force', tune.force);
    eNow = path_error(ref, car(4).x, car(4).y, car(4).yaw, hint(4));     % 블랜디드 차의 지금 위치 → BC 구간?
    inZone = Z.grid(eNow.idx); D.zone(k) = inZone;

    % (2) feature → λ
    prevMode = mode;
    if opt.decision == "camera"
        [line, waited, ok] = waitLane(kk);
        D.waitms(k) = waited * 1000; D.lane(k) = line;
        if lost, break; end
        if ok
            nLaneOk = nLaneOk + 1;
            kap = parseLane(line);
            [mode, lam, info] = decide_lambda(kap, prevMode, cfg, inZone); rule = info.rule;
            D.kap(k, :) = kap; D.states(k, :) = info.states;
        else
            D.timeout(k) = 1; nTimeout = nTimeout + 1; rule = "시간 초과(유지)";
            if nLaneOk == 0 && kk >= 1
                % 처음부터 판단이 하나도 안 옴 (2026-11-16: lane_server를 안 켜고 실행 → 한 바퀴 내내 PID로만 감)
                if opt.save && isfolder(dataDir), rmdir(dataDir, 's'); end
                error(['카메라 판단(LANE 줄)이 %.0f s 동안 하나도 오지 않았습니다. 확인하세요:\n' ...
                    '  1) lane_server.py가 켜져 있고 "Unity 연결됨"이 나왔는지\n' ...
                    '  2) Unity Car_Camera → Car Camera Capture의 Send To Lane Server가 켜져 있는지 (Lane Server Connected)\n' ...
                    '  카메라 없이 돌리려면 switch2_discrete(''decision'', ''ref'')'], 2 * opt.firstWaitTimeout);
            end
            if cfg.force ~= "auto", mode = cfg.force; end
            lam = double(mode == S.modes);
        end
    else
        eF = path_error(ref, car(4).x, car(4).y, car(4).yaw, hint(4));
        kr = mean(path_pose(ref, eF.s + linspace(opt.refAhead(1), opt.refAhead(2), 41)).kappa);   % 부호 둔 평균
        kap = [kr kr kr];
        [mode, lam, info] = decide_lambda(kap, prevMode, cfg, inZone); rule = info.rule;
        D.kap(k, :) = kap; D.states(k, :) = info.states;
    end
    D.mode(k) = mode; D.rule(k) = rule; D.lam(k, :) = lam;

    % (3) 가속도 a[k] (네 차 공통)
    if opt.input == "keyboard"
        aReq = p.accel * (keyUp && ~keyDown) - p.brake * (keyDown && ~keyUp);
    else
        aReq = (tune.v - car(4).v) / Ts;
    end
    gPP = struct('Lf', tune.Lf); gPID = struct('Kp', tune.Kp, 'Ki', tune.Ki, 'Kd', tune.Kd);

    % (4) 네 차: 조향각 → omega → 이산 모델
    % PP 차
    [dCmd, memPP] = pp_steer(car(1), ref, p, gPP, memPP);
    car(1) = bicycle_discrete_step(car(1), [aReq; (dCmd - car(1).delta) / Ts], p, Ts); D.din(k, 1) = dCmd;
    % PID 차
    [dCmd, memPID] = pid_steer(car(2), ref, p, gPID, memPID, Ts);
    car(2) = bicycle_discrete_step(car(2), [aReq; (dCmd - car(2).delta) / Ts], p, Ts); D.din(k, 2) = dCmd;
    % BC 차 (경로 위를 그대로 감 = 레퍼런스)
    dCmd = bc_steer(bc(1), bc(2), aReq, ref, p, Ts);
    bc = path_model_step(bc, [aReq; (dCmd - bc(3)) / Ts], p, Ts); D.din(k, 3) = dCmd;
    qb = path_pose(ref, bc(1));
    car(3) = struct('t', k * Ts, 'x', qb.x, 'y', qb.y, 'yaw', qb.psi, 's', bc(1) - e0.s, 'v', bc(2), ...
        'delta', bc(3), 'yawRate', bc(2) * qb.kappa);
    % 블랜디드 차: 세 조향각을 모두 계산하고 λ로 합침
    [dPP, memBlPP, dbgPP] = pp_steer(car(4), ref, p, gPP, memBlPP);
    [dPID, memBlPID] = pid_steer(car(4), ref, p, gPID, memBlPID, Ts);
    dBC = bc_steer(dbgPP.e.s, car(4).v, aReq, ref, p, Ts);
    D.cand(k, :) = [dPP dPID dBC];
    dCmd = lam * [dPP; dPID; dBC];
    [car(4), used] = bicycle_discrete_step(car(4), [aReq; (dCmd - car(4).delta) / Ts], p, Ts); D.din(k, 4) = dCmd;

    % (5) 결과 x[k+1] 기록: 위치, 출력 조향각, 궤적 오차 (0 - e_lat)
    for c = 1:4
        e = path_error(ref, car(c).x, car(c).y, car(c).yaw, hint(c)); hint(c) = e.idx;
        D.x(k, c) = car(c).x; D.y(k, c) = car(c).y; D.yaw(k, c) = car(c).yaw;
        D.dout(k, c) = car(c).delta; D.etraj(k, c) = 0 - e.e_lat;
        if c == 4, D.xp(k) = e.x_proj; D.yp(k) = e.y_proj; eBl = e; end
    end
    D.v(k) = car(4).v; D.a(k) = used(1); n = k;

    % (6) λ가 바뀌었으면 그 사진(k)을 detection과 함께 저장 요청
    if ~isempty(ctrl) && mode ~= prevMode && opt.decision == "camera" && D.lane(k) ~= ""
        try
            writeline(ctrl, sprintf('SAVE,%d,%s_to_%s,%s,%.3f,%.3f', kk, prevMode, mode, ruleAscii(rule), ...
                tune.kappaTh, tune.eBC));
            D.featureSaved(k) = 1; nFeat = nFeat + 1;
        catch
            ctrl = [];
        end
    end

    % (7) Unity로 블랜디드 차 전송
    if ~isempty(client)
        if sendCar(car(4), dbgPP.xl, dbgPP.yl, eBl, dCmd), nSent = nSent + 1; else
            lost = true; fprintf('Unity로 보내지 못했습니다 (k = %d). 지금까지 결과를 저장합니다.\n', kk); break;
        end
    end

    % (8) 그림 (키보드: 매 스텝, 그 밖: 0.3 s마다)
    if ~isempty(fig) && isvalid(fig) && (opt.input == "keyboard" || toc(tLoop) - tDraw >= 0.3)
        H.update(D, k, titleLines(k)); drawnow; tDraw = toc(tLoop);
    end

    if ~isempty(opt.stepHook) && ~isempty(fig) && isvalid(fig), opt.stepHook(fig, k); end   % 테스트용

    % (9) 실시간 간격 (pause 동안 패널 이벤트도 처리됨)
    if opt.realtime && opt.input == "auto"
        tNext = tNext + Ts;
        w = tNext - toc(tLoop);
        if w > 0, pause(w); else, tNext = toc(tLoop); pause(0.001); end
    elseif ~isempty(fig)
        pause(0.001);
    end
    if car(4).s >= opt.laps * ref.L, break; end
end
loopTime = toc(tLoop);
if ~isempty(client), delete(guard); end
if ~isempty(ctrl), pause(0.3); delete(ctrlGuard); end   % 마지막 저장 요청이 처리될 시간을 둠

%% 5. 지표
M = struct('name', cellstr(S.names));
for c = 1:4
    et = D.etraj(1:n, c);
    M(c).rms_traj = sqrt(mean(et .^ 2)); M(c).max_traj = max(abs(et));
    M(c).rms_din_deg = rad2deg(sqrt(mean(D.din(1:n, c) .^ 2)));
end
ein = D.din(1:n, 3) - D.din(1:n, 4); eout = D.dout(1:n, 3) - D.dout(1:n, 4);
md = D.mode(1:n);
nSwitch = sum(md(2:end) ~= md(1:end - 1));
share = arrayfun(@(m) mean(md == m), S.modes);

%% 6. 데이터 저장 (그림보다 먼저)
if opt.save && n > 0
    T = makeTable(n);
    writetable(T, fullfile(dataDir, 'timeseries.csv'));
    try
        writeXlsx(fullfile(dataDir, 'switching2.xlsx'), n);
    catch ME
        warning('switch2_discrete:xlsx', 'switching2.xlsx를 저장하지 못했습니다: %s (timeseries.csv는 저장됨)', ME.message);
    end
    writeSummary(fullfile(dataDir, 'summary.txt'));
    % BC 구간 (PP 오차 지도, 끝났을 때 값 기준): 경로점마다 BC 구간인지 (plot_switch2가 맵에 그림)
    writetable(table(ref.s, ref.x, ref.y, double(Z.grid), repmat(Z.eBC, ref.N, 1), repmat(Z.lead, ref.N, 1), ...
        'VariableNames', {'ref_s_m', 'ref_x_m', 'ref_y_m', 'bc_zone', 'eBC_m', 'lead_m'}), ...
        fullfile(dataDir, 'pp_error_map.csv'));
end

%% 7. 그림: 실시간 그림 마무리 + 저장된 데이터로 Figure 1, 2를 그려 graph/에 저장
if ~isempty(fig) && isvalid(fig) && n > 0
    H.update(D, n, titleLines(n));
end
if opt.save && n > 0
    try
        plot_switch2(dataDir, 'show', [false, opt.showFigure]);
    catch ME
        warning('switch2_discrete:figure', '그래프를 저장하지 못했습니다: %s (데이터는 %s에 저장됨)', ME.message, dataDir);
    end
end

result = struct('name', runName, 'dataDir', string(dataDir), 'graphDir', string(graphDir), 'steps', n, ...
    'opt', opt, 'tune', tune, 'metrics', M, 'switches', nSwitch, 'share', share, ...
    'rms_in_deg', rad2deg(sqrt(mean(ein .^ 2))), 'rms_out_deg', rad2deg(sqrt(mean(eout .^ 2))), ...
    'timeouts', nTimeout, 'features', nFeat, 'lapDone', car(4).s >= ref.L, 'wallTime', loopTime, ...
    'unity', struct('used', ~isempty(client), 'sent', nSent, 'lost', lost));
if n > 0, result.timeseries = makeTable(n); end   % 저장 여부와 상관없이 표로 돌려줌
fprintf('switching_2 (%s, %s, Ts %.2f s): %d 스텝, 전환 %d번 (PP %.0f%%, PID %.0f%%, BC %.0f%%), 시간 초과 %d번, 사진 %d장\n', ...
    opt.decision, opt.input, Ts, n, nSwitch, 100 * share, nTimeout, nFeat);
for c = 1:4
    fprintf('  %-6s 궤적 오차 RMS %.4f m, 최대 %.3f m\n', S.names(c), M(c).rms_traj, M(c).max_traj);
end
fprintf('  블랜디드 vs 레퍼런스: 입력 조향각 차이 RMS %.2f deg, 출력 조향각 차이 RMS %.2f deg\n', ...
    result.rms_in_deg, result.rms_out_deg);
if opt.save, fprintf('데이터: %s\n그래프: %s\n', dataDir, graphDir); end

    % ---------- 내부 함수 ----------
    function ok = sendCar(st, xl, yl, e, dCmd)
        if nargin < 4, e = path_error(ref, st.x, st.y, st.yaw, NaN); dCmd = NaN; end
        dbg = struct('x_w', e.x_proj, 'y_w', e.y_proj, 'x_d', NaN, 'y_d', NaN, 'x_l', xl, 'y_l', yl, ...
            'x_c', NaN, 'y_c', NaN, 'R', NaN, 'Lf', tune.Lf, 'tau', NaN, 'delta_cmd', dCmd);
        ok = unity_link('state', client, st, e, dbg);
    end

    function readLines()
        % Unity에서 온 줄을 모두 읽어 키 명령(STEP/QUIT)과 판단 줄(LANE)로 나눔
        try
            while client.NumBytesAvailable > 0
                s = strtrim(string(readline(client)));
                if startsWith(s, "LANE,")
                    if numel(split(s, ',')) < 11
                        fprintf('[switch2] 잘못된 LANE 줄을 버림: "%s"\n', s); continue;
                    end
                    laneBuf = [laneBuf(:); s];             % (:): 비었을 때 1x0이 되어도 열로 붙음
                    if numel(laneBuf) > 50, laneBuf = laneBuf(end - 49:end); end
                elseif startsWith(s, "STEP") || startsWith(s, "QUIT")
                    cmdQueue = [cmdQueue(:); s];
                end
            end
        catch
            lost = true;
        end
    end

    function cmd = waitCommand()
        cmd = "";
        while ~lost
            readLines();
            if ~isempty(cmdQueue)
                cmd = cmdQueue(1); cmdQueue = cmdQueue(2:end, 1);
                if startsWith(cmd, "QUIT"), cmd = "QUIT"; end
                return;
            end
            if ~isempty(fig) && isvalid(fig), drawnow; end
            pause(0.005);
        end
    end

    function [line, waited, ok] = waitLane(kWant)
        % "LANE,k,..." 중 k = kWant인 줄을 기다림 (더 오래된 줄은 버림)
        line = ""; ok = false; t0 = tic;
        limit = opt.waitTimeout;
        if kWant < 5, limit = max(limit, opt.firstWaitTimeout); end
        while toc(t0) < limit && ~lost
            readLines();
            for j = 1:numel(laneBuf)
                f = split(laneBuf(j), ',');
                if str2double(f(2)) == kWant
                    line = laneBuf(j); ok = true;
                    laneBuf = laneBuf(j + 1:end, 1);
                    break;
                end
            end
            if ok, break; end
            pause(0.001);
        end
        waited = toc(t0);
    end

    function tl = titleLines(m)
        tl = {sprintf('k = %d   t = %.1f s   v = %.2f m/s   블랜디드: %s  (\\lambda = [%d %d %d], %s)', ...
                m - 1, (m - 1) * Ts, D.v(m), D.mode(m), D.lam(m, :), D.rule(m)), ...
              sprintf('차선 κ [1/m]: outer %s, dashed %s, inner %s   |   BC 구간 %s   |   L_f %.2f  K_p %.3g  K_i %.3g  K_d %.3g  κ_{th} %.3g  e_{BC} %.3g  lead %.2f', ...
                kapStr(D.kap(m, 1)), kapStr(D.kap(m, 2)), kapStr(D.kap(m, 3)), string(ternary(D.zone(m), '안', '밖')), ...
                tune.Lf, tune.Kp, tune.Ki, tune.Kd, tune.kappaTh, tune.eBC, tune.lead)};
    end

    function T = makeTable(m)
        T = table((0:m - 1)', D.t(1:m), D.v(1:m), D.a(1:m), 'VariableNames', {'k', 't_s', 'v_mps', 'a_mps2'});
        for c = 1:4
            tg = S.tags(c);
            T.(tg + "_x_m") = D.x(1:m, c); T.(tg + "_y_m") = D.y(1:m, c); T.(tg + "_yaw_rad") = D.yaw(1:m, c);
            T.(tg + "_delta_in_rad") = D.din(1:m, c);     % 입력 조향각 δ_cmd[k]
            T.(tg + "_delta_out_rad") = D.dout(1:m, c);   % 출력 조향각 δ[k+1]
            T.(tg + "_e_traj_m") = D.etraj(1:m, c);       % 0 - e_lat at x[k+1]
        end
        T.bl_proj_x_m = D.xp(1:m); T.bl_proj_y_m = D.yp(1:m);
        T.lambda0_pp = D.lam(1:m, 1); T.lambda1_pid = D.lam(1:m, 2); T.lambda2_bc = D.lam(1:m, 3);
        T.mode = D.mode(1:m); T.rule = D.rule(1:m);
        T.bl_cand_pp_rad = D.cand(1:m, 1); T.bl_cand_pid_rad = D.cand(1:m, 2); T.bl_cand_bc_rad = D.cand(1:m, 3);
        T.outer_kappa = D.kap(1:m, 1); T.dashed_kappa = D.kap(1:m, 2); T.inner_kappa = D.kap(1:m, 3);
        T.outer_state = D.states(1:m, 1); T.dashed_state = D.states(1:m, 2); T.inner_state = D.states(1:m, 3);
        T.lane_wait_ms = D.waitms(1:m); T.lane_timeout = D.timeout(1:m); T.feature_saved = D.featureSaved(1:m);
        T.bc_zone = double(D.zone(1:m));                  % 1: 블랜디드 차가 BC 구간(PP 오차 지도) 안
        T.lane_line = D.lane(1:m);
        for j = 1:numel(tuneNames), T.("p_" + tuneNames(j)) = PRM(1:m, j); end
        T.p_straightMode = OTH(1:m); T.p_force = FRC(1:m);
    end

    function writeXlsx(file, m)
        % 데이터 저장 형식.xlsx 구성: 시간 주기(k)마다 블랜디드 / 주행 경로 / 입력 / 출력, + 횡오차 (네 차), 카메라
        kcol = (0:m - 1)'; tcol = D.t(1:m);
        dg = @(r) round(rad2deg(r), 4);
        B = table(kcol, tcol, D.etraj(1:m, 4), dg(D.din(1:m, 3) - D.din(1:m, 4)), dg(D.dout(1:m, 3) - D.dout(1:m, 4)), ...
            D.lam(1:m, 1), D.lam(1:m, 2), D.lam(1:m, 3), D.mode(1:m), ...
            'VariableNames', {'시간 주기 k', 't [s]', '블랜디드 제어기법_주행 경로 차이 [m]', ...
            '블랜디드 기법_tracking error (입력 조향각 차이) [deg]', '블랜디드 기법_조향각 오차 (출력 조향각 차이) [deg]', ...
            'λ0 (PP)', 'λ1 (PID)', 'λ2 (BC)', '사용한 제어기'});
        P = table(kcol, tcol, D.x(1:m, 1), D.y(1:m, 1), D.x(1:m, 2), D.y(1:m, 2), D.x(1:m, 3), D.y(1:m, 3), ...
            D.x(1:m, 4), D.y(1:m, 4), 'VariableNames', {'시간 주기 k', 't [s]', '주행 경로_PP x [m]', '주행 경로_PP y [m]', ...
            '주행 경로_PID x [m]', '주행 경로_PID y [m]', '주행 경로_BC x [m]', '주행 경로_BC y [m]', ...
            '주행 경로_블랜디드 x [m]', '주행 경로_블랜디드 y [m]'});
        I = table(kcol, tcol, dg(D.din(1:m, 1)), dg(D.din(1:m, 2)), dg(D.din(1:m, 3)), dg(D.din(1:m, 4)), ...
            'VariableNames', {'시간 주기 k', 't [s]', '조향각_PP [deg]', '조향각_PID [deg]', '조향각_BC [deg]', '조향각_블랜디드 [deg]'});
        O = table(kcol, tcol + Ts, dg(D.dout(1:m, 1)), dg(D.dout(1:m, 2)), dg(D.dout(1:m, 3)), dg(D.dout(1:m, 4)), ...
            'VariableNames', {'시간 주기 k', 't + Ts [s]', '조향각_PP [deg]', '조향각_PID [deg]', '조향각_BC [deg]', '조향각_블랜디드 [deg]'});
        % 횡오차 = 0 - e_lat: 뒤 차축 중앙과 레퍼런스 경로 사이 (레퍼런스 - 주행, 진행 방향 오른쪽 +), 스텝 k의 결과 x[k+1]
        Elat = table(kcol, tcol + Ts, D.etraj(1:m, 1), D.etraj(1:m, 2), D.etraj(1:m, 3), D.etraj(1:m, 4), ...
            'VariableNames', {'시간 주기 k', 't + Ts [s]', '횡오차_PP [m]', '횡오차_PID [m]', '횡오차_BC [m]', '횡오차_블랜디드 [m]'});
        C = table(kcol, tcol, D.kap(1:m, 1), D.states(1:m, 1), D.kap(1:m, 2), D.states(1:m, 2), D.kap(1:m, 3), ...
            D.states(1:m, 3), double(D.zone(1:m)), D.rule(1:m), D.mode(1:m), D.waitms(1:m), D.timeout(1:m), ...
            D.featureSaved(1:m), ...
            'VariableNames', {'시간 주기 k', 't [s]', 'outer κ [1/m]', 'outer 상태', 'dashed κ [1/m]', 'dashed 상태', ...
            'inner κ [1/m]', 'inner 상태', 'BC 구간 (PP 오차 지도)', '규칙', '사용한 제어기', '판단 대기 [ms]', ...
            '시간 초과', 'feature 사진'});
        if isfile(file), delete(file); end
        writetable(B, file, 'Sheet', '블랜디드');
        writetable(P, file, 'Sheet', '주행 경로');
        writetable(I, file, 'Sheet', '입력 조향각');
        writetable(O, file, 'Sheet', '출력 조향각');
        writetable(Elat, file, 'Sheet', '횡오차');   % (L은 축간거리라 다른 이름: 중첩 함수는 변수를 공유)
        writetable(C, file, 'Sheet', '카메라');
    end

    function writeSummary(file)
        fid = fopen(file, 'w', 'n', 'UTF-8');
        fprintf(fid, 'switching_2 (switch2_discrete) - %s, 판단 %s, %s\n', runName, opt.decision, opt.input);
        fprintf(fid, '블랜디드: delta = l0 delta_PP + l1 delta_PID + l2 delta_BC  (PP [1 0 0], PID [0 1 0], BC [0 0 1])\n');
        fprintf(fid, '규칙 (decide_lambda): 보이는 차선이 모두 직선 -> straightMode (기본 PID), 그 밖에는 BC 구간이면 BC, 아니면 PP\n');
        fprintf(fid, 'BC 구간 (pp_error_map, 끝 값 기준): PP만으로 한 바퀴 미리 계산해 |0 - e_lat| >= eBC인 곳과 그 lead m 앞 = 경로의 %.1f%%\n', ...
            100 * mean(Z.grid));
        fprintf(fid, '모델: PP/PID/블랜디드 = bicycle_discrete_step, BC = path_model_step (경로 위), Ts = %.3f s, L = %.2f m\n', Ts, L);
        fprintf(fid, '시작 값: v %.2f, Lf %.2f, Kp %.3g, Ki %.3g, Kd %.3g, kappaTh %.3g, eBC %.3g, lead %.2f, 직선 %s, force %s\n', ...
            PRM(1, :), OTH(1), FRC(1));
        fprintf(fid, '끝 값  : v %.2f, Lf %.2f, Kp %.3g, Ki %.3g, Kd %.3g, kappaTh %.3g, eBC %.3g, lead %.2f, 직선 %s, force %s\n', ...
            PRM(n, :), OTH(n), FRC(n));
        fprintf(fid, '스텝 %d (%.1f s), 한 바퀴 %s, 전환 %d번, 구간 비율 PP %.1f%% / PID %.1f%% / BC %.1f%%\n', n, n * Ts, ...
            string(ternary(car(4).s >= ref.L, '완주', '못 함')), nSwitch, 100 * share);
        if opt.decision == "camera"
            fprintf(fid, '판단 시간 초과 %d번, 판단 대기 평균 %.1f ms (중앙값 %.1f ms), feature 사진 %d장\n', nTimeout, ...
                mean(D.waitms(1:n), 'omitnan'), median(D.waitms(1:n), 'omitnan'), nFeat);
        end
        fprintf(fid, '\n%-10s | %-10s %-10s | %s\n', '차', 'RMS[m]', '최대[m]', '입력 조향각 RMS [deg]');
        for cc = 1:4
            fprintf(fid, '%-10s | %-10.4f %-10.4f | %.2f\n', S.names(cc), M(cc).rms_traj, M(cc).max_traj, M(cc).rms_din_deg);
        end
        fprintf(fid, '\n블랜디드 vs 레퍼런스(BC 차): 주행 경로 차이 RMS %.4f m (최대 %.3f m), 입력 조향각 차이 RMS %.2f deg (최대 %.2f), 출력 조향각 차이 RMS %.2f deg (최대 %.2f)\n', ...
            M(4).rms_traj, M(4).max_traj, rad2deg(sqrt(mean(ein .^ 2))), rad2deg(max(abs(ein))), ...
            rad2deg(sqrt(mean(eout .^ 2))), rad2deg(max(abs(eout))));
        fclose(fid);
    end
end

%% ---------- 파일 함수 ----------
function kap = parseLane(line)
% "LANE,k,dec,nvis,outer_state,outer_kappa,dashed_state,dashed_kappa,inner_state,inner_kappa,ms" → [outer dashed inner]
f = split(line, ',');
kap = nan(1, 3);
for j = 1:3
    if numel(f) >= 4 + 2 * j && f(3 + 2 * j) ~= "none", kap(j) = str2double(f(4 + 2 * j)); end
end
end

function s = kapStr(k)
if isnan(k), s = '-'; else, s = sprintf('%.3f', k); end
end

function s = ruleAscii(rule)
% 사진에 쓸 영문 규칙 이름 (OpenCV 글꼴은 한글을 못 씀)
switch rule
    case "모두 직선", s = "all straight";
    case "PP 오차 구간", s = "PP error zone";
    case "기본 (PP)", s = "default PP";
    case "고정", s = "forced";
    otherwise, s = "other";
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

function releaseTcp(c)
try
    if ~isempty(c) && isvalid(c), delete(c); end
catch
end
end

function makeTunePanel(fig, tune, range, tuneNames, Ts, inputMode)
% 그림 왼쪽 아래 파라미터 패널. 콜백은 값을 fig.UserData.tune에 적고 dirty만 표시 → 메인 루프가 다음 스텝에 적용
fig.UserData = struct('tune', tune, 'dirty', false, 'range', range);
label = struct('v', 'v 목표 속도 [m/s]', 'Lf', 'Lf PP 선행점 [m]', 'Kp', 'Kp PID [rad/m]', ...
    'Ki', 'Ki PID [rad/(m s)]', 'Kd', 'Kd PID [rad s/m]', 'kappaTh', 'κ_th 곡선 기준 [1/m]', ...
    'eBC', 'e_BC PP 오차 → BC [m]', 'lead', 'lead BC 미리 [m]');
pnl = uipanel(fig, 'Units', 'normalized', 'Position', [0.035 0.005 0.55 0.235], 'FontSize', 9, 'BackgroundColor', 'w', ...
    'Title', sprintf('파라미터 조절: 슬라이더 또는 입력칸(Enter), 다음 스텝부터 적용 (Ts = %.2f s 고정)', Ts));
for j = 1:numel(tuneNames)
    nm = char(tuneNames(j));
    colj = double(j > 4); row = j - 4 * colj;            % 왼쪽 열 v, Lf, Kp, Ki / 오른쪽 열 Kd, κ_th, e_BC, lead
    x0 = 0.01 + 0.5 * colj; y0 = 0.95 - 0.19 * row;
    rg = range.(nm);
    en = 'on'; lab = label.(nm);
    if strcmp(nm, 'v') && inputMode == "keyboard", en = 'off'; lab = 'v (키보드: ↑/↓로)'; end
    uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', 'Position', [x0, y0, 0.2, 0.13], ...
        'String', lab, 'HorizontalAlignment', 'left', 'BackgroundColor', 'w');
    uicontrol(pnl, 'Style', 'slider', 'Units', 'normalized', 'Position', [x0 + 0.2, y0 + 0.015, 0.2, 0.115], ...
        'Min', rg(1), 'Max', rg(2), 'Value', min(max(tune.(nm), rg(1)), rg(2)), 'Tag', "tune_" + nm, 'Enable', en, ...
        'Callback', @(src, ~) tuneCallback(src, nm, false));
    uicontrol(pnl, 'Style', 'edit', 'Units', 'normalized', 'Position', [x0 + 0.41, y0 + 0.005, 0.07, 0.135], ...
        'String', sprintf('%.4g', tune.(nm)), 'Tag', "tuneEdit_" + nm, 'Enable', en, ...
        'Callback', @(src, ~) tuneCallback(src, nm, true));
end
% 선택 상자: 모두 직선일 때 쓸 제어기, 고정
opts = ["PID", "PP"]; optsTxt = {'PID', 'PP'};
uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', 'Position', [0.01, 0.0, 0.2, 0.12], ...
    'String', '모두 직선 →', 'HorizontalAlignment', 'left', 'BackgroundColor', 'w');
uicontrol(pnl, 'Style', 'popupmenu', 'Units', 'normalized', 'Position', [0.21, 0.01, 0.12, 0.12], ...
    'String', optsTxt, 'Value', find(opts == tune.straightMode), 'Tag', 'tune_straightMode', ...
    'Callback', @(src, ~) choiceCallback(src, 'straightMode', opts));
fo = ["auto", "PP", "PID", "BC"]; foTxt = {'자동 (규칙)', 'PP 고정', 'PID 고정', 'BC 고정'};
uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', 'Position', [0.51, 0.0, 0.2, 0.12], ...
    'String', '제어기', 'HorizontalAlignment', 'left', 'BackgroundColor', 'w');
uicontrol(pnl, 'Style', 'popupmenu', 'Units', 'normalized', 'Position', [0.71, 0.01, 0.15, 0.12], ...
    'String', foTxt, 'Value', find(fo == tune.force), 'Tag', 'tune_force', ...
    'Callback', @(src, ~) choiceCallback(src, 'force', fo));
end

function tuneCallback(src, nm, fromEdit)
fig = ancestor(src, 'figure');
ud = fig.UserData;
if fromEdit, val = str2double(src.String); else, val = src.Value; end
lo = 0; hi = inf;
switch nm
    case 'v',  lo = 0.05; hi = 1.5;
    case 'Lf', lo = 0.05;
end
if ~isfinite(val) || val < lo || val > hi
    val = ud.tune.(nm);                                     % 잘못된 값이면 되돌림
else
    ud.tune.(nm) = val; ud.dirty = true; fig.UserData = ud;
end
rg = ud.range.(nm);
set(findobj(fig, 'Tag', "tune_" + nm), 'Value', min(max(val, rg(1)), rg(2)));
set(findobj(fig, 'Tag', "tuneEdit_" + nm), 'String', sprintf('%.4g', val));
end

function choiceCallback(src, nm, opts)
fig = ancestor(src, 'figure');
ud = fig.UserData;
ud.tune.(nm) = opts(src.Value); ud.dirty = true;
fig.UserData = ud;
end
