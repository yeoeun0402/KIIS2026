function result = blended_discrete(varargin)
% PP_PID 조향 제어 (기준 논문: Chen et al., "Optimization of Pure Pursuit Controller based on PID Controller
% and Low-pass Filter", ITSC 2018의 PP_PI + D 항) + 이산 기구학 자전거 모델. Unity 키로 한 스텝씩 진행
%   모델 (bicycle_discrete_step.m):
%     액추에이터 = path_model_step.m  (전진: 더블 인테그레이터, 조향: 인테그레이터)
%     움직임     = bicycle_step.m과 같은 기구학 자전거 모델 → 차가 경로에서 벗어날 수 있음
%   제어 (blended_steer.m, 논문 Eq.10 + D):
%     δ = Lowpass( K_pp δ_pp + δ_p + δ_i + δ_d ),  K_pp + K_p + K_i + K_d = 1  (K_d = 0이면 논문의 PP_PI)
%     δ_pp = atan(2 L sin α / L_ad)                     (Eq.3, Pure Pursuit)
%     δ_p  = K_p e_la,  e_la = e + (a + L_ad) sin Δψ    (Eq.4–6, 선행 오차: 횡오차 + 방향 오차)
%     δ_i  = K_i Σ e                                    (Eq.7, 횡오차의 합)
%     δ_d  = K_d (e[k] - e[k-1]) / T_s                  (D, 횡오차의 변화율)
%     Lowpass: 창 길이 ℓ, 이번 값 가중치 w_current       (Eq.8–9, lpf_window.m)
%     ω[t] = (δ[t] - δ_now) / Ts                        (조향 인테그레이터 입력, 90°/s 제한)
%   비교 (논문 4장처럼 PP, PID와): 세 차를 같은 가속도로 나란히 계산, Unity에는 PP_PID 차를 보냄
%     PP    : δ = δ_pp (K_pp 1, PI 0, 필터 없음)
%     PID   : pid_steer.m (PID_control.m과 같은 횡오차 PID, 필터 없음)
%     PP_PID: 위 식
%   가속도 a[t] (세 제어기 공통): 키보드 ↑ +1.0, ↓ -2.0, 없으면 0 / 자동 a = (v - v[t]) / Ts
%
% 그림: 왼쪽 맵 (레퍼런스 경로, 세 궤적), 오른쪽 위 → 아래
%   ① Tracking error (PP_PID): 목표 조향각 δ_ref = atan(Lκ) (노랑), 현재 조향각 (빨강), 차이 (주황)
%   ② 조향각 오차 (δ_ref - δ)의 누적 MSE: PP (초록), PID (파랑), PP_PID (빨강)
%   ③ Trajectory tracking error: 레퍼런스 경로 - 주행 경로 = 0 - e_lat: PP, PID, PP_PID
%   맵 아래 체크박스(또는 그림 창에서 키)로 각 결과를 켜고 끔:
%     경로 (R), PP (1), PID (2), PP_PID (3), 선행점 (L), 목표 조향각 (Y), 현재 조향각 (D), 조향각 오차 (E)
%     PP·PID·PP_PID 체크박스는 맵의 궤적과 그래프 ②③의 선을 함께 켜고 끔
%
% 두 버전 ('input'):
%   "keyboard" (기본): Unity Game 창에서 Space 또는 → 한 번 = 한 스텝, Q = 종료
%   "auto"           : 키 없이 한 바퀴
% 사용 예 (Unity Play 먼저):
%   blended_discrete                                       % 키보드: ↑를 누른 채 Space로 출발
%   blended_discrete('input', 'auto')
%   blended_discrete('input', 'auto', 'useUnity', false)   % Unity 없이 (MATLAB 그림만 실제 속도로 움직임)
%   blended_discrete('input', 'auto', 'useUnity', false, 'realtime', false)   % 기다리지 않고 빨리 (분석용)
%   blended_discrete('input', 'auto', 'Kp', 0.02, 'Ki', 0.004, 'Kd', 0.002, 'Lf', 1.0, 'ell', 4)   % 튜닝 값으로 실행
% 튜닝 파라미터 (blended_params.m): Ts, v, Lf, Kp, Ki, Kd, Kpp, sumToOne, a, ell, wcur, pidKp, pidKi, pidKd, lpfAll
%   실행 중에는 그림 왼쪽 아래 슬라이더·입력칸으로 v, Lf, a, wcur, ell, Kp, Ki, Kd, Kpp를 바꿈 (다음 스텝부터 적용)
% 결과: MATLAB/results/<날짜_시간>_blended_discrete_<input>/ 에 timeseries.csv, summary.txt, result.png

%% 0. 설정: 실행 옵션 + 튜닝 파라미터 (blended_params.m)
opt.input = "keyboard"; opt.tMax = 600;
opt.useUnity = true; opt.showFigure = true;
opt.save = true;                                 % false: results 폴더에 저장하지 않음 (파라미터 비교용)
opt.realtime = true;                             % 자동 버전: 한 스텝을 실제 Ts초에 맞춰 진행 (Unity 없이도)
                                                 % false: 기다리지 않고 최대한 빨리, 그림은 5스텝마다 (분석·테스트용)
opt.liveTune = true;                             % 그림에 파라미터 슬라이더 표시 (실행 중 조절)
opt.stepHook = [];                               % 테스트용: 매 스텝 끝에 hook(fig, k) 호출 (check_live_tuning)
here = fileparts(mfilename('fullpath'));
base = fileparts(here);
if ~exist('vehicle_params', 'file'), addpath(base, '-end'); end
addpath(here);                                   % 이 폴더의 복제본(path_model_step 등)을 먼저 씀
prmArgs = {};
for i = 1:2:numel(varargin)
    name = char(varargin{i});
    if isfield(opt, name) || strcmp(name, 'port')
        opt.(name) = varargin{i + 1};            % 실행 옵션
    else
        prmArgs(end + 1:end + 2) = {name, varargin{i + 1}};   %#ok<AGROW> 튜닝 파라미터 (모르는 이름이면 blended_params가 오류)
    end
end
prm = blended_params(prmArgs{:});
for f = setdiff(fieldnames(prm), {'range'})'
    opt.(f{1}) = prm.(f{1});
end
opt.input = string(opt.input);

p = vehicle_params();
if isfield(opt, 'port'), p.unityPort = opt.port; end   % 통신 테스트용
ref = load_reference_path();
L   = p.L;                                       % 축간거리 0.59 m
Ts  = opt.Ts;                                    % 한 스텝 = 0.1 s
N   = round(opt.tMax / Ts);
if opt.input == "keyboard" && ~opt.useUnity
    error("키보드 버전은 Unity에서 키를 받으므로 'useUnity', true 가 필요합니다.");
end

%% 1. 세 제어기 — 슬라이더로 바꾸면 makeGains로 다시 만듦
names = ["PP", "PID", "PP_PID"];                 % 기록·요약용 이름
texNames = ["PP", "PID", "PP\_PID"];             % 그림(TeX)용 이름
tags  = ["pp", "pid", "bl"];                     % timeseries.csv 열 이름 앞부분 (bl = PP_PID)
G = makeGains();
nC = 3; iB = 3;                                  % iB = PP_PID (Unity로 보내는 차)
tuneNames = ["v", "Lf", "a", "wcur", "ell", "Kp", "Ki", "Kd", "Kpp"];   % 실행 중 바꿀 수 있는 파라미터 (기록 열 p_*)
col = [0.10 0.62 0.10;                           % PP: 초록
       0.00 0.40 0.95;                           % PID: 파랑
       0.85 0.10 0.10];                          % PP_PID: 빨강
cYel = [0.95 0.78 0.00]; cRed = [0.85 0.10 0.10]; cOra = [1.00 0.50 0.00];

%% 2. 출발: Unity 차 위치를 경로에 투영한 점, 경로 방향, 정지, 조향각 = 그 점 곡률에 맞는 값
e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
q0 = path_pose(ref, e0.s);
st0 = struct('t', 0, 'x', q0.x, 'y', q0.y, 'yaw', q0.psi, 's', 0, 'v', 0, ...
    'delta', atan(L * q0.kappa), 'yawRate', 0);
car = repmat(st0, 1, nC);
mem = cell(1, nC);

% 기록 (행 k = 시각 (k-1) Ts. 상태는 N+1행, 그 스텝에서 계산한 값은 N행)
X = nan(N + 1, nC); Y = X; YAW = X; DEL = X;
S = nan(N + 1, 1); V = S; A = nan(N, 1);
DREF = nan(N, nC); DCMD = DREF; ESTEER = DREF; ETRAJ = DREF; OMEGA = DREF;
DPP = DREF; DP = DREF; DI = DREF; DD = DREF; DRAW = DREF;
ELA = nan(N, 1); LAD = ELA; EHEAD = ELA; SE = ELA; DE = ELA;   % PP_PID의 e_la, L_ad, Δψ, Σe, (e[k]-e[k-1])/Ts
PRM = nan(N, numel(tuneNames));                  % 스텝마다 쓴 튜닝 파라미터 값
LX = nan(N, 1); LY = LX;                         % PP_PID 선행점 위치
X(1, :) = st0.x; Y(1, :) = st0.y; YAW(1, :) = st0.yaw; DEL(1, :) = st0.delta;
S(1) = 0; V(1) = 0;

client = [];
nSent = 0;                                       % Unity로 보낸 상태 줄 수
lost = false;                                    % 전송 실패 = Unity 연결 끊김
usedUnity = false;
if opt.useUnity
    client = unity_link('connect', p);
    if isempty(client), error("Unity에 연결할 수 없습니다. Unity에서 Play를 먼저 누르세요."); end
    usedUnity = true;
    % 끝나면(정상 종료, 오류, Ctrl+C 모두) 연결을 반드시 닫음.
    % Unity(CarStateReceiver)는 한 번에 한 연결만 받으므로, 연결이 남아 있으면 다음 실행의 상태를 Unity가 읽지 않음
    unityGuard = onCleanup(@() releaseUnity(client));
    if client.NumBytesAvailable > 0, read(client, client.NumBytesAvailable); end   % 이전 키 버림
    unity_link('reset', client); unity_link('path', client, ref, 0.05);
    nSent = nSent + unity_link('state', client, st0);
    fprintf('Unity 연결됨 (%s:%d): PP_PID 차를 Unity로 보냅니다\n', p.unityIP, p.unityPort);
end
if opt.input == "keyboard"
    fprintf('Unity Game 창 클릭 → ↑를 누른 채 Space로 출발, Space 한 번 = 한 스텝, Q 종료\n');
end

%% 3. 화면: 왼쪽 맵 + 표시 체크박스 + 파라미터 슬라이더, 오른쪽 ① 조향각 추종 ② 누적 MSE ③ 궤적 오차
fig = figure('Name', "PP_PID (" + opt.input + ") - delta = Lowpass(K_pp delta_pp + delta_p + delta_i + delta_d)", ...
    'NumberTitle', 'off', 'Position', [40 40 1500 900], 'Visible', matlab.lang.OnOffSwitchState(opt.showFigure));
showTune = opt.showFigure && opt.liveTune;
if showTune
    ax = axes(fig, 'Position', [0.035 0.39 0.55 0.53]);   % 아래: 맵 범례, 표시 체크박스, 파라미터 슬라이더
    yChk = 0.255;
else
    ax = axes(fig, 'Position', [0.035 0.17 0.55 0.75]);
    yChk = 0.035;
end
draw_map(ax, p);
hRefPath = plot(ax, ref.x, ref.y, 'y-', 'LineWidth', 1.5);
hTr = gobjects(1, nC);
hTr(1) = plot(ax, NaN, NaN, '-', 'Color', col(1, :), 'LineWidth', 1.3);
hTr(2) = plot(ax, NaN, NaN, '-', 'Color', col(2, :), 'LineWidth', 1.3);
hTr(3) = plot(ax, NaN, NaN, '.', 'Color', col(3, :), 'MarkerSize', 8);
hLook  = plot(ax, NaN, NaN, '--', 'Color', col(3, :));
hCar   = plot(ax, st0.x, st0.y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 7);
legend(ax, [hRefPath, hTr], {'레퍼런스 경로', 'PP', 'PID', 'PP\_PID (Unity)'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal');
hTitle = title(ax, '');

axA = axes(fig, 'Position', [0.64 0.725 0.345 0.235]); hold(axA, 'on'); grid(axA, 'on');
yline(axA, 0, 'k-', 'HandleVisibility', 'off');
hA = gobjects(1, 3);
hA(1) = plot(axA, NaN, NaN, '-', 'Color', cYel, 'LineWidth', 2.0);
hA(2) = plot(axA, NaN, NaN, '-', 'Color', cRed, 'LineWidth', 1.2);
hA(3) = plot(axA, NaN, NaN, '-', 'Color', cOra, 'LineWidth', 1.2);
hLegA = legend(axA, hA, {'목표 조향각 \delta_{ref} = atan(L\kappa)', '현재 조향각 \delta', ...
    '\delta_{ref} - \delta'}, 'Location', 'northeast');
ylabel(axA, '[deg]'); title(axA, 'Tracking error (PP\_PID): 목표 조향각 - 현재 조향각');

axB = axes(fig, 'Position', [0.64 0.405 0.345 0.235]); hold(axB, 'on'); grid(axB, 'on');
hB = gobjects(1, nC);
for c = 1:nC, hB(c) = plot(axB, NaN, NaN, '-', 'Color', col(c, :), 'LineWidth', 1.5); end
hLegB = legend(axB, hB, cellstr(texNames), 'Location', 'northeast');
ylabel(axB, 'MSE [deg^2]'); title(axB, 'PP\_PID 유효성: (\delta_{ref} - \delta)의 누적 MSE');

axC = axes(fig, 'Position', [0.64 0.085 0.345 0.235]); hold(axC, 'on'); grid(axC, 'on');
yline(axC, 0, 'k-', 'HandleVisibility', 'off');
hC = gobjects(1, nC);
for c = 1:nC, hC(c) = plot(axC, NaN, NaN, '-', 'Color', col(c, :), 'LineWidth', 1.2); end
hLegC = legend(axC, hC, cellstr(texNames), 'Location', 'northeast');
xlabel(axC, 't = kT_s [s]'); ylabel(axC, '0 - e_{lat} [m]');
title(axC, 'Trajectory tracking error: 레퍼런스 경로 - 주행 경로');
axR = [axA, axB, axC];                                % 세 그래프의 시간축은 updatePlots에서 같이 맞춤

% 표시 켜기·끄기: 체크박스 또는 그림 창에서 키 (이름, 키, 그래픽 핸들, 글자색)
if opt.showFigure
    groups = { ...
        '레퍼런스 경로 (R)', 'r', hRefPath,                             [0.75 0.60 0.00]; ...
        'PP (1)',            '1', [hTr(1), hB(1), hC(1)],                col(1, :); ...
        'PID (2)',           '2', [hTr(2), hB(2), hC(2)],                col(2, :); ...
        'PP_PID (3)',        '3', [hTr(3), hCar, hB(3), hC(3)],          col(3, :); ...
        '선행점 (L)',        'l', hLook,                                 col(3, :); ...
        '목표 조향각 (Y)',   'y', hA(1),                                 [0.75 0.60 0.00]; ...
        '현재 조향각 (D)',   'd', hA(2),                                 cRed; ...
        '조향각 오차 (E)',   'e', hA(3),                                 cOra};
    makeDisplayChecks(fig, groups, yChk);
end
% 파라미터 조절 패널: 콜백은 값을 그림의 UserData에 적어 두기만 하고, 메인 루프가 다음 스텝 시작에 읽어 적용
if showTune
    makeTunePanel(fig, prm, opt, tuneNames, Ts, p.vMax);
end

%% 4. 메인 루프: t → t+1
n = 0;
paced = opt.input == "auto" && logical(opt.realtime);   % 자동 버전 실시간 진행
tLoop = tic;
tNext = 0;                                       % 다음 스텝이 끝나야 할 시각 [s] (실시간 진행)
nLate = 0;                                       % 실시간을 못 따라간 스텝 수 (진단용)
tDraw = -inf;                                    % 마지막으로 그림을 그린 시각 [s]
drawPeriod = 0.2;                                % 실시간: 그림 간격 [s] (그리는 데 걸린 시간의 3배 이상으로 자동 조절)
for k = 1:N
    % (0) 키보드면 Unity에서 STEP이 올 때까지 기다림 (기다리는 동안에도 슬라이더·체크박스 사용 가능)
    if opt.input == "keyboard"
        cmd = "";
        while ~startsWith(cmd, "STEP") && ~startsWith(cmd, "QUIT")
            if client.NumBytesAvailable > 0
                cmd = strtrim(string(readline(client)));
            else
                drawnow;                                  % 기다리는 동안 그림 창 이벤트 처리
                pause(0.005);
            end
        end
        if startsWith(cmd, "QUIT"), break; end
    end

    % (0a) 슬라이더·입력칸에서 바꾼 값 → 이번 스텝부터 적용
    if showTune && isvalid(fig) && fig.UserData.dirty
        ud = fig.UserData;
        for j = 1:numel(tuneNames), opt.(tuneNames(j)) = ud.tune.(tuneNames(j)); end
        if opt.sumToOne, opt.Kpp = 1 - opt.Kp - opt.Ki - opt.Kd; end
        G = makeGains();
        ud.dirty = false; fig.UserData = ud;
    end
    PRM(k, :) = arrayfun(@(nm) opt.(nm), tuneNames);       % 이 스텝에 쓴 튜닝 값

    % (0b) 가속도 a[t]
    if opt.input == "keyboard"
        kk = split(cmd, ',');                                 % "STEP,↑,↓"
        up = numel(kk) >= 2 && kk(2) == "1"; down = numel(kk) >= 3 && kk(3) == "1";
        aReq = p.accel * (up && ~down) - p.brake * (down && ~up);
    else
        aReq = (opt.v - car(iB).v) / Ts;
    end

    % (1)~(3) 세 제어기: 조향각 → omega → 이산 모델 x[t+1]
    for c = 1:nC
        if c == 2
            [dCmd, mem{c}, d] = pid_steer(car(c), ref, p, G{c}, mem{c}, Ts);
        else
            [dCmd, mem{c}, d] = blended_steer(car(c), ref, p, G{c}, mem{c}, Ts);
        end
        omega = (dCmd - car(c).delta) / Ts;               % 조향 인테그레이터 입력
        [car(c), used] = bicycle_discrete_step(car(c), [aReq; omega], p, Ts);
        DREF(k, c) = d.dRef; DCMD(k, c) = d.dCmd; ESTEER(k, c) = d.eSteer; ETRAJ(k, c) = d.eLat;
        DPP(k, c) = d.dPP; DP(k, c) = d.dP; DI(k, c) = d.dI; DD(k, c) = d.dD; DRAW(k, c) = d.dRaw;
        OMEGA(k, c) = used(2);
        X(k + 1, c) = car(c).x; Y(k + 1, c) = car(c).y; YAW(k + 1, c) = car(c).yaw; DEL(k + 1, c) = car(c).delta;
        if c == iB
            dB = d; A(k) = used(1); LX(k) = d.xl; LY(k) = d.yl;
            ELA(k) = d.ela; LAD(k) = d.Lad; EHEAD(k) = d.eHead; SE(k) = d.Se; DE(k) = d.dE;
        end
    end
    S(k + 1) = car(iB).s; V(k + 1) = car(iB).v;
    n = k;

    % (4) Unity로 PP_PID 차 전송 (횡오차, 투영점, 선행점 포함)
    if ~isempty(client)
        eN = path_error(ref, car(iB).x, car(iB).y, car(iB).yaw, mem{iB}.hint);
        dbgU = struct('x_w', eN.x_proj, 'y_w', eN.y_proj, 'x_d', NaN, 'y_d', NaN, ...
            'x_l', dB.xl, 'y_l', dB.yl, 'x_c', NaN, 'y_c', NaN, 'R', NaN, 'Lf', opt.Lf, ...
            'tau', NaN, 'delta_cmd', dB.dCmd);
        if unity_link('state', client, car(iB), eN, dbgU)
            nSent = nSent + 1;
        else
            lost = true;
            fprintf('Unity로 보내지 못했습니다 (k = %d). Unity Play가 멈췄는지 확인하세요. 지금까지 결과를 저장합니다.\n', k);
            break;
        end
    end

    % (5) 화면 갱신
    %   키보드: 매 스텝 / 실시간: 0.2 s 이상 간격 (그리는 시간의 3배) / 빠른 실행: 5스텝마다
    %   키보드·실시간은 drawnow(전체)를 씀: 이때 MATLAB이 슬라이더·체크박스 이벤트를 처리함.
    %   (drawnow limitrate + pause만으로는 이 그림에서 이벤트가 몇 초씩 밀리는 것을 확인함.
    %    전체 drawnow는 매 스텝 하면 실시간을 못 따라가므로 간격을 둠. Unity에는 매 스텝 보냄)
    if opt.showFigure && isvalid(fig)
        if opt.input == "keyboard" || (paced && (k == 1 || toc(tLoop) - tDraw >= drawPeriod))
            tD0 = toc(tLoop);
            updatePlots(k);
            set(hLook, 'XData', [X(k, iB) dB.xl], 'YData', [Y(k, iB) dB.yl]);
            drawnow;
            tDraw = toc(tLoop);
            drawPeriod = max(0.2, 3 * (tDraw - tD0));   % 그리기가 전체 시간의 1/3을 넘지 않게 (실시간 유지)
        elseif ~paced && mod(k, 5) == 0
            updatePlots(k);
            set(hLook, 'XData', [X(k, iB) dB.xl], 'YData', [Y(k, iB) dB.yl]);
            drawnow limitrate;
        end
    end

    % (6) 실시간: 스텝과 스텝 사이가 Ts가 되도록 기다림 (늦으면 몰아서 따라잡지 않음)
    if paced
        tNext = tNext + Ts;
        wait = tNext - toc(tLoop);
        if wait <= 0
            nLate = nLate + 1;
            tNext = toc(tLoop);
        end
        pause(max(wait, 1e-3));
    end

    % (7) 테스트용 훅: 스텝 k가 끝난 뒤 부름 (check_live_tuning이 이 시점에 패널을 조작 → 다음 스텝부터 적용)
    if ~isempty(opt.stepHook) && isvalid(fig), opt.stepHook(fig, k); end

    % 자동 버전: 한 바퀴 돌면 끝
    if opt.input == "auto" && car(iB).s >= ref.L, break; end
end
loopTime = toc(tLoop);                           % 루프에 걸린 실제 시간 [s] (저장 시간 제외)
lapDone = S(n + 1) >= ref.L;                     % 한 바퀴(41.07 m)를 다 돌았는지
if usedUnity, delete(unityGuard); end            % Unity 연결 닫기 → 다음 실행을 Unity가 바로 받음

%% 5. 지표
M = struct('name', cellstr(names));
for c = 1:nC
    es = ESTEER(1:n, c); et = ETRAJ(1:n, c);
    M(c).mse_steer     = mean(es.^2);                       % [rad^2]
    M(c).rms_steer_deg = rad2deg(sqrt(M(c).mse_steer));
    M(c).max_steer_deg = rad2deg(max(abs(es)));
    M(c).mse_traj      = mean(et.^2);                       % [m^2]
    M(c).rms_traj      = sqrt(M(c).mse_traj);
    M(c).max_traj      = max(abs(et));
    M(c).rms_omega_deg = rad2deg(sqrt(mean(OMEGA(1:n, c).^2)));   % 조향 속도 RMS [deg/s] (부드러움)
end
chg = strings(0, 1);                             % 실행 중 슬라이더로 바꾼 파라미터
for j = 1:numel(tuneNames)
    if n == 0, break; end
    pj = PRM(1:n, j);
    kf = find(pj ~= pj(1), 1);
    if ~isempty(kf)
        chg(end + 1, 1) = sprintf('%s: %.4g -> %.4g (처음 바뀐 스텝 k = %d)', tuneNames(j), pj(1), pj(n), kf - 1); %#ok<AGROW>
    end
end

%% 6. 데이터 저장 (timeseries.csv, summary.txt) — 그림보다 먼저
%   그림 저장(7)이 실패하거나 중단돼도 (저장 중 그림 창을 닫음, Ctrl+C 등) 데이터는 남도록 데이터를 먼저 저장
folder = "";
if opt.save
    folder = fullfile(base, 'results', string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_blended_discrete_" + opt.input);
    mkdir(folder);
end

tt = (0:n - 1)' * Ts;
T = table((0:n - 1)', tt, S(1:n), V(1:n), A(1:n), 'VariableNames', {'k', 't_s', 's_m', 'v_mps', 'a_mps2'});
for c = 1:nC
    kc = tags(c);
    T.(kc + "_x_m")           = X(1:n, c);
    T.(kc + "_y_m")           = Y(1:n, c);
    T.(kc + "_yaw_rad")       = YAW(1:n, c);
    T.(kc + "_delta_rad")     = DEL(1:n, c);
    T.(kc + "_delta_ref_rad") = DREF(1:n, c);
    T.(kc + "_delta_cmd_rad") = DCMD(1:n, c);
    T.(kc + "_omega_radps")   = OMEGA(1:n, c);
    T.(kc + "_e_steer_rad")   = ESTEER(1:n, c);
    T.(kc + "_e_traj_m")      = ETRAJ(1:n, c);       % e = 0 - e_lat (PI의 횡오차와 같은 값)
end
T.bl_delta_pp_rad  = DPP(1:n, iB);               % δ_pp (Eq.3)
T.bl_delta_p_rad   = DP(1:n, iB);                % δ_p = K_p e_la (Eq.4)
T.bl_delta_i_rad   = DI(1:n, iB);                % δ_i = K_i Σe (Eq.7)
T.bl_delta_d_rad   = DD(1:n, iB);                % δ_d = K_d (e[k] - e[k-1]) / T_s
T.bl_delta_raw_rad = DRAW(1:n, iB);              % K_pp δ_pp + δ_p + δ_i + δ_d (Low pass 전)
T.bl_de_mps        = DE(1:n);                    % (e[k] - e[k-1]) / T_s
T.bl_e_la_m        = ELA(1:n);                   % e_la = e + (a + L_ad) sin Δψ (Eq.5)
T.bl_e_head_rad    = EHEAD(1:n);                 % Δψ = ψ_path - θ
T.bl_L_ad_m        = LAD(1:n);                   % 차와 선행점 사이 거리
T.bl_sum_e_m       = SE(1:n);                    % Σ e
T.bl_lookahead_x_m = LX(1:n);                    % PP 선행점 (경로를 따라 투영점 + Lf)
T.bl_lookahead_y_m = LY(1:n);
for j = 1:numel(tuneNames)
    T.("p_" + tuneNames(j)) = PRM(1:n, j);       % 스텝마다 쓴 튜닝 값
end
if opt.save
    writetable(T, fullfile(folder, 'timeseries.csv'));
    writeSummary(fullfile(folder, 'summary.txt'));
end

%% 7. 그림 마무리 + result.png 저장 (실패해도 경고만: 데이터는 6에서 이미 저장됨)
if isvalid(fig) && n > 0
    try
        updatePlots(n);
        set(hLook, 'XData', NaN, 'YData', NaN);
        hLegB.String = arrayfun(@(c) sprintf('%s: MSE %.2e rad^2 (%.2f deg^2), RMS %.2f deg', texNames(c), M(c).mse_steer, ...
            M(c).mse_steer * (180 / pi)^2, M(c).rms_steer_deg), 1:nC, 'UniformOutput', false);
        hLegC.String = arrayfun(@(c) sprintf('%s: MSE %.2e m^2, RMS %.4f m, 최대 %.3f m', texNames(c), M(c).mse_traj, ...
            M(c).rms_traj, M(c).max_traj), 1:nC, 'UniformOutput', false);
        hLegA.Location = 'best'; hLegB.Location = 'best'; hLegC.Location = 'best';
        if opt.save, exportgraphics(fig, fullfile(folder, 'result.png'), 'Resolution', 110); end
    catch ME
        warning('blended_discrete:figureSave', ...
            '그림(result.png)을 저장하지 못했습니다: %s\n  데이터(timeseries.csv, summary.txt)는 저장되어 있습니다: %s', ...
            ME.message, folder);
    end
elseif opt.save && n > 0
    fprintf('그림 창이 닫혀 있어 result.png는 저장하지 않았습니다 (timeseries.csv, summary.txt는 저장됨).\n');
end
if ~opt.showFigure && isvalid(fig), close(fig); end

result.folder  = folder;
result.steps   = n;
result.opt     = opt;                            % 끝났을 때 값 (슬라이더로 바꿨으면 바뀐 값)
result.prm     = prm;                            % 시작할 때 값
result.paramChanges = chg;
result.unity   = struct('used', usedUnity, 'sent', nSent, 'lost', lost);
result.distance = S(n + 1);                      % 달린 거리 [m]
result.lapDone  = lapDone;                       % 한 바퀴 완주 여부
result.lateSteps = nLate;                        % 실시간 진행에서 늦은 스텝 수
result.wallTime = loopTime;                      % 루프에 걸린 실제 시간 [s] (그림 저장 시간 제외)
result.metrics = M;
result.pp = M(1); result.pid = M(2); result.pppid = M(3); result.blended = M(3);
result.timeseries = T;
fprintf('PP_PID (%s, v %.2f m/s, Lf %.2f m, K_pp %.3g, Kp %.3g, Ki %.3g, Kd %.3g, a %.2f m, ell %d, w_cur %.2f, Ts %.2f s), %d 스텝\n', ...
    opt.input, prm.v, prm.Lf, prm.Kpp, prm.Kp, prm.Ki, prm.Kd, prm.a, prm.ell, prm.wcur, Ts, n);
if ~isempty(chg), fprintf('  실행 중 바꾼 파라미터: %s\n', strjoin(chg, ' / ')); end
if usedUnity, fprintf('  Unity로 보낸 상태: %d 줄%s\n', nSent, string(ternary(lost, ' (연결 끊김)', ''))); end
for c = 1:nC
    fprintf('  %-6s 조향각 오차 MSE %.2e rad^2 (RMS %.2f deg)   궤적 오차 MSE %.2e m^2 (RMS %.4f m, 최대 %.3f m)\n', ...
        M(c).name, M(c).mse_steer, M(c).rms_steer_deg, M(c).mse_traj, M(c).rms_traj, M(c).max_traj);
end
if opt.save, fprintf('결과: %s\n', folder); end

    function updatePlots(m)
        % 1 ~ m 스텝까지 그림 갱신 (시각 t = (0 ~ m-1) Ts)
        tk = (0:m - 1)' * Ts;
        for cc = 1:nC
            set(hTr(cc), 'XData', X(1:m + 1, cc), 'YData', Y(1:m + 1, cc));
        end
        set(hCar, 'XData', X(m + 1, iB), 'YData', Y(m + 1, iB));
        set(hA(1), 'XData', tk, 'YData', rad2deg(DREF(1:m, iB)));
        set(hA(2), 'XData', tk, 'YData', rad2deg(DEL(1:m, iB)));
        set(hA(3), 'XData', tk, 'YData', rad2deg(ESTEER(1:m, iB)));
        cm = cumsum(ESTEER(1:m, :).^2, 1) ./ (1:m)' * (180 / pi)^2;   % 누적 MSE [deg^2]
        for cc = 1:nC
            set(hB(cc), 'XData', tk, 'YData', cm(:, cc));
            set(hC(cc), 'XData', tk, 'YData', ETRAJ(1:m, cc));
        end
        set(axR, 'XLim', [0, max(tk(end), Ts)]);
        hTitle.String = {sprintf('PP\\_PID   k = %d   t = %.1f s   v = %.2f m/s   \\delta = %.1f deg   e_{la} = %.1f cm   0 - e_{lat} = %.1f cm', ...
            m, m * Ts, V(m + 1), rad2deg(DEL(m + 1, iB)), 100 * ELA(m), 100 * ETRAJ(m, iB)), ...
            sprintf('v_{tar} %.2f  L_f %.2f  a %.2f  K_{pp} %.3g  K_p %.3g  K_i %.3g  K_d %.3g  ℓ %d  w_{cur} %.2f', ...
            opt.v, opt.Lf, opt.a, opt.Kpp, opt.Kp, opt.Ki, opt.Kd, opt.ell, opt.wcur)};
    end

    function Gs = makeGains()
        % 지금 opt 값으로 세 제어기 게인 (PP: 순수 PP / PID: 비교용 횡오차 PID / PP_PID: 논문 식 + D)
        ellO = 1; wO = 1;
        if opt.lpfAll, ellO = opt.ell; wO = opt.wcur; end
        Gs = cell(1, 3);
        Gs{1} = struct('Kpp', 1, 'Kp', 0, 'Ki', 0, 'Kd', 0, 'Lf', opt.Lf, 'a', opt.a, 'ell', ellO, 'wcur', wO);
        Gs{2} = struct('Kp', opt.pidKp, 'Ki', opt.pidKi, 'Kd', opt.pidKd, 'ell', ellO, 'wcur', wO);
        Gs{3} = struct('Kpp', opt.Kpp, 'Kp', opt.Kp, 'Ki', opt.Ki, 'Kd', opt.Kd, 'Lf', opt.Lf, 'a', opt.a, ...
            'ell', opt.ell, 'wcur', opt.wcur);
    end

    function writeSummary(file)
        fid = fopen(file, 'w', 'n', 'UTF-8');
        fprintf(fid, 'PP_PID (blended_discrete) - %s, %s\n', opt.input, string(datetime('now')));
        fprintf(fid, '기준 논문: Chen et al., Optimization of Pure Pursuit Controller based on PID Controller and Low-pass Filter, ITSC 2018 (PP_PI) + D 항\n');
        fprintf(fid, '모델: bicycle_discrete_step (조향 인테그레이터 + 전진 더블 인테그레이터 + 기구학 자전거), Ts = %.3f s, L = %.2f m\n', Ts, L);
        fprintf(fid, '제어: delta = Lowpass(K_pp delta_pp + K_p e_la + K_i sum(e) + K_d (e[k] - e[k-1]) / Ts),  e_la = e + (a + L_ad) sin(dpsi)\n');
        fprintf(fid, '  시작 값: Lf = %.2f m, K_pp = %.4g, Kp = %.4g, Ki = %.4g, Kd = %.4g (합 %.4g, sumToOne %d), a = %.2f m\n', ...
            prm.Lf, prm.Kpp, prm.Kp, prm.Ki, prm.Kd, prm.Kpp + prm.Kp + prm.Ki + prm.Kd, prm.sumToOne, prm.a);
        fprintf(fid, '  Low-pass Filter: 창 길이 ell = %d, w_current = %.3f, w_previous = %.3f\n', prm.ell, prm.wcur, ...
            (1 - prm.wcur) / max(prm.ell - 1, 1));
        fprintf(fid, '  비교 PID: Kp %.3g, Ki %.3g, Kd %.3g (e = -e_lat), Low pass 비교 제어기에도: %d\n', ...
            prm.pidKp, prm.pidKi, prm.pidKd, prm.lpfAll);
        if opt.input == "auto", fprintf(fid, '속도: 자동 %.2f m/s\n', prm.v); else, fprintf(fid, '속도: 키보드\n'); end
        if isempty(chg)
            fprintf(fid, '실행 중 바꾼 파라미터: 없음\n');
        else
            fprintf(fid, '실행 중 바꾼 파라미터 (지표는 바뀌기 전후가 섞인 값, 스텝별 값은 timeseries.csv의 p_* 열):\n');
            fprintf(fid, '  %s\n', chg);
        end
        if usedUnity, fprintf(fid, 'Unity: 상태 %d 줄 전송%s\n', nSent, string(ternary(lost, ' (도중에 연결 끊김)', ''))); end
        fprintf(fid, '스텝: %d (%.1f s), 주행 거리 %.2f m (한 바퀴 %.2f m, %s)\n\n', n, n * Ts, S(n + 1), ref.L, ...
            string(ternary(lapDone, '완주', '한 바퀴 전에 끝남')));
        fprintf(fid, '%-8s | %-38s | %-34s | %s\n', '', '조향각 오차 e = delta_ref - delta', '궤적 오차 0 - e_lat', '조향 속도');
        fprintf(fid, '%-8s | %12s %10s %12s | %12s %9s %9s | %s\n', '제어기', 'MSE[rad^2]', 'RMS[deg]', '최대[deg]', ...
            'MSE[m^2]', 'RMS[m]', '최대[m]', 'RMS[deg/s]');
        for cc = 1:nC
            fprintf(fid, '%-8s | %12.3e %10.3f %12.3f | %12.3e %9.4f %9.4f | %8.2f\n', M(cc).name, M(cc).mse_steer, ...
                M(cc).rms_steer_deg, M(cc).max_steer_deg, M(cc).mse_traj, M(cc).rms_traj, M(cc).max_traj, M(cc).rms_omega_deg);
        end
        fclose(fid);
    end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

function releaseUnity(client)
% Unity 연결 닫기 (onCleanup에서 부름: 정상 종료, 오류, Ctrl+C 모두)
try
    if ~isempty(client) && isvalid(client), delete(client); end
catch
end
end

function makeDisplayChecks(fig, groups, y0)
% 맵 아래 표시 체크박스 두 줄 (각 4개). 그림 창에서 키로도 켜고 끔. 콜백은 이 함수 밖의 변수를 잡지 않음
setappdata(fig, 'dispGroups', groups);
nG = size(groups, 1);
h = gobjects(1, nG);
for gi = 1:nG
    row = floor((gi - 1) / 4); colk = mod(gi - 1, 4);
    h(gi) = uicontrol(fig, 'Style', 'checkbox', 'String', groups{gi, 1}, 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.045 + 0.135 * colk, y0 + 0.035 * (1 - row), 0.13, 0.03], ...
        'ForegroundColor', groups{gi, 4}, 'FontWeight', 'bold', 'Tag', "show_" + gi, ...
        'Callback', @(src, ~) setDisplay(ancestor(src, 'figure'), gi, src.Value));
end
setappdata(fig, 'dispChecks', h);
fig.KeyPressFcn = @displayKey;
end

function setDisplay(fig, gi, on)
% gi번째 그룹 보이기/숨기기 (체크박스 상태도 맞춤)
groups = getappdata(fig, 'dispGroups');
h = getappdata(fig, 'dispChecks');
set(groups{gi, 3}, 'Visible', matlab.lang.OnOffSwitchState(logical(on)));
h(gi).Value = logical(on);
end

function displayKey(fig, evt)
% 그림 창 키: R, 1, 2, 3, L, Y, D, E → 해당 그룹 켜기·끄기
groups = getappdata(fig, 'dispGroups');
h = getappdata(fig, 'dispChecks');
gi = find(strcmpi(groups(:, 2), evt.Key), 1);
if ~isempty(gi), setDisplay(fig, gi, ~h(gi).Value); end
end

function makeTunePanel(fig, prm, opt, tuneNames, Ts, vMax)
% 그림 왼쪽 아래 파라미터 조절 패널 (왼쪽 열 v, Lf, a, wcur, ell / 오른쪽 열 Kp, Ki, Kd, Kpp)
% 콜백은 tuneCallback (값을 fig.UserData에 적기만 함). sumToOne이면 Kpp = 1 - Kp - Ki - Kd로 자동 (직접 못 바꿈)
fig.UserData = struct('tune', struct(), 'dirty', false, 'range', prm.range, 'vMax', vMax, 'sumToOne', opt.sumToOne);
for j = 1:numel(tuneNames), fig.UserData.tune.(tuneNames(j)) = opt.(tuneNames(j)); end
kppLabel = 'Kpp PP 게인';
if opt.sumToOne, kppLabel = 'Kpp = 1-Kp-Ki-Kd'; end
tuneLabel = struct('v', 'v 목표 속도 [m/s]', 'Lf', 'Lf 선행점 거리 [m]', 'a', 'a (e_la 식) [m]', ...
    'wcur', 'w_current (LPF)', 'Kp', 'Kp  P 게인 [rad/m]', 'Ki', 'Ki  I 게인 [rad/m]', ...
    'Kd', 'Kd  D 게인 [rad s/m]', 'Kpp', kppLabel, 'ell', 'ell LPF 창 길이');
pnl = uipanel(fig, 'Units', 'normalized', 'Position', [0.035 0.005 0.55 0.235], 'FontSize', 9, ...
    'Title', sprintf('파라미터 조절: 슬라이더 또는 입력칸(Enter), 다음 스텝부터 적용  (v는 자동 버전, Ts = %.2f s 고정)', Ts));
for j = 1:numel(tuneNames)
    nm = char(tuneNames(j));
    colj = double(j > 5);                              % 0: 왼쪽 열 (5개), 1: 오른쪽 열 (4개)
    row  = j - 5 * colj;                               % 1 ~ 5
    x0 = 0.01 + 0.5 * colj; y0 = 1 - 0.193 * row;
    rg = prm.range.(nm);
    enable = 'on';
    if strcmp(nm, 'Kpp') && opt.sumToOne, enable = 'off'; end
    uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', 'Position', [x0, y0, 0.19, 0.13], ...
        'String', tuneLabel.(nm), 'HorizontalAlignment', 'left');
    hs = uicontrol(pnl, 'Style', 'slider', 'Units', 'normalized', 'Position', [x0 + 0.19, y0 + 0.015, 0.21, 0.115], ...
        'Min', rg(1), 'Max', rg(2), 'Value', min(max(opt.(nm), rg(1)), rg(2)), 'Tag', "tune_" + nm, ...
        'Enable', enable, 'Callback', @(src, ~) tuneCallback(src, nm, false));
    if strcmp(nm, 'ell'), hs.SliderStep = [1, 1] / (rg(2) - rg(1)); end   % 정수 단위
    uicontrol(pnl, 'Style', 'edit', 'Units', 'normalized', 'Position', [x0 + 0.41, y0 + 0.005, 0.07, 0.135], ...
        'String', sprintf('%.4g', opt.(nm)), 'Tag', "tuneEdit_" + nm, 'Enable', enable, ...
        'Callback', @(src, ~) tuneCallback(src, nm, true));
end
end

function tuneCallback(src, nm, fromEdit)
% 슬라이더·입력칸 콜백: 값 확인 → fig.UserData.tune에 적고 dirty 표시 (메인 루프가 다음 스텝 시작에 적용)
fig = ancestor(src, 'figure');
ud  = fig.UserData;
if fromEdit, val = str2double(src.String); else, val = src.Value; end
if strcmp(nm, 'ell'), val = round(val); end
lo = 0; hi = inf;
switch nm
    case 'Lf',   lo = 0.05;
    case 'v',    hi = ud.vMax;
    case 'wcur', lo = 1e-3; hi = 1;
    case 'ell',  lo = 1; hi = 21;
end
newTune = ud.tune;
newTune.(nm) = val;
if ud.sumToOne && any(strcmp(nm, {'Kp', 'Ki', 'Kd'}))
    newTune.Kpp = 1 - newTune.Kp - newTune.Ki - newTune.Kd;   % 합 = 1: Kpp + Kp + Ki + Kd = 1
end
if ~isfinite(val) || val < lo || val > hi || newTune.Kpp < 0
    showTuneValue(fig, nm, ud.tune.(nm), ud.range.(nm));   % 잘못된 값이면 되돌림
    return;
end
ud.tune = newTune;
ud.dirty = true;
fig.UserData = ud;
showTuneValue(fig, nm, val, ud.range.(nm));
if ud.sumToOne, showTuneValue(fig, 'Kpp', newTune.Kpp, ud.range.Kpp); end
end

function showTuneValue(fig, nm, val, rg)
% 슬라이더·입력칸 표시를 val로 맞춤
hS = findobj(fig, 'Tag', "tune_" + nm);
hE = findobj(fig, 'Tag', "tuneEdit_" + nm);
hS.Value  = min(max(val, rg(1)), rg(2));
hE.String = sprintf('%.4g', val);
end
