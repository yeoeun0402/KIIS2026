function result = twin_drive(varargin)
% [twin] Unity 시뮬레이션 + 실차를 같은 키로 동시에 수동 주행 (실시간)
%
% 흐름 (TCP):
%   Unity Game 창 키 (TwinInput.cs) ──55001──▶ MATLAB (이 함수) ──55004──▶ 실차 노드 (~/demo/drive.sh --twin)
%                                                 │                              └─▶ 아두이노 + C920 녹화 → localization
%                                                 └──55001──▶ Unity (차 위치·궤적 표시)
%   MATLAB은 받은 키를 실차에 보내고, 실차가 실제로 적용한 키·속도(APPLY,<key>,<speed>)를 돌려받아
%   그것으로 시뮬레이션 차(twin_model_step)를 움직임 → 시뮬과 실차의 속도 조절이 같은 값(실차 PWM)을 따름.
%   실차 터미널에서 누른 W/S/Space와 안전 정지도 시뮬에 똑같이 들어감.
%   (실차 노드가 echo를 지원하지 않거나 실차 없이 돌리면 예전처럼 받은 키로 바로 시뮬을 움직임)
%
% 실행 순서:
%   1) 터미널:  cd ~/demo && ./drive.sh --twin           (실차 노드가 55004에서 MATLAB을 기다림)
%   2) Unity:   bash ~/version__1/run_unity.sh → SampleScene → Play
%   3) MATLAB:  cd ~/version__1/MATLAB; setup_paths; twin_drive
%   4) Unity Game 창 클릭 → W/S 속도, A/D 조향, Space 정지, Q 종료 (실차 터미널에서도 Space/Q 동작)
%
% 옵션 (이름-값):
%   'useCar'   true   false면 실차 없이 시뮬레이션만 (연결 시험용)
%   'useUnity' true   false면 Unity 없이 (키는 'keys' 옵션으로만, 시험용)
%   'useDash'  true   웹 대시보드(dashboard/twin_dash.py, TCP 55005)가 켜져 있으면 연결: 시뮬 위치를 보내고
%                     무게(MASS,<kg>)·키(KEY,<k>)를 받음. 없으면 그냥 건너뜀
%   'keys'     []     시험용 키 목록: [t_s, 키] 문자열 배열 N x 2 (예: ["0.5","w"; "1.0","a"]) — 주면 Unity 키 대신 이걸 씀
%   그 밖의 이름은 twin_params의 값을 덮어씀 (예: 'kv', 0.012)
%
% 결과: ~/version__1/results/<날짜_시간>_twin/
%   timeseries.csv  (주기 T마다: 시각, 시뮬 좌표, 트랙 좌표, 속도, 조향각, PWM, 횡오차 …)
%   keys.csv        (키와 시각, src = unity(Unity에서 받음) | car(실차가 적용 → 시뮬에 들어감) | sim(실차 없이 바로 시뮬))
%   summary.json    (실차 세션 폴더, 파라미터, 요약 숫자) — analysis/compare_twin.py가 이걸로 실차 기록을 찾음
%   trajectory_track.png (MATLAB 그림: 트랙 좌표 궤적 + 2차선 중심선)
%   unity_top.png / unity_top.json (Unity Top 화면: 맵 + 지나온 궤적, 1 px = 1 cm, 트랙 방향)

%% 1. 옵션
opt.useCar = true;
opt.useUnity = true;
opt.useDash = true;
opt.keys = strings(0, 2);
tpArgs = {};
for k = 1:2:numel(varargin)
    if isfield(opt, varargin{k}), opt.(varargin{k}) = varargin{k + 1};
    else, tpArgs(end + 1:end + 2) = varargin(k:k + 1); end %#ok<AGROW>
end
tp = twin_params(tpArgs{:});
p = vehicle_params();
refSim = twin_reference('sim');
scripted = ~isempty(opt.keys);
if ~opt.useUnity && ~scripted
    error("Unity 없이 돌리려면 'keys'로 시험용 키를 주세요.");
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));     % ~/version__1
runName = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_twin";
folder = fullfile(root, 'results', runName);
mkdir(folder);

%% 2. 연결: 실차 먼저 (HELLO로 step·maxSpeed·세션 폴더 받기), 그다음 Unity
car = []; carInfo = struct('session', "", 't0_epoch', NaN, 'hello', "", 'echo', false);
if opt.useCar
    try
        car = tcpclient(tp.carIP, tp.carPort, "ConnectTimeout", 3);
    catch
        error(['실차 노드에 연결할 수 없습니다 (%s:%d). 먼저 터미널에서 cd ~/demo && ./drive.sh --twin 을 실행하세요.\n' ...
               '실차 없이 시험하려면 twin_drive(''useCar'', false)'], tp.carIP, tp.carPort);
    end
    t1 = tic;
    while car.NumBytesAvailable == 0 && toc(t1) < 3, pause(0.02); end
    if car.NumBytesAvailable > 0
        hello = strtrim(string(readline(car)));
        carInfo.hello = hello;
        kv = parseHello(hello);
        if isfield(kv, 'step'), tp.step = str2double(kv.step); end
        if isfield(kv, 'max_speed'), tp.maxSpeed = str2double(kv.max_speed); end
        if isfield(kv, 'session'), carInfo.session = string(kv.session); end
        if isfield(kv, 't0_epoch'), carInfo.t0_epoch = str2double(kv.t0_epoch); end
        carInfo.echo = isfield(kv, 'echo') && kv.echo == "1";
        if ~carInfo.echo
            warning('[twin] 실차 노드가 APPLY(적용 키 알림)를 안 보냄 → 받은 키로 바로 시뮬을 움직임 (실차 터미널 키는 시뮬에 안 들어감)');
        end
        fprintf('[twin] 실차 연결: step %d · 최고 PWM %d · 세션 %s\n', tp.step, tp.maxSpeed, carInfo.session);
    else
        warning('[twin] 실차 HELLO를 못 받음 → step %d, 최고 PWM %d 가정', tp.step, tp.maxSpeed);
    end
end
unity = [];
if opt.useUnity
    unity = unity_link('connect', p);
    if isempty(unity)
        if ~isempty(car), writeline(car, "KEY,q"); end
        error("Unity에 연결할 수 없습니다. ~/version__1 프로젝트에서 Play를 먼저 누르세요.");
    end
    unity_link('reset', unity);
    unity_link('path', unity, refSim, 0.05);         % 노란 선: 실차와 같은 기준선 (도면 2차선 중심선)
end
dash = [];
if opt.useDash
    try
        dash = tcpclient("127.0.0.1", tp.dashPort, "ConnectTimeout", 1);
        writeline(dash, sprintf("RUN,%s,%.2f,%s", runName, tp.mass, string(carInfo.session)));
        writeline(dash, sprintf("PARAM,%.6f,%.3f,%.4f,%.4f,%.3f,%d,%d", tp.kv, tp.massRef, tp.crr, tp.tauVRef, ...
            tp.pwmDeadRef, tp.maxSpeed, tp.step));       % 대시보드가 슬라이더를 움직일 때 미리 계산해 보여 주기용
        fprintf('[twin] 웹 대시보드 연결 (127.0.0.1:%d) · 무게 %.1f kg\n', tp.dashPort, tp.mass);
    catch
        dash = [];
    end
end
cleanupObj = onCleanup(@() closeAll());

%% 3. 상태 초기화 (출발 자세 = 실차 출발 자세)
q0 = twin_frames('track2sim', tp.startTrack);
st = struct('t', 0, 'x', q0(1), 'y', q0(2), 'yaw', q0(3), 'v', 0, 'delta', -tp.delta0, 'yawRate', 0, ...   % 출발: 바퀴 곧음 (실제 조향 0)
    'pwmCmd', 0, 'pwm', 0, 'pend', zeros(0, 2), 'steerDir', 0, 'steerLeft', 0, 'vs', 0, 'P', diag(tp.ekf.P0));
cols = {'t_s', 'epoch_s', 'x_sim', 'y_sim', 'yaw_sim_rad', 'x_track', 'y_track', 'theta_track_rad', ...
        'v_mps', 'delta_rad', 'yawrate_radps', 'pwm', 'steer_motor', 'e_lat_m', 's_ref_m', 'mass_kg', ...
        'speed_scale', 'obs_on', 'loc_x_track', 'loc_y_track', 'loc_theta_track', 'inn_lon_m', 'inn_lat_m', 'inn_theta_rad', ...
        'est_sigma_pos_m', 'inn_d2', ...
        'kf_x_track', 'kf_y_track', 'kf_theta_track', 'kf_sigma_pos_m', 'kf_nis', 'kf_flag'};   % ArUco 칼만 (live_loc KF 줄, 이 주기 마지막 값)
obsT = NaN; obsRow = nan(1, 8); obsN = 0; obsInn = zeros(0, 3);   % 추정기: 마지막 측정 시각 · 이번 주기 측정/혁신 · 횟수
kfRow = nan(1, 6);                                               % ArUco 칼만: 이번 주기 마지막 KF 줄
ekfT = 0; steerKeysPending = 0; ekfReject = 0;                    % EKF: 마지막 예측 시각 · 그동안 적용된 조향 키 · 연속 버림 수
data = nan(20000, numel(cols)); n = 0;
keyLog = strings(0, 4);                              % t_s, epoch_s, key, src
followCar = carInfo.echo;                            % true: 시뮬은 실차 APPLY만 따름
lastHud = NaN;
hint = NaN; lost = false; reason = "";

t0 = tic; epoch0 = posixtime(datetime('now', 'TimeZone', 'local'));   % 시뮬 t = 0 인 벽시계 (실차 t0_epoch와 맞춰 비교)
fprintf('[twin] 시작: Unity Game 창 클릭 → W/S 속도 · A/D 조향 · Space 정지 · Q 종료 (결과: %s)\n', folder);
logRow();
sendUnity();

%% 4. 메인 루프 (T마다: 모델 적분 → 키 처리 → 실차 전송 → Unity 전송 → 기록)
tNext = tp.T; iScript = 1; running = true;
while running
    tNow = toc(t0);
    if tNow < tNext
        pause(min(0.005, tNext - tNow));
        continue;
    end
    % 모델: 지난 주기 입력으로 지금까지 적분
    while st.t + 1e-9 < tNow
        st = twin_model_step(st, tp, p, min(tp.dtSub, tNow - st.t));
    end
    % EKF 공분산 예측 (평균은 위에서 모델로 적분)
    if tp.obs.enabled && tp.estimator == "ekf"
        st = twin_ekf_predict(st, tp, p, st.t - ekfT, steerKeysPending);
        steerKeysPending = 0;
    end
    ekfT = st.t;
    % 키
    keys = strings(0, 1);
    if scripted
        while iScript <= size(opt.keys, 1) && str2double(opt.keys(iScript, 1)) <= tNow
            keys(end + 1, 1) = opt.keys(iScript, 2); iScript = iScript + 1; %#ok<AGROW>
        end
        if iScript > size(opt.keys, 1) && isempty(keys) && tNow > str2double(opt.keys(end, 1)) + 0.5
            keys = "q";
        end
    end
    if ~isempty(unity), keys = [keys; pollUnity()]; end %#ok<AGROW>
    if ~isempty(dash), keys = [keys; pollDash()]; end %#ok<AGROW>
    for key = keys'
        if ~isempty(car), sendCar("KEY," + key); end    % 실차에 먼저 (지연 최소)
        if followCar
            logKey(tNow, key, "unity");                   % 시뮬에는 실차 APPLY가 오면 들어감 (readCar)
        else
            logKey(tNow, key, "sim");
        end
        if key == "q"
            running = false; reason = "사용자 종료 (Q)";
            break;
        end
        if ~followCar
            st = twin_key(st, key, tp);
            steerKeysPending = steerKeysPending + any(key == ["a", "d"]);
        end
    end
    if ~isempty(car), sendCar("HB"); readCar(); end
    sendHud();
    sendUnity();
    sendDash();
    logRow();
    if lost, running = false; end
    if tNow > tp.tMax, running = false; reason = "최대 시간"; end
    tNext = tNext + tp.T;
    if tNext < toc(t0), tNext = toc(t0) + tp.T; end
end
if ~isempty(car) && ~ismember(reason, ["사용자 종료 (Q)", "실차 종료 (Q)"]), sendCar("KEY,q"); end

result = saveRun();

%% ---- 보조 함수 ----
    function logRow()
        e = path_error(refSim, st.x, st.y, st.yaw, hint);
        hint = e.idx;
        q = twin_frames('sim2track', [st.x, st.y, st.yaw]);
        if n == size(data, 1), data = [data; nan(size(data))]; end
        n = n + 1;
        data(n, :) = [st.t, epoch0 + st.t, st.x, st.y, st.yaw, q, st.v * (1 + st.vs), st.delta, st.yawRate, ...
                      st.pwmCmd, st.steerDir * (st.steerLeft > 0), e.e_lat, e.s, tp.mass, ...
                      st.vs, double(tp.obs.enabled), obsRow, kfRow];
        obsRow = nan(1, 8); kfRow = nan(1, 6);
    end

    function sendUnity()
        if isempty(unity), return; end
        e = path_error(refSim, st.x, st.y, st.yaw, hint);
        s = struct('t', st.t, 'x', st.x, 'y', st.y, 'yaw', st.yaw, 'v', st.v, 'delta', st.delta, 'yawRate', st.yawRate);
        line = sprintf('%.6f,', [s.t, s.x, s.y, s.yaw, s.v, s.delta, s.yawRate, e.e_lat, e.e_head]);
        try, writeline(unity, line(1:end - 1)); catch, lost = true; reason = "Unity 연결 끊김"; end
    end

    function keys = pollUnity()
        keys = strings(0, 1);
        try
            while unity.NumBytesAvailable > 0
                ln = strtrim(string(readline(unity)));
                if startsWith(ln, "KEY,")
                    keys(end + 1, 1) = lower(extractAfter(ln, 4)); %#ok<AGROW>
                elseif ln == "QUIT"                       % StepInput(test1)의 Q도 종료로
                    keys(end + 1, 1) = "q"; %#ok<AGROW>
                end                                        % STEP·PLAY·RESET(test1용)은 무시
            end
        catch
            lost = true; reason = "Unity 연결 끊김";
        end
    end

    function sendCar(line)
        try, writeline(car, line); catch, lost = true; reason = "실차 연결 끊김"; car = []; end
    end

    function readCar()
        % APPLY,<key>,<speed>: 실차가 적용한 키 → 시뮬에 같은 입력 (followCar일 때)
        try
            while ~isempty(car) && car.NumBytesAvailable > 0
                ln = strtrim(string(readline(car)));
                if startsWith(ln, "APPLY,")
                    a = split(ln, ',');
                    ck = a(2); cspd = NaN;
                    if numel(a) >= 3, cspd = str2double(a(3)); end
                    if ~followCar, continue; end
                    logKey(st.t, ck, "car");
                    if ck == "q"
                        running = false; reason = "실차 종료 (Q)";
                    else
                        st = twin_apply(st, ck, cspd, tp);
                        steerKeysPending = steerKeysPending + any(ck == ["a", "d"]);
                    end
                elseif strlength(ln) > 0
                    fprintf('[twin] 실차: %s\n', ln);
                end
            end
        catch
        end
    end

    function logKey(t, key, src)
        keyLog(end + 1, :) = [sprintf('%.3f', t), sprintf('%.3f', epoch0 + t), key, src];
    end

    function sendHud()
        % Unity 화면 아래에 지금 PWM 표시 (실차 터미널의 "속도 +30"과 같은 값) + 무게
        hudNow = [st.pwmCmd, tp.mass, double(tp.obs.enabled), obsN > 0];
        if isempty(unity) || isequal(hudNow, lastHud), return; end
        lastHud = hudNow;
        if followCar, src = "real car"; else, src = "sim only"; end
        if ~tp.obs.enabled, obsTxt = upper(tp.estimator) + " OFF"; elseif obsN > 0, obsTxt = upper(tp.estimator) + " ON"; else, obsTxt = upper(tp.estimator) + " ON (no loc yet)"; end
        try, writeline(unity, sprintf("CMD,HUD,PWM %+d / %d  (%s)   mass %.1f kg   %s", st.pwmCmd, tp.maxSpeed, src, tp.mass, obsTxt)); catch, end
    end

    function applyLoc(ln)
        % 대시보드 → LOC,<epoch>,<x>,<y>,<θ>,<n>,<rms>[,Cxx,Cxy,Cyy,Ctt,Cxt,Cyt] (트랙 좌표, live_loc.py)
        %   → 칼만 필터 갱신 (tp.estimator = "ekf", 측정 공분산 사용) 또는 루엔버그 옵저버
        a = str2double(split(extractAfter(ln, 4), ','));
        if numel(a) < 5 || any(~isfinite(a(1:5))), return; end
        q = twin_frames('track2sim', a(2:4)');
        obsRow(1:3) = a(2:4)';
        if numel(a) >= 12                                 % Unity 그림: 원 측정 점 + 측정 공분산 R 타원 (시뮬 좌표, xy 공분산은 좌표 바꿔도 같음)
            sendUnityLine(sprintf("LOC,%.4f,%.4f,%.4f,%d,%.4e,%.4e,%.4e", q(1), q(2), q(3), round(a(5)), a(7), a(8), a(9)));
        end
        if ~tp.obs.enabled || tp.estimator == "none", return; end
        ageM = (epoch0 + st.t) - a(1);
        if ageM > tp.obs.maxAge, return; end
        if tp.estimator == "ekf"
            if numel(a) >= 12 && all(isfinite(a(7:12)))
                Rt = [a(7) a(8) a(11); a(8) a(9) a(12); a(11) a(12) a(10)];
                J = diag([-1 -1 1]); Rz = J * Rt * J;      % 트랙 → 시뮬 좌표 (x, y 부호 바뀜)
            else
                Rz = diag([0.15^2, 0.15^2, deg2rad(3)^2]) * (1 + 3 * (a(5) < 2));
            end
            [st, innM] = twin_ekf_update(st, q, Rz, ageM, tp, p);
            obsRow(7:8) = [sqrt(max(st.P(1, 1), st.P(2, 2))), innM.d2];
            if innM.used, ekfReject = 0; else, ekfReject = ekfReject + 1; end
            if ekfReject > 45                             % 3초 넘게 다 버림 → 길을 잃었을 수 있음: 위치 불확실성 키워 다시 잡기
                st.P = st.P + diag([0.5^2, 0.5^2, deg2rad(10)^2, 0, 0]); ekfReject = 0;
            end
        else
            if a(5) < tp.obs.minMarkers, return; end
            if isnan(obsT), dtm = 0.07; else, dtm = st.t - obsT; end
            [st, innM] = twin_observer(st, q, ageM, dtm, tp, p);
        end
        if innM.used
            obsT = st.t; obsN = obsN + 1;
            obsRow(4:6) = [innM.lon, innM.lat, innM.th];
            obsInn(end + 1, :) = obsRow(4:6);
        end
    end

    function applyKf(ln)
        % 대시보드 → KF,<epoch>,<x>,<y>,<θ>,<v>,<ω>,<Pxx>,<Pxy>,<Pyy>,<Ptt>,<Pxt>,<Pyt>,<nis>,<flag> (트랙 좌표, live_loc.py · loc_kf.py)
        %   ArUco 측정 칼만 필터 결과. MATLAB 옵저버에는 넣지 않음 (옵저버는 원 측정 LOC 사용) → 기록 + Unity 그림만
        a = str2double(split(extractAfter(ln, 3), ','));
        if numel(a) < 14 || any(~isfinite(a(2:4))), return; end
        kfRow = [a(2:4)', sqrt(a(7) + a(9)), a(13), a(14)];
        q = twin_frames('track2sim', a(2:4)');
        sendUnityLine(sprintf("KF,%.4f,%.4f,%.4f,%.4f,%.4e,%.4e,%.4e,%.2f,%d", q(1), q(2), q(3), a(5), a(7), a(8), a(9), a(13), round(a(14))));
    end

    function sendUnityLine(line)
        if isempty(unity), return; end
        try, writeline(unity, line); catch, end
    end

    function sendDash()
        % 대시보드: SIM,t,epoch,x_track,y_track,theta_track,v,pwm,delta,mass,pwmDead,tauV
        if isempty(dash), return; end
        qd = twin_frames('sim2track', [st.x, st.y, st.yaw]);
        try
            writeline(dash, sprintf("SIM,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%d,%.4f,%.2f,%.2f,%.3f,%d,%.3f", st.t, epoch0 + st.t, ...
                qd(1), qd(2), qd(3), st.v * (1 + st.vs), round(st.pwmCmd), st.delta, tp.mass, tp.pwmDead, tp.tauV, ...
                double(tp.obs.enabled), st.vs));
        catch
            dash = [];                                     % 대시보드가 꺼져도 주행은 계속
        end
    end

    function keys = pollDash()
        % 대시보드 → MATLAB: MASS,<kg> (무게 바꿈) · KEY,<k> (웹 페이지에서 누른 키 = Unity 키와 같음)
        keys = strings(0, 1);
        try
            while ~isempty(dash) && dash.NumBytesAvailable > 0
                ln = strtrim(string(readline(dash)));
                if startsWith(ln, "MASS,")
                    massNew = str2double(extractAfter(ln, 5));
                    if isfinite(massNew) && massNew > 0 && abs(massNew - tp.mass) > 1e-9
                        tp = twin_mass(tp, massNew);
                        fprintf('[twin] 무게 %.1f kg → tauV %.2f s · pwmDead %+.1f\n', tp.mass, tp.tauV, tp.pwmDead);
                    end
                elseif startsWith(ln, "KEY,")
                    keys(end + 1, 1) = lower(extractAfter(ln, 4)); %#ok<AGROW>
                elseif startsWith(ln, "LOC,")
                    applyLoc(ln);
                elseif startsWith(ln, "KF,")
                    applyKf(ln);
                elseif startsWith(ln, "OBS,")
                    wasOn = tp.obs.enabled;
                    tp.obs.enabled = str2double(extractAfter(ln, 4)) ~= 0;
                    if tp.obs.enabled && ~wasOn                % 다시 켬: 그동안 벌어졌을 수 있으니 위치·방향 불확실성을 크게
                        st.P = diag([1.0^2, 1.0^2, deg2rad(20)^2, deg2rad(10)^2, 0.2^2]); ekfReject = 0;
                    end
                    fprintf('[twin] 추정기(%s) %s\n', tp.estimator, string(tp.obs.enabled));
                end
            end
        catch
            dash = [];
        end
    end

    function S = saveRun()
        D = data(1:n, :);
        T = array2table(D, 'VariableNames', cols);
        writetable(T, fullfile(folder, 'timeseries.csv'));
        K = array2table(keyLog, 'VariableNames', {'t_s', 'epoch_s', 'key', 'src'});
        writetable(K, fullfile(folder, 'keys.csv'));

        moved = D(:, 9) ~= 0 | (1:n)' == 1;
        S = struct();
        S.run = runName;
        S.folder = string(folder);
        S.reason = reason;
        S.duration_s = D(end, 1);
        S.epoch_start_s = epoch0;
        S.car_session = carInfo.session;
        S.car_t0_epoch = carInfo.t0_epoch;
        S.car_hello = carInfo.hello;
        S.sim_follows_car_apply = followCar;
        S.keys = sum(keyLog(:, 4) ~= "unity");
        S.distance_m = sum(hypot(diff(D(:, 3)), diff(D(:, 4))));
        S.e_lat_rms_m = sqrt(mean(D(moved, 14).^2, 'omitnan'));
        S.e_lat_max_abs_m = max(abs(D(moved, 14)));
        S.params = tp;
        S.mass_kg_end = tp.mass;
        S.estimator = tp.estimator;
        S.observer_updates = obsN;
        if ~isempty(obsInn)
            S.observer_innovation_rms = struct('lon_m', sqrt(mean(obsInn(:, 1).^2)), 'lat_m', sqrt(mean(obsInn(:, 2).^2)), ...
                'theta_deg', rad2deg(sqrt(mean(obsInn(:, 3).^2))));
        end
        S.speed_scale_end = st.vs;
        S.delta_end_deg = rad2deg(st.delta);
        S.vehicle_L = p.L;
        S.reference = "lane2_centerline.csv (~/demo/track, 도면 2차선 중심선)";
        S.frames = "sim: 매트 가운데 원점 / track: 매트 왼쪽 아래 원점, x_t = 8 - x_s, y_t = 5.5 - y_s, θ_t = θ_s - π";
        fid = fopen(fullfile(folder, 'summary.json'), 'w');
        fprintf(fid, '%s', jsonencode(S, 'PrettyPrint', true));
        fclose(fid);

        plotTrack(D);
        if ~isempty(unity)
            png = fullfile(folder, 'unity_top.png');
            try
                writeline(unity, "CMD,SNAP," + png);
                t1 = tic;
                while ~isfile(png) && toc(t1) < 5, pause(0.1); end
            catch
            end
            if ~isfile(png), warning('[twin] Unity Top 이미지가 저장되지 않음 (Unity가 이미 꺼졌거나 Play 중이 아님)'); end
        end
        fprintf('[twin] 끝 (%s): %.1f s · %.2f m · 키 %d개 · 시뮬 횡오차 RMS %.3f m\n  결과: %s\n', ...
            reason, S.duration_s, S.distance_m, S.keys, S.e_lat_rms_m, folder);
        if S.car_session ~= ""
            fprintf('  실차 세션: %s\n  (drive.sh가 영상·localization을 만든 뒤 자동으로 비교합니다. 직접: python3 %s --sim %s)\n', ...
                S.car_session, fullfile(root, 'analysis', 'compare_twin.py'), folder);
        end
    end

    function plotTrack(D)
        refT = twin_reference('track');
        f = figure('Visible', 'off', 'Position', [100 100 1100 800]);
        hold on; axis equal; grid on; box on;
        plot([0 16 16 0 0], [0 0 11 11 0], 'k-', 'LineWidth', 1);
        plot([refT.x; refT.x(1)], [refT.y; refT.y(1)], '-', 'Color', [0.9 0.7 0], 'LineWidth', 1.5);
        plot(D(:, 6), D(:, 7), 'r-', 'LineWidth', 1.8);
        plot(D(1, 6), D(1, 7), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
        plot(D(end, 6), D(end, 7), 'ks', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
        xlim([-0.3 16.3]); ylim([-0.3 11.3]);
        xlabel('x_{track} [m]'); ylabel('y_{track} [m]');
        legend({'mat 16 x 11 m', 'lane 2 centerline', 'Unity sim trajectory', 'start', 'end'}, 'Location', 'bestoutside');
        title(sprintf('Twin sim %s  (%.1f s, %.2f m)', runName, D(end, 1), sum(hypot(diff(D(:, 3)), diff(D(:, 4))))), ...
            'Interpreter', 'none');
        exportgraphics(f, fullfile(folder, 'trajectory_track.png'), 'Resolution', 150);
        close(f);
    end

    function closeAll()
        try, if ~isempty(car), writeline(car, "KEY,space"); end, catch, end
    end
end

function kv = parseHello(line)
kv = struct();
parts = split(line, ',');
for i = 2:numel(parts)
    a = split(parts(i), '=');
    if numel(a) >= 2, kv.(matlab.lang.makeValidName(a(1))) = strjoin(a(2:end), '='); end
end
end
