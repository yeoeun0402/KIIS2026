function R = twin_replay(sess, locFile, varargin)
% [twin] 실차 주행 기록을 다시 돌려 시뮬(옵저버 켬/끔)을 실차 localization 과 비교 (오프라인 검증용)
%   R = twin_replay('~/demo/sessions/<세션>', '<live_loc 출력 txt>', 'estimator', "ekf")   % "ekf" | "luenberger" | "none"
%   입력: <세션>/drive_log.csv 의 실제 보낸 명령 (V<pwm> · L · R · F) — 실차가 적용한 그대로
%         locFile: live_loc.py --video 출력 (LOC,<영상 초>,x,y,θ,n,rms) → 옵저버 측정 (0.1 s 늦게 들어온다고 가정)
%         <세션>/map/trajectory.csv: 정답 (주행 뒤 build_map)
%   출력: R.err (정답과의 위치 차이, 0.1 s 간격) · R.rms · R.median · R.t
sess = char(sess); if sess(1) == '~', sess = [getenv('HOME') sess(2:end)]; end
tp = twin_params(varargin{:}); p = vehicle_params();
% 명령
o = detectImportOptions(fullfile(sess, 'drive_log.csv'), 'Delimiter', ','); o = setvartype(o, {'kind', 'value'}, 'string');
T = readtable(fullfile(sess, 'drive_log.csv'), o);
T = T(T.kind == "tx", :);
% localization 측정 (영상 초 → 세션 초: 첫 프레임 시각)
vf = readtable(fullfile(sess, 'video_frames.csv')); off = vf.t_s(1);
L = readmatrix(locFile, 'FileType', 'text', 'Delimiter', ',', 'OutputType', 'string');
L = L(L(:, 1) == "LOC", 2:end); L = str2double(L); L(:, 1) = L(:, 1) + off;
G = readtable(fullfile(sess, 'map', 'trajectory.csv'));
q0 = twin_frames('track2sim', tp.startTrack);
st = struct('t', 0, 'x', q0(1), 'y', q0(2), 'yaw', q0(3), 'v', 0, 'delta', -tp.delta0, 'yawRate', 0, ...
    'pwmCmd', 0, 'pwm', 0, 'pend', zeros(0, 2), 'steerDir', 0, 'steerLeft', 0, 'vs', 0, 'P', diag(tp.ekf.P0));
nKeys = 0; useEst = tp.obs.enabled && tp.estimator ~= "none";
dt = 0.01; tEnd = min(G.t_s(end), T.t_s(end)); iC = 1; iL = 1; lastObs = NaN; lag = 0.1;
tt = (0:0.1:tEnd)'; E = nan(numel(tt), 2); k = 1; nUsed = 0;
while st.t < tEnd
    while iC <= height(T) && T.t_s(iC) <= st.t
        v = T.value(iC);
        if startsWith(v, "V")
            pw = str2double(extractAfter(v, 1));
            if pw ~= st.pwmCmd, st.pwmCmd = pw; st.pend(end + 1, :) = [st.t + tp.delay, pw]; end
        elseif v == "L", st.steerDir = 1; st.steerLeft = tp.pulse; nKeys = nKeys + 1;
        elseif v == "R", st.steerDir = -1; st.steerLeft = tp.pulse; nKeys = nKeys + 1;
        elseif v == "F", st.steerDir = 0; st.steerLeft = 0;
        end
        iC = iC + 1;
    end
    while iL <= size(L, 1) && L(iL, 1) + lag <= st.t
        m = twin_frames('track2sim', L(iL, 2:4));
        if useEst && tp.estimator == "ekf"
            if size(L, 2) >= 12 && all(isfinite(L(iL, 7:12)))
                a = L(iL, :); Rt = [a(7) a(8) a(11); a(8) a(9) a(12); a(11) a(12) a(10)]; J = diag([-1 -1 1]); Rz = J * Rt * J;
            else
                Rz = diag([0.15^2, 0.15^2, deg2rad(3)^2]) * (1 + 3 * (L(iL, 5) < 2));
            end
            [st, inn] = twin_ekf_update(st, m, Rz, st.t - L(iL, 1), tp, p);
            if inn.used, nUsed = nUsed + 1; end
        elseif useEst && L(iL, 5) >= tp.obs.minMarkers
            if isnan(lastObs), dtm = 1/15; else, dtm = st.t - lastObs; end
            [st, inn] = twin_observer(st, m, st.t - L(iL, 1), dtm, tp, p);
            if inn.used, lastObs = st.t; nUsed = nUsed + 1; end
        end
        iL = iL + 1;
    end
    st = twin_model_step(st, tp, p, dt);
    if useEst && tp.estimator == "ekf", st = twin_ekf_predict(st, tp, p, dt, nKeys); nKeys = 0; end
    if k <= numel(tt) && st.t >= tt(k) - 1e-9
        qq = twin_frames('sim2track', [st.x, st.y, st.yaw]);
        E(k, :) = qq(1:2); k = k + 1;
    end
end
gx = interp1(G.t_s, G.x_m, tt, 'linear', NaN); gy = interp1(G.t_s, G.y_m, tt, 'linear', NaN);
R.t = tt; R.err = hypot(E(:, 1) - gx, E(:, 2) - gy); ok = isfinite(R.err);
R.rms = sqrt(mean(R.err(ok).^2)); R.median = median(R.err(ok)); R.p95 = prctile(R.err(ok), 95); R.max = max(R.err(ok));
R.updates = nUsed; R.vs = st.vs; R.est = E;
end
