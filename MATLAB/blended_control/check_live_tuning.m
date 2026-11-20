function chk = check_live_tuning(varargin)
% 실행 중 파라미터 변경(슬라이더·입력칸)이 그대로 반영되는지 확인 (PP_PID: 논문 PP_PI + D)
%   check_live_tuning                         % Unity 없이, 실시간 한 바퀴 (패널의 슬라이더가 움직이는 것이 보임)
%   check_live_tuning('useUnity', true)       % Unity Play 먼저: Unity 차도 함께 움직임
%   check_live_tuning('realtime', false)      % 기다리지 않고 빨리 (테스트용)
%
% ① blended_discrete를 자동 버전으로 한 바퀴 돌리면서, 정해진 스텝이 끝날 때 패널을 사용자처럼 조작함
%    (슬라이더 Value / 입력칸 String을 바꾸고 그 콜백을 부름 = 마우스로 끌거나 값을 쓰고 Enter를 누른 것과 같은 경로)
%      k = 40  v    0.8 → 1.0  (슬라이더)      k = 90  Lf   0.8 → 1.3  (입력칸)
%      k = 140 a    0.59 → 0.3 (슬라이더)      k = 190 wcur 0.5 → 0.7  (입력칸)
%      k = 240 Kp   → 0.3     (슬라이더)      k = 290 Ki   → 0.004    (입력칸)
%      k = 340 ell  3 → 5      (슬라이더)      k = 380 Kd   → 0.003    (입력칸)
%      (sumToOne이면 Kp·Ki·Kd를 바꿀 때 Kpp = 1 - Kp - Ki - Kd도 바뀜)
%    스텝 k가 끝난 뒤 바꿨으므로, 기록(timeseries.csv의 k)에서는 정확히 k부터 새 값이어야 함
% ② 기록에서 각 파라미터가 "실제로 쓰인 값"을 거꾸로 잼
%      v    : 차 속도                         Lf  : 기록된 선행점이 경로를 따라 투영점에서 몇 m 앞인지
%      a    : (e_la - e) / sin Δψ - L_ad       Kp  : δ_p / e_la          Ki : δ_i / Σe
%      Kd   : δ_d / ((e[k] - e[k-1]) / T_s)
%      Kpp  : (δ_raw - δ_p - δ_i - δ_d) / δ_pp  wcur: (δ - m) / (δ_raw - m), m = 이전 ℓ-1개 출력의 평균
%      ell  : 스텝마다 ℓ = 1~10 중 Eq.8을 만족하는 값
% ③ 다시 계산: 기록된 차 위치·조향각과 p_* 값만으로 세 제어기(PP, PID, PP_PID)의 δ를 매 스텝 처음부터 계산해 기록과 비교
% 반환 chk.pass, chk.table, chk.replay_err, chk.result.  그림: <결과 폴더>/live_tune_check.png

o = struct('useUnity', false, 'realtime', true, 'save', true);
for i = 1:2:numel(varargin), o.(char(varargin{i})) = varargin{i + 1}; end
here = fileparts(mfilename('fullpath'));
if ~exist('vehicle_params', 'file'), addpath(fileparts(here), '-end'); end
addpath(here);
p = vehicle_params(); ref = load_reference_path(); L = p.L;

%% ① 실행 + 정해진 스텝에 패널 조작 (blended_discrete의 stepHook)
sched = struct('k',    {40,       90,     140,      190,    240,      290,    340,      380}, ...
               'name', {'v',      'Lf',   'a',      'wcur', 'Kp',     'Ki',   'ell',    'Kd'}, ...
               'val',  {1.0,      1.3,    0.3,      0.7,    0.3,      0.004,  5,        0.003}, ...
               'via',  {'slider', 'edit', 'slider', 'edit', 'slider', 'edit', 'slider', 'edit'});
r = blended_discrete('input', 'auto', 'useUnity', o.useUnity, 'realtime', o.realtime, 'save', o.save, ...
    'stepHook', @(fig, k) operatePanel(fig, k, sched));
T = r.timeseries; Ts = r.opt.Ts; n = height(T); k0 = T.k;
prm0 = r.prm;

%% ② 기록에서 잰 값 (스텝마다)
y   = T.bl_delta_cmd_rad;  u = T.bl_delta_raw_rad;
hist0 = T.bl_delta_rad(1);                               % 필터의 처음 이전 출력 = 출발 조향각
prevOut = @(j, m) [flip(y(max(1, j - m):j - 1)); repmat(hist0, max(0, m - (j - 1)), 1)];   % 최신부터 m개
meas = struct();
meas.v   = T.v_mps;
meas.Kp  = ratio(T.bl_delta_p_rad, T.bl_e_la_m, 1e-6);
meas.Ki  = ratio(T.bl_delta_i_rad, T.bl_sum_e_m, 1e-6);
meas.Kd  = ratio(T.bl_delta_d_rad, T.bl_de_mps, 1e-4);
meas.Kpp = ratio(u - T.bl_delta_p_rad - T.bl_delta_i_rad - T.bl_delta_d_rad, T.bl_delta_pp_rad, 1e-4);
meas.a   = nan(n, 1); meas.wcur = nan(n, 1); meas.ell = nan(n, 1); meas.Lf = nan(n, 1);
sdp = sin(T.bl_e_head_rad);
ok_a = abs(sdp) > 1e-3;
meas.a(ok_a) = (T.bl_e_la_m(ok_a) - T.bl_e_traj_m(ok_a)) ./ sdp(ok_a) - T.bl_L_ad_m(ok_a);
for j = 1:n
    ell = T.p_ell(j);
    if ell >= 2
        mPrev = mean(prevOut(j, ell - 1));
        if abs(u(j) - mPrev) > 1e-5, meas.wcur(j) = (y(j) - mPrev) / (u(j) - mPrev); end
    end
    res = inf(1, 10);                                    % ℓ 후보마다 Eq.8을 대입한 차이
    for c = 1:10
        if c == 1, yc = u(j); else, yc = T.p_wcur(j) * u(j) + (1 - T.p_wcur(j)) / (c - 1) * sum(prevOut(j, c - 1)); end
        res(c) = abs(yc - y(j));
    end
    [best, cBest] = min(res);
    if best < 1e-9 && sum(res < 1e-9) == 1, meas.ell(j) = cBest; end   % 한 후보만 맞을 때만 잼
end
hint = NaN; hintL = NaN;
for j = 1:n
    ec = path_error(ref, T.bl_x_m(j), T.bl_y_m(j), T.bl_yaw_rad(j), hint); hint = ec.idx;
    el = path_error(ref, T.bl_lookahead_x_m(j), T.bl_lookahead_y_m(j), 0, hintL); hintL = el.idx;
    meas.Lf(j) = mod(el.s - ec.s, ref.L);
end

rows = strings(0, 1); ok = true(numel(sched) + 1, 1);
for i = 1:numel(sched)
    nm = sched(i).name;
    pc = T.("p_" + nm);
    kNew = k0(find(pc ~= pc(1), 1));                     % 기록에서 새 값이 처음 쓰인 스텝
    if isempty(kNew), kNew = NaN; end
    before = k0 < kNew; after = k0 >= kNew;
    if nm == "v"                                         % 속도는 가속 한계로 몇 스텝 걸림
        after  = k0 >= kNew + ceil((sched(i).val - prm0.v) / (p.accel * Ts)) + 1;
        before = k0 >= 20 & k0 < kNew;                   % 출발 가속 구간 제외
    end
    mb = median(meas.(nm)(before), 'omitnan'); ma = median(meas.(nm)(after), 'omitnan');
    devB = max(abs(meas.(nm)(before) - prm0.(nm)), [], 'omitnan');
    devA = max(abs(meas.(nm)(after) - sched(i).val), [], 'omitnan');
    ok(i) = ~isnan(kNew) && kNew == sched(i).k && all(pc(after) == sched(i).val) && all(pc(before) == prm0.(nm)) && ...
            devA < 1e-6 * max(1, abs(sched(i).val)) && devB < 1e-6 * max(1, abs(prm0.(nm)));
    rows(end + 1, 1) = sprintf('%-4s %-6s | 조작 k = %3d, 기록에서 새 값 시작 k = %3d | 설정 %.4g -> %.4g | 기록에서 잰 값 %.6g -> %.6g | %s', ...
        nm, sched(i).via, sched(i).k, kNew, prm0.(nm), sched(i).val, mb, ma, string(ternary(ok(i), 'OK', 'NG'))); %#ok<AGROW>
end
% Kpp: sumToOne이면 매 스텝 1 - Kp - Ki - Kd와 같아야 함 (Kp·Ki·Kd를 바꾼 스텝에서 함께 바뀜)
kppSet = T.p_Kpp; kppRule = 1 - T.p_Kp - T.p_Ki - T.p_Kd;
devK = max(abs(meas.Kpp - kppSet), [], 'omitnan');
ok(end) = devK < 1e-9 && (~prm0.sumToOne || max(abs(kppSet - kppRule)) < 1e-12);
rows(end + 1, 1) = sprintf('Kpp  자동   | Kp·Ki·Kd 바뀔 때 함께: %.4g -> %.4g (k = %d, %d, %d) | 기록에서 잰 Kpp - 설정 최대 %.1e | K_pp + K_p + K_i + K_d = 1: %s | %s', ...
    kppSet(1), kppSet(end), sched(5).k, sched(6).k, sched(8).k, devK, string(ternary(max(abs(kppSet - kppRule)) < 1e-12, '매 스텝 맞음', '안 맞음')), ...
    string(ternary(ok(end), 'OK', 'NG')));

%% ③ 다시 계산 (기록된 상태 + p_* 값만으로 세 제어기의 δ)
tags = ["pp", "pid", "bl"]; replayErr = zeros(1, 3);
for c = 1:3
    tg = tags(c);
    x = T.(tg + "_x_m"); yy = T.(tg + "_y_m"); yaw = T.(tg + "_yaw_rad"); dl = T.(tg + "_delta_rad");
    ellC = T.p_ell; wC = T.p_wcur;
    if c < 3 && ~r.opt.lpfAll, ellC(:) = 1; end
    out = dl(1); hist = repmat(dl(1), 20, 1); Se = 0; I = 0; ePrev = NaN; hint = NaN; worst = 0;
    for j = 1:n
        ec = path_error(ref, x(j), yy(j), yaw(j), hint); hint = ec.idx;
        e = -ec.e_lat;
        if isnan(ePrev), de = 0; else, de = (e - ePrev) / Ts; end   % (e[k] - e[k-1]) / T_s
        ePrev = e;
        if c == 2                                         % 비교용 PID (PID_control과 같은 식)
            I = I + e * Ts;
            raw = r.opt.pidKp * e + r.opt.pidKi * I + r.opt.pidKd * de;
        else                                              % PP (Kpp 1, PID 0) 또는 PP_PID (논문 Eq.3–7, 10 + D)
            ql = path_pose(ref, ec.s + T.p_Lf(j));
            al = atan2(ql.y - yy(j), ql.x - x(j)) - yaw(j); al = atan2(sin(al), cos(al));
            Lad = hypot(ql.x - x(j), ql.y - yy(j));
            dPP = atan(2 * L * sin(al) / Lad);
            if c == 1
                raw = dPP;
            else
                ela = e + (T.p_a(j) + Lad) * sin(-ec.e_head);
                Se = Se + e;
                raw = T.p_Kpp(j) * dPP + T.p_Kp(j) * ela + T.p_Ki(j) * Se + T.p_Kd(j) * de;
            end
        end
        if ellC(j) <= 1                                   % Low-pass Filter (Eq.8–9)
            out = raw;
        else
            out = wC(j) * raw + (1 - wC(j)) / (ellC(j) - 1) * sum(hist(1:ellC(j) - 1));
        end
        hist = [out; hist(1:end - 1)];
        worst = max(worst, abs(out - T.(tg + "_delta_cmd_rad")(j)));
    end
    replayErr(c) = worst;
end
replayOk = all(replayErr < 1e-9);

chk.pass = all(ok) && replayOk && r.lapDone;
chk.table = rows;
chk.replay_err = replayErr;
chk.result = r;

fprintf('\n실행 중 파라미터 변경 확인 (%d 스텝, 한 바퀴 %s)\n', n, string(ternary(r.lapDone, '완주', '미완주')));
fprintf('  %s\n', rows);
fprintf('  다시 계산한 δ와 기록의 최대 차이: PP %.1e, PID %.1e, PP_PID %.1e rad (%s)\n', ...
    replayErr, string(ternary(replayOk, 'OK', 'NG')));
fprintf('  결과: %s\n', string(ternary(chk.pass, '모두 반영됨', '확인 필요')));

%% 그림: 설정값(p_*) vs 기록에서 잰 값
fig = figure('Name', '실행 중 파라미터 변경 확인', 'NumberTitle', 'off', 'Position', [60 30 1400 1040]);
plotNames = ["v", "Lf", "a", "wcur", "ell", "Kp", "Ki", "Kd", "Kpp"];
units = ["m/s", "m", "m", "", "", "rad/m", "rad/m", "rad s/m", ""];
t = T.t_s;
for i = 1:numel(plotNames)
    nm = plotNames(i);
    ax = subplot(6, 2, i, 'Parent', fig); hold(ax, 'on'); grid(ax, 'on');
    plot(ax, t, meas.(nm), '.', 'Color', [0.85 0.10 0.10], 'MarkerSize', 6);
    plot(ax, t, T.("p_" + nm), 'k-', 'LineWidth', 1.2);
    ks = [sched(strcmp({sched.name}, nm)).k];
    if nm == "Kpp", ks = [sched(5).k, sched(6).k, sched(8).k]; end
    for kk = ks, xline(ax, kk * Ts, 'b--'); end
    pc = T.("p_" + nm); yl = [min(pc), max(pc)]; pad = 0.25 * max(diff(yl), 0.1 * max(abs(yl)));
    ylim(ax, [yl(1) - pad, yl(2) + pad]);
    title(ax, sprintf('%s: %.4g → %.4g', nm, pc(1), pc(end)));
    ylabel(ax, units(i));
    if i == 1, legend(ax, {'기록에서 잰 값', '설정값 p_*', '조작한 시각'}, 'Location', 'best'); end
end
xlabel(subplot(6, 2, 9, 'Parent', fig), 't [s]');
ax = subplot(6, 2, [11 12], 'Parent', fig); hold(ax, 'on');
bar(ax, 1:3, max(replayErr, 1e-18), 'FaceColor', [0.6 0.6 0.6]);
yline(ax, 1e-9, 'r--', '허용 한계 1e-9', 'LabelHorizontalAlignment', 'left');
for c = 1:3
    if replayErr(c) == 0, lbl = '0 (완전히 같음)'; else, lbl = sprintf('%.1e', replayErr(c)); end
    text(ax, c, 1e-16, lbl, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
set(ax, 'YScale', 'log', 'XTick', 1:3, 'XTickLabel', {'PP', 'PID', 'PP\_PID'}); grid(ax, 'on');
xlim(ax, [0.4 3.6]); ylim(ax, [1e-18 1e-6]);
title(ax, '처음부터 다시 계산한 \delta - 기록 (최대) [rad]');
sgtitle(fig, sprintf('실행 중 파라미터 변경 확인 (PP\\_PID): %s', string(ternary(chk.pass, '모두 설정값대로 반영됨', '확인 필요'))));
if strlength(r.folder) > 0
    exportgraphics(fig, fullfile(r.folder, 'live_tune_check.png'), 'Resolution', 110);
    fprintf('  그림: %s\n', fullfile(r.folder, 'live_tune_check.png'));
end
end

function operatePanel(fig, k, sched)
% blended_discrete의 stepHook: 스텝 k가 끝났을 때 예정된 조작이 있으면 패널의 슬라이더·입력칸을 바꾸고 콜백 호출
for s = sched(k == [sched.k])
    if s.via == "slider"
        h = findobj(fig, 'Tag', "tune_" + s.name);
        h.Value = s.val;                              % 슬라이더를 끌어 놓은 것과 같음
    else
        h = findobj(fig, 'Tag', "tuneEdit_" + s.name);
        h.String = sprintf('%.4g', s.val);            % 입력칸에 값을 쓴 것과 같음
    end
    h.Callback(h, []);                                % 마우스를 놓거나 Enter를 누를 때 MATLAB이 부르는 콜백
end
end

function q = ratio(a, b, tol)
% a / b (|b|가 tol보다 작은 스텝은 NaN)
q = a ./ b;
q(abs(b) < tol) = NaN;
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
