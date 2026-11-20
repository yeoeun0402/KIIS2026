function out = plot_lpf_spikes(varargin)
% switching_3: 블랜디드에 LPF를 넣었을 때 궤적 오차가 갑자기 커진 곳을 맵에 빨간 점으로 표시
%   같은 조건(경로 곡률 판단, Unity 없이)으로 두 번 계산해서 같은 시각 k끼리 비교:
%     LPF 없음 = switch3_discrete('ell', 1)  (switching_2와 같음)
%     LPF      = switch3_discrete('ell', ell, 'wcur', wcur)
%   튄 구간: LPF 실행에서 |0 - e_lat| > eTh인 스텝 (mergeGap 스텝 이하로 끊긴 것은 한 구간으로 이어 붙임)
%     빨강 = LPF 때문에 새로 생김: 같은 시간 구간에서 LPF 없음의 최대 |오차| < eTh
%     회색 = LPF 없어도 큼
%   그림:
%     왼쪽  맵: 블랜디드(LPF) 궤적 보라, 튄 구간의 위치 = 빨간 점(회색 점), 번호와 최대 오차
%     오른쪽 ① 궤적 오차 0 - e_lat (LPF 보라, LPF 없음 회색 점선, 튄 구간 빨간 점)
%            ② 블랜디드 입력 조향각 (LPF 후 보라, LPF 전 검정 점선)
%            배경 띠 = 블랜디드(LPF) 차가 그 시각에 쓴 제어기 (PP 초록, PID 파랑, BC 노랑)
%   저장: graph/분석_LPF오차급증_<날짜>.png, .svg (벡터), .csv (구간 표)
%
% 사용 예:
%   cd MATLAB; setup_paths; addpath('switching_3')
%   plot_lpf_spikes                                   % 기본: 0.8 m/s, ℓ 3, w 0.5, 기준 3 cm
%   plot_lpf_spikes('wcur', 0.7, 'eTh', 0.025)        % LPF 값·기준 바꾸기
%   plot_lpf_spikes('straightMode', "PP")             % 그 밖의 switch3_discrete 옵션은 그대로 넘김

opt.ell = 3; opt.wcur = 0.5;    % LPF 실행의 값
opt.eTh = 0.03;                 % 튄 구간 기준 |오차| [m]
opt.mergeGap = 5;               % 이 스텝 수 이하로 끊긴 구간은 이어 붙임
opt.save = true; opt.show = true;
pass = {};                      % switch3_discrete에 그대로 넘길 이름-값
for i = 1:2:numel(varargin)
    nm = char(varargin{i});
    if isfield(opt, nm), opt.(nm) = varargin{i + 1}; else, pass = [pass, {nm, varargin{i + 1}}]; end %#ok<AGROW>
end
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here), '-end'); addpath(here);
S = switch3_style(); p = vehicle_params(); ref = load_reference_path();

%% 1. 두 번 계산 (같은 출발, 같은 판단 방식)
c = [{'decision', 'ref', 'useUnity', false, 'realtime', false, 'save', false, 'showFigure', false}, pass];
r0 = switch3_discrete(c{:}, 'ell', 1);
r1 = switch3_discrete(c{:}, 'ell', opt.ell, 'wcur', opt.wcur);
T0 = r0.timeseries; T1 = r1.timeseries;
n = min(height(T0), height(T1)); T0 = T0(1:n, :); T1 = T1(1:n, :);
t = T1.t_s; Ts = median(diff(t));
e1 = T1.bl_e_traj_m; e0 = T0.bl_e_traj_m;
md = string(T1.mode);

%% 2. 튄 구간 찾기
on = abs(e1) > opt.eTh;
d = diff([false; on; false]); i0 = find(d == 1); i1 = find(d == -1) - 1;
k = 1;
while k < numel(i0)                               % 짧게 끊긴 구간 이어 붙이기
    if i0(k + 1) - i1(k) - 1 <= opt.mergeGap, i1(k) = i1(k + 1); i0(k + 1) = []; i1(k + 1) = [];
    else, k = k + 1; end
end
nS = numel(i0);
seg = table('Size', [nS 13], 'VariableTypes', [repmat("double", 1, 10), "string", "double", "string"], ...
    'VariableNames', {'no', 't_start_s', 't_end_s', 't_peak_s', 'x_peak_m', 'y_peak_m', 'peak_lpf_mm', ...
    'peak_nolpf_mm', 's_start_m', 's_end_m', 'switch_before', 't_switch_s', 'kind'});
for j = 1:nS
    r = i0(j):i1(j);
    [pk, im] = max(abs(e1(r))); kp = r(im);
    pk0 = max(abs(e0(r)));
    ks = find(md(2:kp) ~= md(1:kp - 1), 1, 'last') + 1;   % 최대 오차 전에 마지막으로 제어기가 바뀐 스텝
    if isempty(ks), sw = "없음"; tsw = NaN; else, sw = md(ks - 1) + "→" + md(ks); tsw = t(ks); end
    kind = "LPF 없어도 큼"; if pk0 < opt.eTh, kind = "LPF 때문에 새로 생김"; end
    seg(j, :) = {j, t(r(1)), t(r(end)), t(kp), T1.bl_x_m(kp), T1.bl_y_m(kp), 1000 * pk, 1000 * pk0, ...
        pathS(ref, T1, r(1)), pathS(ref, T1, r(end)), sw, tsw, kind};
end
isNew = seg.kind == "LPF 때문에 새로 생김";

%% 3. 그림
vis = 'on'; if ~opt.show, vis = 'off'; end
fig = figure('Name', 'switching_3 - LPF 때문에 오차가 갑자기 커진 곳', 'NumberTitle', 'off', 'Color', 'w', ...
    'Position', [60 60 1500 860], 'Visible', vis);
cB = S.col(4, :); cE = S.red; cG = [0.45 0.45 0.45];
lpfTxt = sprintf('LPF ℓ %d, w_{current} %.2f', round(opt.ell), opt.wcur);

% 왼쪽: 맵
axM = axes(fig, 'Position', [0.035 0.07 0.53 0.78]);
draw_map(axM, p);
axis(axM, 'off');                                 % 지도의 x, y축 (눈금·이름) 숨김
plot(axM, [T1.bl_x_m(1); T1.bl_x_m], [T1.bl_y_m(1); T1.bl_y_m], '-', 'Color', cB, 'LineWidth', 1.4);
hOld = plot(axM, NaN, NaN, '.', 'Color', cG, 'MarkerSize', 13);
hNew = plot(axM, NaN, NaN, '.', 'Color', cE, 'MarkerSize', 15);
for j = 1:nS
    r = i0(j):i1(j);
    if isNew(j), h = hNew; cc = cE; else, h = hOld; cc = cG; end
    set(h, 'XData', [h.XData, NaN, T1.bl_x_m(r)'], 'YData', [h.YData, NaN, T1.bl_y_m(r)']);
    plot(axM, seg.x_peak_m(j), seg.y_peak_m(j), 'o', 'Color', cc, 'MarkerSize', 11, 'LineWidth', 1.8, ...
        'HandleVisibility', 'off');
    lab = sprintf('%s %.1f s\n%.0f mm (LPF 없음 %.0f)', circled(j), seg.t_peak_s(j), seg.peak_lpf_mm(j), seg.peak_nolpf_mm(j));
    [lx, ly] = labelPos(seg.x_peak_m(j), seg.y_peak_m(j), p);
    plot(axM, [seg.x_peak_m(j), lx], [seg.y_peak_m(j), ly], '-', 'Color', cc, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(axM, lx, ly, lab, 'Color', cc, 'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', 'w', 'EdgeColor', cc, 'Margin', 2, 'HorizontalAlignment', 'center');
end
legend(axM, [hNew, hOld], {sprintf('LPF 때문에 오차가 갑자기 커진 곳 (|오차| > %.0f cm, LPF 없으면 %.0f cm 미만)', ...
    100 * opt.eTh, 100 * opt.eTh), 'LPF 없어도 오차가 큰 곳 (참고)'}, 'Location', 'southoutside', 'Orientation', 'horizontal');
title(axM, {sprintf('블랜디드 + %s: 궤적 오차가 갑자기 커진 위치 (빨간 점)', lpfTxt), ...
    '번호 옆: 그 구간의 최대 |0 - e_{lat}|, 괄호 = 같은 시각 LPF 없음'}, 'FontSize', 10.5, 'FontWeight', 'normal');

% 오른쪽 ① 궤적 오차
pos = [0.62 0.56 0.36 0.31; 0.62 0.09 0.36 0.31];
axE = axes(fig, 'Position', pos(1, :)); hold(axE, 'on'); grid(axE, 'on'); box(axE, 'on');
modeBands(axE, t, md, Ts, S);
yline(axE, 0, 'k-', 'Alpha', 0.4, 'HandleVisibility', 'off');
yline(axE, opt.eTh * [1 -1], ':', 'Color', cG, 'HandleVisibility', 'off');
h0 = plot(axE, t, e0, '--', 'Color', cG, 'LineWidth', 1.1, 'Tag', 'data');
h1 = plot(axE, t, e1, '-', 'Color', cB, 'LineWidth', 1.4, 'Tag', 'data');
eNew = nan(n, 1);
for j = find(isNew)', eNew(i0(j):i1(j)) = e1(i0(j):i1(j)); end
hR = plot(axE, t, eNew, '.', 'Color', cE, 'MarkerSize', 12);
for j = 1:nS
    if isNew(j), cc = cE; else, cc = cG; end
    kp = find(t == seg.t_peak_s(j), 1);
    text(axE, t(kp), e1(kp) + sign(e1(kp)) * 0.006, circled(j), 'Color', cc, 'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
end
ylabel(axE, '[m]');
title(axE, {'① 궤적 오차 0 - e_{lat} (블랜디드)', sprintf('RMS: LPF %.4f m, LPF 없음 %.4f m   |   점선 = ±%.0f cm', ...
    rms(e1), rms(e0), 100 * opt.eTh)}, 'FontSize', 9.5, 'FontWeight', 'normal');
xlim(axE, [0, t(end) + Ts]);
switch3_yaxis(axE, 'm');                          % 세로축: Figure 1, 2와 같은 규칙 (보통 0.02 m 간격)

% 오른쪽 ② 입력 조향각
axD = axes(fig, 'Position', pos(2, :)); hold(axD, 'on'); grid(axD, 'on'); box(axD, 'on');
modeBands(axD, t, md, Ts, S);
yline(axD, 0, 'k-', 'Alpha', 0.4, 'HandleVisibility', 'off');
hPre = plot(axD, t, rad2deg(T1.bl_delta_blend_rad), ':', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.1, 'Tag', 'data');
hPost = plot(axD, t, rad2deg(T1.bl_delta_in_rad), '-', 'Color', cB, 'LineWidth', 1.4, 'Tag', 'data');
switch3_yaxis(axD, 'deg');                        % 세로축: Figure 1, 2와 같은 규칙 (보통 5° 간격)
yd = ylim(axD);
for j = find(isNew)'                              % 오차가 갑자기 커진 구간을 빨간 테두리로
    xr = [seg.t_start_s(j), seg.t_end_s(j) + Ts];
    plot(axD, xr([1 2 2 1 1]), yd([1 1 2 2 1]), '-', 'Color', cE, 'LineWidth', 1.0, 'HandleVisibility', 'off');
end
ylabel(axD, '[deg]'); xlabel(axD, 't = kT_s [s]');
title(axD, {'② 블랜디드 입력 조향각 (LPF 실행)', '빨간 테두리 = 오차가 갑자기 커진 구간 (①의 빨간 점)'}, ...
    'FontSize', 9.5, 'FontWeight', 'normal');
xlim(axD, [0, t(end) + Ts]);

% 공통 범례 (오른쪽 위)
hKey = gobjects(1, 3);
for j = 1:3, hKey(j) = patch(axE, NaN, NaN, S.col(j, :), 'FaceAlpha', min(1, 3 * S.bandAlpha), 'EdgeColor', 'none'); end
lg = legend(axE, [h1, h0, hR, hPost, hPre, hKey], {sprintf('LPF (%s)', lpfTxt), 'LPF 없음 (ℓ 1)', '오차가 갑자기 커진 구간', ...
    '입력 조향각 LPF 후 (차량 모델 입력)', 'LPF 전 δ = λ로 합친 값', '배경: PP', '배경: PID', '배경: BC'}, ...
    'NumColumns', 4, 'FontSize', 8.5);
lg.Units = 'normalized'; lg.Position(1:2) = [0.5 - lg.Position(3) / 2, 0.945];   % 그림 맨 위 가운데

%% 4. 표 출력 + 저장
fprintf('\n=== LPF (%s, w %.2f) 튄 구간: |오차| > %.0f mm ===\n', sprintf('ell %d', round(opt.ell)), opt.wcur, 1000 * opt.eTh);
for j = 1:nS
    fprintf('%s t %.1f-%.1f s, 최대 t %.1f s (%.2f, %.2f): LPF %.0f mm / LPF 없음 %.0f mm, 앞 전환 %s @ %.1f s  [%s]\n', ...
        circled(j), seg.t_start_s(j), seg.t_end_s(j), seg.t_peak_s(j), seg.x_peak_m(j), seg.y_peak_m(j), ...
        seg.peak_lpf_mm(j), seg.peak_nolpf_mm(j), seg.switch_before(j), seg.t_switch_s(j), seg.kind(j));
end
out = struct('segments', seg, 'fig', fig, 'rms_lpf', rms(e1), 'rms_nolpf', rms(e0), 'file', "");
if opt.save
    gDir = fullfile(here, 'graph'); if ~isfolder(gDir), mkdir(gDir); end
    f = fullfile(gDir, "분석_LPF오차급증_" + string(datetime('now', 'Format', 'yyyyMMdd')));
    exportgraphics(fig, f + ".png", 'Resolution', 130);
    exportgraphics(fig, f + ".svg", 'ContentType', 'vector');
    writetable(seg, f + ".csv", 'Encoding', 'UTF-8');
    out.file = f + ".png";
    fprintf('저장: %s.png, .csv\n', f);
end
if ~opt.show, close(fig); end
end

function s = pathS(ref, T, k)
% 그 스텝의 블랜디드 차 경로 위치 s [m] (경로 투영점에서)
e = path_error(ref, T.bl_proj_x_m(k), T.bl_proj_y_m(k), 0);
s = e.s;
end

function h = modeBands(ax, t, md, Ts, S)
% 배경 띠: 같은 제어기가 이어지는 구간마다 직사각형 (switch3_fig1과 같은 색·투명도)
h = gobjects(1, 3);
for j = 1:3
    on = md(:) == S.modes(j); vx = NaN; vy = NaN;
    if any(on)
        d = diff([false; on; false]); a = find(d == 1); b = find(d == -1) - 1;
        vx = [t(a)'; t(b)' + Ts; t(b)' + Ts; t(a)'];
        vy = repmat([-1e4; -1e4; 1e4; 1e4], 1, numel(a));
    end
    h(j) = patch(ax, vx, vy, S.col(j, :), 'FaceAlpha', S.bandAlpha, 'EdgeColor', 'none', ...
        'HitTest', 'off', 'HandleVisibility', 'off');
end
end

function c = circled(j)
nums = '①②③④⑤⑥⑦⑧⑨⑩⑪⑫⑬⑭⑮⑯⑰⑱⑲⑳';
if j <= numel(nums), c = nums(j); else, c = sprintf('(%d)', j); end
end

function [lx, ly] = labelPos(x, y, p)
% 라벨 중심: 트랙 바깥(맵 중심 반대쪽)으로 조금 띄우되, 맵 밖으로 나가면 그 방향만 안쪽으로 뒤집음
hw = 1.15; hh = 0.45;                             % 라벨 반폭·반높이 [m] (대략)
v = [x, y] / max(norm([x, y]), 1e-6);
dx = 1.1 * v(1); dy = 0.75 * v(2);
if abs(x + dx) + hw > p.mapHalfX - 0.1, dx = -sign(x) * (hw + 0.8); end
if abs(y + dy) + hh > p.mapHalfY - 0.1, dy = -sign(y) * (hh + 0.6); end
lx = x + dx; ly = y + dy;
end
