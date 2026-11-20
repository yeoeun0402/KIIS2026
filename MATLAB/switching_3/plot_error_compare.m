function fig = plot_error_compare(dataDir, varargin)
% 궤적 오차 비교 그림: Figure 1 ③(PP / PID / BC)과 Figure 2 아래 그래프(블랜디드)를
% 같은 크기·같은 시간축·같은 세로 범위·같은 눈금 간격으로 위아래에 놓음
%   (a) PP / PID / BC 차의 궤적 오차 (각 제어기만 쓴 차)
%   (b) 블랜디드 차의 궤적 오차 (PP / PID / BC 스위칭)
%   궤적 오차 = 레퍼런스 경로 - 주행 경로 = 0 - e_lat [m] (뒤 차축 중앙, 진행 방향 오른쪽 +)
%   두 그래프 배경 = 블랜디드가 그 시각에 쓴 제어기 (PP 초록, PID 파랑, BC 노랑), 세로축은 switch3_yaxis로 함께 맞춤
%   저장: graph/<실행시각>/error_compare.png, .svg
%
% 사용 예:
%   plot_error_compare('20261117_004540')
%   plot_error_compare(folder, 'save', false)        % 저장하지 않고 보기만
%   plot_error_compare(folder, 'show', false)        % 저장만

opt.show = true; opt.save = true;
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i + 1}; end
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here), '-end'); addpath(here);
dataDir = char(dataDir);
if ~isfolder(dataDir) && isfolder(fullfile(here, 'data', dataDir)), dataDir = fullfile(here, 'data', dataDir); end
[~, runName] = fileparts(dataDir);
T = readtable(fullfile(dataDir, 'timeseries.csv'), 'TextType', 'string', 'Delimiter', ',');
S = switch3_style();
t = T.t_s; Ts = median(diff(t)); if isnan(Ts), Ts = 0.1; end
md = string(T.mode);
E = [T.pp_e_traj_m, T.pid_e_traj_m, T.bc_e_traj_m, T.bl_e_traj_m];
rmsE = sqrt(mean(E .^ 2, 1, 'omitnan')); maxE = max(abs(E), [], 1);
lpf = "";
if ismember('p_ell', T.Properties.VariableNames) && T.p_ell(1) > 1
    lpf = sprintf(", LPF ℓ %d, w_{current} %.2f", round(T.p_ell(1)), T.p_wcur(1));
end
nSw = sum(md(2:end) ~= md(1:end - 1));

vis = 'on'; if ~opt.show, vis = 'off'; end
fig = figure('Name', "궤적 오차 비교 - " + runName, 'NumberTitle', 'off', 'Color', 'w', ...
    'Position', [60 60 1400 760], 'Visible', vis);
pos = [0.07 0.53 0.90 0.32; 0.07 0.09 0.90 0.32];   % 두 그래프 같은 크기
ax = gobjects(1, 2);
for a = 1:2
    ax(a) = axes(fig, 'Position', pos(a, :)); hold(ax(a), 'on'); grid(ax(a), 'on'); box(ax(a), 'on');
    modeBands(ax(a), t, md, Ts, S);
    yline(ax(a), 0, 'k-', 'Alpha', 0.4, 'HandleVisibility', 'off');
    ylabel(ax(a), '0 - e_{lat} [m]');
end
hBC = plot(ax(1), t, E(:, 3), '-', 'Color', S.col(3, :), 'LineWidth', 2.2, 'Tag', 'data');
hPP = plot(ax(1), t, E(:, 1), '-', 'Color', S.col(1, :), 'LineWidth', 1.3, 'Tag', 'data');
hPID = plot(ax(1), t, E(:, 2), '-', 'Color', S.col(2, :), 'LineWidth', 1.3, 'Tag', 'data');
hBl = plot(ax(2), t, E(:, 4), '-', 'Color', S.col(4, :), 'LineWidth', 1.4, 'Tag', 'data');
title(ax(1), sprintf(['(a) 각 제어기만 쓴 차:  PP RMS %.4f m (최대 %.3f)   |   PID RMS %.4f m (최대 %.3f)   |   ' ...
    'BC RMS %.4f m (레퍼런스)'], rmsE(1), maxE(1), rmsE(2), maxE(2), rmsE(3)), 'FontSize', 10.5, 'FontWeight', 'normal');
title(ax(2), sprintf('(b) 블랜디드 (PP / PID / BC 스위칭%s, 전환 %d번):  RMS %.4f m (최대 %.3f)', ...
    lpf, nSw, rmsE(4), maxE(4)), 'FontSize', 10.5, 'FontWeight', 'normal');
xlabel(ax(2), 't = kT_s [s]');
set(ax, 'XLim', [0, t(end) + Ts], 'XTick', 0:5:t(end) + Ts, 'FontSize', 10);
switch3_yaxis(ax, 'm');                           % 두 그래프 같은 세로 범위·같은 눈금 간격
linkaxes(ax, 'xy');

hKey = gobjects(1, 3);                            % 배경 색 견본 (범례용)
for j = 1:3, hKey(j) = patch(ax(1), NaN, NaN, S.col(j, :), 'FaceAlpha', min(1, 3 * S.bandAlpha), 'EdgeColor', 'none'); end
lg = legend(ax(1), [hPP hPID hBC hBl hKey], {'PP', 'PID', 'BC (레퍼런스)', '블랜디드', ...
    '배경: PP 구간', '배경: PID 구간', '배경: BC 구간'}, 'Orientation', 'horizontal', 'FontSize', 9.5);
lg.Units = 'normalized'; lg.Position(1:2) = [0.5 - lg.Position(3) / 2, 0.905];
annotation(fig, 'textbox', [0.07 0.945 0.9 0.04], 'EdgeColor', 'none', 'FontSize', 11, 'Interpreter', 'none', ...
    'String', sprintf('%s   궤적 오차 비교 (레퍼런스 경로 - 주행 경로, 같은 시간축·같은 세로축, 배경 = 블랜디드가 쓴 제어기)', runName));

if opt.save
    gDir = fullfile(here, 'graph', runName);
    if ~isfolder(gDir), mkdir(gDir); end
    exportgraphics(fig, fullfile(gDir, 'error_compare.png'), 'Resolution', 130);
    exportgraphics(fig, fullfile(gDir, 'error_compare.svg'), 'ContentType', 'vector');
    fprintf('저장: %s (error_compare.png, .svg)\n', gDir);
end
if ~opt.show, close(fig); end
end

function modeBands(ax, t, md, Ts, S)
% 배경 띠: 블랜디드가 같은 제어기를 쓴 구간마다 직사각형 (Figure 1, 2와 같은 색·투명도)
for j = 1:3
    on = md(:) == S.modes(j); if ~any(on), continue; end
    d = diff([false; on; false]); a = find(d == 1); b = find(d == -1) - 1;
    patch(ax, [t(a)'; t(b)' + Ts; t(b)' + Ts; t(a)'], repmat([-1e4; -1e4; 1e4; 1e4], 1, numel(a)), S.col(j, :), ...
        'FaceAlpha', S.bandAlpha, 'EdgeColor', 'none', 'HitTest', 'off', 'HandleVisibility', 'off');
end
end
