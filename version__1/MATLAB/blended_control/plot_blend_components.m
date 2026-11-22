function s = plot_blend_components(folder)
% PP_PID 조향 명령의 항별 기여를 그림: switching(PP ↔ PID 전환)이 아니라 매 스텝 더하는지 확인 (논문 Eq.10 + D)
%   plot_blend_components            % results에서 가장 최근 blended_discrete 결과
%   plot_blend_components(folder)    % 결과 폴더 지정 (timeseries.csv가 있는 폴더)
%   위: K_pp delta_pp (초록), delta_p + delta_i + delta_d (파랑), 둘의 합 delta_raw (검정), Low pass 뒤 delta (빨강)
%   아래: PP 몫 = |K_pp delta_pp| / (|K_pp delta_pp| + |delta_p + delta_i + delta_d|)
%         switching이면 0 또는 1만 나오고, blended면 그 사이 값이 계속 나옴
% 반환 s: 합 오차 최대값, 두 항이 동시에 쓰인 스텝 비율, 각 항 RMS [deg], PP 몫 평균
% 그림은 <folder>/blend_components.png로 저장

here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(folder)
    d = dir(fullfile(fileparts(here), 'results', '*_blended_discrete_*'));
    d = d([d.isdir]);
    if isempty(d), error("results에 blended_discrete 결과가 없습니다. 먼저 실행하세요."); end
    [~, i] = max([d.datenum]);
    folder = fullfile(d(i).folder, d(i).name);
end
T = readtable(fullfile(folder, 'timeseries.csv'));
if ismember('p_Kpp', T.Properties.VariableNames)
    Kpp = T.p_Kpp;
else
    Kpp = ones(height(T), 1);                       % p_Kpp 열이 없는 예전 결과: 기본값 1로 봄
    warning("timeseries.csv에 p_Kpp 열이 없어 K_pp = 1로 봅니다.");
end
ppPart  = Kpp .* T.bl_delta_pp_rad;
pidPart = T.bl_delta_p_rad + T.bl_delta_i_rad;
if ismember('bl_delta_d_rad', T.Properties.VariableNames), pidPart = pidPart + T.bl_delta_d_rad; end   % D (없으면 PP_PI 결과)
raw     = T.bl_delta_raw_rad;
share   = abs(ppPart) ./ (abs(ppPart) + abs(pidPart));

s.folder      = folder;
s.sum_err_max = max(abs(raw - (ppPart + pidPart)));           % 0이면 매 스텝 정확히 합
s.both_active = mean(abs(ppPart) > 1e-6 & abs(pidPart) > 1e-6);   % 두 항이 동시에 0이 아닌 스텝 비율
s.rms_pp_deg  = rad2deg(sqrt(mean(ppPart.^2)));
s.rms_pid_deg = rad2deg(sqrt(mean(pidPart.^2)));
s.share_pp_mean = mean(share, 'omitnan');
s.share_pp_mid  = mean(share > 0.05 & share < 0.95, 'omitnan');   % 0도 1도 아닌(섞인) 스텝 비율

fig = figure('Name', 'PP_PID 조향 명령의 항별 기여', 'NumberTitle', 'off', 'Position', [80 80 1200 700]);
ax1 = subplot(2, 1, 1, 'Parent', fig); hold(ax1, 'on'); grid(ax1, 'on');
plot(ax1, T.t_s, rad2deg(ppPart),  '-', 'Color', [0.10 0.62 0.10], 'LineWidth', 1.2);
plot(ax1, T.t_s, rad2deg(pidPart), '-', 'Color', [0.00 0.40 0.95], 'LineWidth', 1.2);
plot(ax1, T.t_s, rad2deg(raw),     'k-', 'LineWidth', 1.0);
plot(ax1, T.t_s, rad2deg(T.bl_delta_cmd_rad), '--', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.2);
legend(ax1, {sprintf('K_{pp} \\delta_{pp} (RMS %.2f deg)', s.rms_pp_deg), ...
    sprintf('\\delta_p + \\delta_i + \\delta_d (RMS %.2f deg)', s.rms_pid_deg), ...
    sprintf('합 \\delta_{raw} (합 오차 최대 %.1e rad)', s.sum_err_max), '\delta = Lowpass(\delta_{raw})'}, 'Location', 'best');
ylabel(ax1, '[deg]');
title(ax1, sprintf('PP\\_PID 조향 명령 = 매 스텝 두 항의 합 (두 항이 동시에 쓰인 스텝 %.1f%%)', 100 * s.both_active));
ax2 = subplot(2, 1, 2, 'Parent', fig); hold(ax2, 'on'); grid(ax2, 'on');
plot(ax2, T.t_s, share, '.', 'Color', [0.10 0.62 0.10]);
yline(ax2, [0 1], 'k:');
ylim(ax2, [-0.05 1.05]);
xlabel(ax2, 't = kT_s [s]'); ylabel(ax2, 'PP 몫');
title(ax2, sprintf(['PP 몫 |K_{pp}\\delta_{pp}| / (|K_{pp}\\delta_{pp}| + |\\delta_p+\\delta_i+\\delta_d|): 평균 %.2f, ' ...
    '0.05~0.95 사이 스텝 %.1f%% (switching이면 0 또는 1만)'], s.share_pp_mean, 100 * s.share_pp_mid));
linkaxes([ax1, ax2], 'x');
exportgraphics(fig, fullfile(folder, 'blend_components.png'), 'Resolution', 110);
fprintf('합 오차 최대 %.1e rad, 두 항 동시 사용 %.1f%% 스텝, PP 몫 평균 %.2f (0.05~0.95 사이 %.1f%%)\n', ...
    s.sum_err_max, 100 * s.both_active, s.share_pp_mean, 100 * s.share_pp_mid);
fprintf('그림: %s\n', fullfile(folder, 'blend_components.png'));
end
