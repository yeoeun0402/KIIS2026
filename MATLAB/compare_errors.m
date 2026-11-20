function compare_errors(ppFolder, pidFolder)
% PP와 PID의 오차 e[t] = 0 - e_lat[t] (목표 - 현재)를 한 그래프에 겹쳐 그림
%   compare_errors                       % results에서 가장 최근의 pp_discrete, pid_discrete 결과
%   compare_errors(ppFolder, pidFolder)  % 비교할 결과 폴더를 직접 지정
% 그림은 results/compare_<날짜_시간>.png로 저장

here = fileparts(mfilename('fullpath'));
res  = fullfile(here, 'results');
if nargin < 1 || isempty(ppFolder),  ppFolder  = latest(res, '*_pp_discrete_*');  end
if nargin < 2 || isempty(pidFolder), pidFolder = latest(res, '*_pid_discrete_*'); end

A = readtable(fullfile(ppFolder, 'timeseries.csv'));
B = readtable(fullfile(pidFolder, 'timeseries.csv'));
ePP  = 0 - A.e_lat_m;
ePID = 0 - B.e_lat_m;

fig = figure('Name', 'PP vs PID 오차 (목표 - 현재)', 'Position', [100 100 1000 450]);
plot(A.t_s, ePP, 'b-', 'LineWidth', 1.2); hold on;
plot(B.t_s, ePID, 'r-', 'LineWidth', 1.2);
yline(0, 'k-');
grid on;
xlabel('t = kT_s [s]');
ylabel('e[t] = 0 - e_{lat}[t] [m]');
legend({sprintf('Pure pursuit (MSE %.6f m^2, RMS %.4f m)', mean(ePP.^2), sqrt(mean(ePP.^2))), ...
        sprintf('PID (MSE %.6f m^2, RMS %.4f m)', mean(ePID.^2), sqrt(mean(ePID.^2))), ''}, 'Location', 'best');
[~, a] = fileparts(ppFolder); [~, b] = fileparts(pidFolder);
title({'횡오차 비교 (목표 - 현재)', strrep(a, '_', '\_') + "  vs  " + strrep(b, '_', '\_')}, 'FontWeight', 'normal');
out = fullfile(res, "compare_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + ".png");
exportgraphics(fig, out, 'Resolution', 110);
fprintf('비교 그림 저장: %s\n', out);
end

function f = latest(res, pattern)
d = dir(fullfile(res, pattern));
d = d([d.isdir]);
if isempty(d), error("results에 %s 결과가 없습니다. 먼저 실행하세요.", pattern); end
[~, i] = max([d.datenum]);
f = fullfile(res, d(i).name);
end
