function T = run_sweep(grid, varargin)
% 파라미터를 여러 값으로 바꿔 가며 오프라인(Unity 없이, 최대 속도)으로 돌리고 요약을 표로 모음
%   grid    : {'이름1', 값들1, '이름2', 값들2, ...}  → 모든 조합을 실행
%   varargin: 모든 실행에 공통으로 줄 옵션 (예: 'mode', 'paper')
%
% 사용 예:
%   run_sweep({'Lf', [0.6 1.0 1.5 2.3]}, 'mode', 'fixed')
%   run_sweep({'alpha_max', [0.5 1 2], 'beta_max', [0.5 0.95 2]}, 'mode', 'paper')   % 논문 Table 1처럼
%
% 결과: 각 실행은 results/ 아래 폴더로, 비교표는 results/sweep_<날짜_시간>.csv 로 저장

names = grid(1:2:end);
vals  = grid(2:2:end);
G = cell(size(vals));
[G{:}] = ndgrid(vals{:});
G = cellfun(@(g) g(:), G, 'UniformOutput', false);
nRun = numel(G{1});

rows = cell(nRun, 1);
for r = 1:nRun
    args = {};
    tag = "sweep";
    for k = 1:numel(names)
        args = [args, names(k), {G{k}(r)}]; %#ok<AGROW>
        tag = tag + "_" + names{k} + "-" + string(G{k}(r));
    end
    fprintf('(%d/%d) %s\n', r, nRun, tag);
    S = pure_pursuit_control(args{:}, varargin{:}, 'useUnity', false, 'realtime', false, ...
        'showFigure', false, 'tag', tag);
    row = cell2struct(num2cell(cellfun(@(g) g(r), G)), names, 2);
    row.mode = S.mode; row.completed = S.completed; row.lap_time = S.lap_time;
    row.e_lat_mean_abs = S.e_lat_mean_abs; row.e_lat_rms = S.e_lat_rms; row.e_lat_mse = S.e_lat_mse;
    row.e_lat_max_abs = S.e_lat_max_abs; row.e_head_rms_deg = S.e_head_rms_deg;
    row.sat_ratio = S.sat_ratio; row.folder = S.folder;
    rows{r} = row;
end
T = struct2table([rows{:}]');

out = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', ...   % MATLAB/results
    "sweep_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + ".csv");
writetable(T, out);
disp(T(:, 1:end - 1));
fprintf('비교표 저장: %s\n', out);
end
