function figs = plot_switch3(dataDir, varargin)
% 저장된 switching_3 데이터 (data/<실행시각>/timeseries.csv)를 읽어 Figure 1, 2를 그리고
% graph/<실행시각>/figure1.png, figure2.png로 저장 (+ 같은 이름의 .svg 벡터 그림)
%   Figure 1: PP 초록, PID 파랑, BC 노랑 (+ 블랜디드 보라 궤적과 쓴 제어기 배경 띠)
%             오른쪽 ① 입력 조향각 ② 출력 조향각 ③ 경로 (궤적 오차)
%   Figure 2: 블랜디드 보라 vs 레퍼런스 갈색, 오차 빨강 (① 주행 경로 차이 ② 입력 조향각 차이 ③ 출력 조향각 차이)
%   두 그림 모두 체크박스로 선을 켜고 끔
%
% 사용 예:
%   plot_switch3('switching_3/data/20261116_101500')     % 다시 그리기 (창 두 개)
%   plot_switch3(folder, 'show', [false true])          % Figure 1은 저장만, Figure 2만 창으로
%   plot_switch3(folder, 'save', false)                 % 저장하지 않고 보기만

opt.show = [true true];
opt.save = true;
for i = 1:2:numel(varargin), opt.(varargin{i}) = varargin{i + 1}; end
here = fileparts(mfilename('fullpath'));
base = fileparts(here);
addpath(base, '-end'); addpath(here, fullfile(here, 'BC'));
dataDir = char(dataDir);
if ~isfolder(dataDir) && isfolder(fullfile(here, 'data', dataDir)), dataDir = fullfile(here, 'data', dataDir); end
[~, runName] = fileparts(dataDir);
T = readtable(fullfile(dataDir, 'timeseries.csv'), 'TextType', 'string', 'Delimiter', ',');
p = vehicle_params(); ref = load_reference_path(); S = switch3_style();

%% 데이터 구조체 (switch3_discrete의 D와 같은 모양)
n = height(T);
e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
q0 = path_pose(ref, e0.s);
D.Ts = median(diff(T.t_s)); if isnan(D.Ts), D.Ts = 0.1; end
D.x0 = repmat(q0.x, 1, 4); D.y0 = repmat(q0.y, 1, 4);
D.t = T.t_s;
get4 = @(suffix) cell2mat(arrayfun(@(tg) T.(tg + suffix), S.tags, 'UniformOutput', false));
D.x = get4("_x_m"); D.y = get4("_y_m");
D.din = get4("_delta_in_rad"); D.dout = get4("_delta_out_rad"); D.etraj = get4("_e_traj_m");
D.xp = T.bl_proj_x_m; D.yp = T.bl_proj_y_m;
D.mode = string(T.mode);

%% Figure 1
md = D.mode;
nSw = sum(md(2:end) ~= md(1:end - 1));
share = arrayfun(@(m) 100 * mean(md == m), S.modes);
rmsT = sqrt(mean(D.etraj .^ 2, 1, 'omitnan'));
H = switch3_fig1(p, ref, struct('withPanel', false, 'visible', opt.show(1), ...
    'name', "switching_3 Figure 1 - " + runName));
H.update(D, n, {sprintf('%s   PP / PID / BC  (블랜디드: 전환 %d번, PP %.0f%% · PID %.0f%% · BC %.0f%%)', ...
        runName, nSw, share), ...
    sprintf('궤적 오차 RMS [m]: PP %.4f   PID %.4f   BC %.4f   블랜디드 %.4f', rmsT)});
H.axMap.Title.Interpreter = 'none';
zf = fullfile(dataDir, 'pp_error_map.csv');                 % BC 구간 (PP 오차 지도), 있으면 맵에 표시
if isfile(zf)
    Zt = readtable(zf);
    if height(Zt) == ref.N, H.setZone(struct('grid', Zt.bc_zone > 0, 'eBC', Zt.eBC_m(1))); end
end

%% Figure 2
f2 = switch3_fig2(p, ref, D, struct('visible', opt.show(2), 'name', "switching_3 Figure 2 - " + runName, ...
    'subtitle', sprintf('%s   블랜디드 방식 vs 레퍼런스 (주어진 경로를 그대로 따라간 BC 차, 같은 시각 k)', runName)));

%% 저장
if opt.save
    gDir = fullfile(here, 'graph', runName);
    if ~isfolder(gDir), mkdir(gDir); end
    exportgraphics(H.fig, fullfile(gDir, 'figure1.png'), 'Resolution', 120);
    exportgraphics(f2, fullfile(gDir, 'figure2.png'), 'Resolution', 120);
    exportgraphics(H.fig, fullfile(gDir, 'figure1.svg'), 'ContentType', 'vector');   % 벡터 (체크박스는 빠짐, 맵 사진은 그림으로 들어감)
    exportgraphics(f2, fullfile(gDir, 'figure2.svg'), 'ContentType', 'vector');
    fprintf('그래프 저장: %s\n', gDir);
end
if ~opt.show(1), close(H.fig); end
if ~opt.show(2), close(f2); end
figs = struct('fig1', H.fig, 'fig2', f2);
end
