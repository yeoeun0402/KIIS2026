function ref = load_reference_path(file)
% 레퍼런스 경로 불러오기 (extract_lane_path.m이 만든 .mat 또는 .csv)
% 반환 ref: x, y, s, psi, kappa (열벡터), N, ds, L(한 바퀴 길이), closed, name

if nargin < 1 || isempty(file)
    file = fullfile(fileparts(mfilename('fullpath')), 'reference_path_outer_lane.mat');
end
if ~isfile(file)
    error("레퍼런스 경로 파일이 없습니다: %s\n먼저 extract_lane_path 를 실행하세요.", file);
end

[~, name, ext] = fileparts(file);
if strcmpi(ext, '.mat')
    S = load(file, 'ref');
    ref = S.ref;
else
    M = readmatrix(file);              % 열: idx, x, y, s, psi, kappa
    ref.x = M(:, 2); ref.y = M(:, 3); ref.s = M(:, 4);
    ref.psi = M(:, 5); ref.kappa = M(:, 6);
    ref.N = numel(ref.x);
    ref.ds = median(diff(ref.s));
    ref.L = ref.s(end) + hypot(ref.x(1) - ref.x(end), ref.y(1) - ref.y(end));
    ref.closed = true;
end
ref.name = string(name);
end
