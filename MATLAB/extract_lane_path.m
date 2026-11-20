function ref = extract_lane_path(varargin)
% Unity 맵 텍스처(image.png)에서 차선 중심선을 뽑아 레퍼런스 경로로 저장
%
% 사용 예:
%   extract_lane_path                    % 바깥 차선 (차가 지금 서 있는 차선)
%   extract_lane_path('lane', 'inner')   % 안쪽 차선
%
% 순서:
%   1) 색으로 아스팔트·흰 페인트 구분 → 점선·표시선을 메워 도로 마스크 만들기
%   2) contourc로 도로의 바깥/안쪽 가장자리(흰 실선 바깥 끝) 두 개 얻기
%   3) 픽셀 → Unity 월드 좌표 (텍스처가 월드에서 180도 돌아가 있음)
%   4) 차선 중심 = 바깥 + f*(안쪽 - 바깥), f = 0.25(바깥 차선) / 0.75(안쪽 차선)
%   5) 폐루프 스무딩 → ds 간격 재샘플링 → s, psi, kappa 계산
%   6) 차 출발 위치에서 시작, 차 진행 방향과 같은 방향으로 정렬
% 결과: reference_path_<lane>_lane.mat / .csv / .png (이 폴더)

opt.lane      = 'outer';
opt.ds        = 0.01;      % 경로점 간격 [m] (논문: 0.01 m)
opt.smoothPos = 0.40;      % 위치 스무딩 창 [m]
opt.smoothK   = 0.30;      % 곡률 스무딩 창 [m]
opt.imageFile = '';
for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k + 1}; end

here = fileparts(mfilename('fullpath'));
if isempty(opt.imageFile)
    opt.imageFile = fullfile(here, '..', 'KIIS2026_AutoCar', 'Assets', 'image.png');
end
switch opt.lane
    case 'outer', f = 0.25;
    case 'inner', f = 0.75;
    otherwise, error("lane은 'outer' 또는 'inner' 입니다.");
end
p = vehicle_params();

%% 1. 도로 마스크
I = double(imread(opt.imageFile));
[H, W, ~] = size(I);
mx = max(I, [], 3); mn = min(I, [], 3); avg = mean(I, 3);
asphalt = (mx - mn) < 20 & avg > 55 & avg < 105;     % 진회색 (약 80,74,76)
white   = mn > 200;

road = closeMask(asphalt, 10);                       % 점선·표시선·횡단보도 메우기
road = road | (white & dilateMask(road, 8));         % 가장자리 흰 실선까지 포함
road = closeMask(road, 3);

%% 2. 바깥/안쪽 가장자리
C = contourc(double(road), [0.5 0.5]);
curves = {};
k = 1;
while k < size(C, 2)
    n = C(2, k);
    curves{end + 1} = C(:, k + 1:k + n)'; %#ok<AGROW>   % [col, row]
    k = k + n + 1;
end
len = cellfun(@(c) size(c, 1), curves);
[~, order] = sort(len, 'descend');
c1 = curves{order(1)}; c2 = curves{order(2)};

%% 3. 픽셀 → 월드 좌표 (Plane UV: u=0 at X=+halfX, v=0 at Z=+halfY, 이미지 위쪽 = v 1)
toWorld = @(c) [p.mapHalfX - 2 * p.mapHalfX * (c(:, 1) - 0.5) / W, ...
               -p.mapHalfY + 2 * p.mapHalfY * (c(:, 2) - 0.5) / H];
b1 = toWorld(c1); b2 = toWorld(c2);
if polyarea(b1(:, 1), b1(:, 2)) > polyarea(b2(:, 1), b2(:, 2))
    outer = b1; inner = b2;
else
    outer = b2; inner = b1;
end
outer = resampleClosed(outer, opt.ds);
inner = resampleClosed(inner, opt.ds);

%% 4. 차선 중심: 바깥 가장자리 각 점에서 가장 가까운 안쪽 가장자리 점까지의 f 지점
nearestIn = zeros(size(outer));
for i0 = 1:2000:size(outer, 1)
    i1 = min(i0 + 1999, size(outer, 1));
    d2 = (outer(i0:i1, 1) - inner(:, 1)').^2 + (outer(i0:i1, 2) - inner(:, 2)').^2;
    [~, j] = min(d2, [], 2);
    nearestIn(i0:i1, :) = inner(j, :);
end
center = outer + f * (nearestIn - outer);
laneWidth = vecnorm(nearestIn - outer, 2, 2) / 2;

%% 5. 폐루프 스무딩 → 재샘플링 → 방향·곡률
center = smoothClosed(center, round(opt.smoothPos / opt.ds));
center = resampleClosed(center, opt.ds);
N = size(center, 1);
x = center(:, 1); y = center(:, 2);

h = 10;                                              % 미분 간격 (10점 = 10 cm)
ip = mod((0:N - 1)' + h, N) + 1; im = mod((0:N - 1)' - h, N) + 1;
dx  = (x(ip) - x(im)) / (2 * h * opt.ds);
dy  = (y(ip) - y(im)) / (2 * h * opt.ds);
ddx = (x(ip) - 2 * x + x(im)) / (h * opt.ds)^2;
ddy = (y(ip) - 2 * y + y(im)) / (h * opt.ds)^2;
psi   = atan2(dy, dx);
kappa = (dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^1.5;
kappa = smoothClosed(kappa, round(opt.smoothK / opt.ds));

%% 6. 출발 위치·방향 맞추기
sx = p.startUnity(1); sy = p.startUnity(2);
syaw = deg2rad(90 - p.startUnity(3));
[~, i0] = min((x - sx).^2 + (y - sy).^2);
if cos(psi(i0) - syaw) < 0                           % 진행 방향이 반대면 뒤집기
    x = flipud(x); y = flipud(y); kappa = -flipud(kappa);
    psi = atan2(sin(flipud(psi) + pi), cos(flipud(psi) + pi));
    i0 = N - i0 + 1;
end
sh = -(i0 - 1);
x = circshift(x, sh); y = circshift(y, sh); psi = circshift(psi, sh); kappa = circshift(kappa, sh);
seg = hypot(diff([x; x(1)]), diff([y; y(1)]));

ref.x = x; ref.y = y;
ref.s = [0; cumsum(seg(1:end - 1))];
ref.psi = psi; ref.kappa = kappa;
ref.N = N; ref.ds = opt.ds; ref.L = sum(seg); ref.closed = true;
ref.lane = string(opt.lane);
ref.source = string(opt.imageFile);
ref.created = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
ref.name = "reference_path_" + opt.lane + "_lane";

%% 저장
base = fullfile(here, ref.name);
save(base + ".mat", 'ref');
T = table((1:N)', x, y, ref.s, psi, kappa, ...
    'VariableNames', {'idx', 'x_m', 'y_m', 's_m', 'psi_rad', 'kappa_1pm'});
writetable(T, base + ".csv");

startErr = path_error(ref, sx, sy, syaw);
fprintf('[%s] 점 %d개, 한 바퀴 %.2f m, |kappa| 최대 %.3f 1/m (반경 %.2f m), 차선 폭 %.2f~%.2f m\n', ...
    ref.name, N, ref.L, max(abs(kappa)), 1 / max(abs(kappa)), min(laneWidth), max(laneWidth));
fprintf('  출발 위치 (%.2f, %.2f)에서 횡오차 %.3f m, 방향오차 %.1f deg\n', ...
    sx, sy, startErr.e_lat, rad2deg(startErr.e_head));
if max(abs(kappa)) > 1 / p.Rmin
    warning("경로 최대 곡률이 차량 최소 회전반경(%.2f m)보다 급합니다.", p.Rmin);
end

%% 확인용 그림 (맵 위에 겹쳐서)
fig = figure('Visible', 'off', 'Position', [100 100 1400 950]);
ax = axes(fig);
draw_map(ax, p);
plot(ax, outer(:, 1), outer(:, 2), 'c-', 'LineWidth', 1);
plot(ax, inner(:, 1), inner(:, 2), 'c-', 'LineWidth', 1);
plot(ax, x, y, 'y-', 'LineWidth', 2);
plot(ax, x(1), y(1), 'ro', 'MarkerFaceColor', 'r');
quiver(ax, x(1), y(1), 0.8 * cos(psi(1)), 0.8 * sin(psi(1)), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
title(ax, sprintf('%s  (L = %.2f m, 빨간 점 = 출발)', strrep(ref.name, '_', '\_'), ref.L));
exportgraphics(fig, base + ".png", 'Resolution', 110);
close(fig);
end

%% ---- 보조 함수 ----
function m = dilateMask(m, r)
[u, v] = meshgrid(-r:r);
m = conv2(double(m), double(u.^2 + v.^2 <= r^2), 'same') > 0.5;
end

function m = closeMask(m, r)
m = ~dilateMask(~dilateMask(m, r), r);   % 팽창 후 침식
end

function out = resampleClosed(xy, ds)
% 닫힌 곡선을 호길이 기준 ds 간격으로 다시 샘플링
xy = xy(any(diff([xy; xy(1, :)]) ~= 0, 2), :);             % 겹친 점 제거
seg = vecnorm(diff([xy; xy(1, :)]), 2, 2);
s = [0; cumsum(seg)];
L = s(end);
sNew = (0:ds:L - ds / 2)';
pad = 5;
sP = [s(end - pad:end - 1) - L; s(1:end - 1); s(1:pad) + L];
xyP = [xy(end - pad + 1:end, :); xy; xy(1:pad, :)];
out = interp1(sP, xyP, sNew, 'spline');
end

function y = smoothClosed(y, win)
% 닫힌 데이터용 가우시안 스무딩 (양 끝을 이어 붙여서 계산)
n = size(y, 1);
pad = min(n, 2 * win);
yp = [y(end - pad + 1:end, :); y; y(1:pad, :)];
yp = smoothdata(yp, 1, 'gaussian', win);
y = yp(pad + 1:pad + n, :);
end
