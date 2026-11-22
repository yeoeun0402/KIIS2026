function e = path_error(ref, x, y, yaw, hint)
% 차량 위치(뒤 차축 중앙)와 레퍼런스 경로 사이의 오차 (모든 제어기 공용)
%   ref  : load_reference_path() 결과 (폐루프)
%   hint : 이전 스텝의 최근접 인덱스. 비우면 전체에서 찾음
%          (있으면 그 주변만 찾아서, 가까이 지나가는 다른 구간으로 튀지 않게 함)
% 반환 e:
%   idx, x_w, y_w : 가장 가까운 경로점 p_w
%   x_proj, y_proj, s : 경로 선분 위로 수직 투영한 점과 그 호길이 [m]
%   psi, kappa    : 투영점의 경로 방향 [rad]과 곡률 [1/m]
%   e_lat         : 횡오차 [m] (경로 진행 방향 기준 왼쪽 +)
%   e_head        : 방향 오차 [rad] (yaw - psi, -pi ~ pi)

N = ref.N;
if nargin < 5 || isempty(hint) || isnan(hint)
    cand = (1:N)';
else
    nBack = round(0.5 / ref.ds);   % 뒤로 0.5 m
    nFwd  = round(2.0 / ref.ds);   % 앞으로 2.0 m
    cand  = mod(hint - 1 + (-nBack:nFwd)', N) + 1;
end

d2 = (ref.x(cand) - x).^2 + (ref.y(cand) - y).^2;
[d2min, k] = min(d2);
if ~(nargin < 5 || isempty(hint) || isnan(hint)) && d2min > 2.0^2
    % 창 안에서 2 m 넘게 떨어졌으면 차가 경로를 잃은 것 → 전체에서 다시 찾음
    cand = (1:N)';
    d2 = (ref.x - x).^2 + (ref.y - y).^2;
    [~, k] = min(d2);
end
i = cand(k);

%% 최근접점 앞뒤 두 선분에 수직 투영해서 더 가까운 쪽 사용
best = inf;
for seg = [mod(i - 2, N) + 1, i]          % (i-1 -> i), (i -> i+1)
    a = seg; b = mod(seg, N) + 1;
    dx = ref.x(b) - ref.x(a); dy = ref.y(b) - ref.y(a);
    len2 = dx^2 + dy^2;
    t = min(max(((x - ref.x(a)) * dx + (y - ref.y(a)) * dy) / len2, 0), 1);
    px = ref.x(a) + t * dx; py = ref.y(a) + t * dy;
    dist2 = (x - px)^2 + (y - py)^2;
    if dist2 < best
        best = dist2;
        e.x_proj = px; e.y_proj = py;
        e.s      = mod(ref.s(a) + t * sqrt(len2), ref.L);
        e.psi    = atan2(dy, dx);
        e.kappa  = (1 - t) * ref.kappa(a) + t * ref.kappa(b);
    end
end

e.idx = i;
e.x_w = ref.x(i);
e.y_w = ref.y(i);
e.e_lat  = -sin(e.psi) * (x - e.x_proj) + cos(e.psi) * (y - e.y_proj);
e.e_head = atan2(sin(yaw - e.psi), cos(yaw - e.psi));
end
