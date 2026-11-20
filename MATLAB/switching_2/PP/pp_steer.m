function [dCmd, mem, d] = pp_steer(car, ref, p, g, mem)
% 순수 Pure Pursuit 조향 한 스텝 (switching_2용)
%   Pure-Pursuit 폴더의 식 (pursuit_steer.m, pp_discrete.m)과 같고,
%   blended_control/blended_discrete.m의 초록 "PP" 차(K_pp = 1, PID 항 0, 필터 없음)와 같은 값을 냄
%
%   선행점 p_l = 차를 경로에 투영한 점에서 경로를 따라 Lf [m] 앞
%   α     = 차 방향과 (차 → p_l) 방향 사이 각,   L_ad = 차와 p_l 사이 거리
%   δ_PP  = atan( 2 L sin α / L_ad )
%
%   car : 차 상태 (x, y, yaw, delta) ← bicycle_discrete_step.m
%   g   : g.Lf [m]
%   mem : 내부 상태 (hint = 이전 최근접 경로점 인덱스). 처음에는 []
%   반환 d: e (path_error 결과), eLat = 0 - e_lat, alpha, Lad, xl, yl (선행점)

if isempty(mem), mem = struct('hint', NaN); end
e = path_error(ref, car.x, car.y, car.yaw, mem.hint);
mem.hint = e.idx;

ql = path_pose(ref, e.s + g.Lf);
dx = ql.x - car.x; dy = ql.y - car.y;
alpha = atan2(dy, dx) - car.yaw;
alpha = atan2(sin(alpha), cos(alpha));
Lad   = hypot(dx, dy);
dCmd  = atan(2 * p.L * sin(alpha) / Lad);

d = struct('e', e, 'eLat', 0 - e.e_lat, 'alpha', alpha, 'Lad', Lad, 'xl', ql.x, 'yl', ql.y);
end
