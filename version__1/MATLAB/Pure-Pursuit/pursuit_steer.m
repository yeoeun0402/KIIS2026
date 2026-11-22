function [delta_cmd, s] = pursuit_steer(st, pl, p)
% 선행점 -> 원 -> 조향각 (모든 모드 공통, 논문 식 2~3)
%   차(p_v)를 지나고 차 방향에 접하면서 선행점 p_l을 지나는 원:
%     eta = 차 방향과 (p_v -> p_l) 사이 각,  Lf = |p_l - p_v|
%     kappa = 2 sin(eta) / Lf,  R = 1/kappa (+ 왼쪽),  delta = atan(kappa * L)
% 반환 s: Lf, eta, kappa, R, x_c, y_c (원 중심), sat (조향각 한계에 걸렸는지)

dx = pl(1) - st.x;
dy = pl(2) - st.y;
s.Lf    = hypot(dx, dy);
s.eta   = atan2(sin(atan2(dy, dx) - st.yaw), cos(atan2(dy, dx) - st.yaw));
s.kappa = 2 * sin(s.eta) / max(s.Lf, 1e-6);

deltaRaw  = atan(s.kappa * p.L);
delta_cmd = min(max(deltaRaw, -p.deltaMax), p.deltaMax);
s.sat     = abs(deltaRaw) > p.deltaMax;

if abs(s.kappa) > 1e-4
    s.R   = 1 / s.kappa;
    s.x_c = st.x - s.R * sin(st.yaw);      % 원 중심 = p_v + R * (왼쪽 법선)
    s.y_c = st.y + s.R * cos(st.yaw);
else
    s.R = NaN; s.x_c = NaN; s.y_c = NaN;   % 거의 직선
end
end
