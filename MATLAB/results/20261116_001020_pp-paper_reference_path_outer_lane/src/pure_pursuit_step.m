function [u, dbg] = pure_pursuit_step(st, ref, p, pp, e)
% Pure pursuit 한 스텝
%   입력 (= 자전거 모델의 출력): st.x, st.y, st.yaw, st.v, st.delta
%   출력 (= 자전거 모델의 입력): u.delta_cmd [rad], u.v_cmd [m/s] (pp.v_mode에 따라 곡선에서 감속)
%   e   : path_error() 결과 (최근접 경로점 p_w를 여기서 가져옴)
%   dbg : 기록·Unity 표시용 내부값 (select_lookahead + pursuit_steer + comp_ms)

tStart = tic;
v_cmd = speedCommand(ref, p, pp, e.idx);
[pl, dbg] = select_lookahead(st, ref, pp, p, e, v_cmd);
[delta_cmd, s] = pursuit_steer(st, pl, p);

dbg.Lf = s.Lf; dbg.eta = s.eta; dbg.kappa_cmd = s.kappa;
dbg.R = s.R; dbg.x_c = s.x_c; dbg.y_c = s.y_c; dbg.sat = s.sat;
dbg.delta_cmd = delta_cmd;
dbg.comp_ms = toc(tStart) * 1000;

u.delta_cmd = delta_cmd;
u.v_cmd = v_cmd;
end

function v_cmd = speedCommand(ref, p, pp, iw)
% 속도 명령 (논문 4.1절, 모든 모드 공통)
%   "const": 항상 v_tar
%   "curve": 차가 곡선 위이거나 선행점이 곡선 위면 AASHTO 속도-반경 관계(식 4를 v에 대해 풂)
%            v = sqrt(9.79 (e_max + f_max) / |kappa|) 로 감속, 아니면 v_tar
%   선행점 위치는 v_tar 기준 r_min만큼 경로를 따라 앞 (경로 위 p_d가 놓이는 거리)
v_cmd = pp.v_tar;
if pp.v_mode == "curve"
    g_f    = 9.79 * (pp.e_max + pp.f_max);
    rAhead = min(max(pp.v_tar^2 / g_f, p.Rmin), pp.r_up);
    iAhead = mod(iw - 1 + round(rAhead / ref.ds), ref.N) + 1;
    kap    = max(abs(ref.kappa(iw)), abs(ref.kappa(iAhead)));
    v_cmd  = min(pp.v_tar, sqrt(g_f / kap));
end
end
