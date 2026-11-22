function [pl, dbg] = select_lookahead(st, ref, pp, p, e, v_cmd)
% 모드별 선행점 선택 (pure pursuit에서 모드마다 다른 부분은 여기뿐)
%   st    : 차량 상태 (x, y, yaw, v, delta)
%   e     : path_error() 결과 (e.idx = 최근접 경로점 p_w)
%   v_cmd : 목표 속도 (paper 모드의 r_min 계산에 사용)
% 반환:
%   pl  : 최종 선행점 [x y]
%   dbg : 기록·표시용 내부값 (i_w, x_w, y_w, i_d, x_d, y_d, x_l, y_l,
%         rmin, alpha, beta, tau, phi, dubins_len, cur_pw, cur_pd)

iw = e.idx;
dbg = struct('i_w', iw, 'x_w', ref.x(iw), 'y_w', ref.y(iw), ...
    'i_d', NaN, 'x_d', NaN, 'y_d', NaN, 'x_l', NaN, 'y_l', NaN, ...
    'rmin', NaN, 'alpha', NaN, 'beta', NaN, 'tau', NaN, 'phi', NaN, ...
    'dubins_len', NaN, 'cur_pw', NaN, 'cur_pd', NaN);

switch pp.mode
    case "fixed"
        id = indexAhead(ref, iw, pp.Lf);

    case "speed"
        Lf = min(max(pp.t_la * abs(st.v), pp.Lmin), pp.Lmax);
        id = indexAhead(ref, iw, Lf);

    case "paper"
        % 식 4: 속도에 따른 최소 회전반경 (차량 한계 ~ r_up 사이로 자름)
        rmin = v_cmd^2 / (9.79 * (pp.e_max + pp.f_max));
        rmin = min(max(rmin, p.Rmin), pp.r_up);
        dbg.rmin = rmin;

        % Step 1 (Algorithm 1): Dubins 길이가 가장 짧은 경로점 = p_d
        id = NaN;
        if pp.step1_on
            [id, dbg.dubins_len] = step1Dubins(st, ref, pp, iw, rmin);
        end
        if isnan(id)
            id = indexAhead(ref, iw, rmin);
        end
end

dbg.i_d = id;
dbg.x_d = ref.x(id);
dbg.y_d = ref.y(id);
pl = [dbg.x_d, dbg.y_d];

% Step 2 (식 5~10): 코너에 들어갈 때 p_d를 p_w 접선 쪽(코너 바깥)으로 옮김
if pp.mode == "paper" && pp.step2_on
    wd    = [dbg.x_d - dbg.x_w, dbg.y_d - dbg.y_w];
    th_wd = atan2(wd(2), wd(1));
    phi   = atan2(sin(ref.psi(iw) - th_wd), cos(ref.psi(iw) - th_wd));   % 식 7
    len_dl = norm(wd) * tan(min(abs(phi), pp.phi_max));                   % 식 6
    th_dl  = th_wd + sign(phi) * pi / 2;   % p_l이 p_w 접선 위에 오는 쪽 (설계 문서 2.7)

    alpha  = min(hypot(st.x - dbg.x_w, st.y - dbg.y_w) / pp.alpha_max, 1);   % 식 9
    cur_pw = abs(ref.kappa(iw));
    cur_pd = abs(ref.kappa(id));
    if cur_pw >= cur_pd                                                   % 식 10
        beta = 0;
    else
        beta = min((cur_pd - cur_pw) / pp.beta_max, 1);
    end
    tau = (1 - alpha) * beta;                                             % 식 8

    pl = pl + tau * len_dl * [cos(th_dl), sin(th_dl)];                    % 식 5
    dbg.phi = phi; dbg.alpha = alpha; dbg.beta = beta; dbg.tau = tau;
    dbg.cur_pw = cur_pw; dbg.cur_pd = cur_pd;
elseif pp.mode == "paper"
    dbg.tau = 0;
end
dbg.x_l = pl(1);
dbg.y_l = pl(2);
end

%% ---- 보조 함수 ----
function id = indexAhead(ref, iw, dist)
% p_w에서 경로를 따라 dist [m] 앞의 경로점 (폐루프)
id = mod(iw - 1 + round(dist / ref.ds), ref.N) + 1;
end

function [id, len] = step1Dubins(st, ref, pp, iw, rmin)
% 후보: p_w에서 직선거리 r_min ~ d_search 인 앞쪽 경로점 (cand_step 간격)
persistent dc
if isempty(dc)
    dc = dubinsConnection('MinTurningRadius', rmin);
elseif dc.MinTurningRadius ~= rmin
    dc.MinTurningRadius = rmin;
end

nMax = round(1.5 * pp.d_search / ref.ds);                 % 곡선에서는 호길이 > 직선거리
k = mod(iw - 1 + (round(rmin / ref.ds):max(1, round(pp.cand_step / ref.ds)):nMax)', ref.N) + 1;
d = hypot(ref.x(k) - ref.x(iw), ref.y(k) - ref.y(iw));
k = k(d >= rmin & d <= pp.d_search);
id = NaN; len = NaN;
if isempty(k), return; end

startPose = [st.x, st.y, st.yaw + st.delta];             % 시작 방향 = 차 방향 + 현재 조향각
[~, cost] = connect(dc, startPose, [ref.x(k), ref.y(k), ref.psi(k)]);
[len, j] = min(cost);
id = k(j);
end
