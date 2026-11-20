function make_readme_figures()
% README.md에 들어가는 개념 그림(docs/img/*.png)을 만드는 스크립트
% 그림 속 값은 모두 프로젝트의 실제 함수(bicycle_step, pursuit_steer, select_lookahead,
% path_error, pure_pursuit_step)로 계산한 것이라, 그림과 코드가 항상 같은 내용을 보여줌
% 실행: MATLAB에서 이 파일이 있는 docs 폴더로 이동한 뒤  make_readme_figures

here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'MATLAB', 'setup_paths.m'));
out = fullfile(here, 'img');
if ~isfolder(out), mkdir(out); end
p = vehicle_params();

fig01_bicycle(out);
fig02_geometry(out, p);
fig03_path_error(out);
fig04_tradeoff(out, p);
fig05_step1(out, p);
fig06_step2(out, p);
fig07_speed(out, p);
fprintf('그림 저장: %s\n', out);
end

%% 1. 기구학 자전거 모델
function fig01_bicycle(out)
[fig, ax] = newFig(980, 640);
axis(ax, 'off');
L = 1; th = deg2rad(20); dl = deg2rad(25);
pr = [0 0]; pf = pr + L * [cos(th) sin(th)];
R = L / tan(dl);
o = pr + R * [-sin(th) cos(th)];
blue = [0.2 0.4 0.9]; red = [0.85 0.1 0.1];

plot(ax, [-0.3 0.9], [0 0], 'k:');                                            % x축 방향 기준선
plot(ax, pf(1) + [0 0.6 * cos(th)], pf(2) + [0 0.6 * sin(th)], 'k:');        % 차 방향 연장선
plot(ax, [o(1) pr(1)], [o(2) pr(2)], '--', 'Color', blue, 'LineWidth', 1.2);
plot(ax, [o(1) pf(1)], [o(2) pf(2)], '--', 'Color', blue, 'LineWidth', 1.2);
plot(ax, [pr(1) pf(1)], [pr(2) pf(2)], 'k-', 'LineWidth', 3);                % 차체 = 축간거리 L
drawBox(ax, pr, th, 0.36, 0.12, [0.25 0.25 0.25]);                            % 뒷바퀴
drawBox(ax, pf, th + dl, 0.36, 0.12, [0.25 0.25 0.25]);                       % 앞바퀴 (δ만큼 꺾임)
plot(ax, o(1), o(2), 'o', 'MarkerFaceColor', blue, 'MarkerEdgeColor', 'w', 'MarkerSize', 9);
plot(ax, pr(1), pr(2), 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'w', 'MarkerSize', 9);

angleArc(ax, pr, 0, th, 0.62, 'k');
angleArc(ax, pf, th, th + dl, 0.48, red);
a0 = atan2(pr(2) - o(2), pr(1) - o(1)); a1 = atan2(pf(2) - o(2), pf(1) - o(1));
angleArc(ax, o, a0, a1, 0.7, red);
vs = pr + 0.2 * [sin(th) -cos(th)];
quiver(ax, vs(1), vs(2), 0.6 * cos(th), 0.6 * sin(th), 0, 'Color', [0 0.6 0], 'LineWidth', 2, 'MaxHeadSize', 0.8);

text(ax, pr(1) - 0.3, pr(2) - 0.33, 'p_v (뒤 차축 중앙, 기준점)', 'Color', red, 'FontSize', 11, 'HorizontalAlignment', 'center');
text(ax, pf(1) + 0.05, pf(2) - 0.2, '앞바퀴', 'FontSize', 11);
mid = (pr + pf) / 2;
text(ax, mid(1) - 0.32, mid(2) + 0.2, 'L (축간거리)', 'FontSize', 11, 'Rotation', rad2deg(th));
text(ax, 0.68 * cos(th / 2), 0.68 * sin(th / 2), '\theta', 'FontSize', 14);
text(ax, pf(1) + 0.55 * cos(th + dl / 2), pf(2) + 0.55 * sin(th + dl / 2), '\delta', 'FontSize', 14, 'Color', red);
text(ax, o(1) + 0.78 * cos((a0 + a1) / 2), o(2) + 0.78 * sin((a0 + a1) / 2), '\delta', 'FontSize', 14, 'Color', red);
text(ax, vs(1) + 0.65 * cos(th), vs(2) + 0.65 * sin(th) - 0.06, 'v', 'FontSize', 14, 'Color', [0 0.6 0]);
mo = (o + pr) / 2;
text(ax, mo(1) - 0.12, mo(2), 'R = L / tan\delta', 'FontSize', 12, 'Color', blue, 'HorizontalAlignment', 'right');
text(ax, o(1) + 0.1, o(2) + 0.1, '회전 중심 o', 'FontSize', 11, 'Color', blue);

text(ax, 1.95, 1.95, {'\bf기구학 자전거 모델\rm', '', ...
    'dx/dt = v cos\theta', 'dy/dt = v sin\theta', 'd\theta/dt = v tan\delta / L', '', ...
    '회전 반지름  R = L / tan\delta', '곡률  \kappa = 1 / R = tan\delta / L', '', ...
    '입력: v (속도), \delta (조향각)', '상태: x, y, \theta'}, ...
    'FontSize', 12, 'VerticalAlignment', 'top', 'EdgeColor', [0.7 0.7 0.7], 'Margin', 8, 'BackgroundColor', 'w');
xlim(ax, [-1.6 3.8]); ylim(ax, [-0.45 2.35]);
saveFig(fig, out, '01_bicycle_model.png');
end

%% 2. Pure pursuit 기하: 선행점 → 원 → 조향각
function fig02_geometry(out, p)
[fig, ax] = newFig(900, 780);
yy = (-1.2:0.01:4.2)'; xx = 0.55 + 0.11 * yy.^2;               % 차 오른쪽에서 휘어지는 경로
st = struct('t', 0, 'x', 0, 'y', 0, 'yaw', pi / 2, 'v', 0.8, 'delta', 0, 'yawRate', 0);
[~, iw] = min((xx - st.x).^2 + (yy - st.y).^2);
s = [0; cumsum(hypot(diff(xx), diff(yy)))];
[~, il] = min(abs(s - (s(iw) + 2.4)));                           % p_w에서 경로를 따라 2.4 m 앞
pl = [xx(il), yy(il)];
[dcmd, g] = pursuit_steer(st, pl, p);
r = abs(g.R); mag = [0.85 0 0.85];

hP = plot(ax, xx, yy, '-', 'Color', [0.95 0.75 0], 'LineWidth', 4);
t = linspace(0, 2 * pi, 300);
hC = plot(ax, g.x_c + r * cos(t), g.y_c + r * sin(t), ':', 'Color', mag, 'LineWidth', 1);
a0 = atan2(st.y - g.y_c, st.x - g.x_c); a1 = atan2(pl(2) - g.y_c, pl(1) - g.x_c);
a = a0 - linspace(0, mod(a0 - a1, 2 * pi), 60);                 % 오른쪽 회전 → 시계 방향 호
hA = plot(ax, g.x_c + r * cos(a), g.y_c + r * sin(a), '-', 'Color', mag, 'LineWidth', 3);
plot(ax, [g.x_c st.x], [g.y_c st.y], '--', 'Color', [0.5 0.5 0.5]);
plot(ax, [g.x_c pl(1)], [g.y_c pl(2)], '--', 'Color', [0.5 0.5 0.5]);
hL = plot(ax, [st.x pl(1)], [st.y pl(2)], 'r--', 'LineWidth', 1.5);
drawBox(ax, [st.x, st.y] + 0.3 * [cos(st.yaw) sin(st.yaw)], st.yaw, 0.99, 0.56, [0.85 0.85 0.85]);
quiver(ax, st.x, st.y, 0, 1.6, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.4);
angleArc(ax, [st.x st.y], pi / 2, pi / 2 + g.eta, 0.9, 'r');
angleArc(ax, [g.x_c g.y_c], a1, a0, 0.6, 'r');
plot(ax, xx(iw), yy(iw), 'o', 'MarkerFaceColor', [0 0.45 1], 'MarkerEdgeColor', 'w', 'MarkerSize', 10);
plot(ax, pl(1), pl(2), 'o', 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', 'w', 'MarkerSize', 11);
plot(ax, st.x, st.y, 'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w', 'MarkerSize', 9);
plot(ax, g.x_c, g.y_c, 'o', 'MarkerFaceColor', mag, 'MarkerEdgeColor', 'w', 'MarkerSize', 9);

text(ax, st.x - 0.25, st.y - 0.25, 'p_v', 'FontSize', 13, 'HorizontalAlignment', 'right');
text(ax, -0.15, 1.7, '차 방향', 'FontSize', 11, 'HorizontalAlignment', 'right');
text(ax, xx(iw) + 0.1, yy(iw) - 0.3, 'p_w (가장 가까운 경로점)', 'FontSize', 11, 'Color', [0 0.35 0.8]);
text(ax, pl(1) + 0.12, pl(2) + 0.08, 'p_l (선행점)', 'FontSize', 12, 'Color', 'r');
text(ax, 0.36, 0.52, 'L_f', 'FontSize', 13, 'Color', 'r');
text(ax, 0.22, 1.0, '\eta', 'FontSize', 15, 'Color', 'r');
text(ax, g.x_c - 0.75, g.y_c + 0.35, '2\eta', 'FontSize', 14, 'Color', 'r');
text(ax, g.x_c + 0.1, g.y_c - 0.2, '원 중심 o', 'FontSize', 11, 'Color', mag);
text(ax, (g.x_c + st.x) / 2 + 0.4, 0.13, 'R', 'FontSize', 13, 'Color', [0.4 0.4 0.4]);
legend(ax, [hP, hA, hC, hL], {'레퍼런스 경로 p_{ref}', '차가 지나갈 원호 (p_v \rightarrow p_l)', ...
    'pure pursuit 원', 'L_f (p_v와 p_l 사이 직선)'}, 'Location', 'northeast');
text(ax, -1.15, 3.55, {sprintf('L_f = %.2f m', g.Lf), sprintf('\\eta = %.1f deg', rad2deg(g.eta)), ...
    sprintf('\\kappa = 2 sin\\eta / L_f = %.3f 1/m', g.kappa), sprintf('R = 1 / \\kappa = %.2f m', g.R), ...
    sprintf('\\delta = atan(\\kappa L) = %.1f deg  (L = %.2f m)', rad2deg(dcmd), p.L), ...
    '(오른쪽 회전이라 \eta, \kappa, R, \delta 가 음수)'}, ...
    'FontSize', 11, 'VerticalAlignment', 'top', 'EdgeColor', [0.7 0.7 0.7], 'Margin', 6, 'BackgroundColor', 'w');
xlim(ax, [-1.25 4.3]); ylim(ax, [-1.0 3.7]);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
title(ax, 'Pure pursuit: 차(p_v)를 지나고 차 방향에 접하면서 선행점(p_l)을 지나는 원');
saveFig(fig, out, '02_pure_pursuit_geometry.png');
end

%% 3. 오차 정의: e_lat, e_head
function fig03_path_error(out)
[fig, ax] = newFig(900, 520);
ds = 0.6;
xw = (0:ds:6.6)'; yw = 0.25 * xw;
seg = hypot(diff(xw), diff(yw));
ref = struct('x', xw, 'y', yw, 's', [0; cumsum(seg)], 'psi', atan2(diff(yw([1 2])), diff(xw([1 2]))) * ones(size(xw)), ...
    'kappa', zeros(size(xw)), 'N', numel(xw), 'ds', ds, 'L', sum(seg) + ds);
car = [2.75 1.35]; yaw = deg2rad(40);
e = path_error(ref, car(1), car(2), yaw);

plot(ax, xw, yw, '-', 'Color', [0.95 0.75 0], 'LineWidth', 4);
plot(ax, xw, yw, 'o', 'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerEdgeColor', 'w', 'MarkerSize', 7);
plot(ax, [e.x_proj car(1)], [e.y_proj car(2)], 'r-', 'LineWidth', 2);
plot(ax, e.x_w, e.y_w, 'o', 'MarkerFaceColor', [0 0.45 1], 'MarkerEdgeColor', 'w', 'MarkerSize', 10);
plot(ax, e.x_proj, e.y_proj, 'kx', 'MarkerSize', 12, 'LineWidth', 2);
quiver(ax, e.x_proj, e.y_proj, 1.1 * cos(e.psi), 1.1 * sin(e.psi), 0, 'Color', [0.75 0.55 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(ax, car(1), car(2), 1.1 * cos(yaw), 1.1 * sin(yaw), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
plot(ax, car(1) + [0 1.1 * cos(e.psi)], car(2) + [0 1.1 * sin(e.psi)], ':', 'Color', [0.75 0.55 0], 'LineWidth', 1.5);
angleArc(ax, car, e.psi, yaw, 0.8, [0 0.5 0]);
plot(ax, car(1), car(2), 'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w', 'MarkerSize', 10);

text(ax, car(1) - 0.15, car(2) + 0.12, 'p_v (차)', 'FontSize', 12, 'HorizontalAlignment', 'right');
text(ax, e.x_w + 0.1, e.y_w - 0.22, 'p_w (가장 가까운 경로점)', 'FontSize', 11, 'Color', [0 0.35 0.8]);
text(ax, e.x_proj - 0.1, e.y_proj + 0.1, '투영점', 'FontSize', 11, 'HorizontalAlignment', 'right');
text(ax, (e.x_proj + car(1)) / 2 - 0.1, (e.y_proj + car(2)) / 2 + 0.02, sprintf('e_{lat} = %+.3f m  (경로 왼쪽 +)', e.e_lat), ...
    'FontSize', 12, 'Color', 'r', 'HorizontalAlignment', 'right');
text(ax, car(1) + 0.9, car(2) + 0.42, sprintf('e_{head} = \\theta - \\psi = %+.1f deg', rad2deg(e.e_head)), 'FontSize', 12, 'Color', [0 0.5 0]);
text(ax, e.x_proj + 1.15 * cos(e.psi), e.y_proj + 1.15 * sin(e.psi) - 0.3, '\psi (경로 방향)', 'FontSize', 11, 'Color', [0.75 0.55 0]);
text(ax, car(1) + 1.15 * cos(yaw), car(2) + 1.15 * sin(yaw), '\theta (차 방향)', 'FontSize', 11);
text(ax, 3.7, 0.2, '경로 = 경로점(회색)을 이은 선분', 'FontSize', 11, 'Color', [0.4 0.4 0.4]);
xlim(ax, [-0.3 6.8]); ylim(ax, [-0.4 2.7]);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
title(ax, '오차는 가장 가까운 "선분"에 수직으로 내린 거리로 계산 (path\_error.m)');
saveFig(fig, out, '03_path_error.png');
end

%% 4. 선행 거리 L_f에 따른 차이 (같은 차, 같은 경로, 선행점만 다름)
function fig04_tradeoff(out, p)
ds = 0.01;
ref = makeRef(stadium(-5, 15, 2, ds), ds);
st0 = struct('t', 0, 'x', -4.5, 'y', -0.7, 'yaw', 0, 'v', 0.8, 'delta', 0, 'yawRate', 0);
C = {pp_params('mode', 'fixed', 'Lf', 0.3), 'fixed  L_f = 0.3 m (가까움)', [0.85 0.1 0.1];
     pp_params('mode', 'fixed', 'Lf', 0.8), 'fixed  L_f = 0.8 m', [0 0.45 0.9];
     pp_params('mode', 'fixed', 'Lf', 2.0), 'fixed  L_f = 2.0 m (멂)', [0.95 0.55 0];
     pp_params('mode', 'paper'), 'paper (논문 방법)', [0 0.6 0]};
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 1100 820]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on'); box(ax1, 'on');
ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
plot(ax1, ref.x, ref.y, '-', 'Color', [0.95 0.75 0], 'LineWidth', 5);
plot(ax1, st0.x, st0.y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
for k = 1:size(C, 1)
    T = simulate(ref, st0, C{k, 1}, p, 25);
    plot(ax1, T(:, 2), T(:, 3), '-', 'Color', C{k, 3}, 'LineWidth', 1.6);
    plot(ax2, T(:, 1), T(:, 4), '-', 'Color', C{k, 3}, 'LineWidth', 1.6);
    fprintf('fig04 %-30s max|e_lat| after 6 s: %.3f m\n', C{k, 2}, max(abs(T(T(:, 1) > 6, 4))));
end
legend(ax1, [{'레퍼런스 경로', '출발'}, C(:, 2)'], 'Location', 'northwest');
xlim(ax1, [-5.2 12.6]); ylim(ax1, [-1.2 4.8]);
xlabel(ax1, 'x [m]'); ylabel(ax1, 'y [m]');
title(ax1, '경로에서 0.7 m 떨어져 출발 → 직선 → 반경 2 m 왼쪽 코너 (v = 0.8 m/s)');
yline(ax2, 0, 'k-');
xlabel(ax2, 't [s]'); ylabel(ax2, 'e_{lat} [m]');
title(ax2, '횡오차: 가까우면 출렁이고, 멀면 늦게 붙고 코너 안쪽을 자름 (t \approx 18 s부터 코너)');
saveFig(fig, out, '04_lookahead_tradeoff.png');
end

function T = simulate(ref, st, pp, p, sStop)
% bicycle_step + pure_pursuit_step으로 경로 위 sStop [m]까지 주행
T = nan(4000, 4);
e = path_error(ref, st.x, st.y, st.yaw);
for k = 1:4000
    if k > 1, e = path_error(ref, st.x, st.y, st.yaw, e.idx); end
    T(k, :) = [st.t, st.x, st.y, e.e_lat];
    if e.s > sStop, break; end
    u = pure_pursuit_step(st, ref, p, pp, e);
    st = bicycle_step(st, u, p, p.dt);
end
T = T(1:k, :);
end

%% 5. 논문 Step 1: Dubins 길이가 가장 짧은 경로점 = p_d
function fig05_step1(out, p)
ds = 0.01;
ref = makeRef(stadium(-10, 30, 5, ds), ds);                     % y = 0 직선 (+x 방향)
pp = pp_params('mode', 'paper');
v = pp.v_tar;
rmin = min(max(v^2 / (9.79 * (pp.e_max + pp.f_max)), p.Rmin), pp.r_up);
dc = dubinsConnection('MinTurningRadius', rmin);
S = {struct('t', 0, 'x', 0, 'y', 0, 'yaw', 0, 'v', v, 'delta', 0, 'yawRate', 0), '(a) 경로 위, 같은 방향';
     struct('t', 0, 'x', 0, 'y', -1.0, 'yaw', 0, 'v', v, 'delta', 0, 'yawRate', 0), '(b) 1 m 옆, 나란히';
     struct('t', 0, 'x', 0, 'y', -1.0, 'yaw', deg2rad(50), 'v', v, 'delta', 0, 'yawRate', 0), '(c) 1 m 옆, 경로 쪽으로 50도'};
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 1750 560]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact');
for c = 1:size(S, 1)
    st = S{c, 1};
    ax = nexttile(tl); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');
    e = path_error(ref, st.x, st.y, st.yaw);
    [~, dbg] = select_lookahead(st, ref, pp, p, e, v);
    iw = e.idx;
    k = mod(iw - 1 + (round(rmin / ref.ds):round(0.2 / ref.ds):round(1.5 * pp.d_search / ref.ds))', ref.N) + 1;
    d = hypot(ref.x(k) - ref.x(iw), ref.y(k) - ref.y(iw));
    k = k(d >= rmin & d <= pp.d_search);
    start = [st.x, st.y, st.yaw + st.delta];
    plot(ax, ref.x, ref.y, '-', 'Color', [0.95 0.75 0], 'LineWidth', 5);
    t = linspace(0, 2 * pi, 200);
    plot(ax, ref.x(iw) + rmin * cos(t), ref.y(iw) + rmin * sin(t), ':', 'Color', [0.4 0.4 0.4]);
    plot(ax, ref.x(iw) + pp.d_search * cos(t), ref.y(iw) + pp.d_search * sin(t), ':', 'Color', [0.4 0.4 0.4]);
    hc = gobjects(0);
    for j = 1:numel(k)
        P = dubinsPoses(dc, start, [ref.x(k(j)), ref.y(k(j)), ref.psi(k(j))]);
        hc = plot(ax, P(:, 1), P(:, 2), '-', 'Color', [0.65 0.65 0.65]);
        plot(ax, ref.x(k(j)), ref.y(k(j)), '.', 'Color', [0.4 0.4 0.4], 'MarkerSize', 10);
    end
    P = dubinsPoses(dc, start, [dbg.x_d, dbg.y_d, ref.psi(dbg.i_d)]);
    hb = plot(ax, P(:, 1), P(:, 2), 'r--', 'LineWidth', 2.5);
    drawBox(ax, [st.x, st.y] + 0.3 * [cos(st.yaw) sin(st.yaw)], st.yaw, 0.99, 0.56, [0.85 0.85 0.85]);
    plot(ax, st.x, st.y, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    hw = plot(ax, dbg.x_w, dbg.y_w, 'o', 'MarkerFaceColor', [0 0.45 1], 'MarkerEdgeColor', 'w', 'MarkerSize', 10);
    hd = plot(ax, dbg.x_d, dbg.y_d, 'o', 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'w', 'MarkerSize', 11);
    text(ax, ref.x(iw) - 0.1, ref.y(iw) + rmin + 0.12, sprintf('r_{min} = %.2f m', rmin), 'FontSize', 10, 'Color', [0.3 0.3 0.3]);
    text(ax, ref.x(iw) - 0.1, ref.y(iw) + pp.d_search + 0.12, sprintf('d_{search} = %.1f m', pp.d_search), 'FontSize', 10, 'Color', [0.3 0.3 0.3]);
    legend(ax, [hc, hb, hw, hd], {'후보까지 Dubins 경로', '가장 짧은 Dubins 경로', 'p_w', 'p_d'}, 'Location', 'southeast');
    xlim(ax, [-1.6 3.4]); ylim(ax, [-1.9 2.9]);
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    title(ax, sprintf('%s:  p_d = p_w에서 %.2f m', S{c, 2}, hypot(dbg.x_d - dbg.x_w, dbg.y_d - dbg.y_w)));
    fprintf('fig05 %d: p_d = (%.2f, %.2f), dubins len %.2f, candidates %d\n', c, dbg.x_d, dbg.y_d, dbg.dubins_len, numel(k));
end
saveFig(fig, out, '05_step1_dubins.png');
end

function P = dubinsPoses(dc, startPose, goalPose)
[seg, ~] = connect(dc, startPose, goalPose);
P = interpolate(seg{1}, linspace(0, seg{1}.Length, 120));
end

%% 6. 논문 Step 2: 코너 진입 때 p_d를 p_w 접선 쪽으로 옮긴 p_l
function fig06_step2(out, p)
ds = 0.01;
ref = makeRef(stadium(-20, 30, 2, ds), ds);                     % x = 10에서 반경 2 m 왼쪽 코너 시작
st = struct('t', 0, 'x', 8.9, 'y', 0, 'yaw', 0, 'v', 1.0, 'delta', 0, 'yawRate', 0);
v = 1.0;
pp = pp_params('mode', 'paper');
pp0 = pp; pp0.step2_on = false;
e = path_error(ref, st.x, st.y, st.yaw);
[pl, dbg] = select_lookahead(st, ref, pp, p, e, v);
[~, dbg0] = select_lookahead(st, ref, pp0, p, e, v);
pd = [dbg0.x_d, dbg0.y_d];
[d1, s1] = pursuit_steer(st, pl, p);
[d0, s0] = pursuit_steer(st, pd, p);
wd = pd - [dbg.x_w, dbg.y_w];
thdl = atan2(wd(2), wd(1)) + sign(dbg.phi) * pi / 2;
full = pd + norm(wd) * tan(abs(dbg.phi)) * [cos(thdl), sin(thdl)];

[fig, ax] = newFig(1000, 700);
plot(ax, ref.x, ref.y, '-', 'Color', [0.95 0.75 0], 'LineWidth', 5);
plot(ax, dbg.x_w + [0 3.6], dbg.y_w + [0 0], '--', 'Color', [0 0.45 1], 'LineWidth', 1.2);   % p_w 접선
h0 = drawArc(ax, st, s0, pd, [0.5 0.5 0.5], '--');
h1 = drawArc(ax, st, s1, pl, [0.85 0 0.85], '-');
quiver(ax, dbg.x_w, dbg.y_w, wd(1), wd(2), 0, 'Color', [0 0.6 0], 'LineWidth', 1.8, 'MaxHeadSize', 0.15);
plot(ax, [pd(1) full(1)], [pd(2) full(2)], ':', 'Color', 'r', 'LineWidth', 1.5);
quiver(ax, pd(1), pd(2), pl(1) - pd(1), pl(2) - pd(2), 0, 'r', 'LineWidth', 2.2, 'MaxHeadSize', 0.8);
drawBox(ax, [st.x, st.y] + 0.3 * [cos(st.yaw) sin(st.yaw)], st.yaw, 0.99, 0.56, [0.85 0.85 0.85]);
plot(ax, st.x, st.y, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
plot(ax, dbg.x_w, dbg.y_w, 'o', 'MarkerFaceColor', [0 0.45 1], 'MarkerEdgeColor', 'w', 'MarkerSize', 10);
plot(ax, pd(1), pd(2), 'o', 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'w', 'MarkerSize', 11);
plot(ax, pl(1), pl(2), 'o', 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', 'w', 'MarkerSize', 11);

text(ax, dbg.x_w - 0.05, dbg.y_w - 0.17, 'p_w', 'FontSize', 13, 'Color', [0 0.35 0.8]);
text(ax, pd(1) + 0.06, pd(2) + 0.1, 'p_d (Step 1)', 'FontSize', 12, 'Color', [0 0.55 0]);
text(ax, pl(1) + 0.08, pl(2) - 0.07, 'p_l = p_d + \tau |p_{dl}| (식 5)', 'FontSize', 12, 'Color', 'r');
text(ax, 11.9, -0.1, 'p_w의 접선 방향', 'FontSize', 11, 'Color', [0 0.35 0.8], 'HorizontalAlignment', 'right');
text(ax, 9.6, 0.32, 'p_{wd}', 'FontSize', 12, 'Color', [0 0.55 0]);
text(ax, 8.55, 1.95, {sprintf('\\phi = \\psi_w - \\angle p_{wd} = %.1f deg  (식 7)', rad2deg(dbg.phi)), ...
    sprintf('|p_{dl}| = |p_{wd}| tan|\\phi| = %.3f m  (식 6, 점선)', norm(full - pd)), ...
    sprintf('\\alpha = %.2f (차가 경로에 가까움),  \\beta = %.2f (앞쪽 곡률이 큼)', dbg.alpha, dbg.beta), ...
    sprintf('\\tau = (1 - \\alpha)\\beta = %.2f  (식 8) → 점선의 %.0f%%만큼 이동', dbg.tau, 100 * dbg.tau), ...
    sprintf('조향각:  p_d로 %.1f deg  →  p_l로 %.1f deg (덜 일찍 꺾음)', rad2deg(d0), rad2deg(d1))}, ...
    'FontSize', 11, 'VerticalAlignment', 'top', 'EdgeColor', [0.7 0.7 0.7], 'Margin', 6, 'BackgroundColor', 'w');
legend(ax, [h0, h1], {'p_d를 따라갈 때의 원호 (Step 2 끔)', 'p_l을 따라갈 때의 원호 (Step 2 켬)'}, 'Location', 'southeast');
xlim(ax, [8.4 12.4]); ylim(ax, [-0.7 2.0]);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
title(ax, sprintf('직선 → 반경 2 m 왼쪽 코너 진입 (v_{cmd} = %.1f m/s): 선행점을 코너 바깥쪽으로 옮겨 코너 컷팅을 줄임', v));
saveFig(fig, out, '06_step2_shift.png');
fprintf('fig06: phi %.2f deg, tau %.2f, shift %.3f m, delta %.2f -> %.2f deg\n', ...
    rad2deg(dbg.phi), dbg.tau, norm(pl - pd), rad2deg(d0), rad2deg(d1));
end

function h = drawArc(ax, st, s, pt, color, style)
r = abs(s.R);
a0 = atan2(st.y - s.y_c, st.x - s.x_c); a1 = atan2(pt(2) - s.y_c, pt(1) - s.x_c);
if s.R > 0, a = a0 + linspace(0, mod(a1 - a0, 2 * pi), 80);
else,       a = a0 - linspace(0, mod(a0 - a1, 2 * pi), 80); end
h = plot(ax, s.x_c + r * cos(a), s.y_c + r * sin(a), style, 'Color', color, 'LineWidth', 2);
end

%% 7. 속도 명령: 곡선에서 감속 (논문 4.1절)
function fig07_speed(out, p)
ref = load_reference_path();
idx = 1:5:ref.N;
V = zeros(numel(idx), 2); vt = [0.8 1.2];
for c = 1:2
    pp = pp_params('mode', 'fixed', 'v_tar', vt(c));    % v_cmd는 모드와 무관 (계산이 빠른 fixed 사용)
    for j = 1:numel(idx)
        i = idx(j);
        st = struct('t', 0, 'x', ref.x(i), 'y', ref.y(i), 'yaw', ref.psi(i), 'v', vt(c), 'delta', 0, 'yawRate', 0);
        u = pure_pursuit_step(st, ref, p, pp, path_error(ref, st.x, st.y, st.yaw, i));
        V(j, c) = u.v_cmd;
    end
end
vLim = sqrt(9.79 * 0.05 / max(abs(ref.kappa)));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 1400 560]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
ax1 = nexttile(tl);
draw_map(ax1, p);
scatter(ax1, ref.x(idx), ref.y(idx), 14, V(:, 2), 'filled');
colormap(ax1, flipud(turbo)); cb = colorbar(ax1); cb.Label.String = 'v_{cmd} [m/s]  (v_{tar} = 1.2)';
title(ax1, 'v_{tar} = 1.2 m/s일 때 경로 위치별 속도 명령');
ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
yyaxis(ax2, 'right');
plot(ax2, ref.s(idx), abs(ref.kappa(idx)), '-', 'Color', [0.6 0.6 0.6]);
ylabel(ax2, '|\kappa_{ref}| [1/m]'); ax2.YAxis(2).Color = [0.4 0.4 0.4];
yyaxis(ax2, 'left');
plot(ax2, ref.s(idx), V(:, 2), 'r-', 'LineWidth', 1.6);
plot(ax2, ref.s(idx), V(:, 1), 'b-', 'LineWidth', 1.6);
yline(ax2, vLim, 'k:', sprintf('가장 급한 곡선의 허용 속도 %.2f m/s', vLim), 'LabelHorizontalAlignment', 'left');
ylim(ax2, [0.6 1.3]); ylabel(ax2, 'v_{cmd} [m/s]'); ax2.YAxis(1).Color = 'k';
xlabel(ax2, '경로를 따라간 거리 s [m]');
legend(ax2, {'v_{cmd} (v_{tar} = 1.2)', 'v_{cmd} (v_{tar} = 0.8)', '', '|\kappa_{ref}|'}, 'Location', 'southwest');
title(ax2, 'v_{cmd} = min(v_{tar}, \surd(9.79(e_{max}+f_{max}) / |\kappa|)),  f_{max} = 0.05');
saveFig(fig, out, '07_speed_rule.png');
end

%% ---- 공통 보조 함수 ----
function [fig, ax] = newFig(w, h)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 w h]);
ax = axes(fig);
hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');
set(ax, 'FontSize', 11);
end

function saveFig(fig, out, name)
exportgraphics(fig, fullfile(out, name), 'Resolution', 120);
close(fig);
end

function drawBox(ax, c, ang, len, wid, color)
P = [len wid; -len wid; -len -wid; len -wid] / 2;
Rm = [cos(ang) -sin(ang); sin(ang) cos(ang)];
P = (Rm * P')' + c;
fill(ax, P(:, 1), P(:, 2), color, 'EdgeColor', [0.2 0.2 0.2], 'FaceAlpha', 0.5);
end

function angleArc(ax, c, a0, a1, r, color)
t = linspace(a0, a1, 40);
plot(ax, c(1) + r * cos(t), c(2) + r * sin(t), '-', 'Color', color, 'LineWidth', 1.4);
end

function xy = stadium(x0, len, R, ds)
% 반시계 경기장: 아래 직선 (x0 -> x0+len, y=0) → 오른쪽 반원 → 위 직선 → 왼쪽 반원 (test_pure_pursuit.m과 같음)
s = (0:ds:len - ds)';
a1 = (-pi / 2:ds / R:pi / 2 - ds / R)';
a2 = (pi / 2:ds / R:3 * pi / 2 - ds / R)';
xy = [x0 + s, 0 * s;
      x0 + len + R * cos(a1), R + R * sin(a1);
      x0 + len - s, 2 * R + 0 * s;
      x0 + R * cos(a2), R + R * sin(a2)];
end

function ref = makeRef(xy, ds)
% extract_lane_path와 같은 방식으로 s, psi, kappa 계산 (test_pure_pursuit.m과 같음)
N = size(xy, 1); x = xy(:, 1); y = xy(:, 2); h = 10;
ip = mod((0:N - 1)' + h, N) + 1; im = mod((0:N - 1)' - h, N) + 1;
dx = (x(ip) - x(im)) / (2 * h * ds); dy = (y(ip) - y(im)) / (2 * h * ds);
ddx = (x(ip) - 2 * x + x(im)) / (h * ds)^2; ddy = (y(ip) - 2 * y + y(im)) / (h * ds)^2;
seg = hypot(diff([x; x(1)]), diff([y; y(1)]));
ref = struct('x', x, 'y', y, 's', [0; cumsum(seg(1:end - 1))], 'psi', atan2(dy, dx), ...
    'kappa', (dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^1.5, 'N', N, 'ds', ds, ...
    'L', sum(seg), 'closed', true, 'name', "demo");
end
