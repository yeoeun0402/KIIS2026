function fig = switch2_fig2(p, ref, D, opts)
% Figure 2: 블랜디드 방식 vs 레퍼런스 (블랜디드 보라, 레퍼런스 갈색, 오차 빨강)
%   레퍼런스 = BC 차 (주어진 경로를 그대로 따라가는 차, 같은 시각 k)
%   ① 주행 경로 차이: 맵 (블랜디드 궤적, 레퍼런스 경로, 1 s마다 블랜디드 위치 → 경로 위 가장 가까운 점 빨간 선)
%                     + 트랙 바깥 색 띠 = 블랜디드가 쓴 제어기 구간, 검은 눈금 = 전환 지점 (switch2_mode_ribbon)
%                     + 그 아래 차이 0 - e_lat [m] (빨강)
%   ② 입력 조향각 차이 (tracking error): 블랜디드 δ_cmd, 레퍼런스 δ_cmd, 차이 = 레퍼런스 - 블랜디드
%   ③ 출력 조향각 차이: 블랜디드 δ, 레퍼런스 δ, 차이 = 레퍼런스 - 블랜디드
%   세 그래프 배경의 옅은 색 띠 = 그 시각에 블랜디드 차가 쓴 제어기 (PP 초록, PID 파랑, BC 노랑, Figure 1과 같음)
%   세로축: ②③ 같은 범위·5° 간격, 주행 경로 차이 0.02 m 간격 (switch2_yaxis, Figure 1과 같은 규칙). 맵은 x, y축 없음
%   체크박스 4개 (블랜디드 / 레퍼런스 / 오차 / 구간 배경)가 세 그래프의 해당 선·띠를 함께 켜고 끔
%   범례는 그림 맨 위에 한 줄 (세 그래프 공통, 배경 띠 색 포함)
%
%   D: switch2_fig1과 같은 데이터 구조체 (+ D.xp, D.yp: 블랜디드 위치를 경로에 투영한 점)

S = switch2_style();
vis = 'on'; if isfield(opts, 'visible') && ~opts.visible, vis = 'off'; end
fig = figure('Name', opts.name, 'NumberTitle', 'off', 'Color', 'w', 'Position', [60 60 1500 860], 'Visible', vis);
n = numel(D.t); tk = D.t; iB = 4; iR = 3;
cB = S.col(4, :); cR = S.brown; cE = S.red;

%% ① 맵 + 경로 차이
axM = axes(fig, 'Position', [0.04 0.35 0.53 0.50]);   % 위: 제목 두 줄 + 실행 이름 줄 자리
draw_map(axM, p);
axis(axM, 'off');                                 % 지도의 x, y축 (눈금·이름) 숨김. 제목은 그대로
hRefPath = plot(axM, [ref.x; ref.x(1)], [ref.y; ref.y(1)], '-', 'Color', cR, 'LineWidth', 2.0);
% 차이가 큰 곳 (|0 - e_lat| > 2 cm)을 맵에 빨간 점으로 (cm 단위 차이는 맵에서 안 보이므로 위치만 표시)
big = abs(D.etraj(:, iB)) > 0.02;
hErrSeg = plot(axM, D.x(big, iB), D.y(big, iB), '.', 'Color', cE, 'MarkerSize', 9);
hBl = plot(axM, [D.x0(iB); D.x(:, iB)], [D.y0(iB); D.y(:, iB)], '-', 'Color', cB, 'LineWidth', 1.4);
uistack(hErrSeg, 'top');
% 블랜디드가 쓴 제어기 구간: 트랙 바깥 색 띠 + 전환 지점 눈금 + 안쪽 구간 이름 (switch2_mode_ribbon)
hRib = switch2_mode_ribbon(axM, ref, D.x0(iB), D.y0(iB), D.xp, D.yp, D.mode);
title(axM, {'① 주행 경로 차이: 블랜디드 궤적 vs 레퍼런스 경로 (빨간 점 = 차이가 2 cm 넘는 곳, 크기는 아래 그래프)', ...
    sprintf('트랙 바깥 띠 = 블랜디드가 쓴 제어기 (PP 초록, PID 파랑, BC 노랑), 검은 눈금 = 전환 지점 (%d번)', ...
    sum(D.mode(2:end) ~= D.mode(1:end - 1)))}, 'FontSize', 10, 'FontWeight', 'normal');

axE = axes(fig, 'Position', [0.06 0.09 0.49 0.17]); hold(axE, 'on'); grid(axE, 'on'); box(axE, 'on');
hBand = modeBands(axE, D, S);                     % 배경 띠 (선보다 먼저 그려 아래에 깔림)
yline(axE, 0, 'k-', 'Alpha', 0.4);
hErrT = plot(axE, tk, D.etraj(:, iB), '-', 'Color', cE, 'LineWidth', 1.3, 'Tag', 'data');
ylabel(axE, '[m]'); xlabel(axE, 't = kT_s [s]');
title(axE, sprintf('주행 경로 차이 = 레퍼런스 경로 - 블랜디드 (0 - e_{lat}):  RMS %.4f m, 최대 %.3f m', ...
    rms(D.etraj(:, iB), 'omitnan'), max(abs(D.etraj(:, iB)))), 'FontSize', 9.5, 'FontWeight', 'normal');

%% ②③ 조향각
pos = [0.62 0.56 0.36 0.31; 0.62 0.09 0.36 0.31];
src = {D.din, D.dout};
ttl = {'② 입력 조향각 차이 (tracking error): \delta_{cmd}', '③ 출력 조향각 차이: \delta'};
axS = gobjects(1, 2); hS = gobjects(2, 3);
for a = 1:2
    axS(a) = axes(fig, 'Position', pos(a, :)); hold(axS(a), 'on'); grid(axS(a), 'on'); box(axS(a), 'on');
    hBand = [hBand, modeBands(axS(a), D, S)]; %#ok<AGROW>
    yline(axS(a), 0, 'k-', 'Alpha', 0.4);
    dB = rad2deg(src{a}(:, iB)); dR = rad2deg(src{a}(:, iR)); dE = dR - dB;
    hS(a, 2) = plot(axS(a), tk, dR, '-', 'Color', cR, 'LineWidth', 2.0, 'Tag', 'data');
    hS(a, 1) = plot(axS(a), tk, dB, '-', 'Color', cB, 'LineWidth', 1.2, 'Tag', 'data');
    hS(a, 3) = plot(axS(a), tk, dE, '-', 'Color', cE, 'LineWidth', 1.0, 'Tag', 'data');
    title(axS(a), {ttl{a}, sprintf('차이 = 레퍼런스 - 블랜디드:  RMS %.2f deg, 최대 %.2f deg', ...
        rms(dE, 'omitnan'), max(abs(dE)))}, 'FontSize', 9.5, 'FontWeight', 'normal');
    ylabel(axS(a), '[deg]');
    xlim(axS(a), [0, max(tk(end) + D.Ts, D.Ts)]);
end
xlabel(axS(2), 't = kT_s [s]');
xlim(axE, [0, max(tk(end) + D.Ts, D.Ts)]);

%% 공통 범례 (그림 맨 위) + 체크박스
hKey = gobjects(1, 3);                            % 범례용 배경 띠 색 견본 (그래프에는 안 보임)
for j = 1:3
    hKey(j) = patch(axS(1), NaN, NaN, S.col(j, :), 'FaceAlpha', min(1, 3 * S.bandAlpha), 'EdgeColor', 'none');
end
lg = legend(axS(1), [hS(1, :), hKey], {'블랜디드', '레퍼런스 (주어진 경로 / 그 조향각)', '오차 (레퍼런스 - 블랜디드)', ...
    '배경·맵 띠: PP 구간', '배경·맵 띠: PID 구간', '배경·맵 띠: BC 구간'}, 'Orientation', 'horizontal');
lg.Units = 'normalized'; lg.Position(1:2) = [0.5 - lg.Position(3) / 2, 0.955];
annotation(fig, 'textbox', [0.04 0.915 0.6 0.035], 'String', opts.subtitle, 'EdgeColor', 'none', ...
    'FontSize', 10, 'Interpreter', 'none');

groups = { ...
    '블랜디드 (보라)', [hBl, hS(:, 1)'], cB; ...
    '레퍼런스 (갈색)', [hRefPath, hS(:, 2)'], cR; ...
    '오차 (빨강)',     [hErrSeg, hErrT, hS(:, 3)'], cE; ...
    '구간 배경',       [hBand, hRib], [0.35 0.35 0.35]};
ax = [axE, axS];
for gi = 1:size(groups, 1)
    uicontrol(fig, 'Style', 'checkbox', 'String', groups{gi, 1}, 'Value', 1, 'Units', 'normalized', ...
        'Position', [0.06 + 0.13 * (gi - 1), 0.005, 0.12, 0.03], 'ForegroundColor', groups{gi, 3}, ...
        'FontWeight', 'bold', 'BackgroundColor', 'w', 'Tag', "show2_" + gi, ...
        'Callback', @(s, ~) toggle(groups{gi, 2}, s.Value, ax));
end
fitAll(ax);
end

function fitAll(ax)
% 세로 범위·눈금: ②③ 조향각은 같은 범위·같은 간격, 주행 경로 차이는 Figure 1 ③과 같은 간격 (switch2_yaxis)
switch2_yaxis(ax(2:3), 'deg');
switch2_yaxis(ax(1), 'm');
end

function h = modeBands(ax, D, S)
% 배경 띠: 블랜디드 차가 쓴 제어기(D.mode)가 같은 구간마다 직사각형 [t_i, t_j + Ts] (switch2_fig1과 같은 색·투명도)
%   세로는 넉넉히 그리고, 축 범위는 fitAll(switch2_yaxis)이 데이터 선(Tag 'data')만으로 정함
h = gobjects(1, 3);
md = string(D.mode(:)); tk = D.t(:);
for j = 1:3
    on = md == S.modes(j);
    vx = NaN; vy = NaN;
    if any(on)
        d = diff([false; on; false]);
        i0 = find(d == 1); i1 = find(d == -1) - 1;
        x0 = tk(i0)'; x1 = tk(i1)' + D.Ts;
        vx = [x0; x1; x1; x0];
        vy = repmat([-1e4; -1e4; 1e4; 1e4], 1, numel(i0));
    end
    h(j) = patch(ax, vx, vy, S.col(j, :), 'FaceAlpha', S.bandAlpha, 'EdgeColor', 'none', ...
        'HitTest', 'off', 'HandleVisibility', 'off');
end
end

function toggle(h, on, ax)
set(h, 'Visible', matlab.lang.OnOffSwitchState(logical(on)));
fitAll(ax);
end
