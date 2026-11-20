function H = switch2_fig1(p, ref, opts)
% Figure 1: PP / PID / BC 세 제어기 + 블랜디드 차 (blended_discrete 그림과 같은 배치)
%   왼쪽: 맵 (PP 초록, PID 파랑, BC 노랑 = 레퍼런스 경로 위, 블랜디드 보라), 그 아래 표시 체크박스
%         (opts.withPanel이면 그 아래 파라미터 패널 자리를 비워 둠: switch2_discrete가 채움)
%   오른쪽 위 → 아래: ① 입력 조향각 δ_cmd  ② 출력 조향각 δ  ③ 경로: 궤적 오차 0 - e_lat
%     세 그래프 배경의 옅은 색 띠 = 그 시각에 블랜디드 차가 쓴 제어기 (PP 초록, PID 파랑, BC 노랑)
%     세로축: ①② 같은 범위·5° 간격, ③ 0.02 m 간격 (switch2_yaxis, Figure 2와 같은 규칙). 맵은 x, y축 없음
%     범례는 세 그래프가 같으므로 맨 위에 한 줄만 (그래프 선을 가리지 않게)
%
%   H = switch2_fig1(p, ref, struct('withPanel', true, 'visible', true, 'name', '...'))
%   H.update(D, m, titleLines)   D: switch2_discrete / plot_switch2의 데이터 구조체, m: 그릴 스텝 수
%     D.t (k Ts), D.x0, D.y0 (1x4 출발 위치), D.x, D.y, D.din, D.dout, D.etraj (Nx4: PP PID BC 블랜디드), D.mode

S = switch2_style();
vis = 'on'; if isfield(opts, 'visible') && ~opts.visible, vis = 'off'; end
H.fig = figure('Name', opts.name, 'NumberTitle', 'off', 'Color', 'w', 'Position', [40 40 1500 900], 'Visible', vis);
if opts.withPanel
    axM = axes(H.fig, 'Position', [0.035 0.39 0.55 0.53]); yChk = 0.255;
else
    axM = axes(H.fig, 'Position', [0.035 0.17 0.55 0.75]); yChk = 0.035;
end

%% 맵
draw_map(axM, p);
axis(axM, 'off');                                 % 지도의 x, y축 (눈금·이름) 숨김. 제목·범례는 그대로
% BC 구간 (PP 오차 지도): 경로 위 넓은 옅은 노란 띠 (setZone으로 채움)
hZone = plot(axM, NaN, NaN, '-', 'Color', [S.col(3, :), 0.35], 'LineWidth', 9);
hTr = gobjects(1, 4);
hTr(3) = plot(axM, NaN, NaN, '-', 'Color', S.col(3, :), 'LineWidth', 3.0);   % BC (= 레퍼런스 경로 위) 맨 아래
hTr(1) = plot(axM, NaN, NaN, '-', 'Color', S.col(1, :), 'LineWidth', 1.3);
hTr(2) = plot(axM, NaN, NaN, '-', 'Color', S.col(2, :), 'LineWidth', 1.3);
hTr(4) = plot(axM, NaN, NaN, '.', 'Color', S.col(4, :), 'MarkerSize', 8);
hCar   = plot(axM, NaN, NaN, 'o', 'Color', S.col(4, :), 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'LineWidth', 1.5);
legend(axM, [hTr, hZone], {'PP', 'PID', 'BC (레퍼런스 경로)', '블랜디드 (Unity)', 'BC 구간 (PP 오차 지도)'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal');
hTitle = title(axM, '', 'FontSize', 10);

%% 오른쪽 세 그래프
pos = [0.64 0.665 0.345 0.205; 0.64 0.375 0.345 0.205; 0.64 0.085 0.345 0.205];
ttl = {'① 입력 조향각 \delta_{cmd}  (제어기 출력 = 차량 모델 입력)', ...
       '② 출력 조향각 \delta  (차량 모델을 거친 실제 조향각, 조향 속도 90°/s 제한)', ...
       '③ 경로: 궤적 오차 = 레퍼런스 경로 - 주행 경로 (0 - e_{lat}),  점선 = ±e_{BC}'};
yl = {'[deg]', '[deg]', '[m]'};
axR = gobjects(1, 3); hL = gobjects(3, 3); hBand = gobjects(3, 3);
for a = 1:3
    axR(a) = axes(H.fig, 'Position', pos(a, :)); hold(axR(a), 'on'); grid(axR(a), 'on'); box(axR(a), 'on');
    for j = 1:3                                     % 배경 띠 (블랜디드가 쓴 제어기)
        hBand(a, j) = patch(axR(a), NaN, NaN, S.col(j, :), 'FaceAlpha', S.bandAlpha, 'EdgeColor', 'none', ...
            'HitTest', 'off', 'HandleVisibility', 'off');
    end
    yline(axR(a), 0, 'k-', 'HandleVisibility', 'off', 'Alpha', 0.4);
    for c = [3 1 2]                                 % BC(굵게)를 맨 아래에 그려 PP·PID 선이 가려지지 않게
        lw = 1.2; if c == 3, lw = 2.2; end
        hL(a, c) = plot(axR(a), NaN, NaN, '-', 'Color', S.col(c, :), 'LineWidth', lw, 'Tag', 'data');
    end
    title(axR(a), ttl{a}, 'FontSize', 9.5, 'FontWeight', 'normal');
    ylabel(axR(a), yl{a});
end
xlabel(axR(3), 't = kT_s [s]');
hEbc = plot(axR(3), NaN, NaN, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.8, 'HandleVisibility', 'off');   % ±e_BC
lg = legend(axR(1), hL(1, :), {'PP', 'PID', 'BC'}, 'Orientation', 'horizontal', 'Location', 'northoutside');
lg.Title.String = '배경 색 = 블랜디드 차가 그 시각에 쓴 제어기 (같은 색)';
lg.Title.FontWeight = 'normal';

%% 표시 체크박스
groups = { ...
    'PP (초록)',       [hTr(1), hL(:, 1)'],      S.col(1, :); ...
    'PID (파랑)',      [hTr(2), hL(:, 2)'],      S.col(2, :); ...
    'BC (노랑)',       [hTr(3), hL(:, 3)'],      [0.70 0.55 0.00]; ...
    '블랜디드 (보라)', [hTr(4), hCar],           S.col(4, :); ...
    '구간 배경',       [hBand(:)', hZone, hEbc], [0.35 0.35 0.35]};
makeChecks(H.fig, groups, yChk, axR);

H.axMap = axM; H.axR = axR; H.update = @update; H.setZone = @setZone;

    function setZone(Z)
        % BC 구간 표시: Z.grid (경로점마다 BC 구간인지), Z.eBC (③에 ±e_BC 점선)
        zx = ref.x; zy = ref.y; zx(~Z.grid) = NaN; zy(~Z.grid) = NaN;
        set(hZone, 'XData', [zx; zx(1)], 'YData', [zy; zy(1)]);
        set(hEbc, 'XData', [0 1e4 NaN 0 1e4], 'YData', Z.eBC * [1 1 NaN -1 -1]);
    end

    function update(D, m, titleLines)
        Ts = D.Ts;
        for cc = 1:4
            set(hTr(cc), 'XData', [D.x0(cc); D.x(1:m, cc)], 'YData', [D.y0(cc); D.y(1:m, cc)]);
        end
        set(hCar, 'XData', D.x(m, 4), 'YData', D.y(m, 4));
        tk = D.t(1:m);
        Y = {rad2deg(D.din(1:m, :)), rad2deg(D.dout(1:m, :)), D.etraj(1:m, :)};
        for aa = 1:3
            for cc = 1:3, set(hL(aa, cc), 'XData', tk, 'YData', Y{aa}(:, cc)); end
        end
        % 배경 띠: 같은 모드가 이어지는 구간마다 직사각형 [t_i, t_j + Ts]
        md = D.mode(1:m);
        for jj = 1:3
            on = md == S.modes(jj);
            [vx, vy] = bands(tk, on, Ts);
            for aa = 1:3, set(hBand(aa, jj), 'XData', vx, 'YData', vy); end
        end
        set(axR, 'XLim', [0, max(tk(end) + Ts, Ts)]);
        fitAll(axR);
        if nargin >= 3, hTitle.String = titleLines; end
    end
end

function [vx, vy] = bands(t, on, Ts)
% on이 참인 연속 구간마다 직사각형 (patch용 4 x 구간 수 행렬). 세로는 넉넉히 (축 범위는 fitAll이 정함)
vx = NaN; vy = NaN;
if ~any(on), return; end
d = diff([false; on(:); false]);
i0 = find(d == 1); i1 = find(d == -1) - 1;
x0 = t(i0)'; x1 = t(i1)' + Ts;
vx = [x0; x1; x1; x0];
vy = repmat([-1e4; -1e4; 1e4; 1e4], 1, numel(i0));
end

function fitAll(axR)
% 세로 범위·눈금: ①② 조향각은 같은 범위·같은 간격, ③ 궤적 오차는 Figure 2와 같은 간격 (switch2_yaxis)
%   보이는 데이터 선(Tag 'data')만으로 정함 (배경 띠는 범위에 넣지 않음)
switch2_yaxis(axR(1:2), 'deg');
switch2_yaxis(axR(3), 'm');
end

function makeChecks(fig, groups, y0, axR)
% 맵 아래 표시 체크박스 한 줄: 켜고 끄면 맵과 오른쪽 그래프의 해당 선을 함께 숨김
nG = size(groups, 1);
for gi = 1:nG
    uicontrol(fig, 'Style', 'checkbox', 'String', groups{gi, 1}, 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.045 + 0.11 * (gi - 1), y0 + 0.035, 0.105, 0.03], ...
        'ForegroundColor', groups{gi, 3}, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'Tag', "show_" + gi, ...
        'Callback', @(src, ~) toggle(groups{gi, 2}, src.Value, axR));
end
end

function toggle(h, on, axR)
set(h, 'Visible', matlab.lang.OnOffSwitchState(logical(on)));
fitAll(axR);
end
