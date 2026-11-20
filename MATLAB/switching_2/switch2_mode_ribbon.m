function h = switch2_mode_ribbon(ax, ref, x0, y0, xp, yp, mode)
% 맵에 블랜디드가 쓴 제어기 구간을 표시 (switch2_fig2의 맵)
%   트랙 바깥 색 띠 = 그 위치에서 블랜디드가 쓴 제어기 (PP 초록, PID 파랑, BC 노랑, 그래프 배경 띠와 같은 색)
%   검은 눈금      = 제어기가 바뀐 지점
%   트랙 안쪽 글자 = 그 구간의 제어기 이름 (0.8 m 이상인 구간만)
%   위치: 스텝 k의 제어기를 정한 곳 = k = 1은 출발점 (x0, y0), 그 뒤는 직전 스텝 결과 위치를 경로에 투영한 점 (xp, yp)
%   반환 h: 그린 객체 전부 (체크박스로 함께 켜고 끔)
S = switch2_style();
md = string(mode(:)); n = numel(md);
h = gobjects(0);
if n == 0, return; end

% 판단 위치의 경로 위치 s
e = path_error(ref, x0, y0, 0); sDec = nan(n + 1, 1); sDec(1) = e.s; hint = e.idx;
for k = 1:n
    if ~isfinite(xp(k)), sDec(k + 1) = sDec(k); continue; end
    e = path_error(ref, xp(k), yp(k), 0, hint); hint = e.idx; sDec(k + 1) = e.s;
end
i0 = [1; find(md(2:end) ~= md(1:end - 1)) + 1]; i1 = [i0(2:end) - 1; n];
off = 0.55;                                       % 띠: 경로(바깥 차선 중심)에서 진행 방향 오른쪽 = 트랙 바깥으로 [m]
for j = 1:numel(i0)                               % 색 띠
    c = S.col(S.modes == md(i0(j)), :); if isempty(c), continue; end
    idx = arcIdx(ref, sDec(i0(j)), sDec(i1(j) + 1));
    [rx, ry] = offsetPts(ref, idx, off);
    h(end + 1) = plot(ax, rx, ry, '-', 'Color', c, 'LineWidth', 6, 'HandleVisibility', 'off'); %#ok<AGROW>
end
for j = 2:numel(i0)                               % 전환 지점 눈금
    ii = nearestIdx(ref, sDec(i0(j)));
    [a1, b1] = offsetPts(ref, ii, off - 0.2); [a2, b2] = offsetPts(ref, ii, off + 0.2);
    h(end + 1) = plot(ax, [a1 a2], [b1 b2], 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off'); %#ok<AGROW>
end
for j = 1:numel(i0)                               % 구간 이름 (트랙 안쪽)
    c = S.col(S.modes == md(i0(j)), :); if isempty(c), continue; end
    sa = sDec(i0(j)); len = mod(sDec(i1(j) + 1) - sa, ref.L);
    if len < 0.8, continue; end
    [lx, ly] = offsetPts(ref, nearestIdx(ref, sa + len / 2), -1.0);
    h(end + 1) = text(ax, lx, ly, md(i0(j)), 'Color', 0.8 * c, 'FontSize', 8.5, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'w', 'EdgeColor', c, 'Margin', 1.5); %#ok<AGROW>
end
end

function idx = arcIdx(ref, sa, sb)
% 경로 위치 sa → sb (앞으로, 한 바퀴 넘어가면 이어서)의 경로점 번호
sa = mod(sa, ref.L); sb = mod(sb, ref.L);
if sb >= sa, idx = find(ref.s >= sa & ref.s <= sb); else, idx = [find(ref.s >= sa); find(ref.s <= sb)]; end
if isempty(idx), idx = nearestIdx(ref, sa); end
end

function i = nearestIdx(ref, s)
[~, i] = min(abs(mod(ref.s - s + ref.L / 2, ref.L) - ref.L / 2));
end

function [x, y] = offsetPts(ref, idx, off)
% 경로점을 진행 방향 오른쪽으로 off [m] 옮긴 점 (음수면 왼쪽 = 트랙 안쪽)
x = ref.x(idx) + off * sin(ref.psi(idx)); y = ref.y(idx) - off * cos(ref.psi(idx));
end
