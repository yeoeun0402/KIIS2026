function switch3_yaxis(axs, kind)
% 같은 양을 그리는 축들의 세로 범위와 눈금 간격을 똑같이 맞춤 (switch3_fig1, switch3_fig2)
%   kind: 'deg' (조향각) 또는 'm' (궤적 오차)
%   범위: 보이는 데이터 선(Tag 'data')과 0을 모두 담고, 위아래를 눈금 간격의 배수로 맞춤
%   눈금 간격: switch3_style의 S.yStep 후보 중 눈금 칸이 12개 이하가 되는 가장 작은 값
%              (보통 조향각 5°, 궤적 오차 0.02 m → 그림·실행이 달라도 같은 간격)
S = switch3_style(); steps = S.yStep.(kind);
v = [];
for a = 1:numel(axs)
    h = findobj(axs(a), 'Tag', 'data', 'Visible', 'on');
    for i = 1:numel(h), v = [v; h(i).YData(:)]; end %#ok<AGROW>
end
v = v(isfinite(v));
if isempty(v), lo = 0; hi = 0; else, lo = min([v; 0]); hi = max([v; 0]); end
pad = 0.03 * (hi - lo);                           % 선이 테두리에 붙지 않게
for st = steps
    l = st * floor((lo - pad) / st - 1e-9); u = st * ceil((hi + pad) / st + 1e-9);
    if (u - l) / st <= 12, break; end
end
set(axs, 'YLim', [l u], 'YTick', round((l:st:u) / st) * st);
end
