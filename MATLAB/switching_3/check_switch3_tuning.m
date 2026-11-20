function check_switch3_tuning()
% switch3_discrete 실행 중 패널 조작이 다음 스텝부터 실제로 적용되는지 확인 (Unity 없이, 경로 곡률 판단)
%   k = 100 끝: Kp 입력칸 1.0, v 슬라이더 1.2, 제어기 선택 "PP 고정"
%   k = 200 끝: 블랜디드 LPF ℓ 입력칸 1 (필터 끔), w_current 입력칸 0.8
%   k = 300 끝: 제어기 선택 "자동 (규칙)", e_BC 입력칸 0.03, 모두 직선 → PP
%   확인: 스텝 101부터 p_Kp = 1, p_v = 1.2, mode = PP / 스텝 201부터 p_ell = 1, p_wcur = 0.8,
%         블랜디드 입력 조향각 = LPF 전 δ / 스텝 301부터 force = auto, p_eBC = 0.03, 직선 = PP
%         PID 차의 입력 조향각이 Kp를 바꾼 직후부터 달라짐 (같은 조건에서 Kp를 안 바꾼 실행과 비교)
c = {'decision', 'ref', 'useUnity', false, 'realtime', false, 'save', false, 'showFigure', true};
r0 = switch3_discrete(c{:});
r1 = switch3_discrete(c{:}, 'stepHook', @hook);
T = r1.timeseries; T0 = r0.timeseries;
ok = all(T.p_Kp(1:100) == 3) && all(T.p_Kp(101:end) == 1) && all(T.p_v(101:end) == 1.2) ...
    && all(T.mode(101:300) == "PP") && all(T.p_force(301:end) == "auto") && all(T.p_eBC(301:end) == 0.03) && all(T.p_straightMode(301:end) == "PP") ...
    && all(T.p_ell(1:200) == 3) && all(T.p_ell(201:end) == 1) && all(T.p_wcur(201:end) == 0.8) ...
    && isequal(T.bl_delta_in_rad(201:end), T.bl_delta_blend_rad(201:end)) ...
    && max(abs(T.bl_delta_in_rad(1:200) - T.bl_delta_blend_rad(1:200))) > 1e-3;
same = max(abs(T.pid_delta_in_rad(1:100) - T0.pid_delta_in_rad(1:100)));
diff1 = max(abs(T.pid_delta_in_rad(101:110) - T0.pid_delta_in_rad(101:110)));
fprintf('패널 값 적용: %d  |  PID 입력 조향각 차이: k<=99 %.2e rad, k=100~109 %.3f rad  |  v 끝 %.2f m/s\n', ...
    ok, same, diff1, T.v_mps(end));
assert(ok && same == 0 && diff1 > 1e-3, 'check_switch3_tuning: 실패');
fprintf('check_switch3_tuning: 통과\n');
close all;
end

function hook(fig, k)
if k == 100
    setEdit(fig, 'Kp', '1.0');
    hs = findobj(fig, 'Tag', 'tune_v'); hs.Value = 1.2; hs.Callback(hs, []);
    hp = findobj(fig, 'Tag', 'tune_force'); hp.Value = 2; hp.Callback(hp, []);
elseif k == 200
    setEdit(fig, 'ell', '1');
    setEdit(fig, 'wcur', '0.8');
elseif k == 300
    hp = findobj(fig, 'Tag', 'tune_force'); hp.Value = 1; hp.Callback(hp, []);
    setEdit(fig, 'eBC', '0.03');
    hp = findobj(fig, 'Tag', 'tune_straightMode'); hp.Value = 2; hp.Callback(hp, []);
end
end

function setEdit(fig, nm, val)
he = findobj(fig, 'Tag', "tuneEdit_" + nm); he.String = val; he.Callback(he, []);
end
