function [mode, lam, info] = decide_lambda(kap, prevMode, cfg, inZone)
% 카메라 feature (차선 3개의 곡률) + PP 오차 지도 → 블랜디드 가중치 λ = [λ0 λ1 λ2]
%   δ_blend = λ0 δ_PP + λ1 δ_PID + λ2 δ_BC
%   PP  : λ = [1 0 0]     PID : λ = [0 1 0]     BC : λ = [0 0 1]
%
% 규칙 (2026-11-16 사용자 지정. 기준값은 switch3_discrete 그림의 패널로 실행 중에 바꿀 수 있음):
%   차선 상태: 보이지 않음 (κ = NaN) / 곡선 (|κ| > kappaTh) / 직선
%   ① 보이는 차선이 모두 직선          → cfg.straightMode (기본 PID: 직선은 PID)
%   ② 그 밖 (곡선, 섞임, 안 보임):
%        차가 BC 구간 안 (inZone)      → BC  (PP로 달려도 오차가 커지는 구간, pp_error_map.m)
%        아니면                        → PP  (기본 주행)
%   cfg.force가 "PP", "PID", "BC"면 위 규칙 대신 그 제어기로 고정 (비교용). "auto"면 규칙대로
%
% 입력: kap      1x3 [outer dashed inner] 곡률 [1/m], 안 보이면 NaN (lane_server의 LANE 줄 값)
%       prevMode 직전 모드 (지금 규칙에서는 쓰지 않음, 기록용)
%       cfg      kappaTh, straightMode, force
%       inZone   차가 지금 BC 구간 안이면 true
% 반환: mode, lam, info (states: 차선별 "none"/"curve"/"straight", rule: 어느 규칙으로 정했는지)

kap = kap(:)';
vis = ~isnan(kap);
curved = vis & abs(kap) > cfg.kappaTh;
states = repmat("none", 1, numel(kap));
states(vis) = "straight";
states(curved) = "curve";

if cfg.force ~= "auto"
    mode = string(cfg.force); rule = "고정";
elseif any(vis) && ~any(curved)
    mode = string(cfg.straightMode); rule = "모두 직선";
elseif inZone
    mode = "BC"; rule = "PP 오차 구간";
else
    mode = "PP"; rule = "기본 (PP)";
end
lam = double(mode == ["PP", "PID", "BC"]);
info = struct('states', states, 'rule', rule, 'nVisible', sum(vis), 'prevMode', string(prevMode));
end
