function [dCmd, sNext] = bc_steer(sNow, v, aReq, ref, p, T)
% BC 조향 한 스텝: 레퍼런스 경로를 그대로 따라가는 조향각 (discrete_path_drive.m의 doStep과 같은 규칙)
%   이 프로젝트에서 BC는 behavior clone이 아니라 "레퍼런스 경로 그대로 따라가는 조향각"을 뜻함
%
%   s(k+1) = 이번 스텝 가속도 aReq로 간 다음 위치 (path_model_step, 조향 입력 0으로 먼저 계산)
%   δ_BC   = atan( L κ(s(k+1)) )      다음 위치의 경로 곡률에 맞는 조향각
%
%   sNow : 지금 경로 위 위치 [m]
%            BC 차 (경로 위를 그대로 감): 그 차의 s
%            블랜디드 차 (경로에서 벗어날 수 있음): 경로에 투영한 점의 s (path_error의 e.s)
%   v, aReq : 지금 속도 [m/s], 이번 스텝 가속도 요청 [m/s^2]
%   반환 sNext = s(k+1)
%   되먹임이 없음: 차가 경로에서 벗어나 있어도 이 값은 벗어난 만큼을 되돌리지 않음

xs = path_model_step([sNow; v; 0], [aReq; 0], p, T);
sNext = xs(1);
dCmd = atan(p.L * path_pose(ref, sNext).kappa);
end
