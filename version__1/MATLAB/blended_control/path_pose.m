function q = path_pose(ref, s)
% 경로를 따라 간 거리 s [m]에서의 위치·방향·곡률 (폐루프라 s가 한 바퀴를 넘어도 됨, 선형 보간)
%   ref : load_reference_path() 결과
%   반환 q.x, q.y [m], q.psi [rad, -pi~pi], q.kappa [1/m]  (s가 벡터면 열벡터)
sP = [ref.s; ref.L];                         % 끝에 첫 점을 한 번 더 붙여 한 바퀴를 닫음
sm = mod(s(:), ref.L);
q.x     = interp1(sP, [ref.x; ref.x(1)], sm);
q.y     = interp1(sP, [ref.y; ref.y(1)], sm);
psi     = interp1(sP, unwrap([ref.psi; ref.psi(1)]), sm);
q.psi   = atan2(sin(psi), cos(psi));
q.kappa = interp1(sP, [ref.kappa; ref.kappa(1)], sm);
end
