function Z = pp_error_map(p, ref, v, Lf, Ts, eBC, lead)
% PP 오차 지도: PP만으로 한 바퀴 돌았을 때 경로 위치 s마다의 궤적 오차 → BC 구간
%   블랜디드 차가 BC를 쓰는 곳 = "PP로 달려도 오차가 커지는 구간"
%   ① PP 차 하나를 같은 출발점·같은 속도 v·같은 Lf로 한 바퀴 (+ 0.2바퀴) 미리 계산 (Unity 없이, 0.3 s 정도)
%   ② |0 - e_lat| >= eBC 인 경로 위치 = PP 오차가 큰 곳
%   ③ BC 구간 = 그곳과, 그곳보다 lead [m] 앞(차가 다가오는 쪽)까지
%      (오차가 커진 뒤에 BC로 바꾸면 BC는 되먹임이 없어 이미 생긴 오차를 되돌리지 못하므로,
%       오차가 생기기 전에 미리 BC로 바꿈. 2026-11-16 비교: 오차가 커진 뒤 바꾸면 RMS 0.1 m 이상으로 나빠짐)
%
% 반환 Z: s, e (PP 차의 경로 위치와 0 - e_lat), grid (ref 점마다 BC 구간이면 true),
%         x, y (BC 구간 경로점, 그림용), v, Lf, eBC, lead

e0 = path_error(ref, p.startUnity(1), p.startUnity(2), deg2rad(90 - p.startUnity(3)));
q0 = path_pose(ref, e0.s);
car = struct('t', 0, 'x', q0.x, 'y', q0.y, 'yaw', q0.psi, 's', 0, 'v', 0, 'delta', atan(p.L * q0.kappa), 'yawRate', 0);
mem = []; hint = NaN;
nMax = ceil(1.2 * ref.L / max(v, 0.05) / Ts) + 50;
s = nan(nMax, 1); e = s;
for k = 1:nMax
    [d, mem] = pp_steer(car, ref, p, struct('Lf', Lf), mem);
    car = bicycle_discrete_step(car, [(v - car.v) / Ts; (d - car.delta) / Ts], p, Ts);
    ee = path_error(ref, car.x, car.y, car.yaw, hint); hint = ee.idx;
    s(k) = ee.s; e(k) = -ee.e_lat;
    if car.s >= 1.2 * ref.L, break; end
end
s = s(1:k); e = e(1:k);

sZ = s(abs(e) >= eBC);
grid = false(ref.N, 1);
win = lead + v * Ts;                          % PP 기록은 v Ts 간격 → 그 간격만큼 더해야 lead = 0일 때도 구간이 이어짐
for i = 1:numel(sZ)
    ahead = mod(sZ(i) - ref.s, ref.L);        % 경로점에서 오차가 큰 곳까지 앞쪽 거리
    grid = grid | ahead <= win;
end
Z = struct('s', s, 'e', e, 'grid', grid, 'x', ref.x(grid), 'y', ref.y(grid), ...
    'v', v, 'Lf', Lf, 'eBC', eBC, 'lead', lead);
end
