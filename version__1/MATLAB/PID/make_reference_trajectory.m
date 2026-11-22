function traj = make_reference_trajectory(ref, p, pid, tEnd, s0)
% 레퍼런스 경로(위치)에 시간과 속도를 붙여 "시각 t에 있어야 할 위치·속도" 표(궤적)를 만듦
%   ref  : load_reference_path() 결과 (폐루프 경로: x, y, s, psi, kappa)
%   p    : vehicle_params() (dt, Rmin)
%   pid  : pid_params() (v_tar, v_mode, e_max, f_max, r_up, a_ref, b_ref)
%   tEnd : 궤적 길이 [s]
%   s0   : 출발점의 경로상 위치 [m] (차 출발 위치를 path_error로 투영한 s)
% 반환 traj (열벡터, 시뮬레이션과 같은 dt 간격, k번째 행 = 시각 (k-1)*dt):
%   t, s (경로를 따라 간 거리, 바퀴가 늘어도 계속 증가), x, y, psi, kappa, v
%   vprof_raw : 경로 점마다의 속도 상한 (감속 다듬기 전, pure pursuit의 v_cmd와 같은 식)
%
% 순서:
%   1) 경로 점마다 속도 상한 (pure_pursuit_step.m의 speedCommand와 같은 식)
%   2) 뒤에서 앞으로 훑어 감속도 b_ref를 넘지 않게 (폐루프라 두 바퀴 훑음)
%   3) 정지에서 a_ref로 출발: v <- min(v_prof(s), v + a_ref dt),  s <- s + v dt
%      (bicycle_step처럼 갱신한 속도로 위치를 옮김 → 차가 속도를 정확히 따르면 e_s = 0)
%   4) 각 시각의 s에서 위치·방향·곡률 보간

N = ref.N;
dt = p.dt;

%% 1. 경로 점마다 속도 상한
if pid.v_mode == "curve"
    g_f    = 9.79 * (pid.e_max + pid.f_max);
    rAhead = min(max(pid.v_tar^2 / g_f, p.Rmin), pid.r_up);       % 경로 위 선행점이 놓이는 거리
    iAhead = mod((0:N - 1)' + round(rAhead / ref.ds), N) + 1;
    kap    = max(abs(ref.kappa), abs(ref.kappa(iAhead)));         % 차 또는 선행점이 곡선 위
    vProf  = min(pid.v_tar, sqrt(g_f ./ kap));
else
    vProf = pid.v_tar * ones(N, 1);
end
traj.vprof_raw = vProf;

%% 2. 감속도 제한: v_i <= sqrt(v_{i+1}^2 + 2 b_ref ds_i)
seg = hypot(ref.x([2:N, 1]) - ref.x, ref.y([2:N, 1]) - ref.y);
for pass = 1:2
    for i = N:-1:1
        j = mod(i, N) + 1;
        vProf(i) = min(vProf(i), sqrt(vProf(j)^2 + 2 * pid.b_ref * seg(i)));
    end
end

%% 3. 시간 적분
sP   = [ref.s; ref.L];                      % 폐루프: 끝에 첫 점을 한 번 더 붙여 보간
vP   = [vProf; vProf(1)];
nT   = floor(tEnd / dt + 1e-9) + 1;
s    = zeros(nT, 1);
v    = zeros(nT, 1);
s(1) = s0;
for k = 1:nT - 1
    v(k + 1) = min(interp1(sP, vP, mod(s(k), ref.L)), v(k) + pid.a_ref * dt);
    s(k + 1) = s(k) + v(k + 1) * dt;
end

%% 4. 위치·방향·곡률 보간
sm = mod(s, ref.L);
psiP = unwrap([ref.psi; ref.psi(1)]);
traj.t     = (0:nT - 1)' * dt;
traj.s     = s;
traj.x     = interp1(sP, [ref.x; ref.x(1)], sm);
traj.y     = interp1(sP, [ref.y; ref.y(1)], sm);
psi        = interp1(sP, psiP, sm);
traj.psi   = atan2(sin(psi), cos(psi));
traj.kappa = interp1(sP, [ref.kappa; ref.kappa(1)], sm);
traj.v     = v;
traj.vprof = vProf;
end
