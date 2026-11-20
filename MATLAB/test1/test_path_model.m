function ok = test_path_model()
% 정해진 경로 위 이산 상태공간 모델 테스트. 실행: test_path_model (MATLAB 폴더에서는 addpath('test1') 먼저)
%   1) 행렬 A, B
%   2) 더블 인테그레이터: 일정 가속도에서 s = a(kT)^2/2, v = a kT (정확히)
%   3) 가속도 한계: +accel / -brake, 속도 0 ~ vMax (후진 없음)
%   4) 조향 인테그레이터 한계: ±steerRate, ±deltaMax
%   5) path_pose: 경로 위의 점, 한 바퀴 뒤 같은 점
%   6) 원 경로에서 조향각이 atan(L/R)로 수렴
%   7) 자동 버전 오프라인 한 바퀴 (Unity 없이)
%   8) 키보드 버전은 Unity 없이 실행하면 오류

base = fileparts(fileparts(mfilename('fullpath')));   % MATLAB (공용 함수)
if ~exist('vehicle_params', 'file'), addpath(base); end
p = vehicle_params();
results = strings(0, 1);
pass = true(0, 1);

%% 1) 행렬
[A, B] = path_model_matrices(0.1);
check(isequal(A, [1 0.1 0; 0 1 0; 0 0 1]) && max(abs(B - [0.005 0; 0.1 0; 0 0.1]), [], 'all') < 1e-15, ...
    'matrices: A = [1 T 0; 0 1 0; 0 0 1], B = [T^2/2 0; T 0; 0 T]');

%% 2) 더블 인테그레이터 (T = 0.1, a = 0.5, 10 스텝 = 1 s)
x = [0; 0; 0];
for k = 1:10, x = path_model_step(x, [0.5; 0], p, 0.1); end
check(abs(x(1) - 0.25) < 1e-12 && abs(x(2) - 0.5) < 1e-12, sprintf('double integrator: s %.6f (0.25), v %.6f (0.5)', x(1), x(2)));

%% 3) 가속도 한계
[~, u] = path_model_step([0; 0; 0], [5; 0], p, 0.1);
check(abs(u(1) - p.accel) < 1e-12, sprintf('accel limit: requested 5 -> %.2f m/s^2', u(1)));
[x, u] = path_model_step([0; 1.45; 0], [1; 0], p, 0.1);
check(abs(x(2) - p.vMax) < 1e-12 && abs(u(1) - 0.5) < 1e-12, sprintf('vMax: v %.3f, a used %.2f', x(2), u(1)));
[x, u] = path_model_step([0; 0.1; 0], [-p.brake; 0], p, 0.1);
check(abs(x(2)) < 1e-12 && abs(u(1) + 1) < 1e-12 && abs(x(1) - 0.005) < 1e-12, ...
    sprintf('no reverse: v %.3g, a used %.2f, s %.4f (0.005)', x(2), u(1), x(1)));

%% 4) 조향 인테그레이터
[~, u] = path_model_step([0; 0; 0], [0; 5], p, 0.1);
check(abs(u(2) - p.steerRate) < 1e-12, sprintf('steer rate limit: requested 5 -> %.3f rad/s', u(2)));
x = path_model_step([0; 0; 0.5], [0; p.steerRate], p, 0.1);
check(abs(x(3) - p.deltaMax) < 1e-12, sprintf('steer angle limit: delta %.4f (deltaMax %.4f)', x(3), p.deltaMax));

%% 5) path_pose
ref = load_reference_path();
s = linspace(0, ref.L, 37)';
q = path_pose(ref, s);
eOn = arrayfun(@(i) path_error(ref, q.x(i), q.y(i), q.psi(i)).e_lat, 1:numel(s));
q2 = path_pose(ref, s + ref.L);
check(max(abs(eOn)) < 1e-3 && max(abs(q2.x - q.x)) < 1e-12, ...
    sprintf('path_pose: on path (max |e_lat| %.1e m), s + L = same point', max(abs(eOn))));

%% 6) 원 경로 (R = 3 m)에서 조향각 수렴: delta -> atan(L/R)
R = 3; ds = 0.01;
a = (0:ds / R:2 * pi - ds / R)';
xy = R * [cos(a), sin(a)];
seg = hypot(diff([xy(:, 1); xy(1, 1)]), diff([xy(:, 2); xy(1, 2)]));
circ = struct('x', xy(:, 1), 'y', xy(:, 2), 's', [0; cumsum(seg(1:end - 1))], 'psi', a + pi / 2, ...
    'kappa', ones(size(a)) / R, 'N', numel(a), 'ds', ds, 'L', sum(seg));
x = [0; 0.8; 0];
for k = 1:5
    xs = path_model_step(x, [0; 0], p, 0.1);
    dRef = atan(p.L * path_pose(circ, xs(1)).kappa);
    x = path_model_step(x, [0; (dRef - x(3)) / 0.1], p, 0.1);
end
check(abs(x(3) - atan(p.L / R)) < 1e-9, sprintf('steering on circle: %.2f deg (atan(L/R) = %.2f deg)', rad2deg(x(3)), rad2deg(atan(p.L / R))));

%% 7) 자동 버전 오프라인 한 바퀴
r = discrete_path_drive('input', 'auto', 'useUnity', false, 'realtime', false, ...
    'resultsDir', fullfile(tempdir, 'kiis_test_results'), 'tag', 'test');
Tb = readtable(fullfile(r.folder, 'timeseries.csv'));
check(r.completed && r.distance >= ref.L && abs(r.v_max - 0.8) < 1e-12, ...
    sprintf('auto lap: completed %d, distance %.2f m, v_max %.3f, %d steps', r.completed, r.distance, r.v_max, r.steps));
check(abs(Tb.v_mps(9) - 0.8) < 1e-12 && Tb.v_mps(8) < 0.8, sprintf('auto accel: v reaches 0.8 at k = 8 (v(7) = %.2f)', Tb.v_mps(8)));
k = 1:50:height(Tb);
eOn = arrayfun(@(i) path_error(ref, Tb.x_m(i), Tb.y_m(i), Tb.yaw_rad(i)).e_lat, k);
check(max(abs(eOn)) < 1e-3, sprintf('auto lap: car stays on path (max |e_lat| %.1e m)', max(abs(eOn))));

%% 8) 키보드 버전은 Unity가 필요
msg = "";
try
    discrete_path_drive('input', 'keyboard', 'useUnity', false);
catch err
    msg = string(err.message);
end
check(contains(msg, "useUnity"), 'keyboard without Unity: error asks for useUnity');

%% 결과
for k = 1:numel(results)
    fprintf('%s  %s\n', ternary(pass(k), 'PASS', 'FAIL'), results(k));
end
ok = all(pass);
fprintf('%d / %d passed\n', sum(pass), numel(pass));

    function check(cond, msg)
        results(end + 1, 1) = string(msg);
        pass(end + 1, 1) = cond;
    end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
