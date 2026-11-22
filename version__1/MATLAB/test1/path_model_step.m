function [xk1, used] = path_model_step(xk, uk, p, T)
% 정해진 경로 위를 따라 움직이는 이산 상태공간 모델 한 스텝: x(k+1) = A x(k) + B u(k)
%   xk = [s; v; delta]   s: 경로를 따라 간 거리 [m], v: 속도 [m/s], delta: 조향각 [rad]
%   uk = [a; omega]      a: 가속도 [m/s^2], omega: 조향 속도 [rad/s]
%   p  : vehicle_params() (accel, brake, vMax, steerRate, deltaMax),  T: 주기 [s]
%   A, B: path_model_matrices.m (전진 = 더블 인테그레이터, 조향 = 인테그레이터)
% 입력 한계 (실제 차와 같음):
%   a     : -brake ~ accel 이고, v(k+1)이 0 ~ vMax 안에 있도록 (후진 없음)
%   omega : ±steerRate 이고, delta(k+1)이 ±deltaMax 안에 있도록
% 반환: xk1 = x(k+1),  used = 실제로 쓴 입력 [a; omega] (한계에 걸리면 요청과 다름)
% 차 위치·방향은 상태가 아니라 경로에서 읽음: path_pose(ref, s)

[A, B] = path_model_matrices(T);
v = xk(2); delta = xk(3);
a     = min(max(uk(1), -p.brake), p.accel);
a     = min(max(a, -v / T), (p.vMax - v) / T);
omega = min(max(uk(2), -p.steerRate), p.steerRate);
omega = min(max(omega, (-p.deltaMax - delta) / T), (p.deltaMax - delta) / T);
used  = [a; omega];
xk1   = A * xk(:) + B * used;
end
