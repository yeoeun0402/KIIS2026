function [st, used] = bicycle_discrete_step(st, uk, p, T)
% 기구학 자전거 모델 + 이산 액추에이터 한 스텝 (조향 = 인테그레이터, 전진 = 더블 인테그레이터)
%   st : 상태 구조체 (t, x, y, yaw, s, v, delta, yawRate)
%          x, y  : 뒤 차축 중앙 위치 [m],  yaw : 차 방향 [rad, -pi~pi]
%          s     : 차가 실제로 달린 거리 (주행 거리계) [m]
%          v     : 속도 [m/s],  delta : 조향각 [rad]
%   uk : [a; omega]  가속도 [m/s^2], 조향 속도 [rad/s]
%   p  : vehicle_params() (L, accel, brake, vMax, steerRate, deltaMax),  T : 주기 [s]
% 계산 순서
%   ① 액추에이터: path_model_step.m 그대로 (입력 한계 포함)
%        [s; v; delta](k+1) = A [s; v; delta](k) + B [a; omega](k)
%        s(k+1) = s(k) + T v(k) + T^2/2 a(k),  v(k+1) = v(k) + T a(k)     전진: 더블 인테그레이터
%        delta(k+1) = delta(k) + T omega(k)                                조향: 인테그레이터
%   ② 기구학 자전거 모델: bicycle_step.m과 같은 식. v·dt 자리에 ①의 이동 거리 ds = s(k+1) - s(k)
%        x(k+1)   = x(k) + ds cos(yaw(k))
%        y(k+1)   = y(k) + ds sin(yaw(k))
%        yaw(k+1) = yaw(k) + ds tan(delta(k+1)) / L
%   a = 0이면 bicycle_step(st, struct('delta_cmd', delta + T omega, 'v_cmd', v), p, T)와 결과가 같음
%   (test_blended.m에서 확인)
% 반환: st = 다음 상태,  used = 실제로 쓴 입력 [a; omega] (한계에 걸리면 요청과 다름)

%% ① 액추에이터 (더블 인테그레이터 + 인테그레이터)
[xk1, used] = path_model_step([st.s; st.v; st.delta], uk, p, T);
ds = xk1(1) - st.s;                   % 이번 스텝에 달린 거리 = T v + T^2/2 a
st.s = xk1(1); st.v = xk1(2); st.delta = xk1(3);

%% ② 기구학 자전거 모델 (뒤 차축 기준)
st.yawRate = st.v * tan(st.delta) / p.L;
st.x   = st.x + ds * cos(st.yaw);
st.y   = st.y + ds * sin(st.yaw);
yaw    = st.yaw + ds * tan(st.delta) / p.L;
st.yaw = atan2(sin(yaw), cos(yaw));   % -pi ~ pi
st.t   = st.t + T;
end
