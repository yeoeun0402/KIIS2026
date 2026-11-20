function st = bicycle_step(st, u, p, dt)
% 액추에이터 제한 + 기구학 자전거 모델 (기준점 = 뒤 차축 중앙, manual_drive.m과 같은 식)
%   st : 상태 구조체 (t, x, y, yaw, v, delta, yawRate)
%   u  : 명령 구조체 (delta_cmd [rad], v_cmd [m/s])
%   p  : vehicle_params()

%% 액추에이터: 조향각은 조향 속도로, 속도는 가속/감속도로 제한
deltaCmd = min(max(u.delta_cmd, -p.deltaMax), p.deltaMax);
st.delta = moveToward(st.delta, deltaCmd, p.steerRate * dt);

vCmd = min(max(u.v_cmd, -p.vRevMax), p.vMax);
if vCmd * st.v >= 0 && abs(vCmd) > abs(st.v)
    rate = p.accel;      % 같은 방향으로 빨라질 때
else
    rate = p.brake;      % 느려지거나 방향을 바꿀 때
end
st.v = moveToward(st.v, vCmd, rate * dt);

%% 기구학 자전거 모델
st.yawRate = st.v / p.L * tan(st.delta);
st.x   = st.x + st.v * cos(st.yaw) * dt;
st.y   = st.y + st.v * sin(st.yaw) * dt;
st.yaw = atan2(sin(st.yaw + st.yawRate * dt), cos(st.yaw + st.yawRate * dt));   % -pi ~ pi
st.t   = st.t + dt;
end

function out = moveToward(cur, target, maxStep)
% cur를 target 쪽으로 최대 maxStep만큼 이동
out = cur + max(-maxStep, min(maxStep, target - cur));
end
