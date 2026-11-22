function st = twin_model_step(st, tp, p, dt)
% [twin] 실차 명령(PWM, 조향 펄스)을 받는 차량 모델 한 스텝 (기구학 자전거 모델, 기준점 = 뒷축 중앙)
%   st: x, y, yaw [시뮬 좌표], v [m/s], delta [rad], pwmCmd (보낸 명령), pwm (바퀴에 반영된 명령),
%       pend (N x 2: [반영 시각, PWM], twin_key가 넣음), steerDir (+1 왼쪽 / -1 오른쪽 / 0), steerLeft [s] (모터 남은 시간)
%   상태 방정식 (dt 동안 입력 일정):
%     pwm    = delay 초 전에 보낸 pwmCmd                  (명령 → 바퀴 반응 지연)
%     v_ref  = kv · sign(pwm) · max(|pwm| - pwmDead, 0)
%     v'     = (v_ref - v) / tauV                         속도: 1차 지연
%     delta' = steerDir · steerRate  (모터가 도는 동안만)  조향: 인테그레이터 (test1과 같은 구조)
%     x' = v cos(yaw),  y' = v sin(yaw),  yaw' = v tan(delta + delta0) / L
%   analysis/fit_twin_params.py 의 replay()와 같은 식 (그 스크립트로 파라미터를 맞춤)
while ~isempty(st.pend) && st.pend(1, 1) <= st.t + 1e-9
    st.pwm = st.pend(1, 2);
    st.pend(1, :) = [];
end
vRef = tp.kv * sign(st.pwm) * max(abs(st.pwm) - tp.pwmDead, 0);
st.v = vRef + (st.v - vRef) * exp(-dt / tp.tauV);

if st.steerLeft > 0
    on = min(dt, st.steerLeft);
    st.delta = min(max(st.delta + st.steerDir * tp.steerRate * on, -tp.deltaMax), tp.deltaMax);
    st.steerLeft = st.steerLeft - on;
    if st.steerLeft <= 0, st.steerDir = 0; st.steerLeft = 0; end
end

ve = st.v;                                        % 실제 속도 = 모델 속도 × (1 + 속도 배율) — 배율은 옵저버가 추정 (없으면 0)
if isfield(st, 'vs'), ve = st.v * (1 + st.vs); end
st.yawRate = ve * tan(st.delta + tp.delta0) / p.L;
yawMid = st.yaw + 0.5 * st.yawRate * dt;          % 중점 방향으로 위치 적분 (dt가 작아 충분)
st.x = st.x + ve * cos(yawMid) * dt;
st.y = st.y + ve * sin(yawMid) * dt;
st.yaw = atan2(sin(st.yaw + st.yawRate * dt), cos(st.yaw + st.yawRate * dt));
st.t = st.t + dt;
end
