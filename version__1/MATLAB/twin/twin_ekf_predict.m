function st = twin_ekf_predict(st, tp, p, dt, nSteerKeys)
% [twin] 칼만 필터(EKF) 예측: 공분산 P 를 dt 만큼 전파 (평균은 twin_model_step 이 이미 적분함)
%   상태 X = [x y yaw delta vs]  (시뮬 좌표, delta = 조향각, vs = 속도 배율: v_실제 = v_모델 (1 + vs))
%   모델 (dt 동안): x' = ve cos(yaw), y' = ve sin(yaw), yaw' = ve tan(delta + delta0)/L, delta' = (조향 모터), vs' = 0
%   F = ∂f/∂X (오일러 선형화),  P ← F P Fᵀ + Q dt (+ 조향 키 하나마다 조향각 분산 qKey)
%   nSteerKeys: 이번 dt 동안 적용된 A/D 키 수 (키 한 번에 꺾이는 양이 실차와 다를 수 있음 → 불확실성 추가)
e = tp.ekf;
if ~isfield(st, 'P') || isempty(st.P), st.P = diag(e.P0); end
if nargin < 5, nSteerKeys = 0; end
if ~isfield(st, 'vs'), st.vs = 0; end
ve = st.v * (1 + st.vs); c = cos(st.yaw); s = sin(st.yaw); dd = st.delta + tp.delta0;
F = eye(5);
F(1, 3) = -ve * s * dt;   F(1, 5) = st.v * c * dt;
F(2, 3) =  ve * c * dt;   F(2, 5) = st.v * s * dt;
F(3, 4) = ve / (p.L * cos(dd)^2) * dt;  F(3, 5) = st.v * tan(dd) / p.L * dt;
q = [e.qPos, e.qPos, e.qYaw, e.qDelta, e.qVs];
q(1:2) = q(1:2) + e.qPosPerSpeed * abs(ve);       % 움직일수록 미끄러짐·모델 오차
Q = diag(q) * dt;
Q(4, 4) = Q(4, 4) + e.qKey * nSteerKeys;
st.P = F * st.P * F' + Q;
st.P = (st.P + st.P') / 2;
end
