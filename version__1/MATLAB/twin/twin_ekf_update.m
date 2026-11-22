function [st, inn] = twin_ekf_update(st, z, Rz, age, tp, p)
% [twin] 칼만 필터(EKF) 갱신: localization 측정 z = [x y yaw] (시뮬 좌표), 측정 공분산 Rz (3x3, live_loc.py 가 계산)
%   측정 모델 h(X) = [x y yaw] (H = [I3 0]),  지연 age 만큼 거슬러 올라간 추정 자세와 비교
%   K = P Hᵀ (H P Hᵀ + Rz)⁻¹,  X ← X + K ν,  P ← (I − K H) P (I − K H)ᵀ + K Rz Kᵀ  (Joseph 형태)
%   마할라노비스 거리 ν ᵀ S⁻¹ ν > gateChi2 이면 버림 (틀린 localization). Rz 가 큰 방향 (마커가 멀리 한 줄)은 거의 안 믿음
%   inn: 차체 좌표 혁신 (lon, lat, th), d2 (마할라노비스²), used
e = tp.ekf;
inn = struct('lon', NaN, 'lat', NaN, 'th', NaN, 'd2', NaN, 'used', false);
if ~isfield(st, 'P') || isempty(st.P), st.P = diag(e.P0); end
if ~isfield(st, 'vs'), st.vs = 0; end
ve = st.v * (1 + st.vs);
age = min(max(age, 0), e.maxAge);
yawM = st.yaw - st.yawRate * age; yawMid = st.yaw - 0.5 * st.yawRate * age;
hz = [st.x - ve * age * cos(yawMid); st.y - ve * age * sin(yawMid); yawM];
nu = z(:) - hz; nu(3) = atan2(sin(nu(3)), cos(nu(3)));
H = [eye(3), zeros(3, 2)];
Rz = (Rz + Rz') / 2 + diag(e.rFloor);
S = H * st.P * H' + Rz;
d2 = nu' / S * nu;
c = cos(st.yaw); s = sin(st.yaw);
inn.lon = c * nu(1) + s * nu(2); inn.lat = -s * nu(1) + c * nu(2); inn.th = nu(3); inn.d2 = d2;
if ~(d2 <= e.gateChi2), return; end
K = st.P * H' / S;
if abs(ve) < e.vMin, K(4:5, :) = 0; end              % 서 있으면 조향각·속도 배율은 관측 불가
dX = K * nu;
st.x = st.x + dX(1); st.y = st.y + dX(2);
st.yaw = atan2(sin(st.yaw + dX(3)), cos(st.yaw + dX(3)));
st.delta = min(max(st.delta + dX(4), -e.deltaMaxEst), e.deltaMaxEst);
st.vs = min(max(st.vs + dX(5), -e.vsMax), e.vsMax);
IKH = eye(5) - K * H;
st.P = IKH * st.P * IKH' + K * Rz * K';
st.P = (st.P + st.P') / 2;
inn.used = true;
end