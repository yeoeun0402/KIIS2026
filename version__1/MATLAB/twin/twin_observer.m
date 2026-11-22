function [st, inn] = twin_observer(st, meas, age, dtm, tp, p)
% [twin] 루엔버그 옵저버 보정 한 번: 실차 localization 측정으로 시뮬 상태(st)를 실차 쪽으로 당김
%   meas : [x y yaw] 시뮬 좌표 (twin_frames('track2sim', …) 로 바꾼 것)
%   age  : 측정이 찍힌 뒤 지난 시간 [s] (카메라 프레임 → 지금). 그만큼 거슬러 올라간 추정 자세와 비교
%   dtm  : 지난 측정 뒤 지난 시간 [s] (연속 시간 게인 × dtm = 이번 보정량)
%   inn  : 차체 좌표 혁신 (lon, lat, th) 과 사용 여부
%
% 오차 모델 (직진 근처 선형화, e = 실차 − 추정, 차체 좌표):
%   ė_lat = v e_θ,  ė_θ = (v/L) e_δ,  ė_δ = 0   |   ė_lon = v e_s,  ė_s = 0
% 옵저버:  x̂' = f(x̂, u) + K(v) (y − ŷ)   (f = twin_model_step, 측정 y = [위치, 방향])
%   위치 옆      : + kLat · e_lat          kLat = 1 / tauPos
%   위치 앞뒤    : + kLon · e_lon          kLon = 2 ζl ωl
%   방향         : + kTh  · e_θ            kTh  = 2 ζh ωh
%   조향각       : + kDel · e_θ            kDel = ωh² L / v      → e_θ, e_δ 오차의 극: s² + 2ζh ωh s + ωh²
%   속도 배율    : + kS   · e_lon          kS   = ωl² / v        → e_lon, e_s 오차의 극: s² + 2ζl ωl s + ωl²
%   (옆 위치는 kLat 로 따로 수렴: 극 −1/tauPos)
o = tp.obs;
inn = struct('lon', NaN, 'lat', NaN, 'th', NaN, 'used', false);
if ~isfield(st, 'vs'), st.vs = 0; end
ve = st.v * (1 + st.vs);
age = min(max(age, 0), o.maxAge);
% 측정 시각의 추정 자세 (지연 보정: 지금 자세에서 age 만큼 거꾸로)
yawM = st.yaw - st.yawRate * age;
yawMid = st.yaw - 0.5 * st.yawRate * age;
xm = st.x - ve * age * cos(yawMid);
ym = st.y - ve * age * sin(yawMid);
dx = meas(1) - xm;  dy = meas(2) - ym;
eth = atan2(sin(meas(3) - yawM), cos(meas(3) - yawM));
if hypot(dx, dy) > o.gate || abs(eth) > deg2rad(o.gateDeg)
    return;                                           % 튀는 측정은 버림
end
c = cos(st.yaw); s = sin(st.yaw);
elon = c * dx + s * dy;  elat = -s * dx + c * dy;
inn = struct('lon', elon, 'lat', elat, 'th', eth, 'used', true);
dt = min(max(dtm, 0.01), 0.3);
kLat = 1 / o.tauPos;
kLon = 2 * o.zetaLon * o.wnLon;
kTh  = 2 * o.zetaHead * o.wnHead;
st.x = st.x + (kLon * elon * c - kLat * elat * s) * dt;
st.y = st.y + (kLon * elon * s + kLat * elat * c) * dt;
st.yaw = atan2(sin(st.yaw + kTh * eth * dt), cos(st.yaw + kTh * eth * dt));
if abs(ve) >= o.vMin
    vv = sign(ve) * max(abs(ve), 0.15);              % 느릴 때 게인이 너무 커지지 않게
    kDel = o.wnHead^2 * p.L / vv;
    kS   = o.wnLon^2 / max(abs(st.v), 0.15);          % 속도 배율: e_lon 변화율 = v · e_s (후진도 앞뒤 축 기준으로 같음)
    st.delta = min(max(st.delta + kDel * eth * dt, -o.deltaMaxObs), o.deltaMaxObs);
    st.vs = min(max(st.vs + sign(st.v) * kS * elon * dt, -o.vsMax), o.vsMax);
end
end
