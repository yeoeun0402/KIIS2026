function test_twin()
% [twin] 테스트 (Unity·실차 없이): 좌표 변환, 기준선, 모델, 키 규칙, 시뮬만 주행
%   >> setup_paths; test_twin
tp = twin_params('delta0', 0); p = vehicle_params();   % 직진·조향 검사는 중립 오프셋 없이 (오프셋은 아래 따로)
nPass = 0; nAll = 0;

% 1. 좌표 변환: 180° 회전이라 두 번 하면 제자리, 출발 자세
q = twin_frames('sim2track', twin_frames('track2sim', tp.startTrack));
check('좌표 변환 왕복', norm(q - tp.startTrack) < 1e-12);
s0 = twin_frames('track2sim', tp.startTrack);
check('출발 자세 시뮬 좌표 (6.735, -1.159, +90°)', norm(s0 - [6.735, -1.159, pi/2]) < 1e-9);

% 2. 기준선: 39.78 m 닫힌 경로, 트랙↔시뮬 같은 모양, 출발 자세가 기준선 위
rt = twin_reference('track'); rs = twin_reference('sim');
check('기준선 길이 39.78 m', abs(rt.L - 39.78) < 0.05);
check('시뮬 기준선 = 트랙 기준선 180° 회전', max(abs(rs.x - (8 - rt.x))) < 1e-12);
e = path_error(rs, s0(1), s0(2), s0(3));
check('출발 자세 횡오차 < 2 cm, 방향 오차 < 2° (기준선은 5 cm 간격 도면 판독값이라 선분 방향이 ±1° 정도 흔들림)', abs(e.e_lat) < 0.02 && abs(rad2deg(e.e_head)) < 2);

% 3. 모델: PWM 40 직진 → 정상 속도 kv*40, 위치는 +y(시뮬, 진행 방향)로만
st0 = struct('t', 0, 'x', s0(1), 'y', s0(2), 'yaw', s0(3), 'v', 0, 'delta', 0, 'yawRate', 0, ...
    'pwmCmd', 0, 'pwm', 0, 'pend', zeros(0, 2), 'steerDir', 0, 'steerLeft', 0);
st = st0;
st = twin_key(twin_key(twin_key(twin_key(st, "w", tp), "w", tp), "w", tp), "w", tp);
check('W 4번 = PWM 명령 40, 바퀴에는 아직 0 (delay)', st.pwmCmd == 40 && st.pwm == 0);
for i = 1:round(tp.delay / 0.01) - 1, st = twin_model_step(st, tp, p, 0.01); end
check('delay 전에는 안 움직임', st.v == 0);
for i = 1:500, st = twin_model_step(st, tp, p, 0.01); end
check('5 s 뒤 속도 = kv*(40 - pwmDead) (1차 지연 수렴)', abs(st.v - tp.kv * (40 - tp.pwmDead)) < 1e-3);
check('직진: 옆으로 안 감 (5 s 동안 0.5 m 이상 앞으로)', abs(st.x - s0(1)) < 1e-9 && st.y > s0(2) + 0.5);
% 4. 조향 펄스: A 한 번 = steerRate*0.12 만큼, 손 떼면 유지
st = twin_key(st, "a", tp);
for i = 1:30, st = twin_model_step(st, tp, p, 0.01); end
check('A 한 번 = steerRate × 0.12 s', abs(st.delta - tp.steerRate * tp.pulse) < 1e-9);
check('A 뒤 차가 왼쪽으로 돎 (yaw 증가)', st.yawRate > 0);
for i = 1:20, st = twin_key(st, "a", tp); st = twin_model_step(st, tp, p, 0.033); end
check('A 계속 누르면 최대 조향각에서 멈춤', abs(st.delta - tp.deltaMax) < 1e-9);
st = twin_key(st, "space", tp);
check('Space: PWM 명령 0, 조향 모터 멈춤, 조향각 유지', st.pwmCmd == 0 && st.steerLeft == 0 && abs(st.delta - tp.deltaMax) < 1e-9);
for i = 1:12, st = twin_key(st, "s", tp); end
check('S는 -maxSpeed에서 멈춤 (후진 한계)', st.pwmCmd == -tp.maxSpeed);
% 조향 중립 오프셋: 키를 안 눌러도 delta0 방향으로 휨 (맞춘 값 -1.6° = 오른쪽)
tpo = twin_params(); st = st0;
for i = 1:4, st = twin_key(st, "w", tpo); end
for i = 1:300, st = twin_model_step(st, tpo, p, 0.01); end
check('delta0 ≠ 0 이면 조향 0 에서도 delta0 쪽으로 휨 (yawRate 부호 = delta0 부호)', tpo.delta0 ~= 0 && sign(st.yawRate) == sign(tpo.delta0));

% 4b. Python 맞추기 스크립트(analysis/fit_twin_params.py replay)와 같은 모델인지: 같은 키 → 같은 위치
%     기준값 = Python replay(맞춘 파라미터, twin_demo_keys, 8.5 s) = (6.6967, -0.6814, 1.7033 rad)  (2026-11-22 맞춤 값)
K = twin_demo_keys(); tk = str2double(K(:, 1)); st = st0; ik = 1;
while st.t < 8.5 - 1e-9
    while ik <= numel(tk) && tk(ik) <= st.t + 1e-9, st = twin_key(st, K(ik, 2), tpo); ik = ik + 1; end
    st = twin_model_step(st, tpo, p, 0.01);
end
check('MATLAB 모델 = Python replay (8.5 s 뒤 위치 차이 < 3 cm)', hypot(st.x - 6.6967, st.y + 0.6814) < 0.03 && abs(st.yaw - 1.7033) < 0.03);

% 4c. 실차 APPLY 따라가기: 속도는 실차가 알려 준 값이 우선, 조향은 twin_key와 같은 펄스
st = twin_apply(st0, "w", 20, tpo);              % 시뮬 규칙(step 10)으론 10이지만 실차가 20이라고 함
check('APPLY: 실차 PWM(20)으로 맞추고 delay 뒤 반영 예약', st.pwmCmd == 20 && size(st.pend, 1) == 1 && st.pend(1, 2) == 20);
st = twin_apply(st, "w", 30, tpo);
check('APPLY: 실차 값과 같으면 예약 하나만 추가', st.pwmCmd == 30 && size(st.pend, 1) == 2);
st = twin_apply(st, "a", 30, tpo);
check('APPLY a: 조향 펄스, 속도 그대로', st.steerDir == 1 && st.steerLeft == tpo.pulse && st.pwmCmd == 30 && size(st.pend, 1) == 2);
st = twin_apply(st, "space", 0, tpo);
check('APPLY space: 정지', st.pwmCmd == 0 && st.steerLeft == 0);
st = twin_apply(st0, "s", NaN, tpo);
check('APPLY 속도 모름(NaN): twin_key 규칙', st.pwmCmd == -tpo.step);

% 4d. 무게: massRef(15 kg)에서는 맞춘 값 그대로, 무거우면 느리고 늦게, 가벼우면 빠르게 (PWM 0이면 멈춤)
tm = twin_params();
check('무게 15 kg = 맞춘 모델 (tauV 0.589, pwmDead 13.55)', tm.mass == 15 && abs(tm.tauV - 0.589) < 1e-12 && abs(tm.pwmDead - 13.55) < 1e-12);
tH = twin_mass(tm, 20); tL = twin_mass(tm, 10);
vss = @(t, pwm) t.kv * sign(pwm) * max(abs(pwm) - t.pwmDead, 0);
check('20 kg: tauV = 0.589·20/15, PWM 40 정상 속도가 15 kg보다 느림', abs(tH.tauV - 0.589 * 20 / 15) < 1e-9 && vss(tH, 40) < vss(tm, 40) && tH.pwmDead > tm.pwmDead);
check('10 kg: 더 빠름, PWM 0이면 0', vss(tL, 40) > vss(tm, 40) && vss(tL, 0) == 0 && tL.tauV < tm.tauV);
check('twin_params(''mass'', 20) = twin_mass(…, 20)', abs(twin_params('mass', 20).pwmDead - tH.pwmDead) < 1e-12);
st = st0; for i = 1:4, st = twin_key(st, "w", tH); end
for i = 1:700, st = twin_model_step(st, tH, p, 0.01); end
check('20 kg 모델로 적분해도 정상 속도에 수렴', abs(st.v - vss(tH, 40)) < 1e-3);

% 4e. 루엔버그 옵저버: '실차' = 조향 오프셋 −6°, 속도 25% 빠름 (모델은 모름). 15 Hz 측정 (2 cm·0.5° 잡음, 0.1 s 지연)
tpo = twin_params('delta0', deg2rad(-1.6)); tTrue = tpo; tTrue.delta0 = deg2rad(-6); tTrue.kv = tpo.kv * 1.25;
rng(1); sT = st0; sE = st0; sN = st0; hist = zeros(0, 4); lastM = -1; obsT0 = NaN;
keysT = [0.5 1 1.5 2]; aT = [4 4.2 9 9.3]; dT = [13 13.2];
for k = 0:2000                                    % 20 s, 10 ms
    tk = k * 0.01;
    for kk = find(abs(keysT - tk) < 0.005), sT = twin_key(sT, "w", tTrue); sE = twin_key(sE, "w", tpo); sN = twin_key(sN, "w", tpo); end
    for kk = find(abs(aT - tk) < 0.005), sT = twin_key(sT, "a", tTrue); sE = twin_key(sE, "a", tpo); sN = twin_key(sN, "a", tpo); end
    for kk = find(abs(dT - tk) < 0.005), sT = twin_key(sT, "d", tTrue); sE = twin_key(sE, "d", tpo); sN = twin_key(sN, "d", tpo); end
    sT = twin_model_step(sT, tTrue, p, 0.01); sE = twin_model_step(sE, tpo, p, 0.01); sN = twin_model_step(sN, tpo, p, 0.01);
    hist(end + 1, :) = [sT.t, sT.x, sT.y, sT.yaw]; %#ok<AGROW>
    if sE.t - lastM >= 1/15 - 1e-9 && sE.t > 0.2
        lastM = sE.t; j = find(hist(:, 1) <= sE.t - 0.1 + 1e-9, 1, 'last');
        m = hist(j, 2:4) + [0.02 * randn, 0.02 * randn, deg2rad(0.5) * randn];
        if isnan(obsT0), dtm = 1/15; else, dtm = sE.t - obsT0; end
        [sE, innT] = twin_observer(sE, m, 0.1, dtm, tpo, p); obsT0 = sE.t; %#ok<ASGLU>
    end
end
errObs = hypot(sE.x - sT.x, sE.y - sT.y); errOpen = hypot(sN.x - sT.x, sN.y - sT.y);
check(sprintf('옵저버: 20 s 뒤 위치 오차 %.3f m (옵저버 없이 %.2f m)', errObs, errOpen), errObs < 0.08 && errOpen > 0.5);
check(sprintf('옵저버: 속도 배율 추정 %.2f (실제 0.25)', sE.vs), abs(sE.vs - 0.25) < 0.08);
check(sprintf('옵저버: 실효 조향 오차 %.1f° (조향 오프셋 -4.4° 를 추정)', rad2deg((sE.delta + tpo.delta0) - (sT.delta + tTrue.delta0))), ...
      abs(rad2deg((sE.delta + tpo.delta0) - (sT.delta + tTrue.delta0))) < 1.5);
sG = twin_observer(st0, [st0.x + 5, st0.y, st0.yaw], 0, 0.07, tpo, p);
check('옵저버: 1 m 넘게 튀는 측정은 버림', sG.x == st0.x && sG.y == st0.y);

% 4f. 칼만 필터: 위와 같은 '실차' + 8~12 s 동안 측정이 0.8 m 틀림 (그 방향 분산이 크다고 같이 알려 줌 = 마커가 멀리 한 줄)
for est = ["ekf", "luenberger"]
    tpe = twin_params('delta0', deg2rad(-1.6), 'estimator', est); tTr = tpe; tTr.delta0 = deg2rad(-6); tTr.kv = tpe.kv * 1.25;
    rng(2); sT = st0; sE = st0; sE.P = diag(tpe.ekf.P0); hist = zeros(0, 4); lastM = -1; obsT0 = NaN; errs = [];
    for k = 0:2000
        tk = k * 0.01; nk = 0;
        for kk = find(abs(keysT - tk) < 0.005), sT = twin_key(sT, "w", tTr); sE = twin_key(sE, "w", tpe); end
        for kk = find(abs(aT - tk) < 0.005), sT = twin_key(sT, "a", tTr); sE = twin_key(sE, "a", tpe); nk = nk + 1; end
        for kk = find(abs(dT - tk) < 0.005), sT = twin_key(sT, "d", tTr); sE = twin_key(sE, "d", tpe); nk = nk + 1; end
        sT = twin_model_step(sT, tTr, p, 0.01); sE = twin_model_step(sE, tpe, p, 0.01);
        if est == "ekf", sE = twin_ekf_predict(sE, tpe, p, 0.01, nk); end
        hist(end + 1, :) = [sT.t, sT.x, sT.y, sT.yaw]; %#ok<AGROW>
        if sE.t - lastM >= 1/15 - 1e-9 && sE.t > 0.2
            lastM = sE.t; j = find(hist(:, 1) <= sE.t - 0.1 + 1e-9, 1, 'last');
            m = hist(j, 2:4) + [0.02 * randn, 0.02 * randn, deg2rad(0.5) * randn];
            Rz = diag([0.03^2, 0.03^2, deg2rad(1)^2]);
            if sE.t > 8 && sE.t < 12                     % 틀린 측정: 차 앞 방향(cos30°, sin30°)으로 0.8 m, 그 방향 분산 (1 m)²
                u = [cosd(30); sind(30)]; m(1:2) = m(1:2) + 0.8 * u';
                Rz(1:2, 1:2) = Rz(1:2, 1:2) + 1.0^2 * (u * u');
            end
            if est == "ekf"
                sE = twin_ekf_update(sE, m, Rz, 0.1, tpe, p);
            else
                if isnan(obsT0), dtm = 1/15; else, dtm = sE.t - obsT0; end
                sE = twin_observer(sE, m, 0.1, dtm, tpe, p); obsT0 = sE.t;
            end
            if sE.t > 8 && sE.t < 13, errs(end + 1) = hypot(sE.x - sT.x, sE.y - sT.y); end %#ok<AGROW>
        end
    end
    eMax.(est) = max(errs); eEnd.(est) = hypot(sE.x - sT.x, sE.y - sT.y);
end
check(sprintf('칼만: 틀린 측정 구간 최대 오차 %.2f m (루엔버그 %.2f m)', eMax.ekf, eMax.luenberger), eMax.ekf < 0.15 && eMax.luenberger > 2 * eMax.ekf);
check(sprintf('칼만: 20 s 뒤 오차 %.3f m', eEnd.ekf), eEnd.ekf < 0.08);

% 5. 시뮬만 주행 (Unity·실차 없이, 시험용 키)
r = twin_drive('useUnity', false, 'useCar', false, 'useDash', false, 'keys', twin_demo_keys());
check('시뮬 주행 기록 저장', isfile(fullfile(r.folder, 'timeseries.csv')) && isfile(fullfile(r.folder, 'summary.json')) ...
      && isfile(fullfile(r.folder, 'trajectory_track.png')));
check('시뮬 주행: 앞으로 0.3 m 이상 (PWM 20 = 0.068 m/s), 횡오차 기록', r.distance_m > 0.3 && isfinite(r.e_lat_rms_m));

fprintf('\n==== test_twin: %d / %d passed ====\n', nPass, nAll);
if nPass < nAll, error('test_twin 실패'); end

    function check(name, ok)
        nAll = nAll + 1;
        if ok, nPass = nPass + 1; fprintf('  PASS  %s\n', name);
        else, fprintf('  FAIL  %s\n', name); end
    end
end
