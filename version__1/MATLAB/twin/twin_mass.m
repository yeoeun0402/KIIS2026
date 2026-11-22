function tp = twin_mass(tp, mass)
% [twin] 차 무게 [kg] → 속도 모델 값 (tauV, pwmDead)
%   운동 방정식:  m v' = a·PWM − b·v − crr·g·(m − massRef)
%     a [N/PWM] : 모터가 PWM 1당 내는 힘,  b [N·s/m] : 모터 역기전력·마찰 (속도에 비례해 줄어드는 힘)
%     massRef에서 맞춘 kv·tauVRef로 정함:  b = massRef / tauVRef,  a = kv · b
%   → 1차 지연 형태 (twin_model_step와 같은 식):  v_ref = kv · (|PWM| − pwmDead),  시정수 tauV
%     tauV    = m / b                            = tauVRef · m / massRef
%     pwmDead = pwmDeadRef + crr·g·(m − massRef) / a
%   massRef보다 가벼우면 pwmDead가 음수 → 같은 PWM에서 더 빠름 (PWM 0이면 그대로 멈춤)
%   조향(steerRate, deltaMax)과 kv는 무게로 바꾸지 않음 (측정 근거 없음)
mass = max(double(mass), 0.5);
b = tp.massRef / tp.tauVRef;
a = tp.kv * b;
tp.mass = mass;
tp.tauV = tp.tauVRef * mass / tp.massRef;
tp.pwmDead = tp.pwmDeadRef + tp.crr * tp.g * (mass - tp.massRef) / a;
end