function st = twin_key(st, key, tp)
% [twin] 키 하나 → 모델 입력 (실차 drive.py의 Drive.key()와 같은 규칙)
%   'w' : pwm + step (한계 maxSpeed)     's' : pwm - step (한계 -maxSpeed)
%   'a' : 왼쪽 조향 펄스 (펌웨어: 120 ms 모터, 다시 오면 그때부터 120 ms)   'd' : 오른쪽
%   'space' : pwm 0 + 조향 모터 정지 (조향각은 그대로)
% 속도 명령(pwmCmd)은 바로 바뀌고, 바퀴에는 tp.delay 뒤에 반영 (st.pend → twin_model_step)
old = st.pwmCmd;
switch key
    case "w"
        st.pwmCmd = min(st.pwmCmd + tp.step, tp.maxSpeed);
    case "s"
        st.pwmCmd = max(st.pwmCmd - tp.step, -tp.maxSpeed);
    case "a"
        st.steerDir = +1; st.steerLeft = tp.pulse;      % MATLAB 좌표: 왼쪽 조향 = delta +
    case "d"
        st.steerDir = -1; st.steerLeft = tp.pulse;
    case "space"
        st.pwmCmd = 0; st.steerDir = 0; st.steerLeft = 0;
end
if st.pwmCmd ~= old
    st.pend(end + 1, :) = [st.t + tp.delay, st.pwmCmd];
end
end
