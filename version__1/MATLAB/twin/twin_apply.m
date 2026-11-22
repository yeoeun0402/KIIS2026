function st = twin_apply(st, key, speed, tp)
% [twin] 실차가 "이 키를 적용했다"고 알려 준 줄 (APPLY,<key>,<speed>) → 시뮬 모델 입력
%   실차 노드(wasd_drive)는 MATLAB 키 · 실차 터미널 키 · 안전 정지(연결 끊김·0.5 s 무신호)를
%   Drive.key()로 적용할 때마다 이 줄을 보낸다. 시뮬은 이것만 따라 움직이므로
%   속도 조절이 실차와 같은 값 하나(실차 Drive.speed)를 따른다.
%   key   : "w" | "s" | "a" | "d" | "space" | "q"
%   speed : 실차가 적용한 뒤의 PWM (NaN이면 twin_key 규칙만 씀)
% 조향(a/d)과 Space의 조향 모터 정지는 twin_key와 같음. 속도는 실차 값을 그대로 씀 (delay 뒤 바퀴에 반영).
old = st.pwmCmd; nPend = size(st.pend, 1);
st = twin_key(st, key, tp);
if ~isnan(speed)
    st.pend = st.pend(1:nPend, :);       % twin_key가 넣은 예약은 버리고 실차 값으로
    st.pwmCmd = speed;
    if speed ~= old
        st.pend(end + 1, :) = [st.t + tp.delay, speed];
    end
end
end
