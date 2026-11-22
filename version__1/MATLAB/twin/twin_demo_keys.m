function K = twin_demo_keys()
% [twin] 시험용 키 순서 (Unity 없이 twin_drive('useUnity', false, 'keys', twin_demo_keys()) 로 흐름 확인)
%   0.5 s W, 0.7 s W (PWM 20) → 2.0~2.6 s A를 누르고 있음(키 반복 0.033 s) → 5.0~5.4 s D 누르고 있음
%   → 8.0 s Space(정지) → 그 뒤 0.5 s 지나면 twin_drive가 Q로 끝냄
t = [0.5; 0.7; (2.0:0.033:2.6)'; (5.0:0.033:5.4)'; 8.0];
k = [repmat("w", 2, 1); repmat("a", numel(2.0:0.033:2.6), 1); repmat("d", numel(5.0:0.033:5.4), 1); "space"];
K = [string(compose('%.3f', t)), k];
end
