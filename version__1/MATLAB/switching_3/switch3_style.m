function S = switch3_style()
% switching_3 그림 공통 색·이름 (switch3_fig1.m, switch3_fig2.m)
%   Figure 1: PP 초록, PID 파랑, BC 노랑, 블랜디드 보라
%   Figure 2: 블랜디드 보라, 레퍼런스 갈색, 오차 빨강
S.names = ["PP", "PID", "BC", "블랜디드"];          % 차 순서 (데이터 열 순서와 같음)
S.tags  = ["pp", "pid", "bc", "bl"];               % timeseries.csv 열 이름 앞부분
S.col   = [0.10 0.62 0.10;                         % PP: 초록
           0.00 0.40 0.95;                         % PID: 파랑
           0.93 0.73 0.00;                         % BC: 노랑
           0.55 0.15 0.75];                        % 블랜디드: 보라
S.brown = [0.55 0.33 0.12];                        % 레퍼런스: 갈색
S.red   = [0.85 0.10 0.10];                        % 오차: 빨강
S.modes = ["PP", "PID", "BC"];                     % λ0, λ1, λ2 순서
S.bandAlpha = 0.13;                                % 블랜디드가 쓴 제어기 배경 띠의 투명도
% 세로축 눈금 간격 후보 (switch3_yaxis): 같은 양은 Figure 1, 2 모두 같은 범위·같은 간격
%   눈금 칸이 12개 이하가 되는 가장 작은 값을 씀 → 보통 조향각 5°, 궤적 오차 0.02 m
S.yStep = struct('deg', [5 10 20], 'm', [0.02 0.05 0.1 0.2 0.5]);
end
