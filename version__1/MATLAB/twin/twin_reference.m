function ref = twin_reference(frame)
% [twin] 실차와 같은 기준선: 도면의 2차선 중심선 (~/demo/track/lane2_centerline.csv 복사본, 5 cm 간격, 39.78 m 한 바퀴)
%   ref = twin_reference('sim')    시뮬레이션 좌표 (Unity 표시 · 시뮬 횡오차)
%   ref = twin_reference('track')  트랙 좌표 (분석용)
% path_error.m 이 쓰는 형식 (x, y, s, psi, kappa, N, ds, L, closed)
if nargin < 1, frame = 'sim'; end
M = readmatrix(fullfile(fileparts(mfilename('fullpath')), 'lane2_centerline.csv'));
s = M(:, 1); x = M(:, 2); y = M(:, 3); psi = deg2rad(M(:, 4)); kappa = M(:, 5);
if strcmp(frame, 'sim')
    q = twin_frames('track2sim', [x, y, psi]);
    x = q(:, 1); y = q(:, 2); psi = q(:, 3);   % 180° 회전: 진행 방향·곡률 부호(좌회전 +)는 그대로
end
ref = struct('x', x, 'y', y, 's', s, 'psi', psi, 'kappa', kappa);
ref.N = numel(x);
ref.ds = median(diff(s));
ref.L = s(end) + hypot(x(1) - x(end), y(1) - y(end));
ref.closed = true;
ref.name = "lane2_centerline (" + frame + ")";
end
