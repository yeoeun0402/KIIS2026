function out = twin_frames(dir, in)
% [twin] 시뮬레이션 좌표 ↔ 실차 트랙 좌표 변환
%   q_track = twin_frames('sim2track', [x y theta])   (N x 3, 각도 rad)
%   q_sim   = twin_frames('track2sim', [x y theta])
% 시뮬레이션: 원점 매트 가운데 (MATLAB x = Unity X, y = Unity Z), Unity 맵 텍스처는 도면과 180° 돌아가 있음
% 트랙(~/demo/track/track.yaml): 원점 매트 왼쪽 아래 모서리, x 오른쪽(16 m), y 위(11 m)
%   x_t = 8 - x_s,  y_t = 5.5 - y_s,  θ_t = θ_s - π   (같은 식을 거꾸로 써도 같음: 180° 회전은 자기 자신이 역변환)
% 둘 다 기준점은 뒷축 중앙 바닥 투영(base_link)
halfX = 8.0; halfY = 5.5;
switch dir
    case {'sim2track', 'track2sim'}
        out = [halfX - in(:, 1), halfY - in(:, 2), wrapPi(in(:, 3) - pi)];
    otherwise
        error("twin_frames: 'sim2track' 또는 'track2sim'");
end
end

function a = wrapPi(a)
a = atan2(sin(a), cos(a));
end
