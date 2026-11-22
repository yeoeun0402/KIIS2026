function h = draw_map(ax, p)
% Unity 맵 텍스처(image.png)를 월드 좌표(x = Unity X, y = Unity Z)로 깔기
% 텍스처는 월드에서 180도 돌아가 있음: 이미지 왼쪽 위 = 월드 (+X, -Z) 쪽

persistent img
if isempty(img)
    img = imread(fullfile(fileparts(mfilename('fullpath')), '..', 'KIIS2026_AutoCar', 'Assets', 'image.png'));
end
[H, W, ~] = size(img);
hold(ax, 'on');
h = image(ax, 'XData', p.mapHalfX * (1 - 1 / W) * [1 -1], ...   % 첫 열 = +X 쪽
              'YData', p.mapHalfY * (1 - 1 / H) * [-1 1], ...   % 첫 행 = -Z 쪽
              'CData', img);
axis(ax, 'equal'); axis(ax, 'xy');
xlim(ax, [-p.mapHalfX p.mapHalfX]); ylim(ax, [-p.mapHalfY p.mapHalfY]);
xlabel(ax, 'x [m] (= Unity X)'); ylabel(ax, 'y [m] (= Unity Z)');
end
