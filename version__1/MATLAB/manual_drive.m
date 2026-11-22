function manual_drive()
% 방향키로 차량을 직접 운전하고, 그 상태를 Unity로 보내는 코드
% 차량 모델: 기구학 자전거 모델 (기준점 = 뒤 차축 중앙)
%   차량 파라미터·모델·Unity 전송은 pure_pursuit_control.m과 같은 공용 파일을 씀
%   (vehicle_params.m, bicycle_step.m, unity_link.m)
%
% 조작 (MATLAB에 뜨는 그림 창을 클릭한 상태에서)
%   ↑ : 가속 (후진 중이면 브레이크)
%   ↓ : 브레이크 (멈춘 뒤 계속 누르면 후진)
%   ← → : 조향 (떼면 가운데로 돌아옴)
%   Space : 정지
%   Q 또는 Esc : 종료 (그림 창을 닫아도 종료)
%
% 실행 순서: Unity Play 먼저 → 이 함수 실행

%% 1. 출발 위치: Unity에서 car의 Inspector 값을 그대로 적기
unityX    = -5.319;   % car Position X
unityZ    =  0.439;   % car Position Z
unityRotY =  0;       % car Rotation Y [deg]

% Plane이 (0,0,0)이므로 Unity X = MATLAB x, Unity Z = MATLAB y
% 좌표 약속: Rotation Y = 90 - yaw
st = struct('t', 0, 'x', unityX, 'y', unityZ, 'yaw', deg2rad(90 - unityRotY), ...
    'v', 0, 'delta', 0, 'yawRate', 0);

%% 2. 차량 파라미터 (공용: vehicle_params.m)
p    = vehicle_params();
dt   = p.dt;                 % 계산·전송 주기 [s] (50 Hz)
drag = 0.5;                  % 키를 뗐을 때 자연 감속 [m/s^2] (수동 주행에서만 사용)

%% 3. Unity 연결
client = unity_link('connect', p);
if isempty(client)
    error("Unity에 연결할 수 없습니다. Unity에서 Play를 먼저 눌렀는지 확인하세요.");
end
cleanupObj = onCleanup(@() clear('client'));

%% 4. 키 입력 창과 궤적 그림
keys = struct('up', false, 'down', false, 'left', false, 'right', false, 'space', false);
running = true;

fig = figure('Name', '수동 주행 - 이 창을 클릭한 뒤 방향키로 조작', 'NumberTitle', 'off', ...
    'KeyPressFcn', @onKeyPress, 'KeyReleaseFcn', @onKeyRelease, ...
    'CloseRequestFcn', @onClose);
ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
plot(ax, p.mapHalfX * [-1 1 1 -1 -1], p.mapHalfY * [-1 -1 1 1 -1], 'k-');   % 맵 테두리
xlim(ax, [-p.mapHalfX p.mapHalfX] * 1.05); ylim(ax, [-p.mapHalfY p.mapHalfY] * 1.05);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
hTraj = plot(ax, st.x, st.y, 'b-', 'LineWidth', 1.5);          % 지나온 길
hCar  = plot(ax, st.x, st.y, 'ro', 'MarkerFaceColor', 'r');    % 차 위치 (뒤 차축)
hHead = plot(ax, [st.x st.x], [st.y st.y], 'r-', 'LineWidth', 2);    % 차 방향
hTxt  = title(ax, '');

maxPts = 200000;
trajX = nan(1, maxPts); trajY = nan(1, maxPts);

%% 5. 메인 루프
step = 0;
t0 = tic;

while running && isvalid(fig)
    % --- 조향 입력 → 목표 조향각 (조향 속도 제한은 bicycle_step에서) ---
    if keys.left && ~keys.right
        deltaTarget = p.deltaMax;      % 왼쪽 +
    elseif keys.right && ~keys.left
        deltaTarget = -p.deltaMax;
    else
        deltaTarget = 0;
    end

    % --- 가속/브레이크 입력 → 다음 속도 (최고 속도 제한은 bicycle_step에서) ---
    v = st.v;
    if keys.space
        v = moveToward(v, 0, p.brake * dt);
    elseif keys.up
        if v < 0, v = v + p.brake * dt; else, v = v + p.accel * dt; end
    elseif keys.down
        if v > 0, v = v - p.brake * dt; else, v = v - p.accel * dt; end
    else
        v = moveToward(v, 0, drag * dt);
    end

    % --- 기구학 자전거 모델 (공용: bicycle_step.m, 뒤 차축 기준) ---
    st = bicycle_step(st, struct('delta_cmd', deltaTarget, 'v_cmd', v), p, dt);

    % --- Unity로 전송: t,x,y,yaw,v,delta,yawrate,e_lat,e_head ---
    if ~unity_link('state', client, st)
        warning("Unity 연결이 끊겼습니다. 종료합니다.");
        break;
    end

    % --- 그림 갱신 (5스텝마다) ---
    step = step + 1;
    if step <= maxPts
        trajX(step) = st.x; trajY(step) = st.y;
    end
    if mod(step, 5) == 0
        set(hTraj, 'XData', trajX(1:min(step, maxPts)), 'YData', trajY(1:min(step, maxPts)));
        set(hCar,  'XData', st.x, 'YData', st.y);
        set(hHead, 'XData', [st.x, st.x + 0.5 * cos(st.yaw)], 'YData', [st.y, st.y + 0.5 * sin(st.yaw)]);
        hTxt.String = sprintf('t = %.1f s   v = %.2f m/s   steer = %.1f deg', ...
            st.t, st.v, rad2deg(st.delta));
    end

    % --- 실제 시간에 맞추기 (키 입력 처리도 여기서 됨) ---
    waitTime = step * dt - toc(t0);
    if waitTime > 0
        pause(waitTime);
    else
        drawnow limitrate;
    end
end

if isvalid(fig), delete(fig); end
fprintf('수동 주행 종료 (%.1f s)\n', st.t);

%% ---- 키 입력 처리 (중첩 함수) ----
    function onKeyPress(~, evt)
        switch evt.Key
            case 'uparrow',    keys.up = true;
            case 'downarrow',  keys.down = true;
            case 'leftarrow',  keys.left = true;
            case 'rightarrow', keys.right = true;
            case 'space',      keys.space = true;
            case {'q', 'escape'}, running = false;
        end
    end

    function onKeyRelease(~, evt)
        switch evt.Key
            case 'uparrow',    keys.up = false;
            case 'downarrow',  keys.down = false;
            case 'leftarrow',  keys.left = false;
            case 'rightarrow', keys.right = false;
            case 'space',      keys.space = false;
        end
    end

    function onClose(src, ~)
        running = false;
        delete(src);
    end
end

function out = moveToward(cur, target, maxStep)
% cur를 target 쪽으로 최대 maxStep만큼 이동
out = cur + max(-maxStep, min(maxStep, target - cur));
end
