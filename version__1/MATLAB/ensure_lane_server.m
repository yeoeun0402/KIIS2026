function info = ensure_lane_server(varargin)
% lane_server.py(카메라 차선 판단 서버)가 켜져 있는지 확인하고, 꺼져 있으면 새 창으로 켠 뒤 준비될 때까지 기다림
%   switch_discrete / switch2_discrete / switch3_discrete가 카메라 판단(decision = "camera")으로 시작할 때 부름
%   (2026-11-20 추가: lane_server를 켜지 않고 실행해서 "LANE 줄이 10 s 동안 하나도 오지 않았습니다" 오류가 났음)
%
%   1) 55002(Unity → lane_server 사진 포트)가 이미 열려 있으면 → 그대로 씀 (서버를 하나 더 켜지 않음)
%   2) 닫혀 있지만 lane_server.py 프로세스가 있으면 (켜지는 중) → 포트가 열릴 때까지 기다림
%   3) 둘 다 아니면 → 새 창 "lane_server"로 켜고 55002·55003이 열릴 때까지 기다림
%      창은 계속 남아 lane_server 출력을 보여 줌. MATLAB 실행이 끝나도 서버는 켜진 채로 남아 다음 실행에 다시 씀
%      시작 중에 오류로 꺼지면 바로 알려 줌 (오류 내용은 그 창에 남아 있음)
%   그 뒤 Unity(Car Camera Capture)가 55002에 연결될 때까지 잠깐 기다림 (Unity가 알아서 다시 접속함)
%   서버가 두 개 켜지면 안 됨: Windows에서는 lane_server의 SO_REUSEADDR 때문에 두 번째 서버도 같은 포트에 붙어
%   연결을 나눠 가질 수 있음 → 그래서 켜기 전에 포트와 프로세스를 모두 확인함
%
% 이름-값 옵션:
%   'python'     lane_server를 돌릴 python (기본: Windows는 venv kiis_yolo의 python.exe,
%                Linux는 <KIIS2026_Unity>/lane_yolo/.venv/bin/python, setup_linux.sh가 만듦)
%   'script'     lane_server.py (기본: <KIIS2026_Unity>/lane_yolo/lane_server.py)
%   'port'       55002   Unity → lane_server 사진
%   'ctrlPort'   55003   MATLAB → lane_server 사진 저장 명령 (switch2/3의 featurePort)
%   'timeout'    120     서버가 준비될 때까지 최대 [s] (재부팅 뒤 첫 실행은 PyTorch·CUDA 준비로 오래 걸릴 수 있음)
%   'unityWait'  5       Unity가 연결될 때까지 최대 [s]. 안 되면 경고만 하고 계속
% 반환: info.status  "running" (이미 켜져 있음) | "started" (새로 켬) | "waited" (켜지는 중이던 서버를 기다림)
%                    | "unknown" (포트 확인 명령 실패: 확인 못 함)
%   Linux (2026-11-21 추가): 포트는 ss, 프로세스는 pgrep으로 확인하고, 새 창은 gnome-terminal로 엶
%                            (gnome-terminal이 없으면 창 없이 백그라운드로 켜고 출력은 tempdir/lane_server_<port>.log)
%       info.waited [s], info.unityConnected

root = fileparts(fileparts(mfilename('fullpath')));      % <KIIS2026_Unity>
if ispc
    defaultPython = 'C:\Users\haepi\venvs\kiis_yolo\Scripts\python.exe';
else
    defaultPython = fullfile(root, 'lane_yolo', '.venv', 'bin', 'python');
end
o = struct('python', defaultPython, ...
    'script', fullfile(root, 'lane_yolo', 'lane_server.py'), ...
    'port', 55002, 'ctrlPort', 55003, 'timeout', 120, 'unityWait', 5);
for i = 1:2:numel(varargin)
    nm = char(varargin{i});
    if ~isfield(o, nm), error("ensure_lane_server: 알 수 없는 옵션 '%s'", nm); end
    o.(nm) = varargin{i + 1};
end
o.python = char(o.python); o.script = char(o.script);
info = struct('status', "running", 'waited', 0, 'unityConnected', false);

t0 = tic;
net = tcpState([o.port o.ctrlPort]);
if ~net.ok
    warning('ensure_lane_server:netstat', '포트를 확인하지 못해(netstat/ss) lane_server를 자동으로 켜지 않습니다.');
    info.status = "unknown"; return;
end

if ~net.listening(1)
    proc = []; flag = "";
    if serverProcessExists()
        info.status = "waited";
        fprintf('lane_server가 켜지는 중입니다. 준비될 때까지 기다립니다 (최대 %.0f s)\n', o.timeout);
    else
        if ~isfile(o.python), error('ensure_lane_server: lane_server를 켤 python이 없습니다: %s', o.python); end
        if ~isfile(o.script), error('ensure_lane_server: lane_server.py가 없습니다: %s', o.script); end
        flag = string(fullfile(tempdir, sprintf('lane_server_%d_exited.flag', o.port)));
        if isfile(flag), delete(flag); end
        proc = startServer(o, flag);
        info.status = "started";
        fprintf('lane_server가 꺼져 있어 새 창(lane_server)으로 켭니다. 모델이 준비될 때까지 기다립니다 (최대 %.0f s)\n', o.timeout);
    end
    % 새로 켠 서버는 사진 저장 포트도 열릴 때까지 (켜지는 중이던 서버는 옛 버전일 수 있어 55002만)
    waitReady(o, info.status == "started" && o.ctrlPort > 0, proc, flag, t0);
    info.waited = toc(t0);
    fprintf('lane_server 준비됨 (%.1f s)\n', info.waited);
end

% Unity(Car Camera Capture)가 55002에 연결될 때까지
tU = tic;
while true
    net = tcpState(o.port);
    if net.client(1), info.unityConnected = true; break; end
    if toc(tU) >= o.unityWait, break; end
    pause(0.25);
end
if info.unityConnected
    fprintf('lane_server 확인: 켜져 있고 Unity(Car Camera Capture)가 연결됨 (포트 %d)\n', o.port);
else
    warning('ensure_lane_server:unity', ['lane_server는 켜져 있지만 Unity(Car Camera Capture)가 %.0f s 안에 연결하지 않았습니다.\n' ...
        '  Unity가 Play 중인지, Car_Camera → Car Camera Capture의 Send To Lane Server가 켜져 있는지 확인하세요.'], o.unityWait);
end
end

%% ---------- 내부 함수 ----------
function s = tcpState(ports)
% netstat(Windows) / ss(Linux) 한 번으로 포트마다 listening(서버가 열어 둠), client(누가 그 포트로 연결해 있음)
s = struct('ok', false, 'listening', false(size(ports)), 'client', false(size(ports)));
if ~ispc
    % ss -tanH 한 줄: "LISTEN 0 5 0.0.0.0:55002 0.0.0.0:*" / "ESTAB 0 0 127.0.0.1:41234 127.0.0.1:55002"
    [st, out] = system('ss -tanH');
    if st ~= 0, return; end
    s.ok = true;
    for j = 1:numel(ports)
        p = num2str(ports(j));
        s.listening(j) = ~isempty(regexp(out, ['(^|\n)LISTEN\s+\d+\s+\d+\s+\S+:' p '\s'], 'once'));
        s.client(j) = ~isempty(regexp(out, ['(^|\n)ESTAB\s+\d+\s+\d+\s+\S+:\d+\s+\S+:' p '(\s|$)'], 'once'));
    end
    return;
end
[st, out] = system('netstat -ano -p TCP');
if st ~= 0, return; end
s.ok = true;
for j = 1:numel(ports)
    p = num2str(ports(j));
    % 대기 중: 로컬 주소가 :포트, 외부 주소 0.0.0.0:0
    s.listening(j) = ~isempty(regexp(out, ['TCP\s+[\d.]+:' p '\s+0\.0\.0\.0:0\s'], 'once'));
    % 연결됨: 외부 주소가 :포트이고 ESTABLISHED, PID가 0이 아님 (TIME_WAIT 찌꺼기 제외)
    s.client(j) = ~isempty(regexp(out, ['TCP\s+[\d.]+:\d+\s+[\d.]+:' p '\s+ESTABLISHED\s+[1-9]'], 'once'));
end
end

function tf = serverProcessExists()
% 명령줄에 lane_server.py가 들어 있는 python 프로세스가 있는지 (따옴표 문제를 피하려고 -EncodedCommand)
if ~ispc
    % [.]: system()이 띄운 sh 자신의 명령줄("lane_server[.]py")은 이 정규식에 걸리지 않음
    [st, ~] = system('pgrep -f "lane_server[.]py"');
    tf = st == 0;
    return;
end
ps = ['@(Get-CimInstance Win32_Process -Filter "Name LIKE ''python%''" | ' ...
      'Where-Object { $_.CommandLine -like ''*lane_server.py*'' }).Count'];
enc = matlab.net.base64encode(unicode2native(ps, 'UTF-16LE'));
[st, out] = system(['powershell -NoProfile -NonInteractive -EncodedCommand ' enc]);
n = str2double(regexp(out, '\d+(?=\s*$)', 'match', 'once'));
tf = st == 0 && n > 0;
end

function proc = startServer(o, flag)
% 새 콘솔 창에서 실행. cmd /k라서 서버가 꺼져도 창이 남아 오류 메시지를 볼 수 있음.
% 서버가 꺼지면 flag 파일을 만들어 MATLAB이 시작 실패를 바로 알아챔
if ~ispc
    proc = startServerLinux(o, flag);
    return;
end
inner = sprintf(['title lane_server & "%s" "%s" --port %d --ctrl-port %d & type nul > "%s" & echo. & ' ...
    'echo [lane_server] 서버가 꺼졌습니다. 위 메시지를 확인하세요. 이 창은 닫아도 됩니다.'], ...
    o.python, o.script, o.port, o.ctrlPort, flag);
psi = System.Diagnostics.ProcessStartInfo('cmd.exe', ['/d /k "' inner '"']);
psi.UseShellExecute = true;                       % 새 창 (MATLAB과 입출력을 공유하지 않음)
psi.WorkingDirectory = fileparts(o.script);
proc = System.Diagnostics.Process.Start(psi);
end

function waitReady(o, needCtrl, proc, flag, t0)
tShown = 0;
while true
    net = tcpState([o.port o.ctrlPort]);
    if net.listening(1) && (~needCtrl || net.listening(2)), return; end
    if flag ~= "" && isfile(flag)
        delete(flag);
        error('ensure_lane_server:exited', ['lane_server가 준비되기 전에 꺼졌습니다. ' ...
            '"lane_server" 창에 남은 오류 메시지를 확인하세요.']);
    end
    if ~isempty(proc) && proc.HasExited
        error('ensure_lane_server:closed', 'lane_server 창이 준비되기 전에 닫혔습니다. 다시 실행하세요.');
    end
    if toc(t0) > o.timeout
        error('ensure_lane_server:timeout', ['%.0f s 안에 lane_server가 준비되지 않았습니다. ' ...
            '"lane_server" 창의 메시지를 확인하세요.'], o.timeout);
    end
    if toc(t0) - tShown >= 10
        tShown = toc(t0);
        if tShown > 1, fprintf('  lane_server 기다리는 중 %.0f s\n', tShown); end
    end
    pause(0.5);
end
end

function proc = startServerLinux(o, flag)
% Linux: gnome-terminal 새 창 "lane_server"에서 실행. 서버가 꺼져도 창이 남음(exec bash), 꺼지면 flag 파일을 만듦.
% MATLAB이 넣어 둔 LD_LIBRARY_PATH는 지움 (MATLAB 라이브러리가 PyTorch·OpenCV와 섞이면 python이 죽을 수 있음)
% 창 프로세스 핸들은 없으므로 proc = [] (시작 실패는 flag 파일과 timeout으로 알아챔)
proc = [];
q = @(x) ['''' strrep(char(x), '''', '''\''''') ''''];   % bash 작은따옴표 감싸기
run = sprintf('cd %s; unset LD_LIBRARY_PATH; %s %s --port %d --ctrl-port %d', ...
    q(fileparts(o.script)), q(o.python), q(o.script), o.port, o.ctrlPort);
[hasTerm, ~] = system('command -v gnome-terminal');
if hasTerm == 0
    inner = sprintf(['%s; touch %s; echo; ' ...
        'echo ''[lane_server] 서버가 꺼졌습니다. 위 메시지를 확인하세요. 이 창은 닫아도 됩니다.''; exec bash'], run, q(flag));
    cmd = ['env -u LD_LIBRARY_PATH gnome-terminal --title=lane_server -- bash -c ' q(inner) ' >/dev/null 2>&1'];
else
    logf = fullfile(tempdir, sprintf('lane_server_%d.log', o.port));
    inner = sprintf('%s; touch %s', run, q(flag));
    cmd = ['env -u LD_LIBRARY_PATH nohup bash -c ' q(inner) ' > ' q(logf) ' 2>&1 &'];
    fprintf('gnome-terminal이 없어 백그라운드로 켭니다. 출력: %s\n', logf);
end
[st, out] = system(cmd);
if st ~= 0
    error('ensure_lane_server:start', 'lane_server를 켜지 못했습니다: %s', strtrim(out));
end
end
