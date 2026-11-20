function out = unity_link(action, varargin)
% Unity(CarStateReceiver)와 TCP로 주고받는 함수 모음
%   client = unity_link('connect', p)              연결 (실패하면 [] 반환)
%   ok = unity_link('state', client, st, e, dbg)   상태 한 줄 전송 (pure pursuit: 21개)
%   ok = unity_link('state', client, st)           상태 한 줄 전송 (manual_drive: 앞 9개만, 오차 칸은 0)
%   ok = unity_link('path',  client, ref, step)    레퍼런스 경로 전송 (표시용, step[m] 간격)
%   ok = unity_link('reset', client)               Unity의 궤적 지우기
%
% 상태 한 줄 (쉼표 구분):
%   t,x,y,yaw,v,delta,yawRate,e_lat,e_head,
%   x_w,y_w,x_d,y_d,x_l,y_l,x_c,y_c,R,Lf,tau,delta_cmd   (없는 값은 NaN)

out = false;
switch action
    case 'connect'
        p = varargin{1};
        try
            out = tcpclient(p.unityIP, p.unityPort, "ConnectTimeout", 3);
        catch
            warning("Unity에 연결할 수 없습니다 (%s:%d). Unity Play를 먼저 눌렀는지 확인하세요.", ...
                p.unityIP, p.unityPort);
            out = [];
        end

    case 'state'
        [client, st] = varargin{1:2};
        msg = [st.t, st.x, st.y, st.yaw, st.v, st.delta, st.yawRate];
        if numel(varargin) >= 4
            [e, dbg] = varargin{3:4};
            msg = [msg, e.e_lat, e.e_head, ...
                   dbg.x_w, dbg.y_w, dbg.x_d, dbg.y_d, dbg.x_l, dbg.y_l, ...
                   dbg.x_c, dbg.y_c, dbg.R, dbg.Lf, dbg.tau, dbg.delta_cmd];
        else
            msg = [msg, 0, 0];
        end
        msg(~isfinite(msg)) = NaN;          % Unity는 Inf를 못 읽으므로 NaN으로
        line = sprintf('%.6f,', msg); line(end) = [];
        out = sendLine(client, line);

    case 'path'
        [client, ref, step] = varargin{:};
        k = unique([1:max(1, round(step / ref.ds)):ref.N, ref.N]);
        xy = [ref.x(k), ref.y(k)]';
        line = ['PATH,', sprintf('%d', numel(k)), sprintf(',%.4f', xy(:))];
        out = sendLine(client, line);

    case 'reset'
        out = sendLine(varargin{1}, 'RESET');
end
end

function ok = sendLine(client, line)
ok = false;
if isempty(client), return; end
try
    writeline(client, line);
    ok = true;
catch
end
end
