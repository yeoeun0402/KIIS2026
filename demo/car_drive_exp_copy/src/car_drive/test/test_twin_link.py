"""[twin] TwinServer: MATLAB → 실차 TCP 키 전달 (실제 소켓, ROS·실차 없이)."""
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from car_drive.twin_link import TwinServer  # noqa: E402


def _poll_until(srv, cond, t=1.0):
    out, end = [], time.monotonic() + t
    while time.monotonic() < end:
        out += srv.poll()
        if cond(out):
            break
        time.sleep(0.01)
    return out


def test_hello_keys_timeout_disconnect():
    events = []
    srv = TwinServer(0, hello="HELLO,step=10,max_speed=60", timeout_s=0.3,
                     on_event=lambda k, v: events.append(v))
    try:
        assert srv.poll() == []                       # 연결 전: 아무것도 안 함
        c = socket.create_connection(("127.0.0.1", srv.port))
        c.settimeout(1.0)
        srv.poll()
        assert c.recv(100).decode().startswith("HELLO,step=10")
        c.sendall(b"KEY,w\nKEY,a\nHB\nKEY,space\nKEY,bogus\nKEY,q\n")
        keys = _poll_until(srv, lambda o: len(o) >= 4)
        assert keys == ["w", "a", " ", "q"]
        # 0.3 s 동안 아무것도 안 오면 정지(' ') 한 번
        keys = _poll_until(srv, lambda o: " " in o, t=1.0)
        assert keys == [" "] and any("timeout" in e for e in events)
        assert _poll_until(srv, lambda o: o, t=0.4) == []   # 정지는 한 번만
        # 다시 오면 정상, 끊기면 정지
        c.sendall(b"HB\n")
        assert _poll_until(srv, lambda o: o, t=0.1) == []
        c.close()
        keys = _poll_until(srv, lambda o: o, t=1.0)
        assert keys == [" "] and not srv.connected and any("disconnected" in e for e in events)
    finally:
        srv.close()


def test_send_apply_echo():
    """차가 적용한 키를 MATLAB 에 APPLY 줄로 알려 줌 (연결 없으면 조용히 무시)."""
    srv = TwinServer(0, hello="HELLO,echo=1", timeout_s=5.0)
    try:
        srv.send("APPLY,w,10")                        # 연결 전: 예외 없이 무시
        c = socket.create_connection(("127.0.0.1", srv.port))
        c.settimeout(1.0)
        _poll_until(srv, lambda o: srv.connected, t=0.5)
        srv.send("APPLY,w,10")
        srv.send("APPLY,space,0")
        buf = b""
        end = time.monotonic() + 1.0
        while buf.count(b"\n") < 3 and time.monotonic() < end:
            buf += c.recv(200)
        assert buf.decode().split("\n")[:3] == ["HELLO,echo=1", "APPLY,w,10", "APPLY,space,0"]
        c.close()
    finally:
        srv.close()
