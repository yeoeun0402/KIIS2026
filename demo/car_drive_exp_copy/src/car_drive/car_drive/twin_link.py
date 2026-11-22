"""[twin] MATLAB(twin_drive.m) → 실차: TCP로 받은 키를 터미널 키처럼 Drive 에 넘긴다 (2026-11-21).

Unity Game 창 키 → MATLAB → 이 서버(127.0.0.1:key_tcp_port, 기본 55004) → Drive.key() → 아두이노.
터미널 키보드도 그대로 동작한다 (Space 정지 · Q 종료는 언제나 터미널에서도 가능).

받는 줄 (한 줄 = 한 명령, ASCII)
  KEY,w | KEY,s | KEY,a | KEY,d | KEY,space | KEY,q     터미널의 w/s/a/d/Space/q 와 같음
  HB                                                     살아 있음 신호 (MATLAB 이 매 주기 보냄)
보내는 줄
  HELLO,step=<n>,max_speed=<n>,session=<세션 폴더>,t0_epoch=<s>,echo=1   연결 직후 한 번 (MATLAB 이 시뮬 모델을 같은 값으로 맞춤)
  APPLY,<key>,<speed>      차가 키를 실제로 적용할 때마다 (MATLAB 키 · 터미널 키 · 안전 정지 모두).
                           key = w|s|a|d|space|q, speed = 적용 뒤 PWM. MATLAB 시뮬은 이 줄만 보고 움직인다
                           → 시뮬과 실차의 속도 조절이 같은 값 하나(Drive.speed)를 따른다 (echo=1, 2026-11-21)

안전
  * 연결된 뒤 timeout_s(기본 0.5 s) 동안 아무 줄도 안 오면 → 정지(Space 와 같음). MATLAB·Unity 가 멈춰도 차가 계속 가지 않게.
    (펌웨어 워치독 500 ms 는 노드가 keepalive 를 보내는 한 걸리지 않으므로, 여기서 따로 막는다)
  * 연결이 끊기면 → 정지. 다시 연결을 기다린다 (터미널 Q 로 끝낼 수 있음).
  * 연결 전에는 차에 아무 명령도 안 보낸다 (속도 0 그대로).
"""
import socket
import time

KEY_MAP = {"w": "w", "s": "s", "a": "a", "d": "d", "space": " ", "q": "q"}


class TwinServer:
    def __init__(self, port, hello="", host="127.0.0.1", timeout_s=0.5, on_event=None):
        self.timeout_s = float(timeout_s)
        self.hello = hello
        self.on_event = on_event or (lambda kind, value: None)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, int(port)))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.port = self.srv.getsockname()[1]
        self.cli = None
        self.buf = b""
        self.last_rx = 0.0
        self.timed_out = False

    @property
    def connected(self):
        return self.cli is not None

    def fileno_list(self):
        """select() 에 넣을 소켓들."""
        return [self.srv] + ([self.cli] if self.cli else [])

    def poll(self, now=None):
        """받은 키 목록(터미널 글자: 'w','s','a','d',' ','q')을 돌려준다. 정지가 필요하면 ' ' 를 끼워 넣는다."""
        now = time.monotonic() if now is None else now
        keys = []
        if self.cli is None:
            try:
                self.cli, addr = self.srv.accept()
                self.cli.setblocking(False)
                self.buf, self.last_rx, self.timed_out = b"", now, False
                self.on_event("twin", f"connected {addr[0]}:{addr[1]}")
                if self.hello:
                    self.cli.sendall((self.hello + "\n").encode("ascii", "replace"))
            except (BlockingIOError, InterruptedError):
                return keys
        try:
            while True:
                chunk = self.cli.recv(4096)
                if not chunk:
                    return keys + self._drop("disconnected")
                self.buf += chunk
        except (BlockingIOError, InterruptedError):
            pass
        except OSError as e:
            return keys + self._drop(f"socket_error {e}")
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            self.last_rx, self.timed_out = now, False
            if line.startswith("KEY,"):
                k = KEY_MAP.get(line[4:].strip().lower())
                if k is not None:
                    keys.append(k)
            # HB 와 모르는 줄은 살아 있음 표시로만 씀
        if not self.timed_out and now - self.last_rx > self.timeout_s:
            self.timed_out = True
            self.on_event("twin", f"timeout {now - self.last_rx:.2f}s -> stop")
            keys.append(" ")
        return keys

    def send(self, line):
        """MATLAB 에 한 줄 보내기 (연결 없으면 무시). 짧은 줄이라 non-blocking 소켓에서도 한 번에 나간다."""
        if self.cli is None:
            return
        try:
            self.cli.sendall((line + "\n").encode("ascii", "replace"))
        except (BlockingIOError, InterruptedError):
            pass
        except OSError as e:
            self._drop(f"send_error {e}")

    def _drop(self, why):
        self.on_event("twin", why + " -> stop")
        try:
            self.cli.close()
        except OSError:
            pass
        self.cli = None
        return [" "]

    def close(self):
        for s in (self.cli, self.srv):
            try:
                if s:
                    s.close()
            except OSError:
                pass
