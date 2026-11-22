"""[twin] 웹 대시보드 서버 — 실차 카메라 · Unity 시뮬 화면 · 무게 조절 · 지도(시뮬 빨강 / localization 노랑)

  python3 -B twin_dash.py            → http://127.0.0.1:8765  (twin.sh 가 자동으로 켜고 브라우저를 엶)
  옵션 --port 8765  --matlab-port 55005  --live-dir /dev/shm/twin_live  --mass 15  --no-loc

화면 (index.html)
  위 왼쪽  실차 카메라   : 주행 노드 ffmpeg 가 덮어쓰는 <live-dir>/cam.jpg (drive.sh --twin, 15 fps)
  위 오른쪽 Unity 시뮬   : TwinStream.cs 가 덮어쓰는 <live-dir>/sim.jpg (10 fps)
  아래 왼쪽 무게 조절    : 바꾸면 MATLAB(twin_drive)에 MASS,<kg> → twin_mass.m 이 tauV·pwmDead 를 다시 계산 (주행 중에도)
  아래 오른쪽 지도       : 매트 16 x 11 m · 2차선 · 시뮬 경로(빨강, MATLAB SIM 줄) · localization 경로(노랑, live_loc.py)
  키보드: 이 페이지에서 W/A/S/D/Space/Q 도 Unity Game 창과 똑같이 동작 (MATLAB 에 KEY,<k>)

연결
  MATLAB → (TCP 55005, 이 서버가 기다림)  RUN,<run>,<mass>,<session> · PARAM,... · SIM,t,epoch,x,y,θ,v,pwm,δ,mass,pwmDead,tauV
  이 서버 → MATLAB                         MASS,<kg> (연결 직후 + 바꿀 때) · KEY,<k>
  live_loc.py (자식 프로세스, stdout)       LOC,epoch,x,y,θ,n,rms · KF,epoch,x,y,θ,v,ω,P… · RESET,epoch  (LOC·KF 는 MATLAB 으로 전달)
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
TRACK = os.path.expanduser("~/demo/track")
MAX_PATH = 200000


class State:
    def __init__(self, mass):
        self.lock = threading.Lock()
        self.mass = float(mass)
        self.param = None                # MATLAB 모델 값 (kv, massRef, crr, tauVRef, pwmDeadRef, maxSpeed, step)
        self.run = ""
        self.session = ""
        self.sim = None                  # 마지막 SIM
        self.loc = None                  # 마지막 LOC
        self.markers = None              # live_loc 이 쓰는 마커 세트 이름 (track.yaml markers.use / MARKERS)
        self.sim_path = []               # [x, y]
        self.loc_path = []               # [x, y, n]
        self.sim_gen = 0                 # 경로를 지우면 +1 (페이지가 처음부터 다시 받게)
        self.loc_gen = 0
        self.matlab = None               # 연결된 MATLAB 소켓
        self.loc_status = "꺼짐"
        self.obs_on = True               # 루엔버그 옵저버 (MATLAB 에 OBS,1/0)
        self.log = []

    def note(self, msg):
        with self.lock:
            self.log.append(f"{time.strftime('%H:%M:%S')} {msg}")
            self.log = self.log[-30:]
        print(f"[dash] {msg}", flush=True)


S = None


# ── MATLAB 연결 (TCP) ────────────────────────────────────────────
def send_matlab(line):
    with S.lock:
        c = S.matlab
    if c is None:
        return False
    try:
        c.sendall((line + "\n").encode())
        return True
    except OSError:
        return False


def matlab_server(port):
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    while True:
        c, _ = srv.accept()
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with S.lock:
            old, S.matlab = S.matlab, c
        if old:
            try:
                old.close()
            except OSError:
                pass
        S.note("MATLAB 연결됨")
        send_matlab(f"MASS,{S.mass:.2f}")
        send_matlab(f"OBS,{int(S.obs_on)}")
        buf = b""
        try:
            while True:
                d = c.recv(65536)
                if not d:
                    break
                buf += d
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    handle_matlab(raw.decode(errors="replace").strip())
        except OSError:
            pass
        with S.lock:
            if S.matlab is c:
                S.matlab = None
        S.note("MATLAB 연결 끊김")


def handle_matlab(line):
    a = line.split(",")
    if a[0] == "SIM" and len(a) >= 12:
        v = [float(x) for x in a[1:14]]
        d = dict(zip(["t", "epoch", "x", "y", "th", "v", "pwm", "delta", "mass", "pwmDead", "tauV", "obs", "vs"], v))
        with S.lock:
            S.sim = d
            p = S.sim_path
            if not p or abs(p[-1][0] - d["x"]) + abs(p[-1][1] - d["y"]) > 0.005:
                if len(p) < MAX_PATH:
                    p.append([round(d["x"], 4), round(d["y"], 4)])
    elif a[0] == "RUN":
        with S.lock:
            S.run = a[1] if len(a) > 1 else ""
            S.session = a[3] if len(a) > 3 else ""
            S.sim_path, S.sim, S.sim_gen = [], None, S.sim_gen + 1
        S.note(f"주행 시작 {S.run}")
        send_matlab(f"MASS,{S.mass:.2f}")
        send_matlab(f"OBS,{int(S.obs_on)}")
    elif a[0] == "PARAM" and len(a) >= 8:
        k = ["kv", "massRef", "crr", "tauVRef", "pwmDeadRef", "maxSpeed", "step"]
        with S.lock:
            S.param = dict(zip(k, [float(x) for x in a[1:8]]))


# ── live_loc.py (자식 프로세스) ──────────────────────────────────
def loc_worker(live_dir):
    cmd = [sys.executable, "-B", os.path.join(HERE, "live_loc.py"), "--jpeg", os.path.join(live_dir, "cam.jpg")]
    while True:
        S.loc_status = "시작 중"
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in p.stdout:
            a = line.strip().split(",")
            if a[0] == "LOC" and len(a) >= 7:
                t, x, y, th, n, rms = float(a[1]), float(a[2]), float(a[3]), float(a[4]), int(a[5]), float(a[6])
                send_matlab(line.strip())                        # 루엔버그 옵저버 측정 (MATLAB twin_observer.m)
                with S.lock:
                    S.loc = {"epoch": t, "x": x, "y": y, "th": th, "n": n, "rms": rms}
                    if len(S.loc_path) < MAX_PATH:
                        S.loc_path.append([round(x, 4), round(y, 4), n])
                S.loc_status = "동작 중"
            elif a[0] == "KF":                                  # ArUco 측정 칼만 필터 결과 (loc_kf.py) → MATLAB → Unity 그림·기록
                send_matlab(line.strip())
            elif a[0] == "RESET":
                with S.lock:
                    S.loc_path, S.loc, S.loc_gen = [], None, S.loc_gen + 1
                S.note("localization: 새 주행 (출발 자세에서 다시)")
            elif a[0] == "MARKERS" and len(a) >= 4:
                S.markers = a[1]
                S.note(f"마커 세트: {a[1]} ({a[2]}, {float(a[3]) * 100:.1f} cm)")
            elif a[0] == "READY":
                S.loc_status = "카메라 기다리는 중"
                S.note(f"localization 지도: {a[1] if len(a) > 1 else ''}")
            elif a[0] == "ERR":
                S.note("localization 오류: " + ",".join(a[2:])[:200])
        p.wait()
        S.loc_status = f"꺼짐 (코드 {p.returncode}) → 5초 뒤 다시"
        time.sleep(5)


# ── 트랙 (지도 그리기용) ─────────────────────────────────────────
def track_json():
    import csv
    cl = []
    with open(os.path.join(TRACK, "lane2_centerline.csv")) as f:
        for r in csv.DictReader(f):
            cl.append([float(r["x_m"]), float(r["y_m"]), float(r["heading_deg"])])
    markers = []
    mp = None
    try:
        sys.path.insert(0, HERE)
        from live_loc import latest_map
        mp = latest_map()
        if mp:
            with open(os.path.join(mp, "markers.csv")) as f:
                for r in csv.DictReader(f):
                    markers.append([int(r["id"]), float(r["x_m"]), float(r["y_m"])])
    except Exception:
        pass
    return {"mat": [16.0, 11.0], "centerline": cl, "lane_width": 0.85,
            "start": [1.265, 6.659, -1.5707963], "markers": markers, "map_dir": mp}


# ── HTTP ─────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    live_dir = "/dev/shm/twin_live"
    track = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if u.path == "/track":
            if Handler.track is None:
                Handler.track = track_json()
            return self._send(200, Handler.track)
        if u.path == "/state":
            q = parse_qs(u.query)
            si, li = int(q.get("si", [0])[0]), int(q.get("li", [0])[0])
            sg, lg = int(q.get("sg", [-1])[0]), int(q.get("lg", [-1])[0])
            now = time.time()
            with S.lock:
                if sg != S.sim_gen:
                    si = 0
                if lg != S.loc_gen:
                    li = 0
                body = {"mass": S.mass, "obs_on": S.obs_on, "markers": S.markers, "param": S.param, "run": S.run, "session": S.session,
                        "sim": S.sim, "loc": S.loc, "matlab": S.matlab is not None, "loc_status": S.loc_status,
                        "sg": S.sim_gen, "lg": S.loc_gen, "si": len(S.sim_path), "li": len(S.loc_path),
                        "sim_new": S.sim_path[si:], "loc_new": S.loc_path[li:], "log": S.log[-8:],
                        "cam_age": file_age(os.path.join(self.live_dir, "cam.jpg"), now),
                        "sim_age": file_age(os.path.join(self.live_dir, "sim.jpg"), now)}
            return self._send(200, body)
        if u.path in ("/cam.mjpg", "/sim.mjpg"):
            return self.mjpeg(os.path.join(self.live_dir, u.path[1:4] + ".jpg"))
        if u.path in ("/cam.jpg", "/sim.jpg"):
            try:
                with open(os.path.join(self.live_dir, u.path[1:]), "rb") as f:
                    return self._send(200, f.read(), "image/jpeg")
            except OSError:
                return self._send(404, {"error": "no frame"})
        return self._send(404, {"error": "not found"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._send(400, {"error": "json"})
        if self.path == "/mass":
            try:
                m = float(body.get("mass"))
            except (TypeError, ValueError):
                return self._send(400, {"error": "mass"})
            m = min(max(m, 1.0), 60.0)
            with S.lock:
                S.mass = m
            ok = send_matlab(f"MASS,{m:.2f}")
            S.note(f"무게 {m:.1f} kg" + ("" if ok else " (MATLAB 연결되면 보냄)"))
            return self._send(200, {"mass": m, "sent": ok})
        if self.path == "/key":
            k = str(body.get("key", "")).lower()
            if k not in ("w", "s", "a", "d", "space", "q"):
                return self._send(400, {"error": "key"})
            return self._send(200, {"sent": send_matlab(f"KEY,{k}")})
        if self.path == "/obs":
            on = bool(body.get("on"))
            with S.lock:
                S.obs_on = on
            ok = send_matlab(f"OBS,{int(on)}")
            S.note(f"옵저버 {'켬' if on else '끔'}" + ("" if ok else " (MATLAB 연결되면 보냄)"))
            return self._send(200, {"on": on, "sent": ok})
        if self.path == "/clear":
            with S.lock:
                S.sim_path, S.loc_path = [], []
                S.sim_gen += 1
                S.loc_gen += 1
            return self._send(200, {"ok": True})
        return self._send(404, {"error": "not found"})

    def mjpeg(self, path):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = None
        try:
            while True:
                try:
                    st = os.stat(path)
                    key = (st.st_ino, st.st_mtime_ns)
                except FileNotFoundError:
                    key = None
                if key and key != last:
                    try:
                        with open(path, "rb") as f:
                            data = f.read()
                    except OSError:
                        data = b""
                    if data:
                        last = key
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                        self.wfile.flush()
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


def file_age(path, now):
    try:
        return round(now - os.stat(path).st_mtime, 2)
    except OSError:
        return None


def main():
    global S
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--matlab-port", type=int, default=55005)
    ap.add_argument("--live-dir", default="/dev/shm/twin_live")
    ap.add_argument("--mass", type=float, default=15.0)
    ap.add_argument("--no-loc", action="store_true", help="실시간 localization 끄기")
    a = ap.parse_args()

    os.makedirs(a.live_dir, exist_ok=True)       # 이 폴더가 있어야 Unity TwinStream 이 sim.jpg 를 씀
    S = State(a.mass)
    Handler.live_dir = a.live_dir
    threading.Thread(target=matlab_server, args=(a.matlab_port,), daemon=True).start()
    if not a.no_loc:
        threading.Thread(target=loc_worker, args=(a.live_dir,), daemon=True).start()
    httpd = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    httpd.daemon_threads = True
    S.note(f"대시보드: http://127.0.0.1:{a.port}  (MATLAB {a.matlab_port}, 화면 폴더 {a.live_dir}, 무게 {a.mass} kg)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
