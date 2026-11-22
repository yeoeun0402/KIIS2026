"""WASD 주행 노드 — 실차 아두이노를 키보드로 직접 몬다 (Unity·모니터 없음).

  ros2 run car_drive wasd_drive                       # 포트 자동
  ros2 run car_drive wasd_drive --ros-args -p port:=/dev/ttyACM0 -p max_speed:=120

실행할 때마다 세션 폴더 하나에 기록한다 (session_log.py, 끄려면 -p record:=false):
  <log_dir>/<YYYYmmdd_HHMMSS>/ drive_log.csv · encoder.csv · serial_raw.log · session.json (+ video.mkv · video_frames.csv)
  log_dir 기본 = $CAR_DRIVE_LOG (run.sh가 <레포>/log로 설정) 또는 ~/car_drive_exp/log

발행:
  /car/pwm            Int16   지금 보내는 속도 명령(-255..255, 음수=후진)
  /car/steer_cmd      String  보낸 조향 명령 L/R/F
  /car/steering_pot   Int32   조향 POT ADC (펌웨어 P 줄, 약 20 Hz)
  /car/wheel_encoder  Int32   뒷바퀴 엔코더 누적 카운트 (펌웨어 E 줄, 약 20 Hz, 1회전 = 80) — 보드 시각은 encoder.csv
  /car/state          Int8    펌웨어 상태 0 정지 / 1 전진 / 2 후진

[twin] -p key_tcp_port:=55004 이면 MATLAB(twin_drive.m)이 TCP로 보내는 키도 받는다 (twin_link.py).
  Unity 에서 누른 W/A/S/D/Space/Q 가 터미널 키와 똑같이 처리됨. 터미널 키보드도 계속 동작 (Space 정지 · Q 종료).
"""
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int8, Int16, Int32, String

from car_drive.drive import KEYS_HELP, Drive
from car_drive.serial_link import BAUD, SerialLink, find_port, parse_line
from car_drive.session_log import SessionLog
from car_drive.twin_link import TwinServer
from car_drive.video_recorder import VideoRecorder, camera_name, find_camera

BOARD_REPLY_TIMEOUT_S = 3.0   # 펌웨어는 50 ms마다 P 줄을 보낸다 — 3초 동안 한 줄도 없으면 통신 실패
KEY_NAMES = {" ": "space", "\x03": "ctrl+c", "\x04": "ctrl+d", "\x1b": "esc", "\n": "enter"}
# [twin] MATLAB 에 APPLY 로 알려 줄 키 (Drive.key 가 처리하는 것만. ctrl+c/d 는 종료라 q 로)
ECHO_NAMES = {"w": "w", "s": "s", "a": "a", "d": "d", "space": "space", "q": "q", "ctrl+c": "q", "ctrl+d": "q"}


class _NullLog:
    dir = None

    def event(self, *_):
        pass

    def raw(self, *_):
        pass

    def flush(self):
        pass

    def close(self, *_):
        return None


class _LinkTap:
    """Drive가 보내는 명령: 시리얼로 먼저 쓰고 → 세션 로그 → 토픽(실패해도 무시)."""

    def __init__(self, link, node):
        self._link, self._node = link, node

    def write(self, text):
        self._link.write(text)          # 차로 가는 명령이 먼저
        cmd = text.strip()
        self._node.log.event("tx", cmd)
        try:
            if cmd.startswith("V"):
                self._node.pub_pwm.publish(Int16(data=int(cmd[1:])))
            elif cmd in ("L", "R", "F"):
                self._node.pub_steer.publish(String(data=cmd))
        except Exception:
            # Ctrl+C면 ROS 컨텍스트가 먼저 닫혀 발행이 실패한다 — 이때도 정지 명령(V0·F·S)은 끝까지 보내야 한다
            pass


class WasdDrive(Node):
    def __init__(self):
        super().__init__("wasd_drive")
        self.declare_parameter("port", "")
        self.declare_parameter("baud", BAUD)
        self.declare_parameter("step", 20)
        self.declare_parameter("max_speed", 150)
        self.declare_parameter("keepalive_hz", 10.0)
        self.declare_parameter("record", True)
        self.declare_parameter("log_dir", os.environ.get("CAR_DRIVE_LOG", "~/car_drive_exp/log"))
        self.declare_parameter("camera", "auto")          # auto = 이름에 C920 · none · /dev/videoN
        self.declare_parameter("video_size", "1280x720")
        self.declare_parameter("video_fps", 30)
        self.declare_parameter("live_jpeg", "")             # [twin] 웹 대시보드용 실시간 JPEG 경로 (비우면 끔)
        self.declare_parameter("key_tcp_port", 0)          # [twin] 0 = 끔 · 55004 = MATLAB twin_drive.m 키 받기
        self.declare_parameter("key_tcp_timeout", 0.5)     # [twin] 연결 뒤 이만큼 아무 줄도 없으면 정지 [s]
        p = {n: self.get_parameter(n).value for n in
             ("port", "baud", "step", "max_speed", "keepalive_hz", "record", "log_dir",
              "camera", "video_size", "video_fps", "key_tcp_port", "key_tcp_timeout", "live_jpeg")}
        self.twin = None

        port = p["port"] or find_port()
        if not port:
            raise RuntimeError("아두이노 포트를 못 찾음 — USB 연결 확인, 또는 -p port:=/dev/ttyACM0")

        self.log = _NullLog()
        if p["record"]:
            self.log = SessionLog(p["log_dir"], repo_root=os.environ.get("CAR_DRIVE_ROOT"),
                                  meta={"port": port, "params": p})
            print(f"● 기록 시작: {self.log.dir}/", flush=True)
        self.video = None
        if p["record"] and p["camera"] != "none":
            self._start_video(p)

        self.get_logger().info(f"{port} 연결 중 (보드 리셋 2초 대기)")
        try:
            self.serial = SerialLink(port, p["baud"])
        except BaseException as e:  # 권한(dialout)·장치 사용 중·연결 대기 중 Ctrl+C — 실패한 실행도 세션에 이유를 남긴다
            if self.video is not None:      # ffmpeg가 혼자 남아 카메라를 붙잡지 않게
                self.log.set_meta("video", self.video.stop())
            meta = self.log.close(f"error: {e!r}" if not isinstance(e, KeyboardInterrupt) else "ctrl+c")
            if meta:
                print(f"● 기록 저장(실패한 실행): {self.log.dir}/", flush=True)
            raise

        self.pub_pwm = self.create_publisher(Int16, "/car/pwm", 10)
        self.pub_steer = self.create_publisher(String, "/car/steer_cmd", 10)
        self.pub_pot = self.create_publisher(Int32, "/car/steering_pot", 10)
        self.pub_enc = self.create_publisher(Int32, "/car/wheel_encoder", 10)
        self.pub_state = self.create_publisher(Int8, "/car/state", 10)

        self._first_rx = threading.Event()
        self._reading = True
        threading.Thread(target=self._read_loop, daemon=True).start()

        self.drive = Drive(_LinkTap(self.serial, self), step=p["step"], max_speed=p["max_speed"])
        self.drive.start()
        self.create_timer(1.0 / float(p["keepalive_hz"]), self.drive.keepalive)
        self.create_timer(1.0, self.log.flush)                  # 비정상 종료에도 1초 전까지는 남게
        self._reply_timer = self.create_timer(BOARD_REPLY_TIMEOUT_S, self._check_board_reply)
        self.get_logger().info(f"포트 열림 · 주행 허용 보냄 · 최고 속도 {self.drive.max_speed} · 보드 응답 확인 중")
        if int(p["key_tcp_port"]) > 0:
            hello = (f"HELLO,step={self.drive.step},max_speed={self.drive.max_speed},"
                     f"session={self.log.dir or ''},t0_epoch={getattr(self.log, 't0_epoch', 0.0):.6f},echo=1")
            self.twin = TwinServer(int(p["key_tcp_port"]), hello=hello, timeout_s=float(p["key_tcp_timeout"]),
                                   on_event=self.log.event)
            print(f"● [twin] MATLAB 키 대기: 127.0.0.1:{self.twin.port} (twin_drive.m) · 터미널 키도 동작", flush=True)

    def _start_video(self, p):
        if p["camera"] == "auto":
            dev, name = find_camera("c920")
        else:
            dev, name = p["camera"], camera_name(p["camera"])
        if dev is None:
            self.log.set_meta("video", {"status": "no_camera"})
            self.get_logger().warn("C920을 못 찾음 → 영상 없이 기록 (다른 카메라: -p camera:=/dev/videoN)")
            return
        self.video = VideoRecorder(self.log.dir, dev, self.log.t0_epoch,
                                   size=p["video_size"], fps=int(p["video_fps"]), live_jpeg=p["live_jpeg"])
        print(f"● 영상 녹화: {dev} ({name}) {p['video_size']}@{p['video_fps']} → video.mkv", flush=True)

    def _check_board_reply(self):
        self._reply_timer.cancel()
        if self.video is not None:
            err = self.video.error_if_dead()
            if err:
                self.log.event("note", "video_failed")
                self.get_logger().error(f"영상 녹화 실패 — 주행·기록은 계속: {err.splitlines()[-1]}")
        if self._first_rx.is_set():
            self.get_logger().info("보드 응답 확인 (펌웨어 줄 수신 중)")
        else:
            self.log.event("note", "no_board_reply")
            self.get_logger().error(
                f"{BOARD_REPLY_TIMEOUT_S:.0f}초 동안 보드 응답 없음 — 펌웨어(./flash.sh)·baud(9600) 확인. 차는 움직이지 않을 수 있음")

    def _read_loop(self):
        while self._reading:
            try:
                line = self.serial.readline()
            except Exception as e:   # USB 뽑힘 등
                if self._reading:
                    self.log.event("note", f"serial_error: {e}")
                    self.get_logger().error(f"시리얼 읽기 실패: {e}")
                return
            if not line:
                continue
            self._first_rx.set()
            self.log.raw(line)
            parsed = parse_line(line)
            if parsed is None:
                continue
            kind, value = parsed
            self.log.event(kind, value)
            try:
                if kind == "pot":
                    self.pub_pot.publish(Int32(data=value))
                elif kind == "enc":
                    self.pub_enc.publish(Int32(data=value[0]))
                else:
                    self.pub_state.publish(Int8(data=value))
            except Exception:
                pass

    def shutdown(self):
        self._reading = False
        try:
            self.drive.stop()
        finally:
            self.serial.close()


def _keyboard_loop(node):
    """종료 이유를 돌려준다: q · ctrl+c · eof (+ twin: MATLAB 이 보낸 q 는 "twin:q")."""
    use_tty = sys.stdin.isatty()
    fd = sys.stdin.fileno() if use_tty else None
    old = termios.tcgetattr(fd) if use_tty else None
    twin = node.twin
    print(KEYS_HELP, flush=True)
    last = None
    try:
        if use_tty:
            tty.setcbreak(fd)
        while rclpy.ok():
            watch = ([sys.stdin] if use_tty else []) + (twin.fileno_list() if twin else [])
            ready, _, _ = select.select(watch, [], [], 0.1)
            keys = []
            if use_tty and sys.stdin in ready:
                k = sys.stdin.read(1)
                if not k:
                    return "eof"
                keys.append(("tty", k))
            if twin is not None:
                keys += [("twin", k) for k in twin.poll()]   # 타임아웃 검사 때문에 매번 부름
            for src, k in keys:
                name = KEY_NAMES.get(k, k)
                node.log.event("key", name if src == "tty" else f"twin:{name}")
                ok = node.drive.key(k)
                if twin is not None and name in ECHO_NAMES:   # 시뮬이 실차와 같은 입력을 받도록 적용한 키를 알려 줌
                    twin.send(f"APPLY,{ECHO_NAMES[name]},{node.drive.speed}")
                if not ok:
                    return name if src == "tty" else f"twin:{name}"
            if node.drive.speed != last:
                last = node.drive.speed
                print(f"\r속도 {last:+4d}   ", end="", flush=True)
        return "ctrl+c"
    finally:
        if use_tty:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if twin is not None:
            twin.close()
        print()


def _print_summary(meta):
    if not meta:
        return
    s = meta.get("summary", {})
    pot = f"{s.get('pot_min')}~{s.get('pot_max')}" if s.get("pot_samples") else "없음"
    reply = f"{s['first_board_reply_s']:.1f}초에 첫 응답" if s.get("first_board_reply_s") is not None else "응답 없음 ⚠"
    if not s.get("enc_samples"):
        enc = "없음 (펌웨어에 E 줄 없음 → ./flash.sh)"
    elif s.get("enc_min") == s.get("enc_max"):
        enc = f"{s['enc_samples']}개 · 카운트 변화 없음 (주행했는데 0이면 배선 D18/D19 확인)"
    else:
        enc = (f"{s['enc_samples']}개 · 순 {s['enc_net']:+d} 카운트 ({s['enc_net'] / 80:+.1f}회전) "
               f"· 범위 {s['enc_min']}~{s['enc_max']}")
    if s.get("enc_board_resets"):
        enc += f" · 보드 리셋 {s['enc_board_resets']}번 ⚠"
    print(f"● 기록 저장: {meta['_dir']}/\n"
          f"  주행 {s.get('duration_s', 0):.0f}초 · 키 {s.get('keys')}번 · 최고 |PWM| {s.get('max_abs_pwm')} · "
          f"조향 POT {pot} ({s.get('pot_samples')}개) · 보드 {reply} · 종료 {meta.get('end_reason')}\n"
          f"  바퀴 엔코더: {enc}\n"
          f"  파일: drive_log.csv · encoder.csv · serial_raw.log · session.json", flush=True)
    v = meta.get("video")
    if not v:
        return
    if v.get("frames"):
        print(f"  영상: {v.get('file')} {v['frames']}프레임 · {v.get('fps_measured')} fps · {v.get('size_mb')} MB "
              f"· 첫 프레임 t={v.get('first_frame_t_s')}s · 프레임 시각 {v.get('frames_csv')}", flush=True)
    elif v.get("status") == "no_camera":
        print("  영상: 없음 (C920 미연결)", flush=True)
    else:
        print(f"  영상: 실패 ⚠ {'; '.join(v.get('ffmpeg_errors', []))}", flush=True)


def main(args=None):
    twin_mode = any("key_tcp_port:=" in a and not a.endswith(":=0") for a in (args or sys.argv))
    if not sys.stdin.isatty() and not twin_mode:
        print("키보드 입력이 필요함 — 터미널에서 직접 실행 (./run.sh)", file=sys.stderr)
        return 1
    rclpy.init(args=args)
    node, executor, spin, code, reason = None, None, None, 0, "ctrl+c"
    try:
        node = WasdDrive()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin = threading.Thread(target=_spin, args=(executor,), daemon=True)
        spin.start()
        reason = _keyboard_loop(node)
    except KeyboardInterrupt:
        reason = "ctrl+c"
    except Exception as e:      # 포트 없음 · 권한(dialout) · 장치 사용 중
        print(f"[wasd_drive] {e}", file=sys.stderr)
        code, reason = 1, f"error: {e}"
    finally:
        # 순서: spin(keepalive 타이머) 정지 → 정지 명령 → 시리얼 닫기 → 기록 닫기 → 노드 정리.
        # spin이 도는 채로 노드를 없애면 C++ 쪽이 abort하고, 정지 뒤에 keepalive가 끼어들 수 있다.
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if spin is not None:
            spin.join(timeout=2.0)
        if node is not None:
            if hasattr(node, "serial"):
                node.shutdown()     # V0 · F · S — 차를 세우고 주행 게이트를 닫는다
                print("정지 명령 보냄 · 종료")
            if getattr(node, "video", None) is not None:
                node.log.set_meta("video", node.video.stop())
            meta = node.log.close(reason)
            if meta:
                _print_summary(dict(meta, _dir=node.log.dir))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


def _spin(executor):
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    sys.exit(main())
