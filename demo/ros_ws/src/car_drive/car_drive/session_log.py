"""세션별 주행 로그 — 로그 폴더 아래 세션마다 폴더 하나.

  <log>/<YYYYmmdd_HHMMSS>/drive_log.csv    t_s,kind,value   (kind: key · tx · pot · enc · state · note)
  <log>/<YYYYmmdd_HHMMSS>/encoder.csv      t_s,board_ms,count  뒷바퀴 엔코더 (펌웨어 E 줄, 약 20 Hz)
                                           board_ms = 카운트를 읽은 보드 시각 → 속도는 이걸로 나누는 게 정확
  <log>/<YYYYmmdd_HHMMSS>/serial_raw.log   보드에서 받은 원문 줄 (t_s<TAB>줄) — 펌웨어가 바뀌어도 다시 해석
  <log>/<YYYYmmdd_HHMMSS>/session.json     시작/끝 시각·포트·설정·git 커밋·펌웨어 해시·요약 (시작 때, 끝날 때 갱신)

t_s = 세션 시작부터의 초(단조 시계). 여러 스레드(키보드·시리얼 읽기·keepalive 타이머)가 같이 쓰므로 잠금.
ROS 없이 동작한다(테스트용).
"""
import csv
import datetime as _dt
import hashlib
import json
import os
import socket
import subprocess
import threading
import time


def _git_commit(root):
    if not root:
        return None
    try:
        out = subprocess.run(["git", "-C", root, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=2)
        dirty = subprocess.run(["git", "-C", root, "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True, timeout=2).stdout.strip()
        return out.stdout.strip() + ("+dirty" if dirty else "") if out.returncode == 0 else None
    except Exception:
        return None


def _file_sha(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()[:12]
    except OSError:
        return None


class SessionLog:
    def __init__(self, log_dir, meta=None, repo_root=None, now=None):
        start = now or _dt.datetime.now()
        self.root = os.path.expanduser(log_dir)
        os.makedirs(self.root, exist_ok=True)
        base = start.strftime("%Y%m%d_%H%M%S")
        name, n = base, 1
        while True:
            try:
                os.mkdir(os.path.join(self.root, name))   # 같은 초에 두 번 실행하면 <시각>-2
                break
            except FileExistsError:
                n += 1
                name = f"{base}-{n}"
        self.name = name
        self.dir = os.path.join(self.root, name)          # 이 세션 폴더
        self._t0 = time.monotonic()
        self.t0_epoch = time.time()     # t_s=0 인 순간의 벽시계 — 영상 프레임(벽시계 기록)을 t_s로 바꿀 때 쓴다
        self._lock = threading.Lock()
        self._csv_f = open(self.path("drive_log.csv"), "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_f, lineterminator="\n")   # 기본 \r\n이면 grep '…$'가 안 맞는다
        self._csv.writerow(["t_s", "kind", "value"])
        self._raw_f = open(self.path("serial_raw.log"), "w", encoding="utf-8")
        self._enc_f = open(self.path("encoder.csv"), "w", newline="", encoding="utf-8")
        self._enc = csv.writer(self._enc_f, lineterminator="\n")
        self._enc.writerow(["t_s", "board_ms", "count"])
        self._closed = False
        # 요약용 집계
        self.counts = {"key": 0, "tx": 0, "pot": 0, "enc": 0, "state": 0, "raw": 0}
        self.max_abs_pwm = 0
        self.pot_min = None
        self.pot_max = None
        self.enc = {"first": None, "last": None, "min": None, "max": None,
                    "last_board_ms": None, "max_gap_ms": None, "board_resets": 0}
        self.first_rx_s = None
        fw = os.path.join(repo_root, "firmware", "car_controller", "car_controller.ino") if repo_root else None
        self.meta = {
            "session": name,
            "start": start.isoformat(timespec="seconds"),
            "t0_epoch": round(self.t0_epoch, 6),
            "end": None,
            "host": socket.gethostname(),
            "git_commit": _git_commit(repo_root),
            "firmware_sha256_12": _file_sha(fw) if fw else None,
            **(meta or {}),
        }
        self._write_json()

    def path(self, suffix):
        """이 세션 파일 경로: <log>/<세션>/<suffix>"""
        return os.path.join(self.dir, suffix)

    def t(self):
        return round(time.monotonic() - self._t0, 3)

    def event(self, kind, value):
        """kind="enc" 이면 value = (count, board_ms) — drive_log.csv엔 count만, encoder.csv엔 둘 다."""
        with self._lock:
            if self._closed:
                return
            t = f"{self.t():.3f}"
            if kind == "enc":
                count, board_ms = (int(x) for x in value)
                self._enc.writerow([t, board_ms, count])
                self._note_enc(count, board_ms)
                value = count
            self._csv.writerow([t, kind, value])
            if kind in self.counts:
                self.counts[kind] += 1
            if kind == "tx" and str(value).startswith("V"):
                try:
                    self.max_abs_pwm = max(self.max_abs_pwm, abs(int(str(value)[1:])))
                except ValueError:
                    pass
            elif kind == "pot":
                v = int(value)
                self.pot_min = v if self.pot_min is None else min(self.pot_min, v)
                self.pot_max = v if self.pot_max is None else max(self.pot_max, v)

    def _note_enc(self, count, board_ms):
        e = self.enc
        if e["first"] is None:
            e["first"] = e["min"] = e["max"] = count
        e["min"], e["max"], e["last"] = min(e["min"], count), max(e["max"], count), count
        prev = e["last_board_ms"]
        if prev is not None:
            if board_ms < prev:                 # 보드 리셋(millis 0부터) — 카운트도 0부터 다시 센다
                e["board_resets"] += 1
            else:
                gap = board_ms - prev           # 50 ms 근처여야 함. 크면 줄이 빠진 것
                e["max_gap_ms"] = gap if e["max_gap_ms"] is None else max(e["max_gap_ms"], gap)
        e["last_board_ms"] = board_ms

    def raw(self, line):
        with self._lock:
            if self._closed:
                return
            t = self.t()
            if self.first_rx_s is None:
                self.first_rx_s = t
            self.counts["raw"] += 1
            self._raw_f.write(f"{t:.3f}\t{line}\n")

    def summary(self):
        return {
            "duration_s": self.t(),
            "first_board_reply_s": self.first_rx_s,
            "keys": self.counts["key"],
            "commands_sent": self.counts["tx"],
            "board_lines": self.counts["raw"],
            "pot_samples": self.counts["pot"],
            "pot_min": self.pot_min,
            "pot_max": self.pot_max,
            "enc_samples": self.counts["enc"],
            "enc_first": self.enc["first"],
            "enc_last": self.enc["last"],
            "enc_min": self.enc["min"],
            "enc_max": self.enc["max"],
            "enc_net": (self.enc["last"] - self.enc["first"]) if self.enc["first"] is not None else None,
            "enc_max_gap_ms": self.enc["max_gap_ms"],
            "enc_board_resets": self.enc["board_resets"],
            "max_abs_pwm": self.max_abs_pwm,
        }

    def close(self, end_reason="quit"):
        with self._lock:
            if self._closed:
                return self.meta
            self._closed = True
            self.meta["end"] = _dt.datetime.now().isoformat(timespec="seconds")
            self.meta["end_reason"] = end_reason
            self.meta["summary"] = self.summary()
            for f in (self._csv_f, self._raw_f, self._enc_f):
                try:
                    f.close()
                except Exception:
                    pass
        self._write_json()
        return self.meta

    def set_meta(self, key, value):
        """session.json에 항목 추가(영상 정보 등) — 닫기 전이면 close()가 같이 저장한다."""
        with self._lock:
            self.meta[key] = value
            closed = self._closed
        if closed:
            self._write_json()

    def flush(self):
        with self._lock:
            if not self._closed:
                self._csv_f.flush()
                self._raw_f.flush()
                self._enc_f.flush()

    def _write_json(self):
        tmp = self.path("session.json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.path("session.json"))
