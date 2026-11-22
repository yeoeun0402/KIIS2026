"""실차 아두이노 메가(firmware/car_controller, 9600 bps)와의 시리얼 연결.

포트 찾기는 pyserial 없이도 되게(flash.sh가 이 함수만 빌려 쓴다) glob으로 한다.
"""
import glob
import os
import threading
import time

BAUD = 9600
BOOT_WAIT_S = 2.0   # 메가는 포트를 여는 순간 리셋된다 → 부트로더가 끝날 때까지 기다린다


def find_port():
    """아두이노 포트 경로. 라이다(CP210x)는 제외. 없으면 None."""
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        name = os.path.basename(p).lower()
        if "arduino" in name or "ch340" in name or "1a86" in name:
            return os.path.realpath(p)
    lidar = {os.path.realpath(p) for p in glob.glob("/dev/serial/by-id/*")
             if "cp210" in p.lower() or "silicon_labs" in p.lower()}
    for p in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")):
        if p not in lidar:
            return p
    return None


def parse_line(line):
    """펌웨어 한 줄 → ("state", 0|1|2) · ("pot", adc) · ("enc", (count, board_ms)) · None.

    E <count> <ms> = 뒷바퀴 엔코더(KY-040) 누적 카운트와 그걸 읽은 보드 millis().
    """
    line = line.strip()
    if line in ("0", "1", "2"):
        return ("state", int(line))
    if line.startswith("P "):
        try:
            return ("pot", int(line[2:]))
        except ValueError:
            return None
    if line.startswith("E "):
        parts = line.split()
        if len(parts) != 3:
            return None          # 줄이 잘리거나 섞인 것 — 원문은 serial_raw.log에 남는다
        try:
            return ("enc", (int(parts[1]), int(parts[2])))
        except ValueError:
            return None
    return None


class SerialLink:
    def __init__(self, port, baud=BAUD, boot_wait_s=BOOT_WAIT_S):
        import serial  # pyserial (sudo apt install python3-serial)
        self.port = port
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._lock = threading.Lock()
        time.sleep(boot_wait_s)
        self._ser.reset_input_buffer()

    def write(self, text):
        with self._lock:
            self._ser.write(text.encode("ascii"))

    def readline(self):
        return self._ser.readline().decode("ascii", errors="ignore").strip()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass
