"""가짜 Arduino Mega — firmware/car_controller.ino 의 명령 해석·워치독을 그대로 흉내 낸다 (모터 없음).

  python3 fake_board.py <pty_path> <out_json>

- 50 ms 마다 "P <adc>", 100 ms 마다/변할 때 상태 0/1/2 를 보낸다 (펌웨어와 같은 주기)
- 받은 바이트를 펌웨어 로직(canGo · useSerialSpeed · commandedSpeed · 500 ms 워치독)으로 해석
- 실제로 모터에 나갔을 '유효 속도'를 매 루프 기록 → 최대 |speed| 가 0 이면 차는 한 번도 안 움직였을 것
- SIGTERM 받으면 요약 JSON 저장
- 조향 POT: car_state.sim.SimCar 의 DC 모터 조향 모델 (L/R 펄스 120 ms → POT 이동, 끝에서 멈춤, 잡음) — 고정 512 아님
- 바퀴 엔코더: 50 ms 마다 "E <count> <ms>" (펌웨어와 같은 형식). 유효 속도 → 1차 지연 바퀴 속도 → 카운트 적분
  (시험용 가정값: PWM 255 = 1.2 m/s, 시정수 0.3 s, 바퀴 지름 0.2 m, 1회전 80 카운트)
"""
import json
import os
import signal
import sys
import time

import serial

sys.path.insert(0, os.path.expanduser("~/demo/ros_ws/src/car_state"))
from car_state.sim import SimCar  # noqa: E402

port, out = sys.argv[1], sys.argv[2]
s = serial.Serial(port, 9600, timeout=0)
can_go, use_serial, cmd_speed, last_rx = False, False, 0, time.monotonic()
t0 = time.monotonic()
rx_log, max_eff, eff_nonzero_s, last_state, steer = [], 0, 0.0, -1, []
buf = b""
last_pot = last_stat = last_enc = 0.0
ENC_CPR, WHEEL_D_M, V_PER_PWM, TAU_S = 80, 0.20, 1.2 / 255, 0.3   # 시험용 가정값 (실차 아님)
wheel_v, enc_pos = 0.0, 0.0      # m/s · 카운트(실수, 출력은 내림)
encs = []
running = True


def stop(*_):
    global running
    running = False


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
prev = prev_step = time.monotonic()
car = SimCar(seed=0)
pots = []
while running:
    now = time.monotonic()
    data = s.read(256)
    if data:
        buf += data
        last_rx = now
    # 펌웨어 readSerialCommand(): 한 글자씩, V 뒤에는 parseInt
    while buf:
        c = chr(buf[0]); buf = buf[1:]
        t = round(now - t0, 3)
        if c in "G1":
            can_go = True
            if not use_serial:
                cmd_speed = 100           # ← V 없이 G 오면 기본 속도로 출발하는 펌웨어 동작
            rx_log.append([t, "G"])
        elif c == "2":
            can_go = True
            if not use_serial:
                cmd_speed = -100
            rx_log.append([t, "2"])
        elif c in "S3":
            can_go = False; rx_log.append([t, "S"])
        elif c == "V":
            num = b""
            while buf[:1] and (buf[:1].isdigit() or (buf[:1] == b"-" and not num)):
                num += buf[:1]; buf = buf[1:]
            if buf[:1] == b"" and not buf.endswith(b"\n"):
                # 숫자가 덜 왔으면 다음 read 까지 기다린다
                more = s.read(16); buf += more
                while buf[:1] and (buf[:1].isdigit() or (buf[:1] == b"-" and not num)):
                    num += buf[:1]; buf = buf[1:]
            v = int(num) if num not in (b"", b"-") else 0
            cmd_speed = max(-255, min(255, v)); use_serial = True
            if not rx_log or rx_log[-1][1] != f"V{cmd_speed}":
                rx_log.append([t, f"V{cmd_speed}"])
        elif c in "LRF":
            steer.append([t, c]); rx_log.append([t, c])
            car.command(now - t0, c)
    wd = use_serial and (now - last_rx > 0.5)
    eff = 0 if (not can_go or wd) else cmd_speed
    max_eff = max(max_eff, abs(eff))
    if eff:
        eff_nonzero_s += now - prev
    prev = now
    state = 1 if eff > 0 else (2 if eff < 0 else 0)
    if state != last_state or now - last_stat >= 0.1:
        s.write(f"{state}\r\n".encode()); last_state, last_stat = state, now
    if now - last_pot >= 0.05:
        s.write(f"P {car.read_pot()}\r\n".encode()); last_pot = now
        pots.append(car.read_pot())
    dt = now - prev_step
    wheel_v += (eff * V_PER_PWM - wheel_v) * min(1.0, dt / TAU_S)
    enc_pos += wheel_v * dt / (3.141592653589793 * WHEEL_D_M) * ENC_CPR
    if now - last_enc >= 0.05:
        c = int(enc_pos // 1)
        s.write(f"E {c} {int((now - t0) * 1000)}\r\n".encode()); last_enc = now
        encs.append(c)
    car.step(prev_step - t0, dt)    # [prev_step, now] 구간
    prev_step = now
    time.sleep(0.005)

summary = {"duration_s": round(time.monotonic() - t0, 2), "max_effective_speed": max_eff,
           "effective_nonzero_s": round(eff_nonzero_s, 3), "final_can_go": can_go,
           "used_serial_speed": use_serial, "first_commands": rx_log[:6], "last_commands": rx_log[-4:],
           "n_commands_logged": len(rx_log), "steer_cmds": steer,
           "pot_min": min(pots) if pots else None, "pot_max": max(pots) if pots else None,
           "enc_samples": len(encs), "enc_last": encs[-1] if encs else None}
with open(out, "w") as f:
    json.dump(summary, f, ensure_ascii=False, indent=1)
