"""키 → 펌웨어 명령 (ROS·시리얼과 분리 — 테스트에서 가짜 link로 검증한다).

펌웨어(firmware/car_controller) 규약에서 지켜야 할 것:
  * V를 받기 전에 G가 오면 펌웨어 기본 속도 100으로 바로 출발한다 → 시작은 반드시 V0 → G.
  * V를 한 번 받으면 500 ms 워치독이 켜진다 → keepalive()로 현재 속도를 계속 보낸다.
    노트북·노드가 죽으면 차는 0.5초 안에 스스로 선다.
  * L/R = 조향 펄스 한 번(120 ms). 키를 누르고 있으면 키 반복으로 계속 돈다.
"""

KEYS_HELP = (
    "W 전진 가속 · S 감속/후진 · A 좌 조향 · D 우 조향 · Space 정지 · Q 종료\n"
    "(주행 허용 G / 정지 S는 노드가 알아서 보낸다)"
)


class Drive:
    def __init__(self, link, step=20, max_speed=150):
        self.link = link
        self.step = int(step)
        self.max_speed = int(max_speed)
        self.speed = 0

    def start(self):
        self.link.write("V0\n")   # 먼저 시리얼 속도 모드로 — 이게 없으면 G가 기본 속도 100으로 출발시킨다
        self.link.write("G")

    def set_speed(self, value):
        self.speed = max(-self.max_speed, min(self.max_speed, int(value)))
        self.link.write(f"V{self.speed}\n")

    def keepalive(self):
        self.link.write(f"V{self.speed}\n")

    def key(self, k):
        """한 글자 처리. 종료 키면 False."""
        k = k.lower()
        if k == "w":
            self.set_speed(self.speed + self.step)
        elif k == "s":
            self.set_speed(self.speed - self.step)
        elif k == "a":
            self.link.write("L")
        elif k == "d":
            self.link.write("R")
        elif k == " ":
            self.set_speed(0)
            self.link.write("F")
        elif k in ("q", "\x03", "\x04"):   # q · Ctrl+C · Ctrl+D
            return False
        return True

    def stop(self):
        self.speed = 0
        self.link.write("V0\n")
        self.link.write("F")
        self.link.write("S")
