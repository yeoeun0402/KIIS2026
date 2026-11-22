"""ROS·실차 없이 도는 단위 테스트:  python3 -m pytest src/car_drive/test"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from car_drive.drive import Drive  # noqa: E402
from car_drive.serial_link import parse_line  # noqa: E402


class FakeLink:
    def __init__(self):
        self.sent = []

    def write(self, text):
        self.sent.append(text)


def test_start_sends_v0_before_g():
    # G가 V보다 먼저 가면 펌웨어 기본 속도 100으로 출발한다
    link = FakeLink()
    Drive(link).start()
    assert link.sent == ["V0\n", "G"]


def test_wasd_mapping_and_clamp():
    link = FakeLink()
    d = Drive(link, step=20, max_speed=50)
    for k in "www":
        assert d.key(k)
    assert d.speed == 50 and link.sent[-1] == "V50\n"     # 최고 속도로 잘림
    d.key("a"); d.key("d")
    assert link.sent[-2:] == ["L", "R"]
    for k in "SSSSSS":                                   # 대문자도
        d.key(k)
    assert d.speed == -50                                # 후진도 같은 한계
    d.key(" ")
    assert d.speed == 0 and link.sent[-2:] == ["V0\n", "F"]
    assert d.key("q") is False


def test_keepalive_repeats_current_speed():
    link = FakeLink()
    d = Drive(link, step=30)
    d.key("w")
    d.keepalive()
    assert link.sent[-1] == "V30\n"


def test_stop_sequence():
    link = FakeLink()
    d = Drive(link)
    d.key("w")
    d.stop()
    assert link.sent[-3:] == ["V0\n", "F", "S"] and d.speed == 0


def test_parse_line():
    assert parse_line("P 610") == ("pot", 610)
    assert parse_line("1\r") == ("state", 1)
    assert parse_line("P x") is None
    assert parse_line("U 0 25") is None


def test_parse_encoder_line():
    assert parse_line("E 1234 56789\r") == ("enc", (1234, 56789))
    assert parse_line("E -80 100") == ("enc", (-80, 100))      # 후진 / 반대 장착이면 음수
    assert parse_line("E 12") is None                          # 잘린 줄
    assert parse_line("E 1 2 3") is None
    assert parse_line("E x 100") is None
