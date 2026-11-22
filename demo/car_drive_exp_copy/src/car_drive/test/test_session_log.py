"""세션 로그 단위 테스트 (ROS 없이):  python3 -m pytest src/car_drive/test"""
import csv
import datetime as dt
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from car_drive.session_log import SessionLog  # noqa: E402


def test_session_files_and_summary(tmp_path):
    log = SessionLog(str(tmp_path), meta={"port": "/dev/ttyACM0"}, now=dt.datetime(2026, 11, 19, 1, 2, 3))
    assert log.dir == os.path.join(str(tmp_path), "20261119_010203") and os.path.isdir(log.dir)
    started = json.load(open(log.path("session.json"), encoding="utf-8"))
    assert started["end"] is None and started["port"] == "/dev/ttyACM0"   # 시작하자마자 남는다

    log.event("key", "w")
    log.event("tx", "V0")
    log.event("tx", "V-80")
    log.event("tx", "L")
    log.raw("P 600")
    log.event("pot", 600)
    log.raw("P 640")
    log.event("pot", 640)
    log.event("state", 1)
    meta = log.close("q")
    log.event("key", "late")            # 닫힌 뒤 쓰기는 무시

    s = meta["summary"]
    assert s["keys"] == 1 and s["commands_sent"] == 3 and s["board_lines"] == 2
    assert (s["pot_min"], s["pot_max"], s["pot_samples"]) == (600, 640, 2)
    assert s["max_abs_pwm"] == 80 and s["first_board_reply_s"] is not None
    saved = json.load(open(log.path("session.json"), encoding="utf-8"))
    assert saved["end_reason"] == "q" and saved["summary"] == s

    rows = list(csv.reader(open(log.path("drive_log.csv"), encoding="utf-8")))
    assert rows[0] == ["t_s", "kind", "value"]
    assert [r[1:] for r in rows[1:]] == [["key", "w"], ["tx", "V0"], ["tx", "V-80"], ["tx", "L"],
                                         ["pot", "600"], ["pot", "640"], ["state", "1"]]
    raw = open(log.path("serial_raw.log"), encoding="utf-8").read().splitlines()
    assert [r.split("\t")[1] for r in raw] == ["P 600", "P 640"]


def test_encoder_log(tmp_path):
    log = SessionLog(str(tmp_path))
    for c, ms in ((0, 1000), (8, 1050), (20, 1100), (15, 1250), (3, 40)):   # 1250: 줄 2개 빠짐 · 40: 보드 리셋
        log.event("enc", (c, ms))
    s = log.close("q")["summary"]
    assert s["enc_samples"] == 5 and (s["enc_first"], s["enc_last"]) == (0, 3)
    assert (s["enc_min"], s["enc_max"]) == (0, 20) and s["enc_net"] == 3
    assert s["enc_max_gap_ms"] == 150 and s["enc_board_resets"] == 1
    rows = list(csv.reader(open(log.path("encoder.csv"), encoding="utf-8")))
    assert rows[0] == ["t_s", "board_ms", "count"]
    assert [r[1:] for r in rows[1:]] == [["1000", "0"], ["1050", "8"], ["1100", "20"], ["1250", "15"], ["40", "3"]]
    drive = list(csv.reader(open(log.path("drive_log.csv"), encoding="utf-8")))
    assert [r[1:] for r in drive[1:]] == [["enc", "0"], ["enc", "8"], ["enc", "20"], ["enc", "15"], ["enc", "3"]]


def test_same_second_gets_new_folder(tmp_path):
    t = dt.datetime(2026, 11, 19, 1, 2, 3)
    a = SessionLog(str(tmp_path), now=t)
    b = SessionLog(str(tmp_path), now=t)
    assert a.dir != b.dir and b.name == "20261119_010203-2" and a.root == b.root == str(tmp_path)
    a.close(); b.close()


def test_no_reply_summary(tmp_path):
    log = SessionLog(str(tmp_path))
    log.event("tx", "V0")
    s = log.close("ctrl+c")["summary"]
    assert s["first_board_reply_s"] is None and s["board_lines"] == 0 and s["pot_min"] is None
    assert s["enc_samples"] == 0 and s["enc_net"] is None and s["enc_max_gap_ms"] is None
    assert open(log.path("encoder.csv"), encoding="utf-8").read() == "t_s,board_ms,count\n"   # 엔코더 없어도 헤더는 남는다
