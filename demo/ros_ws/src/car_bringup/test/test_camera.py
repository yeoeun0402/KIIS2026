import cv2
import numpy as np

from car_bringup.c920_camera import standard_dht


def strip_dht(d):
    out, i = d[:2], 2
    while i < len(d) - 4 and d[i] == 0xFF:
        m, ln = d[i + 1], int.from_bytes(d[i + 2:i + 4], "big")
        if m == 0xDA:
            return out + d[i:]
        if m != 0xC4:
            out += d[i:i + 2 + ln]
        i += 2 + ln
    return out


def test_standard_dht_has_four_tables():
    d = standard_dht()
    assert d.count(b"\xff\xc4") >= 1 and len(d) > 400       # DC/AC × 휘도/색차


def test_inserted_dht_restores_decoding():
    img = (np.random.default_rng(0).random((120, 160, 3)) * 255).astype(np.uint8)
    full = cv2.imencode(".jpg", img)[1].tobytes()
    bare = strip_dht(full)
    assert b"\xff\xc4" not in bare
    i = bare.find(b"\xff\xda")
    fixed = bare[:i] + standard_dht() + bare[i:]
    a = cv2.imdecode(np.frombuffer(full, np.uint8), cv2.IMREAD_COLOR)
    b = cv2.imdecode(np.frombuffer(fixed, np.uint8), cv2.IMREAD_COLOR)
    assert b is not None and np.array_equal(a, b)
