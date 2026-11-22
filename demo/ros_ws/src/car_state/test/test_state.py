"""car_state 순수 계산 시험 (가짜 차량). colcon test."""
import math

import numpy as np
import pytest

from car_state.ekf import CtrvEkf, ctrv
from car_state.sim import SimCar, calib_script, run_script
from car_state.steer import PotDirection, SteerCalib, analyze_sweep, find_segments, fit_circle, fit_table


@pytest.fixture(scope="module")
def drive():
    car = SimCar(backlash_adc=6.0, seed=1)
    sc, tend = calib_script()
    return car, run_script(car, sc, t_end=tend)


def test_ctrv_circle():
    s = np.array([0, 0, 0, 1.0, 1.0])             # 반지름 1 m 원 반 바퀴
    for _ in range(314):
        s = ctrv(s, 0.01)
    assert abs(s[0]) < 0.01 and abs(s[1] - 2.0) < 0.01


def test_ekf_speed_better_than_diff(drive):
    _, d = drive
    ek = CtrvEkf(sigma_pos=0.01, sigma_theta=math.radians(0.7))
    V = []
    for t, x, y, th in zip(d["pose_t"], d["x"], d["y"], d["th"]):
        ek.update(t, [x, y, th]); V.append(ek.s[3])
    m = d["pose_t"] > 2
    e = np.array(V)[m] - d["v_true"][m]
    naive = np.hypot(np.diff(d["x"]), np.diff(d["y"])) / np.diff(d["pose_t"]) - d["v_true"][1:]
    assert np.median(np.abs(e)) < 0.06
    assert np.sqrt(np.mean(e ** 2)) < 0.3 * np.sqrt(np.mean(naive ** 2))


def test_ekf_rejects_jump_then_resets():
    ek = CtrvEkf()
    for i in range(30):
        ek.update(i / 30, [0.01 * i, 0, 0])
    assert ek.update(1.0, [5, 5, 0]) == "rejected"
    r = [ek.update(1 + i / 30, [5, 5, 0]) for i in range(1, 6)]
    assert "reset" in r


def test_sweep(drive):
    car, d = drive
    sw = analyze_sweep(d["pot_t"], d["pot"], d["cmd_t"], d["cmd"])
    assert abs(sw["pot_min"] - car.lo) <= 3 and abs(sw["pot_max"] - car.hi) <= 3
    assert abs(abs(sw["adc_per_pulse_L"]) - 22) < 2 and sw["left_increases_pot"] is False
    assert 50 <= sw["response_ms_L"] <= 150


def test_calibration_recovers_truth(drive):
    car, d = drive
    sw = analyze_sweep(d["pot_t"], d["pot"], d["cmd_t"], d["cmd"])
    segs = find_segments(d["pose_t"], d["x"], d["y"], d["th"], d["pot_t"], d["pot"], d["cmd_t"], d["cmd"], wheelbase=car.L)
    tab = fit_table(segs, car.L, sweep=sw)
    assert abs(tab["backlash_adc"] - 6.0) < 2.0
    assert abs(tab["pot_center"] - car.c) < 5
    cal = SteerCalib(tab["table"]["pot"], np.radians(tab["table"]["delta_deg"]), tab["backlash_adc"])
    pd = PotDirection()
    dirs = np.array([pd.update(v) for v in d["pot"]])
    idx = np.clip(np.searchsorted(d["pot_t"], d["pose_t"]) - 1, 0, None)
    est = np.array([cal(d["pot"][i], dirs[i])[0] for i in idx])
    err = np.degrees(np.abs(est - d["delta_true"]))
    assert np.median(err) < 0.4 and np.percentile(err, 95) < 1.5


def test_calib_monotonic_and_inverse():
    c = SteerCalib([300, 500, 700], np.radians([20, 0, -20]), backlash_adc=6)
    assert abs(math.degrees(c(500)[0])) < 1e-9
    assert c(500, +1)[0] > 0 > c(500, -1)[0]                 # 유격 보정 방향
    assert abs(c.pot_for(math.radians(10)) - 400) < 1e-6
    with pytest.raises(ValueError):
        SteerCalib([300, 500, 700], [0.1, 0.2, 0.1])


def test_fit_circle():
    a = np.linspace(0, 1.5, 50)
    cx, cy, r, rms = fit_circle(1 + 2 * np.cos(a), -1 + 2 * np.sin(a))
    assert abs(r - 2) < 1e-6 and abs(cx - 1) < 1e-6 and rms < 1e-6
