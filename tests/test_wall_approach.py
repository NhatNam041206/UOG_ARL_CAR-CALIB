"""scripts/analyze_wall_approach.py on synthetic wall-approach data."""
from scripts import analyze_wall_approach as wa


def _rows(speed=0.4, x0=4.0, t_stop=6.0, decel=1.0, hz=10.0):
    rows, t, x, v = [], 0.0, x0, speed
    while t < 10.0:
        rows.append((1000.0 + t, x))
        if t >= t_stop:
            v = max(0.0, v - decel / hz)
        x -= v / hz
        t += 1.0 / hz
    return rows


def test_steady_speed_recovered():
    v = wa.steady_speed(_rows(speed=0.4))
    assert v is not None and abs(v - 0.4) < 0.02


def test_stopping_distance_with_stop_time():
    rows = _rows(speed=0.4, t_stop=6.0, decel=1.0)
    res = wa.analyse("synthetic", rows, stop_time=1006.0)
    # v^2 / (2a) = 0.08 m, plus discretisation
    assert res.stopping_distance_m is not None and 0.04 < res.stopping_distance_m < 0.14
    assert res.settle_time_s is not None and res.settle_time_s < 1.5


def test_no_motion_gives_none():
    rows = [(1000.0 + i / 10, 2.0) for i in range(50)]
    assert wa.steady_speed(rows) is None
