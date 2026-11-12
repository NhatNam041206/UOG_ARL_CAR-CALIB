"""Adaptive speed / odometry logic (ros/car_nav/car_nav/speed_filter.py)."""
import math

import numpy as np
import pytest

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.speed_filter import MotionModel, SpeedKF, fuse_forward, scan_observability
from car_nav_sim.world import World

ANG = np.linspace(-math.pi, math.pi, 720, endpoint=False)
CAMPUS = World.load(str(P.SIM_DIR / "worlds" / "campus_path.yaml"))


def _w(x, y, th):
    r = CAMPUS.raycast(x, y, ANG + th, 12.0)
    return scan_observability(list(r), -math.pi, 2 * math.pi / 720)[0]


def test_plain_corridor_is_blind_ends_are_not():
    # the corridor south of AAD-1 (campus_block_left goal 1), driving south
    assert _w(43.5, 20.0, -math.pi / 2) < 0.1          # middle: only side walls
    assert _w(43.5, 13.0, -math.pi / 2) == 1.0          # at the corner
    assert _w(43.5, 26.0, -math.pi / 2) == 1.0          # at the start (end wall behind)


def test_motion_model_follows_delayed_commands():
    m = MotionModel(latency_s=0.65, accel_mps2=1.0, decel_mps2=2.0, nominal_speed=0.5)
    m.command(0.0, True)
    t = 0.0
    while t < 0.6:
        t += 0.1
        m.step(t, 0.1)
    assert m.standing                      # command not yet acting
    for _ in range(10):
        t += 0.1
        m.step(t, 0.1)
    assert m.cruising
    m.command(t, False)
    for _ in range(10):
        t += 0.1
        m.step(t, 0.1)
    assert m.standing


def test_kf_learns_speed_and_ignores_a_dead_rf2o():
    kf = SpeedKF(v=0.5)
    rng = np.random.default_rng(0)
    for _ in range(100):
        kf.predict(0.1)
        kf.update_rf2o(0.42 + rng.normal(0, 0.05), 1.0)
    assert kf.v == pytest.approx(0.42, abs=0.02)
    for _ in range(50):                    # rf2o says "standing" while the car drives
        kf.predict(0.1)
        assert not kf.update_rf2o(0.0, 1.0)
    assert kf.v == pytest.approx(0.42, abs=0.02)


def test_correction_pulls_speed_toward_truth():
    kf = SpeedKF(v=0.5, var=0.05 ** 2)
    kf.update_correction(along_m=-1.0, blind_m=6.0)      # AMCL: the car is 1 m behind odometry
    assert kf.v < 0.5


def _corridor(variation):
    """Drive 4 m observable then 8 m blind; forward error of learned vs fixed 0.5 m/s."""
    rng = np.random.default_rng(1)
    kf, dt, t = SpeedKF(v=0.5), 0.1, 0.0
    true_s = est_s = fixed_s = 0.0
    while true_s < 12.0:
        v = 0.42 * (1 + variation * math.sin(2 * math.pi * t / 30.0))
        w = 1.0 if true_s < 4.0 else 0.0
        rf2o_dx = v * dt * w + rng.normal(0, 0.005)        # rf2o sees nothing when blind
        kf.predict(dt)
        ok = kf.update_rf2o(rf2o_dx / dt, w)
        est_s += fuse_forward(rf2o_dx, kf.v * dt, w if ok else 0.0)
        fixed_s += v * dt if w else 0.5 * dt
        true_s += v * dt
        t += dt
    return abs(est_s - true_s), abs(fixed_s - true_s)


def test_blind_corridor_constant_wrong_speed():
    learned, fixed = _corridor(0.0)          # real 0.42, config 0.5
    assert learned < 0.15 and fixed > 1.4


def test_blind_corridor_drifting_speed_still_better_than_fixed():
    # +-15 % within 30 s is harsh: the speed changes inside the blind stretch,
    # which only the next observable place / AMCL correction can fix.
    learned, fixed = _corridor(0.15)
    assert learned < fixed
