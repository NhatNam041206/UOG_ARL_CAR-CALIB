"""AMCL gate for blind corridors (ros/car_nav/car_nav/localization_gate.py)."""
import math

import numpy as np

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.localization_gate import compose, gate_correction, inverse, reseed_covariance


def test_compose_inverse():
    a = (1.0, -2.0, 0.7)
    b = (0.3, 0.4, -1.2)
    x, y, th = compose(compose(a, b), inverse(b))
    assert math.isclose(x, a[0], abs_tol=1e-9) and math.isclose(y, a[1], abs_tol=1e-9)
    assert math.isclose(th, a[2], abs_tol=1e-9)


def test_first_pose_taken_fully():
    step = gate_correction(None, (1.0, 0.0, 0.0), (5.0, 3.0, 1.0), w=0.0)
    assert np.allclose(step.pose, (5.0, 3.0, 1.0))
    assert np.allclose(compose(step.map_odom, (1.0, 0.0, 0.0)), (5.0, 3.0, 1.0))


def test_blind_keeps_sideways_and_heading_drops_along():
    mo = (0.0, 0.0, 0.0)
    odom = (10.0, 0.0, 0.0)              # car heading +x along the corridor
    amcl = (11.0, 0.2, 0.05)             # AMCL: 1 m further, 0.2 m left, turned 0.05 rad
    step = gate_correction(mo, odom, amcl, w=0.0)
    along_axis = (math.cos(0.05), math.sin(0.05))
    rel = (step.pose[0] - 10.0, step.pose[1])
    assert abs(rel[0] * along_axis[0] + rel[1] * along_axis[1]) < 1e-9   # no along shift
    assert abs(step.pose[1] - 0.2) < 0.06                                 # sideways taken
    assert math.isclose(step.pose[2], 0.05)
    assert step.taken == 0.0 and math.isclose(step.along, 1.0 * math.cos(0.05) + 0.2 * math.sin(0.05))
    full = gate_correction(mo, odom, amcl, w=1.0)
    assert np.allclose(full.pose, amcl)
    half = gate_correction(mo, odom, amcl, w=0.25, obs_full=0.5)
    assert 10.4 < half.pose[0] < 10.6


def test_wandering_amcl_in_blind_corridor():
    """20 m corridor: odometry 5 % short, AMCL's cloud random-walks along it.
    The gated pose ends no worse than the odometry; raw AMCL ends far off."""
    rng = np.random.default_rng(3)
    mo, gated_err, amcl_err = None, 0.0, 0.0
    amcl_drift = 0.0
    for i in range(1, 201):
        true_x = 0.1 * i
        odom = (0.95 * true_x, 0.0, 0.0)
        amcl_drift += 0.02 + rng.normal(0.0, 0.1)   # the cloud slides along the corridor
        w = 1.0 if i < 5 else 0.0          # end wall in view only at the start
        amcl = (true_x + (amcl_drift if i >= 5 else 0.0), rng.normal(0, 0.03), 0.0)
        step = gate_correction(mo, odom, amcl, w)
        mo = step.map_odom
        gated_err, amcl_err = abs(step.pose[0] - true_x), abs(amcl[0] - true_x)
    assert gated_err < 1.1          # ~ the odometry's 5 % of 20 m
    assert amcl_err > 1.5


def test_reseed_covariance_long_along_heading():
    cov = np.array(reseed_covariance(math.pi / 2, 0.15, 0.08, 0.05)).reshape(6, 6)
    xy = cov[:2, :2]
    assert np.allclose(xy, xy.T) and np.all(np.linalg.eigvalsh(xy) > 0)
    assert math.isclose(xy[1, 1], 0.15 ** 2) and math.isclose(xy[0, 0], 0.08 ** 2, rel_tol=1e-9)
    assert math.isclose(cov[5, 5], 0.05 ** 2)


def test_initialpose_retry_waits_resends_and_gives_up():
    from car_nav.localization_gate import InitialPoseRetry
    r = InitialPoseRetry(interval_s=2.0, max_tries=3)
    assert not r.due(100.0)                       # nothing to wait for yet
    r.on_initialpose(10.0)
    assert not r.due(11.9)                        # AMCL still has time
    assert r.due(12.0) and r.tries == 1           # first resend after 2 s
    assert not r.due(13.0)
    assert r.due(14.0) and r.due(16.0) and r.tries == 3
    assert not r.due(30.0) and r.gave_up          # max tries reached: stop, report


def test_initialpose_retry_stops_when_amcl_answers_after_it():
    from car_nav.localization_gate import InitialPoseRetry
    r = InitialPoseRetry(interval_s=2.0, max_tries=10)
    r.on_initialpose(10.0)
    r.on_amcl(9.5)                                # an AMCL pose from BEFORE the initial pose
    assert r.due(12.0)                            # does not count (the startup deadlock, sim 2026-11-17)
    r.on_amcl(12.5)
    assert not r.due(20.0) and not r.gave_up
    r.on_initialpose(30.0)                        # a new initial pose starts a new wait
    assert r.due(32.0) and r.tries == 1


def test_turning_on_the_spot_takes_only_the_heading():
    from car_nav.localization_gate import compose, gate_correction
    mo = (0.0, 0.0, 0.0)
    # corridor along y; mid-turn the car faces across it (east), so AMCL's
    # 1.2 m slide along the corridor is "sideways" and would be taken
    odom = (5.0, 0.0, 0.0)
    amcl = (5.0, 1.2, 0.03)
    step = gate_correction(mo, odom, amcl, w=0.0, hold_position=True)
    pose = compose(step.map_odom, odom)
    assert abs(pose[0] - 5.0) < 1e-9 and abs(pose[1]) < 1e-9 and abs(pose[2] - 0.03) < 1e-9
    free = gate_correction(mo, odom, amcl, w=0.0)
    assert abs(compose(free.map_odom, odom)[1] - 1.2) < 0.01   # without the hold the slide is taken
