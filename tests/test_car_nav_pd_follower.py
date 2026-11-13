"""PD path follower math (ros/car_nav/car_nav/pd_follower.py)."""
import math

import pytest

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.pd_follower import PathTracker, PDGains, pd_curvature

import yaml

G = PDGains(kp=0.7, kd=1.2)
NAV_FF_AHEAD = yaml.safe_load(P.NAV2_YAML.read_text())["pd_path_follower"]["ros__parameters"]["ff_ahead_m"]
STRAIGHT = [(x * 0.05, 0.0) for x in range(100)]


def test_offset_and_heading_signs():
    tr = PathTracker(STRAIGHT).track(1.0, 0.2, 0.0, G)        # left of path
    assert tr.e == pytest.approx(0.2) and pd_curvature(tr, G) < 0     # turn right
    tr = PathTracker(STRAIGHT).track(1.0, 0.0, 0.3, G)        # pointing left
    assert tr.psi_e == pytest.approx(0.3) and pd_curvature(tr, G) < 0
    tr = PathTracker(STRAIGHT).track(1.0, -0.2, 0.0, G)
    assert pd_curvature(tr, G) > 0


def test_feed_forward_reads_path_curvature():
    r = 1.0
    arc = [(r * math.sin(a / 100), r - r * math.cos(a / 100)) for a in range(157)]
    tr = PathTracker(arc).track(0.3, r - math.sqrt(r * r - 0.09), 0.3, G)
    assert tr.kappa_path == pytest.approx(1.0, rel=0.05)
    assert pd_curvature(tr, G) == pytest.approx(1.0, abs=0.1)


def test_tracker_does_not_jump_to_a_later_pass():
    # out 3 m, round a tight loop, back 0.4 m to the side: the car at the start
    # must stay on the first pass, not snap to the return leg.
    path = [(x * 0.05, 0.0) for x in range(61)] + [(3.0, 0.2), (3.0, 0.4)] + \
           [(3.0 - x * 0.05, 0.4) for x in range(1, 61)]
    t = PathTracker(path)
    tr = t.track(0.5, 0.25, 0.0, G)
    assert tr.e == pytest.approx(0.25) and tr.s < 1.0


def _drive(path, x, y, th, latency=0.65, v=0.5, steps=600):
    y0 = y
    t = PathTracker(path)
    queue = [0.0] * int(round(latency / 0.1))
    overshoot = 0.0   # how far it swings past the path to the other side
    for _ in range(steps):
        tr = t.track(x, y, th, G)
        queue.append(max(-1 / 0.55, min(1 / 0.49, pd_curvature(tr, G))))
        k = queue.pop(0)
        for _ in range(5):
            x += v * math.cos(th) * 0.02
            y += v * math.sin(th) * 0.02
            th += v * k * 0.02
        overshoot = max(overshoot, -tr.e * math.copysign(1.0, y0))
        if tr.remaining < 0.1:
            break
    return tr, overshoot


def test_returns_to_path_with_the_cars_delay():
    path = [(x * 0.05, 0.0) for x in range(240)]
    tr, overshoot = _drive(path, 0.0, 0.4, 0.2)    # 0.4 m off, pointing further away
    assert abs(tr.e) < 0.02 and overshoot < 0.08


def _corner_error(ff_ahead, latency=0.65, v=0.5):
    # straight 2 m, 90 deg left arc of radius 1.0 m, straight 2 m
    path = [(x * 0.05, 0.0) for x in range(41)]
    path += [(2.0 + math.sin(a), 1.0 - math.cos(a)) for a in [i * math.pi / 2 / 30 for i in range(1, 31)]]
    path += [(3.0, 1.0 + i * 0.05) for i in range(1, 41)]
    g = PDGains(kp=G.kp, kd=G.kd, ff=G.ff, ff_ahead_m=ff_ahead)
    t = PathTracker(path)
    x, y, th = 0.0, 0.0, 0.0
    queue = [0.0] * int(round(latency / 0.1))
    worst = 0.0
    for _ in range(400):
        tr = t.track(x, y, th, g)
        queue.append(max(-1.33, min(1.33, pd_curvature(tr, g))))
        k = queue.pop(0)
        for _ in range(5):
            x += v * math.cos(th) * 0.02
            y += v * math.sin(th) * 0.02
            th += v * k * 0.02
        worst = max(worst, abs(tr.e))
        if tr.remaining < 0.1:
            break
    return worst


def test_corner_with_the_cars_delay_and_a_true_pose():
    # 2026-11-13 check: with a TRUE pose the 0.65 s delay does not make the
    # PD turn late (0.3 m: ~2 cm off). Reading the curvature further ahead
    # makes it cut the corner instead (0.65 m: ~20 cm inside). So a late turn
    # in the sim comes from the pose estimate lagging, not from the delay.
    assert _corner_error(NAV_FF_AHEAD) < 0.05
    assert _corner_error(0.65) > 3 * _corner_error(0.3)
