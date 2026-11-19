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


def _drive(path, x, y, th, latency=0.25, v=0.4, steps=600):
    y0 = y
    t = PathTracker(path)
    queue = [0.0] * int(round(latency / 0.1))
    overshoot = 0.0   # how far it swings past the path to the other side
    for _ in range(steps):
        tr = t.track(x, y, th, G)
        queue.append(max(-2.5, min(2.5, pd_curvature(tr, G))))
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


def _corner_error(ff_ahead, latency=0.25, v=0.4):
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
        queue.append(max(-2.5, min(2.5, pd_curvature(tr, g))))
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
    # Skid-steer car, 0.25 s delay at 0.4 m/s: reading the path curvature
    # about the delay distance ahead (0.1 m) follows a 1 m-radius corner
    # within ~1 cm; reading it further ahead cuts the corner (0.4 m: ~15 cm).
    assert _corner_error(NAV_FF_AHEAD) < 0.03
    assert _corner_error(0.4) > 3 * _corner_error(0.1)


def test_extrapolate_pose_straight_arc_cap_and_standstill():
    from car_nav.pd_follower import extrapolate_pose
    x, y, th = extrapolate_pose(1.0, 2.0, 0.0, 0.5, 0.0, 0.4)
    assert (x, y, th) == pytest.approx((1.2, 2.0, 0.0))
    # capped distance
    x, y, _ = extrapolate_pose(0.0, 0.0, math.pi / 2, 0.5, 0.0, 2.0, max_dist=0.3)
    assert (x, y) == pytest.approx((0.0, 0.3), abs=1e-12)
    # standing still / no age / negative speed: unchanged
    assert extrapolate_pose(1.0, 1.0, 0.3, 0.0, 0.2, 0.5) == (1.0, 1.0, 0.3)
    assert extrapolate_pose(1.0, 1.0, 0.3, 0.5, 0.2, 0.0) == (1.0, 1.0, 0.3)
    assert extrapolate_pose(1.0, 1.0, 0.3, -0.5, 0.2, 0.5) == (1.0, 1.0, 0.3)
    # quarter circle of radius 1: v 0.5, omega 0.5 for pi seconds = pi/2 m (cap off)
    x, y, th = extrapolate_pose(0.0, 0.0, 0.0, 0.5, 0.5, math.pi, max_dist=10.0)
    assert (x, y, th) == pytest.approx((1.0, 1.0, math.pi / 2), abs=1e-9)
    # matches small-step integration of the car model
    xi, yi, ti = 0.0, 0.0, 0.2
    for _ in range(3000):
        xi += 0.5 * math.cos(ti) * 1e-4; yi += 0.5 * math.sin(ti) * 1e-4; ti += -0.8 * 1e-4
    assert extrapolate_pose(0.0, 0.0, 0.2, 0.5, -0.8, 0.3) == pytest.approx((xi, yi, ti), abs=1e-4)


def test_diff_drive_turns_on_the_spot_then_follows():
    from car_nav.pd_follower import DiffDriveParams, Tracking, diff_drive_command
    p = DiffDriveParams()
    tr = lambda psi: Tracking(index=0, e=0.0, psi_e=psi, kappa_path=0.0, s=0.0, remaining=5.0)  # noqa: E731
    v, w, rot = diff_drive_command(tr(math.radians(120)), 0.0, p, False, 5.0)   # path behind-left
    assert rot and v == 0.0 and w < 0                                          # car points left: turn right
    v, w, rot = diff_drive_command(tr(math.radians(30)), 0.0, p, True, 5.0)     # hysteresis: keep turning
    assert rot and v == 0.0
    v, w, rot = diff_drive_command(tr(math.radians(5)), 0.5, p, True, 5.0)
    assert not rot and v == p.speed_mps and w == pytest.approx(p.speed_mps * 0.5)


def test_diff_drive_slows_in_tight_curves_and_near_the_goal():
    from car_nav.pd_follower import DiffDriveParams, Tracking, diff_drive_command
    p = DiffDriveParams(speed_mps=0.4, max_yaw_rate_rps=1.0, goal_slow_m=0.6, min_speed_mps=0.08)
    tr = Tracking(index=0, e=0.0, psi_e=0.0, kappa_path=0.0, s=0.0, remaining=5.0)
    v, w, _ = diff_drive_command(tr, 5.0, p, False, 5.0)       # 0.2 m radius
    assert v == pytest.approx(0.2) and w == pytest.approx(1.0)
    v, _, _ = diff_drive_command(tr, 0.0, p, False, 0.3)
    assert v == pytest.approx(0.2)
    v, _, _ = diff_drive_command(tr, 0.0, p, False, 0.0)
    assert v == pytest.approx(0.08)


def test_slows_down_before_a_tight_corner():
    from car_nav.pd_follower import DiffDriveParams, Tracking, diff_drive_command
    path = [(x * 0.05, 0.0) for x in range(41)]
    path += [(2.0 + 0.4 * math.sin(a), 0.4 - 0.4 * math.cos(a)) for a in [i * math.pi / 2 / 20 for i in range(1, 21)]]
    t = PathTracker(path)
    assert t.max_curvature_ahead(1.0, 0.8) < 0.2            # corner still > 0.8 m away
    k = t.max_curvature_ahead(1.6, 0.8)
    assert k > 2.0                                          # 0.4 m radius ahead
    p = DiffDriveParams(speed_mps=0.4, lat_accel_mps2=0.25)
    tr = Tracking(index=0, e=0.0, psi_e=0.0, kappa_path=0.0, s=1.6, remaining=3.0)
    v, _, _ = diff_drive_command(tr, 0.0, p, False, 3.0, kappa_ahead=k)
    assert v < 0.36
