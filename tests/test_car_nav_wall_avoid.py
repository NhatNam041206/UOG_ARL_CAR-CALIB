"""LiDAR arc-check wall avoidance under the PD follower (ros/car_nav/car_nav/wall_avoid.py)."""
import math

import numpy as np
import pytest
import yaml

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.pd_follower import PathTracker, PDGains, pd_curvature
from car_nav.wall_avoid import ArcAvoider, AvoidParams, free_distance, points_in_base

NAV = yaml.safe_load(P.NAV2_YAML.read_text())
PD = NAV["pd_path_follower"]["ros__parameters"]
CAR = yaml.safe_load(P.CAR_YAML.read_text())
F = CAR["footprint"]
KMAX = PD["avoid_max_curvature"]
PARAMS = AvoidParams(x_front=F["x_front"], x_rear=F["x_rear"], half_width=F["half_width"],
                     margin_m=PD["avoid_margin_m"], horizon_m=PD["avoid_horizon_m"],
                     release_factor=PD["avoid_release_factor"], n_candidates=PD["avoid_candidates"],
                     max_left=KMAX, max_right=KMAX)
G = PDGains(kp=PD["kp"], kd=PD["kd"], ff=PD["ff"], ff_ahead_m=PD["ff_ahead_m"])
ANG = np.linspace(-math.pi, math.pi, 720, endpoint=False)
HALF = 0.9   # campus 3-tile corridor: 1.8 m between the walls
E = 6.0      # end wall of the corridor (x)
BRANCH = 1.2  # 2-tile branch to the left, x in [E - BRANCH, E]  (sim goal 2, 2026-11-14)


def corridor():
    return [((-50.0, HALF), (50.0, HALF)), ((-50.0, -HALF), (50.0, -HALF))]


def l_junction():
    # main corridor along +x ending at x = E; a 1.2 m branch turns left (+y)
    return [((-50.0, -HALF), (E, -HALF)), ((E, -HALF), (E, 20.0)),
            ((-50.0, HALF), (E - BRANCH, HALF)), ((E - BRANCH, HALF), (E - BRANCH, 20.0))]


def raycast(segs, x, y, th):
    a = ANG + th
    dx, dy = np.cos(a), np.sin(a)
    best = np.full(ANG.shape, np.inf)
    for (x0, y0), (x1, y1) in segs:
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        wx, wy = x0 - x, y0 - y
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (wx * ey - wy * ex) / den
            u = (wx * dy - wy * dx) / den
        ok = (np.abs(den) > 1e-12) & (t > 1e-6) & (u >= 0) & (u <= 1)
        best = np.minimum(best, np.where(ok, t, np.inf))
    return best


def points_at(segs, x, y, th):
    return points_in_base(raycast(segs, x, y, th), -math.pi, ANG[1] - ANG[0], max_range=4.0)


def corners(x, y, th):
    c, s = math.cos(th), math.sin(th)
    for bx, by in ((F["x_front"], F["half_width"]), (F["x_front"], -F["half_width"]),
                   (F["x_rear"], F["half_width"]), (F["x_rear"], -F["half_width"])):
        yield x + c * bx - s * by, y + s * bx + c * by


def body_hits(segs, x, y, th):
    """True when a wall crosses the body outline."""
    cs = list(corners(x, y, th))
    edges = [(cs[0], cs[1]), (cs[1], cs[3]), (cs[3], cs[2]), (cs[2], cs[0])]

    def cross(p1, p2, p3, p4):
        def o(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        return o(p1, p2, p3) * o(p1, p2, p4) < 0 and o(p3, p4, p1) * o(p3, p4, p2) < 0
    return any(cross(a, b, c, d) for a, b in edges for c, d in segs)


def drive(segs, x, y, th, kappa_pd_fn, avoid=True, t_end=14.0, v=0.4, latency=0.25, dt=0.05):
    """Kinematic diff-drive car (base_link = centre) on curvature commands, delayed by `latency`.
    Returns (hit, final pose, number of times the layer took over)."""
    queue = [0.0] * int(round(latency / dt))
    av = ArcAvoider(PARAMS)
    takeovers, was = 0, False
    for i in range(int(t_end / dt)):
        k = kappa_pd_fn(x, y, th)
        if avoid and i % 2 == 0:          # 10 Hz like the node
            ch = av.step(k, points_at(segs, x, y, th))
            k = ch.kappa
            takeovers += ch.overriding and not was
            was = ch.overriding
        elif avoid and was:
            k = av.override_k
        queue.append(max(-KMAX, min(KMAX, k)))
        k_now = queue.pop(0)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += v * k_now * dt
        if body_hits(segs, x, y, th):
            return True, (x, y, th), takeovers
    return False, (x, y, th), takeovers


def left_turn_path():
    cx = E - BRANCH / 2 - 0.75                 # planner radius 0.75 into the branch centre line
    pts = [(x * 0.05, 0.0) for x in range(int(cx / 0.05) + 1)]
    pts += [(cx + 0.75 * math.sin(a), 0.75 - 0.75 * math.cos(a)) for a in np.linspace(0, math.pi / 2, 30)[1:]]
    pts += [(E - BRANCH / 2, 0.75 + i * 0.05) for i in range(1, 80)]
    return pts


def pd_on(path, offset=(0.0, 0.0)):
    tracker = PathTracker(path)

    def fn(x, y, th):
        return pd_curvature(tracker.track(x + offset[0], y + offset[1], th, G), G)
    return fn


def test_free_distance_straight_and_wall_ahead():
    assert free_distance(points_at(corridor(), 0, 0, 0), 0.0, PARAMS, 1.0) == 1.0
    end = [((1.0, -2.0), (1.0, 2.0))]
    f = free_distance(points_at(end, 0, 0, 0), 0.0, PARAMS, 1.0)
    reach = 1.0 - (F["x_front"] + PARAMS.margin_m)       # nose + margin reaches x = 1.0 after 0.6 m
    assert reach - 0.11 <= f <= reach + 0.01             # sampled every 0.1 m


def test_rear_swing_counts():
    # wall 0.07 m beside the rear on the right (body half width 0.30, margin 0.05): a hard
    # LEFT turn swings the rear into it (diff drive turns about the centre, so the rear
    # still swings out)
    y_wall = -(F["half_width"] + PARAMS.margin_m + 0.02)
    wall = [((-0.5, y_wall), (-0.2, y_wall))]
    pts = points_at(wall, 0, 0, 0)
    assert free_distance(pts, KMAX, PARAMS, 1.0) < 0.3
    assert free_distance(pts, 0.0, PARAMS, 1.0) == 1.0


def test_centred_in_corridor_does_nothing():
    av = ArcAvoider(PARAMS)
    for k in (0.0, 0.3, -0.3):
        ch = av.step(k, points_at(corridor(), 0.0, 0.0, 0.0))
        assert not ch.overriding and ch.kappa == k


def test_planned_left_turn_into_narrow_branch_is_not_fought():
    # sim 2026-11-14 goal 2: the old layer saw the inner corner and forced a right turn
    path = left_turn_path()
    hit_free, end_free, _ = drive(l_junction(), 0.0, 0.0, 0.0, pd_on(path), avoid=False, t_end=28)
    hit, end, takeovers = drive(l_junction(), 0.0, 0.0, 0.0, pd_on(path), avoid=True, t_end=28)
    assert not hit_free and not hit
    assert end[1] > 4.0 and abs(end[0] - (E - BRANCH / 2)) < 0.25     # up the branch
    assert takeovers <= 1          # at most a brief nudge where the arc grazes the margin


def test_late_turn_from_lagging_pose_is_saved():
    # the pose lags 0.8-1.2 m along the corridor (seen in the sim): the PD turns too late.
    # A 1.0 m horizon was too short for 0.8 m (the escape arc came too late): 1.3 m.
    path = left_turn_path()
    for lag in (0.8, 1.2):
        hit_free, _, _ = drive(l_junction(), 0.0, 0.0, 0.0, pd_on(path, offset=(-lag, 0.0)), avoid=False, t_end=28)
        hit, end, takeovers = drive(l_junction(), 0.0, 0.0, 0.0, pd_on(path, offset=(-lag, 0.0)), avoid=True,
                                    t_end=28)
        assert hit_free and not hit
        assert takeovers >= 1 and end[1] > 4.0                   # still got into the branch


def test_wrong_pose_drives_into_wall_without_layer_but_not_with_it():
    # pose 0.85 m off sideways: the PD returns to a line 5 cm from the left wall
    def pd_wrong(x, y, th):
        return -0.7 * (y - 0.85) - 1.2 * math.sin(th)
    assert drive(corridor(), 0, 0, 0, pd_wrong, avoid=False)[0]
    hit, _, takeovers = drive(corridor(), 0, 0, 0, pd_wrong, avoid=True)
    assert not hit and takeovers <= 6        # a few slow takeovers in 14 s, no on/off every scan


def test_heading_at_wall_recovers():
    for deg in (15, 25, 35):
        assert drive(corridor(), 0, 0, math.radians(deg), lambda x, y, th: 0.0, avoid=False)[0]
        assert not drive(corridor(), 0, 0, math.radians(deg), lambda x, y, th: 0.0, avoid=True)[0]


def test_spin_circle_radius_is_the_corner_distance():
    from car_nav.wall_avoid import spin_radius
    assert spin_radius(F["x_front"], F["x_rear"], F["half_width"]) == \
        math.hypot(max(F["x_front"], -F["x_rear"]), F["half_width"])
    assert spin_radius(0.35, -0.35, 0.30) == np.float64(math.sqrt(0.35 ** 2 + 0.30 ** 2))   # 0.46 m


def test_spin_clear_needs_the_whole_swept_circle_free():
    from car_nav.wall_avoid import spin_clear, spin_radius
    r = spin_radius(F["x_front"], F["x_rear"], F["half_width"])
    m = PD["spin_clear_margin_m"]
    # centred in a 1.8 m corridor (0.9 m to each wall) and in a 1.2 m one (0.6 m): free
    assert spin_clear(points_at(corridor(), 0, 0, 0), r, m)[0]
    narrow = [((-50.0, 0.6), (50.0, 0.6)), ((-50.0, -0.6), (50.0, -0.6))]
    ok, near = spin_clear(points_at(narrow, 0, 0, 0), r, m)
    assert ok and near == pytest.approx(0.6, abs=0.01)
    # 0.15 m off the middle of the 1.2 m corridor: a corner would hit the wall
    assert not spin_clear(points_at(narrow, 0, 0.15, 0), r, m)[0]
    # a point beside the rear, outside the footprint but inside the circle, blocks (it would be
    # swept); a point just outside the circle does not
    assert not spin_clear([(-0.2, -0.40)], r, m)[0]
    assert spin_clear([(-0.2, -(r + m + 0.01))], r, m)[0]
    assert spin_clear([], r, m) == (True, math.inf)


def test_spin_action_spins_creeps_or_waits():
    from car_nav.pd_follower import spin_action
    assert spin_action(2.0, True) == "spin"
    assert spin_action(1.0, False) == "creep"          # path roughly ahead: keep following slowly
    assert spin_action(-1.2, False) == "creep"
    assert spin_action(2.5, False) == "wait"           # path behind: stand still
    assert spin_action(1.0, False, creep_max_rad=0.5) == "wait"
