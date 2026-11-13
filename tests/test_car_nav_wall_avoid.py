"""LiDAR wall avoidance under the PD follower (ros/car_nav/car_nav/wall_avoid.py)."""
import math

import numpy as np
import yaml

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.wall_avoid import AvoidParams, WallPush, blend, points_in_base, wall_push

NAV = yaml.safe_load(P.NAV2_YAML.read_text())
PD = NAV["pd_path_follower"]["ros__parameters"]
CAR = yaml.safe_load(P.CAR_YAML.read_text())
F = CAR["footprint"]
PARAMS = AvoidParams(x_front=F["x_front"], x_rear=F["x_rear"], half_width=F["half_width"],
                     front_influence_m=PD["avoid_front_influence_m"], side_influence_m=PD["avoid_side_influence_m"],
                     look_ahead_m=PD["avoid_look_ahead_m"], max_curvature=PD["avoid_max_curvature"],
                     rear_margin_m=PD["avoid_rear_margin_m"])
ANG = np.linspace(-math.pi, math.pi, 720, endpoint=False)
HALF = 0.9  # campus corridors are ~1.8 m between the walls


def scan(y0, th, end_x=None):
    """Ranges from base_link at (0, y0, th) in a corridor |y| <= HALF along x (end wall at x = end_x)."""
    a = ANG + th
    r = np.full(ANG.shape, np.inf)
    s, c = np.sin(a), np.cos(a)
    with np.errstate(divide="ignore", invalid="ignore"):
        for wall in (HALF, -HALF):
            t = (wall - y0) / s
            r = np.where((t > 0) & (t < r), t, r)
        if end_x is not None:
            t = end_x / c
            r = np.where((t > 0) & (t < r), t, r)
    return r


def push_at(y0, th, end_x=None):
    return wall_push(points_in_base(scan(y0, th, end_x), -math.pi, ANG[1] - ANG[0]), PARAMS)


def test_centred_in_corridor_does_nothing():
    push = push_at(0.0, 0.0)
    assert push.active == 0.0
    assert blend(0.3, push, PARAMS) == (0.3, 0.0)
    # 0.3 m off centre is still fine (the path follower's job)
    assert push_at(0.3, 0.0).active < 0.05


def test_close_to_left_wall_turns_right():
    push = push_at(0.6, 0.0)
    assert push.left > 0.1 and push.right == 0.0
    kappa, a = blend(0.0, push, PARAMS)
    assert kappa < -0.1 and a == push.left


def test_heading_into_wall_overrides_wrong_pose():
    # the PD (on a wrong pose) still steers left into the left wall
    push = push_at(0.3, math.radians(30))
    kappa, a = blend(+0.5, push, PARAMS)
    assert a > 0.2 and kappa < 0.0


def test_end_wall_turns_the_way_the_path_turns():
    for k_pd in (+0.8, -0.8):
        kappa, a = blend(k_pd, push_at(0.0, 0.0, end_x=0.9), PARAMS)
        # 0.74 m ahead of the nose, before the collision monitor's stop box (0.48 m)
        assert a > 0.2 and math.copysign(1, kappa) == math.copysign(1, k_pd)
        assert abs(kappa) > abs(k_pd)


def test_rear_guard_limits_swing_into_wall():
    # nose pushed right by a left wall, but the rear is already 3 cm from the left wall
    push = WallPush(left=0.6, right=0.0, rear_gap_left=0.03)
    k_guard, _ = blend(0.0, push, PARAMS)
    k_free, _ = blend(0.0, WallPush(left=0.6, right=0.0), PARAMS)
    assert k_free < k_guard <= 0.0
    assert abs(k_guard) < 0.3 * abs(k_free)


def _corners(x, y, th):
    c, s = math.cos(th), math.sin(th)
    for bx, by in ((F["x_front"], F["half_width"]), (F["x_front"], -F["half_width"]),
                   (F["x_rear"], F["half_width"]), (F["x_rear"], -F["half_width"])):
        yield x + c * bx - s * by, y + s * bx + c * by


def drive(y0, th0, kappa_pd_fn, avoid=True, t_end=12.0, v=0.5, latency=0.65, dt=0.05):
    """Kinematic rear-steer car (base_link = front axle), command delayed by `latency`.
    Returns the smallest wall clearance of any body corner."""
    x, y, th = 0.0, y0, th0
    queue = [0.0] * int(round(latency / dt))
    worst = math.inf
    for _ in range(int(t_end / dt)):
        k = kappa_pd_fn(y, th)
        if avoid:
            k, _ = blend(k, push_at(y, th), PARAMS)
        queue.append(max(-PARAMS.max_curvature, min(PARAMS.max_curvature, k)))
        k_now = queue.pop(0)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += v * k_now * dt
        worst = min(worst, min(HALF - abs(cy) for _, cy in _corners(x, y, th)))
    return worst


def test_wrong_pose_drives_into_wall_without_layer_but_not_with_it():
    # The pose is 0.85 m off sideways: the PD "returns to the path" along a
    # line 0.05 m from the real left wall, so the body hits it.
    def pd_on_wrong_pose(y, th):
        return -0.7 * (y - 0.85) - 1.2 * math.sin(th)

    assert drive(0.0, 0.0, pd_on_wrong_pose, avoid=False) < 0.0   # hits the left wall
    assert drive(0.0, 0.0, pd_on_wrong_pose, avoid=True) > 0.02   # stays off it


def test_heading_at_wall_recovers():
    # blind and wrong: PD holds the heading 25 deg towards the left wall
    assert drive(0.0, math.radians(25), lambda y, th: 0.0, avoid=False) < 0.0
    assert drive(0.0, math.radians(25), lambda y, th: 0.0, avoid=True) > 0.0
