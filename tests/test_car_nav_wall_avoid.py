"""LiDAR wall avoidance under the PD follower (ros/car_nav/car_nav/wall_avoid.py)."""
import math

import numpy as np
import yaml

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.wall_avoid import AvoidParams, WallAvoider, WallDist, points_in_base, wall_distances

NAV = yaml.safe_load(P.NAV2_YAML.read_text())
PD = NAV["pd_path_follower"]["ros__parameters"]
CAR = yaml.safe_load(P.CAR_YAML.read_text())
F = CAR["footprint"]
PARAMS = AvoidParams(x_front=F["x_front"], x_rear=F["x_rear"], half_width=F["half_width"],
                     front_exit_m=PD["avoid_front_exit_m"], side_exit_m=PD["avoid_side_exit_m"],
                     enter_ratio=PD["avoid_enter_ratio"], turn_curvature=PD["avoid_turn_curvature"],
                     look_ahead_m=PD["avoid_look_ahead_m"], max_curvature=PD["avoid_max_curvature"],
                     rear_margin_m=PD["avoid_rear_margin_m"])
ANG = np.linspace(-math.pi, math.pi, 720, endpoint=False)
HALF = 0.9  # campus corridors are ~1.8 m between the walls
K = PARAMS.turn_curvature


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


def walls_at(y0, th, end_x=None):
    return wall_distances(points_in_base(scan(y0, th, end_x), -math.pi, ANG[1] - ANG[0]), PARAMS)


def test_centred_in_corridor_does_nothing():
    av = WallAvoider(PARAMS)
    assert av.step(0.3, walls_at(0.0, 0.0)) == 0.3 and av.side is None
    assert av.step(0.3, walls_at(0.3, 0.0)) == 0.3 and av.side is None   # 0.3 m off: the PD's job


def test_fixed_turn_not_growing_with_closeness():
    for y0 in (0.5, 0.6):                          # 0.27, 0.17 m from the left wall (rear still clear)
        av = WallAvoider(PARAMS)
        assert av.step(0.0, walls_at(y0, 0.0)) == -K and av.side == "left"
    av = WallAvoider(PARAMS)
    assert av.step(-1.0, walls_at(0.6, 0.0)) == -1.0     # the PD already turns away harder: keep it
    assert av.step(+0.5, walls_at(0.6, 0.0)) == -K       # the PD (wrong pose) turns into the wall


def test_hysteresis_no_on_off_between_thresholds():
    av = WallAvoider(PARAMS)
    # gap beside the body: 0.9 - y0 - 0.135; enter below 0.30 m, leave above 0.45 m
    assert av.step(0.0, walls_at(0.40, 0.0)) == 0.0          # 0.365 m: not entered
    assert av.step(0.0, walls_at(0.50, 0.0)) == -K           # 0.265 m: enters
    assert av.step(0.0, walls_at(0.40, 0.0)) == -K           # 0.365 m: still avoiding
    assert av.step(0.0, walls_at(0.34, 0.0)) == -K           # 0.425 m: still avoiding
    assert av.step(0.0, walls_at(0.30, 0.0)) == 0.0          # 0.465 m: left the zone
    assert av.side is None


def test_end_wall_turns_the_way_the_path_turns():
    for k_pd in (+0.8, -0.8):
        av = WallAvoider(PARAMS)
        kappa = av.step(k_pd, walls_at(0.0, 0.0, end_x=1.0))
        assert math.copysign(1, kappa) == math.copysign(1, k_pd) and abs(kappa) >= K


def test_rear_guard_limits_swing_into_wall():
    av = WallAvoider(PARAMS)
    k = av.step(0.0, WallDist(left=0.3, rear_gap_left=0.03))
    assert -K < k <= 0.0 and abs(k) <= 0.3 * K


def _corners(x, y, th):
    c, s = math.cos(th), math.sin(th)
    for bx, by in ((F["x_front"], F["half_width"]), (F["x_front"], -F["half_width"]),
                   (F["x_rear"], F["half_width"]), (F["x_rear"], -F["half_width"])):
        yield x + c * bx - s * by, y + s * bx + c * by


def drive(y0, th0, kappa_pd_fn, avoid=True, t_end=12.0, v=0.5, latency=0.65, dt=0.05):
    """Kinematic rear-steer car (base_link = front axle), command delayed by `latency`.
    Returns (smallest wall clearance of any body corner, number of avoid on/off switches)."""
    x, y, th = 0.0, y0, th0
    queue = [0.0] * int(round(latency / dt))
    worst = math.inf
    av = WallAvoider(PARAMS)
    switches, prev = 0, None
    for _ in range(int(t_end / dt)):
        k = kappa_pd_fn(y, th)
        if avoid:
            k = av.step(k, walls_at(y, th))
            switches += av.side != prev
            prev = av.side
        queue.append(max(-PARAMS.max_curvature, min(PARAMS.max_curvature, k)))
        k_now = queue.pop(0)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += v * k_now * dt
        worst = min(worst, min(HALF - abs(cy) for _, cy in _corners(x, y, th)))
    return worst, switches


def test_wrong_pose_drives_into_wall_without_layer_but_not_with_it():
    # The pose is 0.85 m off sideways: the PD "returns to the path" along a
    # line 0.05 m from the real left wall, so the body hits it.
    def pd_on_wrong_pose(y, th):
        return -0.7 * (y - 0.85) - 1.2 * math.sin(th)

    assert drive(0.0, 0.0, pd_on_wrong_pose, avoid=False)[0] < 0.0
    clear, switches = drive(0.0, 0.0, pd_on_wrong_pose, avoid=True)
    assert clear > 0.02
    assert switches <= 12          # a few slow cycles in 12 s, not on/off every scan


def test_heading_at_wall_recovers():
    # blind and wrong: PD holds the heading 25 deg towards the left wall
    for deg in (15, 25, 35):
        assert drive(0.0, math.radians(deg), lambda y, th: 0.0, avoid=False)[0] < 0.0
        assert drive(0.0, math.radians(deg), lambda y, th: 0.0, avoid=True)[0] > 0.04


def test_slightly_angled_in_corridor_does_nothing():
    av = WallAvoider(PARAMS)
    for deg in (-8, 8):
        assert av.step(0.0, walls_at(0.0, math.radians(deg))) == 0.0 and av.side is None
