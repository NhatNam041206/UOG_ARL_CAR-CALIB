"""Safety guard for people: dynamic points (car_nav/dynamic_obstacles.py) and sim actors."""
import ast
import math

import numpy as np
import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav.dynamic_obstacles import dynamic_beams, static_mask
from car_nav_sim.actors import Actor, load_actors

NAV = yaml.safe_load(P.NAV2_YAML.read_text())
CM = NAV["collision_monitor"]["ros__parameters"]
CAR = yaml.safe_load(P.CAR_YAML.read_text())


def _room():
    occ = np.zeros((100, 100), dtype=np.int16)      # 5 x 5 m at 0.05, origin (0, 0)
    occ[:, 0] = occ[:, -1] = occ[0, :] = occ[-1, :] = 100
    return occ


def test_walls_are_static_people_are_not():
    mask = static_mask(_room(), 0.05, 0.3)
    ang = np.array([0.0, math.pi / 2])
    # sensor at (2.5, 2.5): beam 0 hits the east wall (x = 4.95), beam 1 a person 1 m north
    r = [2.45, 1.0]
    dyn = dynamic_beams(r, 0.0, math.pi / 2, (2.5, 2.5, 0.0), mask, (0.0, 0.0), 0.05)
    assert list(dyn) == [False, True]
    # a 0.25 m pose error (sensor believed at 2.75 m, the wall then seen at 4.7 m) is still the wall
    dyn = dynamic_beams([1.95, 1.0], 0.0, math.pi / 2, (2.75, 2.5, 0.0), mask, (0.0, 0.0), 0.05)
    assert not dyn[0]


def test_far_and_invalid_beams_are_ignored():
    mask = static_mask(_room(), 0.05, 0.3)
    dyn = dynamic_beams([math.inf, 0.0, 4.5], 0.0, 0.1, (2.5, 2.5, 0.0), mask, (0.0, 0.0), 0.05, max_range=4.0)
    assert not dyn.any()


def test_actor_trigger_walk_wait_and_end():
    a = Actor("p", 0.2, [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], speed_mps=1.0, trigger=(0.0, 0.0, 1.0), wait_s=1.0)
    a.update(0.0, (5.0, 0.0))
    assert a.position(3.0) == (0.0, 0.0)            # car far away: not started
    a.update(10.0, (0.5, 0.0))                      # car within 1 m: walks from t = 10
    assert a.position(11.0) == pytest.approx((1.0, 0.0))
    assert a.position(12.5) == pytest.approx((2.0, 0.0))    # waiting at the corner
    assert a.position(14.0) == pytest.approx((2.0, 1.0))
    assert a.position(99.0) == pytest.approx((2.0, 2.0))    # stays at the end


@pytest.mark.parametrize("name", ["person_crossing", "person_stops", "person_late"])
def test_person_missions_load(name):
    m = yaml.safe_load((P.SIM_DIR / "missions" / f"{name}.yaml").read_text())
    acts = load_actors(m)
    assert len(acts) == 1 and acts[0].radius == pytest.approx(0.2)


def _guard():
    from car_nav.safety_zone import GuardParams
    g = NAV["safety_guard"]["ros__parameters"]
    p = GuardParams()
    p.update(g)
    return p


def test_guard_params_from_nav2_params():
    p = _guard()
    assert p.danger_range_m >= 0.5 and p.slow_range_m > p.danger_range_m
    assert p.fov_deg <= 180.0                       # front only
    assert NAV["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"] == "cmd_vel_guarded"


def test_guard_stops_slows_and_ignores_what_is_behind():
    from car_nav.safety_zone import decide
    p = _guard()
    xf, hb = CAR["footprint"]["x_front"], CAR["footprint"]["half_width"]
    blob = lambda x, y: np.array([[x + dx, y + dy] for dx in (-0.03, 0, 0.03) for dy in (-0.03, 0.03)])  # noqa: E731
    assert decide(blob(xf + 0.4, 0.0), p, xf, hb).state == "stop"
    assert decide(blob(xf + 1.0, 0.1), p, xf, hb).state == "slow"
    assert decide(blob(xf + 3.0, 0.0), p, xf, hb).state == "clear"
    assert decide(blob(-0.4, 0.0), p, xf, hb).state == "clear"          # behind the LiDAR / car
    assert decide(blob(xf + 0.1, 0.6), p, xf, hb).state == "clear"      # beside the nose, outside fov
    assert decide(blob(xf + 0.15, 0.25), p, xf, hb, turning_on_spot=True).state == "stop"
    assert decide(np.zeros((0, 2)), p, xf, hb).state == "clear"
    p.update({"enabled": False})
    assert decide(blob(xf + 0.4, 0.0), p, xf, hb).state == "off"


def test_ui_params_are_clamped():
    from car_nav.safety_zone import GuardParams
    p = GuardParams()
    changed = p.update({"danger_range_m": 9.0, "slow_range_m": 0.1, "unknown": 1})
    assert p.danger_range_m == 3.0 and p.slow_range_m == 3.0 and "danger_range_m" in changed


def test_ui_map_rle_round_trip():
    from car_nav.ui_pack import rle
    v = np.array([0, 0, 100, 100, 100, -1, 0])
    r = rle(v)
    assert r == [0, 2, 100, 3, -1, 1, 0, 1]
    assert sum(r[1::2]) == v.size


def test_alignment_keeps_walls_static_with_a_bad_pose():
    from car_nav.dynamic_obstacles import align_to_map
    occ = _room()
    thin, wide = static_mask(occ, 0.05, 0.05, unknown_is_static=False), static_mask(occ, 0.05, 0.3)
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    # true sensor at (1.0, 2.5): ranges to the room walls (x 0..5, y 0..5, wall cells ~0.05 m)
    tx, ty = 1.0, 2.5
    r = []
    for a in ang:
        c, s = math.cos(a), math.sin(a)
        ts = [t for t in ((0.05 - tx) / c if c < 0 else (4.95 - tx) / c if c > 0 else math.inf,
                          (0.05 - ty) / s if s < 0 else (4.95 - ty) / s if s > 0 else math.inf) if t > 0]
        r.append(min(ts))
    bad = (1.4, 2.5, 0.0)                                   # 0.4 m off
    before = dynamic_beams(r, -math.pi, ang[1] - ang[0], bad, wide, (0.0, 0.0), 0.05, 2.0).sum()
    fixed = align_to_map(r, -math.pi, ang[1] - ang[0], bad, thin, (0.0, 0.0), 0.05)
    after = dynamic_beams(r, -math.pi, ang[1] - ang[0], fixed, wide, (0.0, 0.0), 0.05, 2.0).sum()
    assert before > 10 and after == 0
    assert abs(fixed[0] - tx) < 0.1
