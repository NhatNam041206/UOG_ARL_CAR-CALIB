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


def test_person_zones_reach_further_than_the_wall_bumper():
    stop = ast.literal_eval(CM["PersonStop"]["forward"]["points"])
    bumper = ast.literal_eval(CM["VelocityStop"]["forward"]["points"])
    nose = CAR["footprint"]["x_front"]
    assert max(x for x, _ in stop) - nose >= 0.5                     # danger range >= 0.5 m
    assert max(x for x, _ in stop) > max(x for x, _ in bumper)
    assert CM["PersonStop"]["sources_names"] == ["dynamic"] == CM["PersonSlow"]["sources_names"]
    assert CM["VelocityStop"]["sources_names"] == ["scan"]
    assert CM["dynamic"]["topic"] == "/scan_dynamic"
    assert 0.0 < CM["PersonSlow"]["slowdown_ratio"] < 1.0


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
