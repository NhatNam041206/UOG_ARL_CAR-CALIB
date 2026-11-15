"""The numpy hot spots give the same answers as the pure-Python code they replaced
(tests/_car_nav_reference.py), on random scans of the campus map."""
import math

import numpy as np
import pytest

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from tests import _car_nav_reference as ref
from car_nav import wall_avoid
from car_nav.speed_filter import scan_observability
from car_nav.wall_avoid import ArcAvoider, AvoidParams, free_distance, near_points, points_in_base
from car_nav_sim.world import World

ANG = np.linspace(-math.pi, math.pi, 720, endpoint=False)
CAMPUS = World.load(str(P.SIM_DIR / "worlds" / "campus_path.yaml"))


def _scan(rng, noise=0.01, dropout=0.02):
    x, y, th = rng.uniform(40, 56), rng.uniform(10, 28), rng.uniform(-math.pi, math.pi)
    r = CAMPUS.raycast(x, y, ANG + th, 12.0) + rng.normal(0, noise, ANG.shape)
    r[rng.random(ANG.shape) < dropout] = np.inf
    return list(r)


@pytest.mark.parametrize("seed", range(4))
def test_scan_observability_same_as_reference(seed):
    rng = np.random.default_rng(seed)
    for _ in range(40):
        r = _scan(rng)
        assert scan_observability(r, -math.pi, 2 * math.pi / 720) == \
            ref.scan_observability_ref(r, -math.pi, 2 * math.pi / 720)


def test_scan_observability_edge_cases():
    for r in ([], [1.0], [1.0, 1.0], [math.inf] * 720, [0.01] * 720, [None, 1.0, 1.0, 1.0]):
        assert scan_observability(r, -math.pi, 0.01) == ref.scan_observability_ref(r, -math.pi, 0.01)


@pytest.mark.parametrize("seed", range(3))
def test_free_distance_same_as_reference(seed):
    rng = np.random.default_rng(100 + seed)
    p = AvoidParams()
    for _ in range(30):
        pts = points_in_base(_scan(rng), -math.pi, 2 * math.pi / 720, max_range=3.19)
        nr, nn = ref.near_points_ref(pts, p, 1.69), near_points(pts, p, 1.69)
        assert [tuple(q) for q in nn] == [tuple(q) for q in nr]
        for k in list(rng.uniform(-1.8, 1.8, 6)) + [0.0, 1e-7]:
            assert free_distance(nn, k, p, 1.3) == pytest.approx(ref.free_distance_ref(nr, k, p, 1.3), abs=1e-12)
    assert free_distance([], 0.5, p, 1.3) == 1.3


def test_arc_avoider_same_choices_as_reference(monkeypatch):
    """Whole ArcAvoider (hysteresis included) over random scan sequences."""
    rng = np.random.default_rng(7)
    p = AvoidParams()
    seqs = []
    for _ in range(25):
        pts = points_in_base(_scan(rng), -math.pi, 2 * math.pi / 720, max_range=3.19)
        seqs.append((pts, float(rng.uniform(-1.6, 1.6))))
    new = ArcAvoider(p)
    got = [new.step(k, pts) for pts, k in seqs]
    monkeypatch.setattr(wall_avoid, "free_distances",
                        lambda pts, ks, pp, h: np.array([ref.free_distance_ref(pts, k, pp, h) for k in ks]))
    monkeypatch.setattr(wall_avoid, "free_distance", ref.free_distance_ref)
    monkeypatch.setattr(wall_avoid, "near_points", ref.near_points_ref)
    old = ArcAvoider(p)
    want = [old.step(k, pts) for pts, k in seqs]
    for a, b in zip(got, want):
        assert a.overriding == b.overriding
        assert a.kappa == pytest.approx(b.kappa, abs=1e-12)
        assert a.pd_free_m == pytest.approx(b.pd_free_m, abs=1e-12)
        assert a.chosen_free_m == pytest.approx(b.chosen_free_m, abs=1e-12)


def test_points_in_base_same_as_reference():
    rng = np.random.default_rng(11)
    for _ in range(20):
        r = _scan(rng)
        a = points_in_base(r, -math.pi, 2 * math.pi / 720, 0.1, -0.02, 0.3, max_range=3.19)
        b = ref.points_in_base_ref(r, -math.pi, 2 * math.pi / 720, 0.1, -0.02, 0.3, max_range=3.19)
        assert a.shape == (len(b), 2)
        assert np.allclose(a, np.array(b).reshape(-1, 2), atol=1e-12, rtol=0)
    assert points_in_base([], -math.pi, 0.01).shape == (0, 2)
