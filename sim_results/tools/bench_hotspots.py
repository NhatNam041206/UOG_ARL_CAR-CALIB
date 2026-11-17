"""Cost per call of the Python hot spots on realistic campus scans, old (pure Python,
tests/_car_nav_reference.py) vs current (numpy). Host CPU, idle machine.
    python sim_results/tools/bench_hotspots.py
"""
import math, os, sys, time
import numpy as np
ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
sys.path[:0] = [os.path.join(ROOT, "ros", "car_nav"), os.path.join(ROOT, "ros", "car_nav_sim"), ROOT]
from car_nav.speed_filter import scan_observability
from car_nav import wall_avoid
from car_nav.wall_avoid import ArcAvoider, AvoidParams, points_in_base
from car_nav_sim.world import World, footprint_polygon
from tests import _car_nav_reference as ref

W = World.load(os.path.join(ROOT, "ros/car_nav_sim/worlds/campus_path.yaml"))
N = 720
ANG = np.linspace(-math.pi, math.pi, N, endpoint=False)
INC = 2 * math.pi / N
cases = {"mid corridor": (43.5, 20.0, -math.pi / 2), "goal-2 approach": (43.5, 13.6, -math.pi / 2),
         "in the turn": (43.9, 12.4, -math.pi / 4)}
p = AvoidParams()


def t_ms(fn, k):
    ts = []
    for _ in range(k):
        t = time.perf_counter(); fn(); ts.append((time.perf_counter() - t) * 1e3)
    return np.median(ts), max(ts)


def arc_steps(free_fn, free1_fn, near_fn, pts):
    saved = wall_avoid.free_distances, wall_avoid.free_distance, wall_avoid.near_points
    if free_fn:
        wall_avoid.free_distances, wall_avoid.free_distance, wall_avoid.near_points = free_fn, free1_fn, near_fn
    try:
        ts = []
        for kap in np.linspace(-1.5, 1.5, 31):
            av = ArcAvoider(p); t = time.perf_counter(); av.step(float(kap), pts); ts.append((time.perf_counter() - t) * 1e3)
        return np.median(ts), max(ts)
    finally:
        wall_avoid.free_distances, wall_avoid.free_distance, wall_avoid.near_points = saved


print("| case | function | old median / max ms | numpy median / max ms |")
print("|---|---|---|---|")
rng = np.random.default_rng(1)
for name, (x, y, th) in cases.items():
    r = [float(v) for v in W.raycast(x, y, th + ANG, 12.0) + rng.normal(0, 0.01, N)]
    o = t_ms(lambda: ref.scan_observability_ref(r, -math.pi, INC), 30)
    n = t_ms(lambda: scan_observability(r, -math.pi, INC), 30)
    print(f"| {name} | scan_observability | {o[0]:.2f} / {o[1]:.2f} | {n[0]:.2f} / {n[1]:.2f} |")
    o = t_ms(lambda: ref.points_in_base_ref(r, -math.pi, INC, max_range=3.19), 30)
    n = t_ms(lambda: points_in_base(r, -math.pi, INC, max_range=3.19), 30)
    print(f"| {name} | points_in_base | {o[0]:.2f} / {o[1]:.2f} | {n[0]:.2f} / {n[1]:.2f} |")
    pts_old = ref.points_in_base_ref(r, -math.pi, INC, max_range=3.19)
    pts_new = points_in_base(r, -math.pi, INC, max_range=3.19)
    o = arc_steps(lambda pts, ks, pp, h: np.array([ref.free_distance_ref(pts, k, pp, h) for k in ks]),
                  ref.free_distance_ref, ref.near_points_ref, pts_old)
    n = arc_steps(None, None, None, pts_new)
    print(f"| {name} | ArcAvoider.step | {o[0]:.2f} / {o[1]:.2f} | {n[0]:.2f} / {n[1]:.2f} |")
    poly = footprint_polygon(x, y, th, 0.16, -0.37, 0.135)
    o = t_ms(lambda: ref.world_clearance_ref(W, poly), 10)
    n = t_ms(lambda: W.clearance(poly), 10)
    print(f"| {name} | World.clearance (sim scorer) | {o[0]:.2f} / {o[1]:.2f} | {n[0]:.2f} / {n[1]:.2f} |")
