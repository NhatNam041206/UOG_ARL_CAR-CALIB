"""Reference copies of the pure-Python hot spots as they were before the numpy
versions (2026-11-16). Only for the equivalence tests: the new code must give the
same answers on the same scans."""
from __future__ import annotations

import math
from typing import Iterable, Sequence

def scan_observability_ref(ranges: Sequence[float], angle_min: float, angle_inc: float,
                       max_range: float = 8.0, facing_cos: float = 0.5,
                       n_low: int = 10, n_high: int = 50, min_span: float = 0.25,
                       max_span: float = 0.8, max_resid: float = 0.03, stride: int = 2) -> tuple[float, int]:
    """(w in 0..1, n facing points). Scan in the base frame (x = forward).

    For every `stride`-th point, a straight line is fitted (PCA) to its
    neighbours, growing the window until it spans >= min_span metres (with
    1 cm range noise a 5 cm window beside the car gives a random normal).
    The point "faces" the driving direction when that wall's normal has
    |cos(angle to x)| >= facing_cos and the wall is straight (residual <=
    max_resid). w = 0 below n_low facing points, 1 above n_high (counted
    on every point, i.e. stride x the sampled ones).
    """
    pts: list[tuple[float, float] | None] = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.05 or r > max_range:
            pts.append(None)
        else:
            a = angle_min + i * angle_inc
            pts.append((r * math.cos(a), r * math.sin(a)))
    n_pts = len(pts)
    n = 0
    for i in range(0, n_pts, stride):
        if pts[i] is None:
            continue
        lo = hi = i
        span = 0.0
        ok = True
        while span < min_span:
            if lo - 1 < 0 or hi + 1 >= n_pts or pts[lo - 1] is None or pts[hi + 1] is None:
                ok = False
                break
            lo, hi = lo - 1, hi + 1
            # a jump between neighbours = two different objects
            if math.dist(pts[lo], pts[lo + 1]) > 0.2 or math.dist(pts[hi], pts[hi - 1]) > 0.2:
                ok = False
                break
            span = math.dist(pts[lo], pts[hi])
            if span > max_span or hi - lo > 80:
                ok = False
                break
        if not ok:
            continue
        seg = pts[lo:hi + 1]
        m = len(seg)
        mx = sum(p[0] for p in seg) / m
        my = sum(p[1] for p in seg) / m
        sxx = sum((p[0] - mx) ** 2 for p in seg) / m
        syy = sum((p[1] - my) ** 2 for p in seg) / m
        sxy = sum((p[0] - mx) * (p[1] - my) for p in seg) / m
        # direction of the line = main axis; residual = smaller eigenvalue
        tr, det = sxx + syy, sxx * syy - sxy * sxy
        disc = math.sqrt(max(tr * tr / 4 - det, 0.0))
        lam_min = tr / 2 - disc
        if lam_min > max_resid ** 2:
            continue
        theta = 0.5 * math.atan2(2 * sxy, sxx - syy)      # line direction
        # normal = direction + 90 deg; |n_x| = |sin(theta)|
        if abs(math.sin(theta)) >= facing_cos:
            n += stride
    w = min(max((n - n_low) / float(n_high - n_low), 0.0), 1.0)
    return w, n


def _arc_pose(k: float, s: float) -> tuple[float, float, float]:
    if abs(k) < 1e-6:
        return s, 0.0, 0.0
    th = k * s
    return math.sin(th) / k, (1.0 - math.cos(th)) / k, th


def free_distance_ref(points: Sequence[tuple[float, float]], k: float, p: AvoidParams, horizon: float) -> float:
    """How far (m along the arc) the car drives on curvature k before its
    body (+ margin) touches a scan point; `horizon` if it never does."""
    xf, xr, hw = p.x_front + p.margin_m, p.x_rear - p.margin_m, p.half_width + p.margin_m
    n = max(1, int(round(horizon / p.step_m)))
    for i in range(1, n + 1):
        s = i * horizon / n
        px, py, th = _arc_pose(k, s)
        c, sn = math.cos(th), math.sin(th)
        for x, y in points:
            dx, dy = x - px, y - py
            bx = c * dx + sn * dy
            if bx > xf or bx < xr:
                continue
            if abs(-sn * dx + c * dy) <= hw:
                return (i - 1) * horizon / n
    return horizon


def near_points_ref(points: Iterable[tuple[float, float]], p: AvoidParams, horizon: float) -> list[tuple[float, float]]:
    """Only points the body can reach within the horizon."""
    reach = horizon + max(p.x_front, -p.x_rear) + p.half_width + p.margin_m
    return [(x, y) for x, y in points if x * x + y * y <= reach * reach]


def points_in_base_ref(ranges: Sequence[float], angle_min: float, angle_inc: float,
                   laser_x: float = 0.0, laser_y: float = 0.0, laser_yaw: float = 0.0,
                   min_range: float = 0.05, max_range: float = 12.0) -> list[tuple[float, float]]:
    """Valid scan points (x, y) in base_link, for a scan from a laser at (laser_x, laser_y, laser_yaw)."""
    out = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < min_range or r > max_range:
            continue
        a = angle_min + i * angle_inc + laser_yaw
        out.append((laser_x + r * math.cos(a), laser_y + r * math.sin(a)))
    return out


def world_clearance_ref(world, polygon):
    """car_nav_sim.world.World.clearance before the nearest-first speed-up."""
    import numpy as np
    from car_nav_sim import world as W
    poly_segs = np.hstack([polygon, np.roll(polygon, -1, axis=0)])
    best = math.inf
    for seg in world.segments():
        for ps in poly_segs:
            best = min(best, W._seg_seg_distance(ps, seg))
            if best == 0.0:
                return 0.0
    for c in world.cylinders:
        if W._point_in_polygon(c.cx, c.cy, polygon):
            return 0.0
        for ps in poly_segs:
            best = min(best, max(0.0, W._point_seg_distance(c.cx, c.cy, ps) - c.r))
    for b in world.boxes:
        if W._point_in_polygon(b.cx, b.cy, polygon) or W._point_in_polygon(*polygon[0], b.corners()):
            return 0.0
    return best
