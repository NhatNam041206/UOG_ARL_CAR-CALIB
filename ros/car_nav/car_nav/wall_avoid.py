"""Reactive wall avoidance from the LiDAR scan (pure math, no ROS imports).

A safety layer under the PD path follower. It does not need to know where
the car is on the map, only where the walls are right now.

Arc check (classical, the idea behind DWA), nam 2026-11-15: a layer that
only looked at "how close is a wall on each side" fought planned turns (the
inner corner of a left turn made it steer right, sim 2026-11-14 goal 2).
So instead it tries the drive out before it happens:

  1. Take the curvature the PD wants and sweep the car's footprint (nose,
     sides AND the rear, which swings out on this rear-steered car) along
     that arc for horizon_m metres (the distance covered in the command
     delay plus a margin). If no scan point falls inside the swept body
     (+ margin), the PD drives: the layer does nothing, however close a
     wall is.
  2. If the PD's arc hits something, try fixed candidate arcs from full
     right to full left and take the one that drives furthest before a
     hit; among those that are clear for the whole horizon, the one
     closest to the PD's curvature.
  3. Hysteresis: once overriding, it keeps its chosen arc while that arc
     stays clear, and hands back to the PD only when the PD's arc is clear
     over release_factor x horizon (longer than the entry check), so it
     does not switch on and off every scan.

Kinematics: base_link is the floor point under the FRONT (fixed) axle; it
moves on a circle of curvature k, the body turns with it, the rear swings
outwards. Units: metres, 1/m, + curvature = left. Scan points must be in
base_link (points_in_base()).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence


@dataclass(frozen=True)
class AvoidParams:
    x_front: float = 0.16          # footprint (car.yaml)
    x_rear: float = -0.37
    half_width: float = 0.135
    margin_m: float = 0.05          # extra clearance around the body
    horizon_m: float = 1.3          # how far ahead an arc must be clear (m of front-axle travel)
    step_m: float = 0.1             # sampling along the arc
    release_factor: float = 1.3     # PD arc must be clear this x horizon to hand back
    max_left: float = 1.33          # candidate arcs span [-max_right, +max_left] (1/m)
    max_right: float = 1.33
    n_candidates: int = 13


def points_in_base(ranges: Sequence[float], angle_min: float, angle_inc: float,
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


def _arc_pose(k: float, s: float) -> tuple[float, float, float]:
    if abs(k) < 1e-6:
        return s, 0.0, 0.0
    th = k * s
    return math.sin(th) / k, (1.0 - math.cos(th)) / k, th


def free_distance(points: Sequence[tuple[float, float]], k: float, p: AvoidParams, horizon: float) -> float:
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


def near_points(points: Iterable[tuple[float, float]], p: AvoidParams, horizon: float) -> list[tuple[float, float]]:
    """Only points the body can reach within the horizon."""
    reach = horizon + max(p.x_front, -p.x_rear) + p.half_width + p.margin_m
    return [(x, y) for x, y in points if x * x + y * y <= reach * reach]


@dataclass
class ArcChoice:
    kappa: float
    overriding: bool
    pd_free_m: float
    chosen_free_m: float


@dataclass
class ArcAvoider:
    p: AvoidParams
    override_k: float | None = field(default=None)

    def reset(self) -> None:
        self.override_k = None

    def candidates(self) -> list[float]:
        n = max(self.p.n_candidates, 3)
        return [-self.p.max_right + (self.p.max_left + self.p.max_right) * i / (n - 1) for i in range(n)]

    def step(self, kappa_pd: float, points: Iterable[tuple[float, float]]) -> ArcChoice:
        p = self.p
        k_pd = min(max(kappa_pd, -p.max_right), p.max_left)
        long_h = p.horizon_m * p.release_factor
        pts = near_points(points, p, long_h)
        if self.override_k is None:
            pd_free = free_distance(pts, k_pd, p, p.horizon_m)
            if pd_free >= p.horizon_m:
                return ArcChoice(kappa_pd, False, pd_free, pd_free)
        else:
            pd_free = free_distance(pts, k_pd, p, long_h)
            if pd_free >= long_h:
                self.override_k = None
                return ArcChoice(kappa_pd, False, pd_free, pd_free)
            # keep the current escape arc while it is still clear
            if free_distance(pts, self.override_k, p, p.horizon_m) >= p.horizon_m:
                return ArcChoice(self.override_k, True, pd_free, p.horizon_m)
        best = None  # (free, -|k - k_pd|, k)
        for k in self.candidates():
            f = free_distance(pts, k, p, p.horizon_m)
            key = (f, -abs(k - k_pd), k)
            if best is None or key > best:
                best = key
        self.override_k = best[2]
        return ArcChoice(best[2], True, pd_free, best[0])
