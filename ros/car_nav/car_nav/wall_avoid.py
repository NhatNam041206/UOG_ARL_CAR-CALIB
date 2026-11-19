"""Reactive wall avoidance from the LiDAR scan (pure math, no ROS imports).

A safety layer under the PD path follower. It does not need to know where
the car is on the map, only where the walls are right now.

Arc check (classical, the idea behind DWA), nam 2026-11-15: a layer that
only looked at "how close is a wall on each side" fought planned turns (the
inner corner of a left turn made it steer right, sim 2026-11-14 goal 2).
So instead it tries the drive out before it happens:

  1. Take the curvature the PD wants and sweep the car's footprint (nose,
     sides AND the rear, which swings out when the car turns about its centre) along
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

import numpy as np


@dataclass(frozen=True)
class AvoidParams:
    x_front: float = 0.35          # footprint (car.yaml)
    x_rear: float = -0.35
    half_width: float = 0.30
    margin_m: float = 0.05          # extra clearance around the body
    horizon_m: float = 1.3          # how far ahead an arc must be clear (m of base_link travel)
    step_m: float = 0.1             # sampling along the arc
    release_factor: float = 1.3     # PD arc must be clear this x horizon to hand back
    max_left: float = 1.33          # candidate arcs span [-max_right, +max_left] (1/m)
    max_right: float = 1.33
    n_candidates: int = 13


def points_in_base(ranges: Sequence[float], angle_min: float, angle_inc: float,
                   laser_x: float = 0.0, laser_y: float = 0.0, laser_yaw: float = 0.0,
                   min_range: float = 0.05, max_range: float = 12.0) -> np.ndarray:
    """Valid scan points as an (N, 2) array (x, y) in base_link, for a scan from a
    laser at (laser_x, laser_y, laser_yaw)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(r.shape[0]) * angle_inc + laser_yaw
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r >= min_range) & (r <= max_range)
    r, a = r[ok], a[ok]
    return np.stack((laser_x + r * np.cos(a), laser_y + r * np.sin(a)), axis=1)


def _arc_pose(k: float, s: float) -> tuple[float, float, float]:
    if abs(k) < 1e-6:
        return s, 0.0, 0.0
    th = k * s
    return math.sin(th) / k, (1.0 - math.cos(th)) / k, th


def _as_array(points) -> np.ndarray:
    a = np.asarray(points, dtype=float)
    return a.reshape(-1, 2)


def free_distances(points, ks: Sequence[float], p: AvoidParams, horizon: float) -> np.ndarray:
    """free_distance() for several curvatures at once (numpy: every arc step x
    every point in one go; the per-point Python loop cost up to ~45 ms per
    control tick on the laptop, 2026-11-16)."""
    pts = _as_array(points)
    ks = np.asarray(ks, dtype=float).reshape(-1)
    n = max(1, int(round(horizon / p.step_m)))
    if pts.shape[0] == 0:
        return np.full(ks.shape, float(horizon))
    xf, xr, hw = p.x_front + p.margin_m, p.x_rear - p.margin_m, p.half_width + p.margin_m
    s = np.arange(1, n + 1) * horizon / n                       # (n,)
    k = ks[:, None]                                             # (K, 1)
    straight = np.abs(k) < 1e-6
    th = np.where(straight, 0.0, k * s[None, :])                # (K, n)
    ksafe = np.where(straight, 1.0, k)
    px = np.where(straight, s[None, :], np.sin(th) / ksafe)
    py = np.where(straight, 0.0, (1.0 - np.cos(th)) / ksafe)
    c, sn = np.cos(th)[..., None], np.sin(th)[..., None]        # (K, n, 1)
    dx = pts[None, None, :, 0] - px[..., None]                  # (K, n, P)
    dy = pts[None, None, :, 1] - py[..., None]
    bx = c * dx + sn * dy
    hit = ((bx <= xf) & (bx >= xr) & (np.abs(-sn * dx + c * dy) <= hw)).any(axis=2)  # (K, n)
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), n)    # index of the first hit step (i - 1)
    return np.where(first < n, first * horizon / n, float(horizon))


def free_distance(points: Sequence[tuple[float, float]], k: float, p: AvoidParams, horizon: float) -> float:
    """How far (m along the arc) the car drives on curvature k before its
    body (+ margin) touches a scan point; `horizon` if it never does."""
    return float(free_distances(points, [k], p, horizon)[0])


def near_points(points: Iterable[tuple[float, float]], p: AvoidParams, horizon: float) -> np.ndarray:
    """Only points the body can reach within the horizon."""
    pts = _as_array(list(points) if not isinstance(points, np.ndarray) else points)
    reach = horizon + max(p.x_front, -p.x_rear) + p.half_width + p.margin_m
    return pts[(pts[:, 0] ** 2 + pts[:, 1] ** 2) <= reach * reach]


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
        cands = self.candidates()
        for k, f in zip(cands, free_distances(pts, cands, p, p.horizon_m)):
            key = (float(f), -abs(k - k_pd), k)
            if best is None or key > best:
                best = key
        self.override_k = best[2]
        return ArcChoice(best[2], True, pd_free, best[0])


def spin_radius(x_front: float, x_rear: float, half_width: float) -> float:
    """Radius of the circle the farthest body corner sweeps when the car turns on
    the spot about base_link (0.46 m for a 0.70 x 0.60 m body centred on it)."""
    return math.hypot(max(abs(x_front), abs(x_rear)), half_width)


def spin_clear(points, radius: float, margin_m: float = 0.04) -> tuple[bool, float]:
    """Can the car turn on the spot? (clear, nearest scan point distance from base_link).

    The body corners sweep a circle of `radius` about base_link, so any scan
    point closer than radius + margin_m is in the way, whichever way it turns.
    Points on the car itself must already be masked (scan_self_mask)."""
    pts = _as_array(points)
    if pts.shape[0] == 0:
        return True, math.inf
    nearest = float(np.sqrt((pts[:, 0] ** 2 + pts[:, 1] ** 2).min()))
    return nearest >= radius + margin_m, nearest
