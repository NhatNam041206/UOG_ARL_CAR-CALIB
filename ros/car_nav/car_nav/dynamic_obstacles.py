"""Which scan points are NOT on the static map (pure math, no ROS imports).

Used by the safety guard (dynamic_scan.py -> /scan_dynamic -> the
collision monitor's PersonStop / PersonSlow zones): walls and furniture
are on the map, a person walking in front of the car is not. A point counts
as static when an occupied map cell lies within `tol_m` of it, so a pose
error up to ~tol_m does not turn walls into "people".
"""
from __future__ import annotations

import math

import numpy as np


def static_mask(occ: np.ndarray, resolution: float, tol_m: float, occupied: int = 65,
                unknown_is_static: bool = True) -> np.ndarray:
    """(H, W) bool: True where an occupied cell (or unknown, if asked) is within tol_m.

    occ is the OccupancyGrid data as (H, W) int (row = y index), -1 = unknown."""
    occ = np.asarray(occ)
    base = occ >= occupied
    if unknown_is_static:
        base = base | (occ < 0)
    r = int(math.ceil(tol_m / resolution))
    out = base.copy()
    h, w = base.shape
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r or (dx == 0 and dy == 0):
                continue
            ys0, ys1 = max(0, -dy), min(h, h - dy)
            xs0, xs1 = max(0, -dx), min(w, w - dx)
            out[ys0 + dy:ys1 + dy, xs0 + dx:xs1 + dx] |= base[ys0:ys1, xs0:xs1]
    return out


def dynamic_beams(ranges, angle_min: float, angle_inc: float, sensor_pose: tuple[float, float, float],
                  mask: np.ndarray, origin: tuple[float, float], resolution: float,
                  max_range: float = 4.0) -> np.ndarray:
    """Bool per beam: True = the point is off the static map (a person, a box someone put down).

    sensor_pose: (x, y, yaw) of the scan frame in the map frame. Points outside
    the map count as dynamic (nothing static is known there)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(r.shape[0]) * angle_inc + sensor_pose[2]
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(r) & (r > 0.0) & (r <= max_range)
    x = sensor_pose[0] + np.where(valid, r, 0.0) * np.cos(a)
    y = sensor_pose[1] + np.where(valid, r, 0.0) * np.sin(a)
    ix = np.floor((x - origin[0]) / resolution).astype(int)
    iy = np.floor((y - origin[1]) / resolution).astype(int)
    h, w = mask.shape
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    static = np.zeros(r.shape, dtype=bool)
    static[inside] = mask[iy[inside], ix[inside]]
    return valid & ~static


def dynamic_points(xy_base: np.ndarray, base_pose: tuple[float, float, float], mask: np.ndarray,
                   origin: tuple[float, float], resolution: float) -> np.ndarray:
    """Bool per point: True = the (N, 2) base_link point is off the static map.

    base_pose: (x, y, yaw) of base_link in the map frame. Same rule as dynamic_beams
    (a point outside the map counts as dynamic)."""
    pts = np.asarray(xy_base, dtype=float).reshape(-1, 2)
    c, s = math.cos(base_pose[2]), math.sin(base_pose[2])
    x = base_pose[0] + c * pts[:, 0] - s * pts[:, 1]
    y = base_pose[1] + s * pts[:, 0] + c * pts[:, 1]
    ix = np.floor((x - origin[0]) / resolution).astype(int)
    iy = np.floor((y - origin[1]) / resolution).astype(int)
    h, w = mask.shape
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    static = np.zeros(pts.shape[0], dtype=bool)
    static[inside] = mask[iy[inside], ix[inside]]
    return ~static


def align_to_map(ranges, angle_min: float, angle_inc: float, sensor_pose: tuple[float, float, float],
                 mask: np.ndarray, origin: tuple[float, float], resolution: float,
                 search_m: float = 0.45, step_m: float = 0.075, search_rad: float = 0.12,
                 step_rad: float = 0.04, max_range: float = 6.0) -> tuple[float, float, float]:
    """Sensor pose corrected so the scan best fits the map walls (grid search).

    The pose estimate can be 0.3-0.5 m off (blind corridors, AMCL updates
    only while driving). Classifying with that pose turned walls into
    "people" and the car waited for them forever (sim 2026-11-17). So the
    scan is first slid over the map: the shift (within +-search_m,
    +-search_rad) that puts the most points on `mask` (a THIN wall mask, a
    few cm) wins; ties keep the estimate. Returns the corrected (x, y, yaw)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(r.shape[0]) * angle_inc
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r > 0.05) & (r <= max_range)
    r, a = r[ok], a[ok]
    if r.shape[0] > 360:                    # every other point is plenty
        r, a = r[::2], a[::2]
    return align_points(r, a, sensor_pose, mask, origin, resolution, search_m, step_m, search_rad, step_rad)


def align_points(r: np.ndarray, a: np.ndarray, sensor_pose: tuple[float, float, float], mask: np.ndarray,
                 origin: tuple[float, float], resolution: float, search_m: float = 0.45, step_m: float = 0.075,
                 search_rad: float = 0.12, step_rad: float = 0.04) -> tuple[float, float, float]:
    """align_to_map for points given as (range, angle) pairs in the sensor frame (at most a few
    hundred; fewer than 20 keep the estimate). Used by the depth camera, whose points are not
    regular beams."""
    if r.shape[0] < 20:
        return sensor_pose
    h, w = mask.shape
    best = (-1, 0.0, 0.0, 0.0)
    n_xy = int(round(search_m / step_m))
    offs = np.arange(-n_xy, n_xy + 1) * step_m
    n_th = int(round(search_rad / step_rad))
    for dth in np.arange(-n_th, n_th + 1) * step_rad:
        ang = a + sensor_pose[2] + dth
        px = r * np.cos(ang)
        py = r * np.sin(ang)
        # all x/y shifts at once: (S, N)
        gx = np.floor((sensor_pose[0] + offs[:, None] + px[None, :] - origin[0]) / resolution).astype(int)
        gy = np.floor((sensor_pose[1] + offs[:, None] + py[None, :] - origin[1]) / resolution).astype(int)
        inx = (gx >= 0) & (gx < w)
        iny = (gy >= 0) & (gy < h)
        hx = np.where(inx, gx, 0)
        hy = np.where(iny, gy, 0)
        # score[i, j] = points on a wall with shift (offs[i], offs[j])
        hit = mask[hy[None, :, :], hx[:, None, :]] & iny[None, :, :] & inx[:, None, :]
        score = hit.sum(axis=2)                       # (Sx, Sy)
        # small preference for the estimate itself on ties
        pen = (np.abs(offs)[:, None] + np.abs(offs)[None, :]) / step_m * 1e-3 + abs(dth) / step_rad * 1e-3
        val = score - pen
        i, j = np.unravel_index(int(np.argmax(val)), val.shape)
        if val[i, j] > best[0]:
            best = (val[i, j], offs[i], offs[j], dth)
    _, dx, dy, dth = best
    return sensor_pose[0] + dx, sensor_pose[1] + dy, sensor_pose[2] + dth
