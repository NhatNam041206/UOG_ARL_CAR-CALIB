"""Simulated depth camera (Astra-like): 3D ray casting against the world, no ROS imports.

Rays leave the camera (pose from car.yaml, like the real mount) through a grid of
width x height pixels (hfov x vfov degrees). A ray hits, nearest first:

  * the floor (z = 0),
  * a box / cylinder of the world: its side walls (z between 0 and `height`) and its top,
    heights as in the world file (a mission's `extra_obstacles` and the actors, 1.8 m, included)

Hits closer than range_min or farther than range_max (in depth = distance along the optical
axis, like a real depth image) return nothing. Output: (N, 3) points in the camera's OPTICAL frame
(x right, y down, z forward), the frame the real driver's /camera/depth/points uses.
"""
from __future__ import annotations

import math

import numpy as np

from car_nav.depth_obstacles import OPTICAL_TO_LINK

from .actuator_model import Pose2D
from .world import World


def pixel_rays(width: int, height: int, hfov_deg: float, vfov_deg: float) -> np.ndarray:
    """(H*W, 3) ray directions in the optical frame with z = 1 (so t = depth)."""
    tx = math.tan(math.radians(hfov_deg) / 2.0)
    ty = math.tan(math.radians(vfov_deg) / 2.0)
    u = (np.arange(width) + 0.5) / width * 2.0 - 1.0
    v = (np.arange(height) + 0.5) / height * 2.0 - 1.0
    uu, vv = np.meshgrid(u * tx, v * ty)
    return np.stack([uu.ravel(), vv.ravel(), np.ones(uu.size)], axis=1)


def _ray_wall(ox, oy, dx, dy, x0, y0, x1, y1):
    """Parameter t along (dx, dy) (any length) to the segment, inf if none. dx, dy: (N,)."""
    ex, ey = x1 - x0, y1 - y0
    denom = dx * ey - dy * ex
    wx, wy = x0 - ox, y0 - oy
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (wx * ey - wy * ex) / denom
        u = (wx * dy - wy * dx) / denom
    ok = (np.abs(denom) > 1e-12) & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
    return np.where(ok, t, np.inf)


def depth_points(world: World, pose: Pose2D, camera, rays_opt: np.ndarray, range_min: float, range_max: float,
                 noise_std: float = 0.0, dropout: float = 0.0, rng=None) -> np.ndarray:
    """Points (N, 3) the camera at `pose` (base_link in the world) measures, optical frame."""
    rot_cam = np.array(camera.rotation())
    d_link = rays_opt @ OPTICAL_TO_LINK.T @ rot_cam.T              # base_link frame
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    d = d_link @ rz.T                                              # world frame, t = depth
    ox = pose.x + c * camera.x - s * camera.y
    oy = pose.y + s * camera.x + c * camera.y
    oz = camera.z
    dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
    best = np.full(d.shape[0], np.inf)
    # floor
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(dz < -1e-9, -oz / dz, np.inf)
    best = np.minimum(best, t_floor)
    reach = range_max * 1.6 + 1.0      # horizontal reach to look at (depth = t, rays are wider than z)
    for b in world.boxes:
        if math.hypot(b.cx - ox, b.cy - oy) > reach + math.hypot(b.sx, b.sy):
            continue
        cn = b.corners()
        for i in range(4):
            t = _ray_wall(ox, oy, dx, dy, *cn[i], *cn[(i + 1) % 4])
            z = oz + t * dz
            best = np.minimum(best, np.where((z >= 0.0) & (z <= b.height), t, np.inf))
        # top face: where the ray crosses z = height, inside the footprint
        with np.errstate(divide="ignore", invalid="ignore"):
            t_top = np.where(dz < -1e-9, (b.height - oz) / dz, np.inf)
        tt = np.where(np.isfinite(t_top), t_top, 0.0)
        hx, hy = ox + tt * dx, oy + tt * dy
        cy_, sy_ = math.cos(b.yaw), math.sin(b.yaw)
        lx = (hx - b.cx) * cy_ + (hy - b.cy) * sy_
        ly = -(hx - b.cx) * sy_ + (hy - b.cy) * cy_
        inside = (np.abs(lx) <= b.sx / 2.0) & (np.abs(ly) <= b.sy / 2.0) & np.isfinite(t_top) & (t_top > 0)
        best = np.minimum(best, np.where(inside, t_top, np.inf))
    for cyl in world.cylinders:
        if math.hypot(cyl.cx - ox, cyl.cy - oy) > reach + cyl.r:
            continue
        fx, fy = ox - cyl.cx, oy - cyl.cy
        a = dx * dx + dy * dy
        bq = fx * dx + fy * dy
        cq = fx * fx + fy * fy - cyl.r * cyl.r
        disc = bq * bq - a * cq
        with np.errstate(invalid="ignore", divide="ignore"):
            sq = np.sqrt(np.where(disc >= 0, disc, np.nan))
            t0 = (-bq - sq) / a
            t1 = (-bq + sq) / a
        t = np.where(t0 > 1e-6, t0, np.where(t1 > 1e-6, t1, np.inf))
        t = np.where(np.isnan(t), np.inf, t)
        z = oz + t * dz
        best = np.minimum(best, np.where((z >= 0.0) & (z <= cyl.height), t, np.inf))
        with np.errstate(divide="ignore", invalid="ignore"):
            t_top = np.where(dz < -1e-9, (cyl.height - oz) / dz, np.inf)
        inside = np.hypot(ox + np.where(np.isfinite(t_top), t_top, 0.0) * dx - cyl.cx,
                          oy + np.where(np.isfinite(t_top), t_top, 0.0) * dy - cyl.cy) <= cyl.r
        best = np.minimum(best, np.where(inside & np.isfinite(t_top) & (t_top > 0), t_top, np.inf))
    ok = np.isfinite(best) & (best >= range_min) & (best <= range_max)
    if dropout > 0 and rng is not None:
        ok &= rng.random(ok.shape) >= dropout
    t = best[ok]
    if noise_std > 0 and rng is not None:
        t = t + rng.normal(0.0, noise_std, t.shape)
    return rays_opt[ok] * t[:, None]
