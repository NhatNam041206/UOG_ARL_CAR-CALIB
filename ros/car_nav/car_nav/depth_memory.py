"""Short-term memory of depth obstacles (pure math, no ROS imports).

The depth camera cannot see what is close to the car: it looks 15 deg down, so a low box
leaves the view ~1 m past the nose and the guard would see it "disappear" and drive on.
This keeps the obstacle points in a fixed frame (odom, the car's odometry frame) so they
are still there when the car is next to them.

A remembered point is forgotten when

* the camera sees free space where it is: the point is inside the current view (farther than
  the camera's blind distance for the floor, within the horizontal field of view and the
  max range), no live point is near it, and no nearer live point hides it (occlusion), for
  `miss_frames` frames in a row,
* it is taller than the LiDAR plane (`lidar_z` + margin) and the LiDAR beam through it sees on past
  it (or nothing) for `miss_frames` frames: a person who left the blind zone is seen going by the
  LiDAR, while a low box (under the plane, the beam passes over it) is never cleared this way,
* it is behind the car (x < x_rear in base_link),
* it is older than `timeout_s` AND outside the protected region (the stop / slow zone in front
  of the car: something the car stands in front of is never forgotten by time, or a stopped
  car would drive into it once the timer ran out; clear it by hand, ObstacleMemory.clear()),
* the memory is over `max_points` (the points nearest to the car are kept).

Points are on a `voxel_m` grid (one per cell, the latest wins). Odometry drift is small over
the seconds an obstacle is near the car.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .depth_obstacles import blind_distance
from .geometry import Pose3D


@dataclass
class MemoryParams:
    timeout_s: float = 30.0
    max_points: int = 600
    voxel_m: float = 0.05
    miss_frames: int = 3             # "seen free" this many frames in a row before forgetting
    near_live_m: float = 0.12        # a live point this close to a remembered one = still there
    hfov_deg: float = 58.0
    vfov_deg: float = 45.0
    view_margin_m: float = 0.1       # inside the view by this much before free space counts
    view_margin_deg: float = 3.0
    lidar_margin_m: float = 0.05     # taller than lidar_z + this: the LiDAR would see it
    lidar_through_m: float = 0.25    # the beam reaches this much farther than the point = sees through


def to_odom(xy_base: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    p = np.asarray(xy_base, dtype=float).reshape(-1, 2)
    return np.stack([pose[0] + c * p[:, 0] - s * p[:, 1], pose[1] + s * p[:, 0] + c * p[:, 1]], axis=1)


def to_base(xy_odom: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = np.asarray(xy_odom, dtype=float).reshape(-1, 2) - np.array(pose[:2])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)


class ObstacleMemory:
    def __init__(self, camera: Pose3D, x_rear: float, params: MemoryParams | None = None) -> None:
        self.camera = camera
        self.x_rear = x_rear
        self.p = params or MemoryParams()
        self.xy = np.zeros((0, 2))       # odom frame
        self.t = np.zeros(0)             # last time seen
        self.miss = np.zeros(0, dtype=int)
        self.h = np.zeros(0)             # highest z seen in the cell

    def clear(self) -> None:
        self.xy, self.t, self.miss = np.zeros((0, 2)), np.zeros(0), np.zeros(0, dtype=int)
        self.h = np.zeros(0)

    def _lidar_sees_through(self, base: np.ndarray, scan, laser: tuple[float, float, float]) -> np.ndarray:
        """Bool per base_link point: the LiDAR beam towards it passes beyond it (or returns nothing)."""
        ranges, a0, inc = scan
        r = np.asarray(ranges, dtype=float)
        c, s = math.cos(-laser[2]), math.sin(-laser[2])
        dx, dy = base[:, 0] - laser[0], base[:, 1] - laser[1]
        lx, ly = c * dx - s * dy, s * dx + c * dy
        ang = np.arctan2(ly, lx)
        rng = np.hypot(lx, ly)
        k = np.round(((ang - a0) % (2 * math.pi)) / inc).astype(int)
        ok = (k >= 0) & (k < r.shape[0])
        beam = np.full(base.shape[0], np.nan)
        beam[ok] = r[k[ok]]
        # a beam that hit something at about the point's range = still there; beyond / no return = free
        return ok & (rng > 0.3) & (~(beam > 0) | (beam > rng + self.p.lidar_through_m))

    def __len__(self) -> int:
        return int(self.xy.shape[0])

    def observable(self, base: np.ndarray, min_height_m: float, max_range_m: float) -> np.ndarray:
        """Bool per base_link point: the camera can show free space there (floor visible,
        inside the horizontal field of view and the range)."""
        p, c = self.p, self.camera
        blind = blind_distance(c, p.vfov_deg, min_height_m) + p.view_margin_m
        dx, dy = base[:, 0] - c.x, base[:, 1] - c.y
        bearing = np.degrees(np.arctan2(dy, np.maximum(dx, 1e-9)))
        return ((base[:, 0] > blind) & (np.hypot(base[:, 0], base[:, 1]) <= max_range_m - p.view_margin_m)
                & (np.abs(bearing) <= p.hfov_deg / 2.0 - p.view_margin_deg) & (dx > 0))

    def _hidden_or_present(self, base: np.ndarray, live_base: np.ndarray) -> np.ndarray:
        """Per remembered point: a live point is near it, or a nearer live point hides it."""
        if live_base.shape[0] == 0 or base.shape[0] == 0:
            return np.zeros(base.shape[0], dtype=bool)
        c = self.camera
        d = np.hypot(base[:, None, 0] - live_base[None, :, 0], base[:, None, 1] - live_base[None, :, 1])
        near = (d <= self.p.near_live_m).any(axis=1)
        ang_m = np.arctan2(base[:, 1] - c.y, base[:, 0] - c.x)
        ang_l = np.arctan2(live_base[:, 1] - c.y, live_base[:, 0] - c.x)
        r_m = np.hypot(base[:, 0] - c.x, base[:, 1] - c.y)
        r_l = np.hypot(live_base[:, 0] - c.x, live_base[:, 1] - c.y)
        same_ray = np.abs(ang_m[:, None] - ang_l[None, :]) <= math.radians(1.5)
        hidden = (same_ray & (r_l[None, :] < r_m[:, None] - 0.05)).any(axis=1)
        return near | hidden

    def update(self, live_base: np.ndarray, pose_odom: tuple[float, float, float], now: float,
               min_height_m: float, max_range_m: float, protect: tuple[float, float] | None = None,
               scan=None, laser: tuple[float, float, float] = (0.0, 0.0, 0.0), lidar_z: float = 0.0) -> np.ndarray:
        """Take one live frame (2D points in base_link, may be empty) seen from pose_odom (base_link in
        the odom frame); returns everything that is known now, in base_link (live + remembered).

        live_base: (M, 2) or (M, 3) with the highest z of each cell as the third column.
        protect = (x_max, half_width) in base_link: points inside never expire by time.
        scan = (ranges, angle_min, angle_increment) of the LiDAR (None = not used), laser = its
        (x, y, yaw) in base_link, lidar_z = its height."""
        p = self.p
        raw = np.asarray(live_base, dtype=float)
        raw = raw.reshape(-1, raw.shape[1] if raw.ndim == 2 and raw.shape[1] in (2, 3) else 2)
        live = raw[:, :2]
        live_h = raw[:, 2] if raw.shape[1] == 3 else np.zeros(raw.shape[0])
        if len(self):
            base = to_base(self.xy, pose_odom)
            seen_free = self.observable(base, min_height_m, max_range_m) & ~self._hidden_or_present(base, live)
            if scan is not None:
                tall = self.h > lidar_z + p.lidar_margin_m
                seen_free |= tall & self._lidar_sees_through(base, scan, laser)
            self.miss = np.where(seen_free, self.miss + 1, 0)
            old = now - self.t > p.timeout_s
            if protect is not None:
                prot = (base[:, 0] <= protect[0]) & (base[:, 0] >= self.x_rear) & (np.abs(base[:, 1]) <= protect[1])
                old &= ~prot
            keep = ~(old | (base[:, 0] < self.x_rear) | (self.miss >= p.miss_frames))
            self.xy, self.t, self.miss, self.h = self.xy[keep], self.t[keep], self.miss[keep], self.h[keep]
        if live.shape[0]:
            xy = np.vstack([self.xy, to_odom(live, pose_odom)])
            t = np.concatenate([self.t, np.full(live.shape[0], now)])
            miss = np.concatenate([self.miss, np.zeros(live.shape[0], dtype=int)])
            hh = np.concatenate([self.h, live_h])
            ij = np.floor(xy / p.voxel_m).astype(np.int64)
            # the latest entry of a cell wins: unique on the reversed array
            _, first = np.unique(ij[::-1], axis=0, return_index=True)
            idx = xy.shape[0] - 1 - first
            self.xy, self.t, self.miss, self.h = xy[idx], t[idx], miss[idx], hh[idx]
        if len(self) > p.max_points:
            d = np.hypot(*to_base(self.xy, pose_odom).T)
            idx = np.argsort(d)[:p.max_points]
            self.xy, self.t, self.miss, self.h = self.xy[idx], self.t[idx], self.miss[idx], self.h[idx]
        return to_base(self.xy, pose_odom) if len(self) else np.zeros((0, 2))
