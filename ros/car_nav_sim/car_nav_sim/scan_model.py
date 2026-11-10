"""Simulated LiDAR measurement in the laser's own frame (no ROS imports)."""
from __future__ import annotations

import math

import numpy as np

from .actuator_model import Pose2D
from .world import World


def laser_scan_ranges(world: World, pose: Pose2D, laser, angles: np.ndarray, lidar: dict, rng) -> np.ndarray:
    """Ranges the (possibly upside-down) laser measures at `pose`."""
    rot = laser.rotation()
    bx = rot[0][0] * np.cos(angles) + rot[0][1] * np.sin(angles)
    by = rot[1][0] * np.cos(angles) + rot[1][1] * np.sin(angles)
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    ox = pose.x + c * laser.x - s * laser.y
    oy = pose.y + s * laser.x + c * laser.y
    world_dirs = pose.theta + np.arctan2(by, bx)
    r = world.raycast(ox, oy, world_dirs, float(lidar.get("range_max_m", 12.0)))
    noise = float(lidar.get("noise_std_m", 0.0))
    if noise > 0:
        r = r + rng.normal(0.0, noise, r.shape)
    drop = float(lidar.get("dropout", 0.0))
    if drop > 0:
        r[rng.random(r.shape) < drop] = np.inf
    r[r < float(lidar.get("range_min_m", 0.15))] = np.inf
    return r
