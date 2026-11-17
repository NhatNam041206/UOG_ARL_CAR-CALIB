"""Upside-down LiDAR -> upright virtual scan (no ROS imports).

The RPLIDAR A1 is mounted upside down (laser frame roll = pi in car.yaml).
Some consumers only use the YAW of the laser -> base_link transform; rf2o
laser odometry is one (it calls getYaw() on that pose), so it would see
every turn mirrored. To keep every Nav2 consumer correct, the scan is
re-expressed in a virtual upright frame `laser_upright` that sits at the
same place with roll = pitch = 0 and the same yaw:

    inverted laser: R = Rz(yaw) Rx(pi), beam a -> base bearing yaw - a
    upright frame:  R = Rz(yaw),        beam a' -> base bearing yaw + a'
    so a' = -a: reverse the ranges and mirror the angle limits.
"""
from __future__ import annotations

import math
from typing import Sequence


def is_inverted(roll: float, pitch: float = 0.0, tol: float = 0.05) -> bool:
    if abs(pitch) > tol:
        raise ValueError("scan_upright only supports a level laser (pitch = 0)")
    r = math.remainder(roll, 2.0 * math.pi)
    if abs(abs(r) - math.pi) < tol:
        return True
    if abs(r) < tol:
        return False
    raise ValueError(f"laser roll {roll} is neither upright (0) nor upside down (pi)")


def upright(
    ranges: Sequence[float],
    intensities: Sequence[float],
    angle_min: float,
    angle_max: float,
    inverted: bool,
) -> tuple[list[float], list[float], float, float]:
    """Return (ranges, intensities, angle_min, angle_max) in the upright frame."""
    if not inverted:
        return list(ranges), list(intensities), angle_min, angle_max
    return list(reversed(ranges)), list(reversed(intensities)), -angle_max, -angle_min


def zero_invalid(ranges: Sequence[float]) -> list[float]:
    """inf / nan / negative -> 0.0 (rf2o's "no return")."""
    return [r if math.isfinite(r) and r > 0.0 else 0.0 for r in ranges]
