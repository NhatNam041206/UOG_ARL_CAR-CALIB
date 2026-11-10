"""Pure geometry helpers shared by the car_nav nodes (no ROS imports).

Kept ROS-free so the repo's normal pytest run can test them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Pose3D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def rotation(self) -> tuple[tuple[float, float, float], ...]:
        """R = Rz(yaw) * Ry(pitch) * Rx(roll) (ROS fixed-axis RPY)."""
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )


@dataclass(frozen=True)
class Footprint:
    x_front: float
    x_rear: float
    half_width: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            self.x_rear - margin <= x <= self.x_front + margin
            and abs(y) <= self.half_width + margin
        )


def beam_to_base(r: float, angle: float, laser: Pose3D) -> tuple[float, float]:
    """One laser beam (range, angle in the laser frame) -> (x, y) in base_link."""
    px, py = r * math.cos(angle), r * math.sin(angle)
    rot = laser.rotation()
    bx = rot[0][0] * px + rot[0][1] * py + laser.x
    by = rot[1][0] * px + rot[1][1] * py + laser.y
    return bx, by


def bearing_deg(x: float, y: float) -> float:
    """base_link bearing in degrees: 0 = straight ahead, +90 = left."""
    return math.degrees(math.atan2(y, x))


def _in_sector(bearing: float, sector: Sequence[float]) -> bool:
    lo, hi = float(sector[0]), float(sector[1])
    b = (bearing - lo) % 360.0
    return b <= (hi - lo) % 360.0


def self_mask(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    laser: Pose3D,
    footprint: Footprint,
    margin: float = 0.03,
    sectors_deg: Iterable[Sequence[float]] = (),
) -> tuple[list[float], int]:
    """Replace beams that hit the car itself with +inf.

    A beam is masked when its end point lies inside the footprint (+margin)
    or its base_link bearing falls in one of `sectors_deg`.
    Returns (new_ranges, masked_count).
    """
    sectors = [tuple(s) for s in sectors_deg]
    out: list[float] = []
    masked = 0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            out.append(r)
            continue
        bx, by = beam_to_base(r, angle_min + i * angle_increment, laser)
        drop = footprint.contains(bx, by, margin)
        if not drop and sectors:
            b = bearing_deg(bx, by)
            drop = any(_in_sector(b, s) for s in sectors)
        if drop:
            out.append(math.inf)
            masked += 1
        else:
            out.append(r)
    return out, masked


def front_distance(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    laser: Pose3D,
    corridor_half_width: float,
    x_min: float = 0.0,
) -> tuple[float | None, int]:
    """Nearest x (base_link) of points straight ahead inside a corridor.

    Returns (distance or None, number of points used).
    """
    best: float | None = None
    n = 0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        bx, by = beam_to_base(r, angle_min + i * angle_increment, laser)
        if bx > x_min and abs(by) <= corridor_half_width:
            n += 1
            if best is None or bx < best:
                best = bx
    return best, n
