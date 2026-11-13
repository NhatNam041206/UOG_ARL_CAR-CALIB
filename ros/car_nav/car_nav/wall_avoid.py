"""Reactive wall avoidance from the LiDAR scan (pure math, no ROS imports).

A safety layer under the PD path follower: it does not need to know where the
car is on the map, only how close the walls are right now. When the planned
path (or a wrong pose) takes the car too close to a wall, it steers away
with a FIXED, moderate curvature; with no wall near it does nothing.

Fixed turn with hysteresis (nam, 2026-11-13: a turn that grows as the wall
gets closer ends in bang-bang steering):

    gap      distance from a scan point to the front of the car's body:
             gap_x = max(x - x_front, 0)  (ahead of the nose),
             gap_y = max(|y| - half_width, 0)  (beside the body)
    d        = hypot(gap_x / front_exit_m, gap_y / side_exit_m)
             (1 = on the edge of the zone; < 1 inside it)
    d_L, d_R = smallest d of the points LEFT (y >= 0) / RIGHT of the car

    idle   -> avoiding a side   when its d < enter_ratio (e.g. 0.67: a wall
                                0.30 m beside the body or 1.07 m ahead)
    avoiding -> idle            only when that side's d >= 1 again
                                (0.45 m beside / 1.6 m ahead)

    while avoiding the LEFT wall:  kappa = min(kappa_pd, -turn_curvature)
    while avoiding the RIGHT wall: kappa = max(kappa_pd, +turn_curvature)

so the car turns away at least turn_curvature, more only if the PD itself
already turns away harder. Between the two thresholds nothing changes,
which keeps it from switching on and off every scan. Both sides inside
the zone about equally (an end wall ahead): it turns the way the path turns.

Only points AHEAD of the front axle (x >= 0) count: the car steers with
its REAR wheels, so turning away from a wall swings the rear TOWARDS it.
Points beside the rear limit that (rear guard): when the rear gap on the
side the rear swings to is below rear_margin_m, the fixed turn is scaled
down so the layer never pushes the rear into the wall to save the nose.

Units: metres, 1/m. base_link = floor point under the front axle, x forward,
y left. Scan points must already be in base_link (see points_in_base()).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class AvoidParams:
    x_front: float = 0.16          # nose, from car.yaml footprint
    x_rear: float = -0.37
    half_width: float = 0.135
    front_exit_m: float = 1.6       # zone edge ahead of the nose
    side_exit_m: float = 0.45       # zone edge beside the body
    enter_ratio: float = 0.67       # start avoiding at this fraction of the zone (0.30 m beside, 1.07 m ahead)
    look_ahead_m: float = 2.0       # ignore points further ahead of the front axle
    turn_curvature: float = 0.4     # 1/m, the fixed turn away (about 1/3 of full lock)
    max_curvature: float = 1.33     # 1/m, full lock (1 / min turning radius 0.75)
    rear_margin_m: float = 0.12     # rear guard
    tie_ratio: float = 0.15         # |d_L - d_R| below this: wall straight ahead, follow the path's turn


@dataclass
class WallDist:
    left: float = math.inf         # smallest normalised distance d of a point on the left (1 = zone edge)
    right: float = math.inf
    rear_gap_left: float = math.inf   # gap beside the rear half of the body (m)
    rear_gap_right: float = math.inf
    nearest_m: float = math.inf    # smallest gap ahead (m, for logs)


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


def wall_distances(points: Iterable[tuple[float, float]], p: AvoidParams) -> WallDist:
    res = WallDist()
    for x, y in points:
        gap_y = max(abs(y) - p.half_width, 0.0)
        if p.x_rear <= x < 0.0:
            # beside the rear half: only feeds the rear guard
            if y > 0:
                res.rear_gap_left = min(res.rear_gap_left, gap_y)
            else:
                res.rear_gap_right = min(res.rear_gap_right, gap_y)
            continue
        if x < 0.0 or x > p.look_ahead_m:
            continue
        gap_x = max(x - p.x_front, 0.0)
        d = math.hypot(gap_x / p.front_exit_m, gap_y / p.side_exit_m)
        if d >= 1.5:
            continue
        res.nearest_m = min(res.nearest_m, math.hypot(gap_x, gap_y))
        if y >= 0.0:
            res.left = min(res.left, d)
        else:
            res.right = min(res.right, d)
    return res


class WallAvoider:
    """Fixed-turn avoidance with hysteresis. `side` is None, "left" or "right"
    (the wall being avoided)."""

    def __init__(self, p: AvoidParams) -> None:
        self.p = p
        self.side: str | None = None

    def reset(self) -> None:
        self.side = None

    def step(self, kappa_pd: float, w: WallDist) -> float:
        p = self.p
        if self.side is not None:
            if (w.left if self.side == "left" else w.right) >= 1.0:
                self.side = None
        if self.side is None:
            near_l, near_r = w.left < p.enter_ratio, w.right < p.enter_ratio
            if near_l and near_r and abs(w.left - w.right) < p.tie_ratio:
                # wall straight ahead: avoid the side the path does not turn to
                self.side = "right" if kappa_pd >= 0.0 else "left"
            elif near_l or near_r:
                self.side = "left" if w.left <= w.right else "right"
        if self.side is None:
            return kappa_pd
        k = p.turn_curvature
        # rear guard: turning right swings the rear left, and the other way round
        rear_gap = w.rear_gap_left if self.side == "left" else w.rear_gap_right
        if rear_gap < p.rear_margin_m:
            k *= max(rear_gap, 0.0) / p.rear_margin_m
        kappa = min(kappa_pd, -k) if self.side == "left" else max(kappa_pd, k)
        return max(-p.max_curvature, min(p.max_curvature, kappa))
