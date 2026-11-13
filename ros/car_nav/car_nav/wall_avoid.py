"""Reactive wall avoidance from the LiDAR scan (pure math, no ROS imports).

A safety layer under the PD path follower: it does not need to know where the
car is on the map, only how close the walls are right now. When the planned
path (or a wrong pose) steers the car towards a wall, it bends the steering
away from it; with no wall near it does nothing.

Classical repulsive field, one value per side:

    gap      distance from a scan point to the front of the car's body:
             gap_x = max(x - x_front, 0)  (ahead of the nose),
             gap_y = max(|y| - half_width, 0)  (beside the body)
    d        = hypot(gap_x / front_influence, gap_y / side_influence)   (0..1 = inside the zone)
    push     = 1 - d              for d < 1, else 0
    push_L / push_R = strongest push from a point LEFT (y > 0) / RIGHT of the car

    a        = max(push_L, push_R)                    how much the layer takes over (0..1)
    steer    = (push_R - push_L) / a                  -1 .. +1, + = turn left (away from the right wall)
               (both sides blocked about equally, e.g. an end wall: turn the way the path turns)
    kappa    = (1 - a) * kappa_pd + a * steer * max_curvature

Only points AHEAD of the front axle (x >= 0) push: the car steers with its
REAR wheels, so turning away from a wall swings the rear TOWARDS it. Points
beside the rear instead limit that swing (rear guard): turning away from a
side whose rear gap is below rear_margin is scaled down, so the layer never
pushes the rear into the wall to save the nose.

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
    front_influence_m: float = 1.0  # a wall this far ahead of the nose starts to push
    side_influence_m: float = 0.45  # a wall this far beside the body starts to push
    look_ahead_m: float = 1.5       # ignore points further ahead of the front axle
    max_curvature: float = 1.33     # 1/m, steering at full lock (1 / min turning radius 0.75)
    rear_margin_m: float = 0.12     # rear guard: rear gap below this stops the swing towards it
    tie_ratio: float = 0.2          # |push_R - push_L| < tie_ratio * a counts as "both sides"


@dataclass
class WallPush:
    left: float = 0.0              # 0..1, strongest push from a wall on the left
    right: float = 0.0
    rear_gap_left: float = math.inf   # gap beside the rear half of the body (m)
    rear_gap_right: float = math.inf
    nearest_m: float = math.inf    # smallest gap ahead (for logs)

    @property
    def active(self) -> float:
        return max(self.left, self.right)


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


def wall_push(points: Iterable[tuple[float, float]], p: AvoidParams) -> WallPush:
    res = WallPush()
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
        d = math.hypot(gap_x / p.front_influence_m, gap_y / p.side_influence_m)
        if d >= 1.0:
            continue
        res.nearest_m = min(res.nearest_m, math.hypot(gap_x, gap_y))
        push = 1.0 - d
        if y >= 0.0:
            res.left = max(res.left, push)
        else:
            res.right = max(res.right, push)
    return res


def blend(kappa_pd: float, push: WallPush, p: AvoidParams) -> tuple[float, float]:
    """(commanded curvature, a = share of the avoidance layer 0..1)."""
    a = push.active
    if a <= 0.0:
        return kappa_pd, 0.0
    diff = push.right - push.left
    if min(push.left, push.right) > 0.0 and abs(diff) < p.tie_ratio * a:
        # wall straight ahead on both sides: turn the way the path wants to
        diff = a if kappa_pd >= 0.0 else -a
    steer = max(-1.0, min(1.0, diff / a))
    # rear guard: turning left swings the rear right, and the other way round
    rear_gap = push.rear_gap_right if steer > 0 else push.rear_gap_left
    if rear_gap < p.rear_margin_m:
        steer *= max(rear_gap, 0.0) / p.rear_margin_m
    kappa = (1.0 - a) * kappa_pd + a * steer * p.max_curvature
    return max(-p.max_curvature, min(p.max_curvature, kappa)), a
