"""PD path follower for the rear-steer car (pure math, no ROS imports).

Classical PD on the path, the same idea as the VISION lane PID:

    kappa_cmd = ff * kappa_path  -  kp * e  -  kd * sin(psi_e)

  e       sideways offset of base_link from the path (m, + = car LEFT of path)
  psi_e   heading error, car heading minus path heading (rad, + = car points
          LEFT of the path). For the car, d(e)/ds = sin(psi_e): the heading
          error IS the derivative of the offset, so the kd term is the D term.
  kappa_path  path curvature just ahead (feed-forward, so a planned curve is
          followed without first building an error).
  kappa_cmd   commanded path curvature (1/m, + = turn left); cmd_vel carries it
          as omega = v * kappa_cmd and actuation.py turns it into a servo angle.

Units: kp in 1/m^2, kd in 1/m. Measured along the path (not time), the error
obeys e'' + kd e' + kp e = 0, so the gains do not depend on speed: kp sets how
quickly the car returns to the path (it settles in roughly 2*pi/sqrt(kp) m),
kd = 2*sqrt(kp) is "critically damped" (no overshoot without delay). The
0.65 s command delay is a dead distance of v*0.65 m (0.33 m at 0.5 m/s); more
kd is what keeps the car from rocking because of it.

base_link is the floor point under the FRONT (fixed) axle, so these are the
exact kinematics for this car (x' = v cos th, y' = v sin th, th' = v kappa).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class PDGains:
    kp: float = 1.0          # 1/m^2
    kd: float = 2.0          # 1/m
    ff: float = 1.0          # path-curvature feed-forward weight (0 = pure PD)
    ff_ahead_m: float = 0.3  # where the feed-forward curvature is read, ahead of the car
    ff_window_m: float = 0.4 # curvature = heading change over this much path


@dataclass
class Tracking:
    index: int          # path segment the car is on
    e: float            # sideways offset (m, + = left of path)
    psi_e: float        # heading error (rad, + = car points left of path)
    kappa_path: float   # path curvature ahead (1/m)
    s: float            # distance along the path of the car's projection (m)
    remaining: float    # path length left after the projection (m)


class PathTracker:
    """Projects the car onto a path given as [(x, y), ...] (map frame).

    The search only moves forward from the last segment (plus a short
    window), so on a path that loops back near itself the car does not
    jump to a later or earlier pass.
    """

    def __init__(self, pts: Sequence[tuple[float, float]], search_ahead_m: float = 2.0) -> None:
        clean: list[tuple[float, float]] = []
        for p in pts:
            if not clean or math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1e-4:
                clean.append((float(p[0]), float(p[1])))
        if len(clean) == 1:
            clean.append((clean[0][0] + 1e-3, clean[0][1]))
        self.pts = clean
        self.s = [0.0]
        for a, b in zip(clean, clean[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.s[-1]
        self.heading = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(clean, clean[1:])]
        self.search_ahead_m = search_ahead_m
        self.index = 0

    def _heading_at(self, s: float) -> float:
        s = min(max(s, 0.0), self.length)
        i = max(0, min(len(self.heading) - 1, _bisect(self.s, s) - 1))
        return self.heading[i]

    def curvature_at(self, s: float, window: float) -> float:
        a, b = max(s - window / 2, 0.0), min(s + window / 2, self.length)
        if b - a < 1e-3:
            return 0.0
        return wrap(self._heading_at(b) - self._heading_at(a)) / (b - a)

    def track(self, x: float, y: float, th: float, gains: PDGains) -> Tracking:
        best = None
        i = self.index
        s_limit = self.s[self.index] + self.search_ahead_m
        while i < len(self.heading) and self.s[i] <= s_limit:
            (ax, ay), (bx, by) = self.pts[i], self.pts[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = min(max(((x - ax) * dx + (y - ay) * dy) / seg2, 0.0), 1.0)
            px, py = ax + t * dx, ay + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if best is None or d2 < best[0] - 1e-12:
                best = (d2, i, t)
            i += 1
        _, i, t = best
        self.index = i
        (ax, ay), (bx, by) = self.pts[i], self.pts[i + 1]
        hd = self.heading[i]
        # signed offset: + when the car is to the LEFT of the path direction
        e = math.cos(hd) * (y - ay) - math.sin(hd) * (x - ax)
        seg_len = self.s[i + 1] - self.s[i]
        s = self.s[i] + t * seg_len
        # past the last point: extend the last segment (offset still valid)
        if i == len(self.heading) - 1 and t >= 1.0:
            s = self.s[i] + (math.cos(hd) * (x - ax) + math.sin(hd) * (y - ay))
        kappa = self.curvature_at(s + gains.ff_ahead_m, gains.ff_window_m)
        return Tracking(index=i, e=e, psi_e=wrap(th - hd), kappa_path=kappa, s=s,
                        remaining=max(self.length - s, 0.0))


def pd_curvature(tr: Tracking, gains: PDGains) -> float:
    return gains.ff * tr.kappa_path - gains.kp * tr.e - gains.kd * math.sin(tr.psi_e)


def _bisect(a: Sequence[float], x: float) -> int:
    lo, hi = 0, len(a)
    while lo < hi:
        mid = (lo + hi) // 2
        if a[mid] <= x:
            lo = mid + 1
        else:
            hi = mid
    return lo
