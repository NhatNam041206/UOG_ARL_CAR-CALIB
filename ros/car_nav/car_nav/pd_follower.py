"""PD path follower for the skid-steer car (pure math, no ROS imports).

Classical PD on the path, the same idea as the VISION lane PID:

    kappa_cmd = ff * kappa_path  -  kp * e  -  kd * sin(psi_e)

  e       sideways offset of base_link from the path (m, + = car LEFT of path)
  psi_e   heading error, car heading minus path heading (rad, + = car points
          LEFT of the path). For the car, d(e)/ds = sin(psi_e): the heading
          error IS the derivative of the offset, so the kd term is the D term.
  kappa_path  path curvature just ahead (feed-forward, so a planned curve is
          followed without first building an error).
  kappa_cmd   commanded path curvature (1/m, + = turn left); cmd_vel carries it
          as omega = v * kappa_cmd and actuation.py turns it into wheel speeds.

Units: kp in 1/m^2, kd in 1/m. Measured along the path (not time), the error
obeys e'' + kd e' + kp e = 0, so the gains do not depend on speed: kp sets how
quickly the car returns to the path (it settles in roughly 2*pi/sqrt(kp) m),
kd = 2*sqrt(kp) is "critically damped" (no overshoot without delay). The
command delay (0.25 s assumed) is a dead distance of v*0.25 m (0.1 m at 0.4
m/s); more kd is what keeps the car from rocking because of it.

base_link is the centre of the skid-steer car (midway between the left and
right wheels), so these are its exact kinematics (x' = v cos th,
y' = v sin th, th' = omega = v kappa). Unlike the old rear-steer car it can
also slow down and turn on the spot; diff_drive_command() turns the PD
curvature into (v, omega) with those extras:

* the speed drops in tight curves so omega stays within max_yaw_rate,
* far off heading (|psi_e| > rotate_enter_rad, e.g. a path that starts
  behind the car) it stops and turns on the spot until |psi_e| <
  rotate_exit_rad, then follows the path,
* it slows down over the last goal_slow_m of the path.
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

    def max_curvature_ahead(self, s: float, dist: float, window: float = 0.3, step: float = 0.1) -> float:
        """Largest |curvature| on the next `dist` metres of path after s."""
        best, d = 0.0, 0.0
        while d <= dist and s + d <= self.length:
            best = max(best, abs(self.curvature_at(s + d, window)))
            d += step
        return best

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


def extrapolate_pose(x: float, y: float, th: float, v: float, omega: float, age: float,
                     max_dist: float = 0.3) -> tuple[float, float, float]:
    """Pose `age` seconds later, driving at speed v (m/s) and yaw rate omega (rad/s).

    The pose the follower reads (map -> base_link) is as old as the last
    odometry message; on a busy computer that is 0.2-0.4 s (sim, 2 CPUs),
    i.e. 0.1-0.2 m at 0.5 m/s. v and omega are the odometry's own latest
    velocities (adaptive_odom: learned speed x delayed drive state, so 0
    while it stands). Only forward motion, capped at max_dist metres; a
    negative age or speed changes nothing. base_link (car centre) moves on
    the arc x' = v cos th, y' = v sin th, th' = omega.
    """
    if age <= 0.0 or v <= 0.0 or max_dist <= 0.0:
        return x, y, th
    d = min(v * age, max_dist)
    t = d / v
    dth = omega * t
    if abs(dth) < 1e-6:
        return x + d * math.cos(th), y + d * math.sin(th), th
    k = dth / d                      # curvature of the arc
    return (x + (math.sin(th + dth) - math.sin(th)) / k,
            y - (math.cos(th + dth) - math.cos(th)) / k,
            wrap(th + dth))


@dataclass(frozen=True)
class DiffDriveParams:
    speed_mps: float = 0.4          # cruise speed
    min_speed_mps: float = 0.08     # slowest forward speed while following
    max_yaw_rate_rps: float = 1.5
    rotate_enter_rad: float = 0.9   # turn on the spot above this heading error (~52 deg)
    rotate_exit_rad: float = 0.25   # ... until below this (~14 deg)
    rotate_gain: float = 2.0        # omega = gain * heading error while turning on the spot
    min_rotate_rps: float = 0.5     # slowest turn on the spot (skid-steer wheels stick below this)
    goal_slow_m: float = 0.6        # slow down linearly over this last stretch
    lat_accel_mps2: float = 0.25    # slow down BEFORE a curve: v <= sqrt(lat_accel / curvature ahead)


def rotate_command(err: float, p: DiffDriveParams) -> float:
    """Turn rate on the spot that reduces heading error `err` (rad, + = car points left of target)."""
    rate = min(max(p.rotate_gain * abs(err), p.min_rotate_rps), p.max_yaw_rate_rps)
    return -math.copysign(rate, err)


def diff_drive_command(tr: Tracking, kappa: float, p: DiffDriveParams, rotating: bool,
                       dist_goal: float, kappa_ahead: float = 0.0) -> tuple[float, float, bool]:
    """(v, omega, rotating) for PD curvature `kappa`; `rotating` is the turn-on-the-spot state."""
    err = tr.psi_e
    if rotating and abs(err) < p.rotate_exit_rad:
        rotating = False
    elif not rotating and abs(err) > p.rotate_enter_rad:
        rotating = True
    if rotating:
        return 0.0, rotate_command(err, p), True
    v = p.speed_mps
    if p.goal_slow_m > 0.0:
        v = min(v, p.speed_mps * max(dist_goal, 0.0) / p.goal_slow_m)
    if kappa_ahead > 1e-6 and p.lat_accel_mps2 > 0.0:
        v = min(v, math.sqrt(p.lat_accel_mps2 / kappa_ahead))
    if abs(kappa) * v > p.max_yaw_rate_rps:
        v = p.max_yaw_rate_rps / abs(kappa)
    v = max(v, p.min_speed_mps)
    omega = min(max(v * kappa, -p.max_yaw_rate_rps), p.max_yaw_rate_rps)
    return v, omega, False


def spin_action(err: float, clear: bool, creep_max_rad: float = math.pi / 2) -> str:
    """What to do when the follower wants to turn on the spot (heading error `err`).

    "spin"  the swept circle is clear: turn.
    "creep" not clear, but the path lies roughly ahead (|err| <= creep_max_rad):
            keep following at the minimum speed (wall avoidance and the collision
            monitor still act) instead of turning.
    "wait"  not clear and the path is behind / beside the car: stand still."""
    if clear:
        return "spin"
    return "creep" if abs(err) <= creep_max_rad else "wait"
