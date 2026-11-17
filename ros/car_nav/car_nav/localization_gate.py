"""Gate for AMCL corrections in blind corridors (pure math, no ROS imports).

Between two long parallel walls the scan says nothing about how far along
the corridor the car is. AMCL's particles then wander along the corridor
(seen in the sim 2026-11-12: AMCL 4.2 m off while the odometry alone was
only 1.35 m off), and the car turns at the wrong place.

The gate sits between AMCL and the map -> odom transform:

  * sideways and heading part of an AMCL correction: always taken
    (the corridor walls measure those well);
  * along-the-heading part: taken in proportion to the scan observability
    w (speed_filter.scan_observability, 0 = plain corridor, 1 = end wall /
    corner / pillar in view): g = min(w / obs_full, 1).

So in a blind corridor the pose moves forward with the odometry only, and
at the next junction (w high) AMCL may pull it to the right place again.

Poses are (x, y, theta) tuples; a transform A -> B is stored as the pose of
B in A.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

Pose = tuple[float, float, float]


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a: Pose, b: Pose) -> Pose:
    """a * b (b expressed in a's frame)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a: Pose) -> Pose:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def along_lateral(dx: float, dy: float, heading: float) -> tuple[float, float]:
    c, s = math.cos(heading), math.sin(heading)
    return c * dx + s * dy, -s * dx + c * dy


@dataclass
class GateStep:
    map_odom: Pose      # new map -> odom
    pose: Pose          # car pose in the map it gives (at the AMCL stamp)
    along: float        # AMCL's along-heading correction (m)
    taken: float        # share of it that was used (0..1)


def gate_correction(map_odom: Pose | None, odom_base: Pose, amcl: Pose, w: float,
                    obs_full: float = 0.5) -> GateStep:
    """One AMCL pose -> the new map -> odom.

    map_odom   current correction (None = none yet: take AMCL fully)
    odom_base  odometry pose of the car at the AMCL stamp
    amcl       AMCL's pose of the car (map frame) at that stamp
    w          scan observability around that stamp (0..1)
    """
    if map_odom is None:
        return GateStep(compose(amcl, inverse(odom_base)), amcl, 0.0, 1.0)
    cur = compose(map_odom, odom_base)
    along, lat = along_lateral(amcl[0] - cur[0], amcl[1] - cur[1], amcl[2])
    g = 1.0 if obs_full <= 0.0 else min(max(w / obs_full, 0.0), 1.0)
    c, s = math.cos(amcl[2]), math.sin(amcl[2])
    a = g * along
    pose = (cur[0] + c * a - s * lat, cur[1] + s * a + c * lat, amcl[2])
    return GateStep(compose(pose, inverse(odom_base)), pose, along, g)


def reseed_covariance(heading: float, sigma_along: float, sigma_lateral: float,
                      sigma_yaw: float) -> list[float]:
    """6x6 row-major covariance (PoseWithCovariance) for a reseed of AMCL:
    long along the heading, short across it."""
    c, s = math.cos(heading), math.sin(heading)
    va, vl = sigma_along ** 2, sigma_lateral ** 2
    cov = [0.0] * 36
    cov[0] = c * c * va + s * s * vl
    cov[7] = s * s * va + c * c * vl
    cov[1] = cov[6] = c * s * (va - vl)
    cov[35] = sigma_yaw ** 2
    return cov


@dataclass
class InitialPoseRetry:
    """Resend the initial pose until AMCL answers (C3, nam 2026-11-17).

    Seen in the sim under CPU load (8 of 32 runs): AMCL takes /initialpose
    ("Setting pose") but never publishes /amcl_pose, so map -> odom never
    appears and the car never starts. pose_gate keeps the last initial pose
    and, while no /amcl_pose has arrived since it, sends it again every
    interval_s, at most max_tries times. Times are seconds (any monotonic clock).
    """
    interval_s: float = 2.0
    max_tries: int = 10
    waiting_since: float | None = None
    last_sent: float = 0.0
    tries: int = 0

    def on_initialpose(self, t: float) -> None:
        """An initial pose from someone else (RViz, the mission, the operator)."""
        self.waiting_since, self.last_sent, self.tries = t, t, 0

    def on_amcl(self, t: float) -> None:
        if self.waiting_since is not None and t >= self.waiting_since:
            self.waiting_since = None

    def due(self, t: float) -> bool:
        """True when the initial pose should be sent again now (and counts the try)."""
        if self.waiting_since is None or self.tries >= self.max_tries or t - self.last_sent < self.interval_s:
            return False
        self.tries += 1
        self.last_sent = t
        return True

    @property
    def gave_up(self) -> bool:
        return self.waiting_since is not None and self.tries >= self.max_tries
