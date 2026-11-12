"""Adaptive forward-speed estimate for the LiDAR-only car (no ROS imports).

Problem: between two long parallel walls the LiDAR cannot see the car move
forward, so rf2o reports ~0 forward motion and AMCL falls metres behind.

Idea (a 1-state Kalman filter on the real forward speed v):

* scan_observability(): how well this scan shows forward motion, 0..1,
  from how many wall points face the driving direction (an end wall, a
  corner, a pillar) instead of running parallel to it (corridor sides).
* MotionModel: the commanded drive state (FORWARD/STOP) delayed by the
  command latency and ramped with the accel/decel limits: 0 = standing,
  1 = cruising. Standing means forward speed is exactly 0.
* SpeedKF: v is a slow random walk. While cruising AND the scan is
  observable, rf2o's forward speed is a measurement (noise grows as the
  observability drops; outliers beyond 3 sigma are ignored). An AMCL correction along the heading after a
  stretch driven on the estimate is a second, weaker measurement.
* fuse_forward(): forward distance per step = w * rf2o + (1 - w) * model,
  w = observability, model = motion fraction * v. Sideways motion and
  rotation always come from rf2o (those it measures well in corridors).

Start value: car.yaml speed.baseline_speed_mps. It then follows the real
speed (battery, floor, slope). A sudden change in the middle of a blind
corridor is only corrected at the next observable place.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


def scan_observability(ranges: Sequence[float], angle_min: float, angle_inc: float,
                       max_range: float = 8.0, facing_cos: float = 0.5,
                       n_low: int = 10, n_high: int = 50, step: int = 2) -> tuple[float, int]:
    """(w in 0..1, n facing points). Scan in the base frame (x = forward).

    A point "faces" the driving direction when the local wall normal (from
    its neighbours `step` beams away) has |cos(angle to x)| >= facing_cos.
    w = 0 below n_low facing points, 1 above n_high, linear between.
    """
    pts: list[tuple[float, float] | None] = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.05 or r > max_range:
            pts.append(None)
        else:
            a = angle_min + i * angle_inc
            pts.append((r * math.cos(a), r * math.sin(a)))
    n = 0
    for i in range(step, len(pts) - step):
        p, a, b = pts[i], pts[i - step], pts[i + step]
        if p is None or a is None or b is None:
            continue
        tx, ty = b[0] - a[0], b[1] - a[1]
        tl = math.hypot(tx, ty)
        if tl < 1e-6 or tl > 0.5:      # gap between two objects, not a surface
            continue
        # normal = tangent rotated 90 deg; |n_x| = |t_y| / |t|
        if abs(ty) / tl >= facing_cos:
            n += 1
    w = min(max((n - n_low) / float(n_high - n_low), 0.0), 1.0)
    return w, n


@dataclass
class MotionModel:
    """Fraction of cruising speed the car has now, from the commands only."""
    latency_s: float = 0.65
    accel_mps2: float = 1.0
    decel_mps2: float = 2.0
    nominal_speed: float = 0.5

    def __post_init__(self) -> None:
        self._queue: list[tuple[float, bool]] = []
        self._drive = False
        self.fraction = 0.0

    def command(self, t: float, forward: bool) -> None:
        if not self._queue or self._queue[-1][1] != forward:
            self._queue.append((t + self.latency_s, forward))

    def step(self, t: float, dt: float) -> float:
        while self._queue and self._queue[0][0] <= t:
            self._drive = self._queue.pop(0)[1]
        v = max(self.nominal_speed, 0.05)
        if self._drive:
            self.fraction = min(1.0, self.fraction + self.accel_mps2 / v * dt)
        else:
            self.fraction = max(0.0, self.fraction - self.decel_mps2 / v * dt)
        return self.fraction

    @property
    def cruising(self) -> bool:
        return self._drive and self.fraction >= 1.0

    @property
    def standing(self) -> bool:
        return not self._drive and self.fraction <= 0.0


@dataclass
class SpeedKF:
    v: float = 0.5               # estimate (m/s)
    var: float = 0.1 ** 2        # its variance
    q: float = 0.01 ** 2         # random walk per second ((m/s)^2/s)
    r_rf2o: float = 0.08 ** 2    # rf2o forward-speed noise at full observability
    v_min: float = 0.05
    v_max: float = 2.0

    def predict(self, dt: float) -> None:
        self.var += self.q * max(dt, 0.0)

    def update(self, z: float, r: float) -> float:
        k = self.var / (self.var + r)
        self.v = min(max(self.v + k * (z - self.v), self.v_min), self.v_max)
        self.var *= (1.0 - k)
        return k

    def update_rf2o(self, v_meas: float, w: float, gate_sigma: float = 3.0) -> bool:
        """rf2o forward speed while cruising; w = scan observability (0..1).

        Returns False (and ignores it) when it is an outlier: further than
        gate_sigma standard deviations from the estimate. That keeps a
        failing rf2o (e.g. reporting 0 while the car drives) from teaching
        the filter that the car stands still."""
        if w <= 0.0:
            return False
        r = self.r_rf2o / w
        if (v_meas - self.v) ** 2 > gate_sigma ** 2 * (self.var + r):
            return False
        self.update(v_meas, r)
        return True

    def update_correction(self, along_m: float, blind_m: float, r_scale: float = 4.0) -> float:
        """AMCL moved the car `along_m` forward (+) along its heading after
        `blind_m` metres driven mostly on the estimate: the true speed was
        about v * (1 + along_m / blind_m)."""
        if blind_m < 0.5:
            return 0.0
        z = self.v * (1.0 + max(-0.3, min(0.3, along_m / blind_m)))
        return self.update(z, self.r_rf2o * r_scale)


def fuse_forward(rf2o_dx: float, model_dx: float, w: float) -> float:
    """Forward distance for one step: rf2o where it can see, the model where it cannot."""
    w = min(max(w, 0.0), 1.0)
    return w * rf2o_dx + (1.0 - w) * model_dx
