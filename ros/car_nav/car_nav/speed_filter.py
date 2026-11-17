"""Adaptive forward-speed estimate for the LiDAR-only car (no ROS imports).

Problem: between two long parallel walls the LiDAR cannot see the car move
forward, so rf2o reports ~0 forward motion and AMCL falls metres behind.

Idea (a 1-state Kalman filter on the real forward speed v):

* scan_observability(): how well this scan shows forward motion, 0..1,
  from how many wall points face the driving direction (an end wall, a
  corner, a pillar) instead of running parallel to it (corridor sides).
* MotionModel: the commanded forward speed (from the wheel command)
  delayed by the command latency and ramped with the accel/decel limits.
  Standing (no speed and no turn commanded) means forward speed is exactly 0.
  On the skid-steer car SpeedKF estimates the RATIO real / commanded speed
  (start 1.0), see adaptive_odom.py.
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

import numpy as np


def scan_observability(ranges: Sequence[float], angle_min: float, angle_inc: float,
                       max_range: float = 8.0, facing_cos: float = 0.5,
                       n_low: int = 10, n_high: int = 50, min_span: float = 0.25,
                       max_span: float = 0.8, max_resid: float = 0.03, stride: int = 2) -> tuple[float, int]:
    """(w in 0..1, n facing points). Scan in the base frame (x = forward).

    For every `stride`-th point, a straight line is fitted (PCA) to its
    neighbours, growing the window until it spans >= min_span metres (with
    1 cm range noise a 5 cm window beside the car gives a random normal).
    The point "faces" the driving direction when that wall's normal has
    |cos(angle to x)| >= facing_cos and the wall is straight (residual <=
    max_resid). w = 0 below n_low facing points, 1 above n_high (counted
    on every point, i.e. stride x the sampled ones).

    numpy, all sampled points at once (2026-11-16: the per-point Python loop
    cost ~12 ms per scan on the laptop, several times that on the Pi 5). Same
    rules as before (tests/_car_nav_reference.py, equivalence test): the window
    grows one point per side per step; it fails on a missing neighbour, a jump
    > 0.2 m between neighbours, a span > max_span or more than 80 points.
    """
    r = np.asarray(ranges, dtype=float)
    n_pts = r.shape[0]
    if n_pts < 3:
        return 0.0, 0
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(r) & (r > 0.05) & (r <= max_range)
    a = angle_min + np.arange(n_pts) * angle_inc
    rr = np.where(valid, r, 0.0)
    x, y = rr * np.cos(a), rr * np.sin(a)
    # jump[j] = distance between points j and j+1
    jump = np.hypot(np.diff(x), np.diff(y)) if n_pts > 1 else np.zeros(0)
    idx = np.arange(0, n_pts, stride)
    idx = idx[valid[idx]]
    win = np.zeros(idx.shape, dtype=int)            # half width k of the accepted window, 0 = none
    active = np.ones(idx.shape, dtype=bool)
    for k in range(1, 41 + 1):
        if not active.any():
            break
        lo, hi = idx - k, idx + k
        inb = (lo >= 0) & (hi < n_pts)
        lo_c, hi_c = np.clip(lo, 0, n_pts - 1), np.clip(hi, 0, n_pts - 1)
        ok = active & inb & valid[lo_c] & valid[hi_c]
        jl = jump[np.clip(lo, 0, max(n_pts - 2, 0))]   # between lo and lo + 1
        jh = jump[np.clip(hi - 1, 0, max(n_pts - 2, 0))]  # between hi - 1 and hi
        ok &= (jl <= 0.2) & (jh <= 0.2)
        span = np.hypot(x[hi_c] - x[lo_c], y[hi_c] - y[lo_c])
        ok &= (span <= max_span) & (2 * k <= 80)
        done = ok & (span >= min_span)
        win[done] = k
        active = ok & ~done
    sel = win > 0
    if not sel.any():
        n = 0
    else:
        i, k = idx[sel], win[sel]
        lo, hi = i - k, i + k
        m = (2 * k + 1).astype(float)
        cx, cy = np.concatenate(([0.0], np.cumsum(x))), np.concatenate(([0.0], np.cumsum(y)))
        mx = (cx[hi + 1] - cx[lo]) / m
        my = (cy[hi + 1] - cy[lo]) / m
        # second moments about the window mean, summed exactly per window
        off = np.arange(-40, 41)
        jj = i[:, None] + off[None, :]
        inwin = np.abs(off)[None, :] <= k[:, None]
        jj = np.clip(jj, 0, n_pts - 1)
        dx = np.where(inwin, x[jj] - mx[:, None], 0.0)
        dy = np.where(inwin, y[jj] - my[:, None], 0.0)
        sxx = (dx * dx).sum(axis=1) / m
        syy = (dy * dy).sum(axis=1) / m
        sxy = (dx * dy).sum(axis=1) / m
        tr, det = sxx + syy, sxx * syy - sxy * sxy
        lam_min = tr / 2 - np.sqrt(np.maximum(tr * tr / 4 - det, 0.0))
        theta = 0.5 * np.arctan2(2 * sxy, sxx - syy)
        facing = (lam_min <= max_resid ** 2) & (np.abs(np.sin(theta)) >= facing_cos)
        n = int(facing.sum()) * stride
    w = min(max((n - n_low) / float(n_high - n_low), 0.0), 1.0)
    return w, n


@dataclass
class MotionModel:
    """Forward speed the car should have now, from the wheel commands only.

    command(t, v, omega): the commanded body speed (m/s) and turn rate
    (rad/s), i.e. (left + right) / 2 and (right - left) / track of the wheel
    command. They act latency_s later and the speed follows with the
    accel/decel limits. `speed` is that model speed (m/s, >= 0 when the car
    does not reverse); adaptive_odom multiplies it by the learned ratio
    real/commanded speed.
    """
    latency_s: float = 0.25
    accel_mps2: float = 1.0
    decel_mps2: float = 2.0
    nominal_speed: float = 0.4
    steady_tol_mps: float = 0.02

    def __post_init__(self) -> None:
        self._queue: list[tuple[float, float, float]] = []
        self._last: tuple[float, float] | None = None
        self.target = 0.0          # delayed commanded speed (m/s)
        self.target_omega = 0.0    # delayed commanded turn rate (rad/s)
        self.speed = 0.0           # model speed after the ramp (m/s)

    def command(self, t: float, v: float, omega: float = 0.0) -> None:
        cmd = (float(v), float(omega))
        if self._last is None or abs(cmd[0] - self._last[0]) > 1e-4 or abs(cmd[1] - self._last[1]) > 1e-4:
            self._queue.append((t + self.latency_s, cmd[0], cmd[1]))
            self._last = cmd

    def step(self, t: float, dt: float) -> float:
        while self._queue and self._queue[0][0] <= t:
            _, self.target, self.target_omega = self._queue.pop(0)
        if self.target > self.speed:
            self.speed = min(self.target, self.speed + self.accel_mps2 * dt)
        else:
            self.speed = max(self.target, self.speed - self.decel_mps2 * dt)
        return self.speed

    @property
    def fraction(self) -> float:
        """Model speed as a share of the nominal cruise speed."""
        return self.speed / max(self.nominal_speed, 0.05)

    @property
    def cruising(self) -> bool:
        """Driving forward at a steady commanded speed (good time to learn the ratio)."""
        return self.target >= 0.1 and abs(self.speed - self.target) <= self.steady_tol_mps

    @property
    def standing(self) -> bool:
        """Commanded to stand still (no speed, no turn) and the speed has ramped down."""
        return abs(self.target) < 1e-3 and abs(self.target_omega) < 1e-3 and abs(self.speed) < 1e-3


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

    def update_rf2o(self, v_meas: float, w: float, gate_sigma: float = 3.0, r_scale: float = 1.0) -> bool:
        """rf2o forward speed while cruising; w = scan observability (0..1).

        Returns False (and ignores it) when it is an outlier: further than
        gate_sigma standard deviations from the estimate. That keeps a
        failing rf2o (e.g. reporting 0 while the car drives) from teaching
        the filter that the car stands still."""
        if w <= 0.0:
            return False
        r = self.r_rf2o * r_scale / w
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
