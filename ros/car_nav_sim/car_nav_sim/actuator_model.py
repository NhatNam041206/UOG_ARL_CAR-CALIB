"""What the real skid-steer car does with a "<left m/s>,<right m/s>" command (no ROS imports).

This is the simulator's stand-in for the ugv-controller + motor driver +
motors + tracks/wheels. Everything that is unknown about the real car is a
parameter here (config/sim.yaml), so a run can be repeated with a worse
car than Nav2 believes it has:

* command latency (queue: a command takes effect latency_s after it is sent)
* wheel speed limit, per-wheel accel / braking limits
* speed_scale: real forward speed = speed_scale x commanded (battery, floor)
* left_gain / right_gain: one side slower than the other (drifts to one side)
* yaw_efficiency: a skid-steer car turns slower than (right - left) / track
  because the wheels slide sideways; real omega = yaw_efficiency x ideal.
  On the spot it may not turn at all below min_spin_wheel_mps (stiction).
* what happens when commands stop arriving (hold the last one, or stop)

Pose is integrated at base_link = the centre of the car.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from car_nav.actuation import DriveConfig, parse_payload


@dataclass
class TruthParams:
    latency_s: float = 0.25
    max_wheel_speed_mps: float = 0.7
    accel_mps2: float = 1.5
    decel_mps2: float = 2.5
    speed_scale: float = 1.0
    left_gain: float = 1.0
    right_gain: float = 1.0
    yaw_efficiency: float = 0.8
    # Turning on the spot: wheels slower than this (m/s) do not get the car to turn.
    min_spin_wheel_mps: float = 0.0
    # Real speed drifts: speed_scale * (1 + speed_variation * sin(2 pi t / speed_period_s)).
    speed_variation: float = 0.0
    speed_period_s: float = 40.0
    # Seconds without any command before the car stops by itself.
    # None = the car holds the last command forever (worst case).
    failsafe_timeout_s: float | None = None
    # Wheel encoders (lite_sim /odom_wheel): relative speed noise per message.
    encoder_noise: float = 0.02

    @classmethod
    def from_dict(cls, d: dict) -> "TruthParams":
        known = {k: d[k] for k in cls.__dataclass_fields__ if k in d}
        return cls(**known)


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class ActuatorModel:
    drive: DriveConfig
    params: TruthParams
    left: float = field(init=False, default=0.0)        # wheel speeds now (m/s, at the tread)
    right: float = field(init=False, default=0.0)
    target: tuple[float, float] = field(init=False, default=(0.0, 0.0))
    v: float = field(init=False, default=0.0)
    omega: float = field(init=False, default=0.0)
    last_cmd_t: float | None = field(init=False, default=None)
    rejected: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        self._queue: deque[tuple[float, float, float]] = deque()

    def command(self, t: float, payload: str) -> bool:
        parsed = parse_payload(payload)
        if parsed is None:
            self.rejected += 1
            return False
        lim = self.params.max_wheel_speed_mps
        left, right = (min(max(w, -lim), lim) for w in parsed)
        self._queue.append((t + self.params.latency_s, left, right))
        self.last_cmd_t = t
        return True

    @staticmethod
    def _ramp(now: float, target: float, accel: float, decel: float, dt: float) -> float:
        # speeding up (away from 0) uses accel, slowing down / reversing uses decel
        if abs(target) > abs(now) and target * now >= 0.0:
            step = accel * dt
        else:
            step = decel * dt
        return now + max(-step, min(step, target - now))

    def step(self, t: float, dt: float) -> tuple[float, float]:
        """Advance dt seconds at time t; returns (v, omega) of base_link."""
        while self._queue and self._queue[0][0] <= t:
            _, l, r = self._queue.popleft()
            self.target = (l, r)
        p = self.params
        if (
            p.failsafe_timeout_s is not None
            and self.last_cmd_t is not None
            and t - self.last_cmd_t > p.failsafe_timeout_s + p.latency_s
        ):
            self.target = (0.0, 0.0)
        self.left = self._ramp(self.left, self.target[0], p.accel_mps2, p.decel_mps2, dt)
        self.right = self._ramp(self.right, self.target[1], p.accel_mps2, p.decel_mps2, dt)
        scale = p.speed_scale * (1.0 + p.speed_variation * math.sin(
            2.0 * math.pi * t / max(p.speed_period_s, 1e-3)))
        l, r = self.left * p.left_gain * scale, self.right * p.right_gain * scale
        v = 0.5 * (l + r)
        omega = p.yaw_efficiency * (r - l) / self.drive.track_width_m
        if abs(v) < 1e-3 and max(abs(l), abs(r)) < p.min_spin_wheel_mps:
            omega = 0.0          # stiction: too slow to turn on the spot
        self.v, self.omega = v, omega
        return v, omega

    def encoder_twist(self) -> tuple[float, float]:
        """What wheel encoders report: the wheels' own speed (left/right gain
        included), but not the slip (speed_scale, yaw_efficiency): (v, omega)."""
        p = self.params
        l, r = self.left * p.left_gain, self.right * p.right_gain
        return 0.5 * (l + r), (r - l) / self.drive.track_width_m


def integrate(pose: Pose2D, v: float, omega: float, dt: float) -> Pose2D:
    """Exact arc integration of base_link (also turning on the spot, v = 0)."""
    if abs(omega) < 1e-9:
        return Pose2D(pose.x + v * dt * math.cos(pose.theta), pose.y + v * dt * math.sin(pose.theta), pose.theta)
    th2 = pose.theta + omega * dt
    r = v / omega
    return Pose2D(
        pose.x + r * (math.sin(th2) - math.sin(pose.theta)),
        pose.y - r * (math.cos(th2) - math.cos(pose.theta)),
        math.atan2(math.sin(th2), math.cos(th2)),
    )
