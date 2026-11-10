"""What the real car does with a "<angle>,<drive_code>" command (no ROS imports).

This is the simulator's stand-in for Pi4 DataProcessingCenter + ESP32 +
servo + motor. Everything that is unknown about the real car is a
parameter here (config/sim.yaml), so a run can be repeated with a worse
car than Nav2 believes it has:

* command latency (queue: a command takes effect latency_s after it is sent)
* fixed FORWARD speed (no speed command exists), accel / braking limits
* servo slew rate
* understeer: true curvature = understeer_factor x what the table says
* what happens when commands stop arriving (hold the last one, or stop)

Pose is integrated at base_link = front-axle centre (it cannot slip
sideways), so the rear swings out in a turn exactly like the real car.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from car_nav.actuation import DRIVE_FORWARD, SteeringTable, parse_payload


@dataclass
class TruthParams:
    speed_mps: float = 0.5
    latency_s: float = 0.65
    accel_mps2: float = 1.0
    decel_mps2: float = 2.0
    servo_rate_dps: float = 300.0
    understeer_factor: float = 1.0
    # Seconds without any command before the car stops by itself.
    # None = the car holds the last command forever (worst case).
    failsafe_timeout_s: float | None = None

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
    table: SteeringTable
    params: TruthParams
    servo: float = field(init=False)
    target_servo: float = field(init=False)
    drive: int = field(init=False, default=0)
    v: float = field(init=False, default=0.0)
    last_cmd_t: float | None = field(init=False, default=None)
    rejected: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        self.servo = self.table.center
        self.target_servo = self.table.center
        self._queue: deque[tuple[float, float, int]] = deque()

    def command(self, t: float, payload: str) -> bool:
        parsed = parse_payload(payload)
        if parsed is None:
            self.rejected += 1
            return False
        angle, code = parsed
        cfg = self.table.cfg
        angle = min(max(angle, cfg.servo_min), cfg.servo_max)
        if code not in (0, 1):
            code = 0  # 2 (BACKWARD) is unconfirmed on the real car: treat as STOP
        self._queue.append((t + self.params.latency_s, float(angle), code))
        self.last_cmd_t = t
        return True

    def step(self, t: float, dt: float) -> tuple[float, float]:
        """Advance dt seconds at time t; returns (v, omega) of base_link."""
        while self._queue and self._queue[0][0] <= t:
            _, self.target_servo, self.drive = self._queue.popleft()
        p = self.params
        if (
            p.failsafe_timeout_s is not None
            and self.last_cmd_t is not None
            and t - self.last_cmd_t > p.failsafe_timeout_s + p.latency_s
        ):
            self.drive = 0
        dmax = p.servo_rate_dps * dt
        self.servo += max(-dmax, min(dmax, self.target_servo - self.servo))
        v_target = p.speed_mps if self.drive == DRIVE_FORWARD else 0.0
        if v_target > self.v:
            self.v = min(v_target, self.v + p.accel_mps2 * dt)
        else:
            self.v = max(v_target, self.v - p.decel_mps2 * dt)
        kappa = p.understeer_factor * self.table.curvature(self.servo)
        return self.v, self.v * kappa


def integrate(pose: Pose2D, v: float, omega: float, dt: float) -> Pose2D:
    """Exact arc integration of the front-axle point."""
    if abs(omega) < 1e-9:
        return Pose2D(pose.x + v * dt * math.cos(pose.theta), pose.y + v * dt * math.sin(pose.theta), pose.theta)
    th2 = pose.theta + omega * dt
    r = v / omega
    return Pose2D(
        pose.x + r * (math.sin(th2) - math.sin(pose.theta)),
        pose.y - r * (math.cos(th2) - math.cos(pose.theta)),
        math.atan2(math.sin(th2), math.cos(th2)),
    )
