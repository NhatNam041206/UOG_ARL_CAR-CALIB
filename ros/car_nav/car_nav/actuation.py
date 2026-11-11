"""cmd_vel -> "<angle>,<drive_code>" mapping for the rear-steer car (no ROS imports).

Shared by the real car (WO-02 MQTT forwarder) and the simulator, so both
turn a Nav2 velocity command into exactly the same wire payload.

Kinematics. base_link is the floor point under the FRONT (fixed) axle and
the car steers with its REAR wheels. The front axle cannot slip sideways,
so base_link moves straight along the heading and the car yaws about it:

    x' = v cos(theta),  y' = v sin(theta),  theta' = v * kappa

kappa is the path curvature of base_link (1/m, + = turning left). For a
rear wheel angle delta and wheelbase L, |kappa| = tan(|delta|) / L. The
rear wheels point RIGHT to turn the car LEFT (opposite of a front-steer
car), which is why the table below is indexed by what the CAR does, not by
which way the wheels point.

Wire format (Pi4 DataProcessingCenter): "<absolute servo angle>,<drive_code>"
where the angle is what map_calibrated_servo() would output (35..140 on
this car, 62 = straight) and drive_code is 1 = FORWARD, 0 = STOP. Code 2
(BACKWARD) is unconfirmed and never sent: NAV2 is forward-only.
"""
from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass
from typing import Any, Sequence

DRIVE_STOP = 0
DRIVE_FORWARD = 1


@dataclass(frozen=True)
class SteeringConfig:
    wheelbase_m: float = 0.28
    servo_center: float = 62.0
    servo_min: float = 35.0
    servo_max: float = 140.0
    # Which way the CAR turns when the servo angle goes above centre.
    servo_up_turns: str = "left"
    assumed_linkage_ratio: float = 1.0
    assumed_max_wheel_deg: float = 30.0
    curvature_table: tuple[tuple[float, float], ...] | None = None

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any]) -> "SteeringConfig":
        s = cfg.get("steering") or {}
        table = s.get("curvature_table")
        return cls(
            wheelbase_m=float(s.get("wheelbase_m", cls.wheelbase_m)),
            servo_center=float(s.get("servo_center", cls.servo_center)),
            servo_min=float(s.get("servo_min", cls.servo_min)),
            servo_max=float(s.get("servo_max", cls.servo_max)),
            servo_up_turns=str(s.get("servo_up_turns", cls.servo_up_turns)).lower(),
            assumed_linkage_ratio=float(s.get("assumed_linkage_ratio", cls.assumed_linkage_ratio)),
            assumed_max_wheel_deg=float(s.get("assumed_max_wheel_deg", cls.assumed_max_wheel_deg)),
            curvature_table=None if not table else tuple((float(a), float(k)) for a, k in table),
        )


class SteeringTable:
    """Absolute servo angle <-> base_link path curvature, per side.

    Built from a measured table when steering.curvature_table is set,
    otherwise from a placeholder: rear wheel angle = servo offset x
    assumed_linkage_ratio, capped at assumed_max_wheel_deg.
    """

    def __init__(self, cfg: SteeringConfig) -> None:
        if cfg.servo_up_turns not in ("left", "right"):
            raise ValueError("steering.servo_up_turns must be 'left' or 'right'")
        self.cfg = cfg
        self.center = cfg.servo_center
        self.measured = cfg.curvature_table is not None
        if self.measured:
            points = sorted(cfg.curvature_table or ())
            if not any(abs(a - cfg.servo_center) < 1e-9 for a, _ in points):
                points.append((cfg.servo_center, 0.0))
                points.sort()
        else:
            points = self._placeholder(cfg)
        self._servo = [a for a, _ in points]
        self._kappa = [k for _, k in points]
        if self._servo[0] < cfg.servo_min - 1e-9 or self._servo[-1] > cfg.servo_max + 1e-9:
            raise ValueError("curvature_table has servo angles outside servo_min..servo_max")
        # The curvature must not change sign the wrong way across centre.
        sign = 1.0 if cfg.servo_up_turns == "left" else -1.0
        for a, k in points:
            if (a - self.center) * k * sign < -1e-9:
                raise ValueError(f"curvature_table point ({a}, {k}) has the wrong sign for servo_up_turns")
        for i in range(1, len(self._kappa)):
            if (self._kappa[i] - self._kappa[i - 1]) * sign < -1e-9:
                raise ValueError("curvature_table must be monotonic in servo angle")

    @staticmethod
    def _placeholder(cfg: SteeringConfig) -> list[tuple[float, float]]:
        sign = 1.0 if cfg.servo_up_turns == "left" else -1.0
        points = []
        lo, hi = int(math.ceil(cfg.servo_min)), int(math.floor(cfg.servo_max))
        for a in range(lo, hi + 1):
            off = a - cfg.servo_center
            wheel = min(abs(off) * cfg.assumed_linkage_ratio, cfg.assumed_max_wheel_deg)
            k = math.copysign(math.tan(math.radians(wheel)) / cfg.wheelbase_m, off) if off else 0.0
            points.append((float(a), sign * k))
        return points

    # ------------------------------------------------------------------ #
    @property
    def max_left_curvature(self) -> float:
        return max(self._kappa)

    @property
    def max_right_curvature(self) -> float:
        """Most negative curvature, returned as a positive number."""
        return -min(self._kappa)

    @property
    def min_turning_radius(self) -> float:
        """Radius on the weaker side: the tightest turn possible both ways."""
        return 1.0 / min(self.max_left_curvature, self.max_right_curvature)

    def curvature(self, servo: float) -> float:
        """Curvature the car drives at a given absolute servo angle."""
        s = min(max(servo, self._servo[0]), self._servo[-1])
        i = bisect_left(self._servo, s)
        if i < len(self._servo) and abs(self._servo[i] - s) < 1e-12:
            return self._kappa[i]
        a0, a1 = self._servo[i - 1], self._servo[i]
        k0, k1 = self._kappa[i - 1], self._kappa[i]
        return k0 + (k1 - k0) * (s - a0) / (a1 - a0)

    def servo_for(self, kappa: float) -> float:
        """Smallest servo deflection that reaches `kappa` (saturates at the side's limit)."""
        if not math.isfinite(kappa) or abs(kappa) < 1e-12:
            return self.center
        # Points on kappa's side, ordered from centre outward.
        side = [(a, k) for a, k in zip(self._servo, self._kappa)
                if k * kappa > 0 or abs(a - self.center) < 1e-9]
        side.sort(key=lambda p: abs(p[0] - self.center))
        prev_a, prev_k = self.center, 0.0
        for a, k in side:
            if abs(k) >= abs(kappa):
                if abs(k - prev_k) < 1e-12:
                    return prev_a
                return prev_a + (a - prev_a) * (kappa - prev_k) / (k - prev_k)
            if abs(k) > abs(prev_k):
                prev_a, prev_k = a, k
        return prev_a


@dataclass(frozen=True)
class MapperConfig:
    # Below this forward speed command the car is told to STOP.
    forward_threshold_mps: float = 0.05
    # Servo angle sent with STOP (straight).
    stop_servo: float | None = None


class CmdVelMapper:
    """Nav2 (v, omega) -> (absolute servo angle, drive_code)."""

    def __init__(self, table: SteeringTable, cfg: MapperConfig = MapperConfig()) -> None:
        self.table = table
        self.cfg = cfg

    @property
    def stop_payload(self) -> tuple[int, int]:
        servo = self.table.center if self.cfg.stop_servo is None else self.cfg.stop_servo
        return int(round(servo)), DRIVE_STOP

    def map(self, v: float, omega: float) -> tuple[int, int]:
        if not (math.isfinite(v) and math.isfinite(omega)) or v < self.cfg.forward_threshold_mps:
            # Forward-only: a reverse or tiny command means STOP.
            return self.stop_payload
        kappa = omega / v
        servo = self.table.servo_for(kappa)
        servo = min(max(servo, self.table.cfg.servo_min), self.table.cfg.servo_max)
        return int(round(servo)), DRIVE_FORWARD


class SteeringSmoother:
    """Smooths the servo angle sent to the car (no ROS imports).

    tau_s:        first-order low-pass time constant (s); 0 = off. The angle
                  covers ~63 % of a step in tau_s, ~95 % in 3 * tau_s.
    max_rate_dps: the angle moves at most this many degrees per second toward
                  the target; 0 = off.
    Both depend on real elapsed time (dt), not on how often step() is called.
    They only change how fast the servo follows Nav2, never the target, so a
    held command is still reached. reset() after STOP so the next drive
    starts from the commanded angle.
    """

    def __init__(self, tau_s: float = 0.0, max_rate_dps: float = 0.0) -> None:
        self.tau_s = max(float(tau_s), 0.0)
        self.max_rate_dps = max(float(max_rate_dps), 0.0)
        self._angle: float | None = None

    @property
    def enabled(self) -> bool:
        return self.tau_s > 0.0 or self.max_rate_dps > 0.0

    def reset(self, angle: float | None = None) -> None:
        self._angle = angle

    def step(self, target: float, dt: float) -> float:
        if self._angle is None or not self.enabled:
            self._angle = float(target)
            return self._angle
        dt = max(dt, 0.0)
        nxt = float(target)
        if self.tau_s > 0.0:
            nxt = self._angle + (1.0 - math.exp(-dt / self.tau_s)) * (target - self._angle)
        if self.max_rate_dps > 0.0:
            lim = self.max_rate_dps * dt
            nxt = min(max(nxt, self._angle - lim), self._angle + lim)
        self._angle = nxt
        return self._angle


def format_payload(angle: int, drive_code: int) -> str:
    return f"{int(angle)},{int(drive_code)}"


def parse_payload(text: str) -> tuple[int, int] | None:
    try:
        a, d = text.strip().split(",")
        return int(float(a)), int(d)
    except (ValueError, AttributeError):
        return None


def table_from_cfg(cfg: dict[str, Any]) -> SteeringTable:
    return SteeringTable(SteeringConfig.from_cfg(cfg))


def curvature_points(table: SteeringTable) -> Sequence[tuple[float, float]]:
    return list(zip(table._servo, table._kappa))
