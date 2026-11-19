"""cmd_vel -> wheel speed command for the skid-steer car (no ROS imports).

Shared by the real car (the ugv-controller adapter) and the simulator, so
both turn a Nav2 velocity command into exactly the same wire payload.

Kinematics (differential drive). base_link is the floor point at the
centre of the car, midway between the left and right wheels (the point a
skid-steer car turns about). With track width b:

    v_left  = v - omega * b / 2          v = (v_left + v_right) / 2
    v_right = v + omega * b / 2          omega = (v_right - v_left) / b
    x' = v cos(theta),  y' = v sin(theta),  theta' = omega

so the car can turn on the spot (v = 0). A real skid-steer car turns
slower than this ideal because the wheels slide sideways; that is a
property of the car (sim.yaml truth.yaw_efficiency) that the LiDAR
odometry and the controllers correct for, not something the mapping
guesses.

Wire format: "<left m/s>,<right m/s>", e.g. "0.350,0.420", on the ROS
topic /car/wheels/cmd (std_msgs/String). A wheel speed beyond
max_wheel_speed_mps is not clipped on its own: both wheels are scaled down
together, so the commanded curvature (where the car goes) is kept and only
the speed drops. Reverse (v < 0) is sent only when allow_reverse is true.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

WHEELS_TOPIC = "/car/wheels/cmd"


@dataclass(frozen=True)
class DriveConfig:
    # Centre-to-centre distance between the left and right wheels (m).
    track_width_m: float = 0.52
    # Fastest a wheel is allowed to turn (m/s at the tread), both ways.
    max_wheel_speed_mps: float = 0.6
    # Nav2's cruise speed (pd_path_follower.speed_mps / RPP desired_linear_vel).
    cruise_speed_mps: float = 0.4
    # Fastest turn Nav2 may ask for (rad/s).
    max_yaw_rate_rps: float = 1.5
    # Wheel speeds below this are sent as 0 (motor dead band).
    min_wheel_speed_mps: float = 0.0
    # Turning on the spot: wheels slower than this do not turn a skid-steer
    # car (the tracks/wheels stick); a slower spin is sent at this speed.
    min_spin_wheel_mps: float = 0.0
    allow_reverse: bool = False
    # True when track_width_m came from car.yaml, False when it is the placeholder.
    measured: bool = False

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any]) -> "DriveConfig":
        d = cfg.get("drive") or {}
        track = d.get("track_width_m")
        measured = track is not None
        if track is None:
            track = d.get("assumed_track_width_m", cls.track_width_m)
        return cls(
            track_width_m=float(track),
            max_wheel_speed_mps=float(d.get("max_wheel_speed_mps", cls.max_wheel_speed_mps)),
            cruise_speed_mps=float(d.get("cruise_speed_mps", cls.cruise_speed_mps)),
            max_yaw_rate_rps=float(d.get("max_yaw_rate_rps", cls.max_yaw_rate_rps)),
            min_wheel_speed_mps=float(d.get("min_wheel_speed_mps", cls.min_wheel_speed_mps)),
            min_spin_wheel_mps=float(d.get("min_spin_wheel_mps", cls.min_spin_wheel_mps)),
            allow_reverse=bool(d.get("allow_reverse", cls.allow_reverse)),
            measured=measured,
        )

    def wheels(self, v: float, omega: float) -> tuple[float, float]:
        """Ideal wheel speeds for (v, omega), no limits."""
        h = 0.5 * self.track_width_m * omega
        return v - h, v + h

    def body(self, left: float, right: float) -> tuple[float, float]:
        """(v, omega) of base_link for wheel speeds (left, right), no slip."""
        return 0.5 * (left + right), (right - left) / self.track_width_m

    @property
    def spin_yaw_rate_rps(self) -> float:
        """Fastest turn on the spot the wheel speed limit allows."""
        return 2.0 * self.max_wheel_speed_mps / self.track_width_m


class CmdVelMapper:
    """Nav2 (v, omega) -> (left, right) wheel speeds (m/s)."""

    def __init__(self, cfg: DriveConfig) -> None:
        if cfg.track_width_m <= 0.0:
            raise ValueError("drive.track_width_m must be > 0")
        self.cfg = cfg

    @property
    def stop_payload(self) -> tuple[float, float]:
        return 0.0, 0.0

    def map(self, v: float, omega: float) -> tuple[float, float]:
        c = self.cfg
        if not (math.isfinite(v) and math.isfinite(omega)):
            return self.stop_payload
        if v < 0.0 and not c.allow_reverse:
            v = 0.0                   # never reverse: keep only the turn
        omega = min(max(omega, -c.max_yaw_rate_rps), c.max_yaw_rate_rps)
        left, right = c.wheels(v, omega)
        peak = max(abs(left), abs(right))
        if peak > c.max_wheel_speed_mps:
            k = c.max_wheel_speed_mps / peak   # same curvature, lower speed
            left, right = left * k, right * k
        if abs(v) < 1e-3 and abs(omega) > 1e-3 and 0.0 < abs(right) < c.min_spin_wheel_mps:
            k = c.min_spin_wheel_mps / abs(right)   # spin on the spot: at least the stiction speed
            left, right = left * k, right * k
        if abs(left) < c.min_wheel_speed_mps and abs(right) < c.min_wheel_speed_mps:
            return self.stop_payload
        return left, right


def format_payload(left: float, right: float) -> str:
    return f"{float(left):.3f},{float(right):.3f}"


def parse_payload(text: str) -> tuple[float, float] | None:
    try:
        a, b = text.strip().split(",")
        left, right = float(a), float(b)
    except (ValueError, AttributeError):
        return None
    if not (math.isfinite(left) and math.isfinite(right)):
        return None
    return left, right


def drive_from_cfg(cfg: dict[str, Any]) -> DriveConfig:
    return DriveConfig.from_cfg(cfg)
