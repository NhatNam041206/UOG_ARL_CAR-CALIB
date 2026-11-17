"""ugv-controller (ESP32-C3) serial protocol, pure functions (no ROS, no serial imports).

The car's motor controller (`ugv-controller`, ESP32-C3 Super Mini, USB CDC
/dev/ttyACM0 at 115200) takes one ASCII line per command:

    "<left> <right>\\n"   integers -500..500, 0 = stop, + = forward
    "stop\\n"             both wheels 0

and drives each side's ESC with a 50 Hz pulse of 1500 + speed us. It
answers "OK: L=... R=...", "ERR: ...", and streams
"IMU: ax=.. ay=.. az=.. gx=.. gy=.. gz=.. temp=.." at 10 Hz (MPU6050:
acceleration in g, rates in deg/s).

The ESCs are open loop: "speed" is a pulse width, not m/s. UgvConfig maps
m/s to those units with a measured (or assumed) scale and the ESC dead band.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

G = 9.80665


@dataclass(frozen=True)
class UgvConfig:
    # Controller units per m/s at the tread. ASSUMED 250 until measured: then
    # NAV2's 0.4 m/s cruise is "100 100", the speed the controller's guide
    # says to start with. If the car is faster than that, adaptive_odom's
    # learned ratio shows it (> 1); measure and set it here.
    units_per_mps: float = 250.0
    # ESC dead band: |units| below this does not turn the wheel. A non-zero
    # wheel command is sent as sign * (deadband + |v| * units_per_mps).
    deadband_units: int = 0
    # Never send more than this (|units|), whatever Nav2 asks. Safety cap.
    max_units: int = 200
    # Wiring: flip a side whose motor runs backwards, or swap the two sides.
    invert_left: bool = False
    invert_right: bool = False
    swap_sides: bool = False
    measured: bool = False

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any]) -> "UgvConfig":
        u = cfg.get("ugv") or {}
        scale = u.get("units_per_mps")
        measured = scale is not None
        if scale is None:
            scale = u.get("assumed_units_per_mps", cls.units_per_mps)
        return cls(
            units_per_mps=float(scale),
            deadband_units=int(u.get("deadband_units", cls.deadband_units) or 0),
            max_units=int(u.get("max_units", cls.max_units)),
            invert_left=bool(u.get("invert_left", cls.invert_left)),
            invert_right=bool(u.get("invert_right", cls.invert_right)),
            swap_sides=bool(u.get("swap_sides", cls.swap_sides)),
            measured=measured,
        )

    def wheel_units(self, mps: float) -> int:
        if not math.isfinite(mps) or abs(mps) < 1e-4:
            return 0
        u = self.deadband_units + abs(mps) * self.units_per_mps
        u = min(u, float(self.max_units), 500.0)
        return int(round(math.copysign(u, mps)))

    def command(self, left_mps: float, right_mps: float) -> str:
        """Wheel speeds (m/s) -> the controller's line, e.g. '100 100\\n'."""
        left, right = self.wheel_units(left_mps), self.wheel_units(right_mps)
        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right
        if self.swap_sides:
            left, right = right, left
        return f"{left} {right}\n"


STOP_LINE = "0 0\n"


@dataclass
class ImuSample:
    ax: float  # m/s^2
    ay: float
    az: float
    gx: float  # rad/s
    gy: float
    gz: float
    temp: float


def parse_imu(line: str) -> ImuSample | None:
    """'IMU: ax=0.012 ay=-0.005 az=0.998 gx=0.12 gy=-0.08 gz=0.03 temp=31.25' -> SI units."""
    line = line.strip()
    if not line.startswith("IMU:"):
        return None
    vals: dict[str, float] = {}
    for tok in line[4:].split():
        k, sep, v = tok.partition("=")
        if not sep:
            continue
        try:
            vals[k] = float(v)
        except ValueError:
            return None
    try:
        d = math.radians
        return ImuSample(vals["ax"] * G, vals["ay"] * G, vals["az"] * G,
                         d(vals["gx"]), d(vals["gy"]), d(vals["gz"]), vals.get("temp", math.nan))
    except KeyError:
        return None


def parse_ok(line: str) -> tuple[int, int] | None:
    """'OK: L=100 (1600us, duty=1310)  R=100 (...)' -> (100, 100)."""
    line = line.strip()
    if not line.startswith("OK: L="):
        return None
    try:
        left = int(line.split("L=", 1)[1].split()[0])
        right = int(line.split("R=", 1)[1].split()[0])
    except (IndexError, ValueError):
        return None
    return left, right
