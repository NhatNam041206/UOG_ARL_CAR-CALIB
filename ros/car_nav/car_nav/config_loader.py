"""Load the car's geometry file (car.yaml) and report unmeasured (null) values.

Which file is read (first match):

1. the explicit `path` argument,
2. the WRITABLE copy named by CAR_NAV_CONFIG_OVERRIDE, if that file exists (the web UI's
   Car tab "Apply to the car" writes it; on the car /data/nav/ui/car.yaml),
3. CAR_NAV_CONFIG (the shipped file, mounted read-only on the car),
4. the copy installed with the package.

`DELETE /api/car/active` removes the override: the shipped file is used again.
"""
from __future__ import annotations

import math
import os
from typing import Any

import yaml

from .geometry import Footprint, Pose3D

DEFAULT_PATH_ENV = "CAR_NAV_CONFIG"
OVERRIDE_ENV = "CAR_NAV_CONFIG_OVERRIDE"


def override_path() -> str:
    """Path of the writable copy ('' = none configured)."""
    return os.environ.get(OVERRIDE_ENV, "")


def shipped_path() -> str:
    """The shipped file (CAR_NAV_CONFIG, else the package copy)."""
    path = os.environ.get(DEFAULT_PATH_ENV, "")
    if path:
        return path
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("car_nav"), "config", "car.yaml")


def active_path() -> str:
    """The file load() reads now."""
    ov = override_path()
    if ov and os.path.isfile(ov):
        return ov
    return shipped_path()


def load(path: str | None = None) -> dict[str, Any]:
    if path is None:
        path = active_path()
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    cfg["_path"] = path
    return cfg


def unmeasured(cfg: dict[str, Any]) -> list[str]:
    """Dotted keys whose value is null (not measured yet)."""
    missing: list[str] = []
    for section in ("laser", "camera", "speed"):
        for key, value in (cfg.get(section) or {}).items():
            if value is None:
                missing.append(f"{section}.{key}")
    return missing


def _num(value: Any) -> float:
    return 0.0 if value is None else float(value)


def laser_pose(cfg: dict[str, Any]) -> Pose3D:
    s = cfg.get("laser") or {}
    return Pose3D(_num(s.get("x")), _num(s.get("y")), _num(s.get("z")),
                  _num(s.get("roll")), _num(s.get("pitch")), _num(s.get("yaw")))


def camera_pose(cfg: dict[str, Any]) -> Pose3D:
    s = cfg.get("camera") or {}
    return Pose3D(_num(s.get("x")), _num(s.get("y")), _num(s.get("z")),
                  _num(s.get("roll")), math.radians(_num(s.get("pitch_deg"))), _num(s.get("yaw")))


def footprint(cfg: dict[str, Any]) -> Footprint:
    s = cfg.get("footprint") or {}
    return Footprint(float(s["x_front"]), float(s["x_rear"]), float(s["half_width"]))
