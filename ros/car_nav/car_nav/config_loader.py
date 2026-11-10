"""Load ros/car_nav/config/car.yaml and report unmeasured (null) values."""
from __future__ import annotations

import math
import os
from typing import Any

import yaml

from .geometry import Footprint, Pose3D

DEFAULT_PATH_ENV = "CAR_NAV_CONFIG"


def load(path: str | None = None) -> dict[str, Any]:
    if path is None:
        path = os.environ.get(DEFAULT_PATH_ENV, "")
    if not path:
        from ament_index_python.packages import get_package_share_directory

        path = os.path.join(get_package_share_directory("car_nav"), "config", "car.yaml")
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
