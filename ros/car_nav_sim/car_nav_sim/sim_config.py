"""Paths and loading for the simulator's own files (sim.yaml, worlds, missions)."""
from __future__ import annotations

import os

import yaml

SIM_CONFIG_ENV = "CAR_NAV_SIM_CONFIG"


def share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory("car_nav_sim")


def resolve(kind: str, name_or_path: str) -> str:
    """'worlds', 'test_floor' -> <share>/worlds/test_floor.yaml; paths pass through."""
    if os.path.sep in name_or_path or name_or_path.endswith(".yaml"):
        return name_or_path
    return os.path.join(share_dir(), kind, f"{name_or_path}.yaml")


def load_sim(path: str | None = None) -> dict:
    path = path or os.environ.get(SIM_CONFIG_ENV) or os.path.join(share_dir(), "config", "sim.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    cfg["_path"] = path
    return cfg
