"""CLI: world yaml -> map (pgm + yaml) and Gazebo SDF.

    ros2 run car_nav_sim make_world test_floor /tmp/car_nav_sim
"""
from __future__ import annotations

import os
import sys

from car_nav import config_loader

from . import sim_config
from .sdf import world_sdf
from .world import World


def build(world_name: str, out_dir: str, start=None) -> tuple[str, str]:
    world = World.load(sim_config.resolve("worlds", world_name))
    map_yaml = world.write_map(out_dir)
    sdf_path = os.path.join(out_dir, f"{world.name}.sdf")
    with open(sdf_path, "w", encoding="utf-8") as fh:
        fh.write(world_sdf(world, config_loader.load(), sim_config.load_sim(), start))
    return map_yaml, sdf_path


def main() -> None:
    if len(sys.argv) < 3:
        print("usage: make_world <world name or yaml> <out dir>")
        sys.exit(2)
    m, s = build(sys.argv[1], sys.argv[2])
    print(f"map: {m}\nsdf: {s}")
