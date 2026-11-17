"""Tile map -> simulator world (scripts/sim/tile_map_to_world.py)."""
import importlib.util
import json
import math

import numpy as np
import pytest
import yaml

from tests import _car_nav_paths as P  # noqa: F401  (puts ros/car_nav_sim on sys.path)
from car_nav_sim.world import World, footprint_polygon

REPO = P.SIM_DIR.parents[1]
spec = importlib.util.spec_from_file_location("tile_map_to_world", REPO / "scripts" / "sim" / "tile_map_to_world.py")
t2w = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t2w)


def _world(cells):
    return World.from_dict(t2w.convert({"map_id": "t", "tile_size_m": 0.6, "runnable_cells": cells}, "t"))


def test_corridor_keeps_full_tile_width():
    # 2 rows x 5 cols corridor running east
    w = _world([[r, c] for r in (0, 1) for c in range(5)])
    fr = t2w.TileFrame(0.6, 0, 1)
    x, y = fr.centre(0, 2)          # row 0 = north row
    up, down = w.raycast(x, y, np.array([math.pi / 2, -math.pi / 2]), 10.0)
    assert up == pytest.approx(0.3) and down == pytest.approx(0.9)
    # walls all round: a ray east along the corridor ends at the east wall
    assert w.raycast(x, y, np.array([0.0]), 10.0)[0] == pytest.approx(2 * 0.6 + 0.3)


def test_anchor_start_pose_is_clear_of_walls():
    cells = [[r, c] for r in (0, 1) for c in range(6)]
    tm = {"map_id": "t", "tile_size_m": 0.6, "runnable_cells": cells,
          "visual_anchors": [{"id": "A", "cell": [0, 1]}]}
    d = t2w.convert(tm, "t", "A")
    w = World.from_dict(d)
    x, y, th = d["start"]
    assert d["anchors"]["A"]["pose"] == d["start"]
    assert w.clearance(footprint_polygon(x, y, th, 0.16, -0.37, 0.135)) > 0.4


def test_campus_world_is_up_to_date():
    tm = json.loads((REPO / "maps" / "tile_maps" / "campus-path.json").read_text())
    fresh = t2w.convert(tm, "campus_path", "AAD-1")
    saved = yaml.safe_load((P.SIM_DIR / "worlds" / "campus_path.yaml").read_text())
    assert saved == yaml.safe_load(yaml.safe_dump(fresh)), "re-run scripts/sim/tile_map_to_world.py"
