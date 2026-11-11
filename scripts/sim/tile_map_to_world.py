"""Tile map (maps/tile_maps/*.json) -> simulator world (ros/car_nav_sim/worlds/*.yaml).

    python3 scripts/sim/tile_map_to_world.py                       # campus-path -> campus_path.yaml
    python3 scripts/sim/tile_map_to_world.py maps/tile_maps/x.json ros/car_nav_sim/worlds/x.yaml

Every runnable cell is floor. Every edge between a runnable cell and a
non-runnable one becomes a thin wall (on the non-runnable side, so the
corridor keeps its full tile width). Assumption: the LiDAR sees something
at every map edge (wall, kerb, bench row). Where the real place is open
(grass, open hall) the sim is easier to localise in than reality.

Frame: x = east (column), y = north (-row), metres, one tile of margin, so
cell (row, col) has its centre at
    x = (col - min_col + 1.5) * tile,   y = (max_row - row + 1.5) * tile.
Anchors of the tile map become named start poses (`anchors:` in the world
file, used by `sim.launch.py start:=<anchor id>`): the pose near the
anchor cell where the car (footprint from car.yaml) has the most room.
Needs numpy + PyYAML (pip install numpy pyyaml).
"""
from __future__ import annotations

import json
import math
import os
import sys

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WALL = 0.10      # wall thickness (m)


class TileFrame:
    def __init__(self, tile: float, min_col: int, max_row: int) -> None:
        self.t, self.c0, self.r0 = tile, min_col, max_row

    def centre(self, row: int, col: int) -> tuple[float, float]:
        return ((col - self.c0 + 1.5) * self.t, (self.r0 - row + 1.5) * self.t)


def _runs(keys: list[int]) -> list[tuple[int, int]]:
    """Sorted ints -> [(first, last)] runs of consecutive values."""
    out: list[tuple[int, int]] = []
    for k in sorted(keys):
        if out and k == out[-1][1] + 1:
            out[-1] = (out[-1][0], k)
        else:
            out.append((k, k))
    return out


def _footprint() -> tuple[float, float, float]:
    with open(os.path.join(REPO, "ros", "car_nav", "config", "car.yaml"), "r", encoding="utf-8") as fh:
        fp = (yaml.safe_load(fh) or {})["footprint"]
    return float(fp["x_front"]), float(fp["x_rear"]), float(fp["half_width"])


def anchor_pose(world, fr: TileFrame, r: int, c: int) -> list[float]:
    """Start pose at an anchor cell: the pose within half a tile of the cell
    centre, facing east / north / west / south, where the car's footprint
    has the most room (ties: facing east or north, closest to the centre)."""
    from car_nav_sim.world import footprint_polygon

    xf, xr, hw = _footprint()
    cx, cy = fr.centre(r, c)
    h = fr.t / 2
    best = None
    for k, yaw in enumerate((0.0, math.pi / 2, math.pi, -math.pi / 2)):
        for dx in (0.0, -h, h):
            for dy in (0.0, -h, h):
                x, y = cx + dx, cy + dy
                clear = world.clearance(footprint_polygon(x, y, yaw, xf, xr, hw))
                key = (round(min(clear, 0.6), 2), -(k >= 2), -abs(dx) - abs(dy))  # room, east/north, centre
                if best is None or key > best[0]:
                    best = (key, [x, y, yaw])
    return best[1]


def convert(tile_map: dict, name: str | None = None, start_anchor: str | None = None) -> dict:
    t = float(tile_map["tile_size_m"])
    cells = {(int(r), int(c)) for r, c in tile_map["runnable_cells"]}
    rows = [r for r, _ in cells]
    cols = [c for _, c in cells]
    fr = TileFrame(t, min(cols), max(rows))
    width = (max(cols) - min(cols) + 3) * t
    height = (max(rows) - min(rows) + 3) * t

    # wall edges, grouped by line so neighbouring edges merge into one box
    horiz: dict[tuple[int, int], list[int]] = {}   # (row line, side) -> cols ; side +1 = wall north of cell
    vert: dict[tuple[int, int], list[int]] = {}    # (col line, side) -> rows ; side +1 = wall east of cell
    for r, c in cells:
        if (r - 1, c) not in cells:
            horiz.setdefault((r, +1), []).append(c)
        if (r + 1, c) not in cells:
            horiz.setdefault((r, -1), []).append(c)
        if (r, c + 1) not in cells:
            vert.setdefault((c, +1), []).append(r)
        if (r, c - 1) not in cells:
            vert.setdefault((c, -1), []).append(r)

    boxes = []
    for (r, side), cs in sorted(horiz.items()):
        _, yc = fr.centre(r, 0)
        y = yc + side * (t / 2 + WALL / 2)
        for a, b in _runs(cs):
            xa, _ = fr.centre(r, a)
            xb, _ = fr.centre(r, b)
            boxes.append({"center": [round((xa + xb) / 2, 4), round(y, 4)],
                          "size": [round(xb - xa + t + 2 * WALL, 4), WALL]})
    for (c, side), rs in sorted(vert.items()):
        xc, _ = fr.centre(0, c)
        x = xc + side * (t / 2 + WALL / 2)
        for a, b in _runs(rs):
            _, ya = fr.centre(a, c)
            _, yb = fr.centre(b, c)
            boxes.append({"center": [round(x, 4), round((ya + yb) / 2, 4)],
                          "size": [WALL, round(abs(ya - yb) + t + 2 * WALL, 4)]})

    sys.path.insert(0, os.path.join(REPO, "ros", "car_nav_sim"))
    from car_nav_sim.world import World

    walls = World.from_dict({"bounds": [0, 0, width, height], "boxes": boxes})
    anchors = {}
    for a in tile_map.get("visual_anchors") or []:
        r, c = int(a["cell"][0]), int(a["cell"][1])
        pose = anchor_pose(walls, fr, r, c)
        anchors[str(a["id"])] = {"label": a.get("label", ""), "cell": [r, c],
                                 "pose": [round(v, 4) for v in pose]}

    if start_anchor is None:
        start_anchor = next(iter(anchors), None)
    if start_anchor is not None:
        start = anchors[start_anchor]["pose"]
    else:
        r, c = min(cells)
        start = [*fr.centre(r, c), 0.0]

    map_id = str(tile_map.get("map_id", "tile_map"))
    return {
        "name": name or map_id.replace("-", "_"),
        "source": f"maps/tile_maps/{map_id}.json via scripts/sim/tile_map_to_world.py (do not edit by hand)",
        "tile_size_m": t,
        "cell_frame": {"min_column": min(cols), "max_row": max(rows),
                       "centre": "x=(col-min_column+1.5)*tile, y=(max_row-row+1.5)*tile"},
        "bounds": [0.0, 0.0, round(width, 3), round(height, 3)],
        "start": [float(v) for v in start],
        "anchors": anchors,
        "boxes": boxes,
    }


def main(argv: list[str]) -> int:
    src = argv[0] if argv else os.path.join(REPO, "maps", "tile_maps", "campus-path.json")
    with open(src, "r", encoding="utf-8") as fh:
        tile_map = json.load(fh)
    name = str(tile_map.get("map_id", "tile_map")).replace("-", "_")
    dst = argv[1] if len(argv) > 1 else os.path.join(REPO, "ros", "car_nav_sim", "worlds", f"{name}.yaml")
    anchor = argv[2] if len(argv) > 2 else ("AAD-1" if name == "campus_path" else None)
    world = convert(tile_map, name, anchor)
    with open(dst, "w", encoding="utf-8") as fh:
        fh.write(f"# {world['name']}: generated from the tile map, do not edit by hand.\n")
        fh.write("# Regenerate: python3 scripts/sim/tile_map_to_world.py\n")
        yaml.safe_dump(world, fh, sort_keys=False, default_flow_style=None, width=120)
    print(f"{dst}: {len(tile_map['runnable_cells'])} cells, {len(world['boxes'])} wall boxes, "
          f"{world['bounds'][2]:.1f} x {world['bounds'][3]:.1f} m, start {world['start']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
