#!/usr/bin/env python3
"""Replay route_raw.avi, count tiles and place the car on the campus tile map.

Examples:
    # Straight run from the AAD-1 sign heading east (towards gym-01)
    python scripts/replay_tile_position.py routes/route-X --start AAD-1 --heading east

    # Follow the shortest map path from AAD-1 to gym-01 (16 tiles)
    python scripts/replay_tile_position.py routes/route-X --start AAD-1 --goal gym-01

    # Cells as row,column; a manually noted anchor passage at frame 240
    python scripts/replay_tile_position.py routes/route-X --start=-31,5 --heading east --anchor 240:gym-01

Writes per route into --out/<route_id>/: tile_position.csv (one row per frame)
and tile_position_map.png (runnable cells grey, planned path light blue,
estimated cells dark blue, CONTRADICTORY cells red, anchors orange).
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from positioning.tile_position import TileMap, TilePositionTracker  # noqa: E402
from scripts.replay_cross_lines import _find_video  # noqa: E402
from vision.cross_line_counter import CrossLineConfig, CrossLineCounter  # noqa: E402

DEFAULT_MAP = Path(__file__).resolve().parents[1] / "maps" / "tile_maps" / "campus-path.json"


def draw_map(tile_map: TileMap, path, visited, bad, scale: int = 10) -> np.ndarray:
    b = tile_map.bounds
    rows = b["maximum_row"] - b["minimum_row"] + 1
    cols = b["maximum_column"] - b["minimum_column"] + 1
    img = np.full((rows * scale, cols * scale, 3), 255, dtype=np.uint8)

    def fill(cell, color):
        y = (cell[0] - b["minimum_row"]) * scale
        x = (cell[1] - b["minimum_column"]) * scale
        if 0 <= y < img.shape[0] and 0 <= x < img.shape[1]:
            cv2.rectangle(img, (x, y), (x + scale - 1, y + scale - 1), color, -1)

    for cell in tile_map.cells:
        fill(cell, (190, 190, 190))
    for cell in path or []:
        fill(cell, (240, 200, 150))
    for cell in visited:
        fill(cell, (160, 60, 0))
    for cell in bad:
        fill(cell, (0, 0, 230))
    for anchor in tile_map.anchors.values():
        fill(anchor.cell, (0, 140, 255))
        y = (anchor.cell[0] - b["minimum_row"]) * scale
        x = (anchor.cell[1] - b["minimum_column"]) * scale
        cv2.putText(img, anchor.anchor_id, (x + scale + 2, y + scale), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 200), 1)
    return img


def replay(route: Path, out_root: Path, tile_map: TileMap, args, config: CrossLineConfig) -> dict:
    video = _find_video(route)
    if video is None:
        return {"route": route.name, "error": "no route_raw.avi"}
    route_id = video.parent.name
    out_dir = out_root / route_id
    out_dir.mkdir(parents=True, exist_ok=True)

    start = tile_map.resolve_cell(args.start)
    path = tile_map.shortest_path(start, tile_map.resolve_cell(args.goal)) if args.goal else None
    tracker = TilePositionTracker(tile_map, start, heading=None if path else args.heading, path=path)
    anchors = {}
    for item in args.anchor:
        frame_s, _, ref = item.partition(":")
        anchors[int(frame_s)] = tile_map.resolve_cell(ref)

    counter = CrossLineCounter(config)
    cap = cv2.VideoCapture(str(video))
    visited: list[tuple[int, int]] = [start]
    bad: list[tuple[int, int]] = []
    position = tracker.update(0)
    frame_idx = 0
    with (out_dir / "tile_position.csv").open("w", newline="", encoding="utf-8") as fh:
        rows = csv.writer(fh)
        rows.writerow(["frame", "tile_count", "counted", "index", "row", "column", "status"])
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            result = counter.update(frame)
            if frame_idx in anchors:
                tracker.apply_anchor(anchors[frame_idx], result.tile_count)
            position = tracker.update(result.tile_count)
            if result.counted_this_frame or frame_idx in anchors:
                (visited if position.status == "OK" else bad).append(position.cell)
            rows.writerow([
                frame_idx, result.tile_count, int(result.counted_this_frame), position.index,
                position.cell[0], position.cell[1], position.status,
            ])
            frame_idx += 1
    cap.release()
    cv2.imwrite(str(out_dir / "tile_position_map.png"), draw_map(tile_map, path, visited, bad))
    summary = {
        "route": route_id,
        "frames": frame_idx,
        "start": start,
        "heading": None if path else args.heading,
        "goal": args.goal,
        "path_tiles": len(path) - 1 if path else None,
        "tile_count": counter.tile_count,
        "distance_m": round(counter.tile_count * tile_map.tile_size_m, 2),
        "end_cell": position.cell,
        "end_status": position.status,
        "contradictory_cells": len(bad),
    }
    (out_dir / "tile_position_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="+", type=Path, help="route folders (containing route_raw.avi)")
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP, help="tile map (.json, or .yaml with PyYAML)")
    parser.add_argument("--start", required=True, help="start anchor id or row,column")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--heading", choices=["north", "south", "east", "west"], help="straight run direction")
    mode.add_argument("--goal", help="goal anchor id or row,column; follows the shortest map path")
    parser.add_argument("--anchor", action="append", default=[], metavar="FRAME:REF",
                        help="car is at anchor/cell REF at FRAME (repeatable)")
    parser.add_argument("--out", type=Path, default=Path("tile_position_replay"), help="output folder")
    args = parser.parse_args()

    tile_map = TileMap.load(args.map)
    config = CrossLineConfig.from_env()
    for route in args.routes:
        try:
            s = replay(route, args.out, tile_map, args, config)
        except ValueError as exc:
            print(f"{route.name}: {exc}")
            continue
        if "error" in s:
            print(f"{s['route']}: {s['error']}")
            continue
        expected = f" (path {s['path_tiles']} tiles)" if s["path_tiles"] is not None else ""
        print(
            f"{s['route']}: tiles={s['tile_count']} (~{s['distance_m']} m){expected} "
            f"end={tuple(s['end_cell'])} {s['end_status']} contradictory={s['contradictory_cells']}"
        )


if __name__ == "__main__":
    main()
