"""Live map position for the dashboard Map tab.

Wraps TilePositionTracker for the Pi5 runtime: the dashboard sends the map
open in its editor plus a start cell and a heading or goal, the main loop
feeds the cross-line tile count every frame, and the current cell goes to
/api/status and the route CSV. Thread-safe because the HTTP server calls in
from its own threads. Map files and route preview live in map_store.

The start (map + start cell + heading/goal) is saved to a file and survives
restarts. Every route start resets the tile count and puts the car back on
the saved start.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Any

import numpy as np

from positioning.map_panel import MapPanel
from positioning.map_store import default_saved_dir, validate_map_data
from positioning.tile_position import HEADINGS, TileMap, TilePositionTracker

logger = logging.getLogger(__name__)

DEFAULT_MAP = Path(__file__).resolve().parents[1] / "maps" / "tile_maps" / "campus-path.json"


class MapSession:
    def __init__(self, tile_map: TileMap | None, start_file: Path | None = None) -> None:
        self._map = tile_map
        self._start_file = start_file
        self._start_body: dict[str, Any] | None = None
        self._lock = threading.Lock()
        self._tracker: TilePositionTracker | None = None
        self._setup: dict[str, Any] = {}
        self._base_count = 0
        self._last_count = 0
        self._position: dict[str, Any] | None = None
        self._trail: list[list[int]] = []
        self._panel: MapPanel | None = None
        self._load_saved_start()

    @classmethod
    def from_env(cls) -> "MapSession":
        path = Path(os.getenv("TILE_MAP_PATH", str(DEFAULT_MAP)))
        start_file = Path(os.getenv("TILE_MAP_START_FILE", str(default_saved_dir() / "start_position.json")))
        try:
            tile_map = TileMap.load(path)
        except (OSError, ValueError, KeyError) as exc:
            logger.warning("Tile map unavailable (%s): %s", path, exc)
            tile_map = None
        return cls(tile_map, start_file)

    def _load_saved_start(self) -> None:
        if self._start_file is None or not self._start_file.is_file():
            return
        try:
            self.configure(json.loads(self._start_file.read_text(encoding="utf-8")), persist=False)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            logger.warning("Saved map start ignored (%s): %s", self._start_file, exc)

    def _persist_start(self, body: dict[str, Any]) -> None:
        if self._start_file is None:
            return
        self._start_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._start_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(body), encoding="utf-8")
        tmp.replace(self._start_file)

    # ------------------------------------------------------------------ #
    # Dashboard API
    # ------------------------------------------------------------------ #
    def map_payload(self) -> dict[str, Any]:
        if self._map is None:
            raise ValueError("no tile map loaded")
        return {
            "map_id": self._map.map_id,
            "tile_size_m": self._map.tile_size_m,
            "bounds": self._map.bounds,
            "cells": sorted([list(c) for c in self._map.cells]),
            "anchors": [
                {"id": a.anchor_id, "label": a.label, "cell": list(a.cell)}
                for a in self._map.anchors.values()
            ],
            "headings": sorted(HEADINGS),
        }

    @staticmethod
    def _resolve(tile_map: TileMap, ref: Any) -> tuple[int, int]:
        """[row, col], an anchor id or a "row,col" string."""
        if isinstance(ref, (list, tuple)) and len(ref) == 2:
            return int(ref[0]), int(ref[1])
        return tile_map.resolve_cell(str(ref or "").strip())

    def configure(self, body: dict[str, Any], persist: bool = True) -> dict[str, Any]:
        """Save and arm the start.

        Body: {"map"?: map data, "start": cell, "heading": ...} or {..., "goal": cell}.
        """
        map_data = validate_map_data(body["map"]) if body.get("map") else None
        tile_map = TileMap.from_dict(map_data) if map_data else self._map
        if tile_map is None:
            raise ValueError("no tile map loaded")
        start = self._resolve(tile_map, body.get("start"))
        if not tile_map.is_runnable(start):
            raise ValueError(f"start cell {start} is not runnable")
        heading = body.get("heading") or None
        goal_ref = body.get("goal") or None
        if (heading is None) == (goal_ref is None):
            raise ValueError("give a heading or a goal")
        path = None
        if goal_ref is not None:
            path = tile_map.shortest_path(start, self._resolve(tile_map, goal_ref))
        tracker = TilePositionTracker(tile_map, start, heading=heading, path=path)
        saved = {
            "map": map_data,
            "start": list(start),
            "heading": heading,
            "goal": None if path is None else list(path[-1]),
        }
        if persist:
            self._persist_start(saved)
        panel = MapPanel(tile_map, None if path is None else [list(c) for c in path])
        with self._lock:
            self._panel = panel
            self._start_body = saved
            self._map = tile_map
            self._tracker = tracker
            self._base_count = self._last_count
            self._setup = {
                "map_id": tile_map.map_id,
                "start": list(start),
                "heading": heading,
                "goal": None if path is None else list(path[-1]),
                "path": None if path is None else [list(c) for c in path],
            }
            self._trail = [list(start)]
            self._position = tracker.update(0).as_dict()
        return self.state()

    def mark_anchor(self, body: dict[str, Any]) -> dict[str, Any]:
        """The car is at this anchor (or "r,c") now."""
        if self._map is None:
            raise ValueError("no tile map loaded")
        tile_map = self._map
        cell = self._resolve(tile_map, body.get("ref"))
        if not tile_map.is_runnable(cell):
            raise ValueError(f"anchor cell {cell} is not runnable")
        with self._lock:
            if self._tracker is None:
                raise ValueError("save a start position in the Map tab first")
            tracker = self._tracker
            path = tracker.path
        if path is not None and cell not in path:
            # Off the planned route: re-plan from the anchor to the goal and
            # count from here (the saved start is unchanged).
            new_path = tile_map.shortest_path(cell, path[-1])
            new_tracker = TilePositionTracker(tile_map, cell, path=new_path)
            panel = MapPanel(tile_map, [list(c) for c in new_path])
            with self._lock:
                self._tracker = new_tracker
                self._base_count = self._last_count
                self._setup["path"] = [list(c) for c in new_path]
                self._panel = panel
                self._position = new_tracker.update(0).as_dict()
                self._trail.append(list(cell))
            return self.state()
        with self._lock:
            count = self._last_count - self._base_count
            self._tracker.apply_anchor(cell, count)
            self._position = self._tracker.update(count).as_dict()
            self._trail.append(list(cell))
        return self.state()

    def apply_turn(self, rotation_deg: float) -> None:
        """A script turn finished: rotate the map heading by the nearest
        multiple of 90 degrees (left positive). Seams counted during the
        turn were frozen by update(), so the car stays on its cell."""
        quarter = int(round(float(rotation_deg) / 90.0))
        with self._lock:
            if self._tracker is None or quarter == 0:
                return
            count = self._last_count - self._base_count
            self._tracker.apply_turn(quarter, count)
            self._position = self._tracker.update(count).as_dict()
            logger.info("Map turn %+d x 90 deg -> heading %s", quarter, self._tracker.heading)

    def apply_lane_shift(self, side: int) -> None:
        """A passive lane shift: the car moved one row sideways (side +1 =
        to its left). Handled like an anchor at the neighbouring cell, so path
        mode re-plans from there."""
        with self._lock:
            if self._tracker is None or self._map is None:
                return
            count = self._last_count - self._base_count
            here = self._tracker.update(count).cell
            dr, dc = self._tracker.direction(count)
            if (dr, dc) == (0, 0):
                return
            left = (-dc, dr)
            cell = (here[0] + side * left[0], here[1] + side * left[1])
        try:
            self.mark_anchor({"ref": list(cell)})
            logger.info("Lane shift %s: map cell %s -> %s", "left" if side > 0 else "right", here, cell)
        except ValueError as exc:
            logger.warning("Lane shift to %s ignored: %s", cell, exc)

    def clear(self, _body: dict[str, Any] | None = None) -> dict[str, Any]:
        """Stop tracking and forget the saved start."""
        if self._start_file is not None:
            self._start_file.unlink(missing_ok=True)
        with self._lock:
            self._start_body = None
            self._panel = None
            self._tracker = None
            self._setup = {}
            self._position = None
            self._trail = []
        return self.state()

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": self._map is not None,
                "configured": self._tracker is not None,
                "saved_start": self._start_body is not None,
                "setup": dict(self._setup),
                "tiles": self._last_count - self._base_count,
                "position": None if self._position is None else dict(self._position),
                "trail": [list(c) for c in self._trail[-400:]],
            }

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def route_start(self) -> None:
        """Route start: the tile counter is back at 0, the car is at the saved start."""
        with self._lock:
            self._last_count = 0
            self._base_count = 0
            body = self._start_body
        if body is None:
            return
        try:
            self.configure(dict(body), persist=False)
        except (ValueError, KeyError) as exc:
            logger.warning("Saved map start no longer valid: %s", exc)

    def saved_route_steps(self) -> dict[str, Any]:
        """Steps for the A* route from the saved start to the saved goal."""
        from positioning.route_steps import path_to_steps

        with self._lock:
            body = self._start_body
            path = self._setup.get("path")
            map_id = self._setup.get("map_id")
        if body is None:
            raise ValueError("save a start position in the Map tab first")
        if not path or body.get("goal") is None:
            raise ValueError("the saved start has no goal: set a goal and save again")
        cells = [(int(r), int(c)) for r, c in path]
        return {"map_id": map_id, "path": [list(c) for c in cells], "steps": path_to_steps(cells)}

    def start_summary(self) -> dict[str, Any] | None:
        """The saved start without the map cells (for route_summary.json)."""
        with self._lock:
            if self._start_body is None:
                return None
            return {
                "map_id": self._setup.get("map_id"),
                "start": self._start_body["start"],
                "heading": self._start_body["heading"],
                "goal": self._start_body["goal"],
            }

    def update(self, tile_count: int, frozen: bool = False) -> None:
        """New net tile count (forward seams minus reversed ones). frozen: the
        car is turning (forward or in reverse), so seams counted now do not
        move it on the map."""
        with self._lock:
            if frozen:
                self._base_count += int(tile_count) - self._last_count
            self._last_count = int(tile_count)
            if self._tracker is None:
                return
            position = self._tracker.update(self._last_count - self._base_count)
            cell = [position.cell[0], position.cell[1]]
            if not self._trail or self._trail[-1] != cell:
                self._trail.append(cell)
            self._position = position.as_dict()

    def render_panel(self, height: int, width: int) -> np.ndarray | None:
        """Map view around the car for the recorded video (None if no start)."""
        with self._lock:
            panel = self._panel
            if panel is None or self._tracker is None:
                return None
            state = {
                "position": None if self._position is None else dict(self._position),
                "trail": list(self._trail[-400:]),
                "tiles": self._last_count - self._base_count,
            }
        return panel.render(height, width, state)

    def telemetry(self) -> dict[str, Any]:
        with self._lock:
            if self._position is None:
                return {"map_row": None, "map_column": None, "map_status": "UNSET"}
            return {
                "map_row": self._position["row"],
                "map_column": self._position["column"],
                "map_status": self._position["status"],
            }
