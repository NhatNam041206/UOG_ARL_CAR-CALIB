"""Live map position for the dashboard Map tab.

Wraps TilePositionTracker for the Pi5 runtime: the dashboard sends the map
open in its editor plus a start cell and a heading or goal, the main loop
feeds the cross-line tile count every frame, and the current cell goes to
/api/status and the route CSV. Thread-safe because the HTTP server calls in
from its own threads. Map files and route preview live in map_store.
"""

from __future__ import annotations

import logging
import os
import threading
from pathlib import Path
from typing import Any

from positioning.map_store import validate_map_data
from positioning.tile_position import HEADINGS, TileMap, TilePositionTracker

logger = logging.getLogger(__name__)

DEFAULT_MAP = Path(__file__).resolve().parents[1] / "routes" / "tile_maps" / "campus-path.json"


class MapSession:
    def __init__(self, tile_map: TileMap | None) -> None:
        self._map = tile_map
        self._lock = threading.Lock()
        self._tracker: TilePositionTracker | None = None
        self._setup: dict[str, Any] = {}
        self._base_count = 0
        self._last_count = 0
        self._position: dict[str, Any] | None = None
        self._trail: list[list[int]] = []

    @classmethod
    def from_env(cls) -> "MapSession":
        path = Path(os.getenv("TILE_MAP_PATH", str(DEFAULT_MAP)))
        try:
            return cls(TileMap.load(path))
        except (OSError, ValueError, KeyError) as exc:
            logger.warning("Tile map unavailable (%s): %s", path, exc)
            return cls(None)

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

    def configure(self, body: dict[str, Any]) -> dict[str, Any]:
        """Body: {"map"?: map data, "start": cell, "heading": ...} or {..., "goal": cell}."""
        tile_map = TileMap.from_dict(validate_map_data(body["map"])) if body.get("map") else self._map
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
        with self._lock:
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
        cell = self._resolve(self._map, body.get("ref"))
        with self._lock:
            if self._tracker is None:
                raise ValueError("set a start first")
            count = self._last_count - self._base_count
            self._tracker.apply_anchor(cell, count)
            self._position = self._tracker.update(count).as_dict()
            self._trail.append(list(cell))
        return self.state()

    def clear(self, _body: dict[str, Any] | None = None) -> dict[str, Any]:
        with self._lock:
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
                "setup": dict(self._setup),
                "tiles": self._last_count - self._base_count,
                "position": None if self._position is None else dict(self._position),
                "trail": [list(c) for c in self._trail[-400:]],
            }

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def counter_reset(self) -> None:
        """The tile counter went back to 0 (route start): keep the position."""
        with self._lock:
            self._base_count -= self._last_count
            self._last_count = 0

    def update(self, tile_count: int) -> None:
        with self._lock:
            self._last_count = int(tile_count)
            if self._tracker is None:
                return
            position = self._tracker.update(self._last_count - self._base_count)
            cell = [position.cell[0], position.cell[1]]
            if not self._trail or self._trail[-1] != cell:
                self._trail.append(cell)
            self._position = position.as_dict()

    def telemetry(self) -> dict[str, Any]:
        with self._lock:
            if self._position is None:
                return {"map_row": None, "map_column": None, "map_status": "UNSET"}
            return {
                "map_row": self._position["row"],
                "map_column": self._position["column"],
                "map_status": self._position["status"],
            }
