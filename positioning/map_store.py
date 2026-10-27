"""Tile-map storage and route preview for the dashboard Map tab.

Follows the ARL_AUTOCAR map survey tool (``autobot.app.map_survey``): the same
schema-2 map data (``map_id``, ``tile_size_m``, ``robot``, ``runnable_cells``,
``visual_anchors``, derived ``bounds``), the same validation rules, and the
same route preview (shortest four-direction route, distance and turn count).

Maps are saved as JSON, which is also valid YAML, so a saved file loads in
ARL_AUTOCAR unchanged (rename it to ``.yaml`` for its survey tool). Bundled
maps in the repo (``routes/tile_maps``) are read-only; maps saved from the
dashboard go to a writable directory outside the release folder so they
survive redeploys.
"""

from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path
from typing import Any

from positioning.tile_position import TileMap

MAP_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
BUNDLED_DIR = Path(__file__).resolve().parents[1] / "routes" / "tile_maps"
DEFAULT_ROBOT = {"width_m": 0.30, "length_m": 0.35, "safety_margin_m": 0.05}


def default_saved_dir() -> Path:
    if os.getenv("TILE_MAP_DIR"):
        return Path(os.environ["TILE_MAP_DIR"])
    return Path(os.getenv("ROUTE_LOG_ROOT", "/data/routes")) / "tile_maps"


def _number(value: Any, name: str, *, allow_zero: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f"{name} must be positive")
    return float(value)


def _cell(value: Any, name: str) -> tuple[int, int]:
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 2
        or any(isinstance(v, bool) or not isinstance(v, int) for v in value)
    ):
        raise ValueError(f"{name} must be [row, column]")
    return int(value[0]), int(value[1])


def validate_map_data(data: Any) -> dict[str, Any]:
    """Check a map the way the survey tool does; return normalised schema-2 data."""
    if not isinstance(data, dict):
        raise ValueError("tile map must be an object")
    map_id = data.get("map_id")
    if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
        raise ValueError("map_id must use lowercase letters, digits, '-' or '_'")
    tile = _number(data.get("tile_size_m"), "tile_size_m")
    raw_robot = data.get("robot") or {}
    if not isinstance(raw_robot, dict):
        raise ValueError("robot must be an object")
    robot = {
        "width_m": _number(raw_robot.get("width_m", DEFAULT_ROBOT["width_m"]), "robot.width_m"),
        "length_m": _number(raw_robot.get("length_m", DEFAULT_ROBOT["length_m"]), "robot.length_m"),
        "safety_margin_m": _number(
            raw_robot.get("safety_margin_m", DEFAULT_ROBOT["safety_margin_m"]),
            "robot.safety_margin_m",
            allow_zero=True,
        ),
    }
    raw_cells = data.get("runnable_cells")
    if not isinstance(raw_cells, list):
        raise ValueError("runnable_cells must be a list")
    cells = sorted({_cell(v, f"runnable_cells[{i}]") for i, v in enumerate(raw_cells)})
    if not cells:
        raise ValueError("at least one runnable cell is required")
    cell_set = set(cells)
    raw_anchors = data.get("visual_anchors") or []
    if not isinstance(raw_anchors, list):
        raise ValueError("visual_anchors must be a list")
    anchors = []
    seen: set[str] = set()
    for i, raw in enumerate(raw_anchors):
        if not isinstance(raw, dict):
            raise ValueError(f"visual_anchors[{i}] must be an object")
        anchor_id = str(raw.get("id") or "").strip()
        if not anchor_id:
            raise ValueError(f"visual_anchors[{i}].id is required")
        if anchor_id in seen:
            raise ValueError("visual anchor ids must be unique")
        seen.add(anchor_id)
        cell = _cell(raw.get("cell"), f"visual_anchors[{i}].cell")
        if cell not in cell_set:
            raise ValueError(f"visual anchor {anchor_id!r} must be on a runnable cell")
        label = raw.get("label")
        label = str(label).strip() or None if label is not None else None
        anchors.append({"id": anchor_id, "label": label, "cell": list(cell)})
    rows = [r for r, _ in cells]
    cols = [c for _, c in cells]
    return {
        "schema_version": 2,
        "map_id": map_id,
        "tile_size_m": tile,
        "robot": robot,
        "bounds": {
            "minimum_row": min(rows),
            "maximum_row": max(rows),
            "minimum_column": min(cols),
            "maximum_column": max(cols),
            "physical_width_m": (max(cols) - min(cols) + 1) * tile,
            "physical_length_m": (max(rows) - min(rows) + 1) * tile,
        },
        "runnable_cells": [list(c) for c in cells],
        "visual_anchors": anchors,
    }


def turn_count(cells: list[tuple[int, int]]) -> int:
    steps = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(cells, cells[1:])]
    return sum(cur != prev for prev, cur in zip(steps, steps[1:]))


def plan_route(data: Any, start: Any, goal: Any) -> dict[str, Any]:
    """Shortest four-direction route on a (possibly unsaved) map."""
    normalised = validate_map_data(data)
    tile_map = TileMap.from_dict(normalised)
    start_cell, goal_cell = _cell(start, "start"), _cell(goal, "goal")
    if not tile_map.is_runnable(start_cell):
        raise ValueError("start_not_runnable")
    if not tile_map.is_runnable(goal_cell):
        raise ValueError("goal_not_runnable")
    try:
        cells = tile_map.shortest_path(start_cell, goal_cell)
    except ValueError as exc:
        raise ValueError("no_route_available") from exc
    return {
        "cells": [list(c) for c in cells],
        "distance_m": (len(cells) - 1) * tile_map.tile_size_m,
        "turn_count": turn_count(cells),
    }


def _read_file(path: Path) -> Any:
    text = path.read_text(encoding="utf-8")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        import yaml  # a hand-written YAML map needs PyYAML; JSON maps do not

        return yaml.safe_load(text)


class MapStore:
    """Saved maps (writable) over bundled repo maps (read-only)."""

    SUFFIXES = (".json", ".yaml", ".yml")

    def __init__(self, saved_dir: Path | None = None, bundled_dir: Path = BUNDLED_DIR) -> None:
        self.saved_dir = Path(saved_dir) if saved_dir is not None else default_saved_dir()
        self.bundled_dir = Path(bundled_dir)

    def _files(self, directory: Path) -> dict[str, Path]:
        if not directory.is_dir():
            return {}
        out: dict[str, Path] = {}
        for path in sorted(directory.iterdir()):
            if path.suffix.lower() in self.SUFFIXES and MAP_ID_PATTERN.fullmatch(path.stem):
                out.setdefault(path.stem, path)
        return out

    def list_maps(self) -> list[dict[str, Any]]:
        saved = self._files(self.saved_dir)
        bundled = self._files(self.bundled_dir)
        return [
            {"id": map_id, "source": "saved" if map_id in saved else "bundled"}
            for map_id in sorted(set(saved) | set(bundled))
        ]

    def _path(self, map_id: str) -> Path:
        if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
            raise ValueError("invalid map name")
        path = self._files(self.saved_dir).get(map_id) or self._files(self.bundled_dir).get(map_id)
        if path is None:
            raise ValueError("map not found")
        return path

    def load(self, map_id: str) -> dict[str, Any]:
        return validate_map_data(_read_file(self._path(map_id)))

    def save(self, data: Any) -> dict[str, Any]:
        normalised = validate_map_data(data)
        self.saved_dir.mkdir(parents=True, exist_ok=True)
        path = self.saved_dir / f"{normalised['map_id']}.json"
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(normalised, indent=1), encoding="utf-8")
        tmp.replace(path)
        return {"saved": normalised["map_id"], "file": str(path), "map": normalised}

    def delete(self, map_id: str) -> dict[str, Any]:
        if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
            raise ValueError("invalid map name")
        path = self._files(self.saved_dir).get(map_id)
        if path is None:
            raise ValueError("only maps saved from the dashboard can be deleted")
        path.unlink()
        return {"deleted": map_id}
