"""Place the car on the campus tile map from the cross-line tile count.

Grid convention (same as ARL_AUTOCAR ``campus-path``): a cell is
``(row, column)``; row grows "south" (down on the rendered map) and column
grows "east" (right).

Two ways to turn a tile count N into a cell:

* Straight: start cell + heading; cell = start + N steps along the heading.
* Path: a list of cells (for example the shortest path from start to goal);
  cell = path[N]. The counter counts seams crossing the car's own direction of
  travel, so the index keeps advancing through 90-degree turns (a corner can be
  off by one tile).

A predicted cell that is not runnable is reported as CONTRADICTORY instead of
being snapped to the nearest valid cell. An anchor (room sign) passage moves
the position to the anchor cell; later counts continue from there.
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

Cell = tuple[int, int]

HEADINGS: dict[str, Cell] = {
    "north": (-1, 0),
    "south": (1, 0),
    "east": (0, 1),
    "west": (0, -1),
}


@dataclass(frozen=True)
class Anchor:
    anchor_id: str
    label: str
    cell: Cell


@dataclass(frozen=True)
class TileMap:
    map_id: str
    tile_size_m: float
    cells: frozenset[Cell]
    anchors: dict[str, Anchor]
    bounds: dict[str, int]

    @classmethod
    def load(cls, path: str | Path) -> "TileMap":
        """Load a campus-path map from JSON, or YAML when PyYAML is installed."""
        path = Path(path)
        text = path.read_text(encoding="utf-8")
        if path.suffix.lower() in (".yaml", ".yml"):
            import yaml  # optional; the JSON copy needs no extra dependency

            data = yaml.safe_load(text)
        else:
            data = json.loads(text)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TileMap":
        cells = frozenset((int(r), int(c)) for r, c in data["runnable_cells"])
        if not cells:
            raise ValueError("map has no runnable cells")
        anchors = {
            str(a["id"]): Anchor(str(a["id"]), str(a.get("label", "")), (int(a["cell"][0]), int(a["cell"][1])))
            for a in data.get("visual_anchors", [])
        }
        bounds = data.get("bounds") or {
            "minimum_row": min(r for r, _ in cells),
            "maximum_row": max(r for r, _ in cells),
            "minimum_column": min(c for _, c in cells),
            "maximum_column": max(c for _, c in cells),
        }
        return cls(
            map_id=str(data.get("map_id", "")),
            tile_size_m=float(data["tile_size_m"]),
            cells=cells,
            anchors=anchors,
            bounds={k: int(v) for k, v in bounds.items() if k.endswith(("_row", "_column"))},
        )

    def is_runnable(self, cell: Cell) -> bool:
        return cell in self.cells

    def resolve_cell(self, ref: str) -> Cell:
        """Return the cell for an anchor id or a "row,column" string."""
        if ref in self.anchors:
            return self.anchors[ref].cell
        try:
            row, col = (int(part) for part in ref.split(","))
        except ValueError as exc:
            raise ValueError(f"not an anchor id or row,column: {ref!r}") from exc
        return (row, col)

    def shortest_path(self, start: Cell, goal: Cell) -> list[Cell]:
        """4-connected shortest path over runnable cells (start and goal included)."""
        for cell in (start, goal):
            if not self.is_runnable(cell):
                raise ValueError(f"cell {cell} is not runnable")
        previous: dict[Cell, Cell | None] = {start: None}
        queue: deque[Cell] = deque([start])
        while queue:
            cell = queue.popleft()
            if cell == goal:
                break
            for dr, dc in HEADINGS.values():
                nxt = (cell[0] + dr, cell[1] + dc)
                if nxt in self.cells and nxt not in previous:
                    previous[nxt] = cell
                    queue.append(nxt)
        if goal not in previous:
            raise ValueError(f"no path from {start} to {goal}")
        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])  # type: ignore[arg-type]
        return path[::-1]


@dataclass(frozen=True)
class TilePosition:
    tile_count: int
    index: int  # steps from the start (or from the last anchor's path index)
    cell: Cell
    status: str  # OK, CONTRADICTORY (cell not runnable), PAST_END (path mode)
    heading: str | None = None  # heading mode: current heading after turns

    def as_dict(self) -> dict[str, Any]:
        return {
            "tile_count": self.tile_count,
            "index": self.index,
            "row": self.cell[0],
            "column": self.cell[1],
            "status": self.status,
            "heading": self.heading,
        }


class TilePositionTracker:
    """Map a running tile count onto the tile map."""

    def __init__(
        self,
        tile_map: TileMap,
        start: Cell,
        *,
        heading: str | None = None,
        path: list[Cell] | None = None,
    ) -> None:
        if (heading is None) == (path is None):
            raise ValueError("give exactly one of heading or path")
        if heading is not None and heading not in HEADINGS:
            raise ValueError(f"heading must be one of {sorted(HEADINGS)}")
        if path is not None and (not path or path[0] != start):
            raise ValueError("path must start at the start cell")
        self.map = tile_map
        self.start = start
        self.heading = heading
        self.path = path
        # index = tile_count + offset; anchors change the offset.
        self._offset = 0
        self._origin = start

    def update(self, tile_count: int) -> TilePosition:
        index = tile_count + self._offset
        if self.path is not None:
            if index >= len(self.path):
                return TilePosition(tile_count, index, self.path[-1], "PAST_END", self.heading)
            cell = self.path[index]
        else:
            dr, dc = HEADINGS[self.heading]  # type: ignore[index]
            cell = (self._origin[0] + dr * index, self._origin[1] + dc * index)
        status = "OK" if self.map.is_runnable(cell) else "CONTRADICTORY"
        return TilePosition(tile_count, index, cell, status, self.heading)

    def direction(self, tile_count: int) -> Cell:
        """Direction of travel (dr, dc) at this tile count."""
        if self.path is None:
            return HEADINGS[self.heading]  # type: ignore[index]
        if len(self.path) < 2:
            return (0, 0)
        # The step out of the current cell, or into it at the end of the path.
        index = max(0, min(tile_count + self._offset, len(self.path) - 2))
        a, b = self.path[index], self.path[index + 1]
        return (b[0] - a[0], b[1] - a[1])

    def apply_turn(self, quarter_turns: int, tile_count: int) -> None:
        """The car turned quarter_turns x 90 degrees (left positive) at its
        current cell. Heading mode: later tiles go along the new heading from
        here. Path mode: the path already holds the corner, nothing changes."""
        if self.path is not None or quarter_turns % 4 == 0:
            return
        here = self.update(tile_count).cell
        dr, dc = HEADINGS[self.heading]  # type: ignore[index]
        for _ in range(quarter_turns % 4):
            dr, dc = -dc, dr  # left: north -> west -> south -> east
        self.heading = next(name for name, d in HEADINGS.items() if d == (dr, dc))
        self._origin = here
        self._offset = -tile_count

    def apply_anchor(self, anchor_cell: Cell, tile_count: int) -> None:
        """The car is at anchor_cell now (tile_count seams counted so far)."""
        if self.path is not None:
            if anchor_cell not in self.path:
                raise ValueError(f"anchor cell {anchor_cell} is not on the path")
            self._offset = self.path.index(anchor_cell) - tile_count
        else:
            self._origin = anchor_cell
            self._offset = -tile_count
