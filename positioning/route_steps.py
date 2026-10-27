"""Turn a map path into route-script steps.

A path of cells (for example the A* route from the saved start to the goal)
becomes straight segments driven by tile count and 90-degree turns in place:

    [{"action": "forward", "tiles": 5}, {"action": "turn_right", "duration_s": 1.5}, ...]

The car is assumed to start facing along the first segment (line it up in the
Pre-calibrate tab). Rows grow south and columns east, so turning from east to
south is a right turn.
"""

from __future__ import annotations

import os
from typing import Any

Cell = tuple[int, int]


def turn_duration_s() -> float:
    """Time for a 90-degree turn in place (TILE_TURN_90_S, calibrate on the floor)."""
    try:
        return float(os.getenv("TILE_TURN_90_S", "1.5"))
    except ValueError:
        return 1.5


def _direction(a: Cell, b: Cell) -> Cell:
    d = (b[0] - a[0], b[1] - a[1])
    if abs(d[0]) + abs(d[1]) != 1:
        raise ValueError(f"path cells {a} and {b} are not neighbours")
    return d


def path_segments(path: list[Cell]) -> list[dict[str, Any]]:
    """Straight runs: [{"direction": (dr, dc), "tiles": n}, ...]."""
    if len(path) < 2:
        raise ValueError("path needs at least two cells")
    segments: list[dict[str, Any]] = []
    for a, b in zip(path, path[1:]):
        d = _direction(tuple(a), tuple(b))
        if segments and segments[-1]["direction"] == d:
            segments[-1]["tiles"] += 1
        else:
            segments.append({"direction": d, "tiles": 1})
    return segments


def _turn(prev: Cell, new: Cell) -> str:
    # z of (prev x new) with x = column, y = row (y down): > 0 is a right turn.
    z = prev[1] * new[0] - prev[0] * new[1]
    if z > 0:
        return "turn_right"
    if z < 0:
        return "turn_left"
    raise ValueError("path reverses direction; a U-turn is not supported")


def path_to_steps(path: list[Cell], turn_s: float | None = None) -> list[dict[str, Any]]:
    turn_s = turn_duration_s() if turn_s is None else float(turn_s)
    steps: list[dict[str, Any]] = []
    prev: Cell | None = None
    for seg in path_segments(path):
        if prev is not None:
            steps.append({"action": _turn(prev, seg["direction"]), "duration_s": turn_s})
        steps.append({"action": "forward", "tiles": seg["tiles"]})
        prev = seg["direction"]
    return steps
