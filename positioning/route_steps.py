"""Turn a map path into route-script steps.

A path of cells (for example the A* route from the saved start to the goal)
becomes straight segments driven by tile count and 90-degree turns in place:

    [{"action": "forward", "tiles": 5}, {"action": "turn_right", "duration_s": 1.5}, ...]

The car is assumed to start facing along the first segment (line it up in the
Pre-calibrate tab). Rows grow south and columns east, so turning from east to
south is a right turn.

Turn steps follow the environment:

* TILE_TURN_DEGREES=true: closed-loop turns, {"action": ..., "degrees": 90}
  (the runner stops when the grid heading has seen the turn), else by time.
* TILE_TURN_ACTION: "turn" for turn_left/turn_right (base states) or
  "steer" for left/right (forward on full steering lock, Ackermann arc).
* TILE_SETTLE_S > 0 adds a settle step of that length after every turn.
* TILE_TURN_ANCHOR=true (with degrees): each turn first nudges forward until
  the cross line after V sits at row y* ({"anchor": "next_line"}).
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


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    return default if raw is None else raw.strip().lower() in {"1", "true", "yes", "on"}


def _settle_s() -> float:
    try:
        return max(0.0, float(os.getenv("TILE_SETTLE_S", "0")))
    except ValueError:
        return 0.0


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
    by_degrees = _env_bool("TILE_TURN_DEGREES", False)
    steer = os.getenv("TILE_TURN_ACTION", "turn").strip().lower() == "steer"
    settle_s = _settle_s()
    anchor = by_degrees and _env_bool("TILE_TURN_ANCHOR", False)
    steps: list[dict[str, Any]] = []
    prev: Cell | None = None
    for seg in path_segments(path):
        if prev is not None:
            action = _turn(prev, seg["direction"])
            if steer:
                action = action.removeprefix("turn_")
            if by_degrees:
                steps.append({"action": action, "degrees": 90.0, **({"anchor": "next_line"} if anchor else {})})
            else:
                steps.append({"action": action, "duration_s": turn_s})
            if settle_s > 0:
                steps.append({"action": "settle", "duration_s": settle_s})
        steps.append({"action": "forward", "tiles": seg["tiles"]})
        prev = seg["direction"]
    return steps


def turn_test_steps(side: str, before_tiles: int = 2, after_tiles: int = 1) -> list[dict[str, Any]]:
    """Calibration test run: straight, one 90 deg turn at the next cross line,
    settle (where the turn error is measured), straight again.

    Always a steered turn (left/right: FORWARD + servo lock), by degrees with
    the next-line anchor: turn_left/turn_right only switch the base GPIO state
    and do not steer, so on this car they do not rotate it (run
    route-20261103T111642Z: TURN_RIGHT for 3.4 s, heading stayed at -2 deg)."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    action = side
    return [
        {"action": "forward", "tiles": int(before_tiles)},
        {"action": action, "degrees": 90.0, "anchor": "next_line"},
        {"action": "settle", "duration_s": _settle_s() or 2.5},
        {"action": "forward", "tiles": int(after_tiles)},
    ]


SEGMENT_KINDS = ("straight", "turn_left", "turn_right")


def segment_kind(step: dict[str, Any]) -> str | None:
    """Which map segment an A* step drives: straight, turn_left or turn_right."""
    action = str(step.get("action", ""))
    if step.get("tiles") is not None and action in ("forward", "straight"):
        return "straight"
    if action in ("turn_left", "left"):
        return "turn_left"
    if action in ("turn_right", "right"):
        return "turn_right"
    return None


def check_straight_command(steps: list[dict[str, Any]]) -> None:
    """A straight command must say where the segment's tile count goes."""
    if sum(1 for s in steps if s.get("tiles") is not None) != 1:
        raise ValueError("straight command needs exactly one forward step by tiles (it gets the segment's tile count)")


def apply_segment_commands(
    steps: list[dict[str, Any]],
    commands: dict[str, list[dict[str, Any]] | None],
    labels: dict[str, str] | None = None,
) -> tuple[list[dict[str, Any]], int]:
    """Replace each A* segment with its configured command steps.

    straight: the command's tiles step takes the segment's tile count.
    turn_left / turn_right: the command replaces the turn and the settle the
    planner put after it (the command brings its own settle if it needs one).
    Segments without a command keep the planner's steps. With labels (preview
    only), each inserted step carries "_cmd": labels[kind]. Returns the new
    steps and how many segments were replaced."""
    out: list[dict[str, Any]] = []
    replaced = 0
    i = 0
    while i < len(steps):
        step = steps[i]
        kind = segment_kind(step)
        command = commands.get(kind) if kind else None
        if not command:
            out.append(dict(step))
            i += 1
            continue
        replaced += 1
        tag = {"_cmd": labels[kind]} if labels and kind in labels else {}
        if kind == "straight":
            for s in command:
                s = dict(s, **tag)
                if s.get("tiles") is not None:
                    s["tiles"] = int(step["tiles"])
                out.append(s)
            i += 1
            continue
        out.extend(dict(s, **tag) for s in command)
        i += 1
        if i < len(steps) and steps[i].get("action") == "settle":
            i += 1
    return out, replaced
