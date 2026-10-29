"""Tests for placing the tile count on the campus tile map."""

from __future__ import annotations

from pathlib import Path

import pytest

from positioning.tile_position import TileMap, TilePositionTracker

CAMPUS_MAP = Path(__file__).resolve().parents[1] / "maps" / "tile_maps" / "campus-path.json"


def _l_map() -> TileMap:
    # An L-shaped corridor: (0,0)..(0,4) east, then (1,4)..(3,4) south.
    cells = [[0, c] for c in range(5)] + [[r, 4] for r in range(1, 4)]
    return TileMap.from_dict({
        "map_id": "l",
        "tile_size_m": 0.6,
        "runnable_cells": cells,
        "visual_anchors": [{"id": "corner", "label": "", "cell": [0, 4]}],
    })


def test_campus_map_loads_with_anchors():
    tile_map = TileMap.load(CAMPUS_MAP)
    assert tile_map.tile_size_m == 0.6
    assert len(tile_map.cells) == 1462
    assert tile_map.anchors["AAD-1"].cell == (-31, 5)
    path = tile_map.shortest_path(tile_map.resolve_cell("AAD-1"), tile_map.resolve_cell("gym-01"))
    assert len(path) - 1 == 16


def test_straight_mode_steps_along_heading_and_flags_off_map():
    tile_map = _l_map()
    tracker = TilePositionTracker(tile_map, (0, 0), heading="east")
    assert tracker.update(3).cell == (0, 3)
    assert tracker.update(4).status == "OK"
    off = tracker.update(5)
    assert off.cell == (0, 5)
    assert off.status == "CONTRADICTORY"


def test_path_mode_follows_turns_and_reports_past_end():
    tile_map = _l_map()
    path = tile_map.shortest_path((0, 0), (3, 4))
    tracker = TilePositionTracker(tile_map, (0, 0), path=path)
    assert tracker.update(6).cell == (2, 4)
    end = tracker.update(9)
    assert end.status == "PAST_END"
    assert end.cell == (3, 4)


def test_anchor_resets_position_in_both_modes():
    tile_map = _l_map()
    path = tile_map.shortest_path((0, 0), (3, 4))
    tracker = TilePositionTracker(tile_map, (0, 0), path=path)
    tracker.apply_anchor(tile_map.resolve_cell("corner"), tile_count=2)  # counter was 2 short
    assert tracker.update(2).cell == (0, 4)
    assert tracker.update(3).cell == (1, 4)

    straight = TilePositionTracker(tile_map, (0, 0), heading="east")
    straight.apply_anchor((0, 2), tile_count=5)  # counter was 3 long
    assert straight.update(6).cell == (0, 3)


def test_invalid_inputs_are_rejected():
    tile_map = _l_map()
    with pytest.raises(ValueError):
        TilePositionTracker(tile_map, (0, 0))
    with pytest.raises(ValueError):
        tile_map.shortest_path((0, 0), (5, 5))
    with pytest.raises(ValueError):
        tile_map.resolve_cell("nowhere")
    path = tile_map.shortest_path((0, 0), (0, 2))
    tracker = TilePositionTracker(tile_map, (0, 0), path=path)
    with pytest.raises(ValueError):
        tracker.apply_anchor((3, 4), tile_count=1)


def test_path_to_steps_degree_turns_with_settle(monkeypatch):
    from positioning.route_steps import path_to_steps

    monkeypatch.setenv("TILE_TURN_DEGREES", "true")
    monkeypatch.setenv("TILE_TURN_ACTION", "steer")
    monkeypatch.setenv("TILE_SETTLE_S", "1.5")
    steps = path_to_steps([(0, 0), (0, 1), (1, 1)])
    assert steps == [
        {"action": "forward", "tiles": 1},
        {"action": "right", "degrees": 90.0},
        {"action": "settle", "duration_s": 1.5},
        {"action": "forward", "tiles": 1},
    ]
