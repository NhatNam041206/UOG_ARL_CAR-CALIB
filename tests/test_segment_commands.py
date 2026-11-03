"""A* segment commands: route presets replace planner steps per segment kind."""

from __future__ import annotations

import pytest

from positioning.route_steps import apply_segment_commands, check_straight_command, segment_kind

PLAN = [
    {"action": "forward", "tiles": 3},
    {"action": "turn_left", "degrees": 90.0, "anchor": "next_line"},
    {"action": "settle", "duration_s": 2.5},
    {"action": "forward", "tiles": 2},
    {"action": "turn_right", "duration_s": 2.0},
    {"action": "forward", "tiles": 1},
]


def test_segment_kinds():
    assert [segment_kind(s) for s in PLAN] == [
        "straight", "turn_left", None, "straight", "turn_right", "straight",
    ]


def test_no_commands_keeps_the_plan():
    assert apply_segment_commands(PLAN, {}) == (PLAN, 0)


def test_straight_command_takes_each_segment_tile_count():
    straight = [{"action": "straight", "tiles": 1}, {"action": "pause", "duration_s": 0.2}]
    steps, n = apply_segment_commands(PLAN, {"straight": straight})
    assert n == 3
    assert [s.get("tiles") for s in steps if s["action"] == "straight"] == [3, 2, 1]
    assert straight[0]["tiles"] == 1  # the preset itself is not changed


def test_turn_command_replaces_the_turn_and_its_settle():
    left = [{"action": "left", "degrees": 90.0, "anchor": "next_line"}, {"action": "settle", "duration_s": 3.0}]
    steps, n = apply_segment_commands(PLAN, {"turn_left": left})
    assert n == 1
    assert steps[1:3] == left
    assert steps[3] == {"action": "forward", "tiles": 2}
    # The right turn has no command: planner step kept.
    assert {"action": "turn_right", "duration_s": 2.0} in steps


def test_straight_command_needs_one_tiles_step():
    check_straight_command([{"action": "forward", "tiles": 1}])
    with pytest.raises(ValueError):
        check_straight_command([{"action": "forward", "duration_s": 1.0}])


def test_preview_labels_mark_inserted_steps():
    left = [{"action": "left", "degrees": 90.0}]
    steps, _ = apply_segment_commands(PLAN, {"turn_left": left}, labels={"turn_left": "turn_left: soft"})
    assert steps[1] == {"action": "left", "degrees": 90.0, "_cmd": "turn_left: soft"}
    assert "_cmd" not in steps[0]
