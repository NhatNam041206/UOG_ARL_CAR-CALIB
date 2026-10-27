"""Script modes: tile-count steps, A* path to steps, map panel in the video."""

from __future__ import annotations

import time
from pathlib import Path

import pytest

import runtime.pi5_script_runner as runner_mod
from positioning.map_session import MapSession
from positioning.map_store import plan_steps
from positioning.route_steps import path_to_steps
from positioning.tile_position import TileMap
from runtime.pi5_script_runner import Pi5ScriptRunner
from runtime.resource_limits import ScriptValidationError, validate_route_steps

CAMPUS_MAP = Path(__file__).resolve().parents[1] / "maps" / "tile_maps" / "campus-path.json"


def test_validate_tile_steps():
    steps = validate_route_steps([
        {"action": "forward", "tiles": 4},
        {"action": "turn_left", "duration_s": 1.5},
    ])
    assert steps == [{"action": "forward", "tiles": 4}, {"action": "turn_left", "duration_s": 1.5}]
    for bad in ({"action": "turn_left", "tiles": 2}, {"action": "forward", "tiles": 0},
                {"action": "forward", "tiles": 1.5}, {"action": "forward", "tiles": 10_000}):
        with pytest.raises(ScriptValidationError):
            validate_route_steps([bad])


def _wait(pred, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.02)
    return False


def _runner(count):
    cmds = []
    runner = Pi5ScriptRunner()
    runner.set_handlers(cmds.append, lambda _a: None, lambda _r: None)
    runner.set_tile_count_getter(lambda: count["n"])
    return runner, cmds


def test_tile_step_stops_after_counted_tiles():
    count = {"n": 10}
    runner, cmds = _runner(count)
    assert runner.submit([{"action": "forward", "tiles": 3}])
    assert _wait(lambda: runner.current_action() == "forward")
    count["n"] = 12
    time.sleep(0.15)
    assert runner.is_running() and runner.status()["step_tiles_done"] == 2
    count["n"] = 13
    assert _wait(lambda: not runner.is_running())
    assert cmds[0] == "FORWARD" and cmds[-1] == "STOP"
    assert runner.status()["last_error"] == ""


def test_tile_step_times_out_and_stops(monkeypatch):
    monkeypatch.setattr(runner_mod, "TILE_STEP_MAX_S_PER_TILE", 0.1)
    count = {"n": 0}
    runner, cmds = _runner(count)
    runner.submit([{"action": "forward", "tiles": 2}, {"action": "forward", "duration_s": 5}])
    assert _wait(lambda: not runner.is_running())
    assert "timed out" in runner.status()["last_error"]
    assert cmds[-1] == "STOP"


def test_tile_steps_need_counter():
    runner = Pi5ScriptRunner()
    with pytest.raises(ValueError):
        runner.submit([{"action": "forward", "tiles": 2}])


def test_path_to_steps_turns():
    path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 3)]
    assert path_to_steps(path, turn_s=1.2) == [
        {"action": "forward", "tiles": 2},
        {"action": "turn_right", "duration_s": 1.2},  # east -> south
        {"action": "forward", "tiles": 2},
        {"action": "turn_left", "duration_s": 1.2},  # south -> east
        {"action": "forward", "tiles": 1},
    ]
    with pytest.raises(ValueError):
        path_to_steps([(0, 0), (0, 1), (0, 0)])


def test_plan_steps_and_saved_route_steps_on_campus():
    data = __import__("json").loads(CAMPUS_MAP.read_text())
    plan = plan_steps(data, [-31, 5], [-31, 21])
    assert plan["steps"] == [{"action": "forward", "tiles": 16}]

    session = MapSession(TileMap.load(CAMPUS_MAP))
    with pytest.raises(ValueError):
        session.saved_route_steps()
    session.configure({"start": "AAD-1", "heading": "east"})
    with pytest.raises(ValueError):
        session.saved_route_steps()  # heading only, no goal
    session.configure({"start": "AAD-1", "goal": "gym-01"})
    out = session.saved_route_steps()
    assert out["steps"] == [{"action": "forward", "tiles": 16}] and len(out["path"]) == 17
    validate_route_steps(out["steps"])


def test_map_frozen_while_turning_and_panel():
    session = MapSession(TileMap.load(CAMPUS_MAP))
    assert session.render_panel(480, 360) is None
    session.configure({"start": "AAD-1", "heading": "east"})
    session.update(3)
    session.update(5, frozen=True)  # two seams seen while turning in place
    session.update(6)
    assert session.telemetry()["map_column"] == 9  # 5 + 3 + 1
    panel = session.render_panel(480, 360)
    assert panel.shape == (480, 360, 3)
