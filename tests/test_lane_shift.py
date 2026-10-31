"""Lane shift detector and the map's sideways move."""

from __future__ import annotations

from positioning.map_session import MapSession
from positioning.tile_position import TileMap
from vision.lane_shift import LaneShiftDetector


def _feed(det: LaneShiftDetector, offsets, t0: float = 0.0, dt: float = 0.1, **kw):
    events = []
    for i, off in enumerate(offsets):
        e = det.update(t0 + i * dt, off, **kw)
        if e is not None:
            events.append(e)
    return events


def test_drift_across_the_left_line_is_a_passive_left_shift():
    events = _feed(LaneShiftDetector(), [0.1, 0.2, 0.3, 0.4, 0.45, 0.48, -0.47, -0.44, -0.42, -0.4])
    assert len(events) == 1
    assert events[0].side == 1 and events[0].active is False


def test_right_shift_and_flicker_and_turns():
    assert _feed(LaneShiftDetector(), [-0.3, -0.4, -0.45, 0.45, 0.4, 0.38])[0].side == -1
    # A tracker flipping between the two lines of one lane is not a shift.
    assert _feed(LaneShiftDetector(), [0.4, -0.4, 0.4, -0.4, 0.4, -0.4, 0.4, -0.4]) == []
    # A big heading change across the jump is a turn.
    det = LaneShiftDetector()
    assert [det.update(i * 0.1, o, psi_deg=p) for i, (o, p) in enumerate(
        [(0.4, 0.0), (0.45, 5.0), (0.48, 10.0), (-0.45, 20.0), (-0.4, 25.0), (-0.4, 30.0)])][-1] is None


def test_shift_just_after_a_commanded_turn_is_active():
    det = LaneShiftDetector()
    det.update(0.0, None, active=True)
    events = _feed(det, [0.9, 0.92, 0.92, -0.44, -0.45, -0.45], t0=1.0)
    assert len(events) == 1 and events[0].active is True
    assert det.passive_count == 0


def test_map_moves_one_row_to_the_left_of_travel():
    cells = [[r, c] for r in range(3) for c in range(5)]
    tile_map = TileMap.from_dict({"map_id": "g", "tile_size_m": 0.6, "runnable_cells": cells})
    session = MapSession(tile_map)
    session.configure({"start": [1, 0], "heading": "east"}, persist=False)
    session.update(2)
    session.apply_lane_shift(+1)  # left of east = north
    position = session.state()["position"]
    assert (position["row"], position["column"]) == (0, 2)
    session.update(3)
    assert (session.state()["position"]["row"], session.state()["position"]["column"]) == (0, 3)
    session.apply_lane_shift(+1)  # off the map: ignored
    assert session.state()["position"]["row"] == 0
