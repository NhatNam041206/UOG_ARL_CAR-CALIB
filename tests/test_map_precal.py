"""Tests for the dashboard Map tab session and the Pre-calibrate checklist."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from positioning.map_session import MapSession
from positioning.tile_position import TileMap
from runtime.calib_tuning import TuneBlockedError
from runtime.pi5_http import Pi5HttpServer
from runtime.precalibrate import PreCalibrator

CAMPUS_MAP = Path(__file__).resolve().parents[1] / "maps" / "tile_maps" / "campus-path.json"


def _session() -> MapSession:
    return MapSession(TileMap.load(CAMPUS_MAP))


def test_map_session_follows_count_and_route_start_returns_to_saved_start(tmp_path):
    start_file = tmp_path / "start_position.json"
    session = MapSession(TileMap.load(CAMPUS_MAP), start_file)
    assert session.telemetry()["map_status"] == "UNSET"
    session.update(7)  # counter already running before the start is set
    state = session.configure({"start": "AAD-1", "goal": "gym-01"})
    assert state["saved_start"] is True and start_file.is_file()
    assert state["setup"]["start"] == [-31, 5]
    assert len(state["setup"]["path"]) == 17
    assert state["tiles"] == 0

    session.update(10)
    assert session.telemetry() == {"map_row": -31, "map_column": 8, "map_status": "OK"}

    session.route_start()  # counter back to 0, car back on the saved start
    session.update(0)
    assert session.telemetry()["map_column"] == 5
    session.update(16)
    assert session.telemetry()["map_column"] == 21
    session.update(17)
    assert session.telemetry()["map_status"] == "PAST_END"
    assert session.start_summary() == {"map_id": "campus-path", "start": [-31, 5], "heading": None, "goal": [-31, 21]}

    # survives a restart
    reloaded = MapSession(TileMap.load(CAMPUS_MAP), start_file)
    assert reloaded.state()["setup"]["start"] == [-31, 5]
    reloaded.clear()
    assert not start_file.exists()
    reloaded.route_start()
    assert reloaded.telemetry()["map_status"] == "UNSET"


def test_map_session_anchor_and_validation():
    session = _session()
    with pytest.raises(ValueError):
        session.configure({"start": "AAD-1"})
    with pytest.raises(ValueError):
        session.mark_anchor({"ref": "AAD-1"})
    session.configure({"start": "AAD-1", "heading": "east"})
    session.update(5)  # overcounted: the car is really at 3 tiles
    state = session.mark_anchor({"ref": "-31,8"})
    assert state["position"]["column"] == 8
    session.update(6)
    assert session.telemetry()["map_column"] == 9
    assert session.map_payload()["anchors"]
    assert session.clear()["configured"] is False


def test_map_session_without_map():
    session = MapSession(None)
    assert session.state()["available"] is False
    with pytest.raises(ValueError):
        session.map_payload()


def _telemetry(**kw):
    base = {
        "frame_width": 640,
        "selected_pair_found": 1,
        "vp_x": 320.0,
        "vp_y": -320.0,
        "vp_angle": 90.0,
        "lane_offset_ratio": 0.0,
        "lane_width_px": 540.0,
    }
    base.update(kw)
    return base


def _feed(precal: PreCalibrator, n: int = 30, frame=None, cross=2, row=-320, **kw):
    frame = np.full((480, 640, 3), 120, np.uint8) if frame is None else frame
    for i in range(n):
        precal.observe(frame, _telemetry(**kw), cross, row, now=1000.0 + i / 30.0)


def _check(status, key):
    return next(c for c in status["checks"] if c["key"] == key)


def test_precal_all_green_when_centered():
    precal = PreCalibrator()
    _feed(precal)
    st = precal.status()
    assert st["ready"], st["checks"]
    assert {c["key"] for c in st["checks"]} == {"light", "fps", "lane", "center", "vp_x", "vp_row", "cross"}


def test_precal_flags_offset_vp_dark_and_no_cross():
    precal = PreCalibrator()
    _feed(precal, frame=np.full((480, 640, 3), 20, np.uint8), cross=0,
          lane_offset_ratio=0.2, vp_x=360.0, vp_y=-200.0)
    st = precal.status()
    assert not st["ready"]
    assert not _check(st, "light")["ok"]
    assert not _check(st, "cross")["ok"]
    center = _check(st, "center")
    assert not center["ok"] and "phải" in center["hint"] and "12 cm" in center["hint"]
    assert not _check(st, "vp_x")["ok"]
    assert not _check(st, "vp_row")["ok"]


class _FakeTune:
    def __init__(self, blocked: bool = False):
        self.applied = []
        self.blocked = blocked

    def apply(self, values):
        if self.blocked:
            raise TuneBlockedError("route running")
        self.applied.append(dict(values))
        return {}


def test_precal_capture_writes_measured_vp():
    precal = PreCalibrator()
    _feed(precal, vp_angle=93.27, vp_y=-301.6)
    tune = _FakeTune()
    out = precal.capture(tune, {"what": ["vp_center_deg"]})
    assert out["applied"] == {"vp_center_deg": 93.3}
    precal.capture(tune, {})
    assert tune.applied[-1] == {"vp_center_deg": 93.3, "vp_horizon_y": -302}

    empty = PreCalibrator()
    _feed(empty, vp_angle=None, vp_y=None, vp_x=None, selected_pair_found=0)
    with pytest.raises(ValueError):
        empty.capture(tune, {})


def _request(port, path, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=data,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=2) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        raw = exc.read()
        try:
            return exc.code, json.loads(raw)
        except json.JSONDecodeError:
            return exc.code, raw.decode()


def test_extra_routes_dispatch_and_map_errors():
    server = Pi5HttpServer(host="127.0.0.1", port=0, token="")
    session = _session()
    precal = PreCalibrator()
    _feed(precal)
    server.add_route("GET", "/api/map/state", lambda _q: session.state())
    server.add_route("POST", "/api/map/start", session.configure)
    server.add_route("POST", "/api/precal/capture", lambda body: precal.capture(_FakeTune(blocked=True), body))
    try:
        server.start()
        deadline = time.monotonic() + 2.0
        while server._server is None and time.monotonic() < deadline:
            time.sleep(0.01)
        port = server._server.server_address[1]
        assert _request(port, "/api/map/state")[1]["configured"] is False
        code, body = _request(port, "/api/map/start", {"start": "AAD-1", "heading": "east"})
        assert code == 200 and body["configured"] is True
        assert _request(port, "/api/map/start", {"start": "nowhere", "heading": "east"})[0] == 400
        assert _request(port, "/api/precal/capture", {})[0] == 409
        assert _request(port, "/api/unknown")[0] == 404
    finally:
        server.stop()
        if server._server is not None:
            server._server.server_close()


# --- map store (ARL_AUTOCAR survey-tool rules) ------------------------------
from positioning.map_store import MapStore, plan_route, validate_map_data  # noqa: E402


def _small_map(**kw):
    data = {
        "schema_version": 2,
        "map_id": "small",
        "tile_size_m": 0.6,
        "robot": {"width_m": 0.3, "length_m": 0.35, "safety_margin_m": 0.05},
        "runnable_cells": [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]],
        "visual_anchors": [{"id": "door", "label": "Door", "cell": [2, 2]}],
    }
    data.update(kw)
    return data


def test_validate_map_rules():
    norm = validate_map_data(_small_map())
    assert norm["bounds"]["physical_width_m"] == pytest.approx(1.8)
    for bad in (
        _small_map(map_id="Bad Name"),
        _small_map(tile_size_m=0),
        _small_map(runnable_cells=[]),
        _small_map(visual_anchors=[{"id": "x", "cell": [5, 5]}]),
        _small_map(visual_anchors=[{"id": "x", "cell": [0, 0]}, {"id": "x", "cell": [0, 1]}]),
    ):
        with pytest.raises(ValueError):
            validate_map_data(bad)


def test_plan_route_distance_and_turns():
    out = plan_route(_small_map(), [0, 0], [2, 2])
    assert out["cells"][-1] == [2, 2]
    assert out["distance_m"] == pytest.approx(2.4)
    assert out["turn_count"] == 1
    with pytest.raises(ValueError, match="goal_not_runnable"):
        plan_route(_small_map(), [0, 0], [9, 9])


def test_map_store_save_load_delete(tmp_path):
    store = MapStore(saved_dir=tmp_path)
    ids = {m["id"]: m["source"] for m in store.list_maps()}
    assert ids["campus-path"] == "bundled"
    store.save(_small_map())
    assert {"id": "small", "source": "saved"} in store.list_maps()
    loaded = store.load("small")
    assert loaded["visual_anchors"][0]["id"] == "door"
    with pytest.raises(ValueError):
        store.delete("campus-path")  # bundled maps are read-only
    store.delete("small")
    with pytest.raises(ValueError):
        store.load("small")


def test_map_session_uses_editor_map():
    session = MapSession(None)
    state = session.configure({"map": _small_map(), "start": [0, 0], "goal": [2, 2]})
    assert state["setup"]["map_id"] == "small"
    session.update(3)
    assert session.telemetry() == {"map_row": 1, "map_column": 2, "map_status": "OK"}
    session.mark_anchor({"ref": [2, 2]})
    assert session.telemetry()["map_row"] == 2


def test_anchor_off_route_replans_to_goal():
    session = _session()
    session.configure({"start": "AAD-1", "goal": "gym-01"})
    session.update(2)
    off_route = [-30, 10]  # a runnable cell next to the AAD-1 -> gym-01 row
    assert session._map.is_runnable(tuple(off_route))
    state = session.mark_anchor({"ref": off_route})
    assert state["position"]["row"] == -30 and state["position"]["column"] == 10
    assert state["setup"]["path"][0] == off_route and state["setup"]["path"][-1] == [-31, 21]
    session.update(3)
    assert session.telemetry()["map_status"] == "OK"
    with pytest.raises(ValueError):
        session.mark_anchor({"ref": [100, 100]})
