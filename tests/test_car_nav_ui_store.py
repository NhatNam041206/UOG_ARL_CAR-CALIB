"""Web UI profiles and test records (ros/car_nav/car_nav/ui_store.py)."""
import pytest

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.ui_store import ProfileStore, safe_name
from car_nav.ui_store import TestRecorder as Recorder
from car_nav.ui_store import TestStore as Store


def test_profile_crud_and_default(tmp_path):
    ps = ProfileStore(str(tmp_path))
    ps.put("corridor 0.8m", {"guard": {"danger_range_m": 0.8}, "drive": {"speed_mps": 0.3}, "note": "lab"})
    assert ps.get("corridor 0.8m")["guard"]["danger_range_m"] == 0.8
    ps.put("corridor 0.8m", {"guard": {"danger_range_m": 0.9}})            # update
    assert ps.get("corridor 0.8m")["guard"]["danger_range_m"] == 0.9
    ps.set_default("corridor 0.8m")
    assert [p["default"] for p in ps.list()] == [True]
    assert ps.delete("corridor 0.8m") and ps.get("corridor 0.8m") is None and ps.default() is None
    assert not ps.delete("nope")


def test_names_cannot_escape_the_folder():
    assert safe_name("../../etc/passwd") == "etcpasswd"
    with pytest.raises(ValueError):
        safe_name("///")


def test_test_record_summary_and_store(tmp_path):
    r = Recorder(name="person crossing", params={"guard": {"danger_range_m": 0.6}})
    r.sample([0.0, 0.0, 0.0], "clear", None, [0.4, 0.4], 0)
    r.event("guard", to="stop", nearest=0.5)
    r.sample([1.0, 0.0, 0.0], "stop", 0.5, [0.0, 0.0], 12)
    r.event("goal", x=3.0, y=0.0)
    s = r.summary()
    assert s["distance_m"] == 1.0 and s["guard_stops"] == 1 and s["min_person_distance_m"] == 0.5
    assert s["max_cmd_speed_mps"] == 0.4 and s["goals"] == 1
    ts = Store(str(tmp_path))
    tid = ts.save(r)
    assert ts.list()[0]["id"] == tid
    ts.update(tid, {"note": "stopped 0.5 m before", "verdict": "ok"})
    assert ts.get(tid)["verdict"] == "ok" and ts.get(tid)["samples"][1]["n_dyn"] == 12
    assert ts.delete(tid) and ts.list() == []
