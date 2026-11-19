"""Stored maps for the web UI (ros/car_nav/car_nav/map_store.py)."""
import numpy as np
import pytest

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.map_store import MapStore, parse_start_pose, read_pgm, safe_map_name


def _grid():
    g = np.full((4, 6), -1, dtype=np.int8)
    g[0, :3] = 0          # lowest row: free
    g[1, 2] = 100         # occupied
    return g


def test_names_are_file_safe():
    assert safe_map_name("lab floor 1") == "lab_floor_1"
    assert safe_map_name("../../etc/passwd") == "etcpasswd"
    for bad in ("///", "save", "  "):
        with pytest.raises(ValueError):
            safe_map_name(bad)


def test_start_pose_parsing():
    assert parse_start_pose({"x": "1.5", "y": 2, "yaw": 7.0})["yaw"] == pytest.approx(0.7168, abs=1e-3)
    assert parse_start_pose(None) is None and parse_start_pose({}) is None
    with pytest.raises(ValueError):
        parse_start_pose({"x": 1})
    with pytest.raises(ValueError):
        parse_start_pose({"x": float("nan"), "y": 0})


def test_save_list_update_rename_delete(tmp_path):
    st = MapStore(str(tmp_path))
    m = st.save_grid("lab", _grid(), 0.05, (-1.0, -2.0), note="first", start_pose={"x": 0, "y": 0, "yaw": 0})
    assert m["width"] == 6 and m["height"] == 4 and m["start_pose"]["x"] == 0
    img = read_pgm(st.yaml_path("lab").replace(".yaml", ".pgm"))
    assert img.shape == (4, 6)
    assert img[3, 0] == 254 and img[2, 2] == 0 and img[0, 5] == 205     # row 0 of the grid is the image's bottom
    with pytest.raises(ValueError):
        st.save_grid("lab", _grid(), 0.05, (0, 0))                        # no silent overwrite
    st.save_grid("lab", _grid(), 0.05, (0, 0), overwrite=True)
    assert st.get("lab")["note"] == "first"                               # note kept on overwrite
    st.update("lab", note="second", start_pose={"x": 1, "y": 2, "yaw": 0.5})
    assert st.get("lab")["note"] == "second" and st.get("lab")["start_pose"]["y"] == 2
    st.update("lab", start_pose=None)
    assert st.get("lab")["start_pose"] is None and st.get("lab")["note"] == "second"
    st.rename("lab", "lab two")
    assert [x["name"] for x in st.list()] == ["lab_two"] and st.get("lab") is None
    assert open(st.yaml_path("lab_two"), encoding="utf-8").read().count("lab_two.pgm") == 1
    st.save_grid("other", _grid(), 0.05, (0, 0))
    with pytest.raises(ValueError):
        st.rename("other", "lab_two")
    assert st.delete("lab_two") and not st.delete("lab_two") and st.yaml_path("lab_two") is None


def test_import_and_seed_once(tmp_path):
    src = MapStore(str(tmp_path / "a"))
    src.save_grid("w", _grid(), 0.05, (0, 0))
    dst = MapStore(str(tmp_path / "b"))
    assert dst.seed(src.yaml_path("w"), "floor", start_pose={"x": 1, "y": 1, "yaw": 0})
    assert dst.get("floor")["start_pose"]["x"] == 1 and dst.get("floor")["source"] == "sim world"
    assert not dst.seed(src.yaml_path("w"), "floor")
    dst.delete("floor")
    assert not dst.seed(src.yaml_path("w"), "floor") and not dst.exists("floor")   # stays deleted


def test_map_without_json_is_listed(tmp_path):
    st = MapStore(str(tmp_path))
    st.save_grid("a", _grid(), 0.05, (0, 0))
    import os
    os.remove(os.path.join(st.dir, "a.json"))
    assert st.list()[0]["name"] == "a" and st.list()[0]["start_pose"] is None
