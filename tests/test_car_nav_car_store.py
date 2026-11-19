"""Car profiles and car.yaml writing (ros/car_nav/car_nav/car_store.py)."""
import os

import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav import config_loader
from car_nav.car_store import (CAR_FIELDS, ActiveCar, CarProfileStore, derived, patch_yaml_text, props_from_cfg,
                               validate_props)


def shipped_props():
    return props_from_cfg(yaml.safe_load(P.CAR_YAML.read_text(encoding="utf-8")))


def test_props_from_shipped_file():
    p = shipped_props()
    assert p["x_front"] == 0.35 and p["x_rear"] == -0.35 and p["half_width"] == 0.30
    assert p["laser_z"] == 0.24 and p["camera_pitch_deg"] == 15.0
    assert p["cruise_speed_mps"] == 0.4 and p["max_wheel_speed_mps"] == 0.6
    assert p["track_width_m"] is None and p["height_m"] is None
    assert set(p) == set(CAR_FIELDS)
    validate_props({k: v for k, v in p.items()})           # the shipped values are valid


def test_validate_rejects_bad_values():
    p = shipped_props()
    for key, bad in (("half_width", 9.0), ("laser_z", -1), ("x_rear", 0.2), ("camera_pitch_deg", 120),
                     ("cruise_speed_mps", float("nan")), ("laser_x", "abc")):
        with pytest.raises(ValueError):
            validate_props({**p, key: bad})
    with pytest.raises(ValueError):
        validate_props({**p, "x_front": -0.5})              # front behind the rear edge
    with pytest.raises(ValueError):
        validate_props({**p, "cruise_speed_mps": 0.9, "max_wheel_speed_mps": 0.5})
    with pytest.raises(ValueError):
        validate_props({**p, "nonsense": 1})
    with pytest.raises(ValueError):
        validate_props({k: v for k, v in p.items() if k != "laser_z"})
    assert validate_props({"laser_z": "0.3"}, partial=True) == {"laser_z": 0.3}
    assert validate_props({**p, "track_width_m": ""})["track_width_m"] is None
    with pytest.raises(ValueError):
        validate_props({**p, "laser_z": None})               # not nullable


def test_patch_keeps_comments_and_untouched_lines():
    text = P.CAR_YAML.read_text(encoding="utf-8")
    new = patch_yaml_text(text, {("laser", "z"): 0.31, ("laser", "yaw"): 1.5708, ("footprint", "x_front"): 0.4})
    cfg = yaml.safe_load(new)
    assert cfg["laser"]["z"] == 0.31 and cfg["laser"]["yaw"] == 1.5708 and cfg["footprint"]["x_front"] == 0.4
    assert cfg["laser"]["x"] == 0.0 and cfg["camera"]["z"] == 1.2 and cfg["drive"]["cruise_speed_mps"] == 0.4
    # every comment survives; the inline comment of laser.z too
    assert new.count("#") == text.count("#")
    assert "z: 0.31" in new and "# height above the floor" in new
    # only the three lines changed
    changed = [(a, b) for a, b in zip(text.split("\n"), new.split("\n")) if a != b]
    assert len(changed) == 3 and len(text.split("\n")) == len(new.split("\n"))


def test_patch_adds_missing_key_and_section():
    text = "footprint:\n  x_front: 0.35\n  half_width: 0.3\n\nlaser:\n  z: 0.2   # c\n"
    new = patch_yaml_text(text, {("footprint", "height_m"): 0.5, ("camera", "z"): 1.0})
    cfg = yaml.safe_load(new)
    assert cfg["footprint"] == {"x_front": 0.35, "half_width": 0.3, "height_m": 0.5}
    assert cfg["camera"] == {"z": 1.0} and cfg["laser"]["z"] == 0.2


def test_active_car_write_backup_and_reset(tmp_path):
    shipped = tmp_path / "shipped.yaml"
    shipped.write_text(P.CAR_YAML.read_text(encoding="utf-8"), encoding="utf-8")
    ov = tmp_path / "data" / "car.yaml"
    car = ActiveCar(str(shipped), str(ov))
    assert car.path() == str(shipped) and not car.is_override()
    props = car.read_props()
    props.update(laser_x=0.1, laser_yaw=3.1416, track_width_m=0.5, height_m=0.4)
    r = car.write_props(props, now=1_800_000_000)
    assert car.is_override() and car.path() == str(ov) and os.path.isfile(r["backup"])
    assert shipped.read_text(encoding="utf-8") == P.CAR_YAML.read_text(encoding="utf-8")   # shipped untouched
    cfg = yaml.safe_load(ov.read_text(encoding="utf-8"))
    assert cfg["laser"]["x"] == 0.1 and cfg["laser"]["yaw"] == 3.1416
    assert cfg["drive"]["track_width_m"] == 0.5 and cfg["footprint"]["height_m"] == 0.4
    assert "# height above the floor" in ov.read_text(encoding="utf-8")               # comments kept
    # second write: backup of the override, old backups pruned to `keep`
    car.keep = 2
    for i in range(4):
        props["laser_x"] = 0.2 + i / 100
        car.write_props(props, now=1_800_000_100 + i * 10)
    assert len(car.backups()) == 2
    assert car.read_props()["laser_x"] == pytest.approx(0.23)
    # config_loader sees the override through the env variable
    os.environ[config_loader.OVERRIDE_ENV] = str(ov)
    try:
        assert config_loader.load()["laser"]["x"] == pytest.approx(0.23)
        assert config_loader.load()["_path"] == str(ov)
    finally:
        del os.environ[config_loader.OVERRIDE_ENV]
    assert car.reset(now=1_800_000_500) and not car.is_override() and car.path() == str(shipped)
    assert car.reset() is None


def test_write_refused_without_override_and_with_bad_props(tmp_path):
    shipped = tmp_path / "s.yaml"
    shipped.write_text(P.CAR_YAML.read_text(encoding="utf-8"), encoding="utf-8")
    with pytest.raises(RuntimeError):
        ActiveCar(str(shipped), "").write_props(shipped_props())
    car = ActiveCar(str(shipped), str(tmp_path / "o.yaml"))
    with pytest.raises(ValueError):
        car.write_props({**shipped_props(), "laser_z": 99})
    assert not (tmp_path / "o.yaml").exists() and car.backups() == []


def test_profile_store_crud(tmp_path):
    st = CarProfileStore(str(tmp_path))
    p = shipped_props()
    st.put("small car", p, "first")
    assert st.get("small car")["props"]["x_front"] == 0.35
    with pytest.raises(FileExistsError):
        st.put("small car", p, overwrite=False)
    st.put("small car", {**p, "laser_x": 0.2})                                     # save over
    assert st.get("small car")["props"]["laser_x"] == 0.2
    st.set_applied("small car")
    assert [(c["name"], c["applied"]) for c in st.list()] == [("small car", True)]
    d = st.rename("small car", "big car")
    assert d["name"] == "big car" and st.get("small car") is None and st.applied() == "big car"
    st.put("other", p)
    with pytest.raises(FileExistsError):
        st.rename("other", "big car")
    with pytest.raises(LookupError):
        st.rename("nope", "x")
    with pytest.raises(ValueError):
        st.put("bad", {**p, "half_width": 5})
    assert st.delete("big car") and st.applied() is None and not st.delete("big car")
    with pytest.raises(ValueError):
        st.put("", p)


def test_derived_numbers_use_blind_distance():
    p = shipped_props()
    d = derived(p)
    assert d["length_m"] == 0.7 and d["width_m"] == 0.6 and d["spin_radius_m"] == pytest.approx(0.461, abs=1e-3)
    # camera 1.2 m high, 15 deg down, vfov 45: lowest ray 37.5 deg -> floor (5 cm) 1.15/tan(37.5)
    assert d["camera_blind_m"] == pytest.approx(1.15 / 0.76733, abs=2e-3)
    assert d["camera_lowest_ray_deg"] == 37.5 and d["camera_highest_ray_deg"] == -7.5
    assert d["lidar_upside_down"] is False
    d2 = derived({**p, "laser_roll": 3.14159})
    assert d2["lidar_upside_down"] is True


def test_apply_blocker_409_rule():
    from car_nav.car_store import apply_blocker
    ok = dict(moving=False, goal_active=False, switching=False, mode="load", map_in_progress=False, writable=True)
    assert apply_blocker(**ok) is None
    assert "driven" in apply_blocker(**{**ok, "moving": True})
    assert "goal" in apply_blocker(**{**ok, "goal_active": True})
    assert "restarting" in apply_blocker(**{**ok, "switching": True})
    assert "read-only" in apply_blocker(**{**ok, "writable": False})
    # Create mode with a map being built: only with force
    assert "lose the map" in apply_blocker(**{**ok, "mode": "create", "map_in_progress": True})
    assert apply_blocker(**{**ok, "mode": "create", "map_in_progress": True, "force": True}) is None
    # force never overrides a moving car or a goal
    assert apply_blocker(**{**ok, "moving": True, "force": True}) is not None
    assert apply_blocker(**{**ok, "goal_active": True, "force": True}) is not None


def test_car_tab_form_has_every_property():
    """The page's form must offer exactly the properties the server validates (a missing one made Save fail)."""
    import re
    html = (P.ROOT / "ros" / "car_nav" / "web" / "index.html").read_text(encoding="utf-8")
    block = html[html.index("const CAR_GROUPS"):html.index("const VFOV")]
    keys = set(re.findall(r'\["([a-z_]+)", "', block))
    assert keys == set(CAR_FIELDS)
