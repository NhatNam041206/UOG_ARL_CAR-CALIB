"""Tests for runtime calibration tuning."""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from runtime.calib_tuning import CalibTuneManager, TuneBlockedError, TuneValidationError


def _fake_calibrator():
    vision = SimpleNamespace(
        _roi_height_pct=0.6,
        _roi_top_width_pct=0.75,
        _roi_bottom_width_pct=1.0,
        _canny_low=50,
        _canny_high=150,
        _hough_threshold=40,
        _hough_min_line_length=40,
        _hough_max_line_gap=20,
        _min_abs_slope=0.3,
        _vp_horizon_y=-518.0,
        _vp_search_px=200.0,
        _vp_row_offset=0.0,
        _lane_width_tol=0.3,
        _single_line_max_frames=30,
    )
    steering = SimpleNamespace(
        _inner_thresh=3.0,
        _outer_thresh=8.0,
        _danger_margin=130,
        _danger_offset_ratio=0.25,
        _vp_center_deg=90.0,
        _lateral_gain=60.0,
        _nudge_deg=10.0,
        _tracking_active=True,
        _last_error=12.0,
    )
    state = SimpleNamespace(
        pid=SimpleNamespace(kp=0.9, ki=0.05, kd=0.08),
        pid_last_error=9.0,
    )
    return SimpleNamespace(
        _vision=vision,
        steering_controller=steering,
        robot_state=state,
        _overlay_drawer=SimpleNamespace(_inner_thresh=3.0, _outer_thresh=8.0, _danger_margin_px=130),
        _pair_hold_frames=5,
        _camera_hfov_deg=0.0,
        _relock_valid_frames=3,
        _telemetry=SimpleNamespace(
            _inner_thresh=3.0,
            _outer_thresh=8.0,
            _danger_margin_px=130,
            _overlay_drawer=SimpleNamespace(_inner_thresh=3.0, _outer_thresh=8.0, _danger_margin_px=130),
        ),
    )


def test_tune_apply_updates_runtime_fields_and_resets_pd_state(tmp_path):
    calibrator = _fake_calibrator()
    manager = CalibTuneManager(calibrator, tmp_path / "calib_tune.json")

    status = manager.apply(
        {
            "roi_height_pct": 0.7,
            "roi_top_width_pct": 0.8,
            "roi_bottom_width_pct": 0.95,
            "canny_low": 60,
            "canny_high": 170,
            "hough_threshold": 55,
            "hough_min_line_length": 65,
            "hough_max_line_gap": 25,
            "min_abs_slope": 0.45,
            "pid_kp": 1.1,
            "pid_kd": 0.2,
            "vp_inner_thresh": 4.0,
            "vp_outer_thresh": 9.0,
            "danger_margin_px": 140,
            "danger_nudge_deg": 12.0,
        }
    )

    assert status["values"]["roi_height_pct"] == pytest.approx(0.7)
    assert calibrator._vision._canny_low == 60
    assert calibrator.steering_controller._danger_margin == 140
    assert calibrator.robot_state.pid.kp == pytest.approx(1.1)
    assert calibrator.robot_state.pid.kd == pytest.approx(0.2)
    assert calibrator.steering_controller._tracking_active is False
    assert calibrator.steering_controller._last_error == pytest.approx(0.0)
    assert calibrator.robot_state.pid_last_error == pytest.approx(0.0)
    assert calibrator._telemetry._danger_margin_px == 140


@pytest.mark.parametrize(
    "values, message",
    [
        ({"canny_low": "x"}, "canny_low must be a number"),
        ({"canny_low": 200, "canny_high": 100}, "canny_high must be >= canny_low"),
        ({"vp_inner_thresh": 10, "vp_outer_thresh": 5}, "vp_outer_thresh must be >="),
        ({"hough_threshold": 0}, "hough_threshold must be between"),
    ],
)
def test_tune_validation_rejects_bad_values(tmp_path, values, message):
    manager = CalibTuneManager(_fake_calibrator(), tmp_path / "calib_tune.json")

    with pytest.raises(TuneValidationError, match=message):
        manager.apply(values)


def test_tune_apply_blocked_when_not_idle(tmp_path):
    manager = CalibTuneManager(
        _fake_calibrator(),
        tmp_path / "calib_tune.json",
        idle_getter=lambda: (False, "blocked: base not STOP"),
    )

    with pytest.raises(TuneBlockedError, match="base not STOP"):
        manager.apply({"pid_kp": 1.2})


def test_tune_save_and_reload_saved_values(tmp_path):
    tune_file = tmp_path / "calib_tune.json"
    manager = CalibTuneManager(_fake_calibrator(), tune_file)
    manager.apply({"pid_kp": 1.25, "danger_margin_px": 150})
    manager.save()

    calibrator = _fake_calibrator()
    loaded = CalibTuneManager(calibrator, tune_file)

    assert loaded.status()["saved"]["exists"] is True
    assert calibrator.robot_state.pid.kp == pytest.approx(1.25)
    assert calibrator.steering_controller._danger_margin == 150


def test_tune_file_missing_new_params_still_loads(tmp_path):
    tune_file = tmp_path / "calib_tune.json"
    manager = CalibTuneManager(_fake_calibrator(), tune_file)
    manager.apply({"pid_kp": 1.25})
    manager.save()
    payload = json.loads(tune_file.read_text(encoding="utf-8"))
    del payload["values"]["vp_horizon_y"]  # file saved before the param existed
    tune_file.write_text(json.dumps(payload), encoding="utf-8")

    calibrator = _fake_calibrator()
    calibrator._vision._vp_horizon_y = -600.0
    loaded = CalibTuneManager(calibrator, tune_file)

    assert loaded.status()["saved"]["exists"] is True
    assert calibrator.robot_state.pid.kp == pytest.approx(1.25)
    assert calibrator._vision._vp_horizon_y == pytest.approx(-600.0)


def test_tune_apply_updates_vp_horizon(tmp_path):
    calibrator = _fake_calibrator()
    manager = CalibTuneManager(calibrator, tmp_path / "calib_tune.json")

    status = manager.apply({"vp_horizon_y": -450})

    assert status["values"]["vp_horizon_y"] == -450
    assert calibrator._vision._vp_horizon_y == pytest.approx(-450.0)


def test_turning_groups_apply_to_runner_and_grid_config(tmp_path):
    from runtime.calib_tuning import CalibTuneManager, turning_bindings
    from runtime.pi5_script_runner import TurnSettings
    from vision.grid_heading import GridHeadingConfig

    turn, grid = TurnSettings(), GridHeadingConfig()
    manager = CalibTuneManager(_fake_calibrator(), tmp_path / "tune.json", extras=turning_bindings(turn, grid))
    status = manager.status()
    groups = {s["group"] for s in status["schema"]}
    assert {"Turning", "Settle", "Grid heading", "Vision"} <= groups
    assert status["values"]["turn_finish_on_time"] == 1

    manager.apply({"turn_latency_s": 0.4, "turn_finish_on_time": 0, "settle_k_psi": 1.5, "grid_dark_thresh": 20})
    assert turn.latency_s == 0.4 and turn.finish_on_time is False and turn.settle_k_psi == 1.5
    assert grid.dark_thresh == 20 and isinstance(grid.dark_thresh, int)

    manager.save()
    turn2, grid2 = TurnSettings(), GridHeadingConfig()
    CalibTuneManager(_fake_calibrator(), tmp_path / "tune.json", extras=turning_bindings(turn2, grid2))
    assert turn2.latency_s == 0.4 and grid2.dark_thresh == 20


def test_profiles_create_load_switch_and_persist_active(tmp_path):
    tune_file = tmp_path / "calib_tune.json"
    calibrator = _fake_calibrator()
    manager = CalibTuneManager(calibrator, tune_file)
    assert manager.active_profile == "default"

    manager.apply({"pid_kp": 1.25})
    manager.save()  # default profile = the original tune file
    manager.apply({"pid_kp": 0.5})
    status = manager.profile_create("hall-dark")
    assert status["profile"]["active"] == "hall-dark"
    assert (tmp_path / "tune_profiles" / "hall-dark.json").is_file()
    assert json.loads(tune_file.read_text())["values"]["pid_kp"] == pytest.approx(1.25)

    manager.profile_load("default")
    assert calibrator.robot_state.pid.kp == pytest.approx(1.25)
    manager.profile_load("hall-dark")
    assert calibrator.robot_state.pid.kp == pytest.approx(0.5)

    # The active profile survives a restart.
    fresh = _fake_calibrator()
    reloaded = CalibTuneManager(fresh, tune_file)
    assert reloaded.active_profile == "hall-dark"
    assert fresh.robot_state.pid.kp == pytest.approx(0.5)
    names = [p["name"] for p in reloaded.profiles()]
    assert names == ["default", "hall-dark"]
    assert reloaded.profile_values("default")["values"]["pid_kp"] == pytest.approx(1.25)


def test_profiles_save_updates_active_profile_only(tmp_path):
    tune_file = tmp_path / "calib_tune.json"
    manager = CalibTuneManager(_fake_calibrator(), tune_file)
    manager.profile_create("turns")
    manager.apply({"pid_kp": 2.0})
    manager.save()
    assert not tune_file.exists()
    saved = json.loads((tmp_path / "tune_profiles" / "turns.json").read_text())
    assert saved["values"]["pid_kp"] == pytest.approx(2.0)


def test_profiles_rename_and_delete_rules(tmp_path):
    tune_file = tmp_path / "calib_tune.json"
    manager = CalibTuneManager(_fake_calibrator(), tune_file)
    manager.profile_create("a")
    manager.profile_rename("a", "b")
    assert manager.active_profile == "b"
    assert (tmp_path / "tune_profiles" / "b.json").is_file()

    with pytest.raises(TuneValidationError, match="switch to another"):
        manager.profile_delete("b")
    with pytest.raises(TuneValidationError, match="default"):
        manager.profile_delete("default")
    with pytest.raises(TuneValidationError, match="default"):
        manager.profile_rename("default", "x")
    with pytest.raises(TuneValidationError, match="already exists"):
        manager.profile_create("b")

    manager.profile_load("default")  # never saved: code defaults
    manager.profile_delete("b")
    assert [p["name"] for p in manager.profiles()] == ["default"]


@pytest.mark.parametrize("name", ["", "../x", "a.b", "has space", "x" * 41, "-lead"])
def test_profile_names_are_single_safe_files(tmp_path, name):
    manager = CalibTuneManager(_fake_calibrator(), tmp_path / "calib_tune.json")
    with pytest.raises(TuneValidationError, match="profile name"):
        manager.profile_create(name)


def test_profile_load_blocked_when_not_idle(tmp_path):
    manager = CalibTuneManager(_fake_calibrator(), tmp_path / "calib_tune.json")
    manager.profile_create("a")
    manager._idle_getter = lambda: (False, "blocked: script running")
    with pytest.raises(TuneBlockedError):
        manager.profile_load("default")


def test_missing_active_profile_falls_back_to_default(tmp_path):
    tune_file = tmp_path / "calib_tune.json"
    manager = CalibTuneManager(_fake_calibrator(), tune_file)
    manager.profile_create("gone")
    (tmp_path / "tune_profiles" / "gone.json").unlink()
    assert CalibTuneManager(_fake_calibrator(), tune_file).active_profile == "default"


def test_frame_flip_is_a_switchable_tune_param(tmp_path):
    from runtime.calib_tuning import FRAME_FLIP_PARAM, attr_binding, flip_code

    view = SimpleNamespace(frame_flip=0)
    manager = CalibTuneManager(
        _fake_calibrator(), tmp_path / "calib_tune.json",
        extras=[attr_binding(FRAME_FLIP_PARAM, view, "frame_flip")],
    )
    manager.apply({"frame_flip": 3})
    assert view.frame_flip == 3 and isinstance(view.frame_flip, int)
    with pytest.raises(TuneValidationError):
        manager.apply({"frame_flip": 4})
    assert [flip_code(m) for m in (0, 1, 2, 3)] == [None, 1, 0, -1]
