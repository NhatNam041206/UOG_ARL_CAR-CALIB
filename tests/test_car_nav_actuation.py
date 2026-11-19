"""cmd_vel -> wheel speed mapping for the skid-steer car (ros/car_nav/car_nav/actuation.py)."""
import math

import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav.actuation import CmdVelMapper, DriveConfig, drive_from_cfg, format_payload, parse_payload

CAR = yaml.safe_load(P.CAR_YAML.read_text())
DRIVE = drive_from_cfg(CAR)
M = CmdVelMapper(DRIVE)


def test_car_yaml_is_diff_drive():
    assert CAR["drive"]["type"] == "diff_drive"
    assert "steering" not in CAR
    assert DRIVE.track_width_m > 0.1
    assert DRIVE.cruise_speed_mps <= DRIVE.max_wheel_speed_mps


def test_straight_and_spin():
    assert M.map(0.4, 0.0) == pytest.approx((0.4, 0.4))
    left, right = M.map(0.0, 1.0)
    assert left == pytest.approx(-right) and right == pytest.approx(DRIVE.track_width_m / 2)
    v, w = DRIVE.body(left, right)
    assert v == pytest.approx(0.0) and w == pytest.approx(1.0)


def test_left_turn_speeds_up_the_right_wheel():
    left, right = M.map(0.3, 0.5)
    assert right > left
    assert DRIVE.body(left, right) == pytest.approx((0.3, 0.5))


def test_saturation_keeps_curvature():
    cfg = DriveConfig(track_width_m=0.2, max_wheel_speed_mps=0.5, max_yaw_rate_rps=10.0)
    m = CmdVelMapper(cfg)
    left, right = m.map(0.5, 2.0)          # ideal right 0.7 > 0.5
    assert max(abs(left), abs(right)) == pytest.approx(0.5)
    v, w = cfg.body(left, right)
    assert w / v == pytest.approx(2.0 / 0.5)


def test_yaw_rate_is_capped():
    left, right = M.map(0.0, 50.0)
    assert DRIVE.body(left, right)[1] <= DRIVE.max_yaw_rate_rps + 1e-9


def test_no_reverse_unless_allowed():
    assert not DRIVE.allow_reverse
    assert M.map(-0.3, 0.0) == pytest.approx((0.0, 0.0))
    left, right = M.map(-0.3, 1.0)        # reverse dropped, the turn kept
    assert DRIVE.body(left, right)[0] == pytest.approx(0.0)
    rev = CmdVelMapper(DriveConfig(allow_reverse=True))
    assert rev.map(-0.3, 0.0) == pytest.approx((-0.3, -0.3))


def test_dead_band_and_bad_input():
    m = CmdVelMapper(DriveConfig(min_wheel_speed_mps=0.05))
    assert m.map(0.02, 0.0) == (0.0, 0.0)
    assert M.map(float("nan"), 0.0) == (0.0, 0.0)


def test_payload_round_trip():
    s = format_payload(0.3456, -0.1)
    assert s == "0.346,-0.100"
    assert parse_payload(s) == pytest.approx((0.346, -0.1))
    assert parse_payload("62,1x") is None and parse_payload("nan,0") is None
    assert parse_payload("garbage") is None


def test_unmeasured_track_uses_the_assumed_value():
    cfg = drive_from_cfg({"drive": {"track_width_m": None, "assumed_track_width_m": 0.25}})
    assert cfg.track_width_m == 0.25 and not cfg.measured
    assert drive_from_cfg({"drive": {"track_width_m": 0.19}}).measured
    assert math.isclose(DriveConfig(track_width_m=0.2, max_wheel_speed_mps=0.5).spin_yaw_rate_rps, 5.0)


def test_slow_spin_is_raised_to_the_stiction_speed():
    m = CmdVelMapper(DriveConfig(track_width_m=0.2, min_spin_wheel_mps=0.08))
    left, right = m.map(0.0, 0.3)            # 0.03 m/s wheels: would not turn
    assert right == pytest.approx(0.08) and left == pytest.approx(-0.08)
    assert m.map(0.0, 0.0) == (0.0, 0.0)
    assert m.map(0.3, 0.1) == pytest.approx((0.29, 0.31))   # driving: unchanged
