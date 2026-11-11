"""cmd_vel -> "<angle>,<drive_code>" mapping (ros/car_nav/car_nav/actuation.py)."""
import math

import pytest
import yaml

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.actuation import (
    DRIVE_FORWARD, DRIVE_STOP, CmdVelMapper, SteeringConfig, SteeringTable,
    SteeringSmoother, format_payload, parse_payload, table_from_cfg,
)


@pytest.fixture(scope="module")
def table():
    return table_from_cfg(yaml.safe_load(P.CAR_YAML.read_text()))


def test_car_yaml_steering_facts(table):
    c = table.cfg
    assert (c.wheelbase_m, c.servo_center, c.servo_min, c.servo_max) == (0.28, 62, 35, 140)
    assert not table.measured


def test_placeholder_limits_per_side(table):
    # right side: 27 servo deg -> tan(27)/0.28 ; left side capped at 30 deg wheel angle
    assert table.max_right_curvature == pytest.approx(math.tan(math.radians(27)) / 0.28)
    assert table.max_left_curvature == pytest.approx(math.tan(math.radians(30)) / 0.28)
    assert table.min_turning_radius == pytest.approx(0.28 / math.tan(math.radians(27)), rel=1e-6)
    assert 0.54 < table.min_turning_radius < 0.56


def test_servo_above_centre_turns_left(table):
    assert table.curvature(70) > 0 and table.curvature(50) < 0 and table.curvature(62) == 0


def test_inverse_round_trip(table):
    for k in (-1.5, -0.8, -0.1, 0.1, 0.8, 1.9):
        assert table.curvature(table.servo_for(k)) == pytest.approx(k, abs=1e-9)


def test_inverse_saturates_on_each_side(table):
    assert table.servo_for(-10.0) == 35
    # left saturates where the wheel-angle cap is reached (62 + 30), not at 140
    assert table.servo_for(10.0) == pytest.approx(92)


def test_mapper_forward_and_stop(table):
    m = CmdVelMapper(table)
    assert m.map(0.5, 0.0) == (62, DRIVE_FORWARD)
    a, d = m.map(0.5, 0.5 * 1.0)  # 1 m radius left turn
    assert d == DRIVE_FORWARD and a > 62
    a, d = m.map(0.5, -0.5 * 1.0)
    assert d == DRIVE_FORWARD and a < 62
    assert m.map(0.5, -5.0) == (35, DRIVE_FORWARD)
    assert m.map(0.01, 0.3) == (62, DRIVE_STOP)
    assert m.map(-0.3, 0.0) == (62, DRIVE_STOP)  # forward only: never reverse
    assert m.map(float("nan"), 0.0) == (62, DRIVE_STOP)


def test_payload_round_trip():
    assert format_payload(48, 1) == "48,1"
    assert parse_payload("48,1") == (48, 1)
    assert parse_payload(" 62.0,0 ") == (62, 0)
    assert parse_payload("garbage") is None


def test_measured_table_is_used_and_validated():
    cfg = SteeringConfig(curvature_table=((35, -1.6), (50, -0.6), (75, 0.7), (110, 2.0)))
    t = SteeringTable(cfg)
    assert t.measured and t.curvature(62) == 0.0
    assert t.min_turning_radius == pytest.approx(1 / 1.6)
    with pytest.raises(ValueError):
        SteeringTable(SteeringConfig(curvature_table=((35, 1.0), (110, 2.0))))  # wrong sign
    with pytest.raises(ValueError):
        SteeringTable(SteeringConfig(curvature_table=((20, -1.0), (110, 2.0))))  # outside limits


def test_servo_up_turns_right_flips_sign():
    t = SteeringTable(SteeringConfig(servo_up_turns="right"))
    assert t.curvature(70) < 0 and t.servo_for(1.0) < 62


def test_smoother_off_passes_through():
    s = SteeringSmoother()
    assert not s.enabled
    assert [s.step(a, 0.1) for a in (62, 100, 40)] == [62, 100, 40]


def test_smoother_lowpass_reaches_target_and_ignores_call_rate():
    a, b = SteeringSmoother(tau_s=0.3), SteeringSmoother(tau_s=0.3)
    a.step(62, 0.1); b.step(62, 0.1)
    for _ in range(10):
        a.step(90, 0.1)
    for _ in range(20):
        b.step(90, 0.05)
    assert a.step(90, 0.0) == pytest.approx(b.step(90, 0.0), abs=1e-6)
    for _ in range(100):
        a.step(90, 0.1)
    assert a.step(90, 0.1) == pytest.approx(90, abs=0.01)


def test_smoother_rate_limit_and_reset():
    s = SteeringSmoother(max_rate_dps=60)
    s.step(62, 0.1)
    assert s.step(140, 0.1) == pytest.approx(68)
    s.reset(None)
    assert s.step(140, 0.1) == 140
