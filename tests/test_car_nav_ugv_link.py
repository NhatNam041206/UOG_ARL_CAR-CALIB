"""ugv-controller serial protocol (ros/car_nav/car_nav/ugv_link.py)."""
import math

import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav.actuation import CmdVelMapper, drive_from_cfg
from car_nav.ugv_link import STOP_LINE, UgvConfig, parse_imu, parse_ok

CAR = yaml.safe_load(P.CAR_YAML.read_text())
UGV = UgvConfig.from_cfg(CAR)


def test_cruise_is_the_guides_start_speed_until_measured():
    assert not UGV.measured
    assert UGV.command(0.4, 0.4) == "100 100\n"
    assert STOP_LINE == "0 0\n" and UGV.command(0.0, 0.0) == STOP_LINE


def test_nav2_never_exceeds_the_safety_cap():
    m = CmdVelMapper(drive_from_cfg(CAR))
    for v, w in ((5.0, 0.0), (0.0, 50.0), (0.6, -3.0)):
        left, right = (int(x) for x in UGV.command(*m.map(v, w)).split())
        assert max(abs(left), abs(right)) <= UGV.max_units <= 500


def test_turn_on_the_spot_signs_and_wiring():
    assert UGV.command(-0.2, 0.2) == "-50 50\n"
    inv = UgvConfig(invert_left=True, swap_sides=True)
    assert inv.command(0.2, 0.4) == "100 -50\n"


def test_deadband_and_cap():
    c = UgvConfig(units_per_mps=200, deadband_units=30, max_units=120)
    assert c.wheel_units(0.0) == 0
    assert c.wheel_units(0.1) == 50 and c.wheel_units(-0.1) == -50
    assert c.wheel_units(2.0) == 120 and c.wheel_units(float("nan")) == 0


def test_measured_scale_from_car_yaml():
    c = UgvConfig.from_cfg({"ugv": {"units_per_mps": 180, "max_units": 300}})
    assert c.measured and c.wheel_units(0.5) == 90 and c.max_units == 300


def test_parse_imu_and_ok_lines():
    s = parse_imu("IMU: ax=0.000 ay=0.000 az=1.000 gx=0.00 gy=0.00 gz=57.30 temp=31.25")
    assert s.az == pytest.approx(9.80665) and s.gz == pytest.approx(1.0, abs=1e-3)
    assert parse_imu("IMU: NOT FOUND (check SDA=GPIO6, SCL=GPIO7, 3.3V, GND)") is None
    assert parse_imu("OK: L=1") is None
    assert parse_ok("OK: L=100 (1600us, duty=1310)  R=-50 (1450us, duty=1187)") == (100, -50)
    assert parse_ok("OK: PIP GPIO3 ON for 2s") is None
    assert math.isnan(parse_imu("IMU: ax=0 ay=0 az=1 gx=0 gy=0 gz=0").temp)
