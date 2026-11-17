"""Upside-down scan -> upright frame (ros/car_nav/car_nav/scan_tools.py)."""
import math

import pytest

from tests import _car_nav_paths  # noqa: F401
from car_nav.geometry import Pose3D, beam_to_base
from car_nav.scan_tools import is_inverted, upright


@pytest.mark.parametrize("yaw", [0.0, 0.4, math.pi])
def test_upright_scan_points_land_in_the_same_place(yaw):
    inv = Pose3D(x=-0.05, z=0.42, roll=math.pi, yaw=yaw)
    up = Pose3D(x=-0.05, z=0.42, yaw=yaw)
    n = 8
    amin, inc = -math.pi, 2 * math.pi / n
    amax = amin + (n - 1) * inc
    ranges = [1.0 + 0.1 * i for i in range(n)]
    r2, _, amin2, amax2 = upright(ranges, [], amin, amax, True)
    assert amax2 - amin2 == pytest.approx(amax - amin)
    for i in range(n):
        p_inv = beam_to_base(ranges[i], amin + i * inc, inv)
        j = n - 1 - i
        p_up = beam_to_base(r2[j], amin2 + j * inc, up)
        assert p_up == pytest.approx(p_inv, abs=1e-9)


def test_is_inverted():
    assert is_inverted(math.pi) and is_inverted(-math.pi) and not is_inverted(0.0)
    with pytest.raises(ValueError):
        is_inverted(1.0)


def test_zero_invalid_for_rf2o():
    from car_nav.scan_tools import zero_invalid

    assert zero_invalid([1.0, float("inf"), float("nan"), -1.0, 0.0]) == [1.0, 0.0, 0.0, 0.0, 0.0]
