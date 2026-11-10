"""car_nav geometry (ros/car_nav/car_nav/geometry.py), no ROS needed."""
import importlib.util
import math
import pathlib
import sys

_PATH = pathlib.Path(__file__).resolve().parents[1] / "ros" / "car_nav" / "car_nav" / "geometry.py"
_spec = importlib.util.spec_from_file_location("car_nav_geometry", _PATH)
geometry = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = geometry
_spec.loader.exec_module(geometry)

Pose3D, Footprint = geometry.Pose3D, geometry.Footprint
UPSIDE_DOWN = Pose3D(x=0.0, z=0.42, roll=math.pi)
FOOTPRINT = Footprint(x_front=0.16, x_rear=-0.37, half_width=0.135)


def test_upright_laser_left_beam_is_left():
    bx, by = geometry.beam_to_base(1.0, math.pi / 2, Pose3D(z=0.42))
    assert abs(bx) < 1e-9 and abs(by - 1.0) < 1e-9


def test_upside_down_laser_flips_left_and_right():
    # A beam the (inverted) laser reports at +90 deg actually points to the car's right.
    bx, by = geometry.beam_to_base(1.0, math.pi / 2, UPSIDE_DOWN)
    assert abs(bx) < 1e-9 and abs(by + 1.0) < 1e-9
    # Straight ahead stays straight ahead.
    bx, by = geometry.beam_to_base(2.0, 0.0, UPSIDE_DOWN)
    assert abs(bx - 2.0) < 1e-9 and abs(by) < 1e-9


def test_laser_offset_and_yaw():
    laser = Pose3D(x=-0.1, z=0.42, roll=math.pi, yaw=math.pi)
    bx, by = geometry.beam_to_base(1.0, 0.0, laser)
    assert abs(bx - (-1.1)) < 1e-9 and abs(by) < 1e-9


def test_self_mask_drops_points_on_the_car():
    # angle 0 -> 0.1 m ahead of the laser at x=0: inside the footprint.
    # angle pi/2 -> 1.0 m to the side: kept.
    ranges, masked = geometry.self_mask([0.1, 1.0], 0.0, math.pi / 2, UPSIDE_DOWN, FOOTPRINT)
    assert masked == 1
    assert math.isinf(ranges[0]) and ranges[1] == 1.0


def test_self_mask_sectors_and_invalid_beams_untouched():
    ranges, masked = geometry.self_mask(
        [float("nan"), 2.0], 0.0, math.pi, UPSIDE_DOWN, FOOTPRINT, sectors_deg=[[170, -170]]
    )
    # beam 1 at laser angle pi -> base bearing 180 -> masked by the sector
    assert masked == 1 and math.isnan(ranges[0]) and math.isinf(ranges[1])


def test_front_distance_uses_corridor():
    # beam 0 ahead at 1.5 m, beam 1 at laser +90 deg (car right) 0.5 m: outside corridor
    d, n = geometry.front_distance([1.5, 0.5], 0.0, math.pi / 2, UPSIDE_DOWN, 0.2)
    assert n == 1 and abs(d - 1.5) < 1e-9
