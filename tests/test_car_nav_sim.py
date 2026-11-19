"""Simulator models (ros/car_nav_sim): actuator truth, kinematics, world, missions."""
import math

import numpy as np
import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav.actuation import CmdVelMapper, drive_from_cfg, format_payload
from car_nav.config_loader import footprint, laser_pose
from car_nav_sim.actuator_model import ActuatorModel, Pose2D, TruthParams, integrate
from car_nav_sim.scan_model import laser_scan_ranges
from car_nav_sim.sdf import world_sdf
from car_nav_sim.world import Box, World, footprint_polygon

CAR = yaml.safe_load(P.CAR_YAML.read_text())
DRIVE = drive_from_cfg(CAR)
MAPPER = CmdVelMapper(DRIVE)
WORLD = World.load(str(P.SIM_DIR / "worlds" / "test_floor.yaml"))
SIM = yaml.safe_load((P.SIM_DIR / "config" / "sim.yaml").read_text())


def drive(model, seconds, dt=0.01, pose=None, t0=0.0):
    pose = pose or Pose2D()
    t = t0
    while t < t0 + seconds - 1e-9:
        v, w = model.step(t, dt)
        pose = integrate(pose, v, w, dt)
        t += dt
    return pose


def test_latency_delays_the_command():
    m = ActuatorModel(DRIVE, TruthParams(latency_s=0.25, accel_mps2=100, decel_mps2=100))
    m.command(0.0, "0.400,0.400")
    assert m.step(0.2, 0.01)[0] == 0.0
    assert m.step(0.3, 0.01)[0] == pytest.approx(0.4)


def test_spin_on_the_spot_with_yaw_efficiency():
    m = ActuatorModel(DRIVE, TruthParams(latency_s=0.0, accel_mps2=100, decel_mps2=100, yaw_efficiency=0.7))
    left, right = MAPPER.map(0.0, 1.0)
    m.command(0.0, format_payload(left, right))
    p = drive(m, 2.0)
    assert math.hypot(p.x, p.y) < 1e-6                       # turned on the spot
    assert p.theta == pytest.approx(0.7 * 2.0, abs=0.02)     # slower than commanded (skid)
    # encoders see the commanded turn, not the slip
    assert m.encoder_twist()[1] == pytest.approx(1.0, abs=0.01)


def test_arc_and_side_gain():
    m = ActuatorModel(DRIVE, TruthParams(latency_s=0.0, accel_mps2=100, decel_mps2=100, yaw_efficiency=1.0))
    m.command(0.0, format_payload(*MAPPER.map(0.3, 0.3)))   # radius 1 m, left
    p = drive(m, math.pi / 2 / 0.3)                          # quarter circle
    assert (p.x, p.y) == pytest.approx((1.0, 1.0), abs=0.02)
    weak = ActuatorModel(DRIVE, TruthParams(latency_s=0.0, accel_mps2=100, right_gain=0.9, yaw_efficiency=1.0))
    weak.command(0.0, "0.400,0.400")
    assert drive(weak, 3.0).y < -0.1                         # weak right side drifts right


def test_stiction_and_failsafe():
    m = ActuatorModel(DRIVE, TruthParams(latency_s=0.0, accel_mps2=100, decel_mps2=100, min_spin_wheel_mps=0.1,
                                         failsafe_timeout_s=0.5))
    m.command(0.0, "-0.050,0.050")
    assert m.step(0.1, 0.01)[1] == 0.0                       # too slow to turn on the spot
    m.command(0.2, "0.300,0.300")
    assert m.step(0.3, 0.01)[0] > 0.2
    v, _ = m.step(1.0, 0.01)                                 # 0.8 s without commands -> stopping
    assert v < 0.3 and m.target == (0.0, 0.0)


def test_bad_payload_rejected():
    m = ActuatorModel(DRIVE, TruthParams())
    assert not m.command(0.0, "62,x") and m.rejected == 1


def test_raycast_box_and_cylinder():
    w = World("t", (-5, -5, 5, 5), [Box(2.0, 0.0, 1.0, 1.0)], [])
    r = w.raycast(0.0, 0.0, np.array([0.0, math.pi / 2]), 10.0)
    assert r[0] == pytest.approx(1.5) and math.isinf(r[1])


def test_upright_scan_sees_the_left_wall_on_the_left():
    w = World("t", (-5, -5, 5, 5), [Box(0.0, 1.0, 10.0, 0.1)], [])  # wall 0.95 m to the left
    laser = laser_pose(CAR)
    assert laser.roll == 0.0 and laser.pitch == 0.0
    angles = np.array([math.pi / 2, -math.pi / 2])
    r = laser_scan_ranges(w, Pose2D(), laser, angles, {"range_max_m": 12.0}, np.random.default_rng(0))
    # The laser is mounted upright: its +90 deg beam points to the car's LEFT.
    assert r[0] == pytest.approx(0.95) and math.isinf(r[1])


def test_map_matches_world(tmp_path):
    occ = WORLD.occupancy(0.05)
    assert occ.shape == (140, 200)
    path = WORLD.write_map(str(tmp_path))
    meta = yaml.safe_load(open(path))
    assert meta["origin"][:2] == [-0.5, -0.5] and (tmp_path / "test_floor.pgm").stat().st_size > 28000


def test_clearance():
    w = World("t", (-5, -5, 5, 5), [Box(1.0, 0.0, 0.2, 2.0)], [])
    f = footprint(CAR)
    poly = footprint_polygon(0.0, 0.0, 0.0, f.x_front, f.x_rear, f.half_width)
    assert w.clearance(poly) == pytest.approx(0.9 - f.x_front)          # wall face at x = 0.9
    assert w.clearance(footprint_polygon(0.6, 0.0, 0.0, f.x_front, f.x_rear, f.half_width)) == 0.0


@pytest.mark.parametrize("mission", sorted(p.stem for p in (P.SIM_DIR / "missions").glob("*.yaml")))
def test_mission_poses_are_free(mission):
    m = yaml.safe_load((P.SIM_DIR / "missions" / f"{mission}.yaml").read_text())
    world = World.load(str(P.SIM_DIR / "worlds" / f"{m.get('world', 'test_floor')}.yaml"))
    fp = footprint(CAR)
    # A car centred in a 1.2 m corridor has exactly (1.2 - 0.6) / 2 = 0.3 m to each wall:
    # a mission in such corridors says so (min_goal_clearance_m) and may not go below it.
    need = float(m.get("min_goal_clearance_m", 0.4))
    assert need >= 0.25
    for x, y, th in [m.get("start") or world.start] + m["goals"]:
        poly = footprint_polygon(x, y, th, fp.x_front, fp.x_rear, fp.half_width)
        assert world.clearance(poly) > need, (mission, x, y)


def test_sdf_has_upright_lidar_and_velocity_control():
    sdf = world_sdf(WORLD, CAR, SIM)
    assert "gz-sim-velocity-control-system" in sdf and "<gz_frame_id>laser</gz_frame_id>" in sdf
    assert "0 0.24 0.0 0 0" in sdf


def test_low_obstacles_are_invisible_to_a_laser_above_them():
    # laser at z 0.24: a 0.15 m high box is not hit, a 0.5 m high one is; the map shows the tall one only
    w = World("t", (-5, -5, 5, 5), [Box(2.0, 0.0, 0.2, 2.0, height=0.15), Box(3.0, 0.0, 0.2, 2.0, height=0.5)],
              [])
    from car_nav_sim.world import Cylinder
    w.cylinders.append(Cylinder(0.0, 2.0, 0.2, height=0.1))
    a = np.array([0.0, math.pi / 2])
    assert w.raycast(0.0, 0.0, a, 10.0, z=0.24)[0] == pytest.approx(2.9)
    assert math.isinf(w.raycast(0.0, 0.0, a, 10.0, z=0.24)[1])
    assert w.raycast(0.0, 0.0, a, 10.0, z=None)[0] == pytest.approx(1.9)
    assert w.raycast(0.0, 0.0, a, 10.0)[1] == pytest.approx(1.8)
    occ = w.occupancy(0.05, min_height=0.24)
    full = w.occupancy(0.05)
    assert occ.sum() < full.sum() and occ.any()
    laser = laser_pose(CAR)
    r = laser_scan_ranges(w, Pose2D(), laser, a, {"range_max_m": 12.0}, np.random.default_rng(0))
    assert r[0] == pytest.approx(2.9) and math.isinf(r[1])
