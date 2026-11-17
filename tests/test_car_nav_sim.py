"""Simulator models (ros/car_nav_sim): actuator truth, kinematics, world, missions."""
import math

import numpy as np
import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav.actuation import CmdVelMapper, format_payload, table_from_cfg
from car_nav.config_loader import footprint, laser_pose
from car_nav_sim.actuator_model import ActuatorModel, Pose2D, TruthParams, integrate
from car_nav_sim.scan_model import laser_scan_ranges
from car_nav_sim.sdf import world_sdf
from car_nav_sim.world import Box, World, footprint_polygon

CAR = yaml.safe_load(P.CAR_YAML.read_text())
TABLE = table_from_cfg(CAR)
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
    m = ActuatorModel(TABLE, TruthParams(latency_s=0.65, accel_mps2=100))
    m.command(0.0, "62,1")
    p = drive(m, 0.6)
    assert p.x == pytest.approx(0.0)
    p = drive(m, 1.0, pose=p, t0=0.6)
    assert p.x > 0.15


def test_fixed_servo_drives_the_table_circle():
    m = ActuatorModel(TABLE, TruthParams(latency_s=0.0, accel_mps2=100, servo_rate_dps=1e6))
    m.command(0.0, "35,1")  # full right
    R = 1 / TABLE.max_right_curvature
    pose, t, pts = Pose2D(), 0.0, []
    while t < 2 * math.pi * R / 0.5:
        v, w = m.step(t, 0.01)
        pose = integrate(pose, v, w, 0.01)
        pts.append((pose.x, pose.y))
        t += 0.01
    # right turn: circle centre at (0, -R) for a start at the origin heading +x
    d = [math.hypot(x, y + R) for x, y in pts[10:]]
    assert max(d) == pytest.approx(R, abs=0.01) and min(d) == pytest.approx(R, abs=0.01)


def test_rear_swings_out_in_a_turn():
    """Front axle stays on its circle; the rear axle point moves outward first."""
    m = ActuatorModel(TABLE, TruthParams(latency_s=0.0, accel_mps2=100, servo_rate_dps=1e6))
    m.command(0.0, "90,1")  # left turn
    p = drive(m, 0.3)
    rear_y = p.y - 0.28 * math.sin(p.theta)
    assert p.y > 0 and rear_y < 0  # front moved left, the tail went right


def test_understeer_and_failsafe():
    m = ActuatorModel(TABLE, TruthParams(latency_s=0.0, understeer_factor=0.5, servo_rate_dps=1e6,
                                         accel_mps2=100, failsafe_timeout_s=0.5))
    m.command(0.0, "35,1")
    v, w = m.step(0.0, 0.01)
    assert w / v == pytest.approx(-0.5 * TABLE.max_right_curvature)
    drive(m, 1.5, t0=0.01)
    assert m.drive == 0
    assert m.command(2.0, "1,2") and m._queue[-1][1:] == (35.0, 0)  # clamp + BACKWARD treated as STOP


def test_raycast_box_and_cylinder():
    w = World("t", (-5, -5, 5, 5), [Box(2.0, 0.0, 1.0, 1.0)], [])
    r = w.raycast(0.0, 0.0, np.array([0.0, math.pi / 2]), 10.0)
    assert r[0] == pytest.approx(1.5) and math.isinf(r[1])


def test_upside_down_scan_sees_the_left_wall_on_the_left():
    w = World("t", (-5, -5, 5, 5), [Box(0.0, 1.0, 10.0, 0.1)], [])  # wall 0.95 m to the left
    laser = laser_pose(CAR)
    angles = np.array([math.pi / 2, -math.pi / 2])
    r = laser_scan_ranges(w, Pose2D(), laser, angles, {"range_max_m": 12.0}, np.random.default_rng(0))
    # The laser is upside down: its +90 deg beam points to the car's RIGHT (no wall there).
    assert math.isinf(r[0]) and r[1] == pytest.approx(0.95)


def test_map_matches_world(tmp_path):
    occ = WORLD.occupancy(0.05)
    assert occ.shape == (140, 200)
    path = WORLD.write_map(str(tmp_path))
    meta = yaml.safe_load(open(path))
    assert meta["origin"][:2] == [-0.5, -0.5] and (tmp_path / "test_floor.pgm").stat().st_size > 28000


def test_clearance():
    w = World("t", (-5, -5, 5, 5), [Box(1.0, 0.0, 0.2, 2.0)], [])
    poly = footprint_polygon(0.0, 0.0, 0.0, 0.16, -0.37, 0.135)
    assert w.clearance(poly) == pytest.approx(0.74)
    assert w.clearance(footprint_polygon(0.85, 0.0, 0.0, 0.16, -0.37, 0.135)) == 0.0


@pytest.mark.parametrize("mission", sorted(p.stem for p in (P.SIM_DIR / "missions").glob("*.yaml")))
def test_mission_poses_are_free(mission):
    m = yaml.safe_load((P.SIM_DIR / "missions" / f"{mission}.yaml").read_text())
    world = World.load(str(P.SIM_DIR / "worlds" / f"{m.get('world', 'test_floor')}.yaml"))
    fp = footprint(CAR)
    for x, y, th in [m.get("start") or world.start] + m["goals"]:
        poly = footprint_polygon(x, y, th, fp.x_front, fp.x_rear, fp.half_width)
        assert world.clearance(poly) > 0.4, (mission, x, y)


def test_sdf_has_inverted_lidar_and_velocity_control():
    sdf = world_sdf(WORLD, CAR, SIM)
    assert "gz-sim-velocity-control-system" in sdf and "<gz_frame_id>laser</gz_frame_id>" in sdf
    assert f"0 0.42 {math.pi} 0 0" in sdf


def _pure_pursuit_max_error(lookahead, radius, side, latency=0.65, understeer=1.0):
    """Closed loop: pure pursuit at base_link -> mapper -> actuator model with latency."""
    s = 1 if side == "L" else -1
    path = [(i * 0.02, 0.0) for i in range(150)]
    path += [(3 + radius * math.sin(a), s * radius * (1 - math.cos(a)))
             for a in np.linspace(0, math.pi / 2, 80)[1:]]
    path += [(3 + radius, s * (radius + i * 0.02)) for i in range(1, 150)]
    mapper = CmdVelMapper(TABLE)
    model = ActuatorModel(TABLE, TruthParams(latency_s=latency, understeer_factor=understeer))
    pose, t, next_cmd, worst = Pose2D(), 0.0, 0.0, 0.0
    while t < 30:
        if t >= next_cmd:
            next_cmd += 0.1  # controller_frequency 10 Hz
            ci = min(range(len(path)), key=lambda i: (path[i][0] - pose.x) ** 2 + (path[i][1] - pose.y) ** 2)
            worst = max(worst, math.dist(path[ci], (pose.x, pose.y)))
            if ci >= len(path) - 5:
                break
            j = ci
            while j < len(path) - 1 and math.dist(path[j], (pose.x, pose.y)) < lookahead:
                j += 1
            dx, dy = path[j][0] - pose.x, path[j][1] - pose.y
            ly = -math.sin(pose.theta) * dx + math.cos(pose.theta) * dy
            k = 2 * ly / (dx * dx + dy * dy)
            model.command(t, format_payload(*mapper.map(0.5, 0.5 * k)))
        v, w = model.step(t, 0.01)
        pose = integrate(pose, v, w, 0.01)
        t += 0.01
    return worst


def test_nav2_lookahead_tracks_turns_despite_latency():
    nav = yaml.safe_load(P.NAV2_YAML.read_text())
    rpp = nav["controller_server"]["ros__parameters"]["FollowPath"]
    r_plan = nav["planner_server"]["ros__parameters"]["GridBased"]["minimum_turning_radius"]
    for side in ("L", "R"):
        assert _pure_pursuit_max_error(rpp["lookahead_dist"], r_plan, side) < 0.12
