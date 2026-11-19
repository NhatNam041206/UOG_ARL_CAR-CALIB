"""nav2_params.yaml must agree with car.yaml (footprint, speed, diff drive, latency)."""
import ast
import xml.etree.ElementTree as ET

import yaml

from tests import _car_nav_paths as P
from car_nav.actuation import drive_from_cfg

CAR = yaml.safe_load(P.CAR_YAML.read_text())
NAV = yaml.safe_load(P.NAV2_YAML.read_text())
DRIVE = drive_from_cfg(CAR)
LATENCY_S = 0.25  # assumed (car.yaml speed.command_latency_s null); also config/sim.yaml truth.latency_s
SCAN_HZ = 7.0     # RPLIDAR A1
DECEL_MPS2 = 2.5  # config/sim.yaml truth.decel_mps2 (unmeasured)


def rp(node):
    return NAV[node]["ros__parameters"]


def test_footprint_matches_car_yaml():
    f = CAR["footprint"]
    want = [[f["x_front"], f["half_width"]], [f["x_front"], -f["half_width"]],
            [f["x_rear"], -f["half_width"]], [f["x_rear"], f["half_width"]]]
    for cm in ("local_costmap", "global_costmap"):
        got = ast.literal_eval(NAV[cm][cm]["ros__parameters"]["footprint"])
        assert got == want, cm


def test_diff_drive_controllers_and_planner():
    fp = rp("controller_server")["FollowPath"]
    assert fp["desired_linear_vel"] == DRIVE.cruise_speed_mps
    assert fp["allow_reversing"] is DRIVE.allow_reverse is False
    assert fp["use_rotate_to_heading"] is True            # turns on the spot
    assert fp["rotate_to_heading_angular_vel"] <= DRIVE.max_yaw_rate_rps
    assert rp("planner_server")["GridBased"]["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert set(rp("behavior_server")["behavior_plugins"]) == {"spin", "wait"}


def test_speeds_fit_the_wheel_limit():
    # driving at cruise speed must leave the wheels room to turn
    assert DRIVE.cruise_speed_mps < DRIVE.max_wheel_speed_mps
    pd = rp("pd_path_follower")
    assert pd["max_yaw_rate_rps"] == DRIVE.max_yaw_rate_rps
    assert pd["max_yaw_rate_rps"] <= DRIVE.spin_yaw_rate_rps


def test_latency_margins():
    v = DRIVE.cruise_speed_mps
    dead = v * LATENCY_S
    cm = rp("collision_monitor")
    vs = cm["VelocityStop"]
    assert vs["type"] == "velocity_polygon"
    fwd = ast.literal_eval(vs["forward"]["points"])
    # bumper: reach past the nose by the distance the car covers before a
    # stop acts (one scan period + latency + braking)
    stop_dist = v / SCAN_HZ + dead + v * v / (2 * DECEL_MPS2)
    assert max(x for x, _ in fwd) >= CAR["footprint"]["x_front"] + stop_dist
    assert max(abs(y) for _, y in fwd) <= CAR["footprint"]["half_width"] + 0.03  # not wider than needed
    # turning on the spot: only the footprint + a few cm, so a wall ahead does not block the turn
    rot = ast.literal_eval(vs["rotation"]["points"])
    assert max(x for x, _ in rot) <= CAR["footprint"]["x_front"] + 0.05
    # every commanded (v, omega) falls into one of the sub-polygons
    lo = min(vs[n]["linear_min"] for n in vs["velocity_polygons"])
    hi = max(vs[n]["linear_max"] for n in vs["velocity_polygons"])
    assert lo <= -0.05 and hi >= DRIVE.max_wheel_speed_mps
    assert all(vs[n]["theta_max"] >= DRIVE.max_yaw_rate_rps for n in vs["velocity_polygons"])
    assert cm["FootprintApproach"]["time_before_collision"] * v >= dead + 0.15
    assert rp("pd_path_follower")["avoid_horizon_m"] >= dead + 0.5


def test_every_scan_consumer_uses_the_upright_scan():
    assert rp("amcl")["scan_topic"] == "scan_nav"
    assert rp("collision_monitor")["scan"]["topic"] == "/scan_nav"
    assert rp("rf2o_laser_odometry")["laser_scan_topic"] == "/scan_odom"  # upright, inf -> 0
    for cm in ("local_costmap", "global_costmap"):
        assert NAV[cm][cm]["ros__parameters"]["obstacle_layer"]["scan"]["topic"] == "/scan_nav"


def test_behavior_tree_spins_but_never_backs_up():
    tags = {e.tag for e in ET.parse(P.BT_XML).iter()}
    assert "Spin" in tags                       # skid-steer: turning on the spot is a recovery
    assert not tags & {"BackUp", "DriveOnHeading"}
    assert "ComputePathToPose" in tags and "FollowPath" in tags


def test_bt_waits_long_enough_for_servers_to_accept_goals():
    # 20 ms (Nav2 default) made goals fail on a slower laptop; the Pi is no faster.
    assert rp("bt_navigator")["default_server_timeout"] >= 200


def test_pd_follower_matches_rpp_and_goal_checker():
    pd = rp("pd_path_follower")
    assert pd["speed_mps"] == rp("controller_server")["FollowPath"]["desired_linear_vel"]
    assert pd["xy_goal_tolerance"] == rp("controller_server")["general_goal_checker"]["xy_goal_tolerance"]
    assert pd["yaw_goal_tolerance"] == rp("controller_server")["general_goal_checker"]["yaw_goal_tolerance"]
    assert all(isinstance(pd[k], float) for k in ("kp", "kd", "ff", "speed_mps", "xy_goal_tolerance"))


def test_pd_behavior_tree_differs_only_in_follow_path_server():
    a = P.BT_XML.read_text()
    b = P.BT_XML.with_name("navigate_forward_only_pd.xml").read_text()
    fp = [e for e in ET.fromstring(b).iter() if e.tag == "FollowPath"]
    assert len(fp) == 1 and fp[0].get("server_name") == "pd_follow_path"
    strip = lambda t: [l for l in t.split("<!--")[1].split("-->")[1].splitlines()]  # noqa: E731
    diff = [x for x, y in zip(strip(a), strip(b)) if x != y]
    assert len(diff) == 1 and "FollowPath path=" in diff[0]


def test_global_costmap_plans_on_the_static_map_only():
    # phantom obstacles from a lagging pose blocked narrow corridors (2026-11-14)
    plugins = NAV["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    assert "obstacle_layer" not in plugins and "static_layer" in plugins
    assert "obstacle_layer" in NAV["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]


def test_inflation_and_guard_widths_fit_the_body():
    f = CAR["footprint"]
    for cm in ("local_costmap", "global_costmap"):
        p = NAV[cm][cm]["ros__parameters"]
        inscribed = f["half_width"] + p["footprint_padding"]
        infl = p["inflation_layer"]["inflation_radius"]
        assert infl >= inscribed + 0.3, cm          # a real gradient beyond the blocked cells
    # the global one reaches the middle of a 1.2 m corridor (0.6 m from each wall) so paths stay centred
    g = NAV["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]
    assert g["inflation_radius"] > 0.6
    sg = rp("safety_guard")
    assert sg["half_width_m"] >= f["half_width"] + 0.1
    assert sg["slow_half_width_m"] > sg["half_width_m"]
    # collision monitor: the rotation polygon covers the footprint
    rot = ast.literal_eval(rp("collision_monitor")["VelocityStop"]["rotation"]["points"])
    assert max(x for x, _ in rot) >= f["x_front"] and max(y for _, y in rot) >= f["half_width"]
    assert rp("amcl")["laser_min_range"] >= f["half_width"]    # the body is never an AMCL beam


def test_laser_and_camera_measurements():
    # owner's measurements 2026-11-19
    assert (CAR["laser"]["z"], CAR["camera"]["z"]) == (0.24, 1.20)
    assert (CAR["camera"]["x"], CAR["camera"]["y"]) == (0.0, 0.0)
    assert CAR["drive"]["track_width_m"] is None and CAR["drive"]["assumed_track_width_m"] == 0.52
