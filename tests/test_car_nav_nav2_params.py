"""nav2_params.yaml must agree with car.yaml (footprint, speed, steering, latency)."""
import ast
import xml.etree.ElementTree as ET

import yaml

from tests import _car_nav_paths as P
from car_nav.actuation import table_from_cfg

CAR = yaml.safe_load(P.CAR_YAML.read_text())
NAV = yaml.safe_load(P.NAV2_YAML.read_text())
LATENCY_S = 0.65  # lidar-nav thread estimate; also config/sim.yaml truth.latency_s
SCAN_HZ = 7.0     # RPLIDAR A1
DECEL_MPS2 = 2.0  # config/sim.yaml truth.decel_mps2 (unmeasured)


def rp(node):
    return NAV[node]["ros__parameters"]


def test_footprint_matches_car_yaml():
    f = CAR["footprint"]
    want = [[f["x_front"], f["half_width"]], [f["x_front"], -f["half_width"]],
            [f["x_rear"], -f["half_width"]], [f["x_rear"], f["half_width"]]]
    for cm in ("local_costmap", "global_costmap"):
        got = ast.literal_eval(NAV[cm][cm]["ros__parameters"]["footprint"])
        assert got == want, cm


def test_forward_only_single_speed():
    fp = rp("controller_server")["FollowPath"]
    assert fp["desired_linear_vel"] == CAR["speed"]["baseline_speed_mps"]
    assert fp["allow_reversing"] is False and fp["use_rotate_to_heading"] is False
    assert fp["use_regulated_linear_velocity_scaling"] is False
    assert rp("planner_server")["GridBased"]["motion_model_for_search"] == "DUBIN"
    assert rp("behavior_server")["behavior_plugins"] == ["wait"]


def test_planner_radius_respects_weak_side():
    table = table_from_cfg(CAR)
    r = rp("planner_server")["GridBased"]["minimum_turning_radius"]
    assert r >= 1.2 * table.min_turning_radius


def test_latency_margins():
    v = CAR["speed"]["baseline_speed_mps"]
    dead = v * LATENCY_S
    fp = rp("controller_server")["FollowPath"]
    assert fp["lookahead_dist"] >= 2 * dead
    cm = rp("collision_monitor")
    stop = ast.literal_eval(cm["PolygonStop"]["points"])
    # bumper: reach past the nose by the distance the car covers before a
    # STOP acts (one scan period + latency + braking at the sim's 2 m/s^2)
    stop_dist = v / SCAN_HZ + dead + v * v / (2 * DECEL_MPS2)
    assert max(x for x, _ in stop) >= CAR["footprint"]["x_front"] + stop_dist
    assert max(abs(y) for _, y in stop) <= CAR["footprint"]["half_width"] + 0.03  # not wider than needed
    # approach scales v to d / T; the bridge stops below the threshold, so
    # the car stops for a blocked path within threshold * T metres.
    thr = rp("cmd_vel_to_control")["forward_threshold_mps"]
    assert thr < v
    assert thr * cm["FootprintApproach"]["time_before_collision"] >= dead + 0.15
    assert rp("controller_server")["general_goal_checker"]["xy_goal_tolerance"] >= dead


def test_every_scan_consumer_uses_the_upright_scan():
    assert rp("amcl")["scan_topic"] == "scan_nav"
    assert rp("collision_monitor")["scan"]["topic"] == "/scan_nav"
    assert rp("rf2o_laser_odometry")["laser_scan_topic"] == "/scan_odom"  # upright, inf -> 0
    for cm in ("local_costmap", "global_costmap"):
        assert NAV[cm][cm]["ros__parameters"]["obstacle_layer"]["scan"]["topic"] == "/scan_nav"


def test_behavior_tree_has_no_spin_or_backup():
    tags = {e.tag for e in ET.parse(P.BT_XML).iter()}
    assert not tags & {"Spin", "BackUp", "DriveOnHeading"}
    assert "ComputePathToPose" in tags and "FollowPath" in tags


def test_bt_waits_long_enough_for_servers_to_accept_goals():
    # 20 ms (Nav2 default) made goals fail on a slower laptop; the Pi is no faster.
    assert rp("bt_navigator")["default_server_timeout"] >= 200


def test_pd_follower_matches_rpp_and_goal_checker():
    pd = rp("pd_path_follower")
    assert pd["speed_mps"] == rp("controller_server")["FollowPath"]["desired_linear_vel"]
    assert pd["xy_goal_tolerance"] == rp("controller_server")["general_goal_checker"]["xy_goal_tolerance"]
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
