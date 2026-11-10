"""nav2_params.yaml must agree with car.yaml (footprint, speed, steering, latency)."""
import ast
import xml.etree.ElementTree as ET

import yaml

from tests import _car_nav_paths as P
from car_nav.actuation import table_from_cfg

CAR = yaml.safe_load(P.CAR_YAML.read_text())
NAV = yaml.safe_load(P.NAV2_YAML.read_text())
LATENCY_S = 0.65  # lidar-nav thread estimate; also config/sim.yaml truth.latency_s


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
    assert max(x for x, _ in stop) >= CAR["footprint"]["x_front"] + 0.1
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
