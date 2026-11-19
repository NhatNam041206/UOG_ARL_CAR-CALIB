"""Depth camera as a second safety-guard source: pure modules (no ROS)."""
import math

import numpy as np
import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav import config_loader
from car_nav.depth_obstacles import (DepthParams, blind_distance, cloud_to_base, encode_xyz, height_range_filter,
                                     obstacle_points, parse_cloud, voxel_downsample)
from car_nav.dynamic_obstacles import dynamic_points, static_mask
from car_nav.geometry import Pose3D
from car_nav.safety_zone import GuardParams, decide, decide_merged, depth_params
from car_nav_sim.actuator_model import Pose2D
from car_nav_sim.depth_model import depth_points, pixel_rays
from car_nav_sim.scan_model import laser_scan_ranges
from car_nav_sim.world import Box, Cylinder, World

CAR = yaml.safe_load(P.CAR_YAML.read_text())
CAM = config_loader.camera_pose(CAR)
LASER = config_loader.laser_pose(CAR)
FP = config_loader.footprint(CAR)
NAV = yaml.safe_load(P.NAV2_YAML.read_text())
SIM = yaml.safe_load((P.SIM_DIR / "config" / "sim.yaml").read_text())


# ---------------------------------------------------------------- transform
def test_optical_point_on_the_axis_goes_to_base_link():
    cam = Pose3D(0.1, 0.0, 1.0, 0.0, math.radians(15.0), 0.0)     # pitch 15 deg down
    p = cloud_to_base(np.array([[0.0, 0.0, 2.0]]), cam)          # 2 m along the optical axis
    assert p[0] == pytest.approx([0.1 + 2.0 * math.cos(math.radians(15)), 0.0,
                                  1.0 - 2.0 * math.sin(math.radians(15))])


def test_optical_right_is_minus_y_and_down_is_minus_z():
    cam = Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    p = cloud_to_base(np.array([[0.5, 0.0, 1.0], [0.0, 0.3, 1.0]]), cam)
    assert p[0] == pytest.approx([1.0, -0.5, 1.0])               # right of the camera = right of the car
    assert p[1] == pytest.approx([1.0, 0.0, 0.7])                # below the axis


def test_car_yaml_camera_pose_floor_hit_distance():
    # the lowest ray of the 45 deg field of view hits the floor 1.35 m ahead of the camera
    d = blind_distance(CAM, 45.0, 0.0)
    assert d == pytest.approx(1.035 / math.tan(math.radians(37.5)), abs=1e-3)
    assert blind_distance(CAM, 45.0, 0.2) < d                    # a box top is visible closer


# ------------------------------------------------------------ height filter
def test_floor_and_overhead_points_are_dropped():
    p = DepthParams(min_height_m=0.05, max_height_m=0.6, max_range_m=3.0, floor_margin_per_m=0.0)
    pts = np.array([[1.0, 0.0, 0.0],      # floor
                    [1.0, 0.0, 0.04],     # still floor
                    [1.0, 0.0, 0.2],      # low box
                    [1.0, 0.0, 0.59],
                    [1.0, 0.0, 0.7],      # above the limit
                    [3.5, 0.0, 0.2],      # too far
                    [2.0, 2.0, 0.2]])     # 2.83 m: kept
    kept = height_range_filter(pts, p)
    assert kept[:, 2].tolist() == [0.2, 0.59, 0.2]
    assert kept[:, 0].tolist() == [1.0, 1.0, 2.0]


def test_floor_margin_grows_with_range():
    p = DepthParams(min_height_m=0.05, floor_margin_per_m=0.03)
    pts = np.array([[1.0, 0.0, 0.10], [3.0, 0.0, 0.10]])
    assert height_range_filter(pts, p)[:, 0].tolist() == [1.0]   # 0.10 > 0.08 at 1 m, < 0.14 at 3 m


# ---------------------------------------------------------------- downsample
def test_voxel_downsample_one_point_per_cell_and_noise_cells():
    xy = np.array([[0.01, 0.01], [0.02, 0.03], [0.03, 0.02],       # one cell, 3 points
                   [0.51, 0.51],                                    # a lone flying pixel
                   [1.01, 0.01], [1.02, 0.02]])
    out = voxel_downsample(xy, 0.05, 1)
    assert out.shape == (3, 2)
    out = voxel_downsample(xy, 0.05, 2)                             # the lone point is noise
    assert out.shape == (2, 2)
    assert voxel_downsample(np.zeros((0, 2)), 0.05).shape == (0, 2)
    cell = voxel_downsample(xy[:3], 0.05, 1)[0]
    assert cell == pytest.approx(xy[:3].mean(axis=0))


def test_obstacle_points_end_to_end_with_the_car_camera():
    # a 0.3 x 0.3 m, 0.2 m high box 1.5 m ahead, seen by the real mount: points on its top
    cam_pts = []
    for bx in np.arange(1.4, 1.7, 0.01):
        for by in np.arange(-0.15, 0.15, 0.01):
            cam_pts.append((bx, by, 0.2))
    base = np.array(cam_pts)
    # base_link -> optical for the test input
    rot = np.array(CAM.rotation())
    link = (base - np.array([CAM.x, CAM.y, CAM.z])) @ rot            # inverse rotation
    opt = np.stack([-link[:, 1], -link[:, 2], link[:, 0]], axis=1)   # y_o = -z_l, x_o = -y_l, z_o = x_l
    pts = obstacle_points(opt, CAM, DepthParams())
    assert pts.shape[0] > 30
    assert pts[:, 0].min() == pytest.approx(1.4, abs=0.05) and pts[:, 0].max() == pytest.approx(1.7, abs=0.05)
    assert abs(pts[:, 1]).max() <= 0.16


# --------------------------------------------------------------- map filter
def test_dynamic_points_walls_are_static_objects_are_not():
    occ = np.zeros((100, 100), dtype=np.int16)
    occ[:, 0] = occ[:, -1] = occ[0, :] = occ[-1, :] = 100
    mask = static_mask(occ, 0.05, 0.3)
    pts = np.array([[2.45, 0.0], [1.0, 0.0]])                       # wall 2.45 m ahead, box 1 m ahead
    assert list(dynamic_points(pts, (2.5, 2.5, 0.0), mask, (0.0, 0.0), 0.05)) == [False, True]


# ------------------------------------------------------------- cloud bytes
def test_parse_cloud_roundtrip_with_padding_and_stride():
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]], dtype="<f4")
    step = 16                                                          # x y z + 4 bytes padding
    buf = np.zeros((3, step), dtype=np.uint8)
    buf[:, :12] = np.frombuffer(encode_xyz(pts), dtype=np.uint8).reshape(3, 12)
    out = parse_cloud(buf.tobytes(), step, (0, 4, 8), 3)
    assert out == pytest.approx(pts)
    assert parse_cloud(buf.tobytes(), step, (0, 4, 8), 3, stride=2).shape == (2, 3)


def test_params_update_and_clamp():
    p = DepthParams()
    assert p.update({"max_height_m": 0.5, "bogus": 1, "min_height_m": None}) == ["max_height_m"]
    p.update({"min_height_m": 0.9})
    assert p.max_height_m > p.min_height_m


# --------------------------------------------------------- guard merge zone
BOX_AHEAD = np.array([[FP.x_front + 0.9 + 0.02 * i, 0.01 * i] for i in range(6)])   # 0.9 m past the nose


def test_merge_depth_off_is_the_lidar_decision():
    p = GuardParams()
    assert not p.depth_enabled
    assert decide_merged(np.zeros((0, 2)), BOX_AHEAD, p, FP.x_front, FP.half_width) == \
        decide(np.zeros((0, 2)), p, FP.x_front, FP.half_width)
    lid = np.array([[FP.x_front + 0.3, 0.0]] * 4)
    assert decide_merged(lid, None, p, FP.x_front, FP.half_width).state == "stop"


def test_merge_depth_stops_earlier_than_the_lidar_zone():
    p = GuardParams(depth_enabled=True)                               # depth stop at 1.1 m, LiDAR 0.6 m
    none = np.zeros((0, 2))
    assert decide(BOX_AHEAD, p, FP.x_front, FP.half_width).state == "slow"       # LiDAR rules: only slow
    d = decide_merged(none, BOX_AHEAD, p, FP.x_front, FP.half_width)
    assert d.state == "stop" and d.nearest_m == pytest.approx(0.9, abs=0.01)
    far = BOX_AHEAD + np.array([0.4, 0.0])                            # 1.3 m: between the depth stop and slow range
    assert decide_merged(none, far, p, FP.x_front, FP.half_width).state == "slow"
    behind = np.array([[-0.8, 0.0]] * 5)
    assert decide_merged(none, behind, p, FP.x_front, FP.half_width).state == "clear"   # front only
    # the worse one wins
    lid = np.array([[FP.x_front + 0.3, 0.0]] * 4)
    assert decide_merged(lid, far, p, FP.x_front, FP.half_width).state == "stop"


def test_depth_params_dict_flows_through_update_and_ui_json():
    p = GuardParams()
    changed = p.update({"depth_enabled": True, "depth_required": 1, "depth_max_height_m": 0.9,
                        "depth_min_height_m": 0.02, "depth_max_range_m": 2.5, "danger_range_m": 0.7})
    assert set(changed) == {"depth_enabled", "depth_required", "depth_max_height_m", "depth_min_height_m",
                            "depth_max_range_m", "danger_range_m"}
    d = p.as_dict()
    assert d["depth_enabled"] is True and d["depth_max_height_m"] == 0.9
    assert depth_params(p).danger_range_m == p.depth_danger_range_m
    assert depth_params(p).slow_range_m >= p.slow_range_m


def test_nav2_yaml_has_every_guard_param():
    g = NAV["safety_guard"]["ros__parameters"]
    for k in GuardParams().as_dict():
        assert k in g, k
    assert g["depth_enabled"] is False                                  # off until tested on the car
    assert "depth_obstacles" in NAV


# ------------------------------------------------------------- simulation
def _world(*boxes):
    return World("t", (-1, -3, 10, 3), list(boxes), [])


def _sim_points(world, pose=Pose2D(0.0, 0.0, 0.0)):
    d = SIM["depth"]
    rays = pixel_rays(int(d["width"]), int(d["height"]), d["hfov_deg"], d["vfov_deg"])
    return depth_points(world, pose, CAM, rays, d["range_min_m"], d["range_max_m"])


def test_sim_camera_sees_a_low_box_the_lidar_misses_and_the_floor():
    w = _world(Box(2.0, 0.0, 0.3, 0.3, 0.0, 0.2))
    ang = np.linspace(-0.3, 0.3, 61)
    lid = laser_scan_ranges(w, Pose2D(0.0, 0.0, 0.0), LASER, ang, {"range_max_m": 12.0, "range_min_m": 0.15}, None)
    assert np.isinf(lid).all()                                          # 0.2 m box under the 0.42 m plane
    pts = _sim_points(w)
    base = cloud_to_base(pts, CAM)
    box = base[(base[:, 2] > 0.05)]
    assert len(box) > 20
    assert box[:, 0].min() == pytest.approx(1.85, abs=0.06) and box[:, 2].max() == pytest.approx(0.2, abs=0.03)
    assert (np.abs(base[base[:, 2] < 0.03][:, 2]) < 0.03).all() and (base[:, 2] < 0.03).sum() > 100   # floor
    kept = obstacle_points(pts, CAM, DepthParams())
    assert len(kept) >= 5 and kept[:, 0].min() > 1.7


def test_sim_camera_blind_zone_and_tall_things():
    w = _world(Box(0.8, 0.0, 0.3, 0.3, 0.0, 0.2))                       # box 0.65..0.95 m ahead: inside the blind zone
    assert len(obstacle_points(_sim_points(w), CAM, DepthParams())) == 0
    tall = World("t", (-1, -3, 10, 3), [Box(3.0, 0.0, 0.4, 2.0, 0.0, 1.0)], [Cylinder(2.5, 1.0, 0.2, 1.8)])
    kept = obstacle_points(_sim_points(tall), CAM, DepthParams())
    assert len(kept) > 5                                               # the wall-like box / person legs
    assert (kept[:, 0] > 2.0).all()


def test_lidar_ignores_shapes_below_its_plane_but_not_tall_ones():
    w = _world(Box(2.0, 0.0, 0.3, 0.3, 0.0, 0.2), Box(4.0, 0.0, 0.3, 0.3, 0.0, 1.0))
    r = w.raycast(0.0, 0.0, np.array([0.0]), 12.0, z=0.42)
    assert r[0] == pytest.approx(3.85)
    assert w.raycast(0.0, 0.0, np.array([0.0]), 12.0)[0] == pytest.approx(1.85)   # no z = sees everything


def test_mission_extra_obstacles_are_in_the_world_but_not_the_map_and_count_for_clearance():
    base = World.load(str(P.SIM_DIR / "worlds" / "test_floor.yaml"))
    mission = yaml.safe_load((P.SIM_DIR / "missions" / "low_box.yaml").read_text())
    n0 = len(base.boxes)
    occ_before = base.occupancy(0.05).sum()
    assert base.add_extras(mission) == 1 and len(base.boxes) == n0 + 1
    box = mission["extra_obstacles"]["boxes"][0]
    assert box["height"] < LASER.z                                      # the LiDAR cannot see it
    from car_nav_sim.world import footprint_polygon
    on_it = footprint_polygon(box["center"][0], box["center"][1], 0.0, FP.x_front, FP.x_rear, FP.half_width)
    assert base.clearance(on_it) == 0.0                                 # touching = collision in the scorer
    assert base.occupancy(0.05).sum() > occ_before                     # (this World now has it) ...
    fresh = World.load(str(P.SIM_DIR / "worlds" / "test_floor.yaml"))   # ... the map comes from the file
    assert fresh.occupancy(0.05).sum() == occ_before
