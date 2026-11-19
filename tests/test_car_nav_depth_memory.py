"""Depth obstacle memory (car_nav/depth_memory.py), driven with the simulated camera."""
import numpy as np
import pytest
import yaml

from tests import _car_nav_paths as P
from car_nav import config_loader
from car_nav.depth_memory import MemoryParams, ObstacleMemory, to_base, to_odom
from car_nav.depth_obstacles import DepthParams, blind_distance, obstacle_points
from car_nav.safety_zone import GuardParams, decide
from car_nav_sim.actuator_model import Pose2D
from car_nav_sim.depth_model import depth_points, pixel_rays
from car_nav_sim.world import Box, World

CAR = yaml.safe_load(P.CAR_YAML.read_text())
SIM = yaml.safe_load((P.SIM_DIR / "config" / "sim.yaml").read_text())["depth"]
CAM = config_loader.camera_pose(CAR)
FP = config_loader.footprint(CAR)
RAYS = pixel_rays(int(SIM["width"]), int(SIM["height"]), SIM["hfov_deg"], SIM["vfov_deg"])
DP = DepthParams()
H = 0.15


def _live(world, x):
    pts = depth_points(world, Pose2D(x, 0.0, 0.0), CAM, RAYS, SIM["range_min_m"], SIM["range_max_m"])
    return obstacle_points(pts, CAM, DP)


def _drive(world, mem, x0, x1, step=0.05, t0=0.0, protect=(FP.x_front + 1.8, 0.5)):
    out, x, t = None, x0, t0
    while x <= x1 + 1e-9:
        out = mem.update(_live(world, x), (x, 0.0, 0.0), t, DP.min_height_m, DP.max_range_m, protect)
        x, t = x + step, t + 0.1
    return out, t


def _box_world(face):
    return World("t", (-1, -3, 12, 3), [Box(face + 0.15, 0.0, 0.3, 0.3, 0.0, H)], [])


def test_box_stays_known_after_it_leaves_the_view_and_the_guard_keeps_stopping():
    face = 4.0
    w = _box_world(face)
    mem = ObstacleMemory(CAM, FP.x_rear)
    p = GuardParams(depth_enabled=True)
    # car creeps up until the box is well inside the camera's blind zone
    x_end = face - FP.x_front - 0.25
    assert not len(_live(w, x_end))                                  # the live camera sees nothing any more
    out, _ = _drive(w, mem, 0.0, x_end)
    assert len(out) >= 5
    box_x = face - x_end
    assert out[:, 0].min() == pytest.approx(box_x, abs=0.25)
    assert decide(out, p, FP.x_front, FP.half_width).state == "stop"   # stays stopped (no flicker)


def test_stop_state_does_not_flicker_over_the_whole_approach():
    face = 4.0
    w = _box_world(face)
    mem = ObstacleMemory(CAM, FP.x_rear)
    p = GuardParams(depth_enabled=True)
    stopped = False
    x = 0.0
    while x < face - FP.x_front - 0.1:
        out = mem.update(_live(w, x), (x, 0.0, 0.0), x * 10, DP.min_height_m, DP.max_range_m)
        st = decide(out, p, FP.x_front, FP.half_width).state
        stopped = stopped or st == "stop"
        if stopped:
            assert st == "stop"            # once stopped it never goes back to clear / slow
            break                          # (the car stands still from here on)
        x += 0.03
    assert stopped


def test_forgets_when_the_camera_sees_free_space_where_it_was():
    face = 4.0
    mem = ObstacleMemory(CAM, FP.x_rear)
    _drive(_box_world(face), mem, 0.0, 2.0)
    assert len(mem) > 5
    empty = World("t", (-1, -3, 12, 3), [], [])                      # the box was taken away
    for k in range(6):
        out = mem.update(_live(empty, 2.0), (2.0, 0.0, 0.0), 30.0 + k, DP.min_height_m, DP.max_range_m)
    assert len(out) == 0


def test_a_hidden_point_is_not_forgotten_when_something_nearer_blocks_the_view():
    mem = ObstacleMemory(CAM, FP.x_rear)
    behind_wall = np.array([[2.5, 0.0]] * 1)
    mem.update(behind_wall, (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
    wall = np.array([[1.8, 0.0], [1.8, 0.05], [1.8, -0.05]])         # nearer, same bearing
    for k in range(6):
        out = mem.update(wall, (0.0, 0.0, 0.0), 1.0 + k, 0.05, 3.0)
    assert (np.abs(out[:, 0] - 2.5) < 0.01).any()


def test_points_outside_the_field_of_view_or_blind_are_kept():
    mem = ObstacleMemory(CAM, FP.x_rear)
    side = np.array([[1.5, 1.5]])                                     # 45 deg: outside the 58 deg view
    near = np.array([[blind_distance(CAM, 45.0, 0.05) - 0.3, 0.0]])
    mem.update(np.vstack([side, near]), (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
    for k in range(10):
        out = mem.update(np.zeros((0, 2)), (0.0, 0.0, 0.0), 1.0 + k, 0.05, 3.0)
    assert len(out) == 2


def test_behind_the_car_is_forgotten_and_motion_is_compensated():
    mem = ObstacleMemory(CAM, FP.x_rear)
    mem.update(np.array([[1.0, 0.0]]), (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
    out = mem.update(np.zeros((0, 2)), (0.4, 0.0, 0.0), 1.0, 0.05, 3.0)     # car moved 0.4 m
    assert out[0] == pytest.approx([0.6, 0.0])
    out = mem.update(np.zeros((0, 2)), (1.0 + abs(FP.x_rear) + 0.1, 0.0, 0.0), 2.0, 0.05, 3.0)
    assert len(out) == 0
    # a turn: a point 1 m ahead is 1 m to the right after a left turn of 90 deg
    mem.update(np.array([[1.0, 0.0]]), (0.0, 0.0, 0.0), 3.0, 0.05, 3.0)
    out = mem.update(np.zeros((0, 2)), (0.0, 0.0, np.pi / 2), 4.0, 0.05, 3.0)
    assert out[0] == pytest.approx([0.0, -1.0], abs=1e-9)


def test_timeout_except_in_the_protected_zone_in_front_and_clear_and_cap():
    p = MemoryParams(timeout_s=10.0, max_points=5)
    mem = ObstacleMemory(CAM, FP.x_rear, p)
    near = np.array([[blind_distance(CAM, 45.0, 0.05) - 0.5, 0.0]])    # blind zone, in front
    side = np.array([[0.2, 0.9]])                                      # blind-ish, beside the car
    mem.update(np.vstack([near, side]), (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
    out = mem.update(np.zeros((0, 2)), (0.0, 0.0, 0.0), 11.0, 0.05, 3.0, protect=(2.0, 0.5))
    assert len(out) == 1 and out[0, 1] == pytest.approx(0.0)            # the one in front never expires
    mem.clear()
    assert len(mem) == 0
    many = np.stack([np.linspace(0.2, 2.0, 20), np.zeros(20)], axis=1)
    out = mem.update(many, (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
    assert len(out) == 5 and out[:, 0].max() < 0.8                      # the nearest ones are kept


def test_odom_base_roundtrip_and_voxel_dedupe():
    pose = (1.0, 2.0, 0.7)
    pts = np.array([[1.0, 0.5], [-0.3, 0.2]])
    assert to_base(to_odom(pts, pose), pose) == pytest.approx(pts)
    mem = ObstacleMemory(CAM, FP.x_rear)
    out = mem.update(np.array([[1.0, 0.0], [1.01, 0.01], [1.2, 0.0]]), (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
    assert len(out) == 2


def test_params_in_yaml_and_guard():
    nav = yaml.safe_load(P.NAV2_YAML.read_text())
    g = nav["safety_guard"]["ros__parameters"]
    assert g["depth_memory_enabled"] is True and g["depth_memory_timeout_s"] == 30.0
    for k in ("odom_frame", "hfov_deg", "vfov_deg", "memory_max_points", "memory_miss_frames"):
        assert k in nav["depth_obstacles"]["ros__parameters"]
    assert GuardParams().update({"depth_memory_enabled": False, "depth_memory_timeout_s": 5}) == \
        ["depth_memory_enabled", "depth_memory_timeout_s"]


def _scan(hit_r=None, n=720):
    """A full-circle LiDAR scan: nothing (inf) except one beam straight ahead at hit_r."""
    r = np.full(n, np.inf)
    if hit_r is not None:
        r[n // 2] = hit_r            # angle_min = -pi: index n/2 is 0 rad
    return (r, -np.pi, 2 * np.pi / n)


def test_lidar_clears_a_tall_obstacle_that_left_but_never_a_low_one():
    near = 0.9
    for height, expect_left in ((1.0, 0), (H, 1)):       # person legs (taller than the LiDAR plane) / low box
        mem = ObstacleMemory(CAM, FP.x_rear)
        mem.update(np.array([[near, 0.0, height]]), (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
        for k in range(5):                               # beam straight ahead returns nothing
            out = mem.update(np.zeros((0, 2)), (0.0, 0.0, 0.0), 1.0 + k, 0.05, 3.0, None, _scan(None),
                             (0.0, 0.0, 0.0), 0.24)
        assert len(out) == expect_left
    # something still there: the beam hits at the point's range -> kept
    mem = ObstacleMemory(CAM, FP.x_rear)
    mem.update(np.array([[near, 0.0, 1.0]]), (0.0, 0.0, 0.0), 0.0, 0.05, 3.0)
    for k in range(5):
        out = mem.update(np.zeros((0, 2)), (0.0, 0.0, 0.0), 1.0 + k, 0.05, 3.0, None, _scan(near + 0.05),
                         (0.0, 0.0, 0.0), 0.24)
    assert len(out) == 1
