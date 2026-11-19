"""Health check rules (ros/car_nav/car_nav/health_checks.py)."""
import math

import numpy as np

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav import health_checks as H
from car_nav.depth_obstacles import OPTICAL_TO_LINK
from car_nav.geometry import Footprint, Pose3D
from car_nav.ugv_link import UgvConfig

FP = Footprint(0.35, -0.35, 0.30)
LASER = Pose3D(0, 0, 0.24)


def _scan(n=720, r=3.0, inside=0):
    ranges = np.full(n, r)
    ranges[:inside] = 0.3            # on the car's own body
    return ranges


def _frames(ranges, k=8):
    st = H.lidar_frame_stats(ranges, -math.pi, 2 * math.pi / len(ranges), 0.15, 12.0, LASER, FP, 0.03, [])
    return [st] * k


def _stamps(hz, n):
    return [i / hz for i in range(n)]


def test_rate_hz():
    assert abs(H.rate_hz(_stamps(7.0, 20)) - 7.0) < 1e-9
    assert H.rate_hz([1.0]) is None


def test_lidar_pass_and_fail_rules():
    ok = H.eval_lidar(_stamps(7, 8), _frames(_scan()), 12.0)
    assert ok["status"] == "pass"
    assert H.eval_lidar(_stamps(3, 8), _frames(_scan()), 12.0)["status"] == "fail"       # too slow
    assert H.eval_lidar(_stamps(7, 8), _frames(_scan(r=20.0)), 12.0)["status"] == "fail"  # nothing valid
    bad = H.eval_lidar(_stamps(7, 8), _frames(_scan(inside=40)), 12.0)                    # body in the scan
    assert bad["status"] == "fail" and "inside the footprint" in bad["detail"]
    assert H.eval_lidar([], [], 12.0)["status"] == "fail"


def test_lidar_masked_sector_is_allowed():
    r = _scan(inside=40)
    # the 40 first beams point to bearings -180 .. -160 deg
    st = H.lidar_frame_stats(r, -math.pi, 2 * math.pi / 720, 0.15, 12.0, LASER, FP, 0.03, [[-181, -159]])
    assert st["inside_footprint"] == 0


def _floor_cloud(cam_true, n=6000, seed=2):
    """Points of a flat floor seen from the true camera pose, in the optical frame."""
    rng = np.random.default_rng(seed)
    base = np.column_stack([rng.uniform(0.6, 3.5, n), rng.uniform(-1.5, 1.5, n), np.zeros(n)])
    rot = np.array(cam_true.rotation())
    link = (base - np.array([cam_true.x, cam_true.y, cam_true.z])) @ rot
    return link @ OPTICAL_TO_LINK            # inverse of optical -> link


def test_floor_fit_consistent_pose_passes():
    from car_nav.depth_obstacles import cloud_to_base
    cam = Pose3D(0, 0, 1.2, 0, math.radians(15), 0)
    pts = cloud_to_base(_floor_cloud(cam), cam)
    fl = H.fit_floor(pts)
    assert abs(fl["z_at_centroid"]) < 0.01 and abs(fl["slope_x_deg"]) < 0.5
    frames = [{"n_valid": 4000, "n_total": 4800, "organized": True}] * 5
    assert H.eval_depth(_stamps(10, 5), frames, cam, fl)["status"] == "pass"


def test_floor_fit_wrong_height_and_pitch_fail_and_hint_points_right():
    from car_nav.depth_obstacles import cloud_to_base
    true = Pose3D(0, 0, 1.2, 0, math.radians(15), 0)
    cfg = Pose3D(0, 0, 1.0, 0, math.radians(8), 0)          # configured: 20 cm too low, 7 deg too flat
    fl = H.fit_floor(cloud_to_base(_floor_cloud(true), cfg))
    res = H.eval_depth(_stamps(10, 5), [{"n_valid": 4000, "n_total": 4800, "organized": True}] * 5, cfg, fl)
    assert res["status"] == "fail"
    assert abs(res["hint"]["camera_pitch_deg"] - 15) < 3.0, res["hint"]
    assert abs(res["hint"]["camera_z_m"] - 1.2) < 0.2, res["hint"]


def test_depth_no_floor_or_no_data_fails():
    cam = Pose3D(0, 0, 1.2, 0, math.radians(15), 0)
    assert H.eval_depth([], [], cam, None)["status"] == "fail"
    assert H.eval_depth(_stamps(10, 5), [{"n_valid": 4000, "n_total": 4800, "organized": True}] * 5, cam, None)["status"] == "fail"
    wall = np.column_stack([np.full(500, 2.0), np.random.default_rng(1).uniform(-1, 1, 500), np.random.default_rng(2).uniform(0.7, 1.5, 500)])
    assert H.fit_floor(wall) is None


def test_imu():
    rng = np.random.default_rng(0)
    s = [(i * 0.1, 0.1 * rng.normal(), 0.1 * rng.normal(), 9.8 + 0.05 * rng.normal(), 0.01 * rng.normal(),
          0.01 * rng.normal(), 0.01 * rng.normal()) for i in range(60)]
    assert H.eval_imu(s)["status"] == "pass"
    moving = [(t, ax, ay, az, gx, gy, 0.8) for t, ax, ay, az, gx, gy, gz in s]
    assert H.eval_imu(moving)["status"] == "fail"
    slow = [(t * 3, *r) for t, *r in s]
    assert H.eval_imu(slow)["status"] == "fail"
    zero_g = [(t, 0, 0, 0.1, gx, gy, gz) for t, ax, ay, az, gx, gy, gz in s]
    assert H.eval_imu(zero_g)["status"] == "fail"
    assert H.eval_imu([])["status"] == "fail"


def test_color_needs_observer():
    st = _stamps(15, 20)
    assert H.eval_color(st, (640, 480), "yes")["status"] == "pass"
    assert H.eval_color(st, (640, 480), None)["status"] == "fail"
    assert H.eval_color(st, (640, 480), "no")["status"] == "fail"
    assert H.eval_color([], None, "yes")["status"] == "fail"


def test_wheel_steps_and_echo():
    steps = H.wheel_steps(0.15)
    assert [s["id"] for s in steps] == ["left_fwd", "left_back", "right_fwd", "right_back", "both_fwd", "spin_left"]
    cfg = UgvConfig(units_per_mps=250, max_units=200)
    assert H.expected_units(0.15, 0.0, cfg) == (38, 0)
    swapped = UgvConfig(units_per_mps=250, max_units=200, swap_sides=True)
    assert H.expected_units(0.15, 0.0, swapped) == (0, 38)
    assert H.eval_echo((38, 0), [(0, 0), (37, 0)]).ok
    assert not H.eval_echo((38, 0), [(0, 0)]).ok
    assert not H.eval_echo((38, 0), []).ok
    assert H.eval_wheel_step(steps[0], H.eval_echo((38, 0), [(38, 0)]), "yes")["status"] == "pass"
    assert H.eval_wheel_step(steps[0], H.eval_echo((38, 0), [(38, 0)]), "no")["status"] == "fail"
    assert H.eval_wheel_step(steps[0], H.eval_echo((38, 0), [(38, 0)]), None)["status"] == "fail"
    assert H.eval_wheel_step(steps[0], H.eval_echo((38, 0), []), "yes")["status"] == "fail"


def _drive(v, t0=0.0, t1=5.0, hz=10):
    odom = [(t0 + i / hz, v * (t0 + i / hz), 0.0) for i in range(int((t1 - t0) * hz) + 1)]
    yaws = [(t, 0.0) for t, _, _ in odom]
    front = [(t, 4.0 - x) for t, x, _ in odom]
    return odom, front, yaws


def test_speed_result_ratio_and_proposal():
    odom, front, yaws = _drive(0.16)                        # the car really does 0.16 of the 0.2 commanded
    r = H.speed_result(0.2, 0.0, (1.5, 4.5), odom, front, yaws, 0.8, 250.0, None, False, "yes")
    assert r["status"] == "pass" and abs(r["real_mps"] - 0.16) < 1e-6 and r["source"] == "wall ahead"
    assert abs(r["units_per_mps_proposed"] - 312.5) < 0.1   # 250 / 0.8
    # no wall: falls back to rf2o
    r2 = H.speed_result(0.2, 0.0, (1.5, 4.5), odom, [], yaws, 0.8, 250.0, None, False, "yes")
    assert r2["source"] == "rf2o odometry" and abs(r2["real_mps"] - 0.16) < 1e-6
    # tape says the LiDAR total was 25 % short: speed scaled up
    r3 = H.speed_result(0.2, 0.0, (1.5, 4.5), odom, front, yaws, 0.8, 250.0, 1.0, False, "yes")
    assert abs(r3["real_mps"] - 0.2) < 1e-6 and r3["status"] == "pass"
    assert H.speed_result(0.2, 0.0, (1.5, 4.5), odom, front, yaws, 0.8, 250.0, 1.6, False, "yes")["status"] == "fail"  # tape 2x


def test_speed_result_fail_cases():
    odom, front, yaws = _drive(0.02)
    assert H.speed_result(0.2, 0, (1.5, 4.5), odom, front, yaws, 0.1, 250.0, None, False, "yes")["status"] == "fail"   # barely moved
    odom, front, yaws = _drive(0.2)
    assert H.speed_result(0.2, 0, (1.5, 4.5), odom, front, yaws, 0.9, 250.0, None, True, "yes")["status"] == "fail"    # guard stopped it
    assert H.speed_result(0.2, 0, (1.5, 4.5), odom, front, yaws, 0.9, 250.0, None, False, None)["status"] == "fail"    # nobody confirmed
    assert H.speed_result(0.2, 0, (1.5, 4.5), [], [], [], None, 250.0, None, False, "yes")["status"] == "fail"
    turning = [(t, 0.1 * t) for t, _, _ in odom]
    assert H.speed_result(0.2, 0, (1.5, 4.5), odom, front, turning, 0.9, 250.0, None, False, "yes")["status"] == "fail"


def test_spin_result_unwraps_yaw():
    yaws = [(i * 0.1, ((0.4 * i * 0.1 + math.pi) % (2 * math.pi)) - math.pi) for i in range(60)]
    r = H.spin_result(0.5, (1.0, 5.0), yaws, 0.52)
    assert r["status"] == "pass" and abs(r["real_rps"] - 0.4) < 1e-3 and abs(r["yaw_efficiency"] - 0.8) < 1e-3
    assert H.spin_result(0.5, (1.0, 5.0), [(0, 0.0), (6, 0.0)], 0.52)["status"] == "fail"


def test_overall_and_store(tmp_path):
    assert H.overall({"a": {"status": "pass"}, "b": {"status": "pass"}}) == "pass"
    assert H.overall({"a": {"status": "pass"}, "b": {"status": "fail"}}) == "fail"
    assert H.overall({}) == "fail"
    st = H.HealthStore(str(tmp_path))
    rid = st.save({"started": "2026-11-19 10:00:00", "overall": "pass", "results": {"imu": {"status": "pass"}}})
    rid2 = st.save({"started": "2026-11-19 10:00:00", "overall": "fail", "results": {}})
    assert rid != rid2 and st.get(rid)["id"] == rid
    assert {r["id"] for r in st.list()} == {rid, rid2}
    assert st.last() is not None and st.delete(rid) and st.get(rid) is None
    assert st.get("../../x") is None


def test_clearance_and_passed_today():
    n = 720
    r = np.full(n, 3.0)
    ang = -math.pi + np.arange(n) * 2 * math.pi / n
    r[np.argmin(np.abs(ang))] = 0.5                       # something 0.5 m behind the centre (rear edge at 0.35)
    c = H.clearance_outside(r, -math.pi, 2 * math.pi / n, 0.15, 12.0, LASER, FP)
    assert abs(c - 0.15) < 0.02
    assert H.clearance_outside(np.full(n, math.inf), -math.pi, 2 * math.pi / n, 0.15, 12.0, LASER, FP) is None
    assert abs(H.min_radius_outside(r, -math.pi, 2 * math.pi / n, 0.15, 12.0, LASER, FP) - 0.5) < 0.02
    now = 1_800_000_000.0
    assert H.passed_today({"overall": "pass", "started_ts": now - 60}, now)
    assert not H.passed_today({"overall": "fail", "started_ts": now - 60}, now)
    assert not H.passed_today({"overall": "pass", "started_ts": now - 3 * 86400}, now)
    assert not H.passed_today(None, now)
