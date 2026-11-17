"""Direct map -> odom -> base_link lookup (ros/car_nav/car_nav/tf_chain.py).

The tf2 comparison runs where tf2_ros is importable (inside the sim image):
    python3 -m pytest -q tests/test_car_nav_tf_chain.py
"""
import math

import numpy as np
import pytest

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.tf_chain import PlanarTfChain

NS = 1_000_000_000


def test_latest_common_time_interpolates_and_composes():
    c = PlanarTfChain()
    assert c.latest() is None
    c.add("map", "odom", 10 * NS, 1.0, 2.0, 0.0)
    c.add("map", "odom", 11 * NS, 2.0, 2.0, math.pi / 2)
    c.add("odom", "base_link", int(10.5 * NS), 1.0, 0.0, 0.0)
    t, x, y, th = c.latest()
    assert t == int(10.5 * NS)
    # map->odom halfway: (1.5, 2.0, 45 deg); base 1 m ahead of the odom origin along 45 deg
    assert (x, y, th) == pytest.approx((1.5 + math.sqrt(0.5), 2.0 + math.sqrt(0.5), math.pi / 4))
    assert not c.add("base_link", "laser", NS, 0, 0, 0)


def test_yaw_interpolates_the_short_way_and_out_of_range_is_none():
    c = PlanarTfChain()
    c.add("map", "odom", 0, 0.0, 0.0, math.radians(170))
    c.add("map", "odom", 2 * NS, 0.0, 0.0, math.radians(-170))
    c.add("odom", "base_link", NS, 0.0, 0.0, 0.0)
    assert abs(abs(c.latest()[3]) - math.pi) < 1e-9          # 180 deg, not 0
    c2 = PlanarTfChain()
    c2.add("map", "odom", 5 * NS, 0, 0, 0)
    c2.add("odom", "base_link", 4 * NS, 0, 0, 0)               # older than all map->odom data
    assert c2.latest() is None


def test_cache_drops_old_samples():
    c = PlanarTfChain(cache_s=1.0)
    for i in range(50):
        c.add("map", "odom", i * NS // 10, float(i), 0.0, 0.0)
    assert len(c._a.t) <= 11


def test_same_answer_as_tf2_buffer():
    tf2_ros = pytest.importorskip("tf2_ros")
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time

    rng = np.random.default_rng(3)
    checked = 0
    for _ in range(30):
        buf = tf2_ros.Buffer()
        c = PlanarTfChain()
        t_mo = 1.0 + rng.uniform(0, 0.3)     # map->odom future-dated like pose_gate / AMCL
        t_ob = 1.0
        for _k in range(int(rng.integers(5, 40))):
            if rng.random() < 0.4:
                t_mo += rng.uniform(0.02, 0.2)
                ev = ("map", "odom", t_mo, rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3.1, 3.1))
            else:
                t_ob += rng.uniform(0.05, 0.3)
                ev = ("odom", "base_link", t_ob, rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(-3.1, 3.1))
            parent, child, t, x, y, yaw = ev
            ts = TransformStamped()
            ns = int(round(t * 1e9))
            ts.header.stamp.sec, ts.header.stamp.nanosec = ns // NS, ns % NS
            ts.header.frame_id, ts.child_frame_id = parent, child
            ts.transform.translation.x, ts.transform.translation.y = x, y
            ts.transform.rotation.z, ts.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            buf.set_transform(ts, "test")
            c.add(parent, child, ns, x, y, yaw)
            try:
                ref = buf.lookup_transform("map", "base_link", Time())
            except Exception:  # noqa: BLE001 - tf2 raises several types
                ref = None
            got = c.latest()
            if ref is None:
                assert got is None
                continue
            assert got is not None
            rt = ref.header.stamp.sec * NS + ref.header.stamp.nanosec
            q = ref.transform.rotation
            ryaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            assert got[0] == rt
            assert got[1] == pytest.approx(ref.transform.translation.x, abs=1e-9)
            assert got[2] == pytest.approx(ref.transform.translation.y, abs=1e-9)
            assert math.cos(got[3] - ryaw) == pytest.approx(1.0, abs=1e-12)
            checked += 1
    assert checked > 100
