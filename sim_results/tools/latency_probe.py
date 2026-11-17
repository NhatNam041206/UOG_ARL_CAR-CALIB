"""Throwaway measurement node for the sim (not part of the nav stack).

Run inside the sim container during a mission (profile_run.sh does this):
    python3 latency_probe.py /data/sim/probe.csv

Every 0.1 s of sim time it writes one CSV row:
  t                      sim time
  scan_age, scan_nav_age  now - stamp of the latest /scan and /scan_nav at arrival (s)
  rf2o_age, odom_age     same for /odom_rf2o and /odom (adaptive_odom output)
  tf_age                 now - stamp of the latest map->base_link (what pd_path_follower sees)
  cmd_dt                 time since the last /cmd_vel_nav (PD output)
  err_now_along/lat      estimate (latest TF) minus truth NOW, along/across the true heading
                         (+ along = estimate ahead of truth)
  err_stamp_along        estimate minus truth AT THE TF STAMP (accuracy without staleness)
  v_true                 true speed
"""
from __future__ import annotations

import bisect
import math
import sys

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Probe(Node):
    def __init__(self, out: str) -> None:
        super().__init__("latency_probe", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.f = open(out, "w", buffering=1)
        self.f.write("t,scan_age,scan_nav_age,rf2o_age,odom_age,tf_age,cmd_dt,err_now_along,err_now_lat,"
                     "err_stamp_along,v_true\n")
        self.age = {"scan": math.nan, "scan_nav": math.nan, "rf2o": math.nan, "odom": math.nan}
        self.last_cmd = None
        self.truth_t: list[float] = []
        self.truth: list[tuple[float, float, float, float]] = []
        # light: /tf read directly (no tf2 buffer), no scan subscriptions (the first
        # version cost ~0.8 core and distorted the CPU-limited runs)
        self.map_odom = None   # (x, y, yaw)
        self.odom_base = None  # (x, y, yaw, stamp)
        sub = self.create_subscription
        sub(TFMessage, "/tf", self._on_tf, 50)
        sub(Odometry, "/odom_rf2o", lambda m: self._age("rf2o", m.header), 20)
        sub(Odometry, "/odom", lambda m: self._age("odom", m.header), 20)
        sub(Twist, "/cmd_vel_nav", self._on_cmd, 20)
        sub(Odometry, "/sim/ground_truth", self._on_truth, 50)
        self.create_timer(0.1, self._tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self, k, h):
        self.age[k] = self.now() - _st(h)

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                q = t.transform.rotation
                self.map_odom = (t.transform.translation.x, t.transform.translation.y, _yaw(q))
            elif t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                q = t.transform.rotation
                self.odom_base = (t.transform.translation.x, t.transform.translation.y, _yaw(q), _st(t.header))

    def _on_cmd(self, _m):
        self.last_cmd = self.now()

    def _on_truth(self, m: Odometry):
        p = m.pose.pose
        self.truth_t.append(_st(m.header))
        self.truth.append((p.position.x, p.position.y, _yaw(p.orientation), math.hypot(
            m.twist.twist.linear.x, m.twist.twist.linear.y)))
        if len(self.truth_t) > 5000:
            del self.truth_t[:1000], self.truth[:1000]

    def _truth_at(self, t):
        i = bisect.bisect_left(self.truth_t, t)
        if not self.truth_t:
            return None
        i = min(max(i, 0), len(self.truth_t) - 1)
        return self.truth[i]

    def _tick(self):
        if not self.truth:
            return
        now = self.now()
        if self.map_odom is None or self.odom_base is None:
            return
        mx, my, mth = self.map_odom
        ox, oy, _oth, ts = self.odom_base
        c0, s0 = math.cos(mth), math.sin(mth)
        ex, ey = mx + c0 * ox - s0 * oy, my + s0 * ox + c0 * oy
        tx, ty, tth, v = self.truth[-1]
        c, s = math.cos(tth), math.sin(tth)
        along, lat = c * (ex - tx) + s * (ey - ty), -s * (ex - tx) + c * (ey - ty)
        ta = self._truth_at(ts)
        sa = math.nan
        if ta is not None:
            c2, s2 = math.cos(ta[2]), math.sin(ta[2])
            sa = c2 * (ex - ta[0]) + s2 * (ey - ta[1])
        cmd_dt = now - self.last_cmd if self.last_cmd is not None else math.nan
        a = self.age
        self.f.write(f"{now:.2f},{a['scan']:.3f},{a['scan_nav']:.3f},{a['rf2o']:.3f},{a['odom']:.3f},"
                     f"{now - ts:.3f},{cmd_dt:.3f},{along:.3f},{lat:.3f},{sa:.3f},{v:.3f}\n")


def main():
    rclpy.init()
    n = Probe(sys.argv[1] if len(sys.argv) > 1 else "/data/sim/probe.csv")
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    try:
        ex.spin()
    except BaseException:  # noqa: BLE001 - killed at the end of the run
        pass


if __name__ == "__main__":
    main()
