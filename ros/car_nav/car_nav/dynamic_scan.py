"""Safety guard input: /scan_nav -> /scan_dynamic (only points that are not on the map).

The collision monitor's PersonStop / PersonSlow zones (nav2_params.yaml)
read /scan_dynamic, so the car stops for a person crossing in front of it
at a larger distance than for a wall it is driving past. Walls stay
handled by the normal bumper (VelocityStop) and the approach zone.

* map: /map (map_server or slam_toolbox), cells within tol_m of an occupied
  (or unknown) cell are static (dynamic_obstacles.static_mask)
* pose: TF map -> scan frame at the scan stamp (latest if not available),
  then slid over the map walls (align_to_map, +-0.45 m / +-7 deg): with a
  0.3-0.5 m pose error walls looked like people and the car waited forever
* fallback, when there is no map or no pose: every beam is passed through
  (fallback:=all, default: safe, the car may stop near walls) or none.

It always publishes, because the collision monitor stops the car when a
source goes silent.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .idle_watch import IdleWatcher
from .dynamic_obstacles import align_to_map, dynamic_beams, static_mask
from .timing import TimingStats


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DynamicScan(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_scan")
        p = self.declare_parameter
        p("global_frame", "map")
        p("tol_m", 0.3)             # a point this close to a map wall is the wall
        p("max_range_m", 2.0)       # only points this close matter (zones reach 1.7 m); a heading error moves far points most
        p("fallback", "all")        # no map / no pose: "all" beams (safe) or "none"
        p("max_tf_age_s", 0.5)
        p("align", True)            # slide the scan onto the map walls before classifying
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._frame = str(g("global_frame"))
        self._tol = float(g("tol_m"))
        self._mask = None
        self._origin = (0.0, 0.0)
        self._res = 0.05
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._timing = TimingStats(self.get_logger())
        # While the car is idle only one scan in idle_period_s is aligned and classified (the guard
        # still gets data well inside its 1 s timeout); the first non-zero command or a goal wakes it
        # and the safety guard waits for a scan newer than that command (idle_logic.resume_gate).
        self._idle = IdleWatcher(self, default_period_s=0.4)
        self._pub = self.create_publisher(LaserScan, "/scan_dynamic", qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan_nav", self._timing.wrap("dyn.scan_cb", self._on_scan),
                                 qos_profile_sensor_data)
        self._n_dyn_log = 0.0
        self.get_logger().info(f"dynamic scan: points > {self._tol} m from map walls -> /scan_dynamic")

    def _on_map(self, msg: OccupancyGrid) -> None:
        t0 = time.perf_counter()
        occ = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self._res = float(msg.info.resolution)
        self._origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self._mask = static_mask(occ, self._res, self._tol)
        self._thin = static_mask(occ, self._res, 0.05, unknown_is_static=False)
        self.get_logger().info(f"map {msg.info.width}x{msg.info.height} @ {self._res} m: static mask "
                               f"in {(time.perf_counter() - t0) * 1e3:.0f} ms")

    def _sensor_pose(self, msg: LaserScan):
        """Scan frame in the map at the scan stamp, else the latest one if it is fresh.
        Never blocks: a waiting lookup held up the callback ~60-130 ms per scan (sim)."""
        tf = None
        try:
            tf = self._tf.lookup_transform(self._frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:  # noqa: BLE001 - tf2 raises several types
            try:
                tf = self._tf.lookup_transform(self._frame, msg.header.frame_id, Time())
            except Exception:  # noqa: BLE001
                return None
            age = (Time.from_msg(msg.header.stamp) - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            if abs(age) > float(self.get_parameter("max_tf_age_s").value):
                return None
        tr = tf.transform.translation
        return tr.x, tr.y, _yaw(tf.transform.rotation)

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._idle.process_now():
            return
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max, out.angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        pose = self._sensor_pose(msg) if self._mask is not None else None
        if pose is None:
            if self.get_parameter("fallback").value == "all":
                out.ranges = list(msg.ranges)
            else:
                out.ranges = [math.inf] * len(msg.ranges)
            self.get_logger().warn("no map or no pose: passing "
                                   f"{self.get_parameter('fallback').value} beams to the safety guard",
                                   throttle_duration_sec=5.0)
            self._pub.publish(out)
            return
        if self.get_parameter("align").value:
            pose = align_to_map(msg.ranges, msg.angle_min, msg.angle_increment, pose, self._thin,
                                self._origin, self._res)
        dyn = dynamic_beams(msg.ranges, msg.angle_min, msg.angle_increment, pose, self._mask, self._origin,
                            self._res, float(self.get_parameter("max_range_m").value))
        r = np.asarray(msg.ranges, dtype=float)
        out.ranges = np.where(dyn, r, np.inf).astype(np.float32)
        n = int(dyn.sum())
        if n and time.monotonic() - self._n_dyn_log > 2.0:
            self._n_dyn_log = time.monotonic()
            self.get_logger().info(f"{n} scan points off the map (person / new obstacle)")
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DynamicScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
