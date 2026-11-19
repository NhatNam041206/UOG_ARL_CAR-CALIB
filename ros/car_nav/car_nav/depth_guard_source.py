"""Safety guard input from the depth camera: /camera/depth/points -> /depth_obstacles.

The LiDAR sees only a plane at 0.42 m; this node adds what is lower (boxes,
steps) or hangs over it (table tops) as 2D points, which safety_guard merges
with /scan_dynamic when its `depth_enabled` is true (see depth_obstacles.py
for the processing and the camera's blind zone).

  in   /camera/depth/points   sensor_msgs/PointCloud2 (param `topic`), in the camera's OPTICAL
                              frame (x right, y down, z forward: camera_depth_optical_frame,
                              what astra_camera publishes). The camera pose comes from car.yaml
                              (camera:), not from TF, so nothing here ever waits for a transform.
  out  /depth_obstacles       sensor_msgs/PointCloud2, frame base_link, live points + remembered ones
                              (depth_memory.py: obstacles that left the camera's view stay while the
                              car is next to them; the guard's depth_memory_enabled / _timeout_s), fields x y z (float32);
                              x, y = obstacle position, z = 0 (2D points, one per 5 cm cell).
                              One message per input frame, empty when nothing is in the way
                              (so the guard can tell "nothing" from "camera dead").
  out  /depth_obstacles/state JSON (String, 2 Hz): {"enabled", "n_cloud", "n_points", "n_dynamic",
                              "map": "ok|off|no_map|no_pose", "proc_ms", "age_s"}

* heights / range: min_height_m, max_height_m, max_range_m follow the safety guard's
  depth_* parameters (/safety_guard/state, i.e. the web UI); the rest are ROS parameters of this node
  (nav2_params.yaml depth_obstacles:).
* map_filter (default true): points that lie on a map wall (cell within tol_m of an occupied or
  unknown cell, after the scan-style alignment onto the walls) are dropped, so walls are not
  "people". No map or no pose: fallback "all" passes every point (safe, may stop near walls).
* {"depth_memory_clear": true} on /ui/safety_params forgets every remembered obstacle.
* Does nothing (no processing) while the guard's depth_enabled is false.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from . import config_loader
from .depth_memory import MemoryParams, ObstacleMemory
from .depth_obstacles import (DepthParams, encode_xyz, obstacle_points3, parse_cloud, points_to_polar)
from .idle_watch import IdleWatcher
from .dynamic_obstacles import align_points, dynamic_points, static_mask
from .timing import TimingStats


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DepthGuardSource(Node):
    def __init__(self) -> None:
        super().__init__("depth_obstacles")
        p = self.declare_parameter
        p("topic", "/camera/depth/points")
        p("optical", True)          # cloud in the optical frame (x right, y down, z forward)
        p("stride", 4)              # use every 4th point of the cloud (640x480 = 307k points)
        p("voxel_m", 0.05)
        p("min_cell_points", 2)
        p("min_range_m", 0.1)
        p("floor_margin_per_m", 0.03)
        p("map_filter", True)
        p("global_frame", "map")
        p("tol_m", 0.35)            # a point this close to a map wall is the wall
        p("align", True)
        p("fallback", "all")        # no map / no pose: pass "all" points (safe) or "none"
        p("max_tf_age_s", 0.5)
        p("min_period_s", 0.08)     # process at most ~12 frames / s
        p("odom_frame", "odom")
        p("hfov_deg", 58.0)
        p("vfov_deg", 45.0)
        p("memory_max_points", 600)
        p("memory_miss_frames", 3)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._params = DepthParams(voxel_m=float(g("voxel_m")), min_cell_points=int(g("min_cell_points")),
                                   min_range_m=float(g("min_range_m")),
                                   floor_margin_per_m=float(g("floor_margin_per_m")))
        cfg = config_loader.load()
        self._camera = config_loader.camera_pose(cfg)
        fp = config_loader.footprint(cfg)
        self._x_front = fp.x_front
        laser = config_loader.laser_pose(cfg)
        self._laser = (laser.x, laser.y, laser.yaw)
        self._lidar_z = laser.z
        self._scan = None
        self._scan_t = 0.0
        self._memory = ObstacleMemory(self._camera, fp.x_rear, MemoryParams(
            max_points=int(g("memory_max_points")), voxel_m=self._params.voxel_m,
            miss_frames=int(g("memory_miss_frames")), hfov_deg=float(g("hfov_deg")), vfov_deg=float(g("vfov_deg"))))
        self._mem_on, self._mem_timeout = True, 30.0
        self._protect = (self._x_front + 1.8, 0.5)      # (x_max, half width) in base_link: never expires by time
        self._enabled = True        # until the guard says otherwise (its depth_enabled)
        self._mask = self._thin = None
        self._origin = (0.0, 0.0)
        self._res = 0.05
        self._last_t = 0.0
        self._stat = {"n_cloud": 0, "n_points": 0, "n_dynamic": 0, "map": "off", "proc_ms": 0.0}
        self._last_frame = 0.0
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._timing = TimingStats(self.get_logger())
        self._idle = IdleWatcher(self, default_period_s=0.4)    # idle: ~2.5 frames / s (guard timeout 1 s)
        self._pub = self.create_publisher(PointCloud2, "/depth_obstacles", qos_profile_sensor_data)
        self._state_pub = self.create_publisher(String, "/depth_obstacles/state", 5)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(String, "/safety_guard/state", self._on_guard, 10)
        self.create_subscription(String, "/ui/safety_params", self._on_ui, 10)
        self.create_subscription(LaserScan, "/scan_nav", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, str(g("topic")), self._timing.wrap("depth.cb", self._on_cloud),
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self._publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"depth obstacles: {g('topic')} -> /depth_obstacles, camera {self._camera}, "
                               f"{self._params.as_dict()}")

    # ----------------------------------------------------------------- inputs
    def _on_map(self, msg: OccupancyGrid) -> None:
        t0 = time.perf_counter()
        occ = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self._res = float(msg.info.resolution)
        self._origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self._mask = static_mask(occ, self._res, float(self.get_parameter("tol_m").value))
        self._thin = static_mask(occ, self._res, 0.05, unknown_is_static=False)
        self.get_logger().info(f"map {msg.info.width}x{msg.info.height}: static mask "
                               f"in {(time.perf_counter() - t0) * 1e3:.0f} ms")

    def _on_guard(self, msg: String) -> None:
        """The guard's depth_* parameters (web UI, profiles) are the live settings."""
        try:
            gp = json.loads(msg.data).get("params") or {}
        except ValueError:
            return
        self._enabled = bool(gp.get("depth_enabled", True))
        self._mem_on = bool(gp.get("depth_memory_enabled", True))
        self._mem_timeout = float(gp.get("depth_memory_timeout_s", 30.0))
        self._memory.p.timeout_s = self._mem_timeout
        if "depth_danger_range_m" in gp and "slow_range_m" in gp:
            self._protect = (self._x_front + max(float(gp["depth_danger_range_m"]), float(gp["slow_range_m"])),
                             float(gp.get("slow_half_width_m", 0.5)))
        changed = self._params.update({"min_height_m": gp.get("depth_min_height_m"),
                                       "max_height_m": gp.get("depth_max_height_m"),
                                       "max_range_m": gp.get("depth_max_range_m")})
        if changed:
            self.get_logger().info(f"depth params changed: { {k: getattr(self._params, k) for k in changed} }")

    def _on_scan(self, msg: LaserScan) -> None:
        """The LiDAR (all obstacles above its plane): clears remembered TALL obstacles that left."""
        self._scan = (np.asarray(msg.ranges, dtype=float), msg.angle_min, msg.angle_increment)
        self._scan_t = time.monotonic()

    def _on_ui(self, msg: String) -> None:
        try:
            if json.loads(msg.data).get("depth_memory_clear"):
                self._memory.clear()
                self.get_logger().info("depth memory cleared (web UI)")
        except (ValueError, AttributeError):
            pass

    def _odom_pose(self):
        """base_link in the odom frame (latest TF, never waits); None if not available / old."""
        try:
            tf = self._tf.lookup_transform(str(self.get_parameter("odom_frame").value), "base_link", Time())
        except Exception:  # noqa: BLE001
            return None
        tr = tf.transform.translation
        return tr.x, tr.y, _yaw(tf.transform.rotation), Time.from_msg(tf.header.stamp)

    def _base_pose(self, stamp):
        """base_link in the map frame (latest TF, never waits); None if unknown or old."""
        try:
            tf = self._tf.lookup_transform(str(self.get_parameter("global_frame").value), "base_link", Time())
        except Exception:  # noqa: BLE001 - tf2 raises several types
            return None
        age = (Time.from_msg(stamp) - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if abs(age) > float(self.get_parameter("max_tf_age_s").value):
            return None
        tr = tf.transform.translation
        return tr.x, tr.y, _yaw(tf.transform.rotation)

    # ------------------------------------------------------------------ frame
    def _on_cloud(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        if not self._enabled or not self._idle.process_now(float(self.get_parameter("min_period_s").value)):
            return
        self._last_t = now
        t0 = time.perf_counter()
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in "xyz"):
            self.get_logger().error("cloud without x, y, z fields", throttle_duration_sec=5.0)
            return
        n = msg.width * msg.height
        raw = parse_cloud(bytes(msg.data), msg.point_step, (offs["x"], offs["y"], offs["z"]), n,
                          msg.is_bigendian, max(int(self.get_parameter("stride").value), 1))
        pts = obstacle_points3(raw, self._camera, self._params, bool(self.get_parameter("optical").value))
        n_found = int(pts.shape[0])
        hts = pts[:, 2]
        pts = pts[:, :2]
        mode = "off"
        if n_found and self.get_parameter("map_filter").value:
            pose = self._base_pose(msg.header.stamp) if self._mask is not None else None
            if pose is None:
                mode = "no_map" if self._mask is None else "no_pose"
                if self.get_parameter("fallback").value != "all":
                    pts, hts = pts[:0], hts[:0]
                self.get_logger().warn(f"depth: {mode}, passing {self.get_parameter('fallback').value} points",
                                       throttle_duration_sec=5.0)
            else:
                mode = "ok"
                if self.get_parameter("align").value:
                    r, a = points_to_polar(pts)
                    pose = align_points(r, a, pose, self._thin, self._origin, self._res)
                dyn = dynamic_points(pts, pose, self._mask, self._origin, self._res)
                pts, hts = pts[dyn], hts[dyn]
        n_live = int(pts.shape[0])
        if not self._mem_on:
            self._memory.clear()
        else:
            od = self._odom_pose()
            if od is not None:
                scan = self._scan if now - self._scan_t < 0.5 else None
                pts = self._memory.update(np.column_stack([pts, hts]), od[:3], now, self._params.min_height_m,
                                          self._params.max_range_m, self._protect, scan, self._laser,
                                          self._lidar_z)
            elif len(self._memory):
                self.get_logger().warn("depth memory: no odom pose, live points only",
                                       throttle_duration_sec=5.0)
        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "base_link"
        out.height, out.width = 1, int(pts.shape[0])
        out.fields = [PointField(name=nm, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, nm in enumerate("xyz")]
        out.is_bigendian, out.is_dense = False, True
        out.point_step, out.row_step = 12, 12 * out.width
        out.data = encode_xyz(np.hstack([pts, np.zeros((pts.shape[0], 1))]))
        self._pub.publish(out)
        self._last_frame = now
        self._stat = {"n_cloud": n, "n_points": n_found, "n_dynamic": n_live, "n_memory": len(self._memory), "map": mode,
                      "proc_ms": round((time.perf_counter() - t0) * 1e3, 1)}

    def _publish_state(self) -> None:
        d = dict(self._stat)
        d["enabled"] = self._enabled
        d["age_s"] = None if self._last_frame == 0.0 else round(time.monotonic() - self._last_frame, 2)
        self._state_pub.publish(String(data=json.dumps(d)))


def main() -> None:
    rclpy.init()
    node = DepthGuardSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
