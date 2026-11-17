"""Web UI backend: serves the page (HTTP) and packs what it shows into two JSON topics.

The browser talks to ROS through rosbridge (WebSocket, port 9090, started
by nav.launch.py ui:=true). To keep the page simple and the traffic small,
this node does the ROS-side work:

* /ui/map    (std_msgs/String, every 3 s): the occupancy map, run-length
             encoded: {"w","h","res","ox","oy","rle":[value, count, ...]}
             values 0 = free, 100 = occupied, -1 = unknown
* /ui/state  (std_msgs/String, 5 Hz): car pose (TF map -> base_link), LiDAR
             points in the map frame (/scan_nav, and /scan_dynamic = not on
             the map), the global plan, the safety guard state and zones.

The page publishes /initialpose, /goal_pose, /ui/estop and /ui/safety_params
itself (see web/index.html). HTTP: http://<car>:<http_port>/ (default 8080).
"""
from __future__ import annotations

import functools
import http.server
import json
import math
import os
import threading

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from . import config_loader
from .ui_pack import rle


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_to_map(msg: LaserScan, pose: tuple[float, float, float], max_range: float = 12.0, step: int = 2):
    r = np.asarray(msg.ranges, dtype=float)[::step]
    a = msg.angle_min + np.arange(0, len(msg.ranges), step) * msg.angle_increment + pose[2]
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r > 0.05) & (r <= max_range)
    x = pose[0] + r[ok] * np.cos(a[ok])
    y = pose[1] + r[ok] * np.sin(a[ok])
    return np.round(np.stack((x, y), axis=1), 2).tolist()


class UiBridge(Node):
    def __init__(self) -> None:
        super().__init__("ui_bridge")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("rosbridge_port", 9090)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("web_dir", "")
        cfg = config_loader.load()
        fp = config_loader.footprint(cfg)
        laser = config_loader.laser_pose(cfg)
        self._static = {"footprint": [fp.x_front, fp.x_rear, fp.half_width],
                        "laser": [laser.x, laser.y, laser.yaw]}
        self._frame = self.get_parameter("global_frame").value
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._map_json = None
        self._scan = self._dyn = None
        self._plan: list = []
        self._guard: dict = {}
        self._map_pub = self.create_publisher(String, "/ui/map", 1)
        self._state_pub = self.create_publisher(String, "/ui/state", 5)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan_nav", lambda m: setattr(self, "_scan", m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_dynamic", lambda m: setattr(self, "_dyn", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self._on_plan, 5)
        self.create_subscription(String, "/safety_guard/state", self._on_guard, 10)
        self.create_timer(3.0, self._publish_map)
        self.create_timer(0.2, self._publish_state)
        self._start_http()

    def _start_http(self) -> None:
        web = self.get_parameter("web_dir").value or os.path.join(get_package_share_directory("car_nav"), "web")
        port = int(self.get_parameter("http_port").value)
        handler = functools.partial(_QuietHandler, directory=web)
        try:
            srv = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
        except OSError as exc:
            self.get_logger().error(f"web UI: cannot listen on port {port}: {exc}")
            return
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.get_logger().info(f"web UI on http://<this computer>:{port}/  (rosbridge "
                               f"ws port {self.get_parameter('rosbridge_port').value}), files {web}")

    def _on_map(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16)
        data = np.where(data < 0, -1, np.where(data >= 50, 100, 0))
        self._map_json = json.dumps({
            "w": msg.info.width, "h": msg.info.height, "res": msg.info.resolution,
            "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
            "rle": rle(data),
        })
        self._publish_map()

    def _publish_map(self) -> None:
        if self._map_json is not None:
            self._map_pub.publish(String(data=self._map_json))

    def _on_plan(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        step = max(1, len(pts) // 300)
        self._plan = [[round(x, 2), round(y, 2)] for x, y in pts[::step] + pts[-1:]] if pts else []

    def _on_guard(self, msg: String) -> None:
        try:
            self._guard = json.loads(msg.data)
        except ValueError:
            pass

    def _lookup(self, child: str):
        try:
            tf = self._tf.lookup_transform(self._frame, child, Time())
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y, _yaw(t.rotation))

    def _publish_state(self) -> None:
        pose = self._lookup("base_link")
        st = dict(self._static)
        st["pose"] = None if pose is None else [round(pose[0], 3), round(pose[1], 3), round(pose[2], 4)]
        st["guard"] = self._guard
        st["plan"] = self._plan
        st["scan"] = st["dyn"] = []
        if self._scan is not None:
            sp = self._lookup(self._scan.header.frame_id)
            if sp is not None:
                st["scan"] = scan_to_map(self._scan, sp)
                if self._dyn is not None:
                    st["dyn"] = scan_to_map(self._dyn, sp, step=1)
        self._state_pub.publish(String(data=json.dumps(st)))


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args) -> None:  # noqa: D401 - silence per-request logs
        pass


def main() -> None:
    rclpy.init()
    node = UiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
