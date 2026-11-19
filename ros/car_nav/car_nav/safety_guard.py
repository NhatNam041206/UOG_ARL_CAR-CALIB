"""Safety guard for people in front of the car: cmd_vel_guarded -> cmd_vel.

Sits between the collision monitor and cmd_vel_to_control:

    ... collision_monitor -> /cmd_vel_guarded -> safety_guard -> /cmd_vel -> cmd_vel_to_control

and decides from /scan_dynamic (scan points that are not on the map, see
dynamic_scan.py) with safety_zone.decide(): stop / slow / clear, only for
points in FRONT of the car. It also stops the car

* while the web UI's emergency stop (/ui/estop, std_msgs/Bool) is on,
* when /scan_dynamic is older than scan_timeout_s (fail safe),

Depth camera (second source, off unless depth_enabled): /depth_obstacles (depth_guard_source.py,
2D points of things the LiDAR plane misses: low boxes, table tops) are judged with the same
front zones, the stop distance being depth_danger_range_m (the camera has a blind zone close to
the car). Stale depth (nothing newer than depth_timeout_s, e.g. the camera is still starting
or died) NEVER stops the car by itself: the guard goes on with the LiDAR only and reports
state "depth_stale" (when the LiDAR sees nothing either) plus depth_status "stale" in its JSON
and a warning in the log. depth_required: true turns that into a stop ("no_depth").
With depth_enabled false the behaviour is exactly the LiDAR-only guard.

and drives on by itself once the zone has been clear for resume_delay_s.
The zone sizes are ROS parameters (nav2_params.yaml safety_guard) and can be
changed live from the web UI: /ui/safety_params (std_msgs/String, JSON such
as {"danger_range_m": 0.8}). State on /safety_guard/state (JSON).
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String

from . import config_loader
from .depth_obstacles import parse_cloud
from .safety_zone import GuardParams, decide_merged
from .wall_avoid import points_in_base


class SafetyGuard(Node):
    def __init__(self) -> None:
        super().__init__("safety_guard")
        self._p = GuardParams()
        for k, v in self._p.as_dict().items():
            self.declare_parameter(k, v)
        self.declare_parameter("scan_timeout_s", 1.0)
        self._p.update({k: self.get_parameter(k).value for k in self._p.as_dict()})
        self.add_on_set_parameters_callback(self._on_set_params)
        cfg = config_loader.load()
        fp = config_loader.footprint(cfg)
        self._x_front, self._half_body = fp.x_front, fp.half_width
        self._laser = config_loader.laser_pose(cfg)
        self._points = np.zeros((0, 2))
        self._scan_t = 0.0
        self._depth = np.zeros((0, 2))
        self._depth_t = -1e9
        self._depth_status = "off"
        self._estop = False
        self._state = "clear"
        self._last_stop = -1e9
        self._nearest = math.inf
        self._cmd_t = 0.0
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/safety_guard/state", 10)
        self.create_subscription(Twist, "cmd_vel_guarded", self._on_cmd, 10)
        self.create_subscription(LaserScan, "/scan_dynamic", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/depth_obstacles", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Bool, "/ui/estop", self._on_estop, 10)
        self.create_subscription(String, "/ui/safety_params", self._on_ui_params, 10)
        # wall-clock timer: the state must update even if /clock stalls (a sim-time timer here
        # never fired in the sim, 2026-11-17)
        self.create_timer(0.2, self._publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"safety guard: {self._p.as_dict()}")

    # ---------------------------------------------------------------- params
    def _on_set_params(self, params) -> SetParametersResult:
        d = {p.name: p.value for p in params if p.name in self._p.as_dict()}
        changed = self._p.update(d)
        if changed:
            self.get_logger().info(f"safety guard params changed: { {k: getattr(self._p, k) for k in changed} }")
        return SetParametersResult(successful=True)

    def _on_ui_params(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"bad /ui/safety_params: {msg.data!r}")
            return
        changed = self._p.update(d)
        if changed:
            self.get_logger().info(f"web UI set: { {k: getattr(self._p, k) for k in changed} }")
        self._publish_state()

    def _on_estop(self, msg: Bool) -> None:
        if bool(msg.data) != self._estop:
            self._estop = bool(msg.data)
            self.get_logger().warn(f"emergency stop {'ON' if self._estop else 'released'} (web UI)")
            if self._estop:
                self._pub.publish(Twist())

    # ----------------------------------------------------------------- data
    def _on_scan(self, msg: LaserScan) -> None:
        self._points = points_in_base(msg.ranges, msg.angle_min, msg.angle_increment, self._laser.x,
                                      self._laser.y, self._laser.yaw, max_range=6.0)
        self._scan_t = time.monotonic()

    def _on_depth(self, msg: PointCloud2) -> None:
        offs = {f.name: f.offset for f in msg.fields}
        n = msg.width * msg.height
        if n and all(k in offs for k in "xy"):
            xyz = parse_cloud(bytes(msg.data), msg.point_step, (offs["x"], offs["y"], offs["x"]), n,
                              msg.is_bigendian)
            self._depth = xyz[:, :2]
        else:
            self._depth = np.zeros((0, 2))
        self._depth_t = time.monotonic()

    def _depth_points(self, now: float):
        """(points or None, status): None = do not use depth (off or stale)."""
        if not self._p.depth_enabled:
            self._depth_status = "off"
            return None
        if now - self._depth_t > self._p.depth_timeout_s:
            if self._depth_status != "stale":
                self.get_logger().warn(f"depth data older than {self._p.depth_timeout_s} s or none yet: "
                                       + ("STOPPING (depth_required)" if self._p.depth_required
                                          else "continuing on the LiDAR only"))
            self._depth_status = "stale"
            return None
        if self._depth_status == "stale":
            self.get_logger().info("depth data is back")
        self._depth_status = "ok"
        return self._depth

    def _evaluate(self, turning: bool) -> str:
        now = time.monotonic()
        if self._estop:
            return "estop"
        if now - self._scan_t > float(self.get_parameter("scan_timeout_s").value):
            return "no_scan"
        depth = self._depth_points(now)
        if depth is None and self._depth_status == "stale" and self._p.depth_required and self._p.enabled:
            return "no_depth"
        dec = decide_merged(self._points, depth, self._p, self._x_front, self._half_body, turning)
        self._nearest = dec.nearest_m
        if dec.state == "stop":
            self._last_stop = now
            return "stop"
        if dec.state in ("clear", "slow") and now - self._last_stop < self._p.resume_delay_s:
            return "hold"      # stopped a moment ago: wait until the zone stays clear
        if dec.state == "clear" and self._depth_status == "stale":
            return "depth_stale"    # LiDAR-only: the car drives on, the page / log say why
        return dec.state

    def _set_state(self, state: str) -> None:
        if state != self._state:
            text = f"safety guard: {self._state} -> {state} (nearest {self._nearest:.2f} m)"
            if state in ("stop", "estop", "no_scan", "no_depth"):
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
            self._state = state

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_t = time.monotonic()
        turning = abs(msg.linear.x) < 0.05 and abs(msg.angular.z) > 0.05
        state = self._evaluate(turning)
        out = Twist()
        if state in ("clear", "off", "depth_stale"):
            out = msg
        elif state == "slow":
            out.linear.x = msg.linear.x * self._p.slow_ratio
            out.angular.z = msg.angular.z * self._p.slow_ratio
        self._set_state(state)
        self._pub.publish(out)

    def _publish_state(self) -> None:
        if time.monotonic() - self._cmd_t > 0.5:      # no commands (idle): still show what it sees
            self._set_state(self._evaluate(False))
        self._state_pub.publish(String(data=json.dumps({
            "state": self._state, "estop": self._estop,
            "nearest_m": None if math.isinf(self._nearest) else round(self._nearest, 3),
            "depth_status": self._depth_status,
            "depth_age_s": None if self._depth_t < -1e8 else round(time.monotonic() - self._depth_t, 2),
            "depth_points": int(self._depth.shape[0]),
            "params": self._p.as_dict(),
            "footprint": {"x_front": self._x_front, "half_width": self._half_body},
        })))


def main() -> None:
    rclpy.init()
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
