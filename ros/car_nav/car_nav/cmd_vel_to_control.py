"""Nav2 /cmd_vel -> "<angle>,<drive_code>" on a ROS topic. NO MQTT.

Uses the shared mapping in actuation.py. Publishes on `control`
(std_msgs/String, default remapped to /car/raspi5/control) at a fixed rate
so the receiver always has a fresh command:

* fresh cmd_vel (younger than watchdog_s): mapped payload, e.g. "48,1"
* no cmd_vel for watchdog_s: the STOP payload "62,0"

In the simulator the actuator model listens to this topic. On the real
car the WO-02 forwarder copies it to MQTT car/raspi5/control (only while
the drive mode is NAV2).
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from . import config_loader
from .actuation import CmdVelMapper, MapperConfig, format_payload, table_from_cfg
from .timing import TimingStats


class CmdVelToControl(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_control")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("watchdog_s", 0.5)
        self.declare_parameter("forward_threshold_mps", 0.05)
        cfg = config_loader.load()
        table = table_from_cfg(cfg)
        self._mapper = CmdVelMapper(
            table, MapperConfig(forward_threshold_mps=float(self.get_parameter("forward_threshold_mps").value))
        )
        self._watchdog = float(self.get_parameter("watchdog_s").value)
        if not table.measured:
            self.get_logger().warn(
                "steering.curvature_table NOT MEASURED: using the placeholder built from "
                f"wheelbase {table.cfg.wheelbase_m} m and linkage ratio {table.cfg.assumed_linkage_ratio}"
            )
        self.get_logger().info(
            f"curvature limits: left {table.max_left_curvature:.2f} 1/m, right "
            f"{table.max_right_curvature:.2f} 1/m -> min turning radius {table.min_turning_radius:.2f} m"
        )
        self._pub = self.create_publisher(String, "control", 10)
        self._timing = TimingStats(self.get_logger())
        self.create_subscription(Twist, "cmd_vel", self._timing.wrap("bridge.cmd_cb", self._on_cmd), 10)
        self._last_cmd: tuple[float, float] | None = None
        self._last_t = 0.0
        self._last_payload = ""
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = (msg.linear.x, msg.angular.z)
        self._last_t = time.monotonic()
        self._tick()

    def _tick(self) -> None:
        if self._last_cmd is None or time.monotonic() - self._last_t > self._watchdog:
            angle, code = self._mapper.stop_payload
        else:
            angle, code = self._mapper.map(*self._last_cmd)
        payload = format_payload(angle, code)
        self._pub.publish(String(data=payload))
        if payload != self._last_payload:
            self.get_logger().debug(f"control {payload}")
            self._last_payload = payload


def main() -> None:
    rclpy.init()
    node = CmdVelToControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
