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
from .actuation import CmdVelMapper, MapperConfig, SteeringSmoother, format_payload, table_from_cfg


class CmdVelToControl(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_control")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("watchdog_s", 0.5)
        self.declare_parameter("forward_threshold_mps", 0.05)
        # Steering smoothing (see actuation.SteeringSmoother). 0 = off.
        self.declare_parameter("steer_smoothing_tau_s", 0.0)
        self.declare_parameter("steer_max_rate_dps", 0.0)
        cfg = config_loader.load()
        table = table_from_cfg(cfg)
        self._mapper = CmdVelMapper(
            table, MapperConfig(forward_threshold_mps=float(self.get_parameter("forward_threshold_mps").value))
        )
        self._watchdog = float(self.get_parameter("watchdog_s").value)
        self._smoother = SteeringSmoother(float(self.get_parameter("steer_smoothing_tau_s").value),
                                          float(self.get_parameter("steer_max_rate_dps").value))
        self._dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self._last_tick = 0.0
        if self._smoother.enabled:
            self.get_logger().info(f"steering smoothing: tau {self._smoother.tau_s} s, "
                                   f"max rate {self._smoother.max_rate_dps} deg/s")
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
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self._last_cmd: tuple[float, float] | None = None
        self._last_t = 0.0
        self._last_payload = ""
        self.create_timer(self._dt, self._tick)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = (msg.linear.x, msg.angular.z)
        self._last_t = time.monotonic()
        self._tick()

    def _tick(self) -> None:
        now = time.monotonic()
        dt = (now - self._last_tick) if self._last_tick else self._dt
        self._last_tick = now
        if self._last_cmd is None or now - self._last_t > self._watchdog:
            angle, code = self._mapper.stop_payload
        else:
            angle, code = self._mapper.map(*self._last_cmd)
        if code == 1:
            angle = int(round(self._smoother.step(angle, dt)))
        else:
            self._smoother.reset(None)
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
