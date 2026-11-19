"""Nav2 /cmd_vel -> "<left m/s>,<right m/s>" wheel command on a ROS topic. NO MQTT.

Uses the shared diff-drive mapping in actuation.py. Publishes on `control`
(std_msgs/String, remapped to /car/wheels/cmd by nav.launch.py) at a fixed
rate so the receiver always has a fresh command:

* fresh cmd_vel (younger than watchdog_s): mapped payload, e.g. "0.356,0.444"
* no cmd_vel for watchdog_s: the STOP payload "0.000,0.000"

Health check override (web UI Health tab, wheels test): "<left>,<right>" on /health/wheels_override
replaces the mapped payload while it is younger than OVERRIDE_FRESH_S. It is the ONLY way a wheel is
driven backwards or alone (the normal path has allow_reverse false). Safety: each wheel is clamped to
OVERRIDE_MAX_MPS, it expires by itself 0.3 s after the last message, it is ignored while the E-STOP
(/ui/estop) is on and while a non-zero /cmd_vel (navigation) is fresh.

In the simulator the actuator model listens to this topic. On the real
car the ugv-controller adapter turns it into the controller's own command.
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from . import car_reload, config_loader
from .actuation import CmdVelMapper, drive_from_cfg, format_payload, parse_payload
from .timing import TimingStats


OVERRIDE_TOPIC = "/health/wheels_override"
OVERRIDE_FRESH_S = 0.3
OVERRIDE_MAX_MPS = 0.25


def override_payload(text: str, age_s: float, estop: bool, nav_active: bool):
    """(left, right) to send instead of the mapped command, or None when the override does not apply."""
    if estop or nav_active or age_s > OVERRIDE_FRESH_S:
        return None
    p = parse_payload(text)
    if p is None:
        return None
    lim = OVERRIDE_MAX_MPS
    return tuple(min(max(w, -lim), lim) for w in p)


class CmdVelToControl(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_control")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("watchdog_s", 0.5)
        cfg = config_loader.load()
        drive = drive_from_cfg(cfg)
        self._mapper = CmdVelMapper(drive)
        self._watchdog = float(self.get_parameter("watchdog_s").value)
        if not drive.measured:
            self.get_logger().warn(
                "drive.track_width_m NOT MEASURED: using the placeholder "
                f"{drive.track_width_m} m (car.yaml drive.assumed_track_width_m)")
        self.get_logger().info(
            f"diff drive: track {drive.track_width_m:.3f} m, wheel limit {drive.max_wheel_speed_mps} m/s, "
            f"yaw limit {drive.max_yaw_rate_rps} rad/s, reverse {'on' if drive.allow_reverse else 'off'}")
        self._pub = self.create_publisher(String, "control", 10)
        self._timing = TimingStats(self.get_logger())
        self.create_subscription(Twist, "cmd_vel", self._timing.wrap("bridge.cmd_cb", self._on_cmd), 10)
        self._last_cmd: tuple[float, float] | None = None
        self._last_t = 0.0
        self._last_payload = ""
        self._ovr = ("", -1e9)
        self._estop = False
        self.create_subscription(String, OVERRIDE_TOPIC, lambda m: setattr(self, "_ovr", (m.data, time.monotonic())), 10)
        self.create_subscription(Bool, "/ui/estop", lambda m: setattr(self, "_estop", bool(m.data)), 10)
        car_reload.watch(self, self._reload)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _reload(self, cfg: dict) -> None:
        drive = drive_from_cfg(cfg)
        self._mapper = CmdVelMapper(drive)
        self.get_logger().info(
            f"diff drive: track {drive.track_width_m:.3f} m, wheel limit {drive.max_wheel_speed_mps} m/s, "
            f"yaw limit {drive.max_yaw_rate_rps} rad/s")

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = (msg.linear.x, msg.angular.z)
        self._last_t = time.monotonic()
        self._tick()

    def _tick(self) -> None:
        if self._last_cmd is None or time.monotonic() - self._last_t > self._watchdog:
            left, right = self._mapper.stop_payload
        else:
            left, right = self._mapper.map(*self._last_cmd)
        now = time.monotonic()
        nav = (self._last_cmd is not None and now - self._last_t <= self._watchdog
               and (abs(self._last_cmd[0]) > 1e-3 or abs(self._last_cmd[1]) > 1e-3))
        ovr = override_payload(self._ovr[0], now - self._ovr[1], self._estop, nav)
        if ovr is not None:
            left, right = ovr
        payload = format_payload(left, right)
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
