"""Print where the nearest obstacle is in base_link terms, once a second.

Used to check the LiDAR transform without RViz: put a box on the car's
LEFT and the probe must say LEFT (+y). Read-only.

    ros2 run car_nav scan_probe
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from . import config_loader
from .geometry import bearing_deg, beam_to_base


def side_word(bearing: float) -> str:
    if -30.0 <= bearing <= 30.0:
        return "AHEAD"
    if 30.0 < bearing < 150.0:
        return "LEFT"
    if -150.0 < bearing < -30.0:
        return "RIGHT"
    return "BEHIND"


class ScanProbe(Node):
    def __init__(self) -> None:
        super().__init__("scan_probe")
        self._laser = config_loader.laser_pose(config_loader.load())
        self._latest: LaserScan | None = None
        self.create_subscription(LaserScan, "scan_filtered", self._on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self._report)

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest = msg

    def _report(self) -> None:
        msg = self._latest
        if msg is None:
            self.get_logger().warn("no /scan_filtered yet")
            return
        best = None
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and r > 0.0 and (best is None or r < best[0]):
                best = (r, i)
        if best is None:
            self.get_logger().info("no valid points")
            return
        r, i = best
        bx, by = beam_to_base(r, msg.angle_min + i * msg.angle_increment, self._laser)
        b = bearing_deg(bx, by)
        self.get_logger().info(
            f"nearest point: x={bx:+.2f} m y={by:+.2f} m bearing={b:+.0f} deg -> {side_word(b)}"
        )


def main() -> None:
    rclpy.init()
    node = ScanProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
