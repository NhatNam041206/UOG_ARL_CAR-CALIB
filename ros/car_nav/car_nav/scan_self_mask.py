"""/scan -> /scan_filtered with the car's own body removed.

Points that land inside the footprint (+margin) in base_link are the car
itself (mast, body), so they are set to +inf. Extra sectors can be masked
with self_mask.sectors_deg in car.yaml. Logs the masked share every 10 s
so the masked angles can be reported.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from . import car_reload, config_loader
from .geometry import bearing_deg, beam_to_base, self_mask
from .timing import TimingStats


class ScanSelfMask(Node):
    def __init__(self) -> None:
        super().__init__("scan_self_mask")
        self._reload(config_loader.load())
        car_reload.watch(self, self._reload)
        self._pub = self.create_publisher(LaserScan, "scan_filtered", qos_profile_sensor_data)
        self._timing = TimingStats(self.get_logger())
        self.create_subscription(LaserScan, "scan", self._timing.wrap("mask.scan_cb", self._on_scan),
                                 qos_profile_sensor_data)
        self._last_report = 0.0
        self._masked_bearings: set[int] = set()

    def _reload(self, cfg: dict) -> None:
        self._laser = config_loader.laser_pose(cfg)
        self._footprint = config_loader.footprint(cfg)
        mask = cfg.get("self_mask") or {}
        self._margin = float(mask.get("margin_m", 0.03))
        self._sectors = [tuple(s) for s in (mask.get("sectors_deg") or [])]
        self._masked_bearings = set()

    def _on_scan(self, msg: LaserScan) -> None:
        ranges, masked = self_mask(
            msg.ranges, msg.angle_min, msg.angle_increment,
            self._laser, self._footprint, self._margin, self._sectors,
        )
        for i, (old, new) in enumerate(zip(msg.ranges, ranges)):
            if new != old:
                bx, by = beam_to_base(old, msg.angle_min + i * msg.angle_increment, self._laser)
                self._masked_bearings.add(int(round(bearing_deg(bx, by))))
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        self._pub.publish(out)
        now = time.monotonic()
        if now - self._last_report > 10.0:
            self._last_report = now
            self.get_logger().info(
                f"self-mask: {masked}/{len(ranges)} beams this scan; "
                f"masked base_link bearings so far (deg): {sorted(self._masked_bearings)}"
            )


def main() -> None:
    rclpy.init()
    node = ScanSelfMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
