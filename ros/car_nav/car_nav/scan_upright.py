"""/scan_filtered -> /scan_nav (laser_upright); a pass-through for the upright-mounted LiDAR.

Every Nav2 consumer (rf2o, AMCL, slam_toolbox, costmaps, collision
monitor) reads /scan_nav. The matching static TF base_link -> laser_upright
(same x, y, z, yaw as the laser, roll = pitch = 0) is published by
nav.launch.py. See scan_tools.py for why.

It also publishes /scan_odom: the same scan with every invalid beam
(inf / nan) set to 0. rf2o treats 0 as "no return" but lets +inf through
on its coarser pyramid levels, so rf2o reads /scan_odom.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from . import car_reload, config_loader
from .scan_tools import is_inverted, upright, zero_invalid
from .timing import TimingStats


class ScanUpright(Node):
    def __init__(self) -> None:
        super().__init__("scan_upright")
        self.declare_parameter("frame_id", "laser_upright")
        self._frame = self.get_parameter("frame_id").value
        laser = config_loader.laser_pose(config_loader.load())
        self._inverted = is_inverted(laser.roll, laser.pitch)
        self.get_logger().info(
            f"laser is {'upside down: mirroring' if self._inverted else 'upright: passing'} scans into {self._frame}"
        )
        self._pub = self.create_publisher(LaserScan, "scan_nav", qos_profile_sensor_data)
        self._pub_odom = self.create_publisher(LaserScan, "scan_odom", qos_profile_sensor_data)
        self._timing = TimingStats(self.get_logger())
        car_reload.watch(self, self._reload)
        self.create_subscription(LaserScan, "scan_filtered", self._timing.wrap("upright.scan_cb", self._on_scan),
                                 qos_profile_sensor_data)

    def _reload(self, cfg: dict) -> None:
        laser = config_loader.laser_pose(cfg)
        self._inverted = is_inverted(laser.roll, laser.pitch)

    def _on_scan(self, msg: LaserScan) -> None:
        ranges, intens, amin, amax = upright(
            msg.ranges, msg.intensities, msg.angle_min, msg.angle_max, self._inverted
        )
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame
        out.angle_min, out.angle_max = amin, amax
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.ranges, out.intensities = ranges, intens
        self._pub.publish(out)
        odom = LaserScan()
        odom.header, odom.angle_min, odom.angle_max = out.header, amin, amax
        odom.angle_increment, odom.time_increment, odom.scan_time = out.angle_increment, out.time_increment, out.scan_time
        odom.range_min, odom.range_max = out.range_min, out.range_max
        odom.ranges = zero_invalid(ranges)
        self._pub_odom.publish(odom)


def main() -> None:
    rclpy.init()
    node = ScanUpright()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
