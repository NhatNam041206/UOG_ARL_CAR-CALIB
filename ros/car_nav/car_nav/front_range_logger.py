"""Log the distance to whatever is straight ahead, for the wall-approach
baseline speed test.

    ros2 run car_nav front_range_logger --ros-args -p run_name:=wall1

Writes /data/nav/wall_runs/<run_name>.csv with columns
wall_time_s, front_x_m, points. Analyse it on the laptop with
scripts/analyze_wall_approach.py. Read-only: this sends nothing to the car.
"""
from __future__ import annotations

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from . import config_loader
from .geometry import front_distance


class FrontRangeLogger(Node):
    def __init__(self) -> None:
        super().__init__("front_range_logger")
        cfg = config_loader.load()
        self._laser = config_loader.laser_pose(cfg)
        self._half = float(self.declare_parameter("corridor_half_width_m", 0.20).value)
        out_dir = str(self.declare_parameter("output_dir", "/data/nav/wall_runs").value)
        run = str(self.declare_parameter("run_name", time.strftime("wall_%Y%m%d_%H%M%S")).value)
        os.makedirs(out_dir, exist_ok=True)
        self._path = os.path.join(out_dir, f"{run}.csv")
        self._fh = open(self._path, "w", encoding="utf-8", buffering=1)
        self._fh.write("wall_time_s,front_x_m,points\n")
        self._rows = 0
        self.create_subscription(LaserScan, "scan_filtered", self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f"logging front distance to {self._path}")

    def _on_scan(self, msg: LaserScan) -> None:
        dist, n = front_distance(msg.ranges, msg.angle_min, msg.angle_increment, self._laser, self._half)
        self._fh.write(f"{time.time():.3f},{'' if dist is None else f'{dist:.3f}'},{n}\n")
        self._rows += 1
        if self._rows % 50 == 0:
            self.get_logger().info(f"{self._rows} rows, front={dist}")

    def close(self) -> None:
        self._fh.close()


def main() -> None:
    rclpy.init()
    node = FrontRangeLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
