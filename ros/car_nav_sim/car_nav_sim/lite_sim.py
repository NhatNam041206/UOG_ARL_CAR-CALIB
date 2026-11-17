"""Lightweight 2D simulator: no Gazebo, no GPU, runs in CI and on any laptop.

* listens to /car/wheels/cmd ("<left m/s>,<right m/s>") like the real car
* moves the skid-steer car with the actuator model (latency, wheel limits,
  speed scale, yaw efficiency)
* publishes /scan in the LiDAR's own frame `laser`, like sllidar_ros2 on
  the car, by ray casting the world shapes
* publishes /odom_wheel: what wheel encoders would report (no slip), for
  adaptive_odom wheel_odom_topic:=/odom_wheel
* publishes /clock (all other nodes run with use_sim_time) and the true
  pose on /sim/ground_truth (frame `world` = map)
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from car_nav import config_loader
from car_nav.actuation import WHEELS_TOPIC, drive_from_cfg

from . import sim_config
from .actuator_model import ActuatorModel, Pose2D, TruthParams, integrate
from .scan_model import laser_scan_ranges
from .world import World


def _stamp(t: float):
    from builtin_interfaces.msg import Time

    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class LiteSim(Node):
    def __init__(self) -> None:
        super().__init__("lite_sim")
        self.declare_parameter("world", "test_floor")
        self.declare_parameter("start", [0.0])  # [x, y, yaw]; [0.0] = world start
        self.declare_parameter("seed", 1)
        car = config_loader.load()
        sim = sim_config.load_sim()
        self.world = World.load(sim_config.resolve("worlds", self.get_parameter("world").value))
        start = list(self.get_parameter("start").value)
        x, y, th = start if len(start) == 3 else self.world.start
        self.pose = Pose2D(x, y, th)
        self.laser = config_loader.laser_pose(car)
        self.model = ActuatorModel(drive_from_cfg(car), TruthParams.from_dict(sim.get("truth") or {}))
        self.enc = Pose2D(0.0, 0.0, 0.0)
        self.lidar = sim.get("lidar") or {}
        lite = sim.get("lite") or {}
        self.dt = 1.0 / float(lite.get("physics_hz", 100.0))
        self.rtf = float(lite.get("realtime_factor", 1.0))
        n = int(self.lidar.get("samples", 720))
        self.inc = 2 * math.pi / n
        self.angles = -math.pi + np.arange(n) * self.inc
        self.scan_period = 1.0 / float(self.lidar.get("rate_hz", 7.0))
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self.t = 0.0
        self.next_scan = 0.0
        self.clock_pub = self.create_publisher(Clock, "/clock", QoSProfile(depth=10))
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.wheel_pub = self.create_publisher(Odometry, "/odom_wheel", 10)
        self._next_wheel = 0.0
        self.create_subscription(String, WHEELS_TOPIC, self._on_control, 50)
        self.get_logger().info(
            f"lite sim: world {self.world.name}, start ({x:.2f}, {y:.2f}, {math.degrees(th):.0f} deg), "
            f"truth {self.model.params}, rtf {self.rtf}"
        )
        self._wall0 = time.monotonic()
        self.create_timer(self.dt / self.rtf, self._tick)

    def _on_control(self, msg: String) -> None:
        self.model.command(self.t, msg.data)

    def _tick(self) -> None:
        # Catch up to wall time x rtf without drifting.
        target = (time.monotonic() - self._wall0) * self.rtf
        steps = 0
        while self.t + self.dt <= target and steps < 50:
            v, w = self.model.step(self.t, self.dt)
            self.pose = integrate(self.pose, v, w, self.dt)
            ev, ew = self.model.encoder_twist()
            self.enc = integrate(self.enc, ev, ew, self.dt)
            self.t += self.dt
            if self.t >= self._next_wheel:
                self._next_wheel += 0.05   # 20 Hz like a typical controller feedback
                self._publish_wheel(ev, ew)
            steps += 1
            if self.t >= self.next_scan:
                self.next_scan += self.scan_period
                self._publish_scan()
        self.clock_pub.publish(Clock(clock=_stamp(self.t)))
        self._publish_truth()

    def _publish_scan(self) -> None:
        r = laser_scan_ranges(self.world, self.pose, self.laser, self.angles, self.lidar, self.rng)
        msg = LaserScan()
        msg.header.stamp = _stamp(self.t)
        msg.header.frame_id = "laser"
        msg.angle_min = float(self.angles[0])
        msg.angle_max = float(self.angles[-1])
        msg.angle_increment = self.inc
        msg.scan_time = self.scan_period
        msg.time_increment = 0.0
        msg.range_min = float(self.lidar.get("range_min_m", 0.15))
        msg.range_max = float(self.lidar.get("range_max_m", 12.0))
        msg.ranges = [float(v) for v in r]
        self.scan_pub.publish(msg)

    def _publish_wheel(self, v: float, w: float) -> None:
        n = float(self.model.params.encoder_noise)
        o = Odometry()
        o.header.stamp = _stamp(self.t)
        o.header.frame_id = "odom_wheel"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.enc.x
        o.pose.pose.position.y = self.enc.y
        o.pose.pose.orientation.z = math.sin(self.enc.theta / 2)
        o.pose.pose.orientation.w = math.cos(self.enc.theta / 2)
        o.twist.twist.linear.x = v * (1.0 + n * float(self.rng.normal()))
        o.twist.twist.angular.z = w * (1.0 + n * float(self.rng.normal()))
        self.wheel_pub.publish(o)

    def _publish_truth(self) -> None:
        o = Odometry()
        o.header.stamp = _stamp(self.t)
        o.header.frame_id = "world"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.pose.x
        o.pose.pose.position.y = self.pose.y
        o.pose.pose.orientation.z = math.sin(self.pose.theta / 2)
        o.pose.pose.orientation.w = math.cos(self.pose.theta / 2)
        o.twist.twist.linear.x = self.model.v
        self.truth_pub.publish(o)


def main() -> None:
    rclpy.init()
    node = LiteSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
