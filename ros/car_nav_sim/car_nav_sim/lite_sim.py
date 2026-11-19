"""Lightweight 2D simulator: no Gazebo, no GPU, runs in CI and on any laptop.

* listens to /car/wheels/cmd ("<left m/s>,<right m/s>") like the real car
* moves the skid-steer car with the actuator model (latency, wheel limits,
  speed scale, yaw efficiency)
* publishes /scan in the LiDAR's own frame `laser`, like sllidar_ros2 on
  the car, by ray casting the world shapes
* publishes /camera/depth/points (depth:=true): a simulated Astra, 3D ray casting from the
  camera pose in car.yaml against the floor, boxes / cylinders with their heights and the
  actors (sim.yaml depth:). The LiDAR (0.42 m) does not see shapes lower than 0.42 m; the
  camera does. A mission's extra_obstacles are in this world but NOT in the map
* publishes /odom_wheel: what wheel encoders would report (no slip), for
  adaptive_odom wheel_odom_topic:=/odom_wheel
* moves the mission's actors (people, actors.py): in the scan, not in the
  map; their positions on /sim/actors (PoseArray, z = radius) for the scorer
* publishes /clock (all other nodes run with use_sim_time) and the true
  pose on /sim/ground_truth (frame `world` = map)
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String

from car_nav import car_reload, config_loader
from car_nav.actuation import WHEELS_TOPIC, drive_from_cfg, parse_payload
from car_nav.depth_obstacles import encode_xyz
from car_nav.ugv_link import UgvConfig

from . import sim_config
from .actors import load_actors
from .actuator_model import ActuatorModel, Pose2D, TruthParams, integrate
from .depth_model import depth_points, pixel_rays
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
        self.declare_parameter("mission", "")   # its actors: list walks in the scan
        self.declare_parameter("depth", False)  # publish the simulated depth camera
        car = config_loader.load()
        sim = sim_config.load_sim()
        self.world = World.load(sim_config.resolve("worlds", self.get_parameter("world").value))
        start = list(self.get_parameter("start").value)
        x, y, th = start if len(start) == 3 else self.world.start
        self.pose = Pose2D(x, y, th)
        mission = self.get_parameter("mission").value
        mdata = {}
        if mission:
            with open(sim_config.resolve("missions", mission), "r", encoding="utf-8") as fh:
                mdata = yaml.safe_load(fh) or {}
        self.actors = load_actors(mdata)
        n_extra = self.world.add_extras(mdata)   # in the world, not in the map (the map is built from the file)
        self._static_cyl = list(self.world.cylinders)
        self.actors_pub = self.create_publisher(PoseArray, "/sim/actors", 10)
        self._next_actors = 0.0
        self.laser = config_loader.laser_pose(car)
        self.camera = config_loader.camera_pose(car)
        dcfg = sim.get("depth") or {}
        self.depth_on = bool(self.get_parameter("depth").value) and bool(dcfg.get("enabled", True))
        self.depth_cfg = dcfg
        self.depth_rays = pixel_rays(int(dcfg.get("width", 80)), int(dcfg.get("height", 60)),
                                     float(dcfg.get("hfov_deg", 58.0)), float(dcfg.get("vfov_deg", 45.0)))
        self.depth_period = 1.0 / float(dcfg.get("rate_hz", 10.0))
        self.next_depth = 0.0
        self.depth_pub = self.create_publisher(PointCloud2, str(dcfg.get("topic", "/camera/depth/points")),
                                               qos_profile_sensor_data)
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
        # stand-in for the ESP32's "OK: L= R=" echo (ugv_serial_bridge publishes it on the car)
        self._ugv = UgvConfig.from_cfg(car)
        self._applied_pub = self.create_publisher(String, "/car/wheels/applied", 10)
        car_reload.watch(self, self._reload_car)
        self.get_logger().info(
            f"lite sim: world {self.world.name}, start ({x:.2f}, {y:.2f}, {math.degrees(th):.0f} deg), "
            f"truth {self.model.params}, rtf {self.rtf}, depth camera {'on' if self.depth_on else 'off'}, "
            f"{n_extra} extra obstacles (not in the map)"
        )
        self._wall0 = time.monotonic()
        self.create_timer(self.dt / self.rtf, self._tick)

    def _reload_car(self, car: dict) -> None:
        """The web UI applied a new car.yaml: the simulated LiDAR / camera mounts and drive follow."""
        self.laser = config_loader.laser_pose(car)
        self.camera = config_loader.camera_pose(car)
        self.model.drive = drive_from_cfg(car)
        self._ugv = UgvConfig.from_cfg(car)

    def _on_control(self, msg: String) -> None:
        if self.model.command(self.t, msg.data):
            l, r = parse_payload(msg.data)
            a, b = self._ugv.command(l, r).split()
            self._applied_pub.publish(String(data=f"{a},{b}"))

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
            for a in self.actors:
                a.update(self.t, (self.pose.x, self.pose.y))
            if self.actors and self.t >= self._next_actors:
                self._next_actors += 0.05
                self._publish_actors()
            if self.t >= self._next_wheel:
                self._next_wheel += 0.05   # 20 Hz like a typical controller feedback
                self._publish_wheel(ev, ew)
            steps += 1
            if self.t >= self.next_scan:
                self.next_scan += self.scan_period
                self._publish_scan()
            if self.depth_on and self.t >= self.next_depth:
                self.next_depth += self.depth_period
                self._publish_depth()
        self.clock_pub.publish(Clock(clock=_stamp(self.t)))
        self._publish_truth()

    def _publish_actors(self) -> None:
        msg = PoseArray()
        msg.header.stamp = _stamp(self.t)
        msg.header.frame_id = "world"
        for a in self.actors:
            x, y = a.position(self.t)
            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, a.radius
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.actors_pub.publish(msg)

    def _publish_scan(self) -> None:
        self.world.cylinders = self._static_cyl + [a.cylinder(self.t) for a in self.actors]
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

    def _publish_depth(self) -> None:
        d = self.depth_cfg
        self.world.cylinders = self._static_cyl + [a.cylinder(self.t) for a in self.actors]
        pts = depth_points(self.world, self.pose, self.camera, self.depth_rays,
                           float(d.get("range_min_m", 0.6)), float(d.get("range_max_m", 4.0)),
                           float(d.get("noise_std_m", 0.01)), float(d.get("dropout", 0.01)), self.rng)
        msg = PointCloud2()
        msg.header.stamp = _stamp(self.t)
        msg.header.frame_id = str(d.get("frame", "camera_depth_optical_frame"))
        msg.height, msg.width = 1, int(pts.shape[0])
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.is_bigendian, msg.is_dense = False, True
        msg.point_step, msg.row_step = 12, 12 * msg.width
        msg.data = encode_xyz(pts)
        self.depth_pub.publish(msg)

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
