"""Adaptive odometry: rf2o + commanded speed x learned ratio (odom_source:=adaptive).

rf2o runs with publish_tf false and publishes /odom_rf2o. This node
publishes /odom and the TF odom -> base_link instead:

    rotation, sideways motion  <- rf2o (it measures these well)
    forward motion             <- w * rf2o + (1 - w) * (model speed * learned ratio)

model speed = the commanded forward speed (left + right) / 2 of the wheel
command on /car/wheels/cmd, delayed by the command latency and ramped; or,
with wheel_odom_topic set, the car's own wheel-encoder speed. The ratio
(real / model speed, start 1.0) absorbs wheel slip, battery and floor.

w = how well the current scan shows forward motion (speed_filter.scan_observability),
so in a plain corridor (w ~ 0) the car keeps moving forward at the learned
speed instead of "standing still" like rf2o alone. The learned speed is a
Kalman filter (speed_filter.SpeedKF, here on the ratio) fed by rf2o where it
can see and by AMCL's corrections along the heading after blind stretches.
A zero wheel command (delayed by the command latency) means standing still.

Logs the learned speed every 5 s and publishes it on /odom_speed_estimate
(std_msgs/Float32) for plots, and the scan observability on
/scan_observability (pose_gate uses it to hold back AMCL in blind corridors).
"""
from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

from . import config_loader
from .actuation import WHEELS_TOPIC, drive_from_cfg, parse_payload
from .speed_filter import MotionModel, SpeedKF, fuse_forward, scan_observability
from .timing import TimingStats


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class AdaptiveOdom(Node):
    def __init__(self) -> None:
        super().__init__("adaptive_odom")
        cfg = config_loader.load()
        self._drive = drive_from_cfg(cfg)
        latency = (cfg.get("speed") or {}).get("command_latency_s")
        p = self.declare_parameter
        p("initial_speed_ratio", 1.0)
        p("command_latency_s", float(latency) if latency is not None else 0.25)
        # nav_msgs/Odometry from the car's wheel encoders ('' = none: use the command)
        p("wheel_odom_topic", "")
        p("wheel_odom_timeout_s", 0.5)
        p("accel_mps2", 1.0)
        p("decel_mps2", 2.0)
        p("speed_random_walk", 0.01)     # m/s per sqrt(s)
        p("rf2o_speed_noise", 0.08)      # m/s at full observability
        p("obs_points_low", 10)
        p("obs_points_high", 50)
        p("obs_max_range_m", 8.0)
        p("correction_min_blind_m", 1.0)
        p("correction_min_along_m", 0.3)
        p("correction_max_blind_m", 6.0)
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("publish_tf", True)
        # pose_gate's /localized_pose (AMCL minus its blind-corridor guesses); /amcl_pose without the gate
        p("localized_pose_topic", "/localized_pose")
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._laser_yaw = float(config_loader.laser_pose(cfg).yaw)
        self._motion = MotionModel(latency_s=float(g("command_latency_s")), accel_mps2=float(g("accel_mps2")),
                                   decel_mps2=float(g("decel_mps2")), nominal_speed=self._drive.cruise_speed_mps)
        # The filter state is the ratio real / model speed; noises are given in m/s
        # at cruise speed and converted to ratio units here.
        vn = max(self._drive.cruise_speed_mps, 0.05)
        self._kf = SpeedKF(v=float(g("initial_speed_ratio")), var=0.2 ** 2,
                           q=(float(g("speed_random_walk")) / vn) ** 2,
                           r_rf2o=(float(g("rf2o_speed_noise")) / vn) ** 2, v_min=0.3, v_max=2.0)
        self._wheel_v: float | None = None
        self._wheel_t = -1e9
        self._obs_args = dict(max_range=float(g("obs_max_range_m")), n_low=int(g("obs_points_low")),
                              n_high=int(g("obs_points_high")))
        self._odom_frame, self._base_frame = g("odom_frame"), g("base_frame")
        self._w = 0.0
        self._n_facing = 0
        self._x = self._y = self._th = 0.0
        self._prev_rf2o: tuple[float, float, float, float] | None = None  # t, x, y, th
        self._hist: deque[tuple[float, float, float, float]] = deque(maxlen=400)  # t, x, y, th (odom)
        self._prev_corr: tuple[float, float, float] | None = None  # map->odom (x, y, th)
        self._blind_m = 0.0
        self._along_acc = 0.0
        self._last_log = 0.0
        self._n_rejected = 0

        self._odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self._speed_pub = self.create_publisher(Float32, "/odom_speed_estimate", 10)
        self._obs_pub = self.create_publisher(Float32, "/scan_observability", 10)
        self._tf = TransformBroadcaster(self) if g("publish_tf") else None
        self._timing = TimingStats(self.get_logger())
        tw = self._timing.wrap
        self.create_subscription(Odometry, "/odom_rf2o", tw("odom.rf2o_cb", self._on_rf2o), 20)
        self.create_subscription(LaserScan, "/scan_nav", tw("odom.scan_cb", self._on_scan), qos_profile_sensor_data)
        self.create_subscription(String, WHEELS_TOPIC, self._on_control, 20)
        if g("wheel_odom_topic"):
            self.create_subscription(Odometry, str(g("wheel_odom_topic")), self._on_wheel_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, str(g("localized_pose_topic")), tw("odom.amcl_cb", self._on_amcl), 10)
        self.get_logger().info(
            f"adaptive odometry: speed ratio {self._kf.v:.2f} (learned online), command latency "
            f"{self._motion.latency_s:.2f} s, model speed from "
            f"{g('wheel_odom_topic') or 'the wheel command'}")

    # ---------------------------------------------------------------- inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_control(self, msg: String) -> None:
        parsed = parse_payload(msg.data)
        if parsed is not None:
            v, omega = self._drive.body(*parsed)
            self._motion.command(self._now(), v, omega)

    def _on_wheel_odom(self, msg: Odometry) -> None:
        self._wheel_v = float(msg.twist.twist.linear.x)
        self._wheel_t = self._now()

    def _model_speed(self, t: float, dt: float) -> float:
        """Forward speed before the learned ratio: encoders if fresh, else the command model."""
        v_cmd = self._motion.step(t, dt)
        if self._wheel_v is not None and self._now() - self._wheel_t <= float(
                self.get_parameter("wheel_odom_timeout_s").value):
            return self._wheel_v
        return v_cmd

    def _on_scan(self, msg: LaserScan) -> None:
        self._timing.value("odom.scan_age_s", self._now() - _stamp(msg))
        self._w, self._n_facing = scan_observability(
            msg.ranges, msg.angle_min + self._laser_yaw, msg.angle_increment, **self._obs_args)
        self._obs_pub.publish(Float32(data=float(self._w)))

    def _on_rf2o(self, msg: Odometry) -> None:
        t = _stamp(msg)
        self._timing.value("odom.rf2o_age_s", self._now() - t)
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        pth = _yaw(msg.pose.pose.orientation)
        prev, self._prev_rf2o = self._prev_rf2o, (t, px, py, pth)
        if prev is None:
            self._publish(t, 0.0, 0.0, 0.0)
            return
        dt = t - prev[0]
        self._timing.value("odom.rf2o_period_s", dt)
        if dt <= 0.0 or dt > 1.0:
            self._timing.count("odom.rf2o_gap_skipped")
            return
        # rf2o motion in its previous body frame
        c, s = math.cos(prev[3]), math.sin(prev[3])
        ddx, ddy = px - prev[1], py - prev[2]
        dx_b, dy_b = c * ddx + s * ddy, -s * ddx + c * ddy
        dth = _wrap(pth - prev[3])

        v_model = self._model_speed(t, dt)
        self._kf.predict(dt)
        if self._motion.standing and abs(v_model) < 1e-3:
            fwd, side, dth = 0.0, 0.0, 0.0          # the car is not moving: ignore rf2o noise
        else:
            w = self._w
            if self._motion.cruising and v_model >= 0.1:
                # rf2o's speed as a ratio measurement; its noise in ratio units grows as v drops
                vn = max(self._drive.cruise_speed_mps, 0.05)
                if not self._kf.update_rf2o(dx_b / dt / v_model, w, r_scale=(vn / v_model) ** 2):
                    w = 0.0                          # rf2o disagrees too much: drive on the model
                    self._n_rejected += 1
            fwd = fuse_forward(dx_b, v_model * self._kf.v * dt, w)
            side = dy_b
        self._blind_m += (1.0 - (self._w if not self._motion.standing else 1.0)) * abs(fwd)
        mid = self._th + 0.5 * dth
        self._x += fwd * math.cos(mid) - side * math.sin(mid)
        self._y += fwd * math.sin(mid) + side * math.cos(mid)
        self._th = _wrap(self._th + dth)
        self._hist.append((t, self._x, self._y, self._th))
        self._publish(t, fwd / dt, side / dt, dth / dt)

        if t - self._last_log > 5.0:
            self._last_log = t
            self.get_logger().info(
                f"speed ratio {self._kf.v:.3f} (+-{math.sqrt(self._kf.var):.3f}), scan observability "
                f"{self._w:.2f} ({self._n_facing} facing points), blind {self._blind_m:.1f} m, "
                f"rf2o outliers {self._n_rejected}")

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        """AMCL correction along the heading -> speed measurement."""
        if not self._hist:
            return
        t = _stamp(msg)
        o = min(self._hist, key=lambda h: abs(h[0] - t))
        if abs(o[0] - t) > 0.3:
            return
        mx, my = msg.pose.pose.position.x, msg.pose.pose.position.y
        mth = _yaw(msg.pose.pose.orientation)
        # map -> odom = map_base * inverse(odom_base)
        cth = _wrap(mth - o[3])
        cx = mx - (math.cos(cth) * o[1] - math.sin(cth) * o[2])
        cy = my - (math.sin(cth) * o[1] + math.cos(cth) * o[2])
        prev, self._prev_corr = self._prev_corr, (cx, cy, cth)
        if prev is None:
            return
        # where the old and the new correction put the car now: the shift along its heading
        def to_map(c):
            return (c[0] + math.cos(c[2]) * o[1] - math.sin(c[2]) * o[2],
                    c[1] + math.sin(c[2]) * o[1] + math.cos(c[2]) * o[2])
        (ax, ay), (bx, by) = to_map(prev), to_map(self._prev_corr)
        if math.hypot(bx - ax, by - ay) > 3.0:      # re-initialised, not a correction
            self._blind_m = self._along_acc = 0.0
            return
        self._along_acc += (bx - ax) * math.cos(mth) + (by - ay) * math.sin(mth)
        g = lambda n: float(self.get_parameter(n).value)  # noqa: E731
        if (self._blind_m >= g("correction_min_blind_m") and abs(self._along_acc) >= g("correction_min_along_m")) \
                or self._blind_m >= g("correction_max_blind_m"):
            v0 = self._kf.v
            self._kf.update_correction(self._along_acc, self._blind_m)
            self.get_logger().info(
                f"AMCL moved the car {self._along_acc:+.2f} m along its heading after {self._blind_m:.1f} m "
                f"blind: speed ratio {v0:.3f} -> {self._kf.v:.3f}")
            self._blind_m = self._along_acc = 0.0

    # --------------------------------------------------------------- output
    def _publish(self, t: float, vx: float, vy: float, wz: float) -> None:
        sec = int(t)
        nsec = int((t - sec) * 1e9)
        qz, qw = math.sin(self._th / 2.0), math.cos(self._th / 2.0)
        o = Odometry()
        o.header.stamp.sec, o.header.stamp.nanosec = sec, nsec
        o.header.frame_id, o.child_frame_id = self._odom_frame, self._base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self._x, self._y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        # forward position gets less certain while driving blind
        o.pose.covariance[0] = o.pose.covariance[7] = 0.01 + (0.1 * self._blind_m) ** 2
        o.pose.covariance[35] = 0.01
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        self._odom_pub.publish(o)
        self._speed_pub.publish(Float32(data=float(self._kf.v)))
        if self._tf is not None:
            tf = TransformStamped()
            tf.header = o.header
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x, tf.transform.translation.y = self._x, self._y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self._tf.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = AdaptiveOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
