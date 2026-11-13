"""pose_gate: publishes map -> odom from AMCL, minus AMCL's guesses in blind corridors.

AMCL runs with tf_broadcast false. For every /amcl_pose this node:

  1. looks up the odometry pose (odom -> base_link) at the AMCL stamp,
  2. keeps AMCL's sideways and heading correction and only the share
     g = min(w / obs_full, 1) of its along-the-heading correction
     (w = /scan_observability from adaptive_odom; without it w = 1 and
     the gate passes AMCL through unchanged), see localization_gate.py,
  3. publishes map -> odom (TF, 20 Hz, future-dated by transform_tolerance
     like AMCL) and the gated car pose on /localized_pose (adaptive_odom
     learns the speed from it).

Reseed: while blind, when AMCL's cloud has wandered more than
reseed_along_m along the corridor away from the gated pose, the gated pose
is sent to AMCL on /initialpose (narrow across the corridor, a little wider
along it), so AMCL does not arrive at the next junction metres off.
An /initialpose from someone else (RViz, the mission) resets the gate: the
next AMCL pose is taken fully.
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .localization_gate import gate_correction, reseed_covariance


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseGate(Node):
    def __init__(self) -> None:
        super().__init__("pose_gate")
        p = self.declare_parameter
        p("global_frame", "map")
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("obs_full", 0.5)               # observability at which AMCL's along correction is taken fully
        p("publish_rate_hz", 20.0)
        p("transform_tolerance", 0.5)    # future-dating of map -> odom (s), like AMCL
        p("observability_timeout_s", 2.0)  # no /scan_observability for this long: w = 1 (pass-through)
        p("reseed_enabled", True)
        p("reseed_along_m", 0.3)
        p("reseed_min_interval_s", 2.0)
        p("reseed_sigma_along_m", 0.15)
        p("reseed_sigma_lateral_m", 0.08)
        p("reseed_sigma_yaw_rad", 0.05)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._map, self._odom, self._base = g("global_frame"), g("odom_frame"), g("base_frame")
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._br = TransformBroadcaster(self)
        self._map_odom = None
        self._w, self._w_t = 1.0, None
        self._last_reseed = 0.0
        self._sent_seed = None
        self._held_back = 0.0
        self._last_log = 0.0
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/localized_pose", 10)
        self._seed_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initialpose, 10)
        self.create_subscription(Float32, "/scan_observability", self._on_obs, 10)
        self.create_timer(1.0 / float(g("publish_rate_hz")), self._publish_tf)
        self.get_logger().info(
            f"pose gate: AMCL along-corridor corrections scaled by observability / {g('obs_full')}")

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_obs(self, msg: Float32) -> None:
        self._w, self._w_t = float(msg.data), time.monotonic()

    def _observability(self) -> float:
        if self._w_t is None or time.monotonic() - self._w_t > self._f("observability_timeout_s"):
            return 1.0
        return self._w

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose
        if self._sent_seed is not None and math.hypot(q.position.x - self._sent_seed[0],
                                                      q.position.y - self._sent_seed[1]) < 0.01:
            return  # our own reseed
        self.get_logger().info("initial pose set: next AMCL pose is taken fully")
        self._map_odom = None

    def _odom_pose(self, stamp):
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self._tf.lookup_transform(self._odom, self._base, t, timeout=Duration(seconds=0.1))
            except Exception:  # noqa: BLE001 - tf2 raises several types
                continue
            tr = tf.transform.translation
            return (tr.x, tr.y, _yaw(tf.transform.rotation))
        return None

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        odom = self._odom_pose(msg.header.stamp)
        if odom is None:
            self.get_logger().warn("no odom -> base_link for the AMCL pose", throttle_duration_sec=2.0)
            return
        q = msg.pose.pose
        amcl = (q.position.x, q.position.y, _yaw(q.orientation))
        w = self._observability()
        step = gate_correction(self._map_odom, odom, amcl, w, self._f("obs_full"))
        self._map_odom = step.map_odom
        held = (1.0 - step.taken) * step.along
        self._held_back = held
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self._map
        out.pose.pose.position.x, out.pose.pose.position.y = step.pose[0], step.pose[1]
        out.pose.pose.orientation.z = math.sin(step.pose[2] / 2.0)
        out.pose.pose.orientation.w = math.cos(step.pose[2] / 2.0)
        out.pose.covariance = msg.pose.covariance
        self._pose_pub.publish(out)
        self._publish_tf()

        now = time.monotonic()
        if (self.get_parameter("reseed_enabled").value and abs(held) > self._f("reseed_along_m")
                and now - self._last_reseed > self._f("reseed_min_interval_s")):
            self._last_reseed = now
            seed = PoseWithCovarianceStamped()
            seed.header = out.header
            seed.pose.pose = out.pose.pose
            seed.pose.covariance = reseed_covariance(step.pose[2], self._f("reseed_sigma_along_m"),
                                                     self._f("reseed_sigma_lateral_m"),
                                                     self._f("reseed_sigma_yaw_rad"))
            self._sent_seed = step.pose
            self._seed_pub.publish(seed)
            self.get_logger().info(f"blind corridor (observability {w:.2f}): AMCL drifted {held:+.2f} m along "
                                   "the corridor, reseeded it at the gated pose")
        if now - self._last_log > 5.0:
            self._last_log = now
            self.get_logger().info(f"observability {w:.2f}, AMCL along correction {step.along:+.2f} m "
                                   f"(taken {step.taken:.0%}, held back {self._held_back:+.2f} m)")

    def _publish_tf(self) -> None:
        if self._map_odom is None:
            return
        x, y, th = self._map_odom
        tf = TransformStamped()
        stamp = self.get_clock().now() + Duration(seconds=self._f("transform_tolerance"))
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id, tf.child_frame_id = self._map, self._odom
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(th / 2.0), math.cos(th / 2.0)
        self._br.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = PoseGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
