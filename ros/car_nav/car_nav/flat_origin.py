"""Unknown / flat mode: map == odom, with the origin where the car is when the mode starts.

Takes odom -> base_link once, then publishes map -> odom (20 Hz, a normal /tf
transform, never /tf_static: tf listeners keep static frames for good) so that
map -> base_link is (0, 0, 0) at that moment: x axis = the way the car faces,
goals are relative to the start. From then on map -> base_link follows the
odometry alone (no map, nothing corrects it).
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .nav_modes import inverse_pose2d


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FlatOrigin(Node):
    def __init__(self) -> None:
        super().__init__("flat_origin")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 20.0)
        self._map = self.get_parameter("map_frame").value
        self._odom = self.get_parameter("odom_frame").value
        self._base = self.get_parameter("base_frame").value
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._br = TransformBroadcaster(self)
        self._inv = None
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _tick(self) -> None:
        if self._inv is None:
            try:
                t = self._tf.lookup_transform(self._odom, self._base, Time()).transform
            except Exception:  # noqa: BLE001 - odometry not up yet
                return
            x, y, yaw = t.translation.x, t.translation.y, _yaw(t.rotation)
            self._inv = inverse_pose2d(x, y, yaw)
            self.get_logger().info(f"flat mode: origin = the car at odom ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)")
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = self._map, self._odom
        msg.transform.translation.x, msg.transform.translation.y = self._inv[0], self._inv[1]
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(self._inv[2] / 2), math.cos(self._inv[2] / 2)
        self._br.sendTransform(msg)


def main() -> None:
    rclpy.init()
    node = FlatOrigin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
