"""Perfect odometry from the simulator's true pose (odom_source:=truth).

Publishes TF odom -> base_link = true pose relative to the start pose, so
Nav2 can be tested without rf2o drift. The default (rf2o) is what the
real car will use.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TruthOdomTf(Node):
    def __init__(self) -> None:
        super().__init__("truth_odom_tf")
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.origin = None
        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, 20)

    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        th = 2 * math.atan2(p.orientation.z, p.orientation.w)
        if self.origin is None:
            self.origin = (p.position.x, p.position.y, th)
        x0, y0, th0 = self.origin
        dx, dy = p.position.x - x0, p.position.y - y0
        c, s = math.cos(-th0), math.sin(-th0)
        x, y, yaw = c * dx - s * dy, s * dx + c * dy, th - th0
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.br.sendTransform(t)
        o = Odometry()
        o.header = t.header
        o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation = t.transform.rotation
        o.twist = msg.twist
        self.odom_pub.publish(o)


def main() -> None:
    rclpy.init()
    node = TruthOdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
