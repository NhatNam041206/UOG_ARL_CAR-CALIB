"""Tell AMCL where the simulated car starts (for runs without a mission).

The mission runner sets AMCL's initial pose itself. When the sim is
started for RViz / Gazebo by hand there is no mission, so AMCL has no
pose and the scan is drawn in the wrong place until someone clicks
"2D Pose Estimate". This node publishes the true start on /initialpose
every second until AMCL answers on /amcl_pose, then exits.

    ros2 run car_nav_sim initial_pose --ros-args -p start:="[1.0, 1.1, 0.0]"
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from . import sim_config
from .world import World


class InitialPose(Node):
    def __init__(self) -> None:
        super().__init__("sim_initial_pose")
        self.declare_parameter("world", "test_floor")
        self.declare_parameter("start", [0.0])
        start = list(self.get_parameter("start").value)
        if len(start) != 3:
            start = list(World.load(sim_config.resolve("worlds", self.get_parameter("world").value)).start)
        self.start = [float(v) for v in start]
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
        self.done = False
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_timer(1.0, self._send)

    def _send(self) -> None:
        if self.done:
            return
        x, y, yaw = self.start
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(3) ** 2
        msg.pose.covariance = cov
        self.pub.publish(msg)

    def _on_amcl(self, _msg) -> None:
        if not self.done:
            self.done = True
            x, y, yaw = self.start
            self.get_logger().info(f"AMCL initial pose set to the sim start ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)")


def main() -> None:
    rclpy.init()
    node = InitialPose()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
