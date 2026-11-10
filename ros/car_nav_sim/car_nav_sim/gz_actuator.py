"""Gazebo backend: "<angle>,<drive_code>" -> actuator model -> body twist.

Gazebo's VelocityControl moves the car at its origin (= base_link, the
front-axle centre), so the (v, omega) from the actuator model are sent
as-is on /model/car/cmd_vel (bridged to Gazebo).
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from car_nav import config_loader
from car_nav.actuation import table_from_cfg

from . import sim_config
from .actuator_model import ActuatorModel, TruthParams


class GzActuator(Node):
    def __init__(self) -> None:
        super().__init__("gz_actuator")
        car = config_loader.load()
        sim = sim_config.load_sim()
        self.model = ActuatorModel(table_from_cfg(car), TruthParams.from_dict(sim.get("truth") or {}))
        self.pub = self.create_publisher(Twist, "/model/car/cmd_vel", 10)
        self.create_subscription(String, "/car/raspi5/control", self._on_control, 50)
        self.dt = 0.02
        self.create_timer(self.dt, self._tick)  # sim time (use_sim_time:=true)
        self.get_logger().info(f"gazebo actuator: truth {self.model.params}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_control(self, msg: String) -> None:
        self.model.command(self._now(), msg.data)

    def _tick(self) -> None:
        v, w = self.model.step(self._now(), self.dt)
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.pub.publish(tw)


def main() -> None:
    rclpy.init()
    node = GzActuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
