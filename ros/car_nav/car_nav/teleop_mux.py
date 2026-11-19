"""Drive by hand from the web page: /cmd_vel_teleop + Nav2's /cmd_vel_nav -> /cmd_vel_mux.

    Nav2 / PD follower -> /cmd_vel_nav -+
                                        +-> teleop_mux -> /cmd_vel_mux -> collision_monitor -> /cmd_vel_guarded
    web page -> /cmd_vel_teleop --------+                                   -> safety_guard -> /cmd_vel

so manual commands pass the same collision monitor and safety guard (people / walls) as Nav2's.
The decisions are in teleop_logic.py: deadman (no command for 0.3 s -> zero), manual overrides Nav2
and cancels its goal, a new goal ends manual control, E-STOP overrides, no reverse unless
drive.allow_reverse. State (JSON, 5 Hz while manual, 1 Hz otherwise) on /teleop_mux/state.
Parameters: hold_s, max_speed_mps, max_yaw_rps, allow_reverse (defaults from car.yaml drive:).
"""
from __future__ import annotations

import json
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from . import config_loader
from .teleop_logic import TeleopConfig, TeleopMux

STEADY = Clock(clock_type=ClockType.STEADY_TIME)


class TeleopMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_mux")
        drive = (config_loader.load().get("drive") or {})
        self.declare_parameter("hold_s", 0.3)
        self.declare_parameter("max_speed_mps", float(drive.get("cruise_speed_mps", 0.4)))
        self.declare_parameter("max_yaw_rps", float(drive.get("max_yaw_rate_rps", 1.5)))
        self.declare_parameter("allow_reverse", bool(drive.get("allow_reverse", False)))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._mux = TeleopMux(TeleopConfig(hold_s=float(g("hold_s")), max_speed_mps=float(g("max_speed_mps")),
                                           max_yaw_rps=float(g("max_yaw_rps")),
                                           allow_reverse=bool(g("allow_reverse"))))
        self._pub = self.create_publisher(Twist, "/cmd_vel_mux", 10)
        self._state_pub = self.create_publisher(String, "/teleop_mux/state", 5)
        self._cancel = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self._seen_goals: set = set()
        self._last_state = 0.0
        self._last_json = ""
        self.create_subscription(Twist, "/cmd_vel_teleop", self._on_teleop, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav, 10)
        self.create_subscription(PoseStamped, "/goal_pose", lambda m: self._goal(), 10)
        self.create_subscription(Bool, "/ui/estop", self._on_estop, 10)
        qos = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._on_status, qos)
        self.create_timer(0.05, self._tick, clock=STEADY)   # wall time: the deadman is wall time too (sim time can run slow)
        self.get_logger().info(f"teleop_mux: /cmd_vel_teleop + /cmd_vel_nav -> /cmd_vel_mux (max {g('max_speed_mps')} m/s, "
                               f"{g('max_yaw_rps')} rad/s, reverse {'on' if g('allow_reverse') else 'off'})")

    def _now(self) -> float:
        return time.monotonic()

    def _on_teleop(self, m: Twist) -> None:
        if self._mux.on_teleop(self._now(), m.linear.x, m.angular.z):
            self.get_logger().info("manual control taken: cancelling the active Nav2 goal")
            self._cancel_goals()
            self._publish_state(force=True)

    def _on_nav(self, m: Twist) -> None:
        if self._mux.nav_passes(self._now()):
            self._pub.publish(m)

    def _goal(self) -> None:
        if self._mux.goal(self._now()):
            self.get_logger().info("a goal was sent: manual control ended")
        self._publish_state(force=True)

    def _on_status(self, msg: GoalStatusArray) -> None:
        ids = {bytes(s.goal_info.goal_id.uuid) for s in msg.status_list if s.status in (1, 2)}
        if ids - self._seen_goals:          # a goal from RViz / a mission / the page that we have not seen
            self._goal()
        self._seen_goals = ids | (self._seen_goals if len(self._seen_goals) < 50 else set())

    def _on_estop(self, m: Bool) -> None:
        self._mux.set_estop(self._now(), bool(m.data))
        self._publish_state(force=True)

    def _cancel_goals(self) -> None:
        if self._cancel.service_is_ready():
            self._cancel.call_async(CancelGoal.Request())     # zero id + zero stamp = every goal

    def _tick(self) -> None:
        now = self._now()
        cmd = self._mux.command(now)
        if cmd is not None:
            t = Twist()
            t.linear.x, t.angular.z = float(cmd[0]), float(cmd[1])
            self._pub.publish(t)
        self._publish_state()

    def _publish_state(self, force: bool = False) -> None:
        now = self._now()
        st = self._mux.state(now)
        period = 0.2 if st["manual"] else 1.0
        txt = json.dumps(st)
        if force or now - self._last_state >= period or txt != self._last_json:
            if not force and txt == self._last_json and now - self._last_state < period:
                return
            self._last_state, self._last_json = now, txt
            self._state_pub.publish(String(data=txt))


def main() -> None:
    rclpy.init()
    node = TeleopMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
