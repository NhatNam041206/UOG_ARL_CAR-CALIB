"""ROS side of idle_logic: an IdleTracker fed by what could make the car move.

Used by dynamic_scan, depth_guard_source and ui_bridge. Wakes (activity) on

* a non-zero /cmd_vel_guarded (Nav2 / collision monitor output) or /cmd_vel (guard output),
* a goal becoming active (/navigate_to_pose/_action/status: accepted or executing),
* the E-STOP changing (/ui/estop),

and counts as idle when none of that happened for `idle_after_s` and no goal is active.
Parameters (declared on the node): idle_pause (true), idle_after_s (10), idle_period_s (0.5).
Localization is never paused by this.
"""
from __future__ import annotations

import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .idle_logic import IdleTracker, is_nonzero, should_process

GOAL_STATUS_TOPIC = "/navigate_to_pose/_action/status"
ACTIVE_STATUSES = (1, 2, 3)   # GoalStatus ACCEPTED, EXECUTING, CANCELING


class IdleWatcher:
    def __init__(self, node, default_after_s: float = 10.0, default_period_s: float = 0.5) -> None:
        self._node = node
        node.declare_parameter("idle_pause", True)
        node.declare_parameter("idle_after_s", default_after_s)
        node.declare_parameter("idle_period_s", default_period_s)
        self._tracker = IdleTracker(float(node.get_parameter("idle_after_s").value))
        self._last_processed = -1e9
        self.was_idle = False
        node.create_subscription(Twist, "/cmd_vel_guarded", self._on_cmd, 10)
        node.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        node.create_subscription(Bool, "/ui/estop", lambda m: self._tracker.activity(), 10)
        qos = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(GoalStatusArray, GOAL_STATUS_TOPIC, self._on_status, qos)

    def _on_cmd(self, msg: Twist) -> None:
        if is_nonzero(msg.linear.x, msg.angular.z):
            self._tracker.activity()

    def _on_status(self, msg: GoalStatusArray) -> None:
        self._tracker.set_goal(any(s.status in ACTIVE_STATUSES for s in msg.status_list))

    def idle(self) -> bool:
        if not bool(self._node.get_parameter("idle_pause").value):
            return False
        self._tracker.idle_after_s = float(self._node.get_parameter("idle_after_s").value)
        idle = self._tracker.idle()
        if idle != self.was_idle:
            self.was_idle = idle
            self._node.get_logger().info("car idle: slow processing" if idle else "car active: full rate")
        return idle

    def process_now(self, min_period_s: float = 0.0) -> bool:
        """True when this frame should be processed (every frame while active, one per idle_period_s idle)."""
        now = time.monotonic()
        if should_process(self.idle(), now, self._last_processed,
                          float(self._node.get_parameter("idle_period_s").value), min_period_s):
            self._last_processed = now
            return True
        return False
