"""PD path follower: a FollowPath action server that replaces Pure Pursuit.

The behaviour tree (behavior_trees/navigate_forward_only_pd.xml) sends the
planned path to the action `pd_follow_path` (nav2_msgs/action/FollowPath)
instead of controller_server. Every 1/rate_hz this node looks up the car pose
(map -> base_link, from AMCL), projects it onto the path and sends

    cmd_vel_nav:  linear.x = speed_mps,  angular.z = speed_mps * kappa_cmd

with kappa_cmd from the PD law in pd_follower.py. The collision monitor and
cmd_vel_to_control downstream are unchanged. Gains are ROS parameters
(nav2_params.yaml, section pd_path_follower) so they can be tuned without a
rebuild.

Stops the car (zero cmd_vel -> STOP) while the pose map -> base_link is
older than max_pose_age_s, e.g. when the computer is overloaded and odometry
/ AMCL fall behind; it drives on once the pose is fresh again.

Done when the car is within xy_goal_tolerance of the path end, or has passed
it (remaining path < tolerance). Aborts when it moves less than
progress_radius_m in progress_time_s (stuck), like Nav2's progress checker.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .pd_follower import PathTracker, PDGains, pd_curvature


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PDPathFollower(Node):
    def __init__(self) -> None:
        super().__init__("pd_path_follower")
        p = self.declare_parameter
        p("kp", 0.7)
        p("kd", 1.2)
        p("ff", 1.0)
        p("ff_ahead_m", 0.3)
        p("speed_mps", 0.5)
        p("rate_hz", 10.0)
        p("xy_goal_tolerance", 0.5)
        p("progress_radius_m", 0.3)
        p("progress_time_s", 10.0)
        p("global_frame", "map")
        p("robot_frame", "base_link")
        p("max_pose_age_s", 1.0)
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._lock = threading.Lock()
        self._active = None  # goal_handle currently driving
        self._srv = ActionServer(
            self, FollowPath, "pd_follow_path", execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"PD path follower: kp {self._f('kp')} 1/m^2, kd {self._f('kd')} 1/m, ff {self._f('ff')}, "
            f"{self._f('speed_mps')} m/s")

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_accepted(self, goal_handle) -> None:
        # A new path (the tree replans at 1 Hz) replaces the one being driven.
        with self._lock:
            self._active = goal_handle
        goal_handle.execute()

    def _pose(self):
        try:
            t = self._tf.lookup_transform(self.get_parameter("global_frame").value,
                                          self.get_parameter("robot_frame").value, Time(),
                                          timeout=Duration(seconds=0.1))
        except Exception as exc:  # noqa: BLE001 - tf2 raises several types
            self.get_logger().warn(f"no pose: {exc}", throttle_duration_sec=2.0)
            return None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > self._f("max_pose_age_s"):
            # Steering on an old pose turns the car long after it has turned
            # (seen in the sim: a 6-8 s backlog drove it into a wall). Stop.
            self.get_logger().warn(f"pose is {age:.2f} s old (> {self._f('max_pose_age_s')} s): stopping",
                                   throttle_duration_sec=1.0)
            return None
        tr = t.transform.translation
        return tr.x, tr.y, _yaw(t.transform.rotation)

    def _stop(self) -> None:
        self._pub.publish(Twist())

    def _result(self, code: int = 0, msg: str = ""):
        res = FollowPath.Result()
        if hasattr(res, "error_code"):
            res.error_code = code
        if msg and hasattr(res, "error_msg"):
            res.error_msg = msg
        return res

    def _execute(self, goal_handle):
        path = goal_handle.request.path
        frame = path.header.frame_id or self.get_parameter("global_frame").value
        if frame != self.get_parameter("global_frame").value:
            self.get_logger().error(f"path frame {frame} != {self.get_parameter('global_frame').value}")
        if not path.poses:
            goal_handle.abort()
            return self._result(1, "empty path")
        tracker = PathTracker([(ps.pose.position.x, ps.pose.position.y) for ps in path.poses])
        gains = PDGains(kp=self._f("kp"), kd=self._f("kd"), ff=self._f("ff"), ff_ahead_m=self._f("ff_ahead_m"))
        v = self._f("speed_mps")
        tol = self._f("xy_goal_tolerance")
        period = 1.0 / self._f("rate_hz")
        gx, gy = tracker.pts[-1]
        anchor, anchor_t = None, self.get_clock().now()
        while rclpy.ok():
            with self._lock:
                superseded = self._active is not goal_handle
            if superseded:
                goal_handle.abort()   # the newer path's goal is driving now
                return self._result()
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                return self._result()
            pose = self._pose()
            if pose is None:
                self._stop()
                time.sleep(period)
                continue
            x, y, th = pose
            tr = tracker.track(x, y, th, gains)
            dist_goal = math.hypot(gx - x, gy - y)
            if dist_goal < tol or tr.remaining < 0.05:
                self._stop()
                goal_handle.succeed()
                return self._result()
            now = self.get_clock().now()
            if anchor is None or math.hypot(x - anchor[0], y - anchor[1]) > self._f("progress_radius_m"):
                anchor, anchor_t = (x, y), now
            elif (now - anchor_t).nanoseconds * 1e-9 > self._f("progress_time_s"):
                self._stop()
                self.get_logger().error("no progress: car moved less than "
                                        f"{self._f('progress_radius_m')} m in {self._f('progress_time_s')} s")
                goal_handle.abort()
                return self._result(105, "failed to make progress")
            kappa = pd_curvature(tr, gains)
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = v * kappa
            self._pub.publish(cmd)
            fb = FollowPath.Feedback()
            fb.distance_to_goal = float(dist_goal)
            fb.speed = float(v)
            goal_handle.publish_feedback(fb)
            self.get_logger().debug(f"e {tr.e:+.2f} m psi {math.degrees(tr.psi_e):+.0f} deg "
                                    f"kpath {tr.kappa_path:+.2f} -> k {kappa:+.2f}")
            time.sleep(period)
        return self._result()


def main() -> None:
    rclpy.init()
    node = PDPathFollower()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
