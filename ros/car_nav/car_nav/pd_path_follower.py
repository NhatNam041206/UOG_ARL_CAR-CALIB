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

Wall avoidance (wall_avoid.py, avoid_enabled): before each command the car's
footprint is swept along the PD's arc on the latest scan; only if that arc
hits a wall within avoid_horizon_m does a clear arc from full right to full
left replace it (hysteresis on the way back). Needs no pose: the safety net
for a wrong pose, without fighting planned turns.

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
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from . import config_loader
from .pd_follower import PathTracker, PDGains, pd_curvature
from .actuation import table_from_cfg
from .wall_avoid import ArcAvoider, AvoidParams, points_in_base
from .tf_chain import PlanarTfChain
from .timing import TimingStats


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
        p("avoid_enabled", True)
        p("avoid_margin_m", 0.05)
        p("avoid_horizon_m", 1.3)
        p("avoid_release_factor", 1.3)
        p("avoid_candidates", 13)
        p("avoid_max_scan_age_s", 0.5)
        cfg = config_loader.load()
        fp = config_loader.footprint(cfg)
        table = table_from_cfg(cfg)
        self._avoid = AvoidParams(
            x_front=fp.x_front, x_rear=fp.x_rear, half_width=fp.half_width,
            margin_m=self._f("avoid_margin_m"), horizon_m=self._f("avoid_horizon_m"),
            release_factor=self._f("avoid_release_factor"),
            max_left=abs(table.max_left_curvature), max_right=abs(table.max_right_curvature),
            n_candidates=int(self.get_parameter("avoid_candidates").value))
        self._avoider = ArcAvoider(self._avoid)
        self._points: list = []
        self._points_t = 0.0
        # map -> base_link straight from /tf (tf_chain.py: same answer as tf2's latest
        # lookup, without the tf2 Python buffer, which cost most of this node's CPU)
        self._chain = PlanarTfChain(self.get_parameter("global_frame").value, "odom",
                                    self.get_parameter("robot_frame").value)
        self._chain_lock = threading.Lock()
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100, callback_group=ReentrantCallbackGroup())
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        # /scan_nav is in laser_upright = base_link shifted to the laser, turned by its yaw
        self._laser = config_loader.laser_pose(cfg)
        self._timing = TimingStats(self.get_logger())
        self._scan_stamp = None
        self._last_cmd_t = None
        self.create_subscription(LaserScan, "/scan_nav", self._timing.wrap("pd.scan_cb", self._on_scan), qos_profile_sensor_data,
                                 callback_group=ReentrantCallbackGroup())
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

    def _on_scan(self, msg: LaserScan) -> None:
        reach = self._avoid.horizon_m * self._avoid.release_factor + 1.0
        self._points = points_in_base(msg.ranges, msg.angle_min, msg.angle_increment, self._laser.x,
                                      self._laser.y, self._laser.yaw, max_range=reach + 0.5)
        self._points_t = time.monotonic()
        self._scan_stamp = Time.from_msg(msg.header.stamp)

    def _avoid_kappa(self, kappa_pd: float) -> float:
        if not self.get_parameter("avoid_enabled").value:
            return kappa_pd
        if time.monotonic() - self._points_t > self._f("avoid_max_scan_age_s"):
            self._avoider.reset()
            return kappa_pd
        was = self._avoider.override_k
        ch = self._avoider.step(kappa_pd, self._points)
        if ch.overriding and (was is None or abs(ch.kappa - was) > 1e-6):
            self.get_logger().info(
                f"wall avoid: PD arc {kappa_pd:+.2f} 1/m hits a wall after {ch.pd_free_m:.2f} m, "
                f"driving arc {ch.kappa:+.2f} 1/m (clear {ch.chosen_free_m:.2f} m)")
        elif not ch.overriding and was is not None:
            self.get_logger().info("wall avoid: PD arc clear again, back to the path")
        return ch.kappa

    def _on_accepted(self, goal_handle) -> None:
        # A new path (the tree replans at 1 Hz) replaces the one being driven.
        with self._lock:
            self._active = goal_handle
        goal_handle.execute()

    def _on_tf(self, msg: TFMessage) -> None:
        with self._chain_lock:
            for t in msg.transforms:
                st = t.header.stamp
                q = t.transform.rotation
                self._chain.add(t.header.frame_id, t.child_frame_id, st.sec * 1_000_000_000 + st.nanosec,
                                t.transform.translation.x, t.transform.translation.y, _yaw(q))

    def _pose(self):
        with self._chain_lock:
            latest = self._chain.latest()
        if latest is None:
            self.get_logger().warn("no pose: map -> odom -> base_link not available yet",
                                   throttle_duration_sec=2.0)
            return None
        t_ns, px, py, pth = latest
        age = (self.get_clock().now().nanoseconds - t_ns) * 1e-9
        self._timing.value("pd.pose_age_s", age)
        if age > self._f("max_pose_age_s"):
            # Steering on an old pose turns the car long after it has turned
            # (seen in the sim: a 6-8 s backlog drove it into a wall). Stop.
            self.get_logger().warn(f"pose is {age:.2f} s old (> {self._f('max_pose_age_s')} s): stopping",
                                   throttle_duration_sec=1.0)
            return None
        return px, py, pth

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
            t_tick = time.perf_counter()
            pose = self._pose()
            if pose is None:
                self._timing.count("pd.tick_no_pose")
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
            t_arc = time.perf_counter()
            kappa = self._avoid_kappa(pd_curvature(tr, gains))
            self._timing.duration("pd.arc_check", time.perf_counter() - t_arc)
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = v * kappa
            self._pub.publish(cmd)
            now_c = self.get_clock().now()
            self._timing.duration("pd.tick", time.perf_counter() - t_tick)
            if self._last_cmd_t is not None:
                self._timing.value("pd.cmd_period_s", (now_c - self._last_cmd_t).nanoseconds * 1e-9)
            self._last_cmd_t = now_c
            if self._scan_stamp is not None:
                self._timing.value("pd.scan_to_cmd_s", (now_c - self._scan_stamp).nanoseconds * 1e-9)
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
    # 2 threads: one runs the FollowPath loop, one serves scan / tf / odom callbacks
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
