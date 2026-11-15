"""Drive a mission (list of goals) through Nav2 and score it against the truth.

    ros2 run car_nav_sim mission --ros-args -p mission:=loop_left -p report_dir:=/data/sim

For every goal it records: Nav2 result, time, distance driven, smallest
clearance between the car's footprint and any obstacle (0 = collision),
final position / heading error, and the worst localisation error (AMCL
pose vs. true pose). It also counts how often the steering command sat at
a servo limit. The report is written as JSON and summarised in the log;
the process exits 0 only if every goal succeeded without a collision.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

try:  # Jazzy nav2_msgs
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover
    CollisionMonitorState = None
from std_msgs.msg import String
import yaml

from car_nav import config_loader
from car_nav.actuation import parse_payload

from . import sim_config
from .world import World, footprint_polygon


TRACE_DT = 0.5


def _yaw(q) -> float:
    return 2.0 * math.atan2(q.z, q.w)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _nav_error(nav: BasicNavigator) -> str | None:
    """Error code + message of the last NavigateToPose result (Jazzy), if any."""
    try:
        res = nav.result_future.result().result
    except Exception:  # noqa: BLE001 - older/newer BasicNavigator, cancelled goal
        return None
    code = getattr(res, "error_code", 0)
    msg = getattr(res, "error_msg", "")
    return f"{code} {msg}".strip() if code else None


def _pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Tracker:
    def __init__(self, world: World, footprint, steer_cfg: dict) -> None:
        self.world = world
        self.fp = footprint
        self.servo_min = float(steer_cfg.get("servo_min", 35))
        self.servo_max = float(steer_cfg.get("servo_max", 140))
        self.truth = None
        self.amcl = None
        self.tf = None  # tf2 Buffer, set in main(): map -> base_link is what Nav2 drives by
        self._loc_t = -math.inf
        self.start = None  # true pose when the odom frame was born (first truth msg)
        self.reset()

    def reset(self) -> None:
        self.dist = 0.0
        self.min_clear = math.inf
        self.max_loc_err = 0.0
        self.n_cmd = 0
        self.n_sat = 0
        self.n_forward = 0
        self.max_odom_drift = 0.0
        self.cm = {}  # collision monitor action -> number of state messages
        self.n_cm = 0
        self._last = None
        self.trace = []  # one row every TRACE_DT s of sim time, for debugging failed goals
        self._trace_t = -math.inf
        self.last_cmd = None
        self.cm_state = ""

    def on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        x, y, th = p.position.x, p.position.y, _yaw(p.orientation)
        if self._last is not None:
            self.dist += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        self.truth = (x, y, th)
        if self.start is None:
            self.start = self.truth
        poly = footprint_polygon(x, y, th, self.fp.x_front, self.fp.x_rear, self.fp.half_width)
        clear = self.world.clearance(poly)
        self.min_clear = min(self.min_clear, clear)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self._loc_t >= 0.1:
            self._loc_t = t
            est_pose = self.estimate()
            if est_pose is not None:
                self.max_loc_err = max(self.max_loc_err, math.hypot(est_pose[0] - x, est_pose[1] - y))
        if t - self._trace_t >= TRACE_DT:
            self._trace_t = t
            e = self.estimate()
            est = [round(e[0], 2), round(e[1], 2), round(math.degrees(e[2]))] if e else None
            self.trace.append({
                "t": round(t, 1), "true": [round(x, 2), round(y, 2), round(math.degrees(th))],
                "est": est, "cmd": self.last_cmd, "cm": self.cm_state, "clear": round(clear, 2),
            })

    def estimate(self):
        """Where Nav2 thinks the car is NOW: map -> base_link (AMCL + odometry since its
        last update). /amcl_pose alone is only published every ~0.1 m and lags."""
        if self.tf is not None:
            try:
                tr = self.tf.lookup_transform("map", "base_link", Time())
                q = tr.transform.rotation
                return (tr.transform.translation.x, tr.transform.translation.y, _yaw(q))
            except Exception:  # noqa: BLE001 - TF not ready yet
                pass
        return self.amcl

    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self.amcl = (p.position.x, p.position.y, _yaw(p.orientation))

    def on_odom(self, msg: Odometry) -> None:
        """Odometry drift: /odom (rf2o or truth) mapped through the start pose vs. truth."""
        if self.start is None or self.truth is None:
            return
        p = msg.pose.pose
        x0, y0, th0 = self.start
        c, s = math.cos(th0), math.sin(th0)
        wx = x0 + c * p.position.x - s * p.position.y
        wy = y0 + s * p.position.x + c * p.position.y
        self.max_odom_drift = max(self.max_odom_drift, math.hypot(wx - self.truth[0], wy - self.truth[1]))

    def on_cm(self, msg) -> None:
        names = {0: "none", 1: "stop", 2: "slowdown", 3: "approach", 4: "limit"}
        key = f"{names.get(msg.action_type, msg.action_type)}:{msg.polygon_name}"
        self.cm[key] = self.cm.get(key, 0) + 1
        self.cm_state = key
        self.n_cm += 1

    def on_control(self, msg: String) -> None:
        parsed = parse_payload(msg.data)
        if parsed is None:
            return
        angle, code = parsed
        self.last_cmd = msg.data
        self.n_cmd += 1
        if code == 1:
            self.n_forward += 1
            if angle <= self.servo_min or angle >= self.servo_max:
                self.n_sat += 1


def main() -> None:
    rclpy.init()
    nav = BasicNavigator(node_name="sim_mission")
    nav.declare_parameter("mission", "loop_left")
    nav.declare_parameter("report_dir", "/data/sim")
    nav.declare_parameter("localizer", "amcl")  # "amcl" or "slam"
    nav.declare_parameter("start", [0.0])
    nav.declare_parameter("odom_source", "")
    mission_path = sim_config.resolve("missions", nav.get_parameter("mission").value)
    with open(mission_path, "r", encoding="utf-8") as fh:
        mission = yaml.safe_load(fh) or {}
    world = World.load(sim_config.resolve("worlds", mission.get("world", "test_floor")))
    start_param = list(nav.get_parameter("start").value)
    start = start_param if len(start_param) == 3 else (mission.get("start") or world.start)
    car = config_loader.load()
    tracker = Tracker(world, config_loader.footprint(car), car.get("steering") or {})
    tracker.tf = Buffer()
    # Own thread: the mission loop spins `nav` only between isTaskComplete() polls, so a
    # listener on it fell seconds behind and the "est" pose / localisation error lagged
    # (sim 2026-11-16: 1.5-2.5 m "error" while the planner's start pose was 0.1 m off).
    _tf_listener = TransformListener(tracker.tf, nav, spin_thread=True)  # noqa: F841
    nav.create_subscription(Odometry, "/sim/ground_truth", tracker.on_truth, 20)
    nav.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", tracker.on_amcl, 10)
    nav.create_subscription(String, "/car/raspi5/control", tracker.on_control, 50)
    nav.create_subscription(Odometry, "/odom", tracker.on_odom, 20)
    if CollisionMonitorState is not None:
        nav.create_subscription(CollisionMonitorState, "/collision_monitor_state", tracker.on_cm, 20)

    localizer = nav.get_parameter("localizer").value
    if localizer == "amcl":
        nav.setInitialPose(_pose(nav, *start))
        nav.waitUntilNav2Active(localizer="amcl")
    else:
        # slam_toolbox owns map->odom; nothing to initialise. "robot_localization"
        # makes BasicNavigator skip the localizer lifecycle check.
        nav.waitUntilNav2Active(localizer="robot_localization")

    # Wait for the truth stream and check the sim started where we think.
    t0 = time.monotonic()
    while tracker.truth is None and time.monotonic() - t0 < 30:
        rclpy.spin_once(nav, timeout_sec=0.1)
    if tracker.truth is None:
        nav.get_logger().error("no /sim/ground_truth: is the simulator running?")
        sys.exit(3)
    if math.hypot(tracker.truth[0] - start[0], tracker.truth[1] - start[1]) > 0.15:
        nav.get_logger().warn(f"true start {tracker.truth} differs from mission start {start}")

    timeout = float(mission.get("goal_timeout_s", 90))
    results = []
    for i, g in enumerate(mission.get("goals") or []):
        gx, gy, gyaw = (float(v) for v in g)
        tracker.reset()
        sim_t0 = nav.get_clock().now().nanoseconds * 1e-9
        nav.goToPose(_pose(nav, gx, gy, gyaw))
        timed_out = False
        while not nav.isTaskComplete():
            if nav.get_clock().now().nanoseconds * 1e-9 - sim_t0 > timeout:
                nav.cancelTask()
                timed_out = True
        result = nav.getResult()
        err = _nav_error(nav)
        sim_dt = nav.get_clock().now().nanoseconds * 1e-9 - sim_t0
        # Let the car coast to a stop (latency + braking) before scoring the end pose.
        settle_t0 = nav.get_clock().now().nanoseconds * 1e-9
        while nav.get_clock().now().nanoseconds * 1e-9 - settle_t0 < 2.0:
            rclpy.spin_once(nav, timeout_sec=0.05)
        tx, ty, tth = tracker.truth
        entry = {
            "goal": [gx, gy, gyaw],
            "result": "TIMEOUT" if timed_out else {
                TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED", TaskResult.FAILED: "FAILED",
            }.get(result, str(result)),
            "nav2_error": err,  # Nav2 error code / message when the goal did not succeed
            "time_s": round(sim_dt, 2),
            "distance_m": round(tracker.dist, 2),
            "min_clearance_m": round(tracker.min_clear, 3),
            "collision": tracker.min_clear <= 0.0,
            "final_xy_error_m": round(math.hypot(tx - gx, ty - gy), 3),
            "final_yaw_error_deg": round(math.degrees(abs(_wrap(tth - gyaw))), 1),
            "max_localization_error_m": round(tracker.max_loc_err, 3),
            "steer_saturated_share": round(tracker.n_sat / tracker.n_forward, 3) if tracker.n_forward else 0.0,
            "forward_share": round(tracker.n_forward / tracker.n_cmd, 3) if tracker.n_cmd else 0.0,
            "max_odom_drift_m": round(tracker.max_odom_drift, 3),
            "collision_monitor": tracker.cm,
            "true_end_pose": [round(tx, 3), round(ty, 3), round(math.degrees(tth), 1)],
            # truth / AMCL pose, last command, collision-monitor state every 0.5 s
            "trace": tracker.trace,
        }
        results.append(entry)
        nav.get_logger().info(f"goal {i + 1}: {json.dumps({k: v for k, v in entry.items() if k != 'trace'})}")

    ok = bool(results) and all(r["result"] == "SUCCEEDED" and not r["collision"] for r in results)
    report = {
        "mission": os.path.basename(mission_path),
        "world": world.name,
        "localizer": localizer,
        "odom_source": nav.get_parameter("odom_source").value,
        "car_yaml": car.get("_path"),
        "sim_yaml": sim_config.load_sim().get("_path"),
        "truth": sim_config.load_sim().get("truth"),
        "passed": ok,
        "goals": results,
    }
    out_dir = nav.get_parameter("report_dir").value
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"report-{os.path.splitext(report['mission'])[0]}-{time.strftime('%Y%m%d-%H%M%S')}.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2)
    nav.get_logger().info(f"{'PASSED' if ok else 'FAILED'}: report {out}")
    nav.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)
