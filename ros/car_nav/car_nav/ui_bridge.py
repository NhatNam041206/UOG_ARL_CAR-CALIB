"""Web UI backend: serves the page (HTTP) and packs what it shows into two JSON topics.

The browser talks to ROS through rosbridge (WebSocket, port 9090, started
by nav.launch.py ui:=true). To keep the page simple and the traffic small,
this node does the ROS-side work:

* /ui/map    (std_msgs/String, every 3 s): the occupancy map, run-length
             encoded: {"w","h","res","ox","oy","rle":[value, count, ...]}
             values 0 = free, 100 = occupied, -1 = unknown
* /ui/state  (std_msgs/String, 5 Hz): car pose (TF map -> base_link), LiDAR
             points in the map frame (/scan_nav, and /scan_dynamic = not on
             the map), the depth camera's obstacle points ("depth", /depth_obstacles, map
             frame), the global plan, the safety guard state and zones.

The page publishes /initialpose, /goal_pose, /ui/estop and /ui/safety_params
itself (see web/index.html). HTTP: http://<car>:<http_port>/ (default 8080).

Profiles and test records (ui_store.py, JSON files under data_dir), REST:

  GET    /api/profiles                     list (name, note, updated, default)
  GET    /api/profiles/<name>              one profile {"guard": {...}, "drive": {...}, "note"}
  PUT    /api/profiles/<name>              create / overwrite (body = the profile)
  DELETE /api/profiles/<name>
  POST   /api/profiles/<name>/apply        send it to safety_guard and pd_path_follower now
  POST   /api/profiles/<name>/default      apply it every time the stack starts
  POST   /api/tests/start  {"name","note"} start recording (5 Hz samples + events)
  POST   /api/tests/stop                   stop and save; returns the summary
  GET    /api/tests/status                 recording or not
  GET    /api/tests                        list with summaries
  GET    /api/tests/<id>[?download=1]      the whole record (download = as a file)
  PUT    /api/tests/<id>   {"note","verdict"}
  DELETE /api/tests/<id>

Maps and map modes (map_store.py, nav_modes.py; maps in <data_dir>/maps):

  GET    /api/maps                         stored maps + the current mode
  GET    /api/maps/<name>                  one map (note, start_pose, size, ...)
  PUT    /api/maps/<name>   {"note", "start_pose": {x,y,yaw} | null, "rename": "new name"}
  DELETE /api/maps/<name>                  (not the map that is loaded now)
  POST   /api/maps/<name>/load  {"force"}  = POST /api/mode {"mode": "load", "map": name}
  POST   /api/maps/save {"name","note","overwrite"}   store the map slam_toolbox has built (Create mode)
  GET    /api/mode                         {"mode","map","state","detail","moving",...}
  POST   /api/mode {"mode": "load|create|flat|idle", "map", "force"}   restart localization + Nav2

Switching runs the restartable part (launch/nav_stack.launch.py) as a child process: it is
stopped and started again, everything else (sensors, odometry, cmd_vel_to_control, rosbridge,
this node) keeps running; zero /cmd_vel is sent meanwhile. 409 while the car is being driven
(wheel command not 0) unless "force": true. state: idle | switching | starting | ready | error.

"drive" values (speed_mps, max_yaw_rate_rps, lat_accel_mps2, kp, kd) are set
on pd_path_follower with its set_parameters service; they act from the next
goal on. /ui/drive_params (std_msgs/String, JSON) does the same live.
"""
from __future__ import annotations

import functools
import http.server
import json
import math
import os
import threading
import time
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from . import config_loader
from .actuation import WHEELS_TOPIC, parse_payload
from .map_store import MapStore, safe_map_name
from .nav_modes import ALL_MODES, MODES, StackProcess, stack_command, wheels_moving
from .depth_obstacles import parse_cloud
from .ui_pack import rle
from .ui_store import ProfileStore, TestRecorder, TestStore

DRIVE_KEYS = ("speed_mps", "max_yaw_rate_rps", "lat_accel_mps2", "kp", "kd")


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_to_map(msg: LaserScan, pose: tuple[float, float, float], max_range: float = 12.0, step: int = 2):
    r = np.asarray(msg.ranges, dtype=float)[::step]
    a = msg.angle_min + np.arange(0, len(msg.ranges), step) * msg.angle_increment + pose[2]
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r > 0.05) & (r <= max_range)
    x = pose[0] + r[ok] * np.cos(a[ok])
    y = pose[1] + r[ok] * np.sin(a[ok])
    return np.round(np.stack((x, y), axis=1), 2).tolist()


class UiBridge(Node):
    def __init__(self) -> None:
        super().__init__("ui_bridge")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("rosbridge_port", 9090)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("web_dir", "")
        self.declare_parameter("data_dir", "/data/nav/ui")   # profiles, test records, maps (a volume on the car)
        self.declare_parameter("stack_args", [""])          # launch arguments for nav_stack.launch.py
        self.declare_parameter("initial_mode", "idle")
        self.declare_parameter("initial_map", "")           # map yaml path (load)
        self.declare_parameter("initial_map_name", "")      # map of the store (load; its start pose is sent)
        self.declare_parameter("seed_map_yaml", "")         # simulator: map added to the store once
        self.declare_parameter("seed_map_name", "")
        self.declare_parameter("seed_start", "")
        data_dir = self.get_parameter("data_dir").value
        self._profiles = ProfileStore(data_dir)
        self._tests = TestStore(data_dir)
        self._maps = MapStore(data_dir)
        self._rec: TestRecorder | None = None
        self._rec_lock = threading.Lock()
        self._drive: dict = {}
        self._wheels = None
        self._guard_prev = None
        self._wheels_t = 0.0
        self._guard_last = None        # last /ui/safety_params, re-sent after a restart of the stack
        self._estop_last = False
        self._restore = False
        self._drive_restore = False
        self._grid = None              # (H x W int8, resolution, (ox, oy)) of the latest /map
        self._pose_try = None          # stored start pose being sent to AMCL after a load
        self._amcl_seen = 0.0
        self._create_start = None      # pose of the car in the map when Create started
        self._stack = StackProcess()
        self._sw_lock = threading.Lock()
        self._mode = {"mode": "idle", "map": None, "state": "idle", "detail": "", "since": time.time()}
        self._stack_args = list(self.get_parameter("stack_args").value or [])
        self._stop_cmd = False
        self._ready_ok = {"nav": False, "loc": False}
        self._default_applied = False
        cfg = config_loader.load()
        fp = config_loader.footprint(cfg)
        laser = config_loader.laser_pose(cfg)
        self._static = {"footprint": [fp.x_front, fp.x_rear, fp.half_width],
                        "laser": [laser.x, laser.y, laser.yaw]}
        self._frame = self.get_parameter("global_frame").value
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._map_json = None
        self._scan = self._dyn = None
        self._depth, self._depth_t = None, 0.0
        self._plan: list = []
        self._guard: dict = {}
        self._map_pub = self.create_publisher(String, "/ui/map", 1)
        self._state_pub = self.create_publisher(String, "/ui/state", 5)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_costmap, 1)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 lambda m: setattr(self, "_amcl_seen", time.monotonic()), 5)
        self._init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 5)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 5)
        self._nav_active = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
        self._loc_active = self.create_client(Trigger, "/lifecycle_manager_localization/is_active")
        self.create_timer(0.1, self._hold_stop)
        self.create_timer(1.0, self._supervise)
        self.create_subscription(LaserScan, "/scan_nav", lambda m: setattr(self, "_scan", m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_dynamic", lambda m: setattr(self, "_dyn", m),
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/depth_obstacles", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self._on_plan, 5)
        self.create_subscription(String, "/safety_guard/state", self._on_guard, 10)
        self.create_subscription(String, WHEELS_TOPIC, self._on_wheels, 20)
        self.create_subscription(PoseStamped, "/goal_pose", lambda m: self._event(
            "goal", x=round(m.pose.position.x, 2), y=round(m.pose.position.y, 2)), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", lambda m: self._event(
            "initial_pose", x=round(m.pose.pose.position.x, 2), y=round(m.pose.pose.position.y, 2)), 10)
        self.create_subscription(Bool, "/ui/estop", self._on_estop, 10)
        self.create_subscription(String, "/ui/safety_params", self._on_guard_params, 10)
        self.create_subscription(String, "/ui/drive_params", self._on_drive_params, 10)
        self._guard_params_pub = self.create_publisher(String, "/ui/safety_params", 10)
        self._estop_pub = self.create_publisher(Bool, "/ui/estop", 10)
        self._pd_params = self.create_client(SetParameters, "/pd_path_follower/set_parameters")
        self._pd_get = self.create_client(GetParameters, "/pd_path_follower/get_parameters")
        self._drive_timer = self.create_timer(2.0, self._read_drive)
        self.create_timer(3.0, self._publish_map)
        self.create_timer(0.2, self._publish_state)
        self._start_http()
        self._seed_maps()
        self._boot_timer = self.create_timer(1.0, self._boot)

    def _start_http(self) -> None:
        web = self.get_parameter("web_dir").value or os.path.join(get_package_share_directory("car_nav"), "web")
        port = int(self.get_parameter("http_port").value)
        handler = functools.partial(_ApiHandler, directory=web, bridge=self)
        try:
            srv = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
        except OSError as exc:
            self.get_logger().error(f"web UI: cannot listen on port {port}: {exc}")
            return
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.get_logger().info(f"web UI on http://<this computer>:{port}/  (rosbridge "
                               f"ws port {self.get_parameter('rosbridge_port').value}), files {web}")

    def _ingest_grid(self, msg: OccupancyGrid, occupied: int, keep: bool) -> None:
        if self._mode["state"] == "switching":
            return                                      # the old stack's last map
        data = np.asarray(msg.data, dtype=np.int16)
        data = np.where(data < 0, -1, np.where(data >= occupied, 100, 0))
        if keep:
            self._grid = (data.astype(np.int8).reshape(msg.info.height, msg.info.width),
                          float(msg.info.resolution),
                          (msg.info.origin.position.x, msg.info.origin.position.y))
        self._map_json = json.dumps({
            "w": msg.info.width, "h": msg.info.height, "res": msg.info.resolution,
            "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
            "rle": rle(data),
        })
        self._publish_map()

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._mode["mode"] in ("load", "create"):
            self._ingest_grid(msg, 50, True)

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        """Flat mode has no /map: the page shows the rolling global costmap (lethal cells = occupied)."""
        if self._mode["mode"] == "flat":
            self._ingest_grid(msg, 99, False)

    def _publish_map(self) -> None:
        if self._map_json is not None:
            self._map_pub.publish(String(data=self._map_json))

    def _on_plan(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        step = max(1, len(pts) // 300)
        self._plan = [[round(x, 2), round(y, 2)] for x, y in pts[::step] + pts[-1:]] if pts else []

    def _on_guard(self, msg: String) -> None:
        try:
            self._guard = json.loads(msg.data)
        except ValueError:
            return
        st = self._guard.get("state")
        if st != self._guard_prev:
            self._event("guard", to=st, nearest=self._guard.get("nearest_m"))
            self._guard_prev = st
        if not self._default_applied:      # the guard is up: apply the start profile once
            self._default_applied = True
            name = self._profiles.default()
            if name:
                self.get_logger().info(f"applying the default profile {name!r}")
                self.apply_profile(name)
            if self._restore:              # the stack was restarted: put back what the page had set
                self._restore = False
                if self._guard_last and not name:
                    self._guard_params_pub.publish(String(data=self._guard_last))
                if self._estop_last:
                    self._estop_pub.publish(Bool(data=True))
                self._drive_restore = bool(self._drive)

    def _on_wheels(self, msg: String) -> None:
        self._wheels = parse_payload(msg.data)
        self._wheels_t = time.monotonic()

    def _on_estop(self, msg: Bool) -> None:
        self._estop_last = bool(msg.data)
        self._event("estop", on=bool(msg.data))

    def _on_guard_params(self, msg: String) -> None:
        self._guard_last = msg.data
        self._event("guard_params", data=msg.data)

    # ------------------------------------------------------------- profiles
    def _on_drive_params(self, msg: String) -> None:
        try:
            self.set_drive(json.loads(msg.data))
        except ValueError:
            self.get_logger().warn(f"bad /ui/drive_params: {msg.data!r}")

    def _read_drive(self) -> None:
        """Once: the follower's current drive values, so the page starts from them."""
        if self._drive or not self._pd_get.service_is_ready():
            if self._drive:
                self._drive_timer.cancel()
            return
        fut = self._pd_get.call_async(GetParameters.Request(names=list(DRIVE_KEYS)))

        def done(f) -> None:
            try:
                vals = f.result().values
            except Exception:  # noqa: BLE001
                return
            for k, v in zip(DRIVE_KEYS, vals):
                if v.type == ParameterType.PARAMETER_DOUBLE:
                    self._drive.setdefault(k, v.double_value)
        fut.add_done_callback(done)

    def set_drive(self, d: dict) -> dict:
        vals = {k: float(d[k]) for k in DRIVE_KEYS if k in d}
        if not vals:
            return {}
        self._drive.update(vals)
        req = SetParameters.Request()
        for k, v in vals.items():
            req.parameters.append(ParamMsg(name=k, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                                        double_value=v)))
        if self._pd_params.service_is_ready():
            self._pd_params.call_async(req)
        else:
            self.get_logger().warn("pd_path_follower parameters not reachable (controller:=rpp?)")
        self._event("drive_params", data=vals)
        return vals

    def apply_profile(self, name: str) -> dict | None:
        prof = self._profiles.get(name)
        if prof is None:
            return None
        if prof.get("guard"):
            self._guard_params_pub.publish(String(data=json.dumps(prof["guard"])))
        if prof.get("drive"):
            self.set_drive(prof["drive"])
        self._event("profile_applied", name=prof.get("name"))
        return prof

    def current(self) -> dict:
        return {"guard": (self._guard or {}).get("params", {}), "drive": dict(self._drive)}

    # ------------------------------------------------------- map modes
    def _seed_maps(self) -> None:
        y = self.get_parameter("seed_map_yaml").value
        n = self.get_parameter("seed_map_name").value
        if not y or not n:
            return
        try:
            vals = [float(v) for v in str(self.get_parameter("seed_start").value).split(",") if v.strip()]
            start = {"x": vals[0], "y": vals[1], "yaw": vals[2] if len(vals) > 2 else 0.0} if len(vals) >= 2 else None
            if self._maps.seed(y, n, start_pose=start, note="the simulated world"):
                self.get_logger().info(f"map store: seeded {n!r} from {y}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"map store: cannot seed {n!r}: {exc!r}")

    def _boot(self) -> None:
        """Once, 1 s after start: start the mode given on the command line."""
        self._boot_timer.cancel()
        mode = str(self.get_parameter("initial_mode").value or "idle")
        if mode == "idle":
            return
        ymap = str(self.get_parameter("initial_map").value or "")
        name = str(self.get_parameter("initial_map_name").value or "")
        if mode == "load" and not ymap:
            if not self._maps.exists(name):
                self._set_mode("idle", None, "error", f"map {name!r} is not in the store")
                return
        threading.Thread(target=self._switch_worker, args=(mode, name or None, ymap or None, bool(name and not ymap)),
                         daemon=True).start()

    def moving(self) -> bool:
        return wheels_moving(self._wheels, time.monotonic() - self._wheels_t)

    def _set_mode(self, mode: str, map_name, state: str, detail: str = "") -> None:
        self._mode = {"mode": mode, "map": map_name, "state": state, "detail": detail, "since": time.time()}
        self.get_logger().info(f"map mode: {mode} map={map_name} {state} {detail}")
        self._event("map_mode", mode=mode, map=map_name, state=state)

    def mode_status(self) -> dict:
        m = dict(self._mode)
        m["moving"] = self.moving()
        m["modes"] = list(ALL_MODES)
        info = self._maps.get(m["map"]) if m["map"] and m["mode"] == "load" else None
        m["start_pose"] = info.get("start_pose") if info else None
        m["can_save"] = m["mode"] == "create" and self._grid is not None and m["state"] == "ready"
        return m

    def request_switch(self, mode: str, map_name: str | None, force: bool) -> dict:
        """Validate, then restart the stack in a worker thread (returns at once)."""
        if mode not in ALL_MODES:
            raise ValueError(f"mode must be one of {ALL_MODES}")
        if mode == "load":
            if not map_name:
                raise ValueError("load needs a map name")
            if not self._maps.exists(map_name):
                raise LookupError(f"no such map {map_name!r}")
            map_name = safe_map_name(map_name)
        else:
            map_name = None
        if self._mode["state"] in ("switching",) or self._sw_lock.locked():
            raise RuntimeError("already switching")
        if self.moving() and not force:
            raise RuntimeError("the car is being driven (wheel command not 0): stop it first, or force")
        threading.Thread(target=self._switch_worker, args=(mode, map_name, None, True), daemon=True).start()
        self._mode = {**self._mode, "state": "switching", "detail": f"to {mode}", "since": time.time()}
        return self.mode_status()

    def _switch_worker(self, mode: str, map_name, ymap, send_pose: bool) -> None:
        if not self._sw_lock.acquire(blocking=False):
            return
        try:
            self._set_mode(self._mode["mode"], self._mode["map"], "switching", f"to {mode}")
            self._stop_cmd = True                          # zero /cmd_vel while the stack is down
            self._pose_try = None
            self._map_json = None
            self._grid = None
            self._create_start = None
            self._ready_ok = {"nav": False, "loc": False}
            self._publish_empty_map()
            if self._stack.running():
                how = self._stack.stop()
                self.get_logger().info(f"nav stack stopped ({how})")
                time.sleep(1.0)                            # let DDS forget the old nodes
            self._guard = {}
            self._default_applied = False
            self._restore = True
            self._plan = []
            if mode == "idle":
                self._stop_cmd = False
                self._set_mode("idle", None, "idle", "no map mode running")
                return
            yaml_path = ymap
            if mode == "load" and not yaml_path:
                yaml_path = self._maps.yaml_path(map_name)
            label = map_name or (os.path.splitext(os.path.basename(ymap))[0] if ymap else None)
            self._set_mode(mode, label if mode == "load" else None, "starting", "")
            self._stack.start(stack_command(mode, yaml_path or "", self._stack_args))
            if mode == "load" and send_pose and map_name:
                info = self._maps.get(map_name) or {}
                if info.get("start_pose"):
                    self._pose_try = {"pose": info["start_pose"], "n": 0}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"switching to {mode} failed: {exc!r}")
            self._set_mode(mode, map_name, "error", repr(exc))
        finally:
            self._sw_lock.release()

    def _publish_empty_map(self) -> None:
        self._map_pub.publish(String(data=json.dumps({"w": 0, "h": 0, "res": 0.05, "ox": 0, "oy": 0, "rle": []})))

    def _hold_stop(self) -> None:
        if self._stop_cmd:
            self._cmd_pub.publish(Twist())

    def _supervise(self) -> None:
        """1 Hz: has the stack come up / died; send the stored start pose to AMCL."""
        st = self._mode["state"]
        if st in ("starting", "ready") and self._mode["mode"] in MODES and not self._stack.running() \
                and not self._sw_lock.locked():
            code = self._stack.proc.poll() if self._stack.proc else None
            self._stop_cmd = True
            self._set_mode(self._mode["mode"], self._mode["map"], "error", f"stack process exited ({code})")
            return
        if st == "starting":
            if time.time() - self._mode["since"] > 120:
                self._set_mode(self._mode["mode"], self._mode["map"], "error", "Nav2 did not become active in 120 s")
                return
            self._check_ready()
        if self._pose_try is not None and self._mode["mode"] == "load":
            pt = self._pose_try          # AMCL is up: send the start pose until it answers (max 10 s)
            if self._ready_ok["loc"]:
                if pt["n"] >= 10 or (pt["n"] and self._amcl_seen > pt["t0"]):
                    self._pose_try = None
                else:
                    if pt["n"] == 0:
                        pt["t0"] = time.monotonic()
                    self._send_initial_pose(pt["pose"])
                    pt["n"] += 1
        if self._drive_restore and self._pd_params.service_is_ready():
            self._drive_restore = False
            self.set_drive(dict(self._drive))

    def _check_ready(self) -> None:
        def ask(key, cli):
            if cli.service_is_ready():
                fut = cli.call_async(Trigger.Request())
                fut.add_done_callback(lambda f: self._on_active(key, f))
        ask("nav", self._nav_active)
        if self._mode["mode"] == "load":
            ask("loc", self._loc_active)
        else:
            self._ready_ok["loc"] = True

    def _on_active(self, key: str, fut) -> None:
        try:
            ok = bool(fut.result().success)
        except Exception:  # noqa: BLE001
            return
        if ok:
            self._ready_ok[key] = True
        if self._mode["state"] == "starting" and self._ready_ok["nav"] and self._ready_ok["loc"]:
            self._stop_cmd = False
            self._set_mode(self._mode["mode"], self._mode["map"], "ready", "")

    def _send_initial_pose(self, p: dict) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(p["x"]), float(p["y"])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(p["yaw"] / 2), math.cos(p["yaw"] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.1 ** 2
        cov[35] = math.radians(5) ** 2
        msg.pose.covariance = cov
        self._init_pub.publish(msg)

    def save_map(self, name: str, note: str = "", overwrite: bool = False) -> dict:
        if self._mode["mode"] != "create" or self._grid is None:
            raise ValueError("nothing to save: start Create mode and drive first")
        data, res, origin = self._grid
        start = self._create_start
        if start is None:
            pose = self._lookup("base_link")
            start = None if pose is None else {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        info = self._maps.save_grid(name, data, res, origin, note=note, start_pose=start, source="slam",
                                    overwrite=overwrite)
        self.get_logger().info(f"map saved as {info['name']!r} ({info['width']}x{info['height']})")
        self._event("map_saved", name=info["name"])
        return info

    # ---------------------------------------------------------------- tests
    def _event(self, kind: str, **data) -> None:
        with self._rec_lock:
            if self._rec is not None:
                self._rec.event(kind, **data)

    def start_test(self, name: str, note: str = "") -> dict:
        with self._rec_lock:
            if self._rec is not None:
                raise ValueError(f"already recording {self._rec.name!r}")
            self._rec = TestRecorder(name=name, note=note, params=self.current())
        self.get_logger().info(f"test {name!r}: recording")
        return {"recording": True, "name": name}

    def stop_test(self) -> dict:
        with self._rec_lock:
            rec, self._rec = self._rec, None
        if rec is None:
            raise ValueError("not recording")
        tid = self._tests.save(rec)
        self.get_logger().info(f"test {rec.name!r} saved as {tid}: {rec.summary()}")
        return {"id": tid, "summary": rec.summary()}

    def test_status(self) -> dict:
        with self._rec_lock:
            r = self._rec
            return {"recording": r is not None, "name": r.name if r else None,
                    "elapsed_s": round(time.time() - r.started, 1) if r else None}

    def _lookup(self, child: str):
        try:
            tf = self._tf.lookup_transform(self._frame, child, Time())
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y, _yaw(t.rotation))

    def _on_depth(self, msg: PointCloud2) -> None:
        self._depth, self._depth_t = msg, time.monotonic()

    def _depth_to_map(self, msg: PointCloud2, pose) -> list:
        """/depth_obstacles (base_link x, y) -> map frame, rounded, at most 400 points."""
        offs = {f.name: f.offset for f in msg.fields}
        n = msg.width * msg.height
        if not n or not all(k in offs for k in "xy"):
            return []
        xy = parse_cloud(bytes(msg.data), msg.point_step, (offs["x"], offs["y"], offs["x"]), n,
                         msg.is_bigendian)[:, :2]
        if len(xy) > 400:
            xy = xy[np.linspace(0, len(xy) - 1, 400).astype(int)]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        x = pose[0] + c * xy[:, 0] - s * xy[:, 1]
        y = pose[1] + s * xy[:, 0] + c * xy[:, 1]
        return np.round(np.stack((x, y), axis=1), 2).tolist()

    def _publish_state(self) -> None:
        pose = self._lookup("base_link")
        st = dict(self._static)
        st["pose"] = None if pose is None else [round(pose[0], 3), round(pose[1], 3), round(pose[2], 4)]
        st["guard"] = self._guard
        st["plan"] = self._plan
        st["scan"] = st["dyn"] = st["depth"] = []
        st["drive"] = self._drive
        st["test"] = self.test_status()
        st["mode"] = {k: self._mode[k] for k in ("mode", "map", "state", "detail")}
        if self._mode["mode"] == "create" and self._create_start is None and pose is not None \
                and self._mode["state"] == "ready":
            self._create_start = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        if self._scan is not None:
            sp = self._lookup(self._scan.header.frame_id)
            if sp is not None:
                st["scan"] = scan_to_map(self._scan, sp)
                if self._dyn is not None:
                    st["dyn"] = scan_to_map(self._dyn, sp, step=1)
        if self._depth is not None and pose is not None and time.monotonic() - self._depth_t < 1.5:
            st["depth"] = self._depth_to_map(self._depth, pose)
        self._state_pub.publish(String(data=json.dumps(st)))
        with self._rec_lock:
            if self._rec is not None:
                self._rec.sample(st["pose"], (self._guard or {}).get("state"), (self._guard or {}).get("nearest_m"),
                                 list(self._wheels) if self._wheels else None, len(st["dyn"]))


class _ApiHandler(http.server.SimpleHTTPRequestHandler):
    """Static files + the /api/ endpoints (see the module docstring)."""

    def __init__(self, *args, bridge: UiBridge, **kwargs) -> None:
        self.bridge = bridge
        super().__init__(*args, **kwargs)

    def log_message(self, *args) -> None:  # noqa: D401 - silence per-request logs
        pass

    def _json(self, code: int, data, filename: str | None = None) -> None:
        body = json.dumps(data, indent=1).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(body)

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        data = json.loads(self.rfile.read(min(n, 1_000_000)).decode("utf-8"))
        return data if isinstance(data, dict) else {}

    def _api(self, method: str) -> None:
        url = urlparse(self.path)
        parts = [p for p in url.path.split("/") if p][1:]     # drop "api"
        b, ps, ts = self.bridge, self.bridge._profiles, self.bridge._tests
        from urllib.parse import unquote
        parts = [unquote(p) for p in parts]
        try:
            if parts[:1] == ["profiles"]:
                if len(parts) == 1 and method == "GET":
                    return self._json(200, {"profiles": ps.list(), "current": b.current()})
                name = parts[1]
                if len(parts) == 2 and method == "GET":
                    d = ps.get(name)
                    return self._json(200, d) if d else self._json(404, {"error": "no such profile"})
                if len(parts) == 2 and method == "PUT":
                    body = self._body()
                    if not body.get("guard") and not body.get("drive"):
                        body = {**b.current(), "note": body.get("note", "")}   # save what is active now
                    return self._json(200, ps.put(name, body))
                if len(parts) == 2 and method == "DELETE":
                    return self._json(200 if ps.delete(name) else 404, {"deleted": name})
                if len(parts) == 3 and parts[2] == "apply" and method == "POST":
                    d = b.apply_profile(name)
                    return self._json(200, d) if d else self._json(404, {"error": "no such profile"})
                if len(parts) == 3 and parts[2] == "default" and method == "POST":
                    if ps.get(name) is None:
                        return self._json(404, {"error": "no such profile"})
                    ps.set_default(name)
                    return self._json(200, {"default": name})
            if parts[:1] == ["mode"]:
                if len(parts) == 1 and method == "GET":
                    return self._json(200, b.mode_status())
                if len(parts) == 1 and method == "POST":
                    body = self._body()
                    return self._json(202, b.request_switch(str(body.get("mode", "")), body.get("map"),
                                                            bool(body.get("force"))))
            if parts[:1] == ["maps"]:
                ms = b._maps
                if len(parts) == 1 and method == "GET":
                    return self._json(200, {"maps": ms.list(), "mode": b.mode_status()})
                if parts[1:] == ["save"] and method == "POST":
                    body = self._body()
                    return self._json(200, b.save_map(body.get("name", ""), body.get("note", ""),
                                                      bool(body.get("overwrite"))))
                name = parts[1]
                if len(parts) == 2 and method == "GET":
                    d = ms.get(name)
                    return self._json(200, d) if d else self._json(404, {"error": "no such map"})
                if len(parts) == 2 and method == "PUT":
                    body = self._body()
                    if ms.get(name) is None:
                        return self._json(404, {"error": "no such map"})
                    d = ms.update(name, note=body.get("note"),
                                  start_pose=body["start_pose"] if "start_pose" in body else ...)
                    if body.get("rename") and safe_map_name(body["rename"]) != d["name"]:
                        d = ms.rename(name, body["rename"])
                        if b._mode["map"] == name:
                            b._mode["map"] = d["name"]
                    return self._json(200, d)
                if len(parts) == 2 and method == "DELETE":
                    if b._mode["mode"] == "load" and b._mode["map"] == safe_map_name(name):
                        return self._json(409, {"error": "this map is loaded now: load another mode or map first"})
                    return self._json(200 if ms.delete(name) else 404, {"deleted": name})
                if len(parts) == 3 and parts[2] == "load" and method == "POST":
                    return self._json(202, b.request_switch("load", name, bool(self._body().get("force"))))
            if parts[:1] == ["tests"]:
                if len(parts) == 1 and method == "GET":
                    return self._json(200, {"tests": ts.list(), "status": b.test_status()})
                if parts[1:] == ["start"] and method == "POST":
                    body = self._body()
                    return self._json(200, b.start_test(body.get("name") or "test", body.get("note", "")))
                if parts[1:] == ["stop"] and method == "POST":
                    return self._json(200, b.stop_test())
                if parts[1:] == ["status"] and method == "GET":
                    return self._json(200, b.test_status())
                tid = parts[1]
                if method == "GET":
                    d = ts.get(tid)
                    if d is None:
                        return self._json(404, {"error": "no such test"})
                    dl = "download" in parse_qs(url.query)
                    return self._json(200, d, filename=f"{tid}.json" if dl else None)
                if method == "PUT":
                    d = ts.update(tid, self._body())
                    return self._json(200, d) if d else self._json(404, {"error": "no such test"})
                if method == "DELETE":
                    return self._json(200 if ts.delete(tid) else 404, {"deleted": tid})
            return self._json(404, {"error": "unknown endpoint"})
        except ValueError as exc:
            return self._json(400, {"error": str(exc)})
        except LookupError as exc:
            return self._json(404, {"error": str(exc.args[0]) if exc.args else "not found"})
        except RuntimeError as exc:
            return self._json(409, {"error": str(exc), "moving": b.moving()})
        except Exception as exc:  # noqa: BLE001 - never kill the server thread
            return self._json(500, {"error": repr(exc)})

    def do_GET(self) -> None:  # noqa: N802
        if self.path.startswith("/api/"):
            return self._api("GET")
        return super().do_GET()

    def do_PUT(self) -> None:  # noqa: N802
        return self._api("PUT")

    def do_POST(self) -> None:  # noqa: N802
        return self._api("POST")

    def do_DELETE(self) -> None:  # noqa: N802
        return self._api("DELETE")


def main() -> None:
    rclpy.init()
    node = UiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stack.stop()          # the child is not ours to leave behind
        node.destroy_node()
        rclpy.try_shutdown()
