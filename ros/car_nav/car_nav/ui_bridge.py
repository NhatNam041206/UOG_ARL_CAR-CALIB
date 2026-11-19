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

from . import config_loader
from .actuation import WHEELS_TOPIC, parse_payload
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
        self.declare_parameter("data_dir", "/data/nav/ui")   # profiles + test records (a volume on the car)
        data_dir = self.get_parameter("data_dir").value
        self._profiles = ProfileStore(data_dir)
        self._tests = TestStore(data_dir)
        self._rec: TestRecorder | None = None
        self._rec_lock = threading.Lock()
        self._drive: dict = {}
        self._wheels = None
        self._guard_prev = None
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
        self.create_subscription(Bool, "/ui/estop", lambda m: self._event("estop", on=bool(m.data)), 10)
        self.create_subscription(String, "/ui/safety_params", lambda m: self._event("guard_params", data=m.data), 10)
        self.create_subscription(String, "/ui/drive_params", self._on_drive_params, 10)
        self._guard_params_pub = self.create_publisher(String, "/ui/safety_params", 10)
        self._pd_params = self.create_client(SetParameters, "/pd_path_follower/set_parameters")
        self._pd_get = self.create_client(GetParameters, "/pd_path_follower/get_parameters")
        self._drive_timer = self.create_timer(2.0, self._read_drive)
        self.create_timer(3.0, self._publish_map)
        self.create_timer(0.2, self._publish_state)
        self._start_http()

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

    def _on_map(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16)
        data = np.where(data < 0, -1, np.where(data >= 50, 100, 0))
        self._map_json = json.dumps({
            "w": msg.info.width, "h": msg.info.height, "res": msg.info.resolution,
            "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
            "rle": rle(data),
        })
        self._publish_map()

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

    def _on_wheels(self, msg: String) -> None:
        self._wheels = parse_payload(msg.data)

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
        node.destroy_node()
        rclpy.try_shutdown()
