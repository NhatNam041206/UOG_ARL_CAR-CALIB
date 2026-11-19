"""Health check measurements for the web UI (runs inside the ui_bridge node; rules in health_checks.py).

One run at a time, one worker thread. The page (web/health.js) or curl drives it over /api/health/...
(see ui_bridge.py): start, answer the observer questions, abort, download the saved run.

Actuator tests (wheels, speed) and how they reach the wheels:

* wheels test   /health/wheels_override ("<left>,<right>" m/s) -> cmd_vel_to_control, which sends it on
                /car/wheels/cmd instead of the mapped command. This BYPASSES the safety guard and the collision
                monitor on purpose (one wheel alone, or backwards, cannot be a /cmd_vel). Therefore: the operator
                must confirm E-STOP in reach AND "wheels off the ground" or "clear space"; without "off the
                ground" the LiDAR must show nothing within 0.5 m of the footprint; wheel speed 0.15 m/s (cmd_vel_to_control
                clamps every wheel to 0.25 m/s), pulses 1.5 s, the override dies by itself 0.3 s after the last message.
* speed test    /cmd_vel_nav (v, omega) -> collision monitor -> safety guard -> /cmd_vel -> cmd_vel_to_control, i.e. the
                same path as a Nav2 goal, so both guards still apply to the drive. Needs a running nav stack (mode
                load / create / flat, state ready): without the guard the test is refused. Forward only, <= 0.3 m/s.

Always: refused while a goal is active, the wheels are moving or the E-STOP is on; every actuator step needs the
page's heartbeat (/ui/health_hb or POST /api/health/heartbeat, newer than 0.5 s) and stops the wheels when it
stops, when the E-STOP is pressed, when a goal is sent, on abort; a run always ends with zero wheels.
"""
from __future__ import annotations

import base64
import json
import math
import threading
import time

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
from std_msgs.msg import Bool, String

from . import config_loader
from . import health_checks as H
from .cmd_vel_to_control import OVERRIDE_TOPIC
from .depth_obstacles import cloud_to_base, parse_cloud
from .geometry import front_distance
from .nav_modes import MODES
from .ugv_link import UgvConfig

ALL_CHECKS = ("lidar", "depth", "camera", "imu", "wheels", "speed")
ACTUATORS = ("wheels", "speed")
HEARTBEAT_S = 0.5
GUARD_OK = ("clear", "slow", "depth_stale", "off")
DEFAULTS = {"wheel_speed_mps": 0.15, "pulse_s": 1.5, "announce_s": 3.0, "answer_timeout_s": 30.0,
            "speed_mps": 0.2, "duration_s": 4.5, "spin_rps": 0.5, "spin_s": 3.0, "tape_timeout_s": 120.0,
            "collect_s": 4.5}


class Abort(Exception):
    """The run was stopped (observer, deadman, E-STOP, a goal was sent)."""


def _clamp(v, lo, hi) -> float:
    return min(max(float(v), lo), hi)


def decode_image(msg: Image, max_w: int = 320):
    """sensor_msgs/Image -> (H, W, 3) uint8 RGB array thinned to at most max_w columns, or None."""
    enc = msg.encoding.lower()
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1}.get(enc)
    if ch is None or msg.width == 0 or msg.height == 0:
        return None
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=msg.height * msg.step)
    a = a.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    k = max(1, int(math.ceil(msg.width / max_w)))
    a = a[::k, ::k]
    if ch == 1:
        a = np.repeat(a, 3, axis=2)
    elif enc.startswith("bgr"):
        a = a[:, :, 2::-1] if ch == 3 else a[:, :, [2, 1, 0]]
    else:
        a = a[:, :, :3]
    return np.ascontiguousarray(a)


def encode_snapshot(rgb: np.ndarray) -> tuple[bytes, str]:
    """RGB array -> (bytes, mime): JPEG with OpenCV or Pillow, else a PNG written with zlib."""
    try:
        import cv2

        ok, buf = cv2.imencode(".jpg", rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            return buf.tobytes(), "image/jpeg"
    except Exception:  # noqa: BLE001
        pass
    try:
        import io

        from PIL import Image as PilImage

        bio = io.BytesIO()
        PilImage.fromarray(rgb).save(bio, "JPEG", quality=70)
        return bio.getvalue(), "image/jpeg"
    except Exception:  # noqa: BLE001
        pass
    import struct
    import zlib

    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(t: bytes, d: bytes) -> bytes:
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    return png, "image/png"


class HealthRunner:
    def __init__(self, node, bridge, data_dir: str) -> None:
        self.n = node
        self.b = bridge
        self.store = H.HealthStore(data_dir)
        p = node.declare_parameter
        p("health_image_topic", "/camera/color/image_raw")
        p("health_depth_topic", "/camera/depth/points")
        p("health_imu_topic", "/imu/data_raw")
        p("health_odom_topic", "/odom_rf2o")
        self._cv = threading.Condition()
        self._lock = threading.RLock()
        self._s: dict = {"state": "idle", "run": None}
        self._abort = threading.Event()
        self._abort_reason = ""
        self._answer = None
        self._pid = 0
        self._hb_t = -1e9
        self._snap: tuple[bytes, str] | None = None
        self._thread: threading.Thread | None = None
        self._actuating = False
        self._ovr_pub = node.create_publisher(String, OVERRIDE_TOPIC, 10)
        self._cmd_pub = node.create_publisher(Twist, "/cmd_vel_nav", 10)
        node.create_subscription(String, "/ui/health_hb", lambda m: self.heartbeat(), 10)
        node.create_subscription(Bool, "/ui/estop", lambda m: m.data and self._on_estop(), 10)
        node.create_subscription(PoseStamped, "/goal_pose", lambda m: self._on_goal(), 10)
        node.create_timer(0.05, self._watchdog)

    # ------------------------------------------------------------------ small helpers
    def heartbeat(self) -> None:
        self._hb_t = time.monotonic()

    def hb_age(self) -> float:
        return time.monotonic() - self._hb_t

    def _on_estop(self) -> None:
        if self._actuating:
            self._stop_run("E-STOP pressed")

    def _on_goal(self) -> None:
        if self._actuating:
            self._stop_run("a navigation goal was sent")

    def _stop_run(self, why: str) -> None:
        if not self._abort.is_set():
            self._abort_reason = why
            self._abort.set()
            self._event("abort", why=why)
        self._zero()
        with self._cv:
            self._cv.notify_all()

    def _watchdog(self) -> None:
        """Deadman: while an actuator test runs, no heartbeat for 0.5 s -> zero the wheels and end the run."""
        if not self._actuating:
            return
        if self.hb_age() > HEARTBEAT_S:
            self._stop_run(f"deadman: no heartbeat from the page for {HEARTBEAT_S} s")
        elif getattr(self.b, "_estop_last", False):
            self._stop_run("E-STOP is on")
        if self._abort.is_set():
            self._zero()

    def _zero(self) -> None:
        self._ovr_pub.publish(String(data="0.000,0.000"))
        self._cmd_pub.publish(Twist())

    def _event(self, kind: str, **d) -> None:
        run = self._s.get("run")
        if run is not None:
            run["events"].append({"t": round(time.time() - run["started_ts"], 2), "kind": kind, **d})

    def _check_safe(self) -> None:
        if self._abort.is_set():
            raise Abort(self._abort_reason or "aborted")
        if self._actuating:
            if self.hb_age() > HEARTBEAT_S:
                self._stop_run(f"deadman: no heartbeat from the page for {HEARTBEAT_S} s")
            elif getattr(self.b, "_estop_last", False):
                self._stop_run("E-STOP is on")
            if self._abort.is_set():
                raise Abort(self._abort_reason)

    def _wait(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._check_safe()
            time.sleep(0.05)

    def _sub(self, msg_type, topic: str, cb, qos):
        return self.n.create_subscription(msg_type, topic, cb, qos)

    def _ask(self, kind: str, title: str, text: str, timeout: float, **extra):
        """Show a question to the observer; returns (answer, value): answer "yes" | "no" | None (no answer in time)."""
        with self._cv:
            self._pid += 1
            pid = self._pid
            self._answer = None
            end = time.monotonic() + timeout
            with self._lock:
                self._s["run"]["prompt"] = {"id": pid, "kind": kind, "title": title, "text": text,
                                            "deadline": time.time() + timeout, **extra}
            try:
                while self._answer is None and not self._abort.is_set() and time.monotonic() < end:
                    self._cv.wait(0.1)
                    if self._actuating and self.hb_age() > HEARTBEAT_S:
                        self._stop_run(f"deadman: no heartbeat from the page for {HEARTBEAT_S} s")
                ans = self._answer
            finally:
                with self._lock:
                    self._s["run"]["prompt"] = None
        if self._abort.is_set():
            raise Abort(self._abort_reason)
        if ans is None:
            self._event("no_answer", prompt=title)
            return None, None
        self._event("answer", prompt=title, answer=ans[0])
        return ans

    def _drive(self, seconds: float, pub, hz: float = 10.0) -> None:
        """Call pub() at hz for `seconds`; every cycle first checks that driving is still allowed."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._check_safe()
            pub()
            time.sleep(1.0 / hz)

    # ------------------------------------------------------------------ REST side
    def status(self) -> dict:
        with self._lock:
            run = json.loads(json.dumps(self._s["run"])) if self._s.get("run") else None
        wheels = getattr(self.b, "_wheels", None)
        age = time.monotonic() - getattr(self.b, "_wheels_t", 0.0)
        return {"state": self._s["state"], "run": run, "last": self.store.last(), "checks": list(ALL_CHECKS),
                "defaults": DEFAULTS, "heartbeat_age_s": None if self._hb_t < 0 else round(self.hb_age(), 2),
                "wheels_cmd": list(wheels) if wheels else None, "wheels_cmd_age_s": round(age, 2),
                "snapshot": self._snap is not None, "thresholds": {k: v for k, v in H.THRESH.items()},
                "estop": bool(getattr(self.b, "_estop_last", False)), "goal_active": self.b.goal_active(),
                "moving": self.b.moving()}

    def snapshot(self):
        return self._snap

    def answer(self, pid, ans: str, value=None) -> dict:
        ans = str(ans).lower()
        if ans not in ("yes", "no"):
            raise ValueError("answer must be yes or no")
        with self._cv:
            cur = (self._s.get("run") or {}).get("prompt")
            if not cur or cur["id"] != pid:
                raise RuntimeError("no such question is open (answered or timed out)")
            self._answer = (ans, value)
            self._cv.notify_all()
        return {"ok": True}

    def abort(self) -> dict:
        if self._s["state"] == "running":
            self._stop_run("aborted by the observer")
        return {"aborted": self._s["state"] == "running"}

    def start(self, body: dict) -> dict:
        if self._s["state"] == "running":
            raise RuntimeError("a health check is already running")
        checks = [str(c) for c in (body.get("checks") or ALL_CHECKS)]
        bad = [c for c in checks if c not in ALL_CHECKS]
        if bad:
            raise ValueError(f"unknown checks: {', '.join(bad)}")
        params = {k: float(body.get("params", {}).get(k, v)) for k, v in DEFAULTS.items()}
        params["wheel_speed_mps"] = _clamp(params["wheel_speed_mps"], 0.08, 0.2)
        params["pulse_s"] = _clamp(params["pulse_s"], 0.5, 2.0)
        params["speed_mps"] = _clamp(params["speed_mps"], 0.05, 0.3)
        params["duration_s"] = _clamp(params["duration_s"], 2.5, 6.0)
        params["spin_rps"] = _clamp(params["spin_rps"], 0.2, 0.8)
        params["spin_s"] = _clamp(params["spin_s"], 2.0, 4.0)
        conf = body.get("confirm") or {}
        act = [c for c in checks if c in ACTUATORS]
        if act:
            self._actuator_blockers(checks, conf)
        elif "imu" in checks and self.b.moving():
            raise RuntimeError("the wheels are being driven: the IMU check needs the car standing still")
        now = time.time()
        run = {"started": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)), "started_ts": now, "checks": checks,
               "params": params, "confirm": {k: bool(v) for k, v in conf.items()}, "results": {}, "current": None,
               "step": "", "prompt": None, "events": [], "overall": None, "finished": None, "run_id": None,
               "abort_reason": ""}
        self._abort.clear()
        self._abort_reason = ""
        self._answer = None
        with self._lock:
            self._s = {"state": "running", "run": run}
        self._actuating = bool(act)
        self._thread = threading.Thread(target=self._run, args=(run, params, conf), daemon=True)
        self._thread.start()
        return {"started": True, "checks": checks}

    def _actuator_blockers(self, checks: list[str], conf: dict) -> None:
        b = self.b
        if getattr(b, "_estop_last", False):
            raise RuntimeError("the E-STOP is on: release it first")
        if b.goal_active():
            raise RuntimeError("a navigation goal is active: actuator tests are refused")
        if b._sw_lock.locked() or b._mode.get("state") == "switching":
            raise RuntimeError("the nav stack is switching mode: wait")
        if b.moving():
            raise RuntimeError("the wheels are being driven (wheel command is not 0)")
        if self.hb_age() > HEARTBEAT_S:
            raise RuntimeError("no heartbeat from the page (needs one every 0.5 s): open the Health tab and keep it visible")
        if not conf.get("estop_in_reach"):
            raise RuntimeError("confirm that the E-STOP is within reach")
        if "wheels" in checks and not (conf.get("wheels_off_ground") or conf.get("clear_space")):
            raise RuntimeError("confirm that the wheels are off the ground or that there is clear space around the car")
        if "speed" in checks:
            if not conf.get("clear_ahead"):
                raise RuntimeError("confirm that 2 m ahead are clear")
            st = (getattr(b, "_guard", None) or {}).get("state")
            if b._mode.get("mode") not in MODES or b._mode.get("state") != "ready" or st is None:
                raise RuntimeError("the speed test drives through the safety guard: start a map mode "
                                   "(flat / load / create) and wait until it is ready")

    # ------------------------------------------------------------------ the run
    def _run(self, run: dict, params: dict, conf: dict) -> None:
        fns = {"lidar": self._lidar, "depth": self._depth, "camera": self._camera, "imu": self._imu,
               "wheels": self._wheels, "speed": self._speed}
        try:
            for name in run["checks"]:
                run["current"] = name
                try:
                    res = fns[name](run, params, conf)
                except Abort as exc:
                    res = H.failed(f"stopped: {exc}")
                    run["results"][name] = res
                    run["abort_reason"] = str(exc)
                    break
                except Exception as exc:  # noqa: BLE001 - a broken check is a failed check
                    self.n.get_logger().error(f"health check {name} crashed: {exc!r}")
                    res = H.failed(f"the check crashed: {exc!r}")
                run["results"][name] = res
                self._event("result", check=name, status=res["status"])
            for name in run["checks"]:        # a check that did not run (aborted run) counts as failed
                run["results"].setdefault(name, H.failed("not run (the run was stopped)"))
        finally:
            self._actuating = False
            for _ in range(5):
                self._zero()
                time.sleep(0.03)
            run["current"], run["prompt"], run["step"] = None, None, ""
            run["overall"] = H.overall(run["results"])
            run["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
            try:
                run["run_id"] = self.store.save({k: v for k, v in run.items() if k != "prompt"})
            except OSError as exc:
                self.n.get_logger().error(f"cannot save the health run: {exc!r}")
            with self._lock:
                self._s["state"] = "done"
            self.n.get_logger().info(f"health check {run['overall'].upper()}: "
                                     + ", ".join(f"{k} {v['status']}" for k, v in run["results"].items()))

    def _step(self, run: dict, text: str) -> None:
        run["step"] = text

    # ------------------------------------------------------------------ LiDAR
    def _lidar(self, run, params, conf) -> dict:
        self._step(run, "LiDAR: listening to /scan")
        cfg = config_loader.load()
        fp, laser = config_loader.footprint(cfg), config_loader.laser_pose(cfg)
        mask = cfg.get("self_mask") or {}
        margin, sectors = float(mask.get("margin_m", 0.03)), [tuple(s) for s in (mask.get("sectors_deg") or [])]
        stamps, frames, rmax = [], [], [12.0]

        def cb(m: LaserScan) -> None:
            stamps.append(time.monotonic())
            rmax[0] = float(m.range_max) or 12.0
            frames.append(H.lidar_frame_stats(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max,
                                              laser, fp, margin, sectors))
        sub = self._sub(LaserScan, "/scan", cb, qos_profile_sensor_data)
        try:
            self._wait(params["collect_s"])
        finally:
            self.n.destroy_subscription(sub)
        return H.eval_lidar(list(stamps), list(frames), rmax[0])

    # ------------------------------------------------------------------ depth camera
    def _depth(self, run, params, conf) -> dict:
        self._step(run, "Depth camera: listening to the point cloud")
        cfg = config_loader.load()
        cam = config_loader.camera_pose(cfg)
        stamps, frames, last = [], [], [None]
        count = [0]

        def cb(m: PointCloud2) -> None:
            stamps.append(time.monotonic())
            count[0] += 1
            if count[0] % 3 != 1:
                return
            offs = {f.name: f.offset for f in m.fields}
            n = m.width * m.height
            if not n or not all(k in offs for k in "xyz"):
                frames.append({"n_valid": 0, "n_total": n, "organized": m.height > 1})
                return
            xyz = parse_cloud(bytes(m.data), m.point_step, (offs["x"], offs["y"], offs["z"]), n, m.is_bigendian)
            ok = np.isfinite(xyz).all(axis=1)
            frames.append({"n_valid": int(ok.sum()), "n_total": int(n), "organized": m.height > 1})
            last[0] = xyz[ok]
        sub = self._sub(PointCloud2, str(self.n.get_parameter("health_depth_topic").value), cb, qos_profile_sensor_data)
        try:
            self._wait(params["collect_s"])
        finally:
            self.n.destroy_subscription(sub)
        floor = None
        if last[0] is not None and last[0].shape[0]:
            pts = last[0]
            if pts.shape[0] > 30000:
                pts = pts[np.linspace(0, pts.shape[0] - 1, 30000).astype(int)]
            floor = H.fit_floor(cloud_to_base(pts, cam, True))
        res = H.eval_depth(list(stamps), list(frames), cam, floor)
        res["camera_pose"] = {"z": cam.z, "pitch_deg": round(math.degrees(cam.pitch), 2)}
        return res

    # ------------------------------------------------------------------ colour camera
    def _camera(self, run, params, conf) -> dict:
        self._step(run, "Colour camera: listening to the image topic")
        stamps, last = [], [None]

        def cb(m: Image) -> None:
            stamps.append(time.monotonic())
            last[0] = m
        topic = str(self.n.get_parameter("health_image_topic").value)
        sub = self._sub(Image, topic, cb, qos_profile_sensor_data)
        try:
            self._wait(3.5)
        finally:
            self.n.destroy_subscription(sub)
        if last[0] is None:
            return H.failed(f"no image arrived on {topic}")
        rgb = decode_image(last[0])
        if rgb is None:
            return H.failed(f"image encoding {last[0].encoding!r} is not supported for the snapshot")
        data, mime = encode_snapshot(rgb)
        self._snap = (data, mime)
        self._step(run, "Colour camera: observer confirms the picture")
        ans, _ = self._ask("image", "Colour camera", "Does this picture look right (sharp, correct colours, "
                           "the right way up, showing what is in front of the car)?", 60.0, image=True)
        res = H.eval_color(list(stamps), (int(last[0].width), int(last[0].height)), ans)
        res["snapshot_b64"] = base64.b64encode(data).decode("ascii")
        res["snapshot_mime"] = mime
        return res

    # ------------------------------------------------------------------ IMU
    def _imu(self, run, params, conf) -> dict:
        self._step(run, "IMU: keep the car still")
        samples = []

        def cb(m: Imu) -> None:
            a, g = m.linear_acceleration, m.angular_velocity
            samples.append((time.monotonic(), a.x, a.y, a.z, g.x, g.y, g.z))
        topic = str(self.n.get_parameter("health_imu_topic").value)
        sub = self._sub(Imu, topic, cb, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        try:
            self._wait(params["collect_s"])
        finally:
            self.n.destroy_subscription(sub)
        if not samples:
            return H.failed(f"no IMU data on {topic} (the simulator has no IMU; on the car: ugv_serial_bridge / the ESP32)")
        return H.eval_imu(samples)

    # ------------------------------------------------------------------ wheels test
    def _wheels(self, run, params, conf) -> dict:
        cfg = config_loader.load()
        fp, laser = config_loader.footprint(cfg), config_loader.laser_pose(cfg)
        ugv = UgvConfig.from_cfg(cfg)
        if not conf.get("wheels_off_ground"):
            self._step(run, "Wheels: checking that nothing is within 0.5 m of the car")
            scan = self._one_scan()
            clr = None if scan is None else H.clearance_outside(scan.ranges, scan.angle_min, scan.angle_increment,
                                                                scan.range_min, scan.range_max, laser, fp)
            if scan is None:
                return H.failed("no /scan: cannot check the space around the car (confirm 'wheels off the ground' to skip)")
            if clr is not None and clr < 0.5:
                return H.failed(f"something is {clr:.2f} m from the car: clear 0.5 m around it, or lift the wheels "
                                "off the ground and confirm that")
        echoes: list[tuple[float, float, float]] = []

        def on_echo(m: String) -> None:
            v = H.parse_applied(m.data)
            if v:
                echoes.append((time.monotonic(), v[0], v[1]))
        sub = self._sub(String, "/car/wheels/applied", on_echo, 20)
        steps = H.wheel_steps(params["wheel_speed_mps"])
        out, ok_all = [], True
        try:
            for i, st in enumerate(steps, 1):
                self._step(run, f"Wheels {i}/{len(steps)}: {st['name']}")
                run["prompt"] = None
                self._announce(run, i, len(steps), st, params["announce_s"])
                payload = f"{st['left']:.3f},{st['right']:.3f}"
                t0 = time.monotonic()
                try:
                    self._drive(params["pulse_s"], lambda: self._ovr_pub.publish(String(data=payload)), hz=10.0)
                finally:
                    for _ in range(3):
                        self._ovr_pub.publish(String(data="0.000,0.000"))
                        time.sleep(0.03)
                t1 = time.monotonic()
                exp = H.expected_units(st["left"], st["right"], ugv)
                echo = H.eval_echo(exp, [(l, r) for t, l, r in echoes if t0 <= t <= t1 + 0.3])
                ans, _ = self._ask("confirm", f"{i}/{len(steps)} {st['name']}",
                                   f"Did it? {st['expect']}.", params["answer_timeout_s"])
                res = H.eval_wheel_step(st, echo, ans)
                out.append(res)
                if res["status"] != "pass":
                    ok_all = False
                    if ans != "yes":
                        break                    # an observer "no" / no answer ends the test: do not drive on
        finally:
            self.n.destroy_subscription(sub)
        for st in steps[len(out):]:
            out.append({"id": st["id"], "name": st["name"], "status": "fail", "detail": "not run", "rules": []})
        bad = [s["name"] for s in out if s["status"] != "pass"]
        return {"status": "fail" if bad else "pass", "detail": ("failed: " + ", ".join(bad)) if bad else "all six steps passed",
                "steps": out, "rules": [], "wheel_speed_mps": params["wheel_speed_mps"],
                "path": "/health/wheels_override -> cmd_vel_to_control -> /car/wheels/cmd (bypasses the guards)"}

    def _announce(self, run, i: int, n: int, st: dict, seconds: float) -> None:
        with self._lock:
            run["prompt"] = {"id": -1, "kind": "notice", "title": f"Step {i}/{n}: {st['name']}",
                             "text": f"The pulse starts in {seconds:.0f} s. Expect: {st['expect']}.",
                             "deadline": time.time() + seconds}
        try:
            self._wait(seconds)
        finally:
            with self._lock:
                run["prompt"] = None

    def _one_scan(self, timeout: float = 3.0):
        box = []
        sub = self._sub(LaserScan, "/scan", lambda m: box.append(m), qos_profile_sensor_data)
        try:
            end = time.monotonic() + timeout
            while not box and time.monotonic() < end:
                self._check_safe()
                time.sleep(0.05)
        finally:
            self.n.destroy_subscription(sub)
        return box[-1] if box else None

    # ------------------------------------------------------------------ speed test
    def _speed(self, run, params, conf) -> dict:
        cfg = config_loader.load()
        fp, laser = config_loader.footprint(cfg), config_loader.laser_pose(cfg)
        ugv = UgvConfig.from_cfg(cfg)
        track = float((cfg.get("drive") or {}).get("track_width_m") or (cfg.get("drive") or {}).get("assumed_track_width_m") or 0.52)
        v, dur = params["speed_mps"], params["duration_s"]
        odom, front, last_scan = [], [], [None]
        odom_topic = str(self.n.get_parameter("health_odom_topic").value)

        def on_odom(m: Odometry) -> None:
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            odom.append((time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, yaw))

        def on_scan(m: LaserScan) -> None:
            last_scan[0] = m
            x, _n = front_distance(m.ranges, m.angle_min, m.angle_increment, laser, 0.10, x_min=0.0)
            if x is not None:
                front.append((time.monotonic(), x))
        subs = [self._sub(Odometry, odom_topic, on_odom, qos_profile_sensor_data),
                self._sub(LaserScan, "/scan", on_scan, qos_profile_sensor_data)]
        try:
            self._step(run, "Speed: waiting for the LiDAR and odometry")
            t_end = time.monotonic() + 4.0
            while (not odom or last_scan[0] is None) and time.monotonic() < t_end:
                self._check_safe()
                time.sleep(0.05)
            if not odom:
                return H.failed(f"no odometry on {odom_topic} (rf2o not running)")
            if last_scan[0] is None:
                return H.failed("no /scan")
            m = last_scan[0]
            nose, _n = front_distance(m.ranges, m.angle_min, m.angle_increment, laser, fp.half_width + 0.05, x_min=0.0)
            free = None if nose is None else nose - fp.x_front
            need = max(1.0, v * dur + 0.7)
            if free is not None and free < need:
                return H.failed(f"only {free:.2f} m clear ahead of the nose, need {need:.1f} m: move the car or the obstacle")
            ans, _ = self._ask("confirm", "Speed test: ready?",
                               f"The car will drive FORWARD at {v:.2f} m/s for {dur:.1f} s (about {v * dur:.1f} m)"
                               + (", then spin on the spot. " if params.get("spin_rps") else ". ")
                               + "Is the floor clear ahead and are you at the E-STOP? Press yes to go.", 60.0)
            if ans != "yes":
                return H.failed("the observer did not confirm the start")
            # forward leg
            self._step(run, f"Speed: driving forward {v:.2f} m/s")
            guard_bad = [False]
            t_start = time.monotonic()
            tw = Twist()
            tw.linear.x = v

            def go() -> None:
                self._cmd_pub.publish(tw)
                if (getattr(self.b, "_guard", None) or {}).get("state") not in GUARD_OK + (None,):
                    guard_bad[0] = True
            try:
                self._drive(dur, go)
            finally:
                self._zero()
            self._step(run, "Speed: stopping")
            try:
                self._drive(1.5, lambda: self._cmd_pub.publish(Twist()))
            finally:
                self._zero()
            t_stop = time.monotonic()
            o2 = [(t, x, y) for t, x, y, _ in odom]
            yaws = [(t, yw) for t, _, _, yw in odom]
            total = window_total(o2, t_start, t_stop)
            tape_ans, tape = None, None
            win = (t_start + 1.5, t_start + dur)
            fres = H.speed_result(v, t_start, win, o2, front, yaws, total, ugv.units_per_mps, None, guard_bad[0], "yes")
            # spin leg
            spin = None
            w = params["spin_rps"]
            if w > 0:
                self._step(run, "Speed: checking the room to spin")
                scan = self._one_scan() or last_scan[0]
                radius = math.hypot(max(fp.x_front, -fp.x_rear), fp.half_width)
                rad = H.min_radius_outside(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                                           scan.range_max, laser, fp)
                if rad is not None and rad < radius + 0.15:
                    spin = {"status": "fail", "detail": f"not enough room to spin: something is {rad:.2f} m from the centre, "
                                                        f"the car's corners sweep {radius:.2f} m (+ 0.15 m)", "rules": []}
                else:
                    self._step(run, f"Speed: spinning left {w:.2f} rad/s")
                    tw2 = Twist()
                    tw2.angular.z = w
                    s0 = time.monotonic()
                    try:
                        self._drive(params["spin_s"], lambda: self._cmd_pub.publish(tw2))
                    finally:
                        self._zero()
                    try:
                        self._drive(1.0, lambda: self._cmd_pub.publish(Twist()))
                    finally:
                        self._zero()
                    yaws = [(t, yw) for t, _, _, yw in odom]
                    spin = H.spin_result(w, (s0 + 1.2, s0 + params["spin_s"]), yaws, track)
            # the observer
            lid = fres.get("real_mps")
            txt = (f"Measured by the LiDAR ({fres.get('source')}): {lid if lid is not None else 'n/a'} m/s of the commanded {v:.2f} m/s; "
                   f"travelled {0 if total is None else total:.2f} m in total. "
                   + (f"Spin: yaw rate {spin.get('real_rps')} rad/s (yaw_efficiency {spin.get('yaw_efficiency')}). " if spin and 'real_rps' in spin else "")
                   + "Optional: type the distance the car rolled forward, measured with a tape (m), then press yes. "
                     "yes = this looks right, no = it does not.")
            tape_ans, tape_val = self._ask("number", "Speed test: confirm", txt, params["tape_timeout_s"])
            tape = None
            if tape_val not in (None, ""):
                try:
                    tape = float(tape_val)
                    tape = tape if 0.05 < tape < 20 else None
                except (TypeError, ValueError):
                    tape = None
            res = H.speed_result(v, t_start, win, o2, front, yaws, total, ugv.units_per_mps, tape, guard_bad[0], tape_ans)
            if spin is not None:
                res["spin"] = spin
                res["rules"].append({"name": "spin", "ok": spin["status"] == "pass", "value": spin["detail"],
                                     "rule": "yaw_efficiency 0.2 .. 1.4, room to turn"})
                if spin["status"] != "pass":
                    prev = "" if res["status"] == "pass" else res["detail"] + "; "
                    res["status"], res["detail"] = "fail", prev + f"spin: {spin['detail']}"
            res["total_path_m"] = total
            res["path"] = "/cmd_vel_nav -> collision monitor -> safety guard -> /cmd_vel -> cmd_vel_to_control"
            return res
        finally:
            for s in subs:
                self.n.destroy_subscription(s)

    # ------------------------------------------------------------------ apply
    def apply_speed(self, run_id: str) -> dict:
        run = self.store.get(run_id)
        if run is None:
            raise LookupError("no such health run")
        sp = (run.get("results") or {}).get("speed") or {}
        val = sp.get("units_per_mps_proposed")
        if sp.get("status") != "pass" or val is None:
            raise RuntimeError("only a PASSED speed test can be applied")
        b = self.b
        if self._s["state"] == "running" or b.moving() or b.goal_active():
            raise RuntimeError("the car must be stopped (no health run, wheel command 0, no goal) to write car.yaml")
        res = b._car_file.write_updates({("ugv", "units_per_mps"): float(val)})
        b._reload_pub.publish(String(data=b._car_file.path()))
        b._load_static(config_loader.load())
        b._event("car_applied", name="health speed test", reset=False)
        return {**res, "units_per_mps": float(val), "from": sp.get("units_per_mps_now")}


def window_total(odom: list[tuple], t0: float, t1: float) -> float | None:
    """Straight distance travelled between t0 and t1 (start to the final stop)."""
    return H.window_distance(odom, t0, t1)
