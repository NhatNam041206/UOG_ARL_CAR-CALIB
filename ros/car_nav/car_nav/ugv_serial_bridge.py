"""/car/wheels/cmd -> ugv-controller (ESP32-C3) over USB serial; its IMU -> /imu/data_raw.

Runs on the real car only (the simulator's actuator model reads
/car/wheels/cmd itself). Every 1/rate_hz it writes one "<left> <right>"
line (ugv_link.UgvConfig: m/s -> controller units, dead band, safety cap):

* fresh wheel command (younger than watchdog_s): that command
* none for watchdog_s (NAV2 stopped or hung): "0 0"
* on exit (Ctrl-C, container stop): "stop", several times

The ugv-controller firmware has NO command timeout: it keeps the last
speed if this node, the computer or the USB link dies. Keep the motor power
switch / E-stop at hand. docs/car_nav/ugv-controller-failsafe.patch adds a
0.5 s timeout for whoever can flash the ESP32; writing at a fixed rate is
what would keep it fed.

Lines from the controller: "IMU: ..." -> sensor_msgs/Imu on /imu/data_raw
(frame imu_link, rates in rad/s, acceleration in m/s^2, no orientation);
"ERR: ..." / "FAILSAFE: ..." are logged as warnings; the last "OK: L= R="
echo is published on /car/wheels/applied ("<left>,<right>" in units).
The serial port is reopened when it disappears (USB unplugged).
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from . import car_reload, config_loader
from .actuation import WHEELS_TOPIC, parse_payload
from .ugv_link import STOP_LINE, UgvConfig, parse_imu, parse_ok


class UgvSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("ugv_serial_bridge")
        p = self.declare_parameter
        p("port", "/dev/ugv")
        p("baud", 115200)
        p("rate_hz", 20.0)
        p("watchdog_s", 0.5)
        p("imu_frame", "imu_link")
        p("enable_motors", True)   # false: read the IMU only, always send "0 0"
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._cfg = UgvConfig.from_cfg(config_loader.load())
        if not self._cfg.measured:
            self.get_logger().warn(
                f"ugv.units_per_mps NOT MEASURED: assuming {self._cfg.units_per_mps:.0f} units per m/s "
                f"(0.4 m/s -> {self._cfg.wheel_units(0.4)}); cap {self._cfg.max_units}")
        self._port, self._baud = str(g("port")), int(g("baud"))
        self._watchdog = float(g("watchdog_s"))
        self._motors = bool(g("enable_motors"))
        self._frame = str(g("imu_frame"))
        self._ser = None
        self._lock = threading.Lock()
        self._last: tuple[float, float] | None = None
        self._last_t = 0.0
        self._last_line = ""
        self._open_warned = 0.0
        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", 20)
        self._applied_pub = self.create_publisher(String, "/car/wheels/applied", 10)
        self.create_subscription(String, WHEELS_TOPIC, self._on_cmd, 20)
        car_reload.watch(self, self._reload)      # the Health tab applies a measured ugv.units_per_mps
        self.create_timer(1.0 / float(g("rate_hz")), self._tick)
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(
            f"ugv bridge: {self._port} @ {self._baud}, {g('rate_hz')} Hz, watchdog {self._watchdog} s, "
            f"motors {'ON' if self._motors else 'OFF (IMU only)'}")

    def _reload(self, cfg: dict) -> None:
        self._cfg = UgvConfig.from_cfg(cfg)
        self.get_logger().info(f"ugv: {self._cfg.units_per_mps:.1f} units per m/s "
                               f"({'measured' if self._cfg.measured else 'ASSUMED'}), cap {self._cfg.max_units}")

    # ------------------------------------------------------------ serial
    def _open(self):
        if self._ser is not None:
            return self._ser
        try:
            import serial  # pyserial (python3-serial)

            self._ser = serial.Serial(self._port, self._baud, timeout=0.1, write_timeout=0.1)
            self.get_logger().info(f"opened {self._port}")
            self._write(STOP_LINE)
        except Exception as exc:  # noqa: BLE001 - SerialException, OSError, ImportError
            now = time.monotonic()
            if now - self._open_warned > 5.0:
                self._open_warned = now
                self.get_logger().error(f"cannot open {self._port}: {exc}")
            self._ser = None
        return self._ser

    def _close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None

    def _write(self, line: str) -> bool:
        ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(line.encode("ascii"))
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"write to {self._port} failed: {exc}; reopening")
            self._close()
            return False

    # ------------------------------------------------------------- ROS
    def _on_cmd(self, msg: String) -> None:
        parsed = parse_payload(msg.data)
        if parsed is None:
            self.get_logger().warn(f"bad wheel command {msg.data!r}", throttle_duration_sec=2.0)
            return
        self._last = parsed
        self._last_t = time.monotonic()

    def _tick(self) -> None:
        with self._lock:
            if self._open() is None:
                return
            if self._motors and self._last is not None and time.monotonic() - self._last_t <= self._watchdog:
                line = self._cfg.command(*self._last)
            else:
                line = STOP_LINE
            self._write(line)
            if line != self._last_line:
                self.get_logger().debug(f"-> {line.strip()}")
                self._last_line = line

    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                time.sleep(0.2)
                continue
            try:
                chunk = ser.read(256)
            except Exception:  # noqa: BLE001
                with self._lock:
                    self._close()
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._on_line(raw.decode("ascii", errors="replace").strip())
            if len(buf) > 1024:
                buf = b""

    def _on_line(self, line: str) -> None:
        if not line:
            return
        imu = parse_imu(line)
        if imu is not None:
            m = Imu()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self._frame
            m.orientation_covariance[0] = -1.0     # no orientation estimate
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = imu.gx, imu.gy, imu.gz
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = imu.ax, imu.ay, imu.az
            self._imu_pub.publish(m)
            return
        ok = parse_ok(line)
        if ok is not None:
            self._applied_pub.publish(String(data=f"{ok[0]},{ok[1]}"))
            return
        if line.startswith(("ERR", "FAILSAFE", "IMU: NOT FOUND", "IMU: read error", "WARN")):
            self.get_logger().warn(f"controller: {line}", throttle_duration_sec=1.0)
        elif line.startswith(("BOOT", "READY")):
            self.get_logger().info(f"controller: {line}")

    def shutdown(self) -> None:
        self._stop.set()
        with self._lock:
            if self._open() is not None:
                for _ in range(3):
                    self._write("stop\n")
                    time.sleep(0.02)
            self._close()


def main() -> None:
    rclpy.init()
    node = UgvSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
