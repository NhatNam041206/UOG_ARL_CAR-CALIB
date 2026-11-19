"""Map modes and the restartable navigation stack (no ROS imports).

Three modes, one restartable part (launch/nav_stack.launch.py):

  load    map_server + AMCL (+ pose_gate) on a stored map
  create  slam_toolbox builds the map while driving
  flat    no map: map == odom (flat_origin publishes map -> odom), global costmap
          without static layer (rolling window + LiDAR obstacles)
  idle    nothing of the restartable part runs (only sensors, odometry, motors, web UI)

StackProcess starts that launch file as a child process and stops it again
(SIGINT to its process group, then SIGTERM, then SIGKILL).
"""
from __future__ import annotations

import math
import os
import signal
import subprocess
import time
from typing import Callable, Sequence

MODES = ("load", "create", "flat")
ALL_MODES = MODES + ("idle",)
MOVING_MPS = 0.03        # a wheel faster than this counts as "the car is being driven"
MOVING_FRESH_S = 1.0


def derive_mode(mode: str = "", slam: str | bool = False, map_yaml: str = "", map_name: str = "") -> str:
    """The mode from nav.launch's arguments. The old arguments keep working:
    slam:=true -> create, map:=<yaml> -> load, nothing -> idle (pick one in the web UI)."""
    mode = (mode or "").strip().lower()
    if mode:
        if mode not in ALL_MODES:
            raise ValueError(f"mode must be one of {ALL_MODES}, got {mode!r}")
        return mode
    if str(slam).strip().lower() == "true":
        return "create"
    if map_yaml or map_name:
        return "load"
    return "idle"


def stack_command(mode: str, map_yaml: str, stack_args: Sequence[str], launch: str = "ros2") -> list[str]:
    """argv of the child that runs the restartable part. mode must be load / create / flat."""
    if mode not in MODES:
        raise ValueError(f"cannot start mode {mode!r}")
    if mode == "load" and not map_yaml:
        raise ValueError("load mode needs a map")
    cmd = [launch, "launch", "car_nav", "nav_stack.launch.py", f"mode:={mode}"]
    if mode == "load":
        cmd.append(f"map:={map_yaml}")
    cmd.extend(a for a in stack_args if a and not a.endswith(":=") and not a.startswith(("mode:=", "map:=")))
    return cmd


def wheels_moving(wheels, age_s: float) -> bool:
    """wheels = (left, right) m/s of the last /car/wheels/cmd, age_s = seconds since it arrived."""
    if not wheels or age_s > MOVING_FRESH_S:
        return False
    return max(abs(wheels[0]), abs(wheels[1])) > MOVING_MPS


def inverse_pose2d(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    """Inverse of the planar transform (x, y, yaw): the pose of the parent in the child's frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (-(c * x + s * y), -(-s * x + c * y), -yaw)


class StackProcess:
    """Runs / stops one child process. popen is injectable for tests."""

    def __init__(self, popen: Callable = subprocess.Popen, killpg: Callable = None) -> None:
        self._popen = popen
        self._killpg = killpg or _killpg
        self.proc = None

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, cmd: Sequence[str]) -> None:
        if self.running():
            raise RuntimeError("stack already running")
        kw = {"start_new_session": True} if os.name == "posix" else {}
        self.proc = self._popen(list(cmd), **kw)

    def stop(self, sigint_wait_s: float = 12.0, term_wait_s: float = 4.0,
             sleep: Callable = time.sleep) -> str:
        """Returns how it ended: 'not running', 'sigint', 'sigterm' or 'sigkill'."""
        p = self.proc
        if p is None or p.poll() is not None:
            self.proc = None
            return "not running"
        how = "sigint"
        for sig, wait, name in ((signal.SIGINT, sigint_wait_s, "sigint"),
                                (signal.SIGTERM, term_wait_s, "sigterm"),
                                (getattr(signal, "SIGKILL", signal.SIGTERM), 5.0, "sigkill")):
            how = name
            self._killpg(p, sig)
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait:
                if p.poll() is not None:
                    self.proc = None
                    return how
                sleep(0.1)
        self.proc = None
        return how


def _killpg(proc, sig) -> None:
    try:
        if os.name == "posix":
            os.killpg(os.getpgid(proc.pid), sig)
        else:
            proc.send_signal(sig)
    except (ProcessLookupError, PermissionError, OSError):
        pass
