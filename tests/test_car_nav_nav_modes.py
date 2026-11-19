"""Map modes and the stack process (ros/car_nav/car_nav/nav_modes.py)."""
import math
import signal

import pytest

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.nav_modes import StackProcess, derive_mode, inverse_pose2d, stack_command, wheels_moving


def test_derive_mode_keeps_old_arguments_working():
    assert derive_mode(slam="true") == "create"
    assert derive_mode(map_yaml="/m.yaml") == "load"
    assert derive_mode(map_name="lab") == "load"
    assert derive_mode() == "idle"
    assert derive_mode(mode="FLAT", slam="true") == "flat"
    with pytest.raises(ValueError):
        derive_mode(mode="slam")


def test_stack_command():
    c = stack_command("load", "/data/m.yaml", ["use_sim_time:=true", "mode:=x", ""])
    assert c[:4] == ["ros2", "launch", "car_nav", "nav_stack.launch.py"]
    assert "mode:=load" in c and "map:=/data/m.yaml" in c and "use_sim_time:=true" in c and "mode:=x" not in c
    assert not any(a.startswith("map:=") for a in stack_command("flat", "/ignored.yaml", []))
    with pytest.raises(ValueError):
        stack_command("load", "", [])
    with pytest.raises(ValueError):
        stack_command("idle", "", [])


def test_wheels_moving():
    assert wheels_moving((0.2, 0.2), 0.1)
    assert wheels_moving((-0.2, 0.2), 0.1)           # turning on the spot counts
    assert not wheels_moving((0.0, 0.0), 0.1)
    assert not wheels_moving((0.4, 0.4), 5.0)        # stale
    assert not wheels_moving(None, 0.0)


def test_inverse_pose_puts_the_start_at_the_origin():
    x, y, yaw = 2.0, -1.0, math.radians(70)
    ix, iy, iyaw = inverse_pose2d(x, y, yaw)
    # compose (ix,iy,iyaw) * (x,y,yaw) = identity
    c, s = math.cos(iyaw), math.sin(iyaw)
    assert c * x - s * y + ix == pytest.approx(0, abs=1e-9)
    assert s * x + c * y + iy == pytest.approx(0, abs=1e-9)
    assert iyaw + yaw == pytest.approx(0)


class FakeProc:
    def __init__(self, dies_on):
        self.dies_on, self.sent, self.pid = dies_on, [], 1
        self.alive = True

    def poll(self):
        return None if self.alive else 0


def test_stop_escalates_until_the_child_dies():
    proc = FakeProc(dies_on=signal.SIGTERM)

    def killpg(p, sig):
        p.sent.append(sig)
        if sig == p.dies_on:
            p.alive = False
    sp = StackProcess(popen=lambda cmd, **kw: proc, killpg=killpg)
    sp.start(["x"])
    assert sp.running()
    with pytest.raises(RuntimeError):
        sp.start(["x"])
    assert sp.stop(sigint_wait_s=0.0, term_wait_s=1.0, sleep=lambda s: None) == "sigterm"
    assert proc.sent == [signal.SIGINT, signal.SIGTERM] and not sp.running()
    assert sp.stop() == "not running"


def test_flat_overrides_have_no_static_layer():
    import yaml
    from tests._car_nav_paths import ROOT
    d = yaml.safe_load((ROOT / "ros/car_nav/config/nav2_flat_overrides.yaml").read_text(encoding="utf-8"))
    g = d["global_costmap"]["global_costmap"]["ros__parameters"]
    assert "static_layer" not in g["plugins"] and "obstacle_layer" in g["plugins"]
    assert g["rolling_window"] is True and g["track_unknown_space"] is False
    base = yaml.safe_load((ROOT / "ros/car_nav/config/nav2_params.yaml").read_text(encoding="utf-8"))
    assert "obstacle_layer" in base["global_costmap"]["global_costmap"]["ros__parameters"]   # its settings live there


def test_launch_files_and_executables_are_wired():
    from tests._car_nav_paths import ROOT
    setup = (ROOT / "ros/car_nav/setup.py").read_text(encoding="utf-8")
    assert "flat_origin = car_nav.flat_origin:main" in setup
    stack = (ROOT / "ros/car_nav/launch/nav_stack.launch.py").read_text(encoding="utf-8")
    for exe in ("map_server", "amcl", "pose_gate", "flat_origin", "planner_server", "safety_guard", "dynamic_scan"):
        assert exe in stack
    nav = (ROOT / "ros/car_nav/launch/nav.launch.py").read_text(encoding="utf-8").split("\"\"\"", 2)[2]     # without the docstring
    for persistent in ("scan_upright", "adaptive_odom", "cmd_vel_to_control", "ui_bridge", "rosbridge_websocket"):
        assert persistent in nav
    for restartable in ("planner_server", "map_server", "safety_guard", "lifecycle_manager"):
        assert restartable not in nav          # these only live in the restartable launch file
