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
