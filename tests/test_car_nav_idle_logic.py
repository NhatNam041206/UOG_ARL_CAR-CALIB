"""Idle / resume rules (ros/car_nav/car_nav/idle_logic.py)."""
from tests import _car_nav_paths as P  # noqa: F401
from car_nav.idle_logic import (IdleTracker, from_rest, is_idle, is_nonzero, resume_gate, should_process)


def test_is_idle():
    assert not is_idle(5.0, 0.0, False, 10.0)          # not long enough
    assert is_idle(10.0, 0.0, False, 10.0)
    assert not is_idle(100.0, 0.0, True, 10.0)         # a goal is active: never idle


def test_should_process_decimates_only_while_idle():
    assert should_process(False, 1.00, 0.99, 0.5)      # active: every frame
    assert not should_process(True, 1.00, 0.80, 0.5)   # idle: one per 0.5 s
    assert should_process(True, 1.31, 0.80, 0.5)
    assert not should_process(False, 1.0, 0.95, 0.5, min_period_s=0.08)   # min period still holds when active
    assert should_process(False, 1.0, 0.90, 0.5, min_period_s=0.08)


def test_tracker_wakes_on_activity_and_goal():
    t = [0.0]
    tr = IdleTracker(10.0, clock=lambda: t[0])
    t[0] = 9.0
    assert not tr.idle()
    t[0] = 10.5
    assert tr.idle()
    tr.activity()                                        # a non-zero command
    assert not tr.idle()
    t[0] = 25.0
    assert tr.idle()
    tr.set_goal(True)                                    # a goal arrives: awake at once
    assert not tr.idle()
    t[0] = 100.0
    assert not tr.idle() and tr.goal_active              # and stays awake while it is active
    tr.set_goal(False)
    assert not tr.idle()                                 # the goal's end counts from "now"... (activity on start)
    t[0] = 120.0
    assert tr.idle()


def test_nonzero_and_from_rest():
    assert not is_nonzero(0.0, 0.0) and not is_nonzero(0.0005, -0.0005)
    assert is_nonzero(0.1, 0.0) and is_nonzero(0.0, -0.3)
    assert from_rest(5.0, 3.0) and not from_rest(5.0, 4.8)


def test_resume_gate_never_passes_a_start_on_data_from_before_it():
    # the scan came 0.4 s BEFORE the command (the pause): blind -> blocked
    assert not resume_gate(start_t=10.0, scan_t=9.6)
    assert resume_gate(start_t=10.0, scan_t=10.05)
    # depth in use: the cloud must be newer too
    assert not resume_gate(10.0, 10.1, depth_in_use=True, depth_t=9.0)
    assert resume_gate(10.0, 10.1, depth_in_use=True, depth_t=10.2)
    assert resume_gate(10.0, 10.1, depth_in_use=False, depth_t=0.0)
