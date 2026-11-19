"""When may expensive work run slower: idle and resume rules as pure functions (no ROS imports).

The car is IDLE when no goal is active and nothing has asked the wheels to move for
`idle_after_s`. While idle, the map alignment of dynamic_scan and the depth node's processing
run at a low rate (`idle_period_s`) instead of for every frame. Localization (AMCL, pose_gate,
odometry) is never paused.

Resume BEFORE the car can move:

* any non-zero /cmd_vel_guarded or /cmd_vel, a goal that becomes active, or the E-STOP changing
  ends the idle state at once (IdleTracker.activity / set_goal), so every following frame is
  processed;
* the safety guard never lets a command through when it comes from rest unless it has seen a
  scan (and depth cloud, when depth is in use) that arrived AFTER that command
  (resume_gate), so a start can never be judged on data from before the pause.
  Costs at most one frame period (~0.15 s) at each start.
"""
from __future__ import annotations

import time
from typing import Callable

NONZERO = 1e-3          # |v| or |w| above this is "a command to move"
REST_S = 0.6            # no non-zero command for this long = the car is at rest (a start follows)


def is_idle(now: float, last_activity_t: float, goal_active: bool, idle_after_s: float) -> bool:
    """True when there is no goal and the last sign of activity is older than idle_after_s."""
    if goal_active:
        return False
    return (now - last_activity_t) >= idle_after_s


def should_process(idle: bool, now: float, last_processed_t: float, idle_period_s: float,
                   min_period_s: float = 0.0) -> bool:
    """Process this frame? Active: every frame (at most one per min_period_s). Idle: one per idle_period_s."""
    period = max(idle_period_s, min_period_s) if idle else min_period_s
    return (now - last_processed_t) >= period


def is_nonzero(linear_x: float, angular_z: float) -> bool:
    return abs(linear_x) > NONZERO or abs(angular_z) > NONZERO


def from_rest(now: float, last_nonzero_t: float, rest_s: float = REST_S) -> bool:
    """Is a non-zero command arriving now a start from rest (nothing non-zero for rest_s)?"""
    return (now - last_nonzero_t) > rest_s


def resume_gate(start_t: float, scan_t: float, depth_in_use: bool = False, depth_t: float = -1e9) -> bool:
    """May the guard pass a command that started at start_t? Only if the LiDAR scan (and the depth
    cloud, if depth is in use) it judges with arrived at or after start_t."""
    if scan_t < start_t:
        return False
    if depth_in_use and depth_t < start_t:
        return False
    return True


class IdleTracker:
    """Idle / active state from events. `clock` is injectable for tests."""

    def __init__(self, idle_after_s: float = 10.0, clock: Callable[[], float] = time.monotonic) -> None:
        self.idle_after_s = idle_after_s
        self._clock = clock
        self._last = clock()
        self._goal = False

    def activity(self, t: float | None = None) -> None:
        """A non-zero command, a goal, an E-STOP change: the car may be about to move."""
        self._last = self._clock() if t is None else t

    def set_goal(self, active: bool) -> None:
        if active != self._goal:      # a goal starting or ending: the car may be moving
            self.activity()
        self._goal = active

    @property
    def goal_active(self) -> bool:
        return self._goal

    def idle(self, now: float | None = None) -> bool:
        return is_idle(self._clock() if now is None else now, self._last, self._goal, self.idle_after_s)
