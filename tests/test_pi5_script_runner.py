"""Tests for direct Jetson route script runner."""

from __future__ import annotations

import time

import pytest

from runtime.pi5_script_runner import Pi5ScriptRunner


@pytest.mark.parametrize("pause_reason", ["object_detected", "manual_override"])
def test_runner_pause_freezes_step_elapsed_and_resume_republishes(pause_reason: str):
    base_cmds: list[str] = []
    servo_angles: list[float] = []
    runner = Pi5ScriptRunner()
    runner.set_handlers(base_cmds.append, servo_angles.append, lambda _state: None)

    assert runner.submit([{"action": "left", "duration_s": 0.25}]) is True
    time.sleep(0.08)
    runner.set_paused(True, pause_reason)
    paused = runner.status()
    elapsed = paused["step_elapsed_s"]

    assert paused["paused"] is True
    assert paused["pause_reason"] == pause_reason
    time.sleep(0.16)
    assert runner.status()["step_elapsed_s"] == pytest.approx(elapsed, abs=0.02)

    runner.set_paused(False)
    assert base_cmds[-1] == "FORWARD"
    assert len(servo_angles) >= 2

    deadline = time.monotonic() + 1.0
    while runner.is_running() and time.monotonic() < deadline:
        time.sleep(0.02)

    assert runner.is_running() is False
    assert base_cmds[-1] == "STOP"


class _FakeHeading:
    """Yaw grows at omega deg/s (left positive) once the step has reset it."""

    def __init__(self, omega: float = 40.0, valid: bool = True, psi: float | None = None, offset: float | None = None):
        self.omega = omega
        self.valid = valid
        self.psi = psi
        self.offset = offset
        self.t0 = time.monotonic()
        self.resets = 0

    def reset(self) -> None:
        self.t0 = time.monotonic()
        self.resets += 1

    def sample(self):
        from vision.grid_heading import YawSample

        dt = time.monotonic() - self.t0
        return YawSample(
            yaw_deg=self.omega * dt, omega_dps=self.omega, psi_deg=self.psi, source="grid",
            age_s=0.0, valid=self.valid, lane_offset_ratio=self.offset,
        )


def _wait_done(runner: Pi5ScriptRunner, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while runner.is_running() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert runner.is_running() is False


def test_degree_turn_stops_at_target_minus_latency_lead():
    heading = _FakeHeading(omega=200.0)
    base_cmds: list[str] = []
    runner = Pi5ScriptRunner()
    runner.turn.latency_s = 0.1
    runner.set_handlers(base_cmds.append, lambda _a: None, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    t0 = time.monotonic()
    assert runner.submit([{"action": "left", "degrees": 90}]) is True
    _wait_done(runner)
    took = time.monotonic() - t0
    turn = runner.status()["last_turn"]
    # omega is capped at 2x nominal (80 deg/s) -> lead 8 deg, stop at >= 82 deg.
    assert turn["lead_deg"] == pytest.approx(8.0)
    assert turn["seen_at_stop_deg"] >= 82.0
    assert turn["fallback_time"] is False
    assert took < 0.8
    assert heading.resets == 1
    assert base_cmds[0] == "FORWARD" and base_cmds[-1] == "STOP"


def test_right_degree_turn_uses_negative_yaw():
    heading = _FakeHeading(omega=-60.0)
    angles: list[float] = []
    runner = Pi5ScriptRunner()
    runner.turn.latency_s = 0.0
    runner.set_handlers(lambda _c: None, angles.append, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    assert runner.submit([{"action": "right", "degrees": 30}]) is True
    _wait_done(runner)
    assert runner.status()["last_turn"]["seen_at_stop_deg"] >= 30.0
    assert angles[0] < runner._center_angle


def test_degree_turn_falls_back_to_time_when_heading_is_lost():
    heading = _FakeHeading(valid=False)
    runner = Pi5ScriptRunner()
    runner.turn.latency_s = 0.05
    runner.turn.heading_lost_s = 0.1
    runner.turn.nominal_omega_dps = 100.0
    runner.turn.finish_on_time = "time" == "time"
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    assert runner.submit([{"action": "left", "degrees": 30}]) is True
    _wait_done(runner)
    turn = runner.status()["last_turn"]
    assert turn["fallback_time"] is True
    assert turn["elapsed_s"] == pytest.approx(0.3, abs=0.1)


def test_degree_turn_stops_script_when_heading_lost_and_mode_stop():
    heading = _FakeHeading(valid=False)
    base_cmds: list[str] = []
    runner = Pi5ScriptRunner()
    runner.turn.latency_s = 0.05
    runner.turn.heading_lost_s = 0.1
    runner.turn.finish_on_time = "stop" == "time"
    runner.set_handlers(base_cmds.append, lambda _a: None, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    assert runner.submit([{"action": "left", "degrees": 90}, {"action": "forward", "duration_s": 1.0}]) is True
    _wait_done(runner)
    assert "heading lost" in runner.status()["last_error"]
    assert "FORWARD" == base_cmds[0] and base_cmds.count("FORWARD") == 1
    assert base_cmds[-1] == "STOP"


def test_degree_steps_need_a_heading_source():
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    with pytest.raises(ValueError):
        runner.submit([{"action": "left", "degrees": 90}])
    with pytest.raises(ValueError):
        runner.submit([{"action": "settle", "duration_s": 1.0}])


def test_settle_steers_right_when_car_points_left_and_is_rate_limited():
    heading = _FakeHeading(omega=0.0, psi=15.0, offset=0.0)
    base_cmds: list[str] = []
    angles: list[float] = []
    runner = Pi5ScriptRunner()
    runner.turn.settle_k_psi = 1.0
    runner.turn.settle_k_offset = 0.0
    runner.turn.settle_max_steer = 20.0
    runner.turn.settle_rate_dps = 50.0
    runner.turn.settle_exit_hold_s = 0.0
    runner.set_handlers(base_cmds.append, angles.append, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    assert runner.vision_pid_active() is True
    assert runner.submit([{"action": "settle", "duration_s": 0.6}]) is True
    time.sleep(0.1)
    assert runner.vision_pid_active() is False
    assert runner.needs_heading() is True
    _wait_done(runner)
    centre = runner._center_angle
    assert base_cmds[0] == "FORWARD"
    assert min(angles) == pytest.approx(centre - 15.0, abs=0.6)
    # Drop the recentre the runner sends when the script ends.
    ramp = angles[:-1]
    steps = [abs(b - a) for a, b in zip(ramp, ramp[1:])]
    assert max(steps) <= 50.0 * 0.15


def test_settle_exits_early_once_aligned():
    heading = _FakeHeading(omega=0.0, psi=1.0, offset=0.02)
    runner = Pi5ScriptRunner()
    runner.turn.settle_exit_hold_s = 0.15
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    t0 = time.monotonic()
    assert runner.submit([{"action": "settle", "duration_s": 3.0}]) is True
    _wait_done(runner)
    assert time.monotonic() - t0 < 1.0
