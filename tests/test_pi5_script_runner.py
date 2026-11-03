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


def test_left_and_right_steps_use_per_side_steer_offsets(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("SERVO_CENTER_ANGLE", "-28")
    runner = Pi5ScriptRunner()
    runner.turn.left_steer_deg = 70.0
    runner.turn.right_steer_deg = 50.0
    assert runner._steer_angle(+1) == pytest.approx(62.0 + 70.0)
    assert runner._steer_angle(-1) == pytest.approx(62.0 - 50.0)
    # A right offset past the servo range is clamped to 0, not sent as a negative angle.
    runner.turn.right_steer_deg = 80.0
    assert runner._steer_angle(-1) == 0.0


def test_turn_steps_report_rotation_for_the_map():
    turns: list[float] = []
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    runner.turn.nominal_omega_dps = 900.0
    runner.set_turn_done_callback(turns.append)
    assert runner.submit([{"action": "right", "duration_s": 0.1}, {"action": "forward", "duration_s": 0.05}]) is True
    deadline = time.monotonic() + 1.0
    while runner.is_running() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert len(turns) == 1
    assert turns[0] < -45.0


def _wait(cond, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while not cond() and time.monotonic() < deadline:
        time.sleep(0.01)


def test_lane_recovery_steers_back_then_resumes_the_forward_step(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("SERVO_CENTER_ANGLE", "-28")
    angles: list[float] = []
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, angles.append, lambda _s: None)
    runner.turn.lane_recover_steer_deg = 15.0
    assert runner.submit([{"action": "forward", "duration_s": 0.4}]) is True
    _wait(lambda: runner.current_action() == "forward")
    assert runner.request_lane_recover(+1) is True  # drifted left
    _wait(lambda: runner.vision_pid_active() is False)
    assert runner.recovering() == 1
    assert angles[-1] == pytest.approx(62.0 - 15.0)  # steer right
    elapsed = runner.status()["step_elapsed_s"]
    time.sleep(0.2)
    assert runner.status()["step_elapsed_s"] == pytest.approx(elapsed, abs=0.06)
    runner.lane_recovered()
    _wait(lambda: runner.vision_pid_active())
    assert runner.recovering() == 0
    _wait(lambda: not runner.is_running())
    assert runner.status()["last_error"] in (None, "")


def test_lane_recovery_times_out_and_stops_the_script():
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    runner.turn.lane_recover_timeout_s = 0.2
    assert runner.submit([{"action": "forward", "duration_s": 5.0}]) is True
    _wait(lambda: runner.current_action() == "forward")
    assert runner.request_lane_recover(-1) is True
    _wait(lambda: not runner.is_running())
    assert "lane recovery failed" in runner.status()["last_error"]


def test_lane_recovery_only_in_forward_steps():
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    assert runner.submit([{"action": "left", "duration_s": 0.3}]) is True
    _wait(lambda: runner.current_action() == "left")
    assert runner.request_lane_recover(1) is False


class _FakeFloor:
    """Car on a tiled floor: FORWARD moves it at speed cm/s; seams every 60 cm
    show at image row y = 1 - distance_ahead / 300 (0 = top)."""

    def __init__(self, speed: float = 30.0, start_cm: float = 0.0) -> None:
        self.speed = speed
        self.s = start_cm
        self.cmd = "STOP"
        self.t = time.monotonic()
        self.cmds: list[str] = []

    def _advance(self) -> None:
        now = time.monotonic()
        if self.cmd == "FORWARD":
            self.s += self.speed * (now - self.t)
        self.t = now

    def base(self, cmd: str) -> None:
        self._advance()
        self.cmd = cmd
        self.cmds.append(cmd)

    def lines(self):
        self._advance()
        ys = [1.0 - (k * 60.0 - self.s) / 300.0 for k in range(0, 10)]
        return time.monotonic(), [y for y in ys if 0.05 < y < 0.95], 0.8


def test_anchor_turn_nudges_to_the_target_row_then_logs_errors_and_suggests():
    floor = _FakeFloor(speed=30.0, start_cm=-3.0)  # V just past the counter line (row 0.79), V+1 at 0.59
    heading = _FakeHeading(omega=300.0, psi=-6.0, offset=-0.1)
    runner = Pi5ScriptRunner()
    runner.turn.latency_s = 0.05
    runner.turn.anchor_y_left = 0.72
    runner.turn.anchor_wait_s = 0.05
    runner.set_handlers(floor.base, lambda _a: None, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    runner.set_anchor_source(floor.lines)
    assert runner.submit([
        {"action": "left", "degrees": 90, "anchor": "next_line"},
        {"action": "settle", "duration_s": 0.3},
    ]) is True
    _wait_done(runner, timeout=10.0)
    status = runner.status()
    assert status["last_error"] in (None, "")
    anchor = status["last_turn"]["anchor"]
    assert abs(anchor["y"] - 0.72) <= runner.turn.anchor_tol + 0.005
    assert anchor["tile_dy"] == pytest.approx(0.2, abs=0.01)  # 60 cm / 300
    assert "STOP" in floor.cmds
    entry = status["turn_log"][-1]
    assert entry["e_psi"] == pytest.approx(6.0)  # psi -6 after a left turn: turned too little
    assert entry["e_y_cm"] == pytest.approx(6.0)  # centre 0.1 left of the car: past the row middle
    sug = status["turn_suggest"]
    assert sug["turn_latency_s"] < 0.05 + 1e-9
    assert sug["turn_anchor_y_left"] < 0.72


def test_anchor_turn_turns_anyway_without_lines():
    # No seam to anchor on (e.g. a floor without visible tiles): the turn still
    # runs where the car is, and the log says the anchor was lost.
    runner = Pi5ScriptRunner()
    runner.turn.anchor_lost_s = 0.2
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    heading = _FakeHeading(omega=300.0)
    runner.set_heading_source(heading.reset, heading.sample)
    runner.set_anchor_source(lambda: (time.monotonic(), [], 0.8))
    assert runner.submit([{"action": "left", "degrees": 90, "anchor": "next_line"}]) is True
    _wait_done(runner)
    st = runner.status()
    assert st["last_error"] == ""
    assert st["last_turn"] is not None
    assert "lost" in st["turn_log"][-1]["anchor_lost"]
    assert st["turn_log"][-1]["anchor_y"] is None


class _VpFloor(_FakeFloor):
    """Seams every 60 cm; a seam d cm ahead of the front axle shows at
    row vp + k / (d - c) of a 480 px frame (vision/ground_range.py)."""

    K, C, VP, H = 150000.0, 10.0, -340.0, 480

    def lines(self):
        self._advance()
        ys = []
        for k in range(0, 12):
            d = k * 60.0 - self.s
            if d - self.C > 1.0:
                y = (self.VP + self.K / (d - self.C)) / self.H
                if 0.05 < y < 0.95:
                    ys.append(y)
        return time.monotonic(), ys, 0.8, self.VP, self.H


def test_anchor_target_comes_from_measured_turn_length_and_vp():
    runner = Pi5ScriptRunner()
    cfg = runner.turn
    cfg.tile_cm, cfg.ground_k_px_cm, cfg.ground_c_cm = 60.0, _VpFloor.K, _VpFloor.C
    cfg.anchor_y_left = 0.6
    assert runner._anchor_target(1.0, -340.0, 480) == (0.6, "manual")  # a = 0
    cfg.a_left_cm = 200.0  # d_V = 170, V+1 at 230 cm
    y, src = runner._anchor_target(1.0, -340.0, 480)
    assert src == "vp"
    assert y == pytest.approx((-340.0 + 150000.0 / 220.0) / 480.0)
    assert runner._anchor_target(1.0, None, None) == (0.6, "manual")
    cfg.a_left_cm = 100.0  # V+1 at 130 cm: below the image
    assert runner._anchor_target(1.0, -340.0, 480)[1] == "manual"


def test_anchor_turn_with_vp_target_stops_with_v_plus_1_at_the_turn_distance():
    floor = _VpFloor(speed=30.0, start_cm=23.0)  # V (240 cm seam) just past the counter line
    heading = _FakeHeading(omega=300.0, psi=0.0, offset=-0.1)
    runner = Pi5ScriptRunner()
    cfg = runner.turn
    cfg.latency_s = 0.05
    cfg.anchor_wait_s = 0.05
    cfg.tile_cm, cfg.ground_k_px_cm, cfg.ground_c_cm = 60.0, _VpFloor.K, _VpFloor.C
    cfg.a_left_cm = 200.0
    runner.set_handlers(floor.base, lambda _a: None, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    runner.set_anchor_source(floor.lines)
    assert runner.submit([
        {"action": "left", "degrees": 90, "anchor": "next_line"},
        {"action": "settle", "duration_s": 0.3},
    ]) is True
    _wait_done(runner, timeout=10.0)
    status = runner.status()
    assert status["last_error"] in (None, "")
    anchor = status["last_turn"]["anchor"]
    assert anchor["target_src"] == "vp"
    assert anchor["vp_row"] == -340.0
    y_px = anchor["y"] * _VpFloor.H
    d = _VpFloor.C + _VpFloor.K / (y_px - _VpFloor.VP)
    assert d == pytest.approx(230.0, abs=4.0)  # V+1 = d_V + L ahead of the front axle
    sug = status["turn_suggest"]
    assert sug["turn_a_left_cm"] == pytest.approx(200.0 + 0.3 * 6.0)  # turned late: start earlier
    assert "turn_a_right_cm" not in sug


def test_active_lane_change_steers_toward_the_side_until_the_crossing(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("SERVO_CENTER_ANGLE", "-28")
    angles: list[float] = []
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, angles.append, lambda _s: None)
    runner.turn.lane_change_steer_deg = 12.0
    assert runner.submit([{"action": "lane_left", "duration_s": 2.0}]) is True
    _wait(lambda: runner.expected_crossing() == 1)
    assert runner.vision_pid_active() is False
    assert angles[-1] == pytest.approx(62.0 + 12.0)  # left is above centre
    runner.lane_crossed()
    _wait(lambda: not runner.is_running())
    assert runner.expected_crossing() == 0
    assert runner.status()["last_error"] in (None, "")


def test_active_lane_change_fails_without_a_crossing():
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    assert runner.submit([{"action": "lane_right", "duration_s": 0.2}]) is True
    _wait(lambda: not runner.is_running())
    assert "lane change right" in runner.status()["last_error"]


def test_lane_change_straightens_with_the_settle_law_after_crossing(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("SERVO_CENTER_ANGLE", "-28")
    angles: list[float] = []
    heading = _FakeHeading(omega=0.0, psi=10.0, offset=0.0)  # still pointing left
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, angles.append, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    runner.turn.lane_align_timeout_s = 0.4
    runner.turn.settle_k_psi = 1.0
    assert runner.submit([{"action": "lane_left", "duration_s": 2.0}]) is True
    _wait(lambda: runner.expected_crossing() == 1)
    assert runner.needs_heading() is True
    runner.lane_crossed()
    _wait(lambda: not runner.is_running())
    assert min(angles) < 62.0  # steered right of centre to cancel the heading
    assert runner.status()["last_error"] in (None, "")


def test_passive_mode_off_ignores_drifts():
    runner = Pi5ScriptRunner()
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    runner.turn.lane_recover = False
    assert runner.submit([{"action": "forward", "duration_s": 0.3}]) is True
    _wait(lambda: runner.current_action() == "forward")
    assert runner.request_lane_recover(1) is False


def test_lane_steps_pass_validation():
    from runtime.resource_limits import validate_route_steps

    steps = validate_route_steps([{"action": "lane_left", "duration_s": 3}, {"action": "lane_right", "duration_s": 3}])
    assert [s["action"] for s in steps] == ["lane_left", "lane_right"]


def _calib_turn(runner: Pi5ScriptRunner, side: str, psi: float, offset: float) -> None:
    from types import SimpleNamespace

    runner._record_turn(side, 90.0, 80.0, 30.0, 10.0, 3.0, "grid", False)
    sample = SimpleNamespace(valid=True, psi_deg=psi, lane_offset_ratio=offset)
    for _ in range(3):
        runner._measure_turn_error(sample, 0.1)


def test_calib_session_keeps_turns_across_scripts_and_deletes_bad_runs():
    runner = Pi5ScriptRunner()
    _calib_turn(runner, "left", -5.0, 0.0)  # no session yet: not collected
    assert runner.calib_status()["log"] == []

    runner.calib_start()
    _calib_turn(runner, "left", -5.0, 0.0)
    runner._turn_log = []  # what a new script submit does
    _calib_turn(runner, "right", 4.0, 0.0)
    _calib_turn(runner, "left", -40.0, 0.0)  # a bad run
    status = runner.calib_status()
    assert [t["id"] for t in status["log"]] == [1, 2, 3]
    assert status["counts"] == {"left": 2, "right": 1}
    assert status["log"][0]["e_psi"] == pytest.approx(5.0)
    with_bad = runner.calib_suggestion()

    status = runner.calib_delete(3)
    assert [t["id"] for t in status["log"]] == [1, 2]
    assert runner.calib_suggestion() != with_bad
    with pytest.raises(ValueError):
        runner.calib_delete(3)

    runner.calib_start()  # new session drops the old one
    assert runner.calib_status()["log"] == []


def test_turn_test_steps_always_steer(monkeypatch: pytest.MonkeyPatch):
    from positioning.route_steps import turn_test_steps
    from runtime.resource_limits import validate_route_steps

    steps = turn_test_steps("left")
    assert [s["action"] for s in steps] == ["forward", "left", "settle", "forward"]
    assert steps[1] == {"action": "left", "degrees": 90.0, "anchor": "next_line"}
    assert validate_route_steps(steps) == steps
    monkeypatch.setenv("TILE_TURN_ACTION", "turn")  # steered either way
    assert turn_test_steps("right")[1]["action"] == "right"
    with pytest.raises(ValueError):
        turn_test_steps("up")


class _JumpyHeading(_FakeHeading):
    """Real yaw at 40 deg/s, but one frame reads +60 deg early in the turn."""

    def sample(self):
        out = super().sample()
        dt = time.monotonic() - self.t0
        if 0.2 <= dt < 0.26:
            from dataclasses import replace

            return replace(out, yaw_deg=60.0)
        return out


def test_degree_turn_ignores_a_single_frame_heading_jump():
    heading = _JumpyHeading(omega=40.0)
    runner = Pi5ScriptRunner()
    runner.turn.latency_s = 0.1
    runner.set_handlers(lambda _c: None, lambda _a: None, lambda _s: None)
    runner.set_heading_source(heading.reset, heading.sample)
    t0 = time.monotonic()
    assert runner.submit([{"action": "left", "degrees": 90}]) is True
    _wait_done(runner, timeout=6.0)
    # Without the gate the 60 deg frame would end the turn at ~0.2 s.
    assert time.monotonic() - t0 > 1.5
    assert runner.status()["last_turn"]["seen_at_stop_deg"] > 75
