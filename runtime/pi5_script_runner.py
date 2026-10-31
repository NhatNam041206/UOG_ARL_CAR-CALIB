"""Route script runner for Pi5 (no direct hardware)."""

from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from config.settings import (
    LANE_ALIGN_TIMEOUT_S,
    LANE_CHANGE_STEER_DEG,
    LANE_RECOVER,
    LANE_RECOVER_STEER_DEG,
    LANE_RECOVER_TIMEOUT_S,
    SETTLE_EXIT_HOLD_S,
    SETTLE_EXIT_OFFSET,
    SETTLE_EXIT_PSI_DEG,
    SETTLE_K_OFFSET,
    SETTLE_K_PSI,
    SETTLE_MAX_STEER,
    SETTLE_RATE_DPS,
    TILE_STEP_MAX_S_PER_TILE,
    GROUND_C_CM,
    GROUND_K_PX_CM,
    TURN_A_LEFT_CM,
    TURN_A_RIGHT_CM,
    TURN_ADAPT_ALPHA,
    TURN_ANCHOR_LOST_S,
    TURN_ANCHOR_MAX_PULSES,
    TURN_ANCHOR_PULSE_S,
    TURN_ANCHOR_SLOW_MARGIN,
    TURN_ANCHOR_TIMEOUT_S,
    TURN_ANCHOR_TOL,
    TURN_ANCHOR_WAIT_S,
    TURN_ANCHOR_Y_LEFT,
    TURN_ANCHOR_Y_RIGHT,
    TURN_TILE_CM,
    TURN_HEADING_LOST_S,
    TURN_LEFT_STEER_DEG,
    TURN_LATENCY_S,
    TURN_NOMINAL_OMEGA_DPS,
    TURN_ON_HEADING_LOST,
    TURN_RIGHT_STEER_DEG,
    TURN_TIMEOUT_FACTOR,
)
from runtime.resource_limits import turn_timeout_s, validate_route_steps
from vision.ground_range import GroundRange

logger = logging.getLogger(__name__)


@dataclass
class TurnSettings:
    """Degree-turn and settle parameters; live-tunable from the dashboard Tune tab."""

    left_steer_deg: float = TURN_LEFT_STEER_DEG
    right_steer_deg: float = TURN_RIGHT_STEER_DEG
    latency_s: float = TURN_LATENCY_S
    nominal_omega_dps: float = TURN_NOMINAL_OMEGA_DPS
    timeout_factor: float = TURN_TIMEOUT_FACTOR
    heading_lost_s: float = TURN_HEADING_LOST_S
    finish_on_time: bool = TURN_ON_HEADING_LOST != "stop"
    settle_k_psi: float = SETTLE_K_PSI
    settle_k_offset: float = SETTLE_K_OFFSET
    settle_max_steer: float = SETTLE_MAX_STEER
    settle_rate_dps: float = SETTLE_RATE_DPS
    settle_exit_psi_deg: float = SETTLE_EXIT_PSI_DEG
    settle_exit_offset: float = SETTLE_EXIT_OFFSET
    settle_exit_hold_s: float = SETTLE_EXIT_HOLD_S
    anchor_y_left: float = TURN_ANCHOR_Y_LEFT
    anchor_y_right: float = TURN_ANCHOR_Y_RIGHT
    anchor_tol: float = TURN_ANCHOR_TOL
    anchor_slow_margin: float = TURN_ANCHOR_SLOW_MARGIN
    anchor_pulse_s: float = TURN_ANCHOR_PULSE_S
    anchor_wait_s: float = TURN_ANCHOR_WAIT_S
    anchor_max_pulses: int = TURN_ANCHOR_MAX_PULSES
    anchor_lost_s: float = TURN_ANCHOR_LOST_S
    anchor_timeout_s: float = TURN_ANCHOR_TIMEOUT_S
    tile_cm: float = TURN_TILE_CM
    a_left_cm: float = TURN_A_LEFT_CM
    a_right_cm: float = TURN_A_RIGHT_CM
    ground_k_px_cm: float = GROUND_K_PX_CM
    ground_c_cm: float = GROUND_C_CM
    adapt_alpha: float = TURN_ADAPT_ALPHA
    lane_recover: bool = LANE_RECOVER
    lane_recover_steer_deg: float = LANE_RECOVER_STEER_DEG
    lane_recover_timeout_s: float = LANE_RECOVER_TIMEOUT_S
    lane_change_steer_deg: float = LANE_CHANGE_STEER_DEG
    lane_align_timeout_s: float = LANE_ALIGN_TIMEOUT_S

BASE_MAP: dict[str, tuple[int, int, int]] = {
    "STOP": (0, 0, 0),
    "FORWARD": (0, 1, 0),
    "BACKWARD": (0, 0, 1),
    "LOCK": (1, 0, 1),
    "UNLOCK": (1, 1, 0),
    "TURN_LEFT": (1, 0, 0),
    "TURN_RIGHT": (0, 1, 1),
}

_VALID_ACTIONS = {
    "forward", "backward", "straight", "left", "right", "turn_left", "turn_right", "stop", "pause", "settle",
    "lane_left", "lane_right",
}
# Active lane change into the next tile row: +1 left, -1 right.
LANE_CHANGE_ACTIONS = {"lane_left": 1, "lane_right": -1}
# Steps that pin the servo themselves, so the vision PID must not publish.
_SERVO_PINNED_ACTIONS = ("left", "right", "backward", "settle", "lane_left", "lane_right")


class Pi5ScriptRunner:
    """Runs route scripts in a background thread using direct callbacks."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._paused = False
        self._pause_reason = ""
        self._steps: list[dict[str, Any]] = []
        self._current_step: dict[str, Any] | None = None
        self._current_step_idx = -1
        self._step_elapsed_s = 0.0
        self._step_duration_s = 0.0
        self._current_base_cmd = "STOP"
        self._current_angle: float | None = None
        self._base_cb: Callable[[str], None] | None = None
        self._servo_cb: Callable[[float], None] | None = None
        self._relay_cb: Callable[[str], None] | None = None
        self._center_angle: float = 90.0 + float(os.getenv("SERVO_CENTER_ANGLE", "-8"))
        # Tile steps: the counter line looks ahead of the car, so a step can
        # stop this many seams early (calibrate on a measured run).
        self._tile_count_lead = int(os.getenv("TILE_STEP_COUNT_LEAD", "0"))
        self._tile_count_getter: Callable[[], int] | None = None
        self._step_tiles_done = 0
        # Degree steps and settle read the grid heading (vision/grid_heading.py).
        self._heading_reset: Callable[[], None] | None = None
        self._heading_sample: Callable[[], Any] | None = None
        self._step_degrees_done = 0.0
        self._turn_done_cb: Callable[[float], None] | None = None
        # Lane recovery: side the car drifted to (+1 left), 0 = none pending.
        self._recover_side = 0
        self._recovering = False
        # Lane crossing the runner is steering for (+1 into the left lane),
        # 0 = none; set while a lane change or a recovery crosses a line.
        self._cross_side = 0
        self._crossed = threading.Event()
        # Turn approach to the next cross line: () -> (t, [line y ratios],
        # counter_y[, vp_row_px, frame_h]); the last two give y* from the VP.
        self._anchor_source: Callable[[], tuple[Any, ...] | None] | None = None
        self._approaching = False
        # One entry per finished turn; e_psi / e_y are filled by the next settle.
        self._turn_log: list[dict[str, Any]] = []
        self._pending_turn: dict[str, Any] | None = None
        self._last_turn: dict[str, Any] | None = None
        self.turn = TurnSettings()
        self._last_error = ""

    def set_handlers(
        self,
        base_cb: Callable[[str], None],
        servo_cb: Callable[[float], None],
        relay_cb: Callable[[str], None],
    ) -> None:
        self._base_cb = base_cb
        self._servo_cb = servo_cb
        self._relay_cb = relay_cb

    def set_tile_count_getter(self, fn: Callable[[], int]) -> None:
        """Running cross-line tile count (needed for {"tiles": N} steps)."""
        self._tile_count_getter = fn

    def _steer_angle(self, side: int) -> float:
        """Servo angle for a `left` (+1) or `right` (-1) step, clamped to 0..180."""
        offset = self.turn.left_steer_deg if side > 0 else self.turn.right_steer_deg
        angle = self._center_angle + side * abs(float(offset))
        clamped = max(0.0, min(180.0, angle))
        if clamped != angle:
            logger.warning("%s steer %.1f clamped to servo %.1f", "left" if side > 0 else "right", angle, clamped)
        return clamped

    def set_heading_source(self, reset_fn: Callable[[], None], sample_fn: Callable[[], Any]) -> None:
        """Turn yaw tracker (needed for {"degrees": N} and settle steps).

        sample_fn returns an object with yaw_deg, omega_dps, psi_deg,
        lane_offset_ratio and valid (vision.grid_heading.YawSample).
        """
        self._heading_reset = reset_fn
        self._heading_sample = sample_fn

    def set_anchor_source(self, fn: Callable[[], tuple[Any, ...] | None]) -> None:
        """Cross lines for {"anchor": "next_line"} turns: fn() returns the
        frame time, the y ratio (centre column) of every line seen, and the
        counter line's y ratio."""
        self._anchor_source = fn

    def set_turn_done_callback(self, fn: Callable[[float], None]) -> None:
        """Called after each completed turn step with the rotation, left positive.

        Degree steps report the degrees asked for (the step ran until the
        heading showed them); time steps report duration x nominal yaw rate.
        The map rounds it to quarter turns.
        """
        self._turn_done_cb = fn

    def needs_heading(self) -> bool:
        """True while the current step reads the heading (main loop runs the estimator)."""
        with self._lock:
            step = self._current_step if self._running else None
        return step is not None and (
            step.get("degrees") is not None
            or step.get("action") == "settle"
            or step.get("action") in LANE_CHANGE_ACTIONS
            or self._recover_side != 0
        )

    def current_action(self) -> str | None:
        with self._lock:
            if not self._running or self._current_step is None:
                return None
            return str(self._current_step.get("action", "stop"))

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def request_lane_recover(self, side: int) -> bool:
        """A passive lane shift to `side` (+1 left) happened: steer back.

        Only when passive mode is on and during a forward step; returns False
        when not taken."""
        with self._lock:
            step = self._current_step if self._running else None
            if (
                not self.turn.lane_recover
                or step is None
                or step.get("action") not in ("forward", "straight")
                or self._recovering
            ):
                return False
            self._recover_side = 1 if side > 0 else -1
        self._crossed.clear()
        return True

    def recovering(self) -> int:
        """Side being recovered from (+1 left, -1 right), 0 when not recovering."""
        with self._lock:
            return self._recover_side

    def expected_crossing(self) -> int:
        """Side of the lane line the runner is steering across (+1 into the
        left lane, -1 right), 0 when it is not changing lanes."""
        with self._lock:
            return self._cross_side

    def lane_crossed(self) -> None:
        """The detector saw the car cross into the lane the runner steered for."""
        self._crossed.set()

    # Older name, kept for callers of the passive recovery.
    lane_recovered = lane_crossed

    def vision_pid_active(self) -> bool:
        """True when current step should let vision PID control servo."""
        with self._lock:
            if self._paused or self._recovering:
                return False
            if self._approaching:
                return True  # driving straight to the turn point
            if not self._running or self._current_step is None:
                return True
            action = self._current_step.get("action", "stop")
        return action not in _SERVO_PINNED_ACTIONS

    def status(self) -> dict[str, Any]:
        with self._lock:
            remaining = max(0.0, self._step_duration_s - self._step_elapsed_s)
            return {
                "running": self._running,
                "paused": self._paused,
                "pause_reason": self._pause_reason,
                "current_step": self._current_step_idx + 1 if self._running else 0,
                "step": dict(self._current_step) if self._current_step is not None else None,
                "total": len(self._steps),
                "step_elapsed_s": self._step_elapsed_s,
                "step_remaining_s": remaining,
                "step_tiles_done": self._step_tiles_done,
                "step_degrees_done": round(self._step_degrees_done, 1),
                "last_turn": dict(self._last_turn) if self._last_turn is not None else None,
                "lane_recovering": self._recover_side,
                "lane_crossing": self._cross_side,
                "turn_log": [dict(t) for t in self._turn_log[-20:]],
                "turn_suggest": self._turn_suggestion_locked(),
                "last_error": self._last_error,
            }

    def submit(self, steps: list[dict[str, Any]]) -> bool:
        normalized_steps = validate_route_steps(steps)
        if any("tiles" in s for s in normalized_steps) and self._tile_count_getter is None:
            raise ValueError("tile steps need the cross-line counter")
        needs_heading = any(s.get("degrees") is not None or s.get("action") == "settle" for s in normalized_steps)
        if needs_heading and self._heading_sample is None:
            raise ValueError("degree and settle steps need the grid heading")
        with self._lock:
            if self._running:
                return False
            self._running = True
            self._paused = False
            self._pause_reason = ""
            self._step_elapsed_s = 0.0
            self._step_duration_s = 0.0
            self._steps = normalized_steps
            self._last_turn = None
            self._last_error = ""
            self._turn_log = []
            self._pending_turn = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        with self._lock:
            self._running = False
            self._paused = False
            self._pause_reason = ""

    def set_paused(self, paused: bool, reason: str = "") -> None:
        base_cmd: str | None = None
        angle: float | None = None
        with self._lock:
            if not self._running:
                self._paused = False
                self._pause_reason = ""
                return
            was_paused = self._paused
            self._paused = bool(paused)
            self._pause_reason = str(reason) if paused else ""
            if was_paused and not self._paused:
                base_cmd = self._current_base_cmd
                angle = self._current_angle

        if base_cmd is not None and self._base_cb is not None:
            self._base_cb(base_cmd)
        if angle is not None and self._servo_cb is not None:
            self._servo_cb(angle)

    def _run(self) -> None:
        logger.info("Route script start (%d steps)", len(self._steps))
        try:
            for idx, step in enumerate(self._steps):
                if not self.is_running():
                    break
                with self._lock:
                    self._current_step_idx = idx
                    self._current_step = step
                self._execute_step(step)
        except Exception as exc:
            logger.exception("Route script crashed: %s", exc)
        finally:
            with self._lock:
                was_paused = self._paused
                self._current_step = None
                self._current_step_idx = -1
                self._step_elapsed_s = 0.0
                self._step_duration_s = 0.0
                self._current_base_cmd = "STOP"
                self._current_angle = None
                self._running = False
                self._paused = False
                self._pause_reason = ""
            if self._base_cb is not None:
                self._base_cb("STOP")
            if self._servo_cb is not None and not was_paused:
                self._servo_cb(self._center_angle)
            logger.info("Route script finished")

    def _execute_step(self, step: dict[str, Any]) -> None:
        action = step.get("action", "stop")
        tiles = step.get("tiles")
        degrees = step.get("degrees")
        cfg = self.turn
        if degrees is not None:
            duration_s = turn_timeout_s(float(degrees), cfg.latency_s, cfg.nominal_omega_dps, cfg.timeout_factor)
            # Left turns give positive yaw.
            turn_sign = 1.0 if action in ("left", "turn_left") else -1.0
            anchor_info: dict[str, Any] | None = None
            if step.get("anchor"):
                anchor_info = self._approach_anchor(turn_sign)
                if anchor_info is None:
                    return
            if self._heading_reset is not None:
                self._heading_reset()
            last_valid_elapsed = 0.0
            fallback_end_s: float | None = None
        elif tiles is not None:
            # Safety timeout: the step fails (and the script stops) if the
            # counter does not see the seams in time.
            duration_s = float(tiles) * TILE_STEP_MAX_S_PER_TILE
            target = max(1, int(tiles) - self._tile_count_lead)
            last_count = int(self._tile_count_getter()) if self._tile_count_getter else 0
            done_tiles = 0
        else:
            duration_s = max(0.0, float(step.get("duration_s", 1.0)))

        if action in LANE_CHANGE_ACTIONS:
            self._lane_change(LANE_CHANGE_ACTIONS[action], duration_s)
            return

        if action in ("forward", "straight"):
            base_cmd = "FORWARD"
            angle: float | None = None
        elif action == "backward":
            base_cmd = "BACKWARD"
            angle = self._center_angle
        elif action == "left":
            base_cmd = "FORWARD"
            angle = self._steer_angle(+1)
        elif action == "right":
            base_cmd = "FORWARD"
            angle = self._steer_angle(-1)
        elif action == "turn_left":
            base_cmd = "TURN_LEFT"
            angle = None
        elif action == "turn_right":
            base_cmd = "TURN_RIGHT"
            angle = None
        elif action == "settle":
            base_cmd = "FORWARD"
            angle = self._center_angle
            settle_ok_s = 0.0
        else:
            base_cmd = "STOP"
            angle = None

        with self._lock:
            self._step_elapsed_s = 0.0
            self._step_duration_s = duration_s
            self._step_tiles_done = 0
            self._step_degrees_done = 0.0
            self._current_base_cmd = base_cmd
            self._current_angle = angle

        self._publish_current_command(base_cmd, angle)

        last_tick = time.monotonic()
        while True:
            time.sleep(0.05)
            now = time.monotonic()
            with self._lock:
                if not self._running:
                    break
                if self._paused:
                    last_tick = now
                    continue
                dt = now - last_tick
                self._step_elapsed_s += dt
                elapsed = self._step_elapsed_s
                timed_out = self._step_elapsed_s >= self._step_duration_s
                last_tick = now
                recover_side = self._recover_side
            if recover_side:
                if not self._recover_lane(recover_side, base_cmd):
                    break
                last_tick = time.monotonic()  # recovery time is not step time
                continue
            if degrees is not None:
                sample = self._heading_sample() if self._heading_sample is not None else None
                valid = sample is not None and bool(sample.valid)
                seen = turn_sign * float(sample.yaw_deg) if sample is not None else 0.0
                if valid:
                    last_valid_elapsed = elapsed
                omega = turn_sign * float(sample.omega_dps) if valid and sample.omega_dps is not None else 0.0
                if omega < 5.0:
                    omega = cfg.nominal_omega_dps
                omega = min(omega, 2.0 * cfg.nominal_omega_dps)
                # The camera sees the car latency_s late and the stop takes
                # as long to act: the car keeps turning omega * latency after this.
                lead = omega * cfg.latency_s
                with self._lock:
                    self._step_degrees_done = seen
                if fallback_end_s is None and valid and seen >= float(degrees) - lead:
                    self._record_turn(action, degrees, seen, omega, lead, elapsed, sample.source, False, anchor_info)
                    break
                # At the start the car has not moved yet (latency), so allow that long.
                lost = elapsed - max(last_valid_elapsed, cfg.latency_s)
                if fallback_end_s is None and not valid and lost > cfg.heading_lost_s:
                    if not cfg.finish_on_time:
                        self._fail_step(f"turn stopped: heading lost at {seen:.0f}/{degrees:.0f} deg")
                        break
                    # Finish open loop: both latencies cancel, so the turn takes degrees / omega.
                    fallback_end_s = float(degrees) / omega
                    logger.warning("Turn: heading lost at %.0f deg, finishing on time (%.2f s)", seen, fallback_end_s)
                if fallback_end_s is not None and elapsed >= fallback_end_s:
                    self._record_turn(action, degrees, seen, omega, lead, elapsed, "time", True, anchor_info)
                    break
                if timed_out:
                    self._fail_step(f"turn timed out: {seen:.0f}/{degrees:.0f} deg in {duration_s:.1f} s")
                    break
            elif action == "settle":
                target = self._center_angle
                sample = self._heading_sample() if self._heading_sample is not None else None
                self._measure_turn_error(sample, elapsed)
                aligned = False
                if sample is not None and sample.valid and sample.psi_deg is not None:
                    target, aligned = self._settle_target(sample)
                settle_ok_s = settle_ok_s + dt if aligned else 0.0
                angle = self._slew_servo(angle, target, dt)
                if cfg.settle_exit_hold_s > 0 and settle_ok_s >= cfg.settle_exit_hold_s:
                    break
                if timed_out:
                    break
            elif tiles is not None:
                # Add up increments instead of subtracting a start value: the
                # counter is reset to 0 when the route session opens, which
                # can happen just after this step has started.
                count = int(self._tile_count_getter())  # type: ignore[misc]
                if count > last_count:
                    done_tiles += count - last_count
                last_count = count
                with self._lock:
                    self._step_tiles_done = done_tiles
                if done_tiles >= target:
                    break
                if timed_out:
                    self._fail_step(f"tile step timed out: {done_tiles}/{target} tiles in {duration_s:.0f} s")
                    break
            elif timed_out:
                if action in ("left", "right", "turn_left", "turn_right"):
                    sign = 1.0 if action in ("left", "turn_left") else -1.0
                    self._turn_done(sign * duration_s * cfg.nominal_omega_dps)
                break

    # ------------------------------------------------------------------ #
    # Turn approach and turn error
    # ------------------------------------------------------------------ #
    def _anchor_lines(self, after_t: float) -> tuple[list[float], float, float | None, float | None] | None:
        """(line ys, counter_y, vp_row_px, frame_h) of a frame taken after
        after_t (waits up to 0.3 s); the last two are None when unknown."""
        deadline = time.monotonic() + 0.3
        while True:
            got = self._anchor_source() if self._anchor_source is not None else None
            if got is not None and got[0] > after_t:
                vp = got[3] if len(got) > 3 else None
                h = got[4] if len(got) > 4 else None
                return (
                    list(got[1]),
                    float(got[2]),
                    None if vp is None else float(vp),
                    None if not h else float(h),
                )
            if time.monotonic() > deadline or not self.is_running():
                return None
            time.sleep(0.02)

    @staticmethod
    def _follow(ys: list[float], last_y: float) -> float | None:
        """The line that continues the anchor: same seam, it only moves down."""
        near = [y for y in ys if -0.02 <= y - last_y <= 0.08]
        return min(near, key=lambda y: abs(y - last_y)) if near else None

    def _set_base(self, base_cmd: str) -> None:
        with self._lock:
            self._current_base_cmd = base_cmd
            self._current_angle = None
        if self._base_cb is not None:
            self._base_cb(base_cmd)

    def _hold(self, seconds: float) -> bool:
        """Sleep while running; paused time does not count. False if stopped."""
        left, last = seconds, time.monotonic()
        while left > 0:
            time.sleep(min(0.02, left))
            now = time.monotonic()
            with self._lock:
                if not self._running:
                    return False
                if not self._paused:
                    left -= now - last
            last = now
        return True

    def _anchor_target(self, sign: float, vp_px: float | None, frame_h: float | None) -> tuple[float, str]:
        """Row y* for V+1 and where it came from ("vp" or "manual").

        From the measured turn: the turn starts with V d_V = a - L/2 ahead of
        the front axle, so V+1 is d_V + L ahead, at row vp + k / (d_V + L - c)
        (vision/ground_range.py). Falls back to the hand-set y* when a or k
        is 0, the VP row is unknown, or the row lands outside the image."""
        cfg = self.turn
        manual = float(cfg.anchor_y_left if sign > 0 else cfg.anchor_y_right)
        a = float(cfg.a_left_cm if sign > 0 else cfg.a_right_cm)
        ground = GroundRange(cfg.ground_k_px_cm, cfg.ground_c_cm)
        if a <= 0 or not ground.enabled or vp_px is None or not frame_h:
            return manual, "manual"
        d_v = a - cfg.tile_cm / 2.0
        row = ground.row_px(d_v + cfg.tile_cm, vp_px)
        if row is None or not 0.05 <= row / frame_h <= 0.95:
            return manual, "manual"
        return row / frame_h, "vp"

    def _approach_anchor(self, sign: float) -> dict[str, Any] | None:
        """Drive to the turn point: the cross line above V (V+1) at row y*.

        Fast forward until the line is slow_margin before y*, stop, then nudge
        with short FORWARD pulses, measuring at standstill (no blur). The
        pulse length is a P controller: missing rows / rows per second, with
        rows per second learned from each pulse. Returns None after failing
        the step."""
        cfg = self.turn
        target, target_src = self._anchor_target(sign, None, None)
        if self._anchor_source is None:
            self._fail_step("turn anchor: no cross-line source")
            return None
        with self._lock:
            self._approaching = True
        try:
            t0 = time.monotonic()
            got = self._anchor_lines(t0 - 1.0)
            # Lock: the nearest line above the counter line (V just crossed it).
            y = None
            if got is not None:
                target, target_src = self._anchor_target(sign, got[2], got[3])
                above = [v for v in got[0] if v < got[1] - 0.02]
                y = max(above) if above else None
            self._set_base("FORWARD")
            last_seen = time.monotonic()
            while y is None or y < target - cfg.anchor_slow_margin:
                if not self._hold(0.03):
                    return None
                got = self._anchor_lines(time.monotonic() - 0.2)
                now = time.monotonic()
                if got is not None:
                    target, target_src = self._anchor_target(sign, got[2], got[3])
                    if y is None:
                        above = [v for v in got[0] if v < got[1] - 0.02]
                        y = max(above) if above else None
                        if y is not None:
                            last_seen = now
                    else:
                        nxt = self._follow(got[0], y)
                        if nxt is not None:
                            y, last_seen = nxt, now
                if now - last_seen > cfg.anchor_lost_s:
                    self._fail_step("turn anchor: next cross line lost")
                    return None
                if now - t0 > cfg.anchor_timeout_s:
                    self._fail_step("turn anchor: timed out before the turn point")
                    return None
            pulses, k, pulse_s = 0, None, cfg.anchor_pulse_s
            while True:
                self._set_base("STOP")
                if not self._hold(cfg.anchor_wait_s):
                    return None
                got = self._anchor_lines(time.monotonic() - 0.05)
                nxt = self._follow(got[0], y) if got is not None else None
                if nxt is None:
                    self._fail_step("turn anchor: next cross line lost at standstill")
                    return None
                # Standstill: the VP row is at its steadiest, re-read y*.
                target, target_src = self._anchor_target(sign, got[2], got[3])
                if pulses and nxt - y > 0.002:
                    rate = (nxt - y) / pulse_s
                    k = rate if k is None else 0.5 * k + 0.5 * rate
                y = nxt
                err = target - y
                if err <= cfg.anchor_tol:
                    break  # at y*, or past it (logged; the adaptation corrects y*)
                if pulses >= cfg.anchor_max_pulses:
                    self._fail_step(f"turn anchor: {pulses} pulses, line at {y:.3f} / {target:.3f}")
                    return None
                pulse_s = max(0.05, min(0.3, err / k)) if k else cfg.anchor_pulse_s
                self._set_base("FORWARD")
                if not self._hold(pulse_s):
                    return None
                pulses += 1
            # Image rows per tile near y*: spacing to the next line above.
            above = [v for v in got[0] if v < y - 0.02]
            tile_dy = y - max(above) if above else None
            info = {
                "y": round(y, 4),
                "target": round(target, 4),
                "target_src": target_src,
                "vp_row": None if got[2] is None else round(got[2], 1),
                "pulses": pulses,
                "rows_per_s": None if k is None else round(k, 4),
                "tile_dy": None if tile_dy is None else round(tile_dy, 4),
            }
            logger.info("Turn anchor reached: %s", info)
            return info
        finally:
            with self._lock:
                self._approaching = False

    def _measure_turn_error(self, sample: Any, elapsed: float) -> None:
        """First settle frames after a turn: heading and lane errors.

        e_psi > 0: turned too little; e_y_cm > 0: the new line runs past the
        middle of the tile row (turned late). Both signs flip with the side,
        since psi and the lane offset are measured left/right of the car."""
        with self._lock:
            entry = self._pending_turn
        if entry is None:
            return
        if elapsed > 1.0:
            with self._lock:
                self._pending_turn = None  # nothing usable early in the settle
            return
        if sample is None or not sample.valid:
            return
        sign = 1.0 if entry["side"] == "left" else -1.0
        psi_list = entry.setdefault("_psi", [])
        off_list = entry.setdefault("_off", [])
        if sample.psi_deg is not None and len(psi_list) < 3:
            psi_list.append(float(sample.psi_deg))
        if sample.lane_offset_ratio is not None and len(off_list) < 3:
            off_list.append(float(sample.lane_offset_ratio))
        if len(psi_list) >= 3 and len(off_list) >= 3:
            psi = sum(psi_list) / len(psi_list)
            off = sum(off_list) / len(off_list)
            with self._lock:
                entry["e_psi"] = round(-sign * psi, 2)
                entry["e_y_cm"] = round(-sign * off * self.turn.tile_cm, 1)
                entry.pop("_psi", None)
                entry.pop("_off", None)
                self._pending_turn = None

    def _turn_suggestion_locked(self) -> dict[str, float] | None:
        """New latency and anchor rows from this run's turns (call under the lock).

        latency -= alpha * e_psi / omega (turned too little: stop later)
        y* -= alpha * e_y * tile_dy / tile_cm (turned late: turn with V+1 higher)
        a += alpha * e_y instead when y* came from the VP and a (turned late:
        the turn runs longer than a says, so start it earlier)
        Steps are capped at 0.1 s, 0.03 rows and 10 cm per turn."""
        cfg = self.turn
        latency = cfg.latency_s
        rows = {"left": cfg.anchor_y_left, "right": cfg.anchor_y_right}
        lengths = {"left": cfg.a_left_cm, "right": cfg.a_right_cm}
        a_used: set[str] = set()
        used = 0
        for t in self._turn_log:
            if t.get("e_psi") is None or t.get("fallback_time"):
                continue
            used += 1
            omega = max(5.0, float(t["omega"] or cfg.nominal_omega_dps))
            latency -= max(-0.1, min(0.1, cfg.adapt_alpha * float(t["e_psi"]) / omega))
            if t.get("anchor_src") == "vp" and t.get("e_y_cm") is not None:
                lengths[t["side"]] += max(-10.0, min(10.0, cfg.adapt_alpha * float(t["e_y_cm"])))
                a_used.add(t["side"])
            elif t.get("anchor_y") is not None and t.get("tile_dy") and t.get("e_y_cm") is not None:
                d = cfg.adapt_alpha * float(t["e_y_cm"]) * float(t["tile_dy"]) / max(1.0, cfg.tile_cm)
                rows[t["side"]] -= max(-0.03, min(0.03, d))
        if not used:
            return None
        out = {
            "turn_latency_s": round(max(0.0, latency), 3),
            "turn_anchor_y_left": round(rows["left"], 4),
            "turn_anchor_y_right": round(rows["right"], 4),
        }
        for side in sorted(a_used):
            out[f"turn_a_{side}_cm"] = round(max(0.0, lengths[side]), 1)
        return out

    def turn_suggestion(self) -> dict[str, float] | None:
        with self._lock:
            return self._turn_suggestion_locked()

    def _settle_target(self, sample: Any) -> tuple[float, bool]:
        """Settle control law: (servo target, aligned) from a valid sample."""
        cfg = self.turn
        offset = float(sample.lane_offset_ratio) if sample.lane_offset_ratio is not None else 0.0
        # psi > 0: the car points left of the grid; offset > 0: lane
        # centre is to the right. Both need a right correction, and
        # right is below centre.
        u = cfg.settle_k_psi * float(sample.psi_deg) + cfg.settle_k_offset * offset
        target = self._center_angle - max(-cfg.settle_max_steer, min(cfg.settle_max_steer, u))
        aligned = (
            abs(float(sample.psi_deg)) <= cfg.settle_exit_psi_deg
            and sample.lane_offset_ratio is not None
            and abs(offset) <= cfg.settle_exit_offset
        )
        return target, aligned

    def _slew_servo(self, angle: float | None, target: float, dt: float) -> float | None:
        """Move the servo toward target at most settle_rate_dps * dt."""
        step_max = self.turn.settle_rate_dps * dt
        current = angle if angle is not None else self._center_angle
        new_angle = current + max(-step_max, min(step_max, target - current))
        if abs(new_angle - current) < 0.5:
            return angle
        with self._lock:
            self._current_angle = new_angle
        if self._servo_cb is not None:
            self._servo_cb(new_angle)
        return new_angle

    def _cross_lane(self, side: int, steer_deg: float, timeout_s: float, base_cmd: str) -> float | None:
        """Steer toward `side` (+1 left) until lane_crossed() or timeout.

        Returns the time it took, None on timeout or stop. Paused time does
        not count."""
        angle = max(0.0, min(180.0, self._center_angle + side * abs(float(steer_deg))))
        with self._lock:
            self._cross_side = side
            self._current_angle = angle
        self._publish_current_command(base_cmd, angle)
        waited, last = 0.0, time.monotonic()
        ok = False
        try:
            while True:
                if self._crossed.wait(0.05):
                    ok = True
                    break
                now = time.monotonic()
                with self._lock:
                    if not self._running:
                        break
                    if not self._paused:
                        waited += now - last
                last = now
                if waited >= timeout_s:
                    break
        finally:
            with self._lock:
                self._cross_side = 0
            self._crossed.clear()
        return waited if ok else None

    def _align_in_lane(self, timeout_s: float) -> None:
        """After a crossing the car still points at the old lane: straighten
        it on the new lane with the settle law until aligned (settle exit
        hold) or timeout. Skipped without a heading source."""
        cfg = self.turn
        angle: float | None = self._center_angle
        if self._heading_sample is None or timeout_s <= 0:
            if self._servo_cb is not None:
                self._servo_cb(self._center_angle)
            return
        elapsed, ok_s, last = 0.0, 0.0, time.monotonic()
        while elapsed < timeout_s:
            time.sleep(0.05)
            now = time.monotonic()
            with self._lock:
                if not self._running:
                    return
                paused = self._paused
            dt = 0.0 if paused else now - last
            last = now
            if paused:
                continue
            elapsed += dt
            sample = self._heading_sample()
            target, aligned = self._center_angle, False
            if sample is not None and sample.valid and sample.psi_deg is not None:
                target, aligned = self._settle_target(sample)
            ok_s = ok_s + dt if aligned else 0.0
            angle = self._slew_servo(angle, target, dt)
            if cfg.settle_exit_hold_s > 0 and ok_s >= cfg.settle_exit_hold_s:
                break
        if self._servo_cb is not None:
            self._servo_cb(self._center_angle)  # the vision PID takes over from centre

    def _lane_change(self, side: int, timeout_s: float) -> None:
        """Active lane change: steer into the next tile row on `side`, wait
        for the lane shift detector to see the crossing, then straighten."""
        cfg = self.turn
        logger.info("Lane change %s", "left" if side > 0 else "right")
        with self._lock:
            self._step_elapsed_s = 0.0
            self._step_duration_s = timeout_s
            self._current_base_cmd = "FORWARD"
        took = self._cross_lane(side, cfg.lane_change_steer_deg, timeout_s, "FORWARD")
        if took is None:
            if self.is_running():
                self._fail_step(f"lane change {'left' if side > 0 else 'right'}: no crossing in {timeout_s:.1f} s")
            return
        logger.info("Lane change crossed in %.1f s, aligning", took)
        self._align_in_lane(cfg.lane_align_timeout_s)
        with self._lock:
            self._current_angle = None

    def _recover_lane(self, side: int, base_cmd: str) -> bool:
        """Passive mode: steer back toward the lane the car drifted out of
        (side +1 = it drifted left, so steer right) until it crosses back,
        then straighten like an active lane change."""
        cfg = self.turn
        with self._lock:
            self._recovering = True
        logger.warning("Lane recovery: drifted %s, steering back", "left" if side > 0 else "right")
        try:
            took = self._cross_lane(-side, cfg.lane_recover_steer_deg, cfg.lane_recover_timeout_s, base_cmd)
            if took is not None:
                logger.info("Lane recovery crossed back in %.1f s", took)
                self._align_in_lane(cfg.lane_align_timeout_s)
        finally:
            with self._lock:
                self._recovering = False
                self._recover_side = 0
                self._current_angle = None
        if took is not None:
            return True
        if self.is_running():
            self._fail_step(f"lane recovery failed after {cfg.lane_recover_timeout_s:.1f} s")
        return False

    def _turn_done(self, rotation_deg: float) -> None:
        if self._turn_done_cb is None:
            return
        try:
            self._turn_done_cb(rotation_deg)
        except Exception:
            logger.exception("turn-done callback failed")

    def _fail_step(self, msg: str) -> None:
        """Stop the script; the runner's finally block sends STOP."""
        logger.warning("Route script stopped: %s", msg)
        with self._lock:
            self._last_error = msg
            self._running = False

    def _record_turn(
        self,
        action: str,
        degrees: float,
        seen: float,
        omega: float,
        lead: float,
        elapsed: float,
        source: str,
        fallback: bool,
        anchor: dict[str, Any] | None = None,
    ) -> None:
        turn = {
            "action": action,
            "degrees": float(degrees),
            "seen_at_stop_deg": round(seen, 1),
            "omega_dps": round(omega, 1),
            "lead_deg": round(lead, 1),
            "elapsed_s": round(elapsed, 2),
            "source": source,
            "fallback_time": fallback,
        }
        if anchor is not None:
            turn["anchor"] = anchor
        logger.info("Turn done: %s", turn)
        entry = {
            "side": "left" if action in ("left", "turn_left") else "right",
            "degrees": float(degrees),
            "seen_at_stop": turn["seen_at_stop_deg"],
            "omega": turn["omega_dps"],
            "fallback_time": fallback,
            "latency_used": self.turn.latency_s,
            "anchor_y": None if anchor is None else anchor.get("y"),
            "anchor_target": None if anchor is None else anchor.get("target"),
            "anchor_src": None if anchor is None else anchor.get("target_src"),
            "tile_dy": None if anchor is None else anchor.get("tile_dy"),
            "e_psi": None,
            "e_y_cm": None,
        }
        with self._lock:
            self._last_turn = turn
            self._turn_log.append(entry)
            self._pending_turn = entry
        self._turn_done((1.0 if action in ("left", "turn_left") else -1.0) * float(degrees))

    def _publish_current_command(self, base_cmd: str, angle: float | None) -> None:
        if self._base_cb is not None:
            self._base_cb(base_cmd)
        if angle is not None and self._servo_cb is not None:
            self._servo_cb(angle)
