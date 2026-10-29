"""Route script runner for Pi5 (no direct hardware)."""

from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from config.settings import (
    SETTLE_EXIT_HOLD_S,
    SETTLE_EXIT_OFFSET,
    SETTLE_EXIT_PSI_DEG,
    SETTLE_K_OFFSET,
    SETTLE_K_PSI,
    SETTLE_MAX_STEER,
    SETTLE_RATE_DPS,
    TILE_STEP_MAX_S_PER_TILE,
    TURN_HEADING_LOST_S,
    TURN_LEFT_STEER_DEG,
    TURN_LATENCY_S,
    TURN_NOMINAL_OMEGA_DPS,
    TURN_ON_HEADING_LOST,
    TURN_RIGHT_STEER_DEG,
    TURN_TIMEOUT_FACTOR,
)
from runtime.resource_limits import turn_timeout_s, validate_route_steps

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
}
# Steps that pin the servo themselves, so the vision PID must not publish.
_SERVO_PINNED_ACTIONS = ("left", "right", "backward", "settle")


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

    def needs_heading(self) -> bool:
        """True while the current step reads the heading (main loop runs the estimator)."""
        with self._lock:
            step = self._current_step if self._running else None
        return step is not None and (step.get("degrees") is not None or step.get("action") == "settle")

    def current_action(self) -> str | None:
        with self._lock:
            if not self._running or self._current_step is None:
                return None
            return str(self._current_step.get("action", "stop"))

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def vision_pid_active(self) -> bool:
        """True when current step should let vision PID control servo."""
        with self._lock:
            if self._paused:
                return False
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
                    self._record_turn(action, degrees, seen, omega, lead, elapsed, sample.source, False)
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
                    self._record_turn(action, degrees, seen, omega, lead, elapsed, "time", True)
                    break
                if timed_out:
                    self._fail_step(f"turn timed out: {seen:.0f}/{degrees:.0f} deg in {duration_s:.1f} s")
                    break
            elif action == "settle":
                target = self._center_angle
                sample = self._heading_sample() if self._heading_sample is not None else None
                if sample is not None and sample.valid and sample.psi_deg is not None:
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
                    settle_ok_s = settle_ok_s + dt if aligned else 0.0
                else:
                    settle_ok_s = 0.0
                step_max = cfg.settle_rate_dps * dt
                current = angle if angle is not None else self._center_angle
                new_angle = current + max(-step_max, min(step_max, target - current))
                if abs(new_angle - current) >= 0.5:
                    angle = new_angle
                    with self._lock:
                        self._current_angle = angle
                    if self._servo_cb is not None:
                        self._servo_cb(angle)
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
                break

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
        logger.info("Turn done: %s", turn)
        with self._lock:
            self._last_turn = turn

    def _publish_current_command(self, base_cmd: str, angle: float | None) -> None:
        if self._base_cb is not None:
            self._base_cb(base_cmd)
        if angle is not None and self._servo_cb is not None:
            self._servo_cb(angle)
