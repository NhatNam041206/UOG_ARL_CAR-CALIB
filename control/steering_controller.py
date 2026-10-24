"""Control module: V2 vanishing-point steering state machine.

Mirrors the SteeringController used by the offline UnifiedCalibrator so the
live loop and the offline processor share one calibration law. States:

* ``GAPPING``        — vision lost (no VP / no intercepts), hold center
* ``DANGER_LEFT``    — car drifted toward the right line, fixed nudge left
  (angle above center)
* ``DANGER_RIGHT``   — car drifted toward the left line, fixed nudge right
  (angle below center)
* ``TRACKING_COAST`` — error inside hysteresis dead-band, hold center
* ``TRACKING_PD``    — error past outer threshold, PD correction active

Danger is decided by the lane-centre offset as a fraction of lane width when
``danger_offset_ratio`` > 0, so it holds for any camera height or lane width
in pixels. With ``danger_offset_ratio`` = 0 the legacy rule applies: each
bottom intercept is compared with a fixed pixel band at the frame edge.

Angles are emitted in the configured servo convention. The VP error stays
90-centered, then output maps onto ``SERVO_CENTER_ANGLE`` ± max offset so
MiniPC MQTT commands match the RPi signed-physical angle convention.
"""

from __future__ import annotations

from models.robot_state import PIDConstants


class SteeringController:
    """State-machine governor implementing vision-lost, danger, and tracking stages."""

    def __init__(
        self,
        pid_constants: PIDConstants,
        danger_margin: int,
        nudge_deg: float,
        inner_thresh: float,
        outer_thresh: float,
        center_angle: float = 90.0,
        max_offset: float = 90.0,
        danger_offset_ratio: float = 0.0,
        vp_center_deg: float = 90.0,
    ) -> None:
        self._pid = pid_constants
        self._danger_margin = max(0, int(danger_margin))
        self._nudge_deg = float(nudge_deg)
        self._danger_offset_ratio = max(0.0, float(danger_offset_ratio))
        # VP angle seen when the car points straight down the lane; differs
        # from 90 when the camera is mounted with a yaw.
        self._vp_center_deg = float(vp_center_deg)
        self._inner_thresh = abs(float(inner_thresh))
        self._outer_thresh = max(abs(float(outer_thresh)), self._inner_thresh)
        self._center = float(center_angle)
        self._max_offset = abs(float(max_offset))
        self._tracking_active = False
        self._last_error = 0.0
        # What produced the last command: PD, COAST, NUDGE, DANGER_PD or HOLD.
        self.last_command_source = "HOLD"

    def compute_steering(
        self,
        vp_angle: float | None,
        left_intercept: int | None,
        right_intercept: int | None,
        frame_width: int,
    ) -> tuple[float, str]:
        """Return steering angle and active state according to 3-stage logic."""
        center = self._center
        lo = center - self._max_offset
        hi = center + self._max_offset

        # Stage 1: Vision Lost (Gapping)
        if vp_angle is None or left_intercept is None or right_intercept is None:
            self._tracking_active = False
            self._last_error = 0.0
            self.last_command_source = "HOLD"
            return center, "GAPPING"

        # Stage 3: Danger Zone override (bypass PD)
        danger = self._danger_state(left_intercept, right_intercept, frame_width)
        self.last_command_source = "NUDGE"
        if danger is not None and self._danger_offset_ratio > 0.0:
            return self._danger_command(danger, float(vp_angle), lo, hi), danger
        if danger == "DANGER_LEFT":
            self._tracking_active = False
            self._last_error = 0.0
            return center + self._nudge_deg, "DANGER_LEFT"
        if danger == "DANGER_RIGHT":
            self._tracking_active = False
            self._last_error = 0.0
            return center - self._nudge_deg, "DANGER_RIGHT"

        # Stage 2: Tracking with hysteresis
        error = float(vp_angle) - self._vp_center_deg
        abs_error = abs(error)
        if abs_error <= self._inner_thresh:
            self._tracking_active = False
            self._last_error = 0.0
            self.last_command_source = "COAST"
            return center, "TRACKING_COAST"

        if abs_error > self._outer_thresh:
            self._tracking_active = True

        if not self._tracking_active:
            self.last_command_source = "COAST"
            return center, "TRACKING_COAST"

        self.last_command_source = "PD"

        pd_correction = self._apply_pd(error)
        steering_angle = max(lo, min(hi, center - pd_correction))
        return steering_angle, "TRACKING_PD"

    def _danger_command(self, danger: str, vp_angle: float, lo: float, hi: float) -> float:
        """Combine the danger nudge with the heading PD (lane-offset rule only).

        The fixed nudge alone is weaker than PD when the car is also yawed
        toward the near line, and it fights PD when the car already points
        back toward the lane centre. So: if PD steers the same way harder,
        use PD; if PD steers the other way with a heading error past the
        outer threshold (car already heading back), use PD; else nudge.
        """
        center = self._center
        nudge_off = self._nudge_deg if danger == "DANGER_LEFT" else -self._nudge_deg
        error = vp_angle - self._vp_center_deg
        pd_off = max(lo, min(hi, center - self._apply_pd(error))) - center
        self._tracking_active = False
        same_way = pd_off * nudge_off > 0
        if (same_way and abs(pd_off) > abs(nudge_off)) or (
            pd_off * nudge_off < 0 and abs(error) > self._outer_thresh
        ):
            self.last_command_source = "DANGER_PD"
            return center + pd_off
        self.last_command_source = "NUDGE"
        return center + nudge_off

    def _danger_state(
        self,
        left_intercept: int,
        right_intercept: int,
        frame_width: int,
    ) -> str | None:
        """Return the danger state for this intercept pair, or None."""
        if self._danger_offset_ratio > 0.0:
            offset = lane_center_offset_ratio(left_intercept, right_intercept, frame_width)
            if offset is None or abs(offset) <= self._danger_offset_ratio:
                return None
            # Lane centre left of the image centre means the car sits right
            # of the lane centre, so recovery is a nudge to the left.
            return "DANGER_LEFT" if offset < 0 else "DANGER_RIGHT"

        if left_intercept < self._danger_margin:
            return "DANGER_LEFT"
        if right_intercept > max(0, int(frame_width) - self._danger_margin):
            return "DANGER_RIGHT"
        return None

    def _apply_pd(self, error: float) -> float:
        """Apply proportional-derivative smoothing and return steering correction."""
        derivative = error - self._last_error
        self._last_error = error
        return (self._pid.kp * error) + (self._pid.kd * derivative)

    def describe_control_state(self, state: str, frame_width: int) -> dict[str, Any]:
        """Return per-frame control metadata for telemetry/debug."""
        margin = self._danger_margin
        if self._danger_offset_ratio > 0.0:
            ratio = self._danger_offset_ratio
            return {
                "danger_boundary": f"|lane offset| > {ratio:.2f} lane width",
                "recovery_direction": "LEFT" if state == "DANGER_LEFT" else ("RIGHT" if state == "DANGER_RIGHT" else "NONE"),
                "danger_threshold_x": f"{ratio:.2f}",
            }
        return {
            "danger_boundary": f"LEFT < {margin}px | RIGHT > {frame_width - margin}px",
            "recovery_direction": "LEFT" if state == "DANGER_LEFT" else ("RIGHT" if state == "DANGER_RIGHT" else "NONE"),
            "danger_threshold_x": str(margin),
        }


def lane_center_offset_ratio(
    left_intercept: int | None,
    right_intercept: int | None,
    frame_width: int,
) -> float | None:
    """Return (lane centre - image centre) / lane width, or None if undefined."""
    if left_intercept is None or right_intercept is None:
        return None
    lane_width = float(right_intercept) - float(left_intercept)
    if lane_width <= 0:
        return None
    lane_center = (float(left_intercept) + float(right_intercept)) / 2.0
    return (lane_center - float(frame_width) / 2.0) / lane_width
