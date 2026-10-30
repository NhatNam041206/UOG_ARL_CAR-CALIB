"""Runtime tuning for live calibration parameters."""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from vision.cross_line_counter import CrossLineConfig

logger = logging.getLogger(__name__)

IdleGetter = Callable[[], tuple[bool, str]]


class TuneValidationError(ValueError):
    """Raised when a tune payload cannot be applied."""


class TuneBlockedError(RuntimeError):
    """Raised when live apply is blocked by vehicle state."""


@dataclass(frozen=True)
class TuneParam:
    key: str
    group: str
    label: str
    value_type: str
    minimum: float
    maximum: float
    step: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "group": self.group,
            "label": self.label,
            "type": self.value_type,
            "min": self.minimum,
            "max": self.maximum,
            "step": self.step,
        }


PARAMS: tuple[TuneParam, ...] = (
    TuneParam("roi_height_pct", "Vision", "ROI height", "float", 0.1, 1.0, 0.01),
    TuneParam("roi_top_width_pct", "Vision", "ROI top width", "float", 0.05, 1.0, 0.01),
    TuneParam("roi_bottom_width_pct", "Vision", "ROI bottom width", "float", 0.05, 1.0, 0.01),
    TuneParam("canny_low", "Vision", "Canny low", "int", 0, 500, 1),
    TuneParam("canny_high", "Vision", "Canny high", "int", 0, 500, 1),
    TuneParam("hough_threshold", "Vision", "Hough threshold", "int", 1, 300, 1),
    TuneParam("hough_min_line_length", "Vision", "Hough min line", "int", 1, 500, 1),
    TuneParam("hough_max_line_gap", "Vision", "Hough max gap", "int", 0, 200, 1),
    TuneParam("min_abs_slope", "Vision", "Min abs slope", "float", 0.0, 5.0, 0.01),
    TuneParam("vp_horizon_y", "Vision", "VP horizon row", "int", -5000, 0, 1),
    TuneParam("vp_search_px", "Vision", "VP row search", "int", 0, 1000, 1),
    TuneParam("pair_hold_frames", "Vision", "Lane pair hold (frames)", "int", 0, 30, 1),
    TuneParam("camera_hfov_deg", "Vision", "Camera HFOV (0 = proxy angle)", "float", 0.0, 170.0, 0.5),
    TuneParam("lane_width_tol", "Vision", "Lane width tolerance (0 = off)", "float", 0.0, 1.0, 0.01),
    TuneParam("single_line_max_frames", "Vision", "Single-line tracking (frames, 0 = off)", "int", 0, 300, 1),
    TuneParam("relock_valid_frames", "Vision", "Relock frames after GAPPING", "int", 0, 30, 1),
    TuneParam("pid_kp", "Steering", "PID Kp", "float", 0.0, 10.0, 0.01),
    TuneParam("pid_kd", "Steering", "PID Kd", "float", 0.0, 10.0, 0.01),
    TuneParam("pid_lateral_gain", "Steering", "PID lateral gain", "float", 0.0, 300.0, 1.0),
    TuneParam("vp_inner_thresh", "Steering", "VP inner", "float", 0.0, 45.0, 0.1),
    TuneParam("vp_outer_thresh", "Steering", "VP outer", "float", 0.0, 45.0, 0.1),
    TuneParam("vp_center_deg", "Steering", "VP straight angle", "float", 45.0, 135.0, 0.1),
    TuneParam("danger_margin_px", "Steering", "Danger margin", "int", 0, 640, 1),
    TuneParam("danger_offset_ratio", "Steering", "Danger lane offset", "float", 0.0, 0.5, 0.01),
    TuneParam("danger_nudge_deg", "Steering", "Danger nudge", "float", 0.0, 45.0, 0.1),
    TuneParam("servo_trim_deg", "Servo", "Straight trim (+ = left)", "float", -30.0, 30.0, 0.5),
    TuneParam("servo_left_gain", "Servo", "Left gain", "float", 0.2, 3.0, 0.05),
    TuneParam("servo_right_gain", "Servo", "Right gain", "float", 0.2, 3.0, 0.05),
    TuneParam("servo_left_kick_deg", "Servo", "Left kick (deg)", "float", 0.0, 30.0, 0.5),
    TuneParam("servo_right_kick_deg", "Servo", "Right kick (deg)", "float", 0.0, 30.0, 0.5),
    TuneParam("cross_enabled", "Cross lines", "Tile counter on (1/0)", "int", 0, 1, 1),
    TuneParam("cross_roi_top", "Cross lines", "Look-ahead ROI top", "float", 0.0, 1.0, 0.01),
    TuneParam("cross_roi_bottom", "Cross lines", "Look-ahead ROI bottom", "float", 0.0, 1.0, 0.01),
    TuneParam("cross_counter_y", "Cross lines", "Counter line (y)", "float", 0.0, 1.0, 0.01),
    TuneParam("cross_counter_h", "Cross lines", "Counter band height", "float", 0.0, 0.5, 0.01),
    TuneParam("cross_dark_thresh", "Cross lines", "Dark-line threshold", "int", 1, 100, 1),
    TuneParam("cross_blackhat_kernel_px", "Cross lines", "Dark-line kernel (px)", "int", 3, 61, 2),
    TuneParam("cross_min_len", "Cross lines", "Min line length (x width)", "float", 0.05, 1.0, 0.01),
    TuneParam("cross_max_tilt_deg", "Cross lines", "Max tilt (deg)", "float", 1.0, 80.0, 0.5),
    TuneParam("cross_slope_tol_deg", "Cross lines", "Tilt tolerance (deg)", "float", 0.5, 45.0, 0.5),
    TuneParam("cross_track_max_dy", "Cross lines", "Track max jump (y)", "float", 0.01, 0.5, 0.01),
    TuneParam("cross_confirm_frames", "Cross lines", "Confirm frames", "int", 1, 30, 1),
    TuneParam("cross_max_miss_frames", "Cross lines", "Coast frames", "int", 0, 60, 1),
    TuneParam("cross_count_min_gap_frames", "Cross lines", "Min frames between counts", "int", 0, 120, 1),
    TuneParam("cross_count_min_gap_ratio", "Cross lines", "Min gap (x median interval, 0 = off)", "float", 0.0, 1.0, 0.05),
)
# Tune key -> CrossLineConfig field on calibrator.cross_counter.config.
_CROSS_FIELDS = {
    param.key: param.key[len("cross_"):]
    for param in PARAMS
    if param.key.startswith("cross_")
}
_SERVO_ATTRS = {
    "servo_trim_deg": ("_servo_trim_deg", 0.0),
    "servo_left_gain": ("_servo_left_gain", 1.0),
    "servo_right_gain": ("_servo_right_gain", 1.0),
    "servo_left_kick_deg": ("_servo_left_kick_deg", 5.0),
    "servo_right_kick_deg": ("_servo_right_kick_deg", 10.0),
}
_PARAMS_BY_KEY = {param.key: param for param in PARAMS}


@dataclass(frozen=True)
class TuneBinding:
    """A tune param that lives outside the calibrator (script runner, grid heading)."""

    param: TuneParam
    getter: Callable[[], float]
    setter: Callable[[float], None]


def attr_binding(param: TuneParam, owner: Any, attr: str) -> TuneBinding:
    """Bind a param to owner.attr, keeping the attribute's type (bool/int/float)."""

    def get() -> float:
        return float(getattr(owner, attr))

    def put(value: float) -> None:
        current = getattr(owner, attr)
        if isinstance(current, bool):
            setattr(owner, attr, bool(value))
        elif isinstance(current, int):
            setattr(owner, attr, int(value))
        else:
            setattr(owner, attr, float(value))

    return TuneBinding(param, get, put)


# Degree turns + settle (runtime.pi5_script_runner.TurnSettings) and the grid
# heading estimator (vision.grid_heading.GridHeadingConfig).
TURN_PARAMS: tuple[tuple[TuneParam, str], ...] = (
    (TuneParam("turn_left_steer_deg", "Turning", "Left step: servo offset from centre (deg)", "float", 0.0, 90.0, 0.5), "left_steer_deg"),
    (TuneParam("turn_right_steer_deg", "Turning", "Right step: servo offset from centre (deg)", "float", 0.0, 90.0, 0.5), "right_steer_deg"),
    (TuneParam("turn_latency_s", "Turning", "Command-to-camera latency (s)", "float", 0.0, 2.0, 0.01), "latency_s"),
    (TuneParam("turn_nominal_omega_dps", "Turning", "Nominal yaw rate (deg/s)", "float", 5.0, 120.0, 1.0), "nominal_omega_dps"),
    (TuneParam("turn_timeout_factor", "Turning", "Timeout (x nominal turn time)", "float", 1.0, 5.0, 0.1), "timeout_factor"),
    (TuneParam("turn_heading_lost_s", "Turning", "Heading lost after (s)", "float", 0.1, 3.0, 0.05), "heading_lost_s"),
    (TuneParam("turn_finish_on_time", "Turning", "Heading lost: finish on time (1) / stop (0)", "int", 0, 1, 1), "finish_on_time"),
    (TuneParam("turn_anchor_y_left", "Turn approach", "Left: V+1 row y* at turn start (0 top, 1 bottom)", "float", 0.05, 0.95, 0.005), "anchor_y_left"),
    (TuneParam("turn_anchor_y_right", "Turn approach", "Right: V+1 row y* at turn start (0 top, 1 bottom)", "float", 0.05, 0.95, 0.005), "anchor_y_right"),
    (TuneParam("turn_anchor_tol", "Turn approach", "Row tolerance at y*", "float", 0.002, 0.1, 0.001), "anchor_tol"),
    (TuneParam("turn_anchor_slow_margin", "Turn approach", "Stop this far before y* (rows)", "float", 0.0, 0.3, 0.005), "anchor_slow_margin"),
    (TuneParam("turn_anchor_pulse_s", "Turn approach", "First nudge pulse (s)", "float", 0.05, 0.5, 0.01), "anchor_pulse_s"),
    (TuneParam("turn_anchor_wait_s", "Turn approach", "Standstill before measuring (s)", "float", 0.1, 1.5, 0.05), "anchor_wait_s"),
    (TuneParam("turn_anchor_max_pulses", "Turn approach", "Max nudges", "int", 1, 30, 1), "anchor_max_pulses"),
    (TuneParam("turn_anchor_lost_s", "Turn approach", "Anchor line lost after (s)", "float", 0.1, 3.0, 0.05), "anchor_lost_s"),
    (TuneParam("turn_tile_cm", "Turn approach", "Tile size L (cm, for e_y)", "float", 10.0, 200.0, 0.5), "tile_cm"),
    (TuneParam("turn_adapt_alpha", "Turn approach", "Suggestion gain alpha", "float", 0.0, 1.0, 0.05), "adapt_alpha"),
    (TuneParam("settle_k_psi", "Settle", "Servo deg per heading deg", "float", 0.0, 5.0, 0.05), "settle_k_psi"),
    (TuneParam("settle_k_offset", "Settle", "Servo deg per lane offset", "float", 0.0, 100.0, 1.0), "settle_k_offset"),
    (TuneParam("settle_max_steer", "Settle", "Max steer from centre (deg)", "float", 0.0, 80.0, 0.5), "settle_max_steer"),
    (TuneParam("settle_rate_dps", "Settle", "Servo slew limit (deg/s)", "float", 5.0, 300.0, 5.0), "settle_rate_dps"),
    (TuneParam("settle_exit_psi_deg", "Settle", "Exit: heading within (deg)", "float", 0.0, 15.0, 0.5), "settle_exit_psi_deg"),
    (TuneParam("settle_exit_offset", "Settle", "Exit: lane offset within", "float", 0.0, 0.5, 0.01), "settle_exit_offset"),
    (TuneParam("lane_recover", "Lane shift", "Steer back after a passive shift (1) / no (0)", "int", 0, 1, 1), "lane_recover"),
    (TuneParam("lane_recover_steer_deg", "Lane shift", "Recovery steer from centre (deg)", "float", 0.0, 60.0, 0.5), "lane_recover_steer_deg"),
    (TuneParam("lane_recover_timeout_s", "Lane shift", "Recovery timeout (s)", "float", 0.5, 10.0, 0.1), "lane_recover_timeout_s"),
    (TuneParam("settle_exit_hold_s", "Settle", "Exit: hold (s, 0 = no early exit)", "float", 0.0, 2.0, 0.05), "settle_exit_hold_s"),
)
GRID_PARAMS: tuple[tuple[TuneParam, str], ...] = (
    (TuneParam("grid_g_px", "Grid heading", "g until learned (px)", "float", 100.0, 3000.0, 1.0), "g_px"),
    (TuneParam("grid_scale", "Grid heading", "Frame scale for line search", "float", 0.25, 1.0, 0.05), "scale"),
    (TuneParam("grid_dark_thresh", "Grid heading", "Dark-line threshold", "int", 1, 100, 1), "dark_thresh"),
    (TuneParam("grid_hough_votes", "Grid heading", "Hough votes (full-frame px)", "int", 10, 300, 1), "hough_votes"),
    (TuneParam("grid_min_line_px", "Grid heading", "Min line length (px)", "int", 10, 400, 1), "min_line_px"),
    (TuneParam("grid_inlier_tol_deg", "Grid heading", "VP inlier tolerance (deg)", "float", 0.5, 10.0, 0.1), "inlier_tol_deg"),
    (TuneParam("grid_horizon_tol_px", "Grid heading", "VP horizon tolerance (px)", "float", 10.0, 400.0, 1.0), "horizon_tol_px"),
    (TuneParam("grid_max_vp_y", "Grid heading", "VP must be above row (px, <0 = above image)", "float", -1000.0, 0.0, 5.0), "max_vp_y"),
    (TuneParam("grid_coast_max_s", "Grid heading", "Coast without heading (s)", "float", 0.0, 2.0, 0.05), "coast_max_s"),
)


def turning_bindings(turn_settings: Any, grid_config: Any) -> list[TuneBinding]:
    """Tune bindings for the Turning, Settle and Grid heading groups."""
    return [attr_binding(param, turn_settings, attr) for param, attr in TURN_PARAMS] + [
        attr_binding(param, grid_config, attr) for param, attr in GRID_PARAMS
    ]


class CalibTuneManager:
    """Applies validated tune values to the live calibrator instance."""

    def __init__(
        self,
        calibrator: Any,
        tune_file: str | Path,
        *,
        idle_getter: IdleGetter | None = None,
        extras: list[TuneBinding] | None = None,
    ) -> None:
        self._calibrator = calibrator
        self._extras = list(extras or [])
        self._params: tuple[TuneParam, ...] = PARAMS + tuple(b.param for b in self._extras)
        self._params_by_key = {param.key: param for param in self._params}
        if len(self._params_by_key) != len(self._params):
            raise ValueError("duplicate tune param keys")
        self._tune_file = Path(tune_file)
        self._idle_getter = idle_getter or (lambda: (True, "idle"))
        self._defaults = self._read_current()
        self._saved_values: dict[str, int | float] | None = None
        self._saved_updated_utc: str | None = None
        self.load_saved()

    @property
    def tune_file(self) -> Path:
        return self._tune_file

    def status(self) -> dict[str, Any]:
        values = self._read_current()
        idle_ok, idle_reason = self._idle_state()
        baseline = self._saved_values if self._saved_values is not None else self._defaults
        return {
            "values": values,
            "defaults": dict(self._defaults),
            "saved": {
                "exists": self._saved_values is not None,
                "updated_utc": self._saved_updated_utc,
                "values": dict(self._saved_values or {}),
                "path": str(self._tune_file),
            },
            "dirty": not _same_values(values, baseline, self._params),
            "schema": [param.as_dict() for param in self._params],
            "idle": {"ok": idle_ok, "reason": idle_reason},
        }

    def apply(self, values: dict[str, Any], *, require_idle: bool = True) -> dict[str, Any]:
        if require_idle:
            idle_ok, reason = self._idle_state()
            if not idle_ok:
                raise TuneBlockedError(reason)
        validated = self._validate(values, base=self._read_current())
        self._apply_values(validated)
        return self.status()

    def save(self) -> dict[str, Any]:
        values = self._read_current()
        updated_utc = _utc_now()
        payload = {
            "version": 1,
            "updated_utc": updated_utc,
            "values": values,
        }
        self._tune_file.parent.mkdir(parents=True, exist_ok=True)
        self._tune_file.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self._saved_values = dict(values)
        self._saved_updated_utc = updated_utc
        return self.status()

    def reset(self, target: str) -> dict[str, Any]:
        if target == "defaults":
            return self.apply(self._defaults)
        if target == "saved":
            if self._saved_values is None:
                raise TuneValidationError("no saved tune values")
            return self.apply(self._saved_values)
        raise TuneValidationError("target must be 'saved' or 'defaults'")

    def load_saved(self) -> bool:
        if not self._tune_file.is_file():
            return False
        try:
            payload = json.loads(self._tune_file.read_text(encoding="utf-8"))
            if payload.get("version") != 1:
                raise TuneValidationError("unsupported tune file version")
            values = payload.get("values")
            if not isinstance(values, dict):
                raise TuneValidationError("tune file values must be an object")
            # Params added after the file was saved keep their current value.
            validated = self._validate(values, base=self._read_current())
        except Exception as exc:  # noqa: BLE001
            logger.warning("Ignoring invalid calib tune file %s: %s", self._tune_file, exc)
            return False
        self._saved_values = dict(validated)
        self._saved_updated_utc = str(payload.get("updated_utc") or "")
        self._apply_values(validated)
        return True

    def _idle_state(self) -> tuple[bool, str]:
        try:
            ok, reason = self._idle_getter()
        except Exception as exc:  # noqa: BLE001
            return False, f"idle check failed: {exc}"
        return bool(ok), str(reason or ("idle" if ok else "blocked"))

    def _validate(
        self,
        values: dict[str, Any],
        *,
        base: dict[str, int | float] | None,
    ) -> dict[str, int | float]:
        if not isinstance(values, dict):
            raise TuneValidationError("values must be an object")
        for key in values:
            if key not in self._params_by_key:
                raise TuneValidationError(f"unknown tune param: {key}")

        merged: dict[str, int | float] = dict(base or {})
        if base is None:
            missing = [param.key for param in self._params if param.key not in values]
            if missing:
                raise TuneValidationError(f"missing tune params: {', '.join(missing)}")

        for key, raw_value in values.items():
            merged[key] = _coerce_value(self._params_by_key[key], raw_value)

        for param in self._params:
            if param.key not in merged:
                raise TuneValidationError(f"missing tune param: {param.key}")

        if float(merged["canny_high"]) < float(merged["canny_low"]):
            raise TuneValidationError("canny_high must be >= canny_low")
        if float(merged["vp_outer_thresh"]) < float(merged["vp_inner_thresh"]):
            raise TuneValidationError("vp_outer_thresh must be >= vp_inner_thresh")
        if float(merged["cross_roi_bottom"]) <= float(merged["cross_roi_top"]):
            raise TuneValidationError("cross_roi_bottom must be > cross_roi_top")
        return merged

    def _read_current(self) -> dict[str, int | float]:
        vision = self._calibrator._vision
        steering = self._calibrator.steering_controller
        state = self._calibrator.robot_state
        return {
            "roi_height_pct": float(vision._roi_height_pct),
            "roi_top_width_pct": float(vision._roi_top_width_pct),
            "roi_bottom_width_pct": float(vision._roi_bottom_width_pct),
            "canny_low": int(vision._canny_low),
            "canny_high": int(vision._canny_high),
            "hough_threshold": int(vision._hough_threshold),
            "hough_min_line_length": int(vision._hough_min_line_length),
            "hough_max_line_gap": int(vision._hough_max_line_gap),
            "min_abs_slope": float(vision._min_abs_slope),
            "vp_horizon_y": int(vision._vp_horizon_y),
            "vp_search_px": int(vision._vp_search_px),
            "pair_hold_frames": int(getattr(self._calibrator, "_pair_hold_frames", 0)),
            "camera_hfov_deg": float(getattr(self._calibrator, "_camera_hfov_deg", 0.0)),
            "lane_width_tol": float(getattr(vision, "_lane_width_tol", 0.0)),
            "single_line_max_frames": int(getattr(vision, "_single_line_max_frames", 0)),
            "relock_valid_frames": int(getattr(self._calibrator, "_relock_valid_frames", 0)),
            "pid_kp": float(state.pid.kp),
            "pid_kd": float(state.pid.kd),
            "pid_lateral_gain": float(steering._lateral_gain),
            "vp_inner_thresh": float(steering._inner_thresh),
            "vp_outer_thresh": float(steering._outer_thresh),
            "vp_center_deg": float(steering._vp_center_deg),
            "danger_margin_px": int(steering._danger_margin),
            "danger_offset_ratio": float(steering._danger_offset_ratio),
            "danger_nudge_deg": float(steering._nudge_deg),
            **{
                key: float(getattr(self._calibrator, attr, default))
                for key, (attr, default) in _SERVO_ATTRS.items()
            },
            **self._read_cross(),
            **{
                b.param.key: (int(b.getter()) if b.param.value_type == "int" else float(b.getter()))
                for b in self._extras
            },
        }

    def _cross_config(self) -> Any:
        counter = getattr(self._calibrator, "cross_counter", None)
        return getattr(counter, "config", None)

    def _read_cross(self) -> dict[str, int | float]:
        config = self._cross_config() or CrossLineConfig()
        values: dict[str, int | float] = {}
        for key, attr in _CROSS_FIELDS.items():
            param = _PARAMS_BY_KEY[key]
            raw = getattr(config, attr)
            values[key] = int(raw) if param.value_type == "int" else float(raw)
        return values

    def _apply_values(self, values: dict[str, int | float]) -> None:
        vision = self._calibrator._vision
        steering = self._calibrator.steering_controller
        state = self._calibrator.robot_state

        vision._roi_height_pct = float(values["roi_height_pct"])
        vision._roi_top_width_pct = float(values["roi_top_width_pct"])
        vision._roi_bottom_width_pct = float(values["roi_bottom_width_pct"])
        vision._canny_low = int(values["canny_low"])
        vision._canny_high = int(values["canny_high"])
        vision._hough_threshold = int(values["hough_threshold"])
        vision._hough_min_line_length = int(values["hough_min_line_length"])
        vision._hough_max_line_gap = int(values["hough_max_line_gap"])
        vision._min_abs_slope = float(values["min_abs_slope"])
        vision._vp_horizon_y = float(values["vp_horizon_y"])
        vision._vp_search_px = float(values["vp_search_px"])
        vision._vp_row_offset = 0.0
        self._calibrator._pair_hold_frames = int(values["pair_hold_frames"])
        self._calibrator._camera_hfov_deg = float(values["camera_hfov_deg"])
        vision._lane_width_tol = float(values["lane_width_tol"])
        vision._single_line_max_frames = int(values["single_line_max_frames"])
        self._calibrator._relock_valid_frames = int(values["relock_valid_frames"])

        state.pid.kp = float(values["pid_kp"])
        state.pid.kd = float(values["pid_kd"])
        steering._lateral_gain = float(values["pid_lateral_gain"])
        steering._inner_thresh = float(values["vp_inner_thresh"])
        steering._outer_thresh = float(values["vp_outer_thresh"])
        steering._vp_center_deg = float(values["vp_center_deg"])
        steering._danger_margin = int(values["danger_margin_px"])
        steering._danger_offset_ratio = float(values["danger_offset_ratio"])
        steering._nudge_deg = float(values["danger_nudge_deg"])
        for key, (attr, _default) in _SERVO_ATTRS.items():
            setattr(self._calibrator, attr, float(values[key]))
        config = self._cross_config()
        if config is not None:
            for key, attr in _CROSS_FIELDS.items():
                current = getattr(config, attr)
                value = values[key]
                setattr(config, attr, bool(value) if isinstance(current, bool) else type(current)(value))
        for binding in self._extras:
            binding.setter(values[binding.param.key])
        steering._tracking_active = False
        steering._last_error = 0.0
        state.pid_last_error = 0.0

        self._sync_overlay(values)

    def _sync_overlay(self, values: dict[str, int | float]) -> None:
        attrs = {
            "_inner_thresh": float(values["vp_inner_thresh"]),
            "_outer_thresh": float(values["vp_outer_thresh"]),
            "_danger_margin_px": int(values["danger_margin_px"]),
        }
        drawer_attrs = {
            "inner_thresh": float(values["vp_inner_thresh"]),
            "outer_thresh": float(values["vp_outer_thresh"]),
            "danger_margin_px": int(values["danger_margin_px"]),
            "danger_offset_ratio": float(values["danger_offset_ratio"]),
            "roi_height_pct": float(values["roi_height_pct"]),
            "roi_top_width_pct": float(values["roi_top_width_pct"]),
            "roi_bottom_width_pct": float(values["roi_bottom_width_pct"]),
        }
        for owner in (self._calibrator, getattr(self._calibrator, "_telemetry", None)):
            if owner is None:
                continue
            for attr, value in attrs.items():
                if hasattr(owner, attr):
                    setattr(owner, attr, value)
            drawer = getattr(owner, "_overlay_drawer", None)
            if drawer is not None:
                for attr, value in drawer_attrs.items():
                    if hasattr(drawer, attr):
                        setattr(drawer, attr, value)


def _coerce_value(param: TuneParam, value: Any) -> int | float:
    if isinstance(value, bool):
        raise TuneValidationError(f"{param.key} must be a number")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise TuneValidationError(f"{param.key} must be a number") from exc
    if not math.isfinite(numeric):
        raise TuneValidationError(f"{param.key} must be finite")
    if numeric < param.minimum or numeric > param.maximum:
        raise TuneValidationError(
            f"{param.key} must be between {param.minimum:g} and {param.maximum:g}"
        )
    if param.value_type == "int":
        if not numeric.is_integer():
            raise TuneValidationError(f"{param.key} must be an integer")
        return int(numeric)
    return float(numeric)


def _same_values(
    left: dict[str, int | float],
    right: dict[str, int | float],
    params: tuple[TuneParam, ...] = PARAMS,
) -> bool:
    for param in params:
        if param.key not in left or param.key not in right:
            return False
        if abs(float(left[param.key]) - float(right[param.key])) > 1e-9:
            return False
    return True


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
