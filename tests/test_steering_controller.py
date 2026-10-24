from __future__ import annotations

import pytest

from control.steering_controller import SteeringController
from models.robot_state import PIDConstants


def _controller(danger_margin: int = 130) -> SteeringController:
    return SteeringController(
        pid_constants=PIDConstants(kp=1.0, ki=0.0, kd=0.3),
        danger_margin=danger_margin,
        nudge_deg=7.0,
        inner_thresh=3.0,
        outer_thresh=8.0,
        center_angle=90.0,
        max_offset=90.0,
    )


def test_gapping_when_vision_lost() -> None:
    angle, state = _controller().compute_steering(
        vp_angle=None,
        left_intercept=None,
        right_intercept=None,
        frame_width=640,
    )
    assert state == "GAPPING"
    assert angle == pytest.approx(90.0)


def test_intercepts_just_outside_margin_stay_in_tracking_not_danger() -> None:
    """Reproduces the real-car report: L=131, R=490, margin=130, heading error
    only +3.1 deg -- should land in TRACKING_COAST, not a DANGER state, since
    neither intercept is actually inside the danger band near either edge."""
    angle, state = _controller(danger_margin=130).compute_steering(
        vp_angle=93.1,
        left_intercept=131,
        right_intercept=490,
        frame_width=640,
    )
    assert state == "TRACKING_COAST"
    assert angle == pytest.approx(90.0)


def test_danger_left_when_left_intercept_encroaches_left_edge() -> None:
    """Left intercept inside the left edge band means both lines shifted left in
    the image, i.e. the car drifted toward the right line -- recover by turning
    left (angle above center, same side PD uses for a VP left of center)."""
    angle, state = _controller(danger_margin=130).compute_steering(
        vp_angle=90.0,
        left_intercept=50,
        right_intercept=490,
        frame_width=640,
    )
    assert state == "DANGER_LEFT"
    assert angle == pytest.approx(97.0)


def test_danger_right_when_right_intercept_encroaches_right_edge() -> None:
    """Right intercept inside the right edge band means the car drifted toward
    the left line -- recover by turning right (angle below center, same side
    PD uses for a VP right of center)."""
    angle, state = _controller(danger_margin=130).compute_steering(
        vp_angle=90.0,
        left_intercept=150,
        right_intercept=600,
        frame_width=640,
    )
    assert state == "DANGER_RIGHT"
    assert angle == pytest.approx(83.0)


@pytest.mark.parametrize(
    "state, expected",
    [("DANGER_LEFT", "LEFT"), ("DANGER_RIGHT", "RIGHT"), ("TRACKING_PD", "NONE")],
)
def test_recovery_direction_matches_nudge_side(state: str, expected: str) -> None:
    details = _controller().describe_control_state(state, 640)
    assert details["recovery_direction"] == expected


def test_tracking_coast_inside_inner_threshold() -> None:
    angle, state = _controller().compute_steering(
        vp_angle=91.0,
        left_intercept=200,
        right_intercept=440,
        frame_width=640,
    )
    assert state == "TRACKING_COAST"
    assert angle == pytest.approx(90.0)


def test_tracking_pd_past_outer_threshold() -> None:
    angle, state = _controller().compute_steering(
        vp_angle=100.0,
        left_intercept=200,
        right_intercept=440,
        frame_width=640,
    )
    assert state == "TRACKING_PD"
    assert angle != pytest.approx(90.0)


def _ratio_controller(ratio: float = 0.25, vp_center_deg: float = 90.0) -> SteeringController:
    return SteeringController(
        pid_constants=PIDConstants(kp=1.0, ki=0.0, kd=0.0),
        danger_margin=130,
        nudge_deg=7.0,
        inner_thresh=3.0,
        outer_thresh=8.0,
        center_angle=90.0,
        max_offset=90.0,
        danger_offset_ratio=ratio,
        vp_center_deg=vp_center_deg,
    )


def test_offset_ratio_ignores_wide_lane_that_crosses_both_edge_bands() -> None:
    """Lower camera run 121051: L=-27, R=576 (lane wider than the frame band
    allows). The legacy rule flagged DANGER_LEFT every frame; the lane centre
    is only 0.08 lane widths off the image centre, so PD must run."""
    _, state = _ratio_controller().compute_steering(
        vp_angle=78.0,
        left_intercept=-27,
        right_intercept=576,
        frame_width=640,
    )
    assert state == "TRACKING_PD"


def test_offset_ratio_nudges_toward_lane_centre() -> None:
    # Lane centre far left of the image centre: car sits right, nudge left.
    angle, state = _ratio_controller().compute_steering(
        vp_angle=90.0, left_intercept=-250, right_intercept=300, frame_width=640,
    )
    assert state == "DANGER_LEFT"
    assert angle == pytest.approx(97.0)
    # Lane centre far right of the image centre: car sits left, nudge right.
    angle, state = _ratio_controller().compute_steering(
        vp_angle=90.0, left_intercept=340, right_intercept=890, frame_width=640,
    )
    assert state == "DANGER_RIGHT"
    assert angle == pytest.approx(83.0)


def test_vp_center_deg_shifts_the_straight_ahead_reference() -> None:
    _, state = _ratio_controller(vp_center_deg=78.0).compute_steering(
        vp_angle=78.0, left_intercept=40, right_intercept=600, frame_width=640,
    )
    assert state == "TRACKING_COAST"


def test_offset_danger_uses_pd_when_it_steers_the_same_way_harder() -> None:
    # Car right of lane centre and yawed right (VP left): PD left beats the nudge.
    controller = _ratio_controller()
    angle, state = controller.compute_steering(
        vp_angle=60.0, left_intercept=-250, right_intercept=300, frame_width=640,
    )
    assert state == "DANGER_LEFT"
    assert angle == pytest.approx(120.0)
    assert controller.last_command_source == "DANGER_PD"


def test_offset_danger_uses_pd_when_car_already_heads_back() -> None:
    # Car left of lane centre (nudge right) but already yawed hard right
    # (VP left of the reference): PD straightens it left; adding more right
    # (the nudge) would drive it across the lane.
    controller = _ratio_controller()
    angle, state = controller.compute_steering(
        vp_angle=36.0, left_intercept=292, right_intercept=743, frame_width=640,
    )
    assert state == "DANGER_RIGHT"
    assert angle > 90.0
    assert controller.last_command_source == "DANGER_PD"


def test_offset_danger_keeps_nudge_when_heading_is_small() -> None:
    controller = _ratio_controller()
    angle, state = controller.compute_steering(
        vp_angle=92.0, left_intercept=-250, right_intercept=300, frame_width=640,
    )
    assert state == "DANGER_LEFT"
    assert angle == pytest.approx(97.0)
    assert controller.last_command_source == "NUDGE"
