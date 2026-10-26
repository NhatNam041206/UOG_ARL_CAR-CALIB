from __future__ import annotations

import numpy as np

import unified_calibration_components as ucc
from tests.test_vp_row_tracking import _lane_pair, _processor
from unified_calibration_components import UnifiedCalibrator, _x_at_row


def _vision():
    vision = _processor(-380, 0)
    vision._lane_width_tol = 0.3
    vision._single_line_max_frames = 30
    return vision


def _bottoms(pair):
    return [round(_x_at_row(line, 480)) for line in pair]


def test_width_gate_rejects_implausible_pair() -> None:
    vision = _vision()
    assert _bottoms(vision._apply_geometric_filter(_lane_pair(300, -380, 40, 600), 640, 480)) == [40, 600]
    # A wall edge close to the centre would give a 200 px "lane": not a lane.
    lines = _lane_pair(300, -380, 250, 450)
    assert vision._apply_geometric_filter(lines, 640, 480) is None


def test_prefers_lines_near_previous_pair() -> None:
    vision = _vision()
    vision._lane_width_tol = 0.0
    vision._apply_geometric_filter(_lane_pair(300, -380, 40, 600), 640, 480)
    # An extra line just inside the real left line would win on "innermost".
    lines = _lane_pair(300, -380, 45, 605) + _lane_pair(300, -380, 200, 605)[:1]
    assert _bottoms(vision._apply_geometric_filter(lines, 640, 480)) == [45, 605]


def test_single_line_infers_other_from_lane_width() -> None:
    vision = _vision()
    vision._apply_geometric_filter(_lane_pair(300, -380, 40, 600), 640, 480)
    right_only = _lane_pair(300, -380, 40, 610)[1:]
    pair = vision._apply_geometric_filter(right_only, 640, 480)
    assert vision.last_pair_kind == "SINGLE_RIGHT"
    assert _bottoms(pair) == [50, 610]
    # The inferred line meets the visible one on the VP row.
    vp = ucc.GeometryCalculator.calculate_vanishing_point(*pair)
    assert abs(vp[0] - 300) <= 2 and abs(vp[1] + 380) <= 2


def test_single_line_is_capped_and_needs_a_previous_pair() -> None:
    right_only = _lane_pair(300, -380, 40, 600)[1:]
    assert _vision()._apply_geometric_filter(right_only, 640, 480) is None

    vision = _vision()
    vision._single_line_max_frames = 2
    vision._apply_geometric_filter(_lane_pair(300, -380, 40, 600), 640, 480)
    found = [vision._apply_geometric_filter(right_only, 640, 480) is not None for _ in range(3)]
    assert found == [True, True, False]


def test_relock_waits_for_stable_pair_after_gapping() -> None:
    cv2 = ucc.cv2
    calibrator = UnifiedCalibrator(telemetry_enabled=False)
    calibrator._vision._roi_height_pct = 1.0
    calibrator._vision._vp_horizon_y = -380
    calibrator._pair_hold_frames = 0
    calibrator._relock_valid_frames = 2
    lane = np.full((480, 640, 3), 200, np.uint8)
    for line in _lane_pair(300, -380, 40, 600):
        cv2.line(lane, line[:2], line[2:], (40, 40, 40), 4)
    blank = np.full((480, 640, 3), 200, np.uint8)

    frames = [lane, blank, lane, lane, lane]
    states = [calibrator.process_frame(f, i).telemetry for i, f in enumerate(frames)]
    assert states[0]["fsm_state"] != "GAPPING"
    assert states[1]["fsm_state"] == "GAPPING"
    assert states[2]["command_source"] == "RELOCK"
    assert states[2]["fsm_state"] == "GAPPING" and states[2]["relock_frames"] == 1
    assert states[3]["fsm_state"] != "GAPPING" and states[3]["command_source"] != "RELOCK"
