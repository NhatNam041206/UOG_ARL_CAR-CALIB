from __future__ import annotations

from unified_calibration_components import VisionProcessor


def _lane_pair(vp_x: float, vp_y: float, bottom_left: float, bottom_right: float, h: int = 480):
    """Two lane segments in the lower image that converge at (vp_x, vp_y)."""

    def segment(x_bottom: float) -> tuple[int, int, int, int]:
        y1, y2 = h - 1, 200
        x_at = lambda y: x_bottom + (vp_x - x_bottom) * (h - 1 - y) / (h - 1 - vp_y)
        return (int(round(x_at(y1))), y1, int(round(x_at(y2))), y2)

    return [segment(bottom_left), segment(bottom_right)]


def _processor(horizon: float, search: float) -> VisionProcessor:
    vision = VisionProcessor(roi_height_pct=1.0)
    vision._vp_horizon_y = horizon
    vision._vp_search_px = search
    vision._vp_row_offset = 0.0
    return vision


def test_fixed_row_misses_pair_when_vp_moved() -> None:
    lines = _lane_pair(300, -380, -30, 580)
    assert _processor(-518, 0)._apply_geometric_filter(lines, 640, 480) is None


def test_search_finds_pair_and_tracks_vp_row() -> None:
    vision = _processor(-518, 200)
    lines = _lane_pair(300, -380, -30, 580)
    for _ in range(30):
        assert vision._apply_geometric_filter(lines, 640, 480) is not None
    assert abs(vision.vp_row - (-380)) < 10


def test_tracked_row_stays_within_search_window() -> None:
    vision = _processor(-518, 100)
    lines = _lane_pair(300, -250, -30, 580)
    for _ in range(30):
        vision._apply_geometric_filter(lines, 640, 480)
    assert -618 <= vision.vp_row <= -418


def test_calibrator_holds_last_pair_for_brief_dropouts() -> None:
    import numpy as np
    import unified_calibration_components as ucc
    from unified_calibration_components import UnifiedCalibrator

    # Draw with the module's own cv2: another test swaps sys.modules["cv2"].
    cv2 = ucc.cv2

    calibrator = UnifiedCalibrator(telemetry_enabled=False)
    calibrator._vision._roi_height_pct = 1.0
    calibrator._vision._vp_horizon_y = -380
    calibrator._pair_hold_frames = 2
    lane = np.full((480, 640, 3), 200, np.uint8)
    for line in _lane_pair(300, -380, 40, 600):
        cv2.line(lane, line[:2], line[2:], (40, 40, 40), 4)
    blank = np.full((480, 640, 3), 200, np.uint8)

    states = [calibrator.process_frame(f, i).telemetry for i, f in enumerate([lane, blank, blank, blank])]
    assert states[0]["selected_pair_found"] == 1
    assert [s["pair_held_frames"] for s in states[1:3]] == [1, 2]
    assert states[1]["fsm_state"] != "GAPPING"
    assert states[3]["fsm_state"] == "GAPPING"


def test_vp_angle_true_degrees_with_hfov() -> None:
    from unified_calibration_components import GeometryCalculator

    assert GeometryCalculator.map_vp_to_angle(320, 640, 50.0) == 90.0
    # Image edge sits at half the HFOV.
    assert abs(GeometryCalculator.map_vp_to_angle(640, 640, 50.0) - 115.0) < 1e-6
    assert abs(GeometryCalculator.map_vp_to_angle(0, 640, 50.0) - 65.0) < 1e-6
    # Legacy proxy unchanged.
    assert GeometryCalculator.map_vp_to_angle(640, 640) == 180.0
