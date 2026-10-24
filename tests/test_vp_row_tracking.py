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
