"""Floor distance from seam row + VP row (vision/ground_range.py)."""

from __future__ import annotations

import pytest

from vision.ground_range import GroundRange, fit_ground_range, seam_points

K, C, VP = 150000.0, 10.0, -340.0


def test_row_and_distance_invert_each_other():
    g = GroundRange(K, C)
    row = g.row_px(230.0, VP)
    assert row == pytest.approx(-340.0 + 150000.0 / 220.0)
    assert g.distance_cm(row, VP) == pytest.approx(230.0)
    # The car pitches: the VP row moves, the same seam moves with it.
    assert g.distance_cm(row + 20.0, VP + 20.0) == pytest.approx(230.0)
    assert GroundRange().distance_cm(row, VP) is None
    assert g.row_px(5.0, VP) is None  # behind the camera


def test_fit_recovers_k_and_c_from_seams_of_one_frame():
    g = GroundRange(K, C)
    lines = [g.row_px(d, VP) for d in (200.0, 260.0, 320.0, 380.0)]
    points = seam_points(list(reversed(lines)), VP, nearest_cm=200.0, tile_cm=60.0)
    assert [p[0] for p in points] == [200.0, 260.0, 320.0, 380.0]  # nearest = lowest row
    fit = fit_ground_range(points)
    assert fit.k_px_cm == pytest.approx(K, rel=1e-6)
    assert fit.c_cm == pytest.approx(C, abs=1e-6)
    assert fit.rms_cm == pytest.approx(0.0, abs=1e-6)


def test_fit_rejects_too_few_or_inconsistent_points():
    with pytest.raises(ValueError):
        fit_ground_range([(200.0, 300.0, VP)])
    with pytest.raises(ValueError):
        fit_ground_range([(200.0, 300.0, VP), (260.0, 300.0, VP)])
    with pytest.raises(ValueError):  # farther seam drawn lower: k < 0
        fit_ground_range([(200.0, 200.0, VP), (260.0, 300.0, VP)])
