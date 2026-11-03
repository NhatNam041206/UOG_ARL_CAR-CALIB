"""Tests for the transverse tile-seam counter on synthetic floor frames."""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest

from vision.cross_line_counter import CrossLineConfig, CrossLineCounter

W, H = 640, 480


def _floor(seams_y: list[float], tilt_deg: float = 0.0, contrast: int = 40) -> np.ndarray:
    """Light floor with two steep longitudinal seams and the given transverse seams."""
    frame = np.full((H, W, 3), 200, dtype=np.uint8)
    cv2.line(frame, (200, 0), (60, H - 1), (60, 60, 60), 3)
    cv2.line(frame, (440, 0), (600, H - 1), (60, 60, 60), 3)
    slope = math.tan(math.radians(tilt_deg))
    shade = 200 - contrast
    for y in seams_y:
        y0 = int(y * H - slope * W / 2)
        y1 = int(y * H + slope * W / 2)
        cv2.line(frame, (0, y0), (W - 1, y1), (shade, shade, shade), 3)
    return frame


def _drive(counter: CrossLineCounter, start: float, speed: float, frames: int, spacing: float, **kw) -> int:
    """Scroll a periodic seam pattern down the image; return the final count."""
    for i in range(frames):
        offset = (start + i * speed) % spacing
        seams = [offset + k * spacing for k in range(int(1 / spacing) + 2)]
        counter.update(_floor([y for y in seams if 0.0 <= y <= 1.0], **kw))
    return counter.tile_count


def test_detect_reports_y_and_tilt_of_a_faint_seam():
    counter = CrossLineCounter(CrossLineConfig())
    lines = counter.detect(_floor([0.3], tilt_deg=8.0, contrast=15))
    assert len(lines) == 1
    assert abs(lines[0].y - 0.3) < 0.02
    assert abs(lines[0].tilt_deg - 8.0) < 2.0


def test_detect_ignores_steep_longitudinal_seams():
    counter = CrossLineCounter(CrossLineConfig())
    assert counter.detect(_floor([])) == []


def test_counts_each_seam_crossing_once():
    counter = CrossLineCounter(CrossLineConfig())
    # Seams 0.4 frame-heights apart moving 0.025 per frame: 16 frames per tile.
    # 200 frames move the pattern 5.0 heights, i.e. 12.5 tiles past the line.
    count = _drive(counter, start=0.0, speed=0.025, frames=200, spacing=0.4)
    assert count in (12, 13)


def test_counts_with_vehicle_yaw():
    counter = CrossLineCounter(CrossLineConfig())
    count = _drive(counter, start=0.0, speed=0.025, frames=200, spacing=0.4, tilt_deg=20.0)
    assert count in (12, 13)


def test_stationary_seam_on_counter_line_is_not_counted():
    counter = CrossLineCounter(CrossLineConfig())
    cfg = counter.config
    for _ in range(50):
        counter.update(_floor([cfg.counter_y + 0.01]))
    assert counter.tile_count == 0


def test_seam_lost_below_lookahead_roi_still_counts_by_coasting():
    cfg = CrossLineConfig()
    counter = CrossLineCounter(cfg)
    y = cfg.roi_top + 0.02
    while y < cfg.counter_y + cfg.counter_h:
        # Blur: the seam is invisible once it leaves the look-ahead ROI.
        counter.update(_floor([y] if y <= cfg.roi_bottom else []))
        y += 0.03
    assert counter.tile_count == 1


def test_reset_clears_count_and_disabled_counter_does_nothing():
    counter = CrossLineCounter(CrossLineConfig())
    _drive(counter, start=0.0, speed=0.025, frames=60, spacing=0.4)
    assert counter.tile_count > 0
    counter.reset()
    assert counter.tile_count == 0
    counter.config.enabled = False
    _drive(counter, start=0.0, speed=0.025, frames=60, spacing=0.4)
    assert counter.tile_count == 0


def test_from_env_reads_overrides(monkeypatch):
    monkeypatch.setenv("CROSS_COUNTER_Y", "0.6")
    monkeypatch.setenv("CROSS_ENABLED", "false")
    cfg = CrossLineConfig.from_env()
    assert cfg.counter_y == 0.6
    assert cfg.enabled is False


def test_draw_keeps_frame_shape():
    counter = CrossLineCounter(CrossLineConfig())
    frame = _floor([0.3])
    result = counter.update(frame)
    out = counter.draw(frame.copy(), result)
    assert out.shape == frame.shape
    assert set(result.telemetry()) == {"cross_tile_count", "cross_net_count", "cross_direction", "cross_counted", "cross_lines", "cross_tracks"}


def test_tune_manager_reads_and_applies_cross_params(tmp_path):
    from runtime.calib_tuning import CalibTuneManager, TuneValidationError
    from tests.test_calib_tuning import _fake_calibrator

    calibrator = _fake_calibrator()
    calibrator.cross_counter = CrossLineCounter(CrossLineConfig())
    manager = CalibTuneManager(calibrator, tmp_path / "tune.json")
    values = manager.status()["values"]
    assert values["cross_counter_y"] == calibrator.cross_counter.config.counter_y
    assert values["cross_enabled"] == 1

    manager.apply({"cross_counter_y": 0.55, "cross_enabled": 0, "cross_confirm_frames": 3})
    cfg = calibrator.cross_counter.config
    assert cfg.counter_y == 0.55
    assert cfg.enabled is False
    assert cfg.confirm_frames == 3

    with pytest.raises(TuneValidationError):
        manager.apply({"cross_roi_top": 0.5, "cross_roi_bottom": 0.4})


def _reverse(counter: CrossLineCounter, start: float, speed: float, frames: int, spacing: float) -> None:
    """Scroll the seam pattern up the image (car reversing)."""
    for i in range(frames):
        offset = (start - i * speed) % spacing
        seams = [offset + k * spacing for k in range(int(1 / spacing) + 2)]
        counter.update(_floor([y for y in seams if 0.0 <= y <= 1.0]))


def test_reversing_counts_seams_moving_up_and_lowers_the_net_count():
    counter = CrossLineCounter(CrossLineConfig())
    _drive(counter, start=0.0, speed=0.025, frames=200, spacing=0.4)
    forward = counter.tile_count
    assert counter.net_count == forward
    counter.set_direction(-1)
    _reverse(counter, start=0.0, speed=0.025, frames=100, spacing=0.4)
    back = counter.tile_count - forward
    assert back in (5, 6, 7)
    assert counter.net_count == forward - back
    assert counter.direction == -1


def test_forward_counter_ignores_seams_moving_up():
    counter = CrossLineCounter(CrossLineConfig())
    _reverse(counter, start=0.0, speed=0.025, frames=100, spacing=0.4)
    assert counter.tile_count <= 1
