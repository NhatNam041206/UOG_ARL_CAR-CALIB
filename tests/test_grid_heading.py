"""Grid heading and turn yaw on synthetic tile-floor frames."""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest

from vision.grid_heading import GridHeadingConfig, GridHeadingEstimator, TurnYawTracker, wrap90


def tile_floor(yaw_deg: float, w: int = 640, h: int = 480, f: float = 400.0, pitch_deg: float = 55.0,
               cam_h: float = 0.25, tile: float = 0.3, shift: float = 0.0) -> np.ndarray:
    """Light floor with dark seams; the car is yawed left by yaw_deg relative to the grid."""
    img = np.full((h, w), 190, np.uint8)
    psi, th = math.radians(yaw_deg), math.radians(pitch_deg)
    fwd = np.array([-math.sin(psi), math.cos(psi)])
    right = np.array([math.cos(psi), math.sin(psi)])

    def project(pts: np.ndarray) -> np.ndarray:
        rel = pts - np.array([0.0, shift])
        x = rel @ right
        z = rel @ fwd
        y = np.full_like(x, cam_h)
        yp = y * math.cos(th) - z * math.sin(th)
        zp = y * math.sin(th) + z * math.cos(th)
        uv = np.stack([w / 2 + f * x / zp, h / 2 + f * yp / zp], axis=1)
        uv[zp < 0.05] = np.nan
        return uv

    s = np.linspace(-6, 6, 600)
    for k in np.arange(-20, 21) * tile:
        for pts in (np.stack([np.full_like(s, k), s], 1), np.stack([s, np.full_like(s, k)], 1)):
            uv = project(pts)
            ok = ~np.isnan(uv[:, 0]) & (np.abs(uv[:, 0]) < 4 * w) & (np.abs(uv[:, 1]) < 4 * h)
            poly = uv[ok].astype(np.int32)
            if len(poly) >= 2:
                cv2.polylines(img, [poly], False, 90, 2, cv2.LINE_AA)
    return cv2.GaussianBlur(img, (3, 3), 0)


@pytest.mark.parametrize("yaw", [0.0, 8.0, -12.0, 20.0, 35.0])
def test_estimator_recovers_heading_sign_and_size(yaw: float):
    est = GridHeadingEstimator(GridHeadingConfig(g_px=400.0 / math.cos(math.radians(55.0))))
    psi = est.estimate(tile_floor(yaw)).psi_deg
    assert psi is not None
    assert psi == pytest.approx(yaw, abs=2.0)


def test_heading_is_modulo_90():
    est = GridHeadingEstimator(GridHeadingConfig(g_px=400.0 / math.cos(math.radians(55.0))))
    psi = est.estimate(tile_floor(100.0)).psi_deg
    assert psi is not None and wrap90(psi - 100.0) == pytest.approx(0.0, abs=2.0)


def test_tracker_unwraps_a_left_turn_past_45_degrees():
    tracker = TurnYawTracker(GridHeadingEstimator(GridHeadingConfig(g_px=400.0 / math.cos(math.radians(55.0)))))
    t = 0.0
    for yaw in np.arange(0.0, 96.0, 3.0):
        tracker.update(tile_floor(float(yaw)), now=t)
        t += 0.07
    s = tracker.sample(now=t - 0.07)
    assert s.valid
    assert s.yaw_deg == pytest.approx(93.0, abs=4.0)
    assert s.omega_dps == pytest.approx(3.0 / 0.07, rel=0.3)


def test_tracker_right_turn_is_negative():
    tracker = TurnYawTracker(GridHeadingEstimator(GridHeadingConfig(g_px=400.0 / math.cos(math.radians(55.0)))))
    t = 0.0
    for yaw in np.arange(0.0, -61.0, -3.0):
        tracker.update(tile_floor(float(yaw)), now=t)
        t += 0.07
    assert tracker.sample(now=t - 0.07).yaw_deg == pytest.approx(-60.0, abs=4.0)


def test_tracker_coasts_briefly_then_goes_invalid():
    tracker = TurnYawTracker(GridHeadingEstimator(GridHeadingConfig(g_px=400.0 / math.cos(math.radians(55.0)))))
    t = 0.0
    for yaw in (0.0, 3.0, 6.0, 9.0):
        tracker.update(tile_floor(yaw), now=t)
        t += 0.07
    blank = np.full((480, 640), 190, np.uint8)
    s = tracker.update(blank, now=t)
    assert s.source in ("coast", "phase") and s.valid and s.yaw_deg > 9.0
    s = tracker.update(blank, now=t + 2.0)
    assert not s.valid


def test_tracker_reset_zeroes_yaw():
    tracker = TurnYawTracker(GridHeadingEstimator(GridHeadingConfig(g_px=400.0 / math.cos(math.radians(55.0)))))
    tracker.update(tile_floor(0.0), now=0.0)
    tracker.update(tile_floor(10.0), now=0.1)
    tracker.reset()
    assert tracker.sample(now=0.1).yaw_deg == 0.0
