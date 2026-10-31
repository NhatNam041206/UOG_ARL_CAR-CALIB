"""Car heading relative to the floor tile grid, and yaw tracking for turns.

Heading (GridHeadingEstimator), per frame, no focal length or pitch needed:

1. Dark-line filter: a square black-hat keeps the thin dark grout of both
   seam families (longitudinal and transverse); HoughLinesP finds segments.
2. Two vanishing points by RANSAC: the family with the most segments
   through one point first, then the same on the remaining segments.
3. The two families are perpendicular on the floor, so their vanishing
   points satisfy x1 * x2 = -g^2 (x measured from the frame's centre
   column, both on the horizon). g is learned online from frames that show
   both; until then GRID_HEADING_G_PX is used.
4. psi = atan(x / g) for the vanishing point closest to the centre column.
   psi > 0 means the car points left of that seam family. The grid looks
   the same every 90 degrees, so psi is only known modulo 90 (-45..45).

Yaw (TurnYawTracker): reset() at the start of a turn, then update() every
frame. Wrapped psi steps are unwrapped and summed, so the yaw since reset is
continuous past +-45 degrees. While the grid is not visible, a phase
correlation of the upper image band gives the horizontal image shift; its
pixels-per-degree factor is learned from frames where both measures exist.
Without either, the tracker coasts on the last yaw rate for a short time.
Turning left gives positive yaw.

Both are measurements only; the route script runner decides what to do.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any

import cv2
import numpy as np

from config.settings import _get_bool, _get_float, _get_int


@dataclass
class GridHeadingConfig:
    enabled: bool = True
    scale: float = 0.5  # line search runs on a resized frame; pixel sizes below are full-frame
    blackhat_kernel_px: int = 9
    dark_thresh: int = 12
    hough_votes: int = 60
    min_line_px: int = 70
    max_gap_px: int = 12
    max_lines: int = 60
    ransac_iters: int = 120
    inlier_tol_deg: float = 2.0
    min_inliers: int = 4
    g_px: float = 675.0  # used until enough two-VP frames are seen
    horizon_y: float | None = None  # seed; learned online when None
    horizon_tol_px: float = 80.0
    # A floor VP lies above the top of the image (horizon ~-340 px on the car).
    # Below this row a "VP" is a tile corner where two seams of different
    # families cross, which would win RANSAC with segments of both families.
    max_vp_y: float = -60.0
    pc_band: float = 0.25  # top fraction of the frame used by phase correlation
    pc_scale: float = 0.5
    pc_min_response: float = 0.05
    pc_min_std: float = 4.0  # a flat band (wall, blur) has no shift to measure
    coast_max_s: float = 0.5

    @classmethod
    def from_env(cls) -> "GridHeadingConfig":
        d = cls()
        return cls(
            enabled=_get_bool("GRID_HEADING_ENABLED", d.enabled),
            scale=_get_float("GRID_HEADING_SCALE", d.scale),
            blackhat_kernel_px=_get_int("GRID_HEADING_BLACKHAT_PX", d.blackhat_kernel_px),
            dark_thresh=_get_int("GRID_HEADING_DARK_THRESH", d.dark_thresh),
            hough_votes=_get_int("GRID_HEADING_HOUGH_VOTES", d.hough_votes),
            min_line_px=_get_int("GRID_HEADING_MIN_LINE_PX", d.min_line_px),
            max_gap_px=_get_int("GRID_HEADING_MAX_GAP_PX", d.max_gap_px),
            max_lines=_get_int("GRID_HEADING_MAX_LINES", d.max_lines),
            ransac_iters=_get_int("GRID_HEADING_RANSAC_ITERS", d.ransac_iters),
            inlier_tol_deg=_get_float("GRID_HEADING_INLIER_TOL_DEG", d.inlier_tol_deg),
            min_inliers=_get_int("GRID_HEADING_MIN_INLIERS", d.min_inliers),
            g_px=_get_float("GRID_HEADING_G_PX", d.g_px),
            horizon_tol_px=_get_float("GRID_HEADING_HORIZON_TOL_PX", d.horizon_tol_px),
            max_vp_y=_get_float("GRID_HEADING_MAX_VP_Y", d.max_vp_y),
            pc_band=_get_float("GRID_HEADING_PC_BAND", d.pc_band),
            pc_scale=_get_float("GRID_HEADING_PC_SCALE", d.pc_scale),
            pc_min_response=_get_float("GRID_HEADING_PC_MIN_RESPONSE", d.pc_min_response),
            pc_min_std=_get_float("GRID_HEADING_PC_MIN_STD", d.pc_min_std),
            coast_max_s=_get_float("GRID_HEADING_COAST_MAX_S", d.coast_max_s),
        )

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class GridHeading:
    """One frame's heading; x/y of vanishing points are relative to the centre column."""

    psi_deg: float | None
    vps: list[tuple[float, float, int]]  # (x, y, inliers)
    n_lines: int
    g_px: float
    horizon_y: float | None
    ms: float

    def telemetry(self) -> dict[str, Any]:
        return {
            "grid_psi": None if self.psi_deg is None else round(self.psi_deg, 2),
            "grid_vps": len(self.vps),
            "grid_lines": self.n_lines,
            "grid_g_px": round(self.g_px, 1),
            "grid_ms": round(self.ms, 2),
        }


def wrap90(deg: float) -> float:
    """Wrap an angle difference into [-45, 45)."""
    return (deg + 45.0) % 90.0 - 45.0


class GridHeadingEstimator:
    """Heading of the car relative to the tile grid, modulo 90 degrees."""

    def __init__(self, config: GridHeadingConfig | None = None, seed: int = 0) -> None:
        self.config = config or GridHeadingConfig.from_env()
        self._rng = np.random.default_rng(seed)
        self._g_samples: deque[float] = deque(maxlen=60)
        self._horizon_samples: deque[float] = deque(maxlen=60)
        self._horizon_seed: float | None = self.config.horizon_y

    def set_horizon_seed(self, horizon_y: float | None) -> None:
        """Horizon row from the lane tune (vp_horizon_y); used until learned."""
        self._horizon_seed = None if horizon_y is None else float(horizon_y)

    @property
    def g_px(self) -> float:
        if len(self._g_samples) >= 8:
            return float(np.median(self._g_samples))
        return float(self.config.g_px)

    @property
    def horizon_y(self) -> float | None:
        if len(self._horizon_samples) >= 8:
            return float(np.median(self._horizon_samples))
        return self._horizon_seed

    def lines(self, gray: np.ndarray) -> np.ndarray:
        """Seam segments in full-frame pixels."""
        cfg = self.config
        sc = max(0.1, min(1.0, float(cfg.scale)))
        small = gray if sc == 1.0 else cv2.resize(gray, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
        k = max(3, int(round(cfg.blackhat_kernel_px * sc)) | 1)
        dark = cv2.morphologyEx(small, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
        _, mask = cv2.threshold(dark, cfg.dark_thresh, 255, cv2.THRESH_BINARY)
        found = cv2.HoughLinesP(
            mask,
            1,
            np.pi / 180,
            max(10, int(cfg.hough_votes * sc)),
            minLineLength=cfg.min_line_px * sc,
            maxLineGap=cfg.max_gap_px * sc,
        )
        if found is None:
            return np.zeros((0, 4))
        segs = found.reshape(-1, 4).astype(float) / sc
        if len(segs) > cfg.max_lines:
            lengths = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
            segs = segs[np.argsort(-lengths)[: cfg.max_lines]]
        return segs

    def _vp_ransac(self, segs: np.ndarray) -> tuple[np.ndarray | None, np.ndarray]:
        """Best vanishing point (homogeneous, normalised) and its inlier mask."""
        n = len(segs)
        empty = np.zeros(n, dtype=bool)
        if n < 2:
            return None, empty
        ones = np.ones((n, 1))
        hom = np.cross(np.hstack((segs[:, :2], ones)), np.hstack((segs[:, 2:], ones)))
        mid = (segs[:, :2] + segs[:, 2:]) / 2.0
        seg_ang = np.arctan2(segs[:, 3] - segs[:, 1], segs[:, 2] - segs[:, 0])
        tol = math.radians(self.config.inlier_tol_deg)
        # All hypotheses at once: M random pairs -> M candidate points, then an
        # M x n inlier table (a Python loop per hypothesis costs ~10x more).
        m = max(1, self.config.ransac_iters)
        i = self._rng.integers(0, n, m)
        j = (i + self._rng.integers(1, n, m)) % n
        pts = np.cross(hom[i], hom[j])
        keep = np.abs(pts[:, 2]) > 1e-9
        if not keep.any():
            return None, empty
        pts = pts[keep] / pts[keep, 2:3]
        pts = pts[pts[:, 1] < self.config.max_vp_y]
        if not len(pts):
            return None, empty
        to_vp = np.arctan2(pts[:, 1:2] - mid[None, :, 1], pts[:, 0:1] - mid[None, :, 0])
        d = np.abs((seg_ang[None, :] - to_vp + np.pi / 2) % np.pi - np.pi / 2)
        masks = d < tol
        best = int(np.argmax(masks.sum(axis=1)))
        best_p, best_mask = pts[best], masks[best]
        return best_p, best_mask

    def estimate(self, frame: np.ndarray) -> GridHeading:
        cfg = self.config
        t0 = time.perf_counter()
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cx = gray.shape[1] / 2.0
        segs = self.lines(gray)
        vps: list[tuple[float, float, int]] = []
        rest = segs
        for _ in range(2):
            p, mask = self._vp_ransac(rest)
            if p is None or int(mask.sum()) < cfg.min_inliers:
                break
            vps.append((float(p[0] - cx), float(p[1]), int(mask.sum())))
            rest = rest[~mask]

        hz = self.horizon_y
        if hz is not None:
            vps = [v for v in vps if abs(v[1] - hz) <= cfg.horizon_tol_px]
        # Learn g and the horizon from frames that show both families.
        if len(vps) == 2 and vps[0][0] * vps[1][0] < 0 and min(abs(vps[0][0]), abs(vps[1][0])) > 60:
            self._g_samples.append(math.sqrt(-vps[0][0] * vps[1][0]))
            self._horizon_samples.append((vps[0][1] + vps[1][1]) / 2.0)
        g = self.g_px
        psi = None
        near = [v for v in vps if abs(v[0]) < g]
        if near:
            x = min(near, key=lambda v: abs(v[0]))[0]
            psi = math.degrees(math.atan(x / g))
        return GridHeading(psi, vps, len(segs), g, hz, (time.perf_counter() - t0) * 1000.0)


@dataclass
class YawSample:
    """What the script runner reads: yaw since reset and the latest heading."""

    yaw_deg: float  # since reset(), left positive
    omega_dps: float | None  # yaw rate, left positive
    psi_deg: float | None  # heading vs tile grid (mod 90), left positive
    source: str  # grid | phase | coast | none
    age_s: float  # since the last real measurement
    valid: bool
    lane_offset_ratio: float | None = None


class TurnYawTracker:
    """Unwrap grid heading (with phase correlation fallback) into yaw since reset()."""

    def __init__(self, estimator: GridHeadingEstimator | None = None) -> None:
        self.estimator = estimator or GridHeadingEstimator()
        self.config = self.estimator.config
        self._px_per_deg: float | None = None
        self.last_heading: GridHeading | None = None
        self._reset_pending = False
        self.reset()

    def request_reset(self) -> None:
        """Thread-safe reset from the script runner; applied on the next update()."""
        self._reset_pending = True

    def reset(self) -> None:
        self._reset_pending = False
        self._yaw = 0.0
        self._omega: float | None = None
        self._last_psi: float | None = None
        self._last_t: float | None = None
        self._last_meas_t: float | None = None
        self._prev_band: np.ndarray | None = None
        self._source = "none"
        self._lane_offset: float | None = None
        self._last_psi_seen: float | None = None

    def _band(self, gray: np.ndarray) -> np.ndarray:
        cfg = self.config
        h = max(8, int(gray.shape[0] * cfg.pc_band))
        band = gray[:h]
        if cfg.pc_scale != 1.0:
            band = cv2.resize(band, None, fx=cfg.pc_scale, fy=cfg.pc_scale, interpolation=cv2.INTER_AREA)
        return band.astype(np.float32)

    def _phase_dx(self, band: np.ndarray) -> float | None:
        """Horizontal shift (full-resolution px) since the previous band."""
        prev = self._prev_band
        if prev is None or prev.shape != band.shape:
            return None
        if float(band.std()) < self.config.pc_min_std or float(prev.std()) < self.config.pc_min_std:
            return None
        win = cv2.createHanningWindow((band.shape[1], band.shape[0]), cv2.CV_32F)
        (dx, _dy), response = cv2.phaseCorrelate(prev, band, win)
        if response < self.config.pc_min_response:
            return None
        return float(dx) / max(1e-6, self.config.pc_scale)

    def update(
        self,
        frame: np.ndarray,
        now: float | None = None,
        lane_offset_ratio: float | None = None,
        use_phase: bool = True,
    ) -> YawSample:
        now = time.monotonic() if now is None else now
        if self._reset_pending:
            self.reset()
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        heading = self.estimator.estimate(gray) if self.config.enabled else None
        self.last_heading = heading
        self._lane_offset = lane_offset_ratio
        psi = heading.psi_deg if heading is not None else None
        dt = None if self._last_t is None else max(1e-3, now - self._last_t)

        dx = None
        if use_phase:
            band = self._band(gray)
            dx = self._phase_dx(band)
            self._prev_band = band

        step = None
        if psi is not None:
            if self._last_psi is not None:
                step = wrap90(psi - self._last_psi)
                # Image content moves right when the car turns left.
                if dx is not None and abs(step) > 0.5:
                    k = dx / step
                    self._px_per_deg = k if self._px_per_deg is None else 0.8 * self._px_per_deg + 0.2 * k
            self._last_psi = psi
            self._source = "grid"
            self._last_meas_t = now
        elif dx is not None and self._px_per_deg is not None and abs(self._px_per_deg) > 1e-3:
            step = dx / self._px_per_deg
            if self._last_psi is not None:
                self._last_psi = wrap90(self._last_psi + step)
            self._source = "phase"
            self._last_meas_t = now
        elif dt is not None and self._omega is not None and self._age(now) <= self.config.coast_max_s:
            step = self._omega * dt
            if self._last_psi is not None:
                self._last_psi = wrap90(self._last_psi + step)
            self._source = "coast"
        else:
            self._source = "none"

        if step is not None:
            self._yaw += step
            if dt is not None and self._source != "coast":
                rate = step / dt
                self._omega = rate if self._omega is None else 0.6 * self._omega + 0.4 * rate
        self._last_t = now
        self._last_psi_seen = psi
        return self.sample(now)

    def _age(self, now: float) -> float:
        return float("inf") if self._last_meas_t is None else now - self._last_meas_t

    def sample(self, now: float | None = None) -> YawSample:
        now = time.monotonic() if now is None else now
        if self._reset_pending:
            return YawSample(0.0, None, None, "none", float("inf"), False, self._lane_offset)
        age = self._age(now)
        return YawSample(
            yaw_deg=self._yaw,
            omega_dps=self._omega,
            psi_deg=self._last_psi_seen,
            source=self._source,
            age_s=age,
            valid=age <= self.config.coast_max_s,
            lane_offset_ratio=self._lane_offset,
        )

    def telemetry(self) -> dict[str, Any]:
        """Same keys every frame (the route CSV header is fixed by its first row)."""
        if self.last_heading is not None:
            tel = self.last_heading.telemetry()
        else:
            tel = dict.fromkeys(("grid_psi", "grid_vps", "grid_lines", "grid_g_px", "grid_ms"))
        tel.update({
            "turn_yaw": round(self._yaw, 2),
            "turn_omega": None if self._omega is None else round(self._omega, 1),
            "turn_yaw_src": self._source,
            "turn_px_per_deg": None if self._px_per_deg is None else round(self._px_per_deg, 2),
        })
        return tel
