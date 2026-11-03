"""Count transverse (cross-corridor) tile seams passing under the vehicle.

Pipeline per frame, all positions as ratios of the frame size:

1. Dark-line filter: a black-hat with a vertical kernel keeps thin dark
   features that run roughly horizontally (grout between tiles) and drops
   the steep longitudinal seams, glare spots and broad shadows.
2. Look-ahead ROI [roi_top, roi_bottom]: standard Hough limited to
   max_tilt_deg around horizontal finds lines described by (y at the
   frame's centre column, tilt angle).
3. Tracking: each line is matched to a track by predicted y and tilt. A
   track is confirmed after confirm_frames detections, which builds the
   "detected lines list".
4. Counter ROI: a band of height counter_h centred on counter_y. A confirmed
   track whose y crosses counter_y going down counts one tile. A track that
   drops out (motion blur smears seams low in the frame) is carried by its
   velocity for up to max_miss_frames, so it still counts.

Reversing (set_direction(-1)): seams move up the image. New tracks start
just below the counter band (down to reverse_roi_extra under it) and a
track counts when it crosses counter_y going up.

tile_count is the number of seams crossed since reset() in either
direction (what a tile step waits for); net_count adds forward crossings
and subtracts reverse ones (the car's displacement, used for the map).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any

import cv2
import numpy as np

from config.settings import _get_bool, _get_float, _get_int


@dataclass
class CrossLineConfig:
    """Tunable parameters; ratios are of frame height unless noted."""

    enabled: bool = True
    roi_top: float = 0.12
    roi_bottom: float = 0.38
    counter_y: float = 0.45
    counter_h: float = 0.10
    blackhat_kernel_px: int = 15
    dark_thresh: int = 6
    min_len: float = 0.35  # Hough votes needed, as a ratio of frame width
    max_tilt_deg: float = 35.0
    merge_dy: float = 0.05
    merge_dtilt_deg: float = 6.0
    slope_tol_deg: float = 8.0
    track_max_dy: float = 0.12
    confirm_frames: int = 2
    max_miss_frames: int = 8
    count_min_gap_frames: int = 4
    # A count also needs this fraction of the median recent interval between
    # counts (0 = off); rejects shadow edges and ghost lines next to a seam.
    count_min_gap_ratio: float = 0.5
    # Reversing: how far below the counter band seams are detected (new
    # tracks start there and move up).
    reverse_roi_extra: float = 0.2

    @classmethod
    def from_env(cls) -> "CrossLineConfig":
        d = cls()
        return cls(
            enabled=_get_bool("CROSS_ENABLED", d.enabled),
            roi_top=_get_float("CROSS_ROI_TOP", d.roi_top),
            roi_bottom=_get_float("CROSS_ROI_BOTTOM", d.roi_bottom),
            counter_y=_get_float("CROSS_COUNTER_Y", d.counter_y),
            counter_h=_get_float("CROSS_COUNTER_H", d.counter_h),
            blackhat_kernel_px=_get_int("CROSS_BLACKHAT_KERNEL_PX", d.blackhat_kernel_px),
            dark_thresh=_get_int("CROSS_DARK_THRESH", d.dark_thresh),
            min_len=_get_float("CROSS_MIN_LEN", d.min_len),
            max_tilt_deg=_get_float("CROSS_MAX_TILT_DEG", d.max_tilt_deg),
            merge_dy=_get_float("CROSS_MERGE_DY", d.merge_dy),
            merge_dtilt_deg=_get_float("CROSS_MERGE_DTILT_DEG", d.merge_dtilt_deg),
            slope_tol_deg=_get_float("CROSS_SLOPE_TOL_DEG", d.slope_tol_deg),
            track_max_dy=_get_float("CROSS_TRACK_MAX_DY", d.track_max_dy),
            confirm_frames=_get_int("CROSS_CONFIRM_FRAMES", d.confirm_frames),
            max_miss_frames=_get_int("CROSS_MAX_MISS_FRAMES", d.max_miss_frames),
            count_min_gap_frames=_get_int("CROSS_COUNT_MIN_GAP_FRAMES", d.count_min_gap_frames),
            count_min_gap_ratio=_get_float("CROSS_COUNT_MIN_GAP_RATIO", d.count_min_gap_ratio),
        )

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class CrossLine:
    """One transverse line: y at the centre column (ratio) and tilt in degrees."""

    y: float
    tilt_deg: float
    coverage: float  # merged horizontal extent / frame width

    def y_at(self, x_ratio: float, aspect: float) -> float:
        """y ratio at column x_ratio; aspect = width / height."""
        return self.y + math.tan(math.radians(self.tilt_deg)) * (x_ratio - 0.5) * aspect


@dataclass
class _Track:
    track_id: int
    y: float
    tilt_deg: float
    vy: float = 0.0
    hits: int = 1
    misses: int = 0
    counted: bool = False

    @property
    def predicted_y(self) -> float:
        return self.y + self.vy


@dataclass
class CrossLineResult:
    tile_count: int
    counted_this_frame: bool
    lines: list[CrossLine]
    tracks: list[dict[str, Any]]
    net_count: int = 0
    direction: int = 1

    def telemetry(self) -> dict[str, Any]:
        return {
            "cross_tile_count": self.tile_count,
            "cross_net_count": self.net_count,
            "cross_direction": self.direction,
            "cross_counted": self.counted_this_frame,
            "cross_lines": len(self.lines),
            "cross_tracks": sum(1 for t in self.tracks if t["confirmed"]),
        }


class CrossLineCounter:
    """Detect transverse seams and count how many pass the counter band."""

    def __init__(self, config: CrossLineConfig | None = None) -> None:
        self.config = config or CrossLineConfig.from_env()
        self.reset()

    def reset(self) -> None:
        self._tracks: list[_Track] = []
        self._next_id = 1
        self._tile_count = 0
        self._net_count = 0
        self._direction = 1
        self._frames_since_count = 10**6
        self._intervals: list[int] = []
        self._last_dark: np.ndarray | None = None
        self._last_lines: list[CrossLine] = []

    @property
    def tile_count(self) -> int:
        return self._tile_count

    @property
    def net_count(self) -> int:
        return self._net_count

    @property
    def direction(self) -> int:
        return self._direction

    def set_direction(self, direction: int) -> None:
        """+1 driving forward, -1 reversing. A change drops the tracks (they
        move the other way now) but keeps the counts."""
        direction = -1 if direction < 0 else 1
        if direction == self._direction:
            return
        self._direction = direction
        self._tracks = []
        self._intervals = []
        self._frames_since_count = 10**6

    def _detect_bottom(self) -> float:
        cfg = self.config
        bottom = cfg.counter_y + cfg.counter_h / 2.0
        if self._direction < 0:
            bottom += max(0.0, cfg.reverse_roi_extra)
        return max(0.0, min(1.0, bottom))

    # ------------------------------------------------------------------ #
    # Detection
    # ------------------------------------------------------------------ #
    def detect(self, frame: np.ndarray) -> list[CrossLine]:
        """Return transverse lines between roi_top and the bottom of the counter band."""
        cfg = self.config
        h, w = frame.shape[:2]
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        y0 = int(h * max(0.0, min(1.0, cfg.roi_top)))
        y1 = int(h * self._detect_bottom())
        if y1 - y0 < 4:
            self._last_dark = None
            return []
        band = gray[y0:y1]
        k = max(3, int(cfg.blackhat_kernel_px) | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, k))
        dark = cv2.morphologyEx(cv2.GaussianBlur(band, (5, 3), 0), cv2.MORPH_BLACKHAT, kernel)
        _, mask = cv2.threshold(dark, cfg.dark_thresh, 255, cv2.THRESH_BINARY)
        self._last_dark = mask

        # Standard Hough integrates votes along the whole line, so a faint seam
        # that spans the frame still wins over short dark specks.
        votes_min = max(10, int(w * cfg.min_len))
        max_tilt = math.radians(max(1.0, min(80.0, cfg.max_tilt_deg)))
        found = cv2.HoughLinesWithAccumulator(
            mask,
            1,
            np.pi / 180,
            votes_min,
            min_theta=np.pi / 2 - max_tilt,
            max_theta=np.pi / 2 + max_tilt,
        )
        if found is None:
            return []
        lines: list[CrossLine] = []
        # Results are sorted by votes; keep the strongest line per seam.
        for rho, theta, votes in found.reshape(-1, 3).tolist():
            sin_t = math.sin(theta)
            if abs(sin_t) < 1e-6:
                continue
            y_centre = ((rho - (w / 2.0) * math.cos(theta)) / sin_t + y0) / h
            tilt = math.degrees(theta) - 90.0
            if any(
                abs(l.y - y_centre) <= cfg.merge_dy and abs(l.tilt_deg - tilt) <= cfg.merge_dtilt_deg
                for l in lines
            ):
                continue
            lines.append(CrossLine(y=float(y_centre), tilt_deg=float(tilt), coverage=float(votes) / w))
        lines.sort(key=lambda l: l.y)
        # The tile grid is one family of parallel seams: lines whose tilt is
        # far from the frame's dominant tilt are furniture or skirting edges.
        if len(lines) >= 2:
            ref = float(np.average([l.tilt_deg for l in lines], weights=[l.coverage for l in lines]))
            lines = [l for l in lines if abs(l.tilt_deg - ref) <= cfg.slope_tol_deg * 2.0]
        return lines

    # ------------------------------------------------------------------ #
    # Tracking + counting
    # ------------------------------------------------------------------ #
    def update(self, frame: np.ndarray) -> CrossLineResult:
        """Process one frame and return the running tile count."""
        cfg = self.config
        if not cfg.enabled or frame is None or frame.size == 0:
            return CrossLineResult(self._tile_count, False, [], self._track_dicts(), self._net_count, self._direction)
        lines = self.detect(frame)
        self._last_lines = lines
        self._frames_since_count += 1
        counted = False

        unmatched = list(range(len(lines)))
        # Match tracks top-down so that two seams in view keep their order.
        for track in sorted(self._tracks, key=lambda t: t.y):
            best = None
            best_cost = None
            for idx in unmatched:
                line = lines[idx]
                dy = line.y - track.predicted_y
                if abs(dy) > cfg.track_max_dy or abs(line.tilt_deg - track.tilt_deg) > cfg.slope_tol_deg:
                    continue
                # Seams only move down the image while the car drives forward
                # (up while reversing); allow a small jitter the other way.
                if self._direction > 0 and line.y < track.y - cfg.merge_dy:
                    continue
                if self._direction < 0 and line.y > track.y + cfg.merge_dy:
                    continue
                cost = abs(dy)
                if best_cost is None or cost < best_cost:
                    best, best_cost = idx, cost
            prev_y = track.y
            if best is not None:
                line = lines[best]
                unmatched.remove(best)
                new_vy = line.y - track.y
                track.vy = new_vy if track.hits == 1 else 0.5 * track.vy + 0.5 * new_vy
                track.y = line.y
                track.tilt_deg = 0.7 * track.tilt_deg + 0.3 * line.tilt_deg
                track.hits += 1
                track.misses = 0
            else:
                track.misses += 1
                track.y = track.predicted_y
            if self._should_count(track, prev_y):
                track.counted = True
                if self._frames_since_count >= self._min_gap():
                    self._remember_interval(self._frames_since_count)
                    self._tile_count += 1
                    self._net_count += self._direction
                    self._frames_since_count = 0
                    counted = True

        kept: list[_Track] = []
        # Two tracks on one seam (blur edges, a ghost line) collapse into the
        # older one, so the seam cannot count twice.
        for t in sorted(self._tracks, key=lambda t: -t.hits):
            if t.misses > cfg.max_miss_frames or t.y > 1.0 + cfg.track_max_dy:
                continue
            if self._direction < 0 and t.y < cfg.roi_top - cfg.track_max_dy:
                continue
            twin = next((k for k in kept if abs(k.y - t.y) <= cfg.merge_dy), None)
            if twin is not None:
                twin.counted = twin.counted or t.counted
                continue
            kept.append(t)
        self._tracks = kept
        if self._direction > 0:
            new_top = cfg.roi_top
            new_bottom = min(cfg.roi_bottom, cfg.counter_y - cfg.counter_h / 2.0)
        else:
            # Reversing: seams come up from below the counter band.
            new_top = cfg.counter_y + cfg.counter_h / 2.0
            new_bottom = self._detect_bottom()
        for idx in unmatched:
            line = lines[idx]
            # New seams may only appear in the look-ahead ROI; a line first
            # seen inside the counter band has no history to confirm it.
            if new_top <= line.y <= new_bottom:
                self._tracks.append(_Track(self._next_id, line.y, line.tilt_deg))
                self._next_id += 1
        return CrossLineResult(
            self._tile_count, counted, lines, self._track_dicts(), self._net_count, self._direction
        )

    def _remember_interval(self, frames: int) -> None:
        if frames >= 10**5:
            return
        # Long intervals are stops, not the driving rhythm; keep them out.
        if len(self._intervals) >= 3 and frames > 2.5 * float(np.median(self._intervals)):
            return
        self._intervals = (self._intervals + [frames])[-5:]

    def _min_gap(self) -> float:
        cfg = self.config
        gap = float(cfg.count_min_gap_frames)
        if cfg.count_min_gap_ratio > 0 and len(self._intervals) >= 3:
            gap = max(gap, cfg.count_min_gap_ratio * float(np.median(self._intervals)))
        return gap

    def _should_count(self, track: _Track, prev_y: float) -> bool:
        cfg = self.config
        if track.counted or track.hits < cfg.confirm_frames:
            return False
        if track.misses > cfg.max_miss_frames:
            return False
        # A coasting track may only count while it is inside the band.
        half = cfg.counter_h / 2.0 + abs(track.vy)
        if self._direction < 0:
            if track.misses > 0 and track.y < cfg.counter_y - half:
                return False
            return prev_y > cfg.counter_y >= track.y
        if track.misses > 0 and track.y > cfg.counter_y + half:
            return False
        return prev_y < cfg.counter_y <= track.y

    def _track_dicts(self) -> list[dict[str, Any]]:
        return [
            {
                "id": t.track_id,
                "y": round(t.y, 4),
                "tilt_deg": round(t.tilt_deg, 2),
                "vy": round(t.vy, 4),
                "confirmed": t.hits >= self.config.confirm_frames,
                "coasting": t.misses > 0,
                "counted": t.counted,
            }
            for t in self._tracks
        ]

    # ------------------------------------------------------------------ #
    # Overlay
    # ------------------------------------------------------------------ #
    def draw(self, frame: np.ndarray, result: CrossLineResult) -> np.ndarray:
        """Draw ROIs, detected lines, tracks and the count onto frame (in place)."""
        cfg = self.config
        h, w = frame.shape[:2]
        aspect = w / float(h)
        top = int(cfg.roi_top * h)
        bottom = int(cfg.roi_bottom * h)
        cv2.rectangle(frame, (0, top), (w - 1, bottom), (255, 200, 0), 1)
        band_top = int((cfg.counter_y - cfg.counter_h / 2.0) * h)
        band_bottom = int((cfg.counter_y + cfg.counter_h / 2.0) * h)
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, band_top), (w - 1, band_bottom), (0, 165, 255), -1)
        cv2.addWeighted(overlay, 0.18, frame, 0.82, 0, frame)
        cv2.line(frame, (0, int(cfg.counter_y * h)), (w - 1, int(cfg.counter_y * h)), (0, 165, 255), 1)
        for line in result.lines:
            p0 = (0, int(line.y_at(0.0, aspect) * h))
            p1 = (w - 1, int(line.y_at(1.0, aspect) * h))
            cv2.line(frame, p0, p1, (255, 0, 255), 1)
        for t in result.tracks:
            if not t["confirmed"]:
                continue
            ty = int(t["y"] * h)
            color = (0, 255, 0) if not t["counted"] else (0, 0, 255)
            cv2.circle(frame, (w // 2, ty), 5, color, -1)
            cv2.putText(frame, f"#{t['id']}", (w // 2 + 8, ty - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        label = f"TILES {result.tile_count}" + ("  +1" if result.counted_this_frame else "")
        cv2.putText(frame, label, (w - 170, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4)
        cv2.putText(frame, label, (w - 170, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 165, 255), 2)
        return frame

