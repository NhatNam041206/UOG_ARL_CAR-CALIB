"""Lane shift detector: did the car move into the next tile row?

The lane tracker reports ``lane_offset_ratio`` (lane centre relative to the
car, as a fraction of the lane width; > 0 means the centre is to the right).
When the car crosses a lane line the tracker locks onto the next pair of
seams and the offset jumps by about one lane width: from about +0.5 to about
-0.5 when the car crosses its left line into the lane on the left, and the
other way round for the right.

A shift is *active* when the car was told to turn (a script turn step); the
map already handles that through the turn. Any other shift is *passive*: the
car drifted, or a manual driver changed lanes. Then the map moves one row
sideways.

A tracker that only flickers between the two lines of one lane (seen on
2026-10-28 with a stale VP row) must not count, so a shift needs the offset
held near one edge before the jump, held on the other side after it, and no
large heading change across it.
"""

from __future__ import annotations

import os
from collections import deque
from dataclasses import dataclass


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


@dataclass
class LaneShiftConfig:
    jump_ratio: float = 0.6  # offset change across the jump (lane widths)
    edge_ratio: float = 0.25  # |offset| held before the jump (car near an edge)
    after_ratio: float = 0.15  # |offset| held after the jump, other sign
    hold_frames: int = 3  # frames held on each side
    max_gap_s: float = 0.6  # longest gap without an offset across the jump
    max_psi_change_deg: float = 12.0  # a turn, not a lane shift
    cooldown_s: float = 1.0
    # The tracker re-locks onto the new lane during and just after a turn
    # (2026-10-28 runs: in the 1 s stop after the turn), so a shift that
    # close to a commanded turn is active too.
    active_hold_s: float = 2.0

    @classmethod
    def from_env(cls) -> "LaneShiftConfig":
        d = cls()
        return cls(
            jump_ratio=_env_float("LANE_SHIFT_JUMP_RATIO", d.jump_ratio),
            edge_ratio=_env_float("LANE_SHIFT_EDGE_RATIO", d.edge_ratio),
            after_ratio=_env_float("LANE_SHIFT_AFTER_RATIO", d.after_ratio),
            hold_frames=int(_env_float("LANE_SHIFT_HOLD_FRAMES", d.hold_frames)),
            max_gap_s=_env_float("LANE_SHIFT_MAX_GAP_S", d.max_gap_s),
            max_psi_change_deg=_env_float("LANE_SHIFT_MAX_PSI_CHANGE_DEG", d.max_psi_change_deg),
            cooldown_s=_env_float("LANE_SHIFT_COOLDOWN_S", d.cooldown_s),
            active_hold_s=_env_float("LANE_SHIFT_ACTIVE_HOLD_S", d.active_hold_s),
        )


@dataclass(frozen=True)
class LaneShift:
    side: int  # +1 into the lane on the left, -1 into the lane on the right
    active: bool  # a commanded turn was running
    t: float
    offset_before: float
    offset_after: float

    def as_dict(self) -> dict[str, object]:
        return {
            "side": "left" if self.side > 0 else "right",
            "kind": "active" if self.active else "passive",
            "t": round(self.t, 3),
            "offset_before": round(self.offset_before, 3),
            "offset_after": round(self.offset_after, 3),
        }


class LaneShiftDetector:
    def __init__(self, config: LaneShiftConfig | None = None) -> None:
        self.config = config or LaneShiftConfig.from_env()
        # (t, offset, psi, active) of recent frames with an offset.
        self._hist: deque[tuple[float, float, float | None, bool]] = deque(maxlen=64)
        self._last_event_t = -1e9
        self._last_active_t = -1e9
        self.last: LaneShift | None = None
        self.passive_count = 0

    def reset(self) -> None:
        self._hist.clear()

    def update(
        self,
        now: float,
        offset: float | None,
        psi_deg: float | None = None,
        active: bool = False,
    ) -> LaneShift | None:
        """Feed one frame; returns a LaneShift on the frame that confirms one.

        active: a commanded turn is running on this frame.
        """
        cfg = self.config
        if active:
            self._last_active_t = now
        if offset is None:
            return None
        if self._hist and now - self._hist[-1][0] > cfg.max_gap_s:
            self._hist.clear()
        self._hist.append((now, float(offset), psi_deg, now - self._last_active_t <= cfg.active_hold_s))
        n = max(1, cfg.hold_frames)
        if len(self._hist) < 2 * n or now - self._last_event_t < cfg.cooldown_s:
            return None
        frames = list(self._hist)[-2 * n :]
        before, after = frames[:n], frames[n:]
        sign = 1.0 if before[-1][1] > 0 else -1.0
        if not all(sign * f[1] >= cfg.edge_ratio for f in before):
            return None
        if not all(-sign * f[1] >= cfg.after_ratio for f in after):
            return None
        if sign * (before[-1][1] - after[0][1]) < cfg.jump_ratio:
            return None
        psis = [f[2] for f in frames if f[2] is not None]
        if len(psis) >= 2 and max(psis) - min(psis) > cfg.max_psi_change_deg:
            return None
        # Offset +edge -> -edge: the lane centre moved from right to left of
        # the car, so the car crossed its left line.
        event = LaneShift(
            side=1 if sign > 0 else -1,
            active=any(f[3] for f in frames),
            t=now,
            offset_before=before[-1][1],
            offset_after=after[0][1],
        )
        self._last_event_t = now
        self._hist.clear()
        self.last = event
        if not event.active:
            self.passive_count += 1
        return event

    def telemetry(self) -> dict[str, object]:
        last = self.last
        return {
            "lane_shift_side": None if last is None else ("left" if last.side > 0 else "right"),
            "lane_shift_kind": None if last is None else ("active" if last.active else "passive"),
            "lane_shift_t": None if last is None else round(last.t, 3),
            "lane_shift_passive_count": self.passive_count,
        }
