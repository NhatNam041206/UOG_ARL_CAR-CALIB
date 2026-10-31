"""Bounded resource helpers for dashboard, route scripts, and stored artifacts."""

from __future__ import annotations

import math
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from config.settings import (
    DATA_MAX_BYTES,
    DATA_RETENTION_DAYS,
    DISK_MIN_FREE_BYTES,
    ROUTE_MAX_STEP_DEGREES,
    ROUTE_MAX_STEP_DURATION_S,
    ROUTE_MAX_STEP_TILES,
    ROUTE_MAX_STEPS,
    ROUTE_MAX_TOTAL_DURATION_S,
    TILE_STEP_MAX_S_PER_TILE,
    TURN_ANCHOR_TIMEOUT_S,
    TURN_LATENCY_S,
    TURN_NOMINAL_OMEGA_DPS,
    TURN_TIMEOUT_FACTOR,
)

# Actions that may run by tile count: the car drives forward under vision
# steering, so the cross-line counter sees the seams pass.
TILE_ROUTE_ACTIONS = frozenset({"forward", "straight"})

# Actions that may run by degrees: the grid heading watches the yaw and the
# runner stops the turn when it is reached.
DEGREE_ROUTE_ACTIONS = frozenset({"left", "right", "turn_left", "turn_right"})

VALID_ROUTE_ACTIONS = frozenset({
    "forward", "backward", "straight", "left", "right", "turn_left", "turn_right", "stop", "pause", "settle",
    "lane_left", "lane_right",
})


def turn_timeout_s(
    degrees: float,
    latency_s: float = TURN_LATENCY_S,
    nominal_omega_dps: float = TURN_NOMINAL_OMEGA_DPS,
    timeout_factor: float = TURN_TIMEOUT_FACTOR,
) -> float:
    """Safety timeout of a degrees step; also what it counts toward the route total."""
    return timeout_factor * float(degrees) / max(1.0, nominal_omega_dps) + latency_s

# Dashboard configuration is persistent control data, not disposable telemetry.
# Keep this explicit even when a deployment accidentally mounts log_root and
# route_root to the same host directory.
PROTECTED_STORAGE_NAMES = frozenset({"presets.json", "calib_tune.json"})
LOG_ARTIFACT_SUFFIXES = frozenset({".csv", ".log"})


class ScriptValidationError(ValueError):
    """Raised when dashboard-provided route steps exceed safe limits."""


def validate_route_steps(raw_steps: Any) -> list[dict[str, Any]]:
    """Validate and normalize dashboard route steps before they reach the runner."""
    if not isinstance(raw_steps, list):
        raise ScriptValidationError("steps must be a list")
    if not raw_steps:
        raise ScriptValidationError("at least one step is required")
    if len(raw_steps) > ROUTE_MAX_STEPS:
        raise ScriptValidationError(f"at most {ROUTE_MAX_STEPS} steps are allowed")

    total_duration = 0.0
    normalized: list[dict[str, Any]] = []
    for index, raw_step in enumerate(raw_steps, start=1):
        if not isinstance(raw_step, dict):
            raise ScriptValidationError(f"step {index} must be an object")
        action = str(raw_step.get("action", "")).strip().lower()
        if action not in VALID_ROUTE_ACTIONS:
            raise ScriptValidationError(f"step {index} has unsupported action: {action!r}")
        if raw_step.get("tiles") not in (None, ""):
            tiles = raw_step.get("tiles")
            if isinstance(tiles, bool) or not isinstance(tiles, (int, float)) or int(tiles) != tiles:
                raise ScriptValidationError(f"step {index} tiles must be a whole number")
            tiles = int(tiles)
            if action not in TILE_ROUTE_ACTIONS:
                raise ScriptValidationError(f"step {index}: only forward/straight can run by tiles")
            if not 1 <= tiles <= ROUTE_MAX_STEP_TILES:
                raise ScriptValidationError(f"step {index} tiles must be between 1 and {ROUTE_MAX_STEP_TILES}")
            # The safety timeout is what the step can cost in time.
            total_duration += tiles * TILE_STEP_MAX_S_PER_TILE
            if total_duration > ROUTE_MAX_TOTAL_DURATION_S:
                raise ScriptValidationError(
                    f"total route duration must not exceed {ROUTE_MAX_TOTAL_DURATION_S:g} seconds "
                    f"(tile steps count {TILE_STEP_MAX_S_PER_TILE:g} s per tile)"
                )
            normalized.append({"action": action, "tiles": tiles})
            continue
        if raw_step.get("degrees") not in (None, ""):
            degrees = raw_step.get("degrees")
            if isinstance(degrees, bool) or not isinstance(degrees, (int, float)) or not math.isfinite(degrees):
                raise ScriptValidationError(f"step {index} degrees must be a number")
            if action not in DEGREE_ROUTE_ACTIONS:
                raise ScriptValidationError(f"step {index}: only left/right/turn_left/turn_right can run by degrees")
            if not 5.0 <= float(degrees) <= ROUTE_MAX_STEP_DEGREES:
                raise ScriptValidationError(
                    f"step {index} degrees must be between 5 and {ROUTE_MAX_STEP_DEGREES:g}"
                )
            anchor = raw_step.get("anchor") or None
            if anchor is not None and anchor != "next_line":
                raise ScriptValidationError(f"step {index} anchor must be \"next_line\"")
            total_duration += turn_timeout_s(float(degrees)) + (TURN_ANCHOR_TIMEOUT_S if anchor else 0.0)
            if total_duration > ROUTE_MAX_TOTAL_DURATION_S:
                raise ScriptValidationError(
                    f"total route duration must not exceed {ROUTE_MAX_TOTAL_DURATION_S:g} seconds "
                    "(degree steps count their safety timeout)"
                )
            normalized.append({"action": action, "degrees": float(degrees), **({"anchor": anchor} if anchor else {})})
            continue
        try:
            duration = float(raw_step.get("duration_s", 0.0))
        except (TypeError, ValueError) as exc:
            raise ScriptValidationError(f"step {index} duration_s must be numeric") from exc
        if not math.isfinite(duration):
            raise ScriptValidationError(f"step {index} duration_s must be finite")
        if not 0.05 <= duration <= ROUTE_MAX_STEP_DURATION_S:
            raise ScriptValidationError(
                f"step {index} duration_s must be between 0.05 and {ROUTE_MAX_STEP_DURATION_S:g} seconds"
            )
        total_duration += duration
        if total_duration > ROUTE_MAX_TOTAL_DURATION_S:
            raise ScriptValidationError(
                f"total route duration must not exceed {ROUTE_MAX_TOTAL_DURATION_S:g} seconds"
            )
        normalized.append({"action": action, "duration_s": duration})
    return normalized


@dataclass(frozen=True)
class StorageHealth:
    writable: bool
    reason: str
    free_bytes: int | None
    managed_bytes: int


class StorageManager:
    """Retains bounded data and turns storage failures into a non-fatal state."""

    def __init__(self, log_root: str | Path, route_root: str | Path) -> None:
        self.log_root = Path(log_root)
        self.route_root = Path(route_root)
        self._writable = True
        self._reason = ""
        self._active_paths: set[Path] = set()
        self.log_root.mkdir(parents=True, exist_ok=True)
        self.route_root.mkdir(parents=True, exist_ok=True)

    def set_active_paths(self, paths: Iterable[str | Path]) -> None:
        self._active_paths = {Path(path).resolve() for path in paths}

    def mark_error(self, error: BaseException | str) -> None:
        self._writable = False
        self._reason = str(error) or "storage error"

    def recover_if_possible(self) -> bool:
        self.retain()
        health = self.health()
        if (
            health.free_bytes is not None
            and health.free_bytes >= DISK_MIN_FREE_BYTES
            and health.managed_bytes <= DATA_MAX_BYTES
        ):
            self._writable = True
            self._reason = ""
        return self._writable

    def retain(self) -> None:
        now = time.time()
        max_age_s = max(0, DATA_RETENTION_DAYS) * 86400
        candidates = self._candidates()
        for path, _size, modified in candidates:
            if max_age_s and now - modified > max_age_s:
                self._remove(path)

        candidates = self._candidates()
        total = sum(size for _path, size, _modified in candidates)
        for path, size, _modified in candidates:
            if total <= DATA_MAX_BYTES:
                break
            if self._remove(path):
                total -= size

    def health(self) -> StorageHealth:
        try:
            free_bytes = shutil.disk_usage(self.log_root).free
        except OSError:
            free_bytes = None
        managed_bytes = sum(size for _path, size, _modified in self._candidates())
        within_quota = managed_bytes <= DATA_MAX_BYTES
        writable = self._writable and within_quota and (free_bytes is None or free_bytes >= DISK_MIN_FREE_BYTES)
        reason = self._reason
        if not writable and not reason and free_bytes is not None:
            reason = "managed data quota exceeded" if not within_quota else "low disk space"
        return StorageHealth(writable=writable, reason=reason, free_bytes=free_bytes, managed_bytes=managed_bytes)

    def _candidates(self) -> list[tuple[Path, int, float]]:
        entries: list[tuple[Path, int, float]] = []
        for root, route_only in ((self.log_root, False), (self.route_root, True)):
            if not root.exists():
                continue
            for child in root.iterdir():
                if child.name in PROTECTED_STORAGE_NAMES:
                    continue
                if route_only and not self._is_route_artifact(child):
                    continue
                if not route_only and not self._is_log_artifact(child):
                    continue
                if self._is_active(child):
                    continue
                try:
                    entries.append((child, _path_size(child), child.stat().st_mtime))
                except OSError:
                    continue
        return sorted(entries, key=lambda item: item[2])

    @staticmethod
    def _is_route_artifact(path: Path) -> bool:
        """Only completed route directories and their matching archives expire."""
        return path.name.startswith("route-") and (
            path.is_dir() or (path.is_file() and path.suffix.lower() == ".zip")
        )

    @staticmethod
    def _is_log_artifact(path: Path) -> bool:
        """Never treat arbitrary files beneath a log mount as disposable logs."""
        return path.is_file() and path.suffix.lower() in LOG_ARTIFACT_SUFFIXES

    def _is_active(self, candidate: Path) -> bool:
        try:
            resolved = candidate.resolve()
        except OSError:
            return True
        return any(resolved == active or active.is_relative_to(resolved) or resolved.is_relative_to(active) for active in self._active_paths)

    @staticmethod
    def _remove(path: Path) -> bool:
        try:
            if path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()
            return True
        except OSError:
            return False


def _path_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(entry.stat().st_size for entry in path.rglob("*") if entry.is_file())
