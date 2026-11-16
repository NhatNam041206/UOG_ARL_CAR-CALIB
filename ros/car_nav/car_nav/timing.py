"""Timing budget for the car nodes (measurement only, no behaviour change).

Each node keeps a TimingStats; callbacks are wrapped with stats.wrap(name, fn)
(processing time, wall clock) and values such as the pose age at use are
added with stats.value(name, v). Every period_s it logs one line per name:

    timing <name>: n 70 p50 3.1 p95 5.2 max 9.0 ms      (durations)
    timing <name>: n 70 p50 0.21 p95 0.40 max 0.9 s     (values)
    timing <name>: count 3                              (counts, e.g. skipped messages)

and starts a new window. sim_results/tools/timing_summary.py combines the
lines of a run. Off with the ROS parameter timing_log:=false.
"""
from __future__ import annotations

import time
from typing import Callable


def _pct(v: list[float], q: float) -> float:
    s = sorted(v)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


class TimingStats:
    def __init__(self, logger, period_s: float = 10.0, enabled: bool = True) -> None:
        self._log = logger
        self._period = period_s
        self.enabled = enabled
        self._dur: dict[str, list[float]] = {}
        self._val: dict[str, list[float]] = {}
        self._cnt: dict[str, int] = {}
        self._t0 = time.monotonic()

    def wrap(self, name: str, fn: Callable) -> Callable:
        if not self.enabled:
            return fn

        def timed(*args, **kwargs):
            t = time.perf_counter()
            try:
                return fn(*args, **kwargs)
            finally:
                self.duration(name, time.perf_counter() - t)
        return timed

    def duration(self, name: str, seconds: float) -> None:
        if self.enabled:
            self._dur.setdefault(name, []).append(seconds)
            self._maybe_log()

    def value(self, name: str, v: float) -> None:
        if self.enabled:
            self._val.setdefault(name, []).append(float(v))
            self._maybe_log()

    def count(self, name: str, n: int = 1) -> None:
        if self.enabled:
            self._cnt[name] = self._cnt.get(name, 0) + n
            self._maybe_log()

    def _maybe_log(self) -> None:
        now = time.monotonic()
        if now - self._t0 < self._period:
            return
        self._t0 = now
        for k, v in self._dur.items():
            if v:
                self._log.info(f"timing {k}: n {len(v)} p50 {_pct(v, .5) * 1e3:.2f} p95 {_pct(v, .95) * 1e3:.2f} "
                               f"max {max(v) * 1e3:.2f} ms")
        for k, v in self._val.items():
            if v:
                self._log.info(f"timing {k}: n {len(v)} p50 {_pct(v, .5):.3f} p95 {_pct(v, .95):.3f} "
                               f"max {max(v):.3f} s")
        for k, n in self._cnt.items():
            self._log.info(f"timing {k}: count {n}")
        self._dur, self._val, self._cnt = {}, {}, {}
