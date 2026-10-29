"""LatestFrameCapture hands out only the newest frame."""

from __future__ import annotations

import threading
import time

import numpy as np

from runtime.latest_frame import LatestFrameCapture


class _FakeCap:
    def __init__(self, n: int, period: float) -> None:
        self.n, self.period, self.i = n, period, 0
        self.released = threading.Event()

    def read(self):
        time.sleep(self.period)
        if self.i >= self.n:
            return False, None
        self.i += 1
        return True, np.full((2, 2), self.i, np.uint8)

    def isOpened(self):
        return True

    def release(self):
        self.released.set()


def test_slow_consumer_skips_stale_frames():
    cap = LatestFrameCapture(_FakeCap(n=50, period=0.01))
    ok, first = cap.read()
    assert ok
    time.sleep(0.1)  # the loop is busy; ~10 frames arrive meanwhile
    ok, frame = cap.read()
    assert ok and int(frame[0, 0]) - int(first[0, 0]) >= 5
    cap.release()


def test_end_of_stream_reports_failure():
    fake = _FakeCap(n=2, period=0.005)
    cap = LatestFrameCapture(fake, timeout_s=0.3)
    assert cap.read()[0]
    deadline = time.monotonic() + 1.0
    while cap.read()[0] and time.monotonic() < deadline:
        pass
    assert cap.read()[0] is False
    cap.release()
    assert fake.released.is_set()
