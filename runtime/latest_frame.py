"""Camera reader that always hands out the newest frame.

cv2.VideoCapture queues a few frames inside the driver; when the loop is
slower than the camera, read() returns a frame that is already several
periods old. The 2026-10-28 runs showed ~0.65 s from a command to the first
frame that shows it, part of which is this queue. LatestFrameCapture drains
the camera in a thread and keeps only the last frame, so read() is at most
one frame period old. It exposes the VideoCapture methods main_pi5 uses.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import cv2
import numpy as np


class LatestFrameCapture:
    def __init__(self, cap: cv2.VideoCapture, timeout_s: float = 1.0) -> None:
        self._cap = cap
        self._timeout_s = timeout_s
        self._cond = threading.Condition()
        self._frame: np.ndarray | None = None
        self._ok = True
        self._seq = 0
        self._taken = 0
        self._stamp = 0.0
        self._stop = False
        self._thread = threading.Thread(target=self._grab, daemon=True)
        self._thread.start()

    def _grab(self) -> None:
        while not self._stop:
            ok, frame = self._cap.read()
            with self._cond:
                self._ok = bool(ok) and frame is not None
                if self._ok:
                    self._frame = frame
                    self._seq += 1
                    self._stamp = time.monotonic()
                self._cond.notify_all()
            if not ok:
                break

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Newest frame not handed out yet; waits for the next one if needed."""
        deadline = time.monotonic() + self._timeout_s
        with self._cond:
            while self._ok and self._seq == self._taken:
                left = deadline - time.monotonic()
                if left <= 0:
                    return False, None
                self._cond.wait(left)
            if not self._ok or self._frame is None:
                return False, None
            self._taken = self._seq
            return True, self._frame

    @property
    def frame_age_s(self) -> float:
        with self._cond:
            return time.monotonic() - self._stamp if self._seq else float("inf")

    def isOpened(self) -> bool:  # noqa: N802 - VideoCapture API
        return self._cap.isOpened()

    def set(self, prop: int, value: Any) -> bool:
        return self._cap.set(prop, value)

    def get(self, prop: int) -> float:
        return self._cap.get(prop)

    def release(self) -> None:
        self._stop = True
        self._thread.join(timeout=self._timeout_s)
        self._cap.release()
